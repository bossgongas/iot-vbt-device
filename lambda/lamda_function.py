import json
import boto3
from datetime import datetime
from decimal import Decimal
import numpy as np

print('Loading function')

# Initialize the Timestream clients
timestream_write_client = boto3.client('timestream-write')
timestream_query_client = boto3.client('timestream-query')

# Define global mass variable (mass of the barbell in kg)
mass = 20  # Example mass value

# Custom JSON encoder for Decimal type
class DecimalEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, Decimal):
            return float(obj)
        return super(DecimalEncoder, self).default(obj)

def query_exercise_data(database_name, table_name, exercise_id):
    query_string = f"""
    SELECT time, AccX, AccY, AccZ, GyroX, GyroY, GyroZ
    FROM "{database_name}"."{table_name}"
    WHERE ExerciseID = '{exercise_id}' AND time > ago(15m)
    ORDER BY time ASC
    """
    response = timestream_query_client.query(QueryString=query_string)
    return response['Rows']

def parse_time(time_str):
    # Truncate the nanoseconds to microseconds and parse
    return datetime.strptime(time_str[:26], '%Y-%m-%d %H:%M:%S.%f')

def median_filter(data, kernel_size=3):
    pad_width = kernel_size // 2
    padded_data = np.pad(data, pad_width, mode='edge')
    filtered_data = np.array([np.median(padded_data[i:i+kernel_size]) for i in range(len(data))])
    return filtered_data

def low_pass_filter(data, alpha=0.1):
    filtered_data = np.zeros_like(data)
    filtered_data[0] = data[0]
    for i in range(1, len(data)):
        filtered_data[i] = alpha * data[i] + (1 - alpha) * filtered_data[i - 1]
    return filtered_data

def combined_filter(data, median_kernel_size=3, alpha=0.1):
    # Apply median filter using numpy
    median_filtered_data = median_filter(data, kernel_size=median_kernel_size)
    # Apply low pass filter using numpy
    low_pass_filtered_data = low_pass_filter(median_filtered_data, alpha=alpha)
    return low_pass_filtered_data

def detect_reps(positions, threshold=1):
    initial_position = positions[0]
    rep_count = 0
    cumulative_movement = 0

    for position in positions:
        cumulative_movement += np.linalg.norm(position - initial_position)
        if cumulative_movement > threshold:
            rep_count += 1
            cumulative_movement = 0  # Reset for the next rep

    return round((rep_count / 3))

def calculate_vbt_statistics(rows):
    if not rows:
        return None

    # Extract accelerometer and gyroscope data
    times = [parse_time(row['Data'][0]['ScalarValue']) for row in rows]
    accelX = np.array([float(row['Data'][1]['ScalarValue']) for row in rows])
    accelY = np.array([float(row['Data'][2]['ScalarValue']) for row in rows])
    accelZ = np.array([float(row['Data'][3]['ScalarValue']) for row in rows])
    gyroX = np.array([float(row['Data'][4]['ScalarValue']) for row in rows])
    gyroY = np.array([float(row['Data'][5]['ScalarValue']) for row in rows])
    gyroZ = np.array([float(row['Data'][6]['ScalarValue']) for row in rows])
    
    accelX -= accelX[0]
    accelY -= accelY[0]
    accelZ -= accelZ[0]

    # Apply combined filter
    filtered_accelX = combined_filter(accelX)
    filtered_accelY = combined_filter(accelY)
    filtered_accelZ = combined_filter(accelZ)

    if len(filtered_accelX) == 0 or len(filtered_accelY) == 0 or len(filtered_accelZ) == 0:
        raise ValueError("Filtered acceleration data is empty")

    # Calculate time differences (in seconds)
    time_deltas = np.diff([t.timestamp() for t in times])  # convert to seconds

    if len(time_deltas) == 0:
        raise ValueError("Time deltas data is empty")

    # Calculate velocities by integrating acceleration
    velocityX = np.cumsum(filtered_accelX[:-1] * time_deltas)
    velocityY = np.cumsum(filtered_accelY[:-1] * time_deltas)
    velocityZ = np.cumsum(filtered_accelZ[:-1] * time_deltas)

    if len(velocityX) == 0 or len(velocityY) == 0 or len(velocityZ) == 0:
        raise ValueError("Velocity data is empty")

    # Calculate mean velocity
    mean_velocity = np.mean(np.sqrt(velocityX**2 + velocityY**2 + velocityZ**2))

    # Calculate peak power (assuming power = force * velocity, and force can be derived from mass * acceleration)
    global mass
    forces = mass * filtered_accelZ[:-1]  # Simplistic force calculation in the Z direction
    powers = forces * velocityZ
    power = powers[-1]

    # Calculate positions by integrating velocities
    positions = np.vstack((np.cumsum(velocityX * time_deltas),
                           np.cumsum(velocityY * time_deltas),
                           np.cumsum(velocityZ * time_deltas))).T

    # Detect number of reps
    num_reps = detect_reps(positions)

    # Calculate range of motion (peak-to-peak range of positions)
    range_of_motion = np.ptp(positions, axis=0)

    # Calculate bar path (simplified as positions)
    bar_path = positions
    
    # Calculate the Range of Motion in degrees: 
    gyro_data = np.vstack([gyroX, gyroY, gyroZ]).T
    angular_displacement = np.cumsum(gyro_data[:-1] * time_deltas[:, np.newaxis], axis=0)

    # Convert angular displacement from radians to degrees
    rom_degrees = np.degrees(angular_displacement[-1])
    
    # Calculate resultant angular velocity from gyroscope data
    angular_velocity = np.sqrt(gyroX**2 + gyroY**2 + gyroZ**2)
    
    # Calculate mean acceleration
    accel_magnitudes = np.sqrt(filtered_accelX**2 + filtered_accelY**2 + filtered_accelZ**2)
    mean_acceleration = np.mean(accel_magnitudes)


    return {
        'mean_velocity': mean_velocity,
        'power': power,
        'num_reps': num_reps,
        'range_of_motion_x': range_of_motion[0],
        'range_of_motion_y': range_of_motion[1],
        'range_of_motion_z': range_of_motion[2],
        'bar_path_x': bar_path[-1, 0].tolist(),
        'bar_path_y': bar_path[-1, 1].tolist(),
        'bar_path_z': bar_path[-1, 2].tolist(),
        'instant_velocity_x': velocityX[-1].tolist(),
        'instant_velocity_y': velocityY[-1].tolist(),
        'instant_velocity_z': velocityZ[-1].tolist(),
        'rom_degrees': rom_degrees[-1].tolist(), 
        'angular_velocity': angular_velocity[-1].tolist(),
        'mean_acceleration': mean_acceleration
    }

def store_statistics(database_name, table_name, exercise_id, stats):
    timestamp_ms = str(int(datetime.utcnow().timestamp() * 1000))
    record = {
        'Dimensions': [
            {'Name': 'ExerciseID', 'Value': exercise_id}
        ],
        'MeasureName': 'exercise_stats',
        'MeasureValueType': 'MULTI',
        'Time': timestamp_ms,
        'MeasureValues': [
            {'Name': 'mean_velocity', 'Value': str(stats['mean_velocity']), 'Type': 'DOUBLE'},
            {'Name': 'inst_velocity', 'Value': str(stats['instant_velocity_z']), 'Type': 'DOUBLE'},
            {'Name': 'power', 'Value': str(stats['power']), 'Type': 'DOUBLE'},
            {'Name': 'num_reps', 'Value': str(stats['num_reps']), 'Type': 'BIGINT'},
            #{'Name': 'range_of_motion_x', 'Value': str(stats['range_of_motion_x']), 'Type': 'DOUBLE'},
            #{'Name': 'range_of_motion_y', 'Value': str(stats['range_of_motion_y']), 'Type': 'DOUBLE'},
            {'Name': 'range_of_motion_z', 'Value': str(stats['range_of_motion_z']), 'Type': 'DOUBLE'},
            #{'Name': 'bar_path_x', 'Value': json.dumps(stats['bar_path_x']), 'Type': 'VARCHAR'},
            #{'Name': 'bar_path_y', 'Value': json.dumps(stats['bar_path_y']), 'Type': 'VARCHAR'},
            {'Name': 'bar_path_z', 'Value': json.dumps(stats['bar_path_z']), 'Type': 'VARCHAR'}, 
            {'Name': 'rom_degrees', 'Value': json.dumps(stats['rom_degrees']), 'Type': 'VARCHAR'},
            {'Name': 'angular_velocity', 'Value': json.dumps(stats['angular_velocity']), 'Type': 'VARCHAR'},
            {'Name': 'mean_acceleration', 'Value': str(stats['mean_acceleration']), 'Type': 'DOUBLE'}
        ]
    }

    timestream_write_client.write_records(
        DatabaseName=database_name,
        TableName=table_name,
        Records=[record]
    )

def lambda_handler(event, context):
    source_database = 'VBT_TIMESTREAM_DB'
    source_table = 'VBT_TIMESTREAM_TABLE'
    target_database = 'VBT_TIMESTREAM_DB'
    target_table = 'VBT_STATS'

    print('Received event:', json.dumps(event, indent=2))

    # Extract values from the event
    exercise_id = str(event.get('ExerciseID', '0'))
    accelX = Decimal(str(event.get('AccX', 0)))
    accelY = Decimal(str(event.get('AccY', 0)))
    accelZ = Decimal(str(event.get('AccZ', 0)))
    gyroX = Decimal(str(event.get('GyroX', 0)))
    gyroY = Decimal(str(event.get('GyroY', 0)))
    gyroZ = Decimal(str(event.get('GyroZ', 0)))
    

    # Use the current time for all records
    timestamp_ms = str(int(datetime.utcnow().timestamp() * 1000))

    # Prepare the data to be stored in Timestream
    record = {
        'Dimensions': [
            {'Name': 'ExerciseID', 'Value': exercise_id}
        ],
        'MeasureName': 'sensor_data',
        'MeasureValueType': 'MULTI',
        'Time': timestamp_ms,
        'MeasureValues': [
            {'Name': 'AccX', 'Value': str(accelX), 'Type': 'DOUBLE'},
            {'Name': 'AccY', 'Value': str(accelY), 'Type': 'DOUBLE'},
            {'Name': 'AccZ', 'Value': str(accelZ), 'Type': 'DOUBLE'},
            {'Name': 'GyroX', 'Value': str(gyroX), 'Type': 'DOUBLE'},
            {'Name': 'GyroY', 'Value': str(gyroY), 'Type': 'DOUBLE'},
            {'Name': 'GyroZ', 'Value': str(gyroZ), 'Type': 'DOUBLE'}
        ]
    }

    try:
        # Save the sensor data to Timestream
        timestream_write_client.write_records(
            DatabaseName=source_database,
            TableName=source_table,
            Records=[record]
        )
        print("Data saved:", json.dumps([record], indent=2, cls=DecimalEncoder))

        # Query the exercise data
        rows = query_exercise_data(source_database, source_table, exercise_id)
        if rows:
            # Calculate VBT statistics
            stats = calculate_vbt_statistics(rows)
            if stats:
                # Store the statistics in the VBT_STATS table
                store_statistics(target_database, target_table, exercise_id, stats)

        return {
            'statusCode': 200,
            'body': json.dumps('Data written to Timestream and statistics calculated.')
        }
    except timestream_write_client.exceptions.RejectedRecordsException as e:
        print("RejectedRecords:", e.response['RejectedRecords'])
        return {
            'statusCode': 400,
            'body': json.dumps('Some records were rejected. See logs for details.')
        }
    except Exception as e:
        print("Unable to add item. Error:", str(e))
        return {
            'statusCode': 500,
            'body': json.dumps('Failed to write data to Timestream')
        }
