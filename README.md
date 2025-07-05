# 🏋️‍♂️ VBT Device – Velocity-Based Training IoT System

This project objective consists of implementing a wearable IoT device for **Velocity-Based Training (VBT)**, capable of capturing motion data during strength exercises, analyzing it in real time, and offering useful feedback for athletes and coaches.

> 📚 **Course**: Internet das Coisas (IoT)  
> 🏫 **Institution**: Universidade de Coimbra – DEEC  
> 📅 **Year**: 2023/2024  
> 👥 **Authors**: Gonçalo Bastos - eusoudebastos@gmail.com, Leonardo Gonçalves - leoleocordeiro@gmail.com

---

## 🎯 Project Overview

VBT prioritizes movement speed over traditional load-based approaches. The proposed device helps optimize training by:

- Measuring **movement velocity**, **power output**, and **symmetry**
- Counting reps and tracking the **bar path**
- Providing **real-time feedback** via dashboards and mobile interfaces

---

## 🛠️ Technologies & Architecture

| Component          | Technology/Tool                          |
|--------------------|-------------------------------------------|
| Microcontroller    | ESP32 (initially planned Zolertia Re-mote) |
| Sensor             | MPU-6050 (originally Grove 6-axis IMU)    |
| Communication      | I2C, MQTT over TLS (port 8883)            |
| Cloud Platform     | AWS IoT Core, Lambda, Cognito, Timestream |
| Data Dashboard     | Grafana                                   |
| Programming        | Arduino IDE (C++), AWS Lambda (Python)    |

---

## 📡 System Architecture

The following diagram shows the complete end-to-end architecture of the VBT system, from sensor acquisition to cloud-based processing and user interface:

<p align="center">
  <img src="images/diagrama_integracao.png" alt="System Integration Diagram" width="85%">
</p>


1. **Data Acquisition**  
   ESP32 collects IMU data and processes it locally.

2. **Data Transmission**  
   Data is sent to AWS IoT Core via MQTT with secure TLS.

3. **Cloud Processing**  
   - **AWS Lambda** processes messages
   - Metrics are calculated (velocity, power, reps)

4. **Data Storage**  
   - **AWS Timestream** stores time-series exercise data

5. **Visualization**  
   - **Grafana** dashboards provide interactive performance tracking.

---

## 📱 Mobile App & Interface

- Initially planned via **AWS Amplify**, but replaced by **Grafana dashboards**
- Includes:
  - Real-time velocity monitoring
  - Historical performance tracking
  - Custom insights and training suggestions

---

## 🧪 Implementation Notes

- Shift from Zolertia to ESP32 due to library compatibility
- Use of MQTT with TLS and IAM policies
- Cloud pipeline working end-to-end with Lambda and Grafana
- Future improvement: Optimize Lambda processing logic

---

## 🔐 Security Notes

- Uses TLS encryption for MQTT communication
- AWS Cognito for user authentication
- IAM policies to secure device and data access

---

## 📌 License

This project was developed for educational purposes as part of the **IoT course @ UC** and is not licensed for commercial use.




