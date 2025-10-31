# System-Vital-Code

# HealthSense-RTOS: Real-Time Biometric Monitor
This project implements a comprehensive, multi-sensor health monitoring system using an ESP32 microcontroller, leveraging FreeRTOS for robust, concurrent data acquisition and transmission. It continuously measures and streams vital signs—including ECG, Heart Rate (HR), SpO₂, Body Temperature, and Non-Invasive Blood Pressure (NIBP) — to a remote server via WiFi.

# Software Architecture
The system's core is built around FreeRTOS tasks to handle different processes simultaneously:

# Task	Core	Function
TaskECGFunction	Core 0	High-speed, continuous sampling of raw ECG data.
TaskECGSendFunction	Core 1	Batches ECG samples and adds the large array pointer to the WiFi Queue.
TaskSensorsFunction	Core 1	Manages slower, scheduled readings (Temp, HR/SpO₂, BP) and formats the data string.
TaskWiFiSendFunction	Core 1	Handles connection, uses the wifiMutex, and sends data chunks from both ECG and Sensor Queues.

# Note on Blood Pressure Measurement
The blood pressure measurement function (measureBloodPressure) uses a simplified oscillometric method based on pressure oscillation detection during gradual cuff deflation. This implementation is for demonstration and experimentation purposes only and should not be used for actual medical diagnosis. Reliable clinical blood pressure measurement requires advanced algorithms and hardware calibration.
