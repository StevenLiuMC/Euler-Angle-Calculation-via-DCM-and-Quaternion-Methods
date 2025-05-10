# Euler Angle Calculation via DCM and Quaternion Methods

This project implements and compares two methods for calculating 3D orientation (Euler angles) using an ESP32 microcontroller and LSM9DS1 IMU sensor: **Direct Cosine Matrix (DCM)** and **Quaternion**.

## Overview

The system captures gyroscope data in real-time, processes it using both DCM and Quaternion methods, and calculates roll, pitch, and yaw angles. The implementation includes bias calibration, matrix normalization, and accuracy comparison with accelerometer readings. Additionally, the system is configured as a WiFi access point to transmit orientation data to a receiver device.

## Features

-   **Dual Method Implementation**: Simultaneous computation using both DCM and Quaternion approaches.
-   **Gyroscope Bias Calibration**: Automatic calibration at startup to reduce drift.
-   **Accuracy Measurement**: Compares calculated angles with accelerometer reference values.
-   **Real-time Visualization**: Serial plotter compatible output format.
-   **Wireless Communication**: WiFi transmission of orientation data.
-   **Normalized Calculations**: Matrix orthogonality preservation and quaternion normalization.
-   **Dynamic Bias Application**: Smart bias application for better accuracy at varying speeds.

## Hardware Requirements

-   ESP32 Development Board (x2 - one for sensor server, one for receiver)
-   LSM9DS1 IMU Sensor
-   USB cables for programming and serial monitoring
-   Power source (USB or battery)

## Software Dependencies

-   Arduino IDE
-   ESP32 Board Package
-   LSM9DS1 Library
-   ESP Async Web Server Library
-   WiFi Library
-   HTTPClient Library

## Pin Connections

**ESP32 (Sensor) <---> LSM9DS1**
-   `3.3V` -------------- `VIN`
-   `GND` --------------- `GND`
-   `D21` --------------- `SDA`
-   `D22` --------------- `SCL`

## Project Structure

The project consists of two main parts:

1.  **`main_server.cpp` (Main Server):**
    * Configures the ESP32 as an Access Point.
    * Initializes the IMU sensor.
    * Implements DCM and Quaternion calculations.
    * Provides a web server endpoint for orientation data.
2.  **`main_receiver.cpp` (Main Receiver):**
    * Connects to the ESP32 Access Point.
    * Periodically fetches orientation data.
    * Parses and displays the received angles.

## Setup Instructions

1.  **Install Required Libraries:**
    * Install the ESP32 board package in Arduino IDE.
    * Install the LSM9DS1 library (e.g., from Adafruit or SparkFun).
    * Install the ESPAsyncWebServer library.
    * Ensure WiFi and HTTPClient libraries are available (usually part of the ESP32 core).
2.  **Upload Server Code:**
    * Open `main_server.cpp` in Arduino IDE.
    * Select the correct ESP32 board and port.
    * Upload the code to the first ESP32.
3.  **Upload Receiver Code:**
    * Open `main_receiver.cpp` in Arduino IDE.
    * Select the correct ESP32 board and port.
    * Upload the code to the second ESP32.
4.  **Monitor Results:**
    * Open the Serial Monitor for both ESP32 boards.
    * Set the baud rate to 9600.
    * Observe the orientation data.

## Configuration Options

The following parameters can be adjusted in the code (e.g., in `main_server.cpp`):

```cpp
// WiFi Credentials
const char* ssid = "ESP32-Access-Point-Steven";
const char* password = "12345678";

// Timing
const long interval = 200;  // Fetch interval in milliseconds
float dt = 0.01;            // Time step for integration

// Sensor Calibration
int numSamples = 200;       // Number of samples for gyroscope calibration
```

## DCM Method

The DCM method represents orientation using a $3 \times 3$ rotation matrix that transforms coordinates from a body-fixed frame to an earth-fixed frame. Each element of the matrix represents the cosine of the angle between one of the body-fixed axes and one of the earth-fixed axes.

Key implementation aspects:
-   Angular velocity matrix construction
-   Matrix multiplication for DCM update
-   Normalization to maintain orthogonality
-   Extraction of Euler angles

## Quaternion Method

The Quaternion method represents orientation using a four-dimensional extension of complex numbers. This approach avoids gimbal lock and provides a more computationally efficient representation.

Key implementation aspects:
-   Quaternion propagation equation implementation
-   Normalization to maintain unit quaternion
-   Extraction of Euler angles

## Performance Analysis

Through experimental testing, the following observations were made:

-   **Accuracy**: Quaternion method generally shows higher accuracy than DCM due to lower computational complexity, resulting in less accumulated drift.
-   **Computation Efficiency**: Quaternion updates are more efficient than DCM matrix operations, making them better suited for real-time applications.
-   **Drift Handling**: Both methods experience similar drift over time, primarily due to gyroscope sensor limitations rather than algorithmic differences.

**Sensor Limitation Factors:**
-   Bias instability from sensor imperfections
-   Temperature effects on the gyroscope
-   Motion speed affecting integration accuracy

## Known Limitations

-   Gyroscope drift affects both methods over time.
-   Rapid motion may result in increased integration errors.
-   Yaw measurements (Z-axis rotation) lack absolute reference without magnetometer data.
-   No filtering implemented for dynamic error correction (like Madgwick or Mahony filters).

## Future Improvements

-   Implementation of sensor fusion algorithms (Madgwick or Mahony filters).
-   Integration of magnetometer data for absolute heading reference.
-   Dynamic temperature compensation.
-   Adaptive sampling rate based on motion intensity.
-   Enhanced visualization tools.
