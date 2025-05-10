#include <Wire.h>
#include <LSM9DS1.h>
#include "WiFi.h"
#include "ESPAsyncWebServer.h"
#include <math.h>

#ifndef DEG_TO_RAD
#define DEG_TO_RAD 0.017453292519943295769236907684886
#endif

#ifndef RAD_TO_DEG
#define RAD_TO_DEG 57.295779513082320876798154814105
#endif

// WiFi Credentials
const char* ssid = "ESP32-Access-Point-Steven";
const char* password = "12345678";

// Initialize Server and Sensor
AsyncWebServer server(80);
LSM9DS1Class imu(Wire);

float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;

// Quaternion representation
struct Quaternion {
    float w, x, y, z;
};

// Direction Cosine Matrix (DCM)
float DCM[3][3] = { {1, 0, 0}, {0, 1, 0}, {0, 0, 1} };

// Initial quaternion
Quaternion q = {1, 0, 0, 0};

// Time tracking
unsigned long lastUpdate = 0; // Time of last update
float dt = 0.05; // Time step

// Function prototypes
void updateIMU();
void updateDCM(float gx, float gy, float gz);
void updateQuaternion(float gx, float gy, float gz);
void computeEulerFromDCM(float &roll, float &pitch, float &yaw);
void computeEulerFromQuaternion(float &roll, float &pitch, float &yaw);
void normalizeDCM(float DCM[3][3]);

void normalizeDCM(float DCM[3][3]) {
  
  float renorm;

  // Normalize each row
  for (int i = 0; i < 3; i++) {
      renorm = sqrt(DCM[i][0] * DCM[i][0] + DCM[i][1] * DCM[i][1] + DCM[i][2] * DCM[i][2]);
      DCM[i][0] /= renorm;
      DCM[i][1] /= renorm;
      DCM[i][2] /= renorm;
  }
}


void calibrateGyro() {
  float sumX = 0, sumY = 0, sumZ = 0;
  int numSamples = 200;

  for (int i = 0; i < numSamples; i++) {
      float gx, gy, gz;
      IMU.readGyroscope(gx, gy, gz);
      sumX += gx;
      sumY += gy;
      sumZ += gz;
      delay(10);
  }

  gyroBiasX = sumX / numSamples;
  gyroBiasY = sumY / numSamples;
  gyroBiasZ = sumZ / numSamples;
}

// Read IMU Data and Update Orientation
void updateIMU() {
  float gx, gy, gz, ax, ay, az, mx, my, mz;

  if (IMU.readGyroscope(gx, gy, gz) && IMU.readAcceleration(ax, ay, az) && IMU.readMagneticField(mx, my, mz)) {
    double alpha = 0.01;
    float gyrox, gyroy, gyroz = 0;

    if (gx < 2*gyroBiasX) {
        gyrox = gx - gyroBiasX;
    } else {
        gyrox = gx;
    }
    
    if (gy < 2*gyroBiasY) {
        gyroy = gy - gyroBiasY;
    } else {
        gyroy = gy;
    }
    
    if (gz < 2*gyroBiasZ) {
        gyroz = gz - gyroBiasZ;
    } else {
        gyroz = gz;
    }
    
    

    gyrox *= DEG_TO_RAD; // Convert to radians
    gyroy *= DEG_TO_RAD;
    gyroz *= DEG_TO_RAD;

      unsigned long currentTime = millis();
      dt = (currentTime - lastUpdate) / 1000.0;
      lastUpdate = currentTime;

      updateDCM(gyrox, gyroy, gyroz);
      updateQuaternion(gyrox, gyroy, gyroz);
  }
}


// Update DCM using gyroscope data
void updateDCM(float gx, float gy, float gz) {
  // Construct the Skew-Symmetric Angular Velocity Matrix
  float omega[3][3] = {
    { 0, -gz, gy },
    { gz, 0, -gx },
    { -gy, gx, 0 }
};


  float tempDCM[3][3];

  // Update DCM using matrix multiplication: DCM' = DCM + DCM * omega * dt
  for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
          tempDCM[i][j] = DCM[i][j] + dt * (
              DCM[i][0] * omega[0][j] +
              DCM[i][1] * omega[1][j] +
              DCM[i][2] * omega[2][j]
          );
      }
  }

  //Normalize the Updated DCM
  normalizeDCM(tempDCM);

  // Store the Corrected DCM Back
  for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
          DCM[i][j] = tempDCM[i][j];
      }
  }
}

void computeEulerFromAccelerometer(float ax, float ay, float az, float &rollAcc, float &pitchAcc) {
  // Compute roll and pitch based on accelerometer readings
  rollAcc = atan2(-ay, az) * RAD_TO_DEG;
  pitchAcc = atan2(-ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG;
}

// Update Quaternion using gyroscope data
void updateQuaternion(float gx, float gy, float gz) {
  float qw = q.w, qx = q.x, qy = q.y, qz = q.z;

  q.w += (-qx * gx - qy * gy - qz * gz) * (dt / 2);
  q.x += (qw * gx + qy * gz - qz * gy) * (dt / 2);
  q.y += (qw * gy - qx * gz + qz * gx) * (dt / 2);
  q.z += (qw * gz + qx * gy - qy * gx) * (dt / 2);

  float norm = sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
  q.w /= norm;
  q.x /= norm;
  q.y /= norm;
  q.z /= norm;
}

// Compute Euler angles from DCM
void computeEulerFromDCM(float &roll, float &pitch, float &yaw) {
  roll = atan2(DCM[2][1], DCM[2][2]) * RAD_TO_DEG;
  pitch = asin(-DCM[2][0]) * RAD_TO_DEG;
  yaw = atan2(DCM[1][0], DCM[0][0]) * RAD_TO_DEG;
}

// Compute Euler angles from Quaternion
void computeEulerFromQuaternion(float &roll, float &pitch, float &yaw) {
  roll = atan2(2.0 * (q.w * q.x + q.y * q.z), 1 - 2.0 * (q.x * q.x + q.y * q.y)) * RAD_TO_DEG;
  pitch = asin(2.0 * (q.w * q.y - q.z * q.x)) * RAD_TO_DEG;
  yaw = atan2(2.0 * (q.w * q.z + q.x * q.y), 1 - 2.0 * (q.y * q.y + q.z * q.z)) * RAD_TO_DEG;
}

void setup() {
    Serial.begin(9600);
    while (!Serial);
    
    if (!IMU.begin()) {
        Serial.println("Failed to initialize IMU!");
        while (1);
    }
    
    Serial.println("IMU Initialized");
    // Set up WiFi as Access Point
    WiFi.softAP(ssid, password);
    Serial.print("AP IP address: ");
    Serial.println(WiFi.softAPIP());
    // Start the Web Server
    server.begin();

   
    delay(2000);  // Wait 2s for sensor stabilization
    calibrateGyro();
    lastUpdate = millis();
    Serial.println("Starts Measuring");
}



#define ENABLE_ACCURACY_MEASUREMENT  // Comment this to disable accuracy measurement

void loop() {
    updateIMU();  // Read IMU & update DCM, Quaternion

    // Compute Euler Angles
    float rollDCM, pitchDCM, yawDCM;
    float rollQ, pitchQ, yawQ;
    computeEulerFromDCM(rollDCM, pitchDCM, yawDCM);
    computeEulerFromQuaternion(rollQ, pitchQ, yawQ);

#ifdef ENABLE_ACCURACY_MEASUREMENT  // Accuracy measurement enabled
    // Read accelerometer data
    float ax, ay, az;
    if (IMU.readAcceleration(ax, ay, az)) {
        float rollAcc, pitchAcc;
        computeEulerFromAccelerometer(ax, ay, az, rollAcc, pitchAcc);

        // Compute accuracy errors
        float rollErrorDCM = abs(rollDCM - rollAcc);
        float pitchErrorDCM = abs(pitchDCM - pitchAcc);
        float rollErrorQ = abs(rollQ - rollAcc);
        float pitchErrorQ = abs(pitchQ - pitchAcc);

        // Compute accuracy percentage (avoid division by zero)
        float rollAccuracyDCM = (abs(rollAcc) > 0.01) ? (100.0 - (rollErrorDCM / abs(rollAcc)) * 100.0) : 100.0;
        float pitchAccuracyDCM = (abs(pitchAcc) > 0.01) ? (100.0 - (pitchErrorDCM / abs(pitchAcc)) * 100.0) : 100.0;
        float rollAccuracyQ = (abs(rollAcc) > 0.01) ? (100.0 - (rollErrorQ / abs(rollAcc)) * 100.0) : 100.0;
        float pitchAccuracyQ = (abs(pitchAcc) > 0.01) ? (100.0 - (pitchErrorQ / abs(pitchAcc)) * 100.0) : 100.0;

        // Output formatted for Arduino Serial Plotter (with accuracy data)
        Serial.print(rollDCM); Serial.print("\t");
        Serial.print(pitchDCM); Serial.print("\t");
        Serial.print(yawDCM); Serial.print("\t");
        Serial.print(rollQ); Serial.print("\t");
        Serial.print(pitchQ); Serial.print("\t");
        Serial.print(yawQ); Serial.print("\t");
        Serial.print(rollAcc); Serial.print("\t");
        Serial.println(pitchAcc); //Serial.print("\t");
        // Serial.print(rollAccuracyDCM); Serial.print("\t");  // DCM Roll Accuracy (%)
        // Serial.print(pitchAccuracyDCM); Serial.print("\t"); // DCM Pitch Accuracy (%)
        // Serial.print(rollAccuracyQ); Serial.print("\t");    // Quaternion Roll Accuracy (%)
        // Serial.println(pitchAccuracyQ);  // Quaternion Pitch Accuracy (%)

        // HTTP Route to Send IMU Data
        server.on("/angles", HTTP_GET, [&rollDCM, &pitchDCM, &yawDCM, &rollQ, &pitchQ, &yawQ](AsyncWebServerRequest *request) {
          String jsonPayload = "{\"rollDCM\": " + String(rollDCM, 2) + 
                               ", \"pitchDCM\": " + String(pitchDCM, 2) + 
                               ", \"yawDCM\": " + String(yawDCM, 2) + 
                               ", \"rollQ\": " + String(rollQ, 2) + 
                               ", \"pitchQ\": " + String(pitchQ, 2) + 
                               ", \"yawQ\": " + String(yawQ, 2) + "}";
          request->send(200, "application/json", jsonPayload);
      });
      
    }
#else  // Accuracy measurement disabled
    // Output formatted for Arduino Serial Plotter (without accuracy data)
    Serial.print(rollDCM); Serial.print("\t");
    Serial.print(pitchDCM); Serial.print("\t");
    Serial.print(yawDCM); Serial.print("\t");
    Serial.print(rollQ); Serial.print("\t");
    Serial.print(pitchQ); Serial.print("\t");
    Serial.println(yawQ);  // End line for Serial Plotter
#endif

    delay(50);  // Small delay to allow sensor updates
}
