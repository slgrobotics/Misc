/*!
 *@file detectGesture.ino
 *@brief Detect gestures
 *@details  This code detects the position, score of faces, and gestures along with their scores. It interacts with the DFRobot GestureFaceDetection sensor to perform face and gesture detection and display the results.
 *@copyright   Copyright (c) 2025 DFRobot Co.Ltd (http://www.dfrobot.com)
 *@license     The MIT license (MIT)
 *@author [thdyyl](yuanlong.yu@dfrobot.com)
 *@version  V1.0
 *@date  2025-03-31
 *@https://github.com/DFRobot/DFRobot_GestureFaceDetection
 */

#include "DFRobot_GestureFaceDetection.h"

// Devkitc_v4 connections:
//     GND (Black)
//     5V  (Red)
//     GPIO 21 - SDA (Green)
//     GPIO 22 - SCL (Blue)


// Define the I2C device ID for the GestureFaceDetection sensor
#define DEVICE_ID 0x72

// Create an instance of DFRobot_GestureFaceDetection_I2C with the specified device ID
DFRobot_GestureFaceDetection_I2C gfd(DEVICE_ID);

// Buffer for formatted output
char str[100];

void setup() {
  // Wait for the sensor to start.
  delay(5000);

  // Initialize I2C communication
  gfd.begin(&Wire);

  // Initialize serial communication for debugging purposes
  Serial.begin(115200);

  while (!gfd.begin()) {
    Serial.println("Communication with device failed, please check connection");
    delay(1000);
  }
  Serial.print("face detection threshold: ");
  Serial.println(gfd.getFaceDetectThres());

  Serial.print("gesture detection threshold: ");
  Serial.println(gfd.getGestureDetectThres());

  Serial.print("gesture detection range: ");
  Serial.println(gfd.getDetectThres());
}

void loop() {
  // Check if any faces are detected
  if (gfd.getFaceNumber() > 0) {
    // Retrieve face score and location
    uint16_t faceScore = gfd.getFaceScore();
    uint16_t faceX = gfd.getFaceLocationX();
    uint16_t faceY = gfd.getFaceLocationY();

    // Print the face detection results
    sprintf(str, "detect face at (x = %d, y = %d, score = %d)\n", faceX, faceY, faceScore);
    Serial.print(str);

    // Print the gesture detection results
    // - 1: LIKE (👍) - blue
    // - 2: OK (👌) - green
    // - 3: STOP (🤚) - red
    // - 4: YES (✌) - yellow
    // - 5: SIX (🤙) - purple
    uint16_t gestureType = gfd.getGestureType();
    uint16_t gestureScore = gfd.getGestureScore();

    // Print the gesture detection results
    sprintf(str, "detect gesture %d, score = %d\n", gestureType, gestureScore);
    Serial.print(str);
  }

  // Delay before the next loop iteration
  delay(500);
}
