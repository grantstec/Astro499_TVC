/** sensors.h
* ===========================================================
* Name: Flight Controller Sensors Interface
* Section: TVC499
* Project: Flight Controller
* Purpose: Sensor data acquisition and processing
* ===========================================================
*/

#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_BMP3XX.h>
#include <utility/imumaths.h>


// Constants
#define REF_PRESSURE_HPA 0  // Default reference pressure in hPa



/**
 * @brief Initialize IMU and altimeter sensors
 * @param bno Pointer to BNO085 sensor object
 * @param bmp Pointer to BMP3XX sensor object
 * @return True if initialization successful, false otherwise
 */
bool initializeSensors(Adafruit_BNO08x* bno, Adafruit_BMP3XX* bmp, double* quaternions, double* accelerometer, double& refPressure);

/**
 * @brief Get gyroscope data from IMU with offset compensation
 * @param bno Pointer to BNO085 sensor object
 * @param gyroRates Output array for gyroscope rates [x,y,z]
 * @param gyroOffsets Input array containing gyroscope offsets [x,y,z]
 */
void updateIMU(Adafruit_BNO08x* bno, double* gyroRates, double* quaternions, double* eulerAngles, double* accelerometer, double dt);

void initializeQuaternions(Adafruit_BNO08x* bno, double* quaternions, double* accelerometer);

/**
 * @brief Get altitude and pressure data from altimeter
 * @param bmp Pointer to BMP3XX sensor object
 * @param altData Array to store altitude, pressure, and temperature [altitude,pressure,temp]
 * @param refPressure Reference pressure for altitude calculation
 * @return True if reading successful, false otherwise
 */
bool updateAltimeter(Adafruit_BMP3XX* bmp, double altData[3], double& refPressure);

/**
 * @brief Extract data from sensor event into data array
 * @param event Pointer to sensor event structure
 * @param data Array to store extracted data [x,y,z]
 */
void returnData(sensors_event_t* event, double data[3]);


/**
 * @brief IMU zeroing function to reset gyro data
 * @param bno Pointer to BNO085 sensor object
 */
void resetIMU();

void resetSensors(Adafruit_BNO08x* bno, Adafruit_BMP3XX* bmp, double* quaternions, double* accelerometer, double& refPressure);
/**
 * @brief Set reference pressure for altitude calculation
 * @param bmp Pointer to BMP3XX sensor object
 * @param refPressure Pointer to reference pressure value
 */
void zeroAltimeter(Adafruit_BMP3XX* bmp, double& refPressure);

#endif // SENSORS_H