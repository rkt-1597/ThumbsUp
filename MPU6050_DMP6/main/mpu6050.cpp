// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//		2023-03-10 - Fit to esp-idf v5
//		2019-07-08 - Added Auto Calibration and offset generator
//		   - and altered FIFO retrieval sequence to avoid using blocking code
//		2016-04-18 - Eliminated a potential infinite loop
//		2013-05-08 - added seamless Fastwire support
//				   - added note about gyro calibration
//		2012-06-21 - added note about Arduino 1.0.1 + Leonardo compatibility error
//		2012-06-20 - improved FIFO overflow handling and simplified read process
//		2012-06-19 - completely rearranged DMP initialization code and simplification
//		2012-06-13 - pull gyro and accel data from FIFO packet instead of reading directly
//		2012-06-09 - fix broken FIFO read sequence and change interrupt detection to RISING
//		2012-06-05 - add gravity-compensated initial reference frame acceleration output
//				   - add 3D math helper file to DMP6 example sketch
//				   - add Euler output and Yaw/Pitch/Roll output formats
//		2012-06-04 - remove accel offset clearing for better results (thanks Sungon Lee)
//		2012-06-01 - fixed gyro sensitivity to be 2000 deg/sec instead of 250
//		2012-05-30 - basic DMP initialization working

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/message_buffer.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/i2c.h"
#include "cJSON.h"

#include "parameter.h"

extern QueueHandle_t xQueueTrans;
extern int conditionValue;
extern MessageBufferHandle_t xMessageBufferToClient;

static const char *TAG = "IMU";

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

//#include "MPU6050.h" // not necessary if using MotionApps include file
#include "MPU6050_6Axis_MotionApps20.h"

#define RAD_TO_DEG (180.0/M_PI)
#define DEG_TO_RAD 0.0174533

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;	// set true if DMP init was successful
uint8_t mpuIntStatus;	// holds actual interrupt status byte from MPU
uint8_t devStatus;		// return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;	// expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;		// count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;			// [w, x, y, z]			quaternion container
VectorInt16 aa;			// [x, y, z]			accel sensor measurements
VectorInt16 aaReal;		// [x, y, z]			gravity-free accel sensor measurements
VectorInt16 aaWorld;	// [x, y, z]			world-frame accel sensor measurements
VectorFloat gravity;	// [x, y, z]			gravity vector
float euler[3];			// [psi, theta, phi]	Euler angle container
float ypr[3];			// [yaw, pitch, roll]	yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

// display quaternion values in easy matrix form: w x y z
void getQuaternion() {
	mpu.dmpGetQuaternion(&q, fifoBuffer);
	printf("quat x:%6.2f y:%6.2f z:%6.2f w:%6.2f\n", q.x, q.y, q.z, q.w);
}

// display Euler angles in degrees
void getEuler() {
	mpu.dmpGetQuaternion(&q, fifoBuffer);
	mpu.dmpGetEuler(euler, &q);
	printf("euler psi:%6.2f theta:%6.2f phi:%6.2f\n", euler[0] * RAD_TO_DEG, euler[1] * RAD_TO_DEG, euler[2] * RAD_TO_DEG);
}

// display Euler angles in degrees
void getYawPitchRoll() {
// 	mpu.dmpGetQuaternion(&q, fifoBuffer);
// 	mpu.dmpGetGravity(&gravity, &q);
// 	mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
// #if 0
// 	float _roll = ypr[2] * RAD_TO_DEG;
// 	float _pitch = ypr[1] * RAD_TO_DEG;
// 	float _yaw = ypr[0] * RAD_TO_DEG;
// 	ESP_LOGI(TAG, "roll:%f pitch:%f yaw:%f",_roll, _pitch, _yaw);
// #endif
// 	//printf("ypr roll:%3.1f pitch:%3.1f yaw:%3.1f\n",ypr[2] * RAD_TO_DEG, ypr[1] * RAD_TO_DEG, ypr[0] * RAD_TO_DEG);
// 	ESP_LOGI(TAG, "roll:%f pitch:%f yaw:%f",ypr[2] * RAD_TO_DEG, ypr[1] * RAD_TO_DEG, ypr[0] * RAD_TO_DEG);
static TickType_t fanConditionStart = 0; // Start time when FAN condition is first satisfied
    static TickType_t lightConditionStart = 0; // Start time when LIGHT condition is first satisfied
    static uint8_t fanState = 0;  // 0: FAN OFF, 1: FAN ON
    static uint8_t lightState = 0;  // 0: LIGHT OFF, 1: LIGHT ON

    const TickType_t delayTime = 7000 / portTICK_PERIOD_MS; // 7 seconds in ticks

    // Read the MPU data
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    TickType_t currentTime = xTaskGetTickCount(); // Get current tick count
    ESP_LOGI(TAG, "roll:%f pitch:%f yaw:%f", ypr[2] * RAD_TO_DEG, ypr[1] * RAD_TO_DEG, ypr[0] * RAD_TO_DEG);

    // --- FAN ON/OFF Logic ---
    if ((ypr[2] * RAD_TO_DEG) > 60.0) {  // FAN ON condition
        if (fanState == 0) { // FAN is currently OFF
            if (fanConditionStart == 0) fanConditionStart = currentTime; // Note the start time
            if ((currentTime - fanConditionStart) >= delayTime) {
                printf("FAN ON\n");
				conditionValue = 1; // 2 represents FAN ON
            	ESP_LOGE(TAG, "Calculated Condition Value: %d", conditionValue);
            	xQueueSend(xQueueTrans, &conditionValue, 0);
				ESP_LOGI(TAG, "Queued Condition Value: %d", conditionValue);
                fanState = 1; // Set FAN state to ON
                fanConditionStart = 0; // Reset the timer
            }
        }
    } else if ((ypr[2] * RAD_TO_DEG) < -60.0) {  // FAN OFF condition
        if (fanState == 1) { // FAN is currently ON
            if (fanConditionStart == 0) fanConditionStart = currentTime; // Note the start time
            if ((currentTime - fanConditionStart) >= delayTime) {
                printf("FAN OFF\n");
				conditionValue = 2; // 2 represents FAN ON
            	ESP_LOGE(TAG, "Calculated Condition Value: %d", conditionValue);
            	xQueueSend(xQueueTrans, &conditionValue, 0);
				ESP_LOGI(TAG, "Queued Condition Value: %d", conditionValue);
                fanState = 0; // Set FAN state to OFF
                fanConditionStart = 0; // Reset the timer
            }
        }
    } else {
        fanConditionStart = 0; // Reset the timer if condition is not continuously satisfied
    }

    // --- LIGHT ON/OFF Logic ---
    if (((ypr[1] * RAD_TO_DEG) > 40.0)) {  // LIGHT ON condition
        if (lightState == 0) { // LIGHT is currently OFF
            if (lightConditionStart == 0) lightConditionStart = currentTime; // Note the start time
            if ((currentTime - lightConditionStart) >= delayTime) {
                printf("LIGHT ON\n");
				conditionValue = 3; // 2 represents FAN ON
            	ESP_LOGE(TAG, "Calculated Condition Value: %d", conditionValue);
            	xQueueSend(xQueueTrans, &conditionValue, 0);
				ESP_LOGI(TAG, "Queued Condition Value: %d", conditionValue);
                lightState = 1; // Set LIGHT state to ON
                lightConditionStart = 0; // Reset the timer
            }
        }
    } else {  // LIGHT OFF condition
        if (lightState == 1) { // LIGHT is currently ON
            if (lightConditionStart == 0) lightConditionStart = currentTime; // Note the start time
            if ((currentTime - lightConditionStart) >= delayTime) {
                printf("LIGHT OFF\n");
				conditionValue = 4; // 2 represents FAN ON
				ESP_LOGE(TAG, "Calculated Condition Value: %d", conditionValue);
            	xQueueSend(xQueueTrans, &conditionValue, 0);
				ESP_LOGI(TAG, "Queued Condition Value: %d", conditionValue);

                lightState = 0; // Set LIGHT state to OFF
                lightConditionStart = 0; // Reset the timer
            }
        } else {
            lightConditionStart = 0; // Reset the timer if condition is not continuously satisfied
        }
    }

}

// display real acceleration, adjusted to remove gravity
void getRealAccel() {
	mpu.dmpGetQuaternion(&q, fifoBuffer);
	mpu.dmpGetAccel(&aa, fifoBuffer);
	mpu.dmpGetGravity(&gravity, &q);
	mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
	printf("areal x=%d y:%d z:%d\n", aaReal.x, aaReal.y, aaReal.z);
}

// display initial world-frame acceleration, adjusted to remove gravity
// and rotated based on known orientation from quaternion
void getWorldAccel() {
	mpu.dmpGetQuaternion(&q, fifoBuffer);
	mpu.dmpGetAccel(&aa, fifoBuffer);
	mpu.dmpGetGravity(&gravity, &q);
	mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
	mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
	printf("aworld x:%d y:%d z:%d\n", aaWorld.x, aaWorld.y, aaWorld.z);
}

void mpu6050(void *pvParameters){
	// Initialize mpu6050
	mpu.initialize();

	// Get Device ID
	uint8_t buffer[1];
	I2Cdev::readByte(MPU6050_ADDRESS_AD0_LOW, MPU6050_RA_WHO_AM_I, buffer);
	ESP_LOGI(TAG, "getDeviceID=0x%x", buffer[0]);

	// Initialize DMP
	devStatus = mpu.dmpInitialize();
	ESP_LOGI(TAG, "devStatus=%d", devStatus);
	if (devStatus != 0) {
		ESP_LOGE(TAG, "DMP Initialization failed [%d]", devStatus);
		while(1) {
			vTaskDelay(1);
		}
	}

	// This need to be setup individually
	// supply your own gyro offsets here, scaled for min sensitivity
	mpu.setXAccelOffset(1418);
	mpu.setYAccelOffset(1692);
	mpu.setZAccelOffset(1347);
	mpu.setXGyroOffset(124);
	mpu.setYGyroOffset(49);
	mpu.setZGyroOffset(0);

	// Calibration Time: generate offsets and calibrate our MPU6050
	mpu.CalibrateAccel(6);
	mpu.CalibrateGyro(6);
	mpu.setDMPEnabled(true);

	while(1){
		if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet
			getYawPitchRoll();
			float _roll = ypr[2] * RAD_TO_DEG;
			float _pitch = ypr[1] * RAD_TO_DEG;
			float _yaw = ypr[0] * RAD_TO_DEG;

			// Send UDP packet
			POSE_t pose;
			pose.roll = _roll;
			pose.pitch = _pitch;
			pose.yaw = _yaw;
			if (xQueueSend(xQueueTrans, &pose, 100) != pdPASS ) {
				ESP_LOGE(TAG, "xQueueSend fail");
			}

			// // Send WEB request
			// cJSON *request;
			// request = cJSON_CreateObject();
			// cJSON_AddStringToObject(request, "id", "data-request");
			// cJSON_AddNumberToObject(request, "roll", _roll);
			// cJSON_AddNumberToObject(request, "pitch", _pitch);
			// cJSON_AddNumberToObject(request, "yaw", _yaw);
			// char *my_json_string = cJSON_Print(request);
			// ESP_LOGD(TAG, "my_json_string\n%s",my_json_string);
			// size_t xBytesSent = xMessageBufferSend(xMessageBufferToClient, my_json_string, strlen(my_json_string), 100);
			// if (xBytesSent != strlen(my_json_string)) {
			// 	ESP_LOGE(TAG, "xMessageBufferSend fail");
			// }
			// cJSON_Delete(request);
			// cJSON_free(my_json_string);

			//getQuaternion();
			//getEuler();
			//getRealAccel();
			//getWorldAccel();
		}

		// Best result is to match with DMP refresh rate
		// Its last value in components/MPU6050/MPU6050_6Axis_MotionApps20.h file line 310
		// Now its 0x13, which means DMP is refreshed with 10Hz rate
		vTaskDelay(1000/portTICK_PERIOD_MS);
	}

	// Never reach here
	vTaskDelete(NULL);
}
