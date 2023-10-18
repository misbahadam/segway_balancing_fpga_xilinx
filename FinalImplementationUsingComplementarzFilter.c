#include <stdio.h>
#include <stdint.h>
#include "xil_printf.h"
#include "xil_cache.h"
#include "xparameters.h"
#include "xgpio.h"
#include "xadcps.h"
#include "PWM.h"
#include "sleep.h"
#include "MotorFeedback.h"
#include "PmodACL.h"
#include "PmodGYRO.h"
#include "PmodDHB1.h"
#include <stdarg.h>
#include <math.h>
#include <time.h>

// Define constants for sensor and motor control
#define GYRO_SENSITIVITY 0.1f
#define ACCEL_SENSITIVITY 0.01f
#define MOTOR_PWM_MAX 5
#define SENSOR_EDGES_TARGET 5 // Adjust this value as needed

#ifdef __MICROBLAZE__
#define CLK_FREQ XPAR_CPU_M_AXI_DP_FREQ_HZ
#else
#define CLK_FREQ XPAR_PS7_UART_1_UART_CLK_FREQ_HZ
#endif

#define GPIO_BASEADDR XPAR_PMODDHB1_0_AXI_LITE_GPIO_BASEADDR
#define PWM_BASEADDR XPAR_PMODDHB1_0_PWM_AXI_BASEADDR
#define MOTOR_FB_BASEADDR XPAR_PMODDHB1_0_MOTOR_FB_AXI_BASEADDR
#define GEARBOX_RATIO 19 //or 48



/************ Global Variables ************/

PmodDHB1 pmodDHB1;
MotorFeedback motorFeedback;
PmodGYRO myDevice;
PmodACL acl; //initialized all the 4 classes to access their methods

// Kalmann filter variables
float roll = 0.0f;      // Roll angle estimate and still have to find a way to calculate it
float rollRate = 0.0f;  // Roll rate estimate
float gyroBias = 1.042f;  // Gyro bias estimate

// EKF matrices and parameters
float P[2][2] = {{1.0f, 0.0f}, {0.0f, 1.0f}};  // Error covariance matrix
float Q[2][2] = {{0.001f, 0.0f}, {0.0f, 0.003f}};  // Process noise covariance
float R = 0.001f;  // Sensor noise covariance

// Motor control variables
int motorPwm = 0;

// FIR filter variables
#define FILTER_ORDER 5
float filterCoeff[FILTER_ORDER] = {0.2f, 0.2f, 0.2f, 0.2f, 0.2f};
float filterBuffer[FILTER_ORDER] = {0.0f};

volatile int16_t sensorEdges = 0;

int16_t xAxis = 0;
int16_t yAxis = 0;
int16_t zAxis = 0;
int16_t temp = 0;
float gx = 0;
float gy = 0;
float gz = 0;
float gx1 = 0;
float gy1 = 0;
float gz1 = 0;
float ax = 0;
float ay = 0;
float az = 0;
float x = 0;
float y = 0;
float z = 0;

#define SIZE 10
float gxArray[SIZE];
float gyArray[SIZE];
float gzArray[SIZE];
float axArray[SIZE];
float ayArray[SIZE];
float azArray[SIZE];

// Complementary filter variables
float alpha = 0.90f; // Weight for the gyroscope data
float beta = 0.10f;  // Weight for the accelerometer data

int16_t rgxArray[SIZE];
int16_t rgyArray[SIZE];
int16_t rgzArray[SIZE];
int16_t raxArray[SIZE];
int16_t rayArray[SIZE];
int16_t razArray[SIZE];

int indexarr = 0;


/************ Function Prototypes ************/

void readSensors();

void updateKalmanFilter(float gyro, float accel, float dt);

void driveMotor();

void EnableCaches();

void DisableCaches();

void DemoCleanup();

// Function to read gyroscope and accelerometer values
void readSensors() {

		// Read gyroscope value (angular velocity in degrees/sec)
		xAxis = GYRO_getX(&myDevice);
		yAxis = GYRO_getY(&myDevice);
		zAxis = GYRO_getZ(&myDevice);
		temp = GYRO_getTemp(&myDevice);

		// Store raw values in arrays
		rgxArray[indexarr] = xAxis;
		rgyArray[indexarr] = yAxis;
		rgzArray[indexarr] = zAxis;

		//This formula scales and converts the raw gyroscope reading for the X-axis (xAxis).
		//It subtracts an offset value of 200, multiplies by a scaling factor of 250, and divides by the maximum value of the raw reading (32768).
		//The resulting value (gx) represents the angular velocity in degrees per second for the X-axis.
		gx = (xAxis - 237) * 250 / 32768; //removed bias
		gy = (yAxis + 39) * 250 / 32768;
		gz = (zAxis - 9.04) * 250 / 32768;

		xil_printf("gx calculated : \r\n",gx);
		xil_printf("gy calculated : \r\n",gy);
		xil_printf("gz calculated : \r\n",gz);

		//If we have to convert the above readings into radians per sec then:
		gx1 = gx * 0.0174533f;
		gy1 = gy * 0.0174533f;
		gz1 = gz * 0.0174533f;

		xil_printf("gx1 calculated : \r\n",gx1);
		xil_printf("gy1 calculated : \r\n",gy1);
		xil_printf("gz1 calculated : \r\n",gz1);

		/*// Store gx, gy, gz in arrays
		gxArray[indexarr] = gx;
		gyArray[indexarr] = gy;
		gzArray[indexarr] = gz;*/

		/*// Increment the index for the next data point
		indexarr = (indexarr + 1) % SIZE;*/

		// Read accelerometer value (acceleration in m/s^2)
		ACL_ReadAccelG(&acl, &x, &y, &z);

/*		// Store raw values in arrays
		raxArray[indexarr] = x;
		rayArray[indexarr] = y;
		razArray[indexarr] = z;*/


		//ax = (x - 0.080) * 9.811; // The scaling factor is applied to convert the accelerometer reading from its raw units to meters per second squared (m/s²).
		//ay = (y + 0.018) * 9.811;
		//az = (z - 0.953) * 9.811;
		ax = (x - 0.01105) * 9.811;
		ay = (y - 0.003937) * 9.811;
		az = (z + 0.01562) * 9.811;

		xil_printf("ax calculated : \r\n",ax);
		xil_printf("ay calculated : \r\n",ay);
		xil_printf("az calculated : \r\n",az);

		// Calculate roll angle (in degrees)
		//roll = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / M_PI;

		/*// Store ax, ay, az in arrays
		axArray[indexarr] = ax;
		ayArray[indexarr] = ay;
		azArray[indexarr] = az;*/

		// Increment the index for the next data point
		/*indexarr = (indexarr + 1) % SIZE;
		if (indexarr == 9)
		{
			indexarr = 0;
		}*/
}

// Function to perform matrix multiplication: C = A * B
void matrixMultiply(float A[][2], float B[][2], float C[][2], int rowsA, int colsA, int colsB) {
    for (int i = 0; i < rowsA; i++) {
        for (int j = 0; j < colsB; j++) {
            C[i][j] = 0.0f;
            for (int k = 0; k < colsA; k++) {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

// Function to perform matrix transpose: A = A'
void matrixTranspose(float A[][2], int rows, int cols) {
    float temp;
    for (int i = 0; i < rows; i++) {
        for (int j = i + 1; j < cols; j++) {
            temp = A[i][j];
            A[i][j] = A[j][i];
            A[j][i] = temp;
        }
    }
}

// Function to perform matrix addition: C = A + B
void matrixAdd(float A[][2], float B[][2], float C[][2], int rows, int cols) {
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            C[i][j] = A[i][j] + B[i][j];
        }
    }
}

// Function to apply FIR filter to sensor readings
float applyFIRFilter(float newValue) {
    float filteredValue = 0.0f;

    // Shift values in filter buffer
    for (int i = FILTER_ORDER - 1; i > 0; i--) {
        filterBuffer[i] = filterBuffer[i - 1];
    }

    // Add new value to filter buffer
    filterBuffer[0] = newValue;

    // Apply filter coefficients
    for (int i = 0; i < FILTER_ORDER; i++) {
        filteredValue += filterCoeff[i] * filterBuffer[i];
    }

    return filteredValue;
}

// Function to update the complementary filter
void updateComplementaryFilter(float gyro, float accel, float dt) {
    // Apply FIR filter to sensor readings
    //float filteredAccel = applyFIRFilter(accel);
    //float filteredGyro = applyFIRFilter(gyro);

    // Calculate roll angle from accelerometer data
    float accelRoll = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / M_PI;

    // Calculate roll angle from gyroscope data
    float gyroRoll = roll + gyro * dt;

    // Combine the two roll angle estimates using the complementary filter
    roll = alpha * gyroRoll + beta * accelRoll;
}

// Function to update the Kalman filter
void updateKalmanFilter(float gyro, float accel, float dt) {
    // Apply FIR filter to sensor readings
    //float filteredAccel = applyFIRFilter(accel);
    //float filteredGyro = applyFIRFilter(gyro);

    // Prediction step
    float A[2][2] = {{1.0f, -dt}, {0.0f, 1.0f}};  // System dynamics matrix
    float B[2][1] = {{dt}, {0.0f}};  // Input matrix
    //float U[1] = {filteredGyro - gyroBias};  // Input (gyro measurement minus bias)
    float U[1] = {gyro - gyroBias};
    //this still has to be calibarated(gyrobias)

    // Predict state and error covariance
    float rollPred = A[0][0] * roll + A[0][1] * rollRate + B[0][0] * U[0];
    float rollRatePred = A[1][0] * roll + A[1][1] * rollRate + B[1][0] * U[0];
    float P_pred[2][2] = {{0.0f}};

    // Calculate predicted error covariance P_pred = A * P * A^T + Q
    // (Assuming matrix multiplication and addition functions are defined)
    matrixMultiply(A, P, P_pred, 2, 2, 2);
    matrixTranspose(A, 2, 2);
    matrixMultiply(P_pred, A, P_pred, 2, 2, 2);
    matrixAdd(P_pred, Q, P_pred, 2, 2);

    // Update step
    float H[1][2] = {{1.0f, 0.0f}};  // Measurement mapping matrix
    float S = H[0][0] * P_pred[0][0] * H[0][0] + R;  // Innovation covariance
    float K[2] = {P_pred[0][0] * H[0][0] / S};  // Kalman gain

    // Update state and error covariance
    //float innovation = filteredAccel - rollPred;  // Innovation (measurement residual)
    float innovation = accel - rollPred;
    roll = rollPred + K[0] * innovation;
    rollRate = rollRatePred;
    P[0][0] = (1.0f - K[0] * H[0][0]) * P_pred[0][0];
}

// Function to drive the motor based on sensor readings
void driveMotor() {

	// Drive the motor until the target number of sensor edges is reached
    while (sensorEdges < SENSOR_EDGES_TARGET) {

        // Update Kalman filter
        //updateKalmanFilter(gx, az, 0.01f); //shoudl check what other axis readings to be sent (gy or gz, ay or az)

        //use the calculated state for further processing like PID controller and quaternion
        //to find the drive angle or can also say control input which is then fed into the motor

        // Calculate motor PWM based on roll angle
        //motorPwm = (int)((roll / 90.0f) * MOTOR_PWM_MAX);

        //DHB1_setMotorSpeeds(&pmodDHB1, 50, 50);

        //DHB1_setMotorSpeeds(&pmodDHB1, 50, 50);



        // Update sensor edges based on motor feedback
        sensorEdges = MotorFeedback_getDistanceTraveled(&motorFeedback);


    }
    MotorFeedback_clearPosCounter(&motorFeedback);
    //DHB1_setMotorSpeeds(&pmodDHB1, 0, 0);
    // Stop the motor and clear counter
    //MotorFeedback_clearPosCounter(&motorFeedback);
    //DHB1_motorDisable(&pmodDHB1);
}

void DemoCleanup()
{
	GYRO_end(&myDevice);
	DisableCaches();
}

void EnableCaches()
{
#ifdef __MICROBLAZE__
#ifdef XPAR_MICROBLAZE_USE_ICACHE
	Xil_ICacheEnable();
#endif
#ifdef XPAR_MICROBLAZE_USE_DCACHE
	Xil_DCacheEnable();
#endif
#endif
}

void DisableCaches()
{
#ifdef __MICROBLAZE__
#ifdef XPAR_MICROBLAZE_USE_DCACHE
	Xil_DCacheDisable();
#endif
#ifdef XPAR_MICROBLAZE_USE_ICACHE
	Xil_ICacheDisable();
#endif
#endif
}

// Function to write data to the text file
void writeToLogFile(float data) {
    // Format the data and timestamp
    char buffer[128];
    sprintf(buffer, "%.3f/n", data);

    // Open the file in append mode
    FILE *file = fopen("C:/Users/misbah/log.txt", "a");
    if (file) {
        // Write data to the file
        fputs(buffer, file);

        // Close the file
        fclose(file);
    }
}

int main(void)
{

	EnableCaches();
	DHB1_begin(&pmodDHB1, GPIO_BASEADDR, PWM_BASEADDR, CLK_FREQ, MOTOR_PWM_MAX);

	// Motor initializations
	MotorFeedback_init(
			&motorFeedback,
			MOTOR_FB_BASEADDR,
			CLK_FREQ,
			SENSOR_EDGES_TARGET,
			GEARBOX_RATIO);

	DHB1_setMotorSpeeds(&pmodDHB1, 50, 50);
	MotorFeedback_clearPosCounter(&motorFeedback);
	DHB1_motorDisable(&pmodDHB1);

	ACL_begin(&acl, XPAR_PMODACL_0_AXI_LITE_GPIO_BASEADDR,
				  XPAR_PMODACL_0_AXI_LITE_SPI_BASEADDR);
	ACL_SetMeasure(&acl, 0);
	ACL_SetGRange(&acl, ACL_PAR_GRANGE_PM4G);
	ACL_SetMeasure(&acl, 1);
	ACL_CalibrateOneAxisGravitational(&acl, ACL_PAR_AXIS_ZP);
	sleep(1); // After calibration, some delay is required for the new settings
			  // to take effect.

	GYRO_begin(&myDevice, XPAR_PMODGYRO_0_AXI_LITE_SPI_BASEADDR,
				   XPAR_PMODGYRO_0_AXI_LITE_GPIO_BASEADDR);
	// Set threshold registers
	GYRO_setThsXH(&myDevice, 0x0B);
	GYRO_setThsXL(&myDevice, 0x00);
	GYRO_setThsYH(&myDevice, 0x0B);
	GYRO_setThsYL(&myDevice, 0x00);
	GYRO_setThsZH(&myDevice, 0x0B);
	GYRO_setThsZL(&myDevice, 0x00);

	uint8_t mode = GYRO_INT1_XHIE;
	//| GYRO_INT1_ZHIE | GYRO_INT1_YHIE | GYRO_INT1_ZLIE | GYRO_INT1_YLIE | GYRO_INT1_XLIE;
	GYRO_enableInt1(&myDevice, mode);

	uint8_t mode2 = GYRO_REG3_I2_DRDY;

	//| GYRO_REG3_I2_WTM | GYRO_REG3_I2_ORUN | GYRO_REG3_I2_EMPTY;

	GYRO_enableInt2(&myDevice, mode2);


    // Main loop
    while (1) {
        // Read sensor values
        readSensors();

        // Update Kalman filter
        // updateKalmanFilter(gx, az, 0.01f);

        updateComplementaryFilter(gx, az, 0.01f);

        sensorEdges = MotorFeedback_getDistanceTraveled(&motorFeedback);

        DHB1_motorEnable(&pmodDHB1);

        // Write data to the log file
        //writeToLogFile(gx);

        if (roll > 1)
		{
			DHB1_setDirs(&pmodDHB1, 1, 1);
			//sensorEdges = MotorFeedback_getDistanceTraveled(&motorFeedback);
			DHB1_setMotorSpeeds(&pmodDHB1, 40, 40);
			//driveMotor();

		}
		else if (roll < -1)
		{
			DHB1_setDirs(&pmodDHB1, 0, 0);
			//sensorEdges = MotorFeedback_getDistanceTraveled(&motorFeedback);
			DHB1_setMotorSpeeds(&pmodDHB1, 40, 40);
			//driveMotor();
		}
		else
		{
			DHB1_setMotorSpeeds(&pmodDHB1, 0, 0);
		}
        //sleep(2);

    }

    return 0;
}
