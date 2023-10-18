#include <stdio.h>
#include <stdint.h>
#include "xil_printf.h"
#include "xparameters.h"
#include "xgpio.h"
#include "xadcps.h"
#include "MotorFeedback.h"
#include "PmodACL.h"
#include "PmodGYRO.h"
#include "PmodDHB1.h"

// Define constants for sensor and motor control
#define GYRO_SENSITIVITY 0.1f
#define ACCEL_SENSITIVITY 0.01f
#define MOTOR_PWM_MAX 1000
#define SENSOR_EDGES_TARGET 5  // Adjust this value as needed

#define CLK_FREQ XPAR_PS7_UART_1_UART_CLK_FREQ_HZ // FCLK0 frequency not found in xparameters.h
#define GPIO_BASEADDR XPAR_PMODDHB1_0_AXI_LITE_GPIO_BASEADDR
#define PWM_BASEADDR XPAR_PMODDHB1_0_PWM_AXI_BASEADDR
#define MOTOR_FB_BASEADDR XPAR_PMODDHB1_0_MOTOR_FB_AXI_BASEADDR
#define GEARBOX_RATIO 19

// Initialize GPIO and ADC instances
XGpio motorCtrl;
XAdcPs adc; //still not sure if I have to use it or not

/************ Global Variables ************/

PmodDHB1 pmodDHB1;
MotorFeedback motorFeedback;
PmodGYRO myDevice;
PmodACL acl; //initialized all the 4 classes to access their methods

// Kalmann filter variables
float roll = 0.0f;      // Roll angle estimate and still have to find a way to calculate it
float rollRate = 0.0f;  // Roll rate estimate
float gyroBias = 0.0f;  // Gyro bias estimate

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

// Function to read gyroscope and accelerometer values
void readSensors(float* gyro, float* accel) {
    // Read gyroscope value (angular velocity in degrees/sec)
    //*gyro = (float)XAdcPs_GetAdcData(&adc, XADCPS_SEQ_CH_AUX12) * GYRO_SENSITIVITY;

    // Read accelerometer value (acceleration in m/s^2)
    //*accel = (float)XAdcPs_GetAdcData(&adc, XADCPS_SEQ_CH_AUX14) * ACCEL_SENSITIVITY;
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

// Function to update the Kalman filter
void updateKalmanFilter(float gyro, float accel, float dt) {
    // Apply FIR filter to sensor readings
    float filteredAccel = applyFIRFilter(accel);
    float filteredGyro = applyFIRFilter(gyro);

    // Prediction step
    float A[2][2] = {{1.0f, -dt}, {0.0f, 1.0f}};  // System dynamics matrix
    float B[2][1] = {{dt}, {0.0f}};  // Input matrix
    float U[1] = {filteredGyro - gyroBias};  // Input (gyro measurement minus bias)
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
    float innovation = filteredAccel - rollPred;  // Innovation (measurement residual)
    roll = rollPred + K[0] * innovation;
    rollRate = rollRatePred;
    P[0][0] = (1.0f - K[0] * H[0][0]) * P_pred[0][0];
}

int main() {

	EnableCaches();
	DHB1_begin(&pmodDHB1, GPIO_BASEADDR, PWM_BASEADDR, CLK_FREQ, MOTOR_PWM_MAX);

	// Motor initializations
	MotorFeedback_init(
			&motorFeedback,
			MOTOR_FB_BASEADDR,
			CLK_FREQ,
			SENSOR_EDGES_TARGET,
			GEARBOX_RATIO);
	DHB1_motorDisable(&pmodDHB1);

    // Main loop
    while (1) {
        // Read sensor values
        float gyro, accel;
        readSensors(&gyro, &accel);

        // Update Kalman filter
        updateKalmanFilter(gyro, accel, 0.01f);

        // Control the motor based on sensor readings
        driveMotor();
    }

    return 0;
}

// Function to drive the motor based on sensor readings
void driveMotor() {

    // Drive the motor until the target number of sensor edges is reached
    while (sensorEdges < SENSOR_EDGES_TARGET) {
        // Read sensor values
        float gyro, accel;
        readSensors(&gyro, &accel);

        // Update Kalman filter
        updateKalmanFilter(gyro, accel, 0.01f);

        //use the calculated state for further processing like PID controller and quaternion
        //to find the drive angle or can also say control input which is then fed into the motor

        // Calculate motor PWM based on roll angle
        motorPwm = (int)((roll / 90.0f) * MOTOR_PWM_MAX);

        // Set motor PWM
        // (Assuming the motor control GPIO pins are connected to appropriate outputs)
        XGpio_DiscreteWrite(&motorCtrl, 1, motorPwm);

        // Update sensor edges based on motor feedback
        sensorEdges = MotorFeedback_getDistanceTraveled(&motorFeedback);

    }

    // Stop the motor
    XGpio_DiscreteWrite(&motorCtrl, 1, 0);
}
