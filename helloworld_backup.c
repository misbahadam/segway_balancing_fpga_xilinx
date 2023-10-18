/******************************************************************************
 *
 * Copyright (C) 2009 - 2014 Xilinx, Inc.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * Use of the Software is limited solely to applications:
 * (a) running on a Xilinx device, or
 * (b) that interact with a Xilinx device through a bus or interconnect.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * XILINX  BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
 * OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * Except as contained in this notice, the name of the Xilinx shall not be used
 * in advertising or otherwise to promote the sale, use or other dealings in
 * this Software without prior written authorization from Xilinx.
 *
 ******************************************************************************/

/*
 * helloworld.c: simple test application
 *
 * This application configures UART 16550 to baud rate 9600.
 * PS7 UART (Zynq) is not initialized by this application, since
 * bootrom/bsp configures it to baud rate 115200
 *
 * ------------------------------------------------
 * | UART TYPE   BAUD RATE                        |
 * ------------------------------------------------
 *   uartns550   9600
 *   uartlite    Configurable only in HW design
 *   ps7_uart    115200 (configured by bootrom/bsp)
 */

/************ Include Files ************/

#include "MotorFeedback.h"
#include "PmodDHB1.h"
#include <stdio.h>
#include "PmodACL.h"
#include "PmodGYRO.h"
#include "PWM.h"
#include "sleep.h"
#include "xil_cache.h"
#include "xparameters.h"
#include "xil_printf.h"
#include "math.h"
#include <stdint.h>
#include "platform.h"

/************ Macro Definitions ************/

#define GPIO_BASEADDR     XPAR_PMODDHB1_0_AXI_LITE_GPIO_BASEADDR
#define PWM_BASEADDR      XPAR_PMODDHB1_0_PWM_AXI_BASEADDR
#define MOTOR_FB_BASEADDR XPAR_PMODDHB1_0_MOTOR_FB_AXI_BASEADDR

#ifdef __MICROBLAZE__
#define CLK_FREQ XPAR_CPU_M_AXI_DP_FREQ_HZ
#else
#define CLK_FREQ 100000000 // FCLK0 frequency not found in xparameters.h
#endif

#define PWM_PER              2
#define SENSOR_EDGES_PER_REV 4
#define GEARBOX_RATIO        19
#define SENSORFS 100
#define INV_SAMPLE_RATE  (1.0f / SENSORFS)

#define twoKpDef	(2.0f * 0.5f)	// 2 * proportional gain
#define twoKiDef	(2.0f * 0.01f)	// 2 * integral gain
#define SIZE 10

// Constants
#define DT 0.01 // Time step (seconds)
#define GYRO_NOISE 0.1 // Gyroscope noise standard deviation (degrees/second)
#define ACCEL_NOISE 0.1 // Accelerometer noise standard deviation (g)

// State variables
double pitch = 0; // Pitch angle (degrees)
double pitch_dot = 0; // Pitch angular velocity (degrees/second)
double roll = 0; // Roll angle (degrees)
double roll_dot = 0; // Roll angular velocity (degrees/second)

// Covariance matrix
double P[2][2] = {{1, 0}, {0, 1}}; // Initial covariance matrix (degrees^2 and (degrees/second)^2)

// Process model
double F[2][2] = {{1, DT}, {0, 1}}; // State transition matrix
double Q[2][2] = {{GYRO_NOISE*GYRO_NOISE*DT*DT/4, GYRO_NOISE*GYRO_NOISE*DT/2},
                  {GYRO_NOISE*GYRO_NOISE*DT/2, GYRO_NOISE*GYRO_NOISE}}; // Process noise matrix

// Measurement model
double H[2] = {1, 0}; // Measurement matrix
double R[1] = {ACCEL_NOISE*ACCEL_NOISE}; // Measurement noise covariance

float filtvalX[SIZE];
float filtvalY[SIZE];
float filtvalZ[SIZE];

float *klfilt(float arrayX[SIZE],float arrayY[SIZE],float arrayZ[SIZE] );
float stdev(float *data);

// Function prototypes
void predictionStep(double gyro_pitch, double gyro_roll);
void correctionStep(double accel_pitch, double accel_roll);

//static float twoKp = twoKpDef;		// 2 * proportional gain (Kp)
//static float twoKi = twoKiDef;		// 2 * integral gain (Ki)
//static float q0 = 0.75f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; // quaternion of sensor frame relative to auxiliary frame
//static float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float invSqrt(float number) {
	const float x2 = number * 0.5F;
	const float threehalfs = 1.5F;

	union {
		float f;
		uint32_t i;
	} conv = { .f = number };
	conv.i = 0x5f3759df - (conv.i >> 1);
	conv.f *= threehalfs - (x2 * conv.f * conv.f);
	return conv.f;
}

struct array
{
    int arr[8];
};

struct array getarray();


/************ Function Prototypes ************/

void DemoInitialize();

void DemoRun();

void DemoCleanup();

void drive(int16_t sensor_edges);

void EnableCaches();

void DisableCaches();

void TestAclGyr();

/************ Global Variables ************/

PmodDHB1 pmodDHB1;
MotorFeedback motorFeedback;
PmodGYRO myDevice;
PmodACL acl;

int main() {
	//init_platform();
	DemoInitialize();

	print("Hello World\n\r");
	DemoRun();
	DemoCleanup();

	//cleanup_platform();




	return 0;

}

void DemoInitialize() {
	EnableCaches();
//	DHB1_begin(&pmodDHB1, GPIO_BASEADDR, PWM_BASEADDR, CLK_FREQ, PWM_PER);
//	MotorFeedback_init(&motorFeedback,
//	MOTOR_FB_BASEADDR,
//	CLK_FREQ,
//	SENSOR_EDGES_PER_REV,
//	GEARBOX_RATIO);
//	DHB1_motorDisable(&pmodDHB1);

	xil_printf("ACL Demo Initializing");
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

	// Set Threshold Registers
	GYRO_setThsXH(&myDevice, 0x0B);
	GYRO_setThsYH(&myDevice, 0x0B);
	GYRO_setThsZH(&myDevice, 0x0B);

	GYRO_enableInt1(&myDevice, GYRO_INT1_XHIE);    // Threshold interrupt
	GYRO_enableInt2(&myDevice, GYRO_REG3_I2_DRDY); // Data Rdy/FIFO interrupt
}

void DemoRun() {

//	DHB1_setMotorSpeeds(&pmodDHB1, 50, 50);
//
//	MotorFeedback_clearPosCounter(&motorFeedback);
//
//	DHB1_motorDisable(&pmodDHB1);  // Disable PWM before changing direction,

	float xAxis[10] = { 0 };
	float yAxis[10] = { 0 };
	float zAxis[10] = { 0 };

	float xACL = 0;
	float yACL = 0;
	float zACL = 0;




	int16_t xAvg = 0;
	int16_t yAvg = 0;
	int16_t zAvg = 0;

	int16_t xSum = 0;
	int16_t ySum = 0;
	int16_t zSum = 0;

	float x, y, z;
	float temp;

	GYRO_setThsXH(&myDevice, 0x0B);
	GYRO_setThsYH(&myDevice, 0x0B);
	GYRO_setThsZH(&myDevice, 0x0B);
	GYRO_setThsXL(&myDevice, 0x00);
	GYRO_setThsYL(&myDevice, 0x00);
	GYRO_setThsZL(&myDevice, 0x00);
	int trig = 0;
	int i = 0;

	print("Starting...\n\r");

	while (1) {
		memset(xAxis, 0, sizeof(xAxis));
		memset(yAxis, 0, sizeof(yAxis));
		memset(zAxis, 0, sizeof(zAxis));

		xAvg = 0;
		yAvg = 0;
		zAvg = 0;

		xSum = 0;
		ySum = 0;
		zSum = 0;

		//ACL_ReadAccelG(&acl, &x, &y, &z);

		if (GYRO_Int1Status(&myDevice) != 0) {
			xil_printf("\x1B[2J");
			xil_printf("\x1B[H");
			xil_printf("Threshold reached\n\r");
			trig = 1;
		}
		if (GYRO_Int2Status(&myDevice) != 0) {
			if (trig == 1) {
				trig = 0;
			} else {
				xil_printf("\x1B[2J"); // Clear screen
				xil_printf("\x1B[H");  // Reset cursor to 0,0
			}
			xil_printf("Data is ready\n\r\n\r");
			for (i = 0; i < 10; i++) {
				xAxis[i] = (~(GYRO_getX(&myDevice) - 0x01)) * -1;
				yAxis[i] = (~(GYRO_getY(&myDevice) - 0x01)) * -1;
				zAxis[i] = (~(GYRO_getZ(&myDevice) - 0x01)) * -1;
				ACL_ReadAccelG(&acl, &xACL, &yACL, &zACL);

				/*xAxis[i] = GYRO_getX(&myDevice);
				yAxis[i] = GYRO_getY(&myDevice);
				zAxis[i] = GYRO_getZ(&myDevice);*/

				temp = GYRO_getTemp(&myDevice);
				xSum += xAxis[i];
				ySum += yAxis[i];
				zSum += zAxis[i];
				usleep(100000);
			}
			xAvg = xSum / 5;
			yAvg = ySum / 5;
			zAvg = zSum / 5;
			for(i = 0; i < SIZE; i++){

			}

			// Inputs
			double gyro_pitch = 0; // Gyroscope pitch measurement (degrees/second)
			double gyro_roll = 0; // Gyroscope roll measurement (degrees/second)
			double accel_pitch = 0; // Accelerometer pitch measurement (g)
			double accel_roll = 0; // Accelerometer roll measurement (g)

			    // Main loop
			while (1) {
			        // Read sensor values
			        //scanf("%lf%lf%lf%lf", &gyro_pitch, &gyro_roll, &accel_pitch, &accel_roll);

			        // Perform prediction step
			        predictionStep(gyro_pitch, gyro_roll);

			        // Perform correction step
			        correctionStep(accel_pitch, accel_roll);

			        // Print estimates
			        printf("Pitch: %f, Roll: %f\n", pitch, roll);
			    }

			struct array x=getarray();
			//fvalueX = filtvalX;
			xil_printf("Data can be checked");

		}

	}

}

void DemoCleanup() {
	GYRO_end(&myDevice);
	DisableCaches();
}

void drive(int16_t sensor_edges) {
	DHB1_motorEnable(&pmodDHB1);
	int16_t dist = MotorFeedback_getDistanceTraveled(&motorFeedback);
	while (dist < sensor_edges) {
		dist = MotorFeedback_getDistanceTraveled(&motorFeedback);
	}
	MotorFeedback_clearPosCounter(&motorFeedback);
	DHB1_motorDisable(&pmodDHB1);
}

void EnableCaches() {
#ifdef __MICROBLAZE__
#ifdef XPAR_MICROBLAZE_USE_ICACHE
	Xil_ICacheEnable();
#endif
#ifdef XPAR_MICROBLAZE_USE_DCACHE
	Xil_DCacheEnable();
#endif
#endif
}

void DisableCaches() {
#ifdef __MICROBLAZE__
#ifdef XPAR_MICROBLAZE_USE_DCACHE
	Xil_DCacheDisable();
#endif
#ifdef XPAR_MICROBLAZE_USE_ICACHE
	Xil_ICacheDisable();
#endif
#endif
}

float *klfilt(float *sensorX, float *sensorY, float *sensorZ){
float orig_val, upd_val, prev_val;
float curr_pk,curr_xk;
float prev_pk,prev_xk;
float init_pk = 1;
float init_xk = 0;
float Kk;
float stdX = stdev(sensorX);
float stdY = stdev(sensorY);
float stdZ = stdev(sensorZ);

static float filtval[SIZE];

int i = 0;
for(i= 0; i<SIZE;i++){
    orig_val = sensorX[i];
    prev_pk = init_pk;
    prev_xk = init_xk;
    Kk = prev_pk/(prev_pk+stdX);
    upd_val = prev_xk + (Kk*(orig_val-prev_xk));

    curr_pk = (1-Kk)*prev_pk;
    curr_xk = upd_val;
    filtval[i] = upd_val;
    filtvalX[i] = upd_val;

    init_pk = curr_pk;
    init_xk = curr_pk;
}
return filtval;
}

float stdev(float *data){
    int length = sizeof(data) / sizeof(float);
    int i = 0;
    float sum = 0;
    float avg;
    for(i = 0; i < length; i++){
        sum += data[i];
    }
    avg = sum/length;
    float vsum = 0;
    for(i = 0; i < length; i++){
        vsum += (data[i]-avg)*(data[i]-avg);
    }
    float variance =vsum/length;
    float std = 1/invSqrt(variance);

    return std;
}

struct array getarray()
{
    struct array y;
    printf("Enter the elements in an array : ");
    for(int i=0;i<8;i++)
    {
        y.arr[i]=i;
    }
    return y;
}

void predictionStep(double gyro_pitch, double gyro_roll) {
    // Predict state
    pitch += DT*pitch_dot;
    pitch_dot += DT*gyro_pitch;
    roll += DT*roll_dot;
    roll_dot += DT*gyro_roll;

    // Predict covariance
    double F_T[2][2] = {{F[0][0], F[1][0]}, {F[0][1], F[1][1]}}; // Transpose of F
    double P_pred[2][2]; // Predicted covariance
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            P_pred[i][j] = 0;
            for (int k = 0; k < 2; k++) {
                P_pred[i][j] += F[i][k]*P[k][j];
            }
            for (int k = 0; k < 2; k++) {
                P_pred[i][j] += P[i][k]*F_T[k][j];
            }
            for (int k = 0; k < 2; k++) {
                P_pred[i][j] += Q[i][k]*Q[j][k];
            }
        }
    }
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            P[i][j] = P_pred[i][j];
        }
    }
}

void correctionStep(double accel_pitch, double accel_roll) {
    // Compute innovation
    double z[2] = {accel_pitch, accel_roll}; // Measurement
    double y[2]; // Innovation
    for (int i = 0; i < 2; i++) {
        y[i] = z[i] - H[i]*pitch - H[i]*roll;
    }

    // Compute Kalman gain
    double S = H[0]*P[0][0]*H[0] + H[0]*P[1][0]*H[1] + H[1]*P[0][1]*H[0] + H[1]*P[1][1]*H[1] + R[0];
    double K[2]; // Kalman gain
    for (int i = 0; i < 2; i++) {
        K[i] = (P[i][0]*H[0] + P[i][1]*H[1])/S;
    }

    // Update state
    pitch += K[0]*y[0];
    pitch_dot += K[1]*y[0];
    roll += K[0]*y[1];
    roll_dot += K[1]*y[1];

    // Update covariance
    double I_KH[2][2] = {{1 - K[0]*H[0], -K[0]*H[1]}, {-K[1]*H[0], 1 - K[1]*H[1]}};
    double P_new[2][2]; // Updated covariance
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            P_new[i][j] = 0;
            for (int k = 0; k < 2; k++) {
                P_new[i][j] += I_KH[i][k]*P[k][j];
            }
        }
    }
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            P[i][j] = P_new[i][j];
        }
    }
}
