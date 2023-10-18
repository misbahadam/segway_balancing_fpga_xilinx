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

/******************************************************************************/
/*                                                                            */
/* main.c -- Example program using the PmodDHB1 IP                            */
/*                                                                            */
/******************************************************************************/
/* Author: Arvin Tang                                                         */
/*                                                                            */
/******************************************************************************/
/* File Description:                                                          */
/*                                                                            */
/* This demo drives 2 motors in the 4 possible directions. When mounted on a  */
/* 2-wheel chassis, the motors will be driven such that the robot goes        */
/* forward, goes backward, turns left, and turns right.                       */
/*                                                                            */
/******************************************************************************/
/* Revision History:                                                          */
/*                                                                            */
/*    09/14/2017(atangzwj): Created                                           */
/*    02/03/2018(atangzwj): Validated for Vivado 2017.4                       */
/*                                                                            */
/******************************************************************************/

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

static float twoKp = twoKpDef;		// 2 * proportional gain (Kp)
static float twoKi = twoKiDef;		// 2 * integral gain (Ki)
static float q0 = 0.75f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; // quaternion of sensor frame relative to auxiliary frame
static float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;
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

/************ Function Prototypes ************/

void DemoInitialize();

void DemoRun();

void DemoCleanup();

void drive(int16_t sensor_edges);

void EnableCaches();

void DisableCaches();

/************ Global Variables ************/

PmodDHB1 pmodDHB1;
MotorFeedback motorFeedback;
PmodGYRO myDevice;
PmodACL acl;

/************ Function Definitions ************/

int main(void) {
	DemoInitialize();
	DemoRun();
	DemoCleanup();
	return 0;
}

void DemoInitialize() {
	EnableCaches();
	DHB1_begin(&pmodDHB1, GPIO_BASEADDR, PWM_BASEADDR, CLK_FREQ, PWM_PER);
	MotorFeedback_init(&motorFeedback,
	MOTOR_FB_BASEADDR,
	CLK_FREQ,
	SENSOR_EDGES_PER_REV,
	GEARBOX_RATIO);
	DHB1_motorDisable(&pmodDHB1);

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
	GYRO_setThsXH(&myDevice, 0x0F);
	GYRO_setThsYH(&myDevice, 0x0F);
	GYRO_setThsZH(&myDevice, 0x0F);

	GYRO_enableInt1(&myDevice, GYRO_INT1_XHIE);    // Threshold interrupt
	GYRO_enableInt2(&myDevice, GYRO_REG3_I2_DRDY); // Data Rdy/FIFO interrupt
}

void DemoRun() {

	DHB1_setMotorSpeeds(&pmodDHB1, 50, 50);

	MotorFeedback_clearPosCounter(&motorFeedback);

	DHB1_motorDisable(&pmodDHB1);  // Disable PWM before changing direction,

	int16_t xAxis = 0;
	int16_t yAxis = 0;
	int16_t zAxis = 0;
	int8_t temp = 0;

	float angleY = 0;
	float angleX = 0;
	float tt = 0;
	float ww = 0;

	int16_t gx, gy, gz;

	float x, y, z;
	float ax, ay, az;
	float gx1, gy1, gz1;
	float gx2, gy2, gz2;

	float recipNorm;
	float

	halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;
	float sinp;

	GYRO_setThsXH(&myDevice, 0x0F);
	GYRO_setThsYH(&myDevice, 0x0F);
	GYRO_setThsZH(&myDevice, 0x0F);
	int trig = 0;

	print("Starting...\n\r");

	while (1) {
		ACL_ReadAccelG(&acl, &x, &y, &z);
		ax = (x - 0.080) * 9.811;   //-0.420 ???  -0.15 -0.015
		ay = (y + 0.018) * 9.811;   //+0.010  +0.02 bis 0.03
		az = (z - 0.953) * 9.811;   //-1.080

		usleep(10000);

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

			xAxis = GYRO_getX(&myDevice);
			yAxis = GYRO_getY(&myDevice);
			zAxis = GYRO_getZ(&myDevice);
			temp = GYRO_getTemp(&myDevice);

			gx = (xAxis - 200) * 250 / 32768;
			gy = yAxis * 250 / 32768;
			gz = zAxis * 250 / 32768;

			gx1 = (float) gx * 0.0174533f;
			;
			gy1 = (float) gy * 0.0174533f;
			;
			gz1 = (float) gz * 0.0174533f;
			;

			if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

				// Normalise accelerometer measurement
				recipNorm = invSqrt(ax * ax + ay * ay + az * az);
				ax *= recipNorm;
				ay *= recipNorm;
				az *= recipNorm;

				// Estimated direction of gravity and vector perpendicular to magnetic flux
				halfvx = q1 * q3 - q0 * q2;
				halfvy = q0 * q1 + q2 * q3;
				halfvz = q0 * q0 - 0.5f + q3 * q3;

				// Error is sum of cross product between estimated and measured direction of gravity
				halfex = (ay * halfvz - az * halfvy);
				halfey = (az * halfvx - ax * halfvz);
				halfez = (ax * halfvy - ay * halfvx);

				// Compute and apply integral feedback if enabled
				if (twoKi > 0.0f) {
					// integral error scaled by Ki
					integralFBx += twoKi * halfex * INV_SAMPLE_RATE;
					integralFBy += twoKi * halfey * INV_SAMPLE_RATE;
					integralFBz += twoKi * halfez * INV_SAMPLE_RATE;
					gx1 += integralFBx;	// apply integral feedback
					gy1 += integralFBy;
					gz1 += integralFBz;
				} else {
					integralFBx = 0.0f;	// prevent integral windup
					integralFBy = 0.0f;
					integralFBz = 0.0f;
				}

				// Apply proportional feedback
				gx1 += twoKp * halfex;
				gy1 += twoKp * halfey;
				gz1 += twoKp * halfez;
			}

			// Integrate rate of change of quaternion
			gx1 *= (0.5f * INV_SAMPLE_RATE);	// pre-multiply common factors
			gy1 *= (0.5f * INV_SAMPLE_RATE);
			gz1 *= (0.5f * INV_SAMPLE_RATE);
			qa = q0;
			qb = q1;
			qc = q2;
			q0 += (-qb * gx1 - qc * gy1 - q3 * gz1);
			q1 += (qa * gx1 + qc * gz1 - q3 * gy1);
			q2 += (qa * gy1 - qb * gz1 + q3 * gx1);
			q3 += (qa * gz1 + qb * gy1 - qc * gx1);

			// Normalise quaternion
			recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
			q0 *= recipNorm;
			q1 *= recipNorm;
			q2 *= recipNorm;
			q3 *= recipNorm;

			float sinp = 2.0 * (q0 * q2 - q3 * q1);
			float sss = sqrt(1 - sinp * sinp);
			//float angleYoffset=1.1f;
			angleY = 57.29578f * atan2(sinp, sss) + 1.1f;

			float _Kp = 1.0, _Ki = 0.0, _Kd = 0.0;
			// Calculate error
			float setpoint = 0.0f, _integral = 0.0f, _pre_error = 0.00001f;
			float error = setpoint - angleY;

			// Proportional term
			float Pout = _Kp * error;

			// Integral term
			_integral += error * 0.01;
			float Iout = _Ki * _integral;

			// Derivative term
			float derivative = (error - _pre_error) / 0.01;
			float Dout = _Kd * derivative;

			// Calculate total output
			float out = Pout + Iout + Dout;

			int driveangle = fabs(out);

			printf("%f\n", angleY);

			if (angleY > 1) {
				DHB1_setMotorSpeeds(&pmodDHB1, 50, 50);
				DHB1_setDirs(&pmodDHB1, 1, 1);
				drive(1);

			} else if (angleY < -1) {
				DHB1_setMotorSpeeds(&pmodDHB1, 50, 50);
				DHB1_setDirs(&pmodDHB1, 0, 0);
				drive(1);
			} else {
				DHB1_setMotorSpeeds(&pmodDHB1, 0, 0);

			}

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
