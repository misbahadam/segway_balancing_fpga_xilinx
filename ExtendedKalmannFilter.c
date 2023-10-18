#include <stdio.h>
#include <math.h>

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

// Function prototypes
void predictionStep(double gyro_pitch, double gyro_roll);
void correctionStep(double accel_pitch, double accel_roll);

int main(void) {
    // Inputs
    double gyro_pitch = 0; // Gyroscope pitch measurement (degrees/second)
    double gyro_roll = 0; // Gyroscope roll measurement (degrees/second)
    double accel_pitch = 0; // Accelerometer pitch measurement (g)
    double accel_roll = 0; // Accelerometer roll measurement (g)

    // Main loop
    while (1) {
        // Read sensor values
        scanf("%lf%lf%lf%lf", &gyro_pitch, &gyro_roll, &accel_pitch, &accel_roll);

        // Perform prediction step
        predictionStep(gyro_pitch, gyro_roll);

        // Perform correction step
        correctionStep(accel_pitch, accel_roll);

        // Print estimates
        printf("Pitch: %f, Roll: %f\n", pitch, roll);
    }

    return 0;
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


