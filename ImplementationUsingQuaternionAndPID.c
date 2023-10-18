#include <math.h>

// Constants
#define PI 3.14159265358979323846

// State variables
float theta;      // Orientation (pitch angle)
float theta_dot;  // Angular velocity

// Sensor measurements
float accel_x;    // Accelerometer value along the x-axis
float gyro_z;     // Gyroscope value around the z-axis

// Kalman filter variables
float Q_angle = 0.001;   // Process noise covariance for angle
float Q_gyro = 0.003;    // Process noise covariance for gyro bias
float R_accel = 0.03;    // Measurement noise covariance for accelerometer
float dt = 0.01;         // Time step

float P[2][2] = {{1, 0}, {0, 1}};  // Covariance matrix
float K[2];                        // Kalman gain
float y;                           // Measurement residual
float S;                           // Innovation covariance

void updateState(float accel, float gyro) {
  // Update orientation using the gyroscope
  theta += (gyro - theta_dot) * dt;

  // Update angular velocity
  theta_dot = gyro;

  // Measurement update step
  y = accel - theta;
  S = P[0][0] + R_accel;
  K[0] = P[0][0] / S;
  K[1] = P[1][0] / S;

  // State estimation update
  theta += K[0] * y;
  theta_dot += K[1] * y;

  // Covariance matrix update
  float P00_temp = P[0][0];
  float P01_temp = P[0][1];

  P[0][0] -= K[0] * P00_temp;
  P[0][1] -= K[0] * P01_temp;
  P[1][0] -= K[1] * P00_temp;
  P[1][1] -= K[1] * P01_temp;
}

void calculateQuaternion(float theta, float* quaternion) {
  float halfTheta = theta / 2.0;
  quaternion[0] = cos(halfTheta);        // Scalar component (q0)
  quaternion[1] = 0.0;                   // x-component (q1)
  quaternion[2] = sin(halfTheta);        // y-component (q2)
  quaternion[3] = 0.0;                   // z-component (q3)
}

float calculateBalancingAngle(float* quaternion) {
  // Define reference quaternion for upright orientation (no tilt)
  float q_ref[4] = {1.0, 0.0, 0.0, 0.0};

  // Calculate error quaternion
  float q_err[4];
  q_err[0] = quaternion[0] * q_ref[0] + quaternion[1] * q_ref[1] +
              quaternion[2] * q_ref[2] + quaternion[3] * q_ref[3];
  q_err[1] = quaternion[0] * q_ref[1] - quaternion[1] * q_ref[0] -
              quaternion[2] * q_ref[3] + quaternion[3] * q_ref[2];
  q_err[2] = quaternion[0] * q_ref[2] + quaternion[1] * q_ref[3] -
              quaternion[2] * q_ref[0] - quaternion[3] * q_ref[1];
  q_err[3] = quaternion[0] * q_ref[3] - quaternion[1] * q_ref[2] +
              quaternion[2] * q_ref[1] - quaternion[3] * q_ref[0];

  // Calculate rotation angle from error quaternion
  float rotationAngle = 2.0 * acos(q_err[0]);

  return rotationAngle;
}

int main() {
  // Initialize state variables
  theta = 0.0;
  theta_dot = 0.0;

  // Simulated sensor data
  accel_x = 0.2;   // Replace with actual accelerometer values
  gyro_z = 0.03;   // Replace with actual gyroscope values

  // Main loop
  while (1) {
    // Call the update function with sensor measurements
    updateState(accel_x, gyro_z);

    // Calculate the quaternion based on the estimated orientation
    float quaternion[4];
    calculateQuaternion(theta, quaternion);

    // Calculate the balancing angle from the quaternion
    float balancingAngle = calculateBalancingAngle(quaternion);

    // Use the balancing angle for motor control or other actions

    // Simulate a delay for the next iteration
    // Replace with appropriate delay function for your platform
    for (int i = 0; i < 10000; i++) {
      // Delay loop
    }
  }

  return 0;
}

