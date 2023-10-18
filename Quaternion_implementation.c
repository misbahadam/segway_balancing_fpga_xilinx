#include <stdio.h>
#include <math.h>

#define ACCELEROMETER_SENSITIVITY 8192.0
#define GYROSCOPE_SENSITIVITY 65.536
#define M_PI 3.14159265358979323846

// Function to convert raw accelerometer readings to acceleration values
void convertAccelerometerRaw(int16_t *rawData, float *acceleration)
{
    for (int i = 0; i < 3; i++)
        acceleration[i] = rawData[i] / ACCELEROMETER_SENSITIVITY;
}

// Function to convert raw gyroscope readings to angular velocity values
void convertGyroscopeRaw(int16_t *rawData, float *angularVelocity)
{
    for (int i = 0; i < 3; i++)
        angularVelocity[i] = rawData[i] / GYROSCOPE_SENSITIVITY;
}

// Function to calculate quaternion from gyroscope readings
void calculateQuaternion(float *angularVelocity, float *quaternion, float dt)
{
    float norm;
    float gx = angularVelocity[0], gy = angularVelocity[1], gz = angularVelocity[2];

    // Rate of change of quaternion
    float qDot[4] = {0};

    // Quaternion derivative from gyroscope
    qDot[0] = 0.5 * (-quaternion[1] * gx - quaternion[2] * gy - quaternion[3] * gz);
    qDot[1] = 0.5 * (quaternion[0] * gx + quaternion[2] * gz - quaternion[3] * gy);
    qDot[2] = 0.5 * (quaternion[0] * gy - quaternion[1] * gz + quaternion[3] * gx);
    qDot[3] = 0.5 * (quaternion[0] * gz + quaternion[1] * gy - quaternion[2] * gx);

    // Integrate to yield quaternion
    quaternion[0] += qDot[0] * dt;
    quaternion[1] += qDot[1] * dt;
    quaternion[2] += qDot[2] * dt;
    quaternion[3] += qDot[3] * dt;

    // Normalize quaternion
    norm = sqrt(quaternion[0] * quaternion[0] + quaternion[1] * quaternion[1] +
                quaternion[2] * quaternion[2] + quaternion[3] * quaternion[3]);

    quaternion[0] /= norm;
    quaternion[1] /= norm;
    quaternion[2] /= norm;
    quaternion[3] /= norm;
}

int main()
{
    int16_t accelerometerRaw[3];
    int16_t gyroscopeRaw[3];
    float accelerometerData[3];
    float gyroscopeData[3];
    float quaternion[4] = {1.0, 0.0, 0.0, 0.0}; // Initial quaternion with no rotation
    float dt = 0.01;                            // Time step in seconds

    // Read accelerometer and gyroscope data from sensors
    // (Replace with your own code to read sensor data)

    // Conversion to acceleration values
    convertAccelerometerRaw(accelerometerRaw, accelerometerData);

    // Conversion to angular velocity values
    convertGyroscopeRaw(gyroscopeRaw, gyroscopeData);

    // Quaternion calculation
    calculateQuaternion(gyroscopeData, quaternion, dt);

    // Print quaternion values
    printf("Quaternion: %.4f, %.4f, %.4f, %.4f\n",
           quaternion[0], quaternion[1], quaternion[2], quaternion[3]);

    return 0;
}
