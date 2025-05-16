#pragma once

class KalmanFilter {
public:
    KalmanFilter();

    // Set tuning parameters
    void setParameters(float q_angle, float q_bias, float r_measure);

    // Call this every loop with accel angle, gyro rate, and dt
    float update(float measured_angle, float gyro_rate, float dt);

    // Get current estimate
    float getAngle() const;

private:
    float angle; // Estimated angle
    float bias;  // Estimated gyro bias
    float rate;  // Unbiased rate

    float P[2][2]; // Error covariance matrix

    float Q_angle;   // Process noise variance for angle
    float Q_bias;    // Process noise variance for gyro bias
    float R_measure; // Measurement noise variance
};
