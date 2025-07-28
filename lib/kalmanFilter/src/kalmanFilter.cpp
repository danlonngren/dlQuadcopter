#include "kalmanFilter.h"

KalmanFilter::KalmanFilter() : 
    angle(0), 
    bias(0), 
    rate(0),
    Q_angle(0.001f), 
    Q_bias(0.003f), 
    R_measure(0.03f) {
    P[0][0] = 0; P[0][1] = 0;
    P[1][0] = 0; P[1][1] = 0;
}

void KalmanFilter::setParameters(float q_angle, float q_bias, float r_measure) {
    Q_angle = q_angle;
    Q_bias = q_bias;
    R_measure = r_measure;
}

float KalmanFilter::update(float measured_angle, float gyro_rate, float dt) {
    // Prediction step
    rate = gyro_rate - bias;
    angle += dt * rate;

    P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    // Correction step
    float y = measured_angle - angle;
    float S = P[0][0] + R_measure;

    float K[2];
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    angle += K[0] * y;
    bias  += K[1] * y;

    float P00_temp = P[0][0];
    float P01_temp = P[0][1];

    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;

    return angle;
}

float KalmanFilter::getAngle() const {
    return angle;
}
