#ifndef KALMAN_FILTER3_H_
#define KALMAN_FILTER3_H_

// State being tracked
//extern float z_;  // position
//extern float v_;  // velocity

void kalmanFilter3_Configure(float zMeasVariance, float zAccelVariance, float zAccelBiasVariance, float zInitial, float vInitial, float aBiasInitial);
void kalmanFilter3_Update(float z, float a, float dt, float* pZ, float* pV);


#endif

