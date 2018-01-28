#ifndef KALMAN_FILTER2_H_
#define KALMAN_FILTER2_H_

// State being tracked
//extern float z_;  // position
//extern float v_;  // velocity

void kalmanFilter2_Configure(float zVariance, float zAccelVariance,  float zInitial, float vInitial);
void kalmanFilter2_Update(float z, float zAccelVariance,float dt, float* pZ, float* pV);


#endif

