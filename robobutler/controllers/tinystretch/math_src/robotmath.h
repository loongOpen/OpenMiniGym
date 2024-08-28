#ifndef __MYMATH_H__
#define __MYMATH_H__

float To_180_degreesw(float x);
float limitw(float x,float min,float max);
float deadw(float x,float zoom);
void invet22w(const float A[4], float *dA, float inA[4]);
void invet33w(const float A[9], float *dA, float inA[9]);
float sindw(float in);
float cosdw(float in);
void DigitalLPFw(float in, float* out, float cutoff_freq, float dt);
float maxw(float in1, float in2);
float minw(float in1, float in2);
#endif