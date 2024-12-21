#include <stdio.h>
#include <stdint.h>
#include <math.h>

#define G -9.80665 / 1000 // (mm/ms^2)

uint8_t crc8(uint8_t *data, size_t len);
int8_t float2int8(double f);
float quadratic(float a, float b, float c);
int gravity_pitch_offset(float v0, int min_p, int max_p, float x, float dz, float xv);