#include "utils.hpp"

uint8_t crc8(uint8_t *data, size_t len)
{
    uint8_t crc = 0xff;
    size_t i, j;
    for (i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (j = 0; j < 8; j++)
        {
            if ((crc & 0x80) != 0)
                crc = (uint8_t)((crc << 1) ^ 0x31);
            else
                crc <<= 1;
        }
    }
    return crc;
}

int8_t float2int8(double f)
{
    return (int8_t)(std::fmax(std::fmin(127, f), -127));
}

float quadratic(float a, float b, float c)
{
    // negative side only
    return (-b - sqrt(pow(b, 2) - 4 * a * c)) / (2 * a);
}

int gravity_pitch_offset(float v0, int min_p, int max_p, float x, float dz, float xv)
{
    double prev_dst = 10000000000;
    int p;
    for (p = min_p; p <= max_p; p++)
    {
        double v0x = v0 * cos(p * M_PI / 180);  // Initial X vel
        double v0y = v0 * sin(p * M_PI / 180);  // Initial Z vel
        double t = quadratic(G / 2.f, v0y, dz); // Time to target z position
        double xt = x + xv * t;                 // Target position when bullet is at same z hight
        double dst = abs(xt - t * v0x);         // Difference between bullet x position and target x position
        if (dst > prev_dst)
        {
            p--;
            break;
        }
        prev_dst = dst;
    }
    return p;
}
