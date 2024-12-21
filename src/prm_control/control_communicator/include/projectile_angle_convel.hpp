#define MUZZLE_VELOCITY 30.00
#define SHOT_IMPOSSIBLE -100
#define RAD_2_DEG 180 / PI
#define OoM -10

#include <iostream>
#include <complex>
#include <algorithm>
#include <cmath>

static std::complex<double> complex_sqrt(const std::complex<double> & z);
static std::complex<double> complex_cbrt(const std::complex<double> & z);
void solve_quartic(const std::complex<double> coefficients[5], std::complex<double> roots[4]);

struct vec3
{
    double x = 0, y = 0, z = 0;
    std::string id = "N/A";

    vec3();
    vec3(double a, double b, double c);

    vec3 operator + (vec3 const &a);
    vec3 operator - (vec3 const &a);
    double operator * (vec3 const &a);
    vec3 operator * (double c);
    vec3 operator / (double c);

    double norm();
    void print();
};

bool compare_complex(std::complex<double> a, std::complex<double> b);
double smallest_positive_real_soln(std::complex<double> roots[4]);
void pitch_yaw_gravity_model_movingtarget_const_v(vec3 Pos, vec3 Vel, vec3 G, double time_delay, double* pitch, double* yaw, bool* impossible);