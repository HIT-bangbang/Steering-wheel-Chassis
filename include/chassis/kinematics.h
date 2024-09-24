#ifndef KINEMATICS_H
#define KINEMATICS_H
#include<cmath>

struct Wheel {
    double speed;  // 轮子的速度
    double angle;  // 轮子的角度（角度单位是度）
};

struct Cmd {
    double vx;     // 轮子的x速度
    double vy;     // 轮子的y速度
    double omega;         // 轮子的角度角速度 rad/s
};

class Kinematics
{
private:
    Wheel fl, fr, rl, rr;

    float width;
    float length;
    float radius;


public:
    Kinematics(float _width, float _length);
    void inverse(Cmd cmd);  // 逆解
    void forward(float omega_A, float omega_B); // 正解
    Wheel calculate_swerve_wheel(double vx, double vy, double omega, double L, double W, bool isFront, bool isLeft);
};

#endif