#include "kinematics.h"
#include "commen.hpp"

Kinematics::Kinematics(float _width, float _length)
{
    width = _width;
    length = _length;
}

/**
 * @brief: 运动学正解： 当前实际轮子转速 ----- 地盘实际的线速度角速度
 * @param {float} omega_A   车轮A的转速，单位rad/s
 * @param {float} omega_B
 * @return {*}
 */
void Kinematics::forward(float omega_A, float omega_B)
{

}

/**
 * @brief: 上位机发来的指令线速度和角速度 ----> 两个轮子的期望线速度 m/s
 * @param {float} cmd_angular 角速度 m/s
 * @return {*}
 */
void Kinematics::inverse(Cmd cmd)
{
    // 计算四个舵轮的线速度和角度
    fl = calculate_swerve_wheel(cmd.vx, cmd.vy, cmd.omega, length, width, true, true);   // 前左轮
    fr = calculate_swerve_wheel(cmd.vx, cmd.vy, cmd.omega, length, width, true, false);  // 前右轮
    rl = calculate_swerve_wheel(cmd.vx, cmd.vy, cmd.omega, length, width, false, true);  // 后左轮
    rr = calculate_swerve_wheel(cmd.vx, cmd.vy, cmd.omega, length, width, false, false); // 后右轮
}

// 计算每个舵轮的速度和角度
Wheel Kinematics::calculate_swerve_wheel(double vx, double vy, double omega, double L, double W, bool isFront, bool isLeft) {
    double x_component = isLeft ? (vx - omega * L / 2) : (vx + omega * L / 2);
    double y_component = isFront ? (vy + omega * W / 2) : (vy - omega * W / 2);

    Wheel wheel;
    wheel.speed = std::sqrt(x_component * x_component + y_component * y_component);
    wheel.angle = std::atan2(y_component, x_component);
    return adjust_wheel_angle(wheel);
}

// 将角度保持在 -90 到 90 度范围（第一四象限）内，并处理轮速方向
Wheel Kinematics::adjust_wheel_angle(Wheel wheel) {

    // 厂家轮子和我的坐标系是反的
    wheel.angle = -wheel.angle;

// 如果在第三四象限了，就同时对称角度并反转速度
if (wheel.angle > _PI/2 )
{
    wheel.angle -= _PI;
    wheel.speed = -wheel.speed;
}
else if (wheel.angle < -_PI/2 )
{
    wheel.angle += _PI;
    wheel.speed = -wheel.speed;
}


// 如果是-90度，就变为+90度，把速度反向
if (fabs(wheel.angle + _PI/2) < 0.001 )
{
    wheel.angle = -wheel.angle;
    wheel.speed = -wheel.speed;
}

    return wheel;
}