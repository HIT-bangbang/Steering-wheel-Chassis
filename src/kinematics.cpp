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
 * @brief: 上位机发来的指令线速度和角速度 ----> 两个轮子的期望转速 rad/s
 * @param {float} cmd_angular 角速度 m/s
 * @return {*}
 */
void Kinematics::inverse(Cmd cmd)
{
    // 计算四个舵轮的速度和角度
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
    wheel.angle = std::atan2(y_component, x_component) * 180 / M_PI;  // 角度转换为度数
    return wheel;
}