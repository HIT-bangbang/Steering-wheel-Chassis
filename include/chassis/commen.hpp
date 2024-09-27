#ifndef COMMEN_H
#define COMMEN_H

//! 底盘机械属性

#define WIDTH 0.5   // 宽度
#define LENGTH 1.0  // 长度
#define WHEEL_RADIUS 0.1

// ! 电机can ID

// 前左
#define M_FL_STEER_ID 0xA
#define M_FL_DDRIVE_ID 0xA

// 前右
#define M_FR_STEER_ID 0xA
#define M_FR_DDRIVE_ID 0xA

// 后左
#define M_RL_STEER_ID 0xA
#define M_RL_DDRIVE_ID 0xA

// 后右
#define M_RR_STEER_ID 0xA
#define M_RR_DDRIVE_ID 0xA

//! 一圈的脉冲数
#define TICKS_PER_CYCLE 65536
#define TICKS_PER_RAD 10430.37835

// ! pid

#define KP 0.05       // PID_P
#define KI 0.1       // PID_I
#define KD 0.0       // PID_D
#define RAMP -1.0       // PID 斜率限制 0.0及负值表示无斜率限制
#define PID_LIMIT 1.0       // PID 积分限幅 和 输出限幅

//! 打印输出颜色

#define RESET   "\033[0m"
#define BLACK   "\033[30m"      /* Black */
#define RED     "\033[31m"      /* Red */
#define GREEN   "\033[32m"      /* Green */
#define YELLOW  "\033[33m"      /* Yellow */
#define BLUE    "\033[34m"      /* Blue */
#define MAGENTA "\033[35m"      /* Magenta */
#define CYAN    "\033[36m"      /* Cyan */
#define WHITE   "\033[37m"      /* White */
#define BOLDBLACK   "\033[1m\033[30m"      /* Bold Black */
#define BOLDRED     "\033[1m\033[31m"      /* Bold Red */
#define BOLDGREEN   "\033[1m\033[32m"      /* Bold Green */
#define BOLDYELLOW  "\033[1m\033[33m"      /* Bold Yellow */
#define BOLDBLUE    "\033[1m\033[34m"      /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m"      /* Bold Magenta */
#define BOLDCYAN    "\033[1m\033[36m"      /* Bold Cyan */
#define BOLDWHITE   "\033[1m\033[37m"      /* Bold White */

//! 数学工具

#define _PI 3.14159265358979

//限幅
#define _constrain(value,low,high) ((value)<(low)?(low):((value)>(high)?(high):(value)))

#endif


