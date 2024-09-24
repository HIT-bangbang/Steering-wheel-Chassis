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


// ! pid

#define KP 0.05       // PID_P
#define KI 0.1       // PID_I
#define KD 0.0       // PID_D
#define RAMP -1.0       // PID 斜率限制 0.0及负值表示无斜率限制
#define PID_LIMIT 1.0       // PID 积分限幅 和 输出限幅

//限幅
#define _constrain(value,low,high) ((value)<(low)?(low):((value)>(high)?(high):(value)))

#endif


