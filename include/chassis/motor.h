#ifndef MOTOR_H
#define MOTOR_H

#include "socketcan.h"
#include "commen.hpp"
#define MODE_POS 1
#define MODE_VEL 3
#define TORQUE 4




class Motor
{
private:

    float radius;   // 车轮半径
    short ctrl_mode = 0;    // 控制模式
    Socketcan *socketcanPtr;    // Socket can 接口
    unsigned int nodeId;

public:
    Motor(float _radius, Socketcan *_socketcanPtr, unsigned int _nodeId);
    //~Motor();

    int StartNMT();                    // 发送NMT报文 启动该节点
    int set_mode(short _ctrl_mode);    // 设置模式
    int enable();                      // 电机使能
    int set_vel(int32_t vel, int16_t current);                   // 速度控制
    int return_to_zero();  // 绝对位置回零
    int set_pos(float angle, int32_t vel);                     // 位置控制
    int execute_posCtrl();  // 执行位置控制
    int suspend();  // 暂停
    int recover();  // 恢复
    int disable();  // 失能

};



#endif