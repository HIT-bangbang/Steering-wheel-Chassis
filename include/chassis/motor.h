#ifndef MOTOR_H
#define MOTOR_H

#include "socketcan.h"

#define MODE_POS 1
#define MODE_VEL 3
#define TORQUE 4




class Motor
{
private:

    float radius;   // 车轮半径
    short ctrl_mode;
    Socketcan *socketcanPtr;    // Socket can 接口
    unsigned int nodeId;

public:
    Motor(float _radius, Socketcan *_socketcanPtr, unsigned int _nodeId);
    ~Motor();

    int StartNMT();                    // 发送NMT报文 启动该节点
    int set_mode(short ctrl_mode);    // 设置模式
    int enable();                      // 电机使能
    int set_speed();                   // 速度控制
    int set_pos();                     // 位置控制
};



#endif