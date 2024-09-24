#include"motor.h"

Motor::Motor(float _radius, Socketcan *_socketcanPtr, unsigned int _nodeId)
{
    radius = _radius;
    socketcanPtr = _socketcanPtr;
    nodeId = _nodeId;
}

int Motor::StartNMT()                    // 发送NMT报文 启动该节点
{
    struct can_frame NMT_frame;
	NMT_frame.can_id  = 0x000;
	NMT_frame.can_dlc = 2;
	NMT_frame.data[0] = 0x01;
	NMT_frame.data[1] = nodeId;

    socketcanPtr->can_write(NMT_frame);     // 发送
}

int Motor::set_mode(short ctrl_mode)    // 设置模式
{
    struct can_frame mode_fame;
    mode_fame.can_id  = 0x200 + nodeId;
    mode_fame.can_dlc = 1;
    
    if (ctrl_mode == MODE_POS)  // 位置模式
    {
        std::cout<<"change to position ctrl mode" << std::endl;
        mode_fame.data[0] = 0x01;
        socketcanPtr->can_write(mode_fame);

    }
    else if (ctrl_mode == MODE_VEL) // 速度模式
    {
        std::cout<<"change to position ctrl mode" << std::endl;
        mode_fame.data[0] = 0x03;
        socketcanPtr->can_write(mode_fame);
    }
    else        // 错误
    {
        std::cout<<"ERROR:NOT suppoted ctrl mode" << std::endl;
    }
}

// 使能电机
int Motor::enable()
{
    struct can_frame state_fame;
    state_fame.can_id  = 0x200 + nodeId;
	state_fame.can_dlc = 2;
	state_fame.data[0] = 0x06;
	state_fame.data[1] = 0x00;
    std::cout << "change state to ready to switch on" << std::endl;
    socketcanPtr->can_write(state_fame); // 切换至 ready to switch on 状态

	state_fame.data[0] = 0x07;
	state_fame.data[1] = 0x00;
    std::cout << "change state to switched on" << std::endl;
    socketcanPtr->can_write(state_fame); // 切换至 switched on 状态

	state_fame.data[0] = 0x0F;
	state_fame.data[1] = 0x00;
    std::cout << "change state to operation enable" << std::endl;
    socketcanPtr->can_write(state_fame); // 切换至 operation enable 状态，使能伺服电机
}

int Motor::set_speed()
{
    if (ctrl_mode != MODE_VEL)
    {
        std::cout << "ctrl mode not match" << std::endl;
        return -1;
    }
    struct can_frame state_fame;
    state_fame.can_id  = 0x200 + nodeId;
	state_fame.can_dlc = 2;
	state_fame.data[0] = 0x06;
	state_fame.data[1] = 0x00;
    std::cout << "change state to ready to switch on" << std::endl;
    socketcanPtr->can_write(state_fame); // 切换至 ready to switch on 状态

}

int Motor::set_pos()
{
    if (ctrl_mode != MODE_POS)
    {
        std::cout << "ctrl mode not match" << std::endl;
        return -1;
    }


}