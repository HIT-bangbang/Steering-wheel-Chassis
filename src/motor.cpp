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

int Motor::set_mode(short _ctrl_mode)    // 设置模式
{
    ctrl_mode = _ctrl_mode;
    
    struct can_frame mode_fame;
    mode_fame.can_id  = 0x300 + nodeId;
    mode_fame.can_dlc = 1;
    
    if (ctrl_mode == MODE_POS)  // 位置模式
    {
        std::cout<< BLUE <<"change to position ctrl mode" << std::endl;
        mode_fame.data[0] = 0x01;
        socketcanPtr->can_write(mode_fame);

    }
    else if (ctrl_mode == MODE_VEL) // 速度模式
    {
        std::cout<< BLUE <<"change to position ctrl mode" << std::endl;
        mode_fame.data[0] = 0x03;
        socketcanPtr->can_write(mode_fame);
    }
    else        // 错误
    {
        std::cout<< RED <<"ERROR:NOT suppoted ctrl mode" << std::endl;
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

// 数据为6个byte长度，前4 byte 为速度*10 INT32类型  后2个byte为电流，0表示无限制？保持为0即可
int Motor::set_vel(int32_t vel, int16_t current = 0)
{
    if (ctrl_mode != MODE_VEL)
    {
        std::cout << "ctrl mode not match" << std::endl;
        return -1;
    }
    int32_t rpm = vel*10;
    struct can_frame vel_fame;
    vel_fame.can_id  = 0x400 + nodeId;
	vel_fame.can_dlc = 6;
	std::memset(vel_fame.data, 0, sizeof(vel_fame.data));  // 全部赋值为0
	std::memcpy(vel_fame.data, &rpm, sizeof(rpm));     // 直接使用 memcpy() 可以同时解决字节序的问题
    std::cout << "set velocity to " << vel << " rad/s" << std::endl;
    socketcanPtr->can_write(vel_fame);      // 切换至 ready to switch on 状态
}

// data为8个byte。高4byte为位置指令。低4byte为（速度*10）。 每65536个脉冲为一圈
int Motor::set_pos(float angle, int32_t vel)
{
    if (ctrl_mode != MODE_POS)
    {
        std::cout << "ctrl mode not match" << std::endl;
        return -1;
    }

    int32_t ticks = angle * TICKS_PER_RAD; //换算成脉冲数量
    int32_t rpm = vel*10;   // 速度值需要乘以10

    struct can_frame pos_fame;
    pos_fame.can_id  = 0x500 + nodeId;
	pos_fame.can_dlc = 8;
	std::memset(pos_fame.data, 0, sizeof(pos_fame.data));
    std::memcpy(pos_fame.data, &ticks, sizeof(ticks));     // 设置转动的脉冲数
    std::memcpy(pos_fame.data + 4, &rpm, sizeof(rpm));     // 设置旋转的速度
    std::cout << "set target angle to " << angle << " rad" << std::endl;
    socketcanPtr->can_write(pos_fame);      // 发送目标位置和速度，注意此时并未执行

    execute_posCtrl();  // 执行位置控制指令
}

int Motor::execute_posCtrl()
{
    struct can_frame ctrl_fame;
    ctrl_fame.can_id  = 0x200 + nodeId;
	ctrl_fame.can_dlc = 2;

	std::memset(ctrl_fame.data, 0, sizeof(ctrl_fame.data));
    ctrl_fame.data[0] = 0x3F;
    ctrl_fame.data[1] = 0x00;    //200+(ID) 3F 00
    
    std::cout<< RED << "motor ID: " << nodeId << " execute_posCtrl" << std::endl;
    socketcanPtr->can_write(ctrl_fame);      // 发送执行绝对位置控制指令
}

int Motor::suspend()
{
    struct can_frame state_fame;
    state_fame.can_id  = 0x200 + nodeId;
    state_fame.can_dlc = 2;
    state_fame.data[0] = 0x03;
    state_fame.data[1] = 0x00;
    std::cout<< YELLOW << "motor ID: " << nodeId << " suspend" << std::endl;
    socketcanPtr->can_write(state_fame); // 暂停
}

int Motor::recover()
{
    struct can_frame state_fame;
    state_fame.can_id  = 0x200 + nodeId;
    state_fame.can_dlc = 2;
    state_fame.data[0] = 0x0F;
    state_fame.data[1] = 0x00;
    std::cout<< GREEN << "motor ID: " << nodeId << " recover" << std::endl;
    socketcanPtr->can_write(state_fame); // 恢复
}

int Motor::disable()
{
    struct can_frame state_fame;
    state_fame.can_id  = 0x200 + nodeId;
    state_fame.can_dlc = 2;
    state_fame.data[0] = 0x0F;
    state_fame.data[1] = 0x00;
    std::cout<< RED << "motor ID" << nodeId << "disable" << std::endl;
    socketcanPtr->can_write(state_fame); // 失能
}

int Motor::return_to_zero()
{
    struct can_frame state_fame;
    state_fame.can_id  = 0x200 + nodeId;
	state_fame.can_dlc = 2;
	state_fame.data[0] = 0x0F;
	state_fame.data[1] = 0x80;
    std::cout<< RED << "return to zero" << std::endl;
    socketcanPtr->can_write(state_fame); // 切换至 ready to switch on 状态
}