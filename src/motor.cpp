#include"motor.h"

Motor::Motor(float _radius, Socketcan *_socketcanPtr, unsigned int _nodeId)
{
    radius = _radius;
    socketcanPtr = _socketcanPtr;
    nodeId = _nodeId;
}

/**
 * @brief: 发送NMT报文 启动该节点
 * @return {*}
 */
int Motor::StartNMT()
{
    struct can_frame NMT_frame;
	NMT_frame.can_id  = 0x000;
	NMT_frame.can_dlc = 2;
	NMT_frame.data[0] = 0x01;
	NMT_frame.data[1] = nodeId;

    socketcanPtr->can_write(NMT_frame);     // 发送
}

/**
 * @brief: 设置模式
 * @param {short} _ctrl_mode 控制模式
 * @return {*}
 */
int Motor::set_mode(short _ctrl_mode)
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
        std::cout<< BLUE <<"change to vel ctrl mode" << std::endl;
        mode_fame.data[0] = 0x03;
        socketcanPtr->can_write(mode_fame);
    }
    else        // 错误
    {
        std::cout<< RED <<"ERROR:NOT suppoted ctrl mode" << std::endl;
    }
}


/**
 * @brief: 使能
 * @return {*}
 */
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

/**
 * @brief: 速度控制
 * @param {int32_t} vel 转速 rpm
 * @param {int16_t} current 限制电流，设置为0表示无限制
 * @return {*}
 */
int Motor::set_vel(int32_t vel, int16_t current = 0)
{
    if (ctrl_mode != MODE_VEL)
    {
        std::cout << "ctrl mode: mismatch/NOT set" << std::endl;
        return -1;
    }
    // 数据为6个byte长度，前4 byte 为速度*10 INT32类型  后2个byte为电流，0表示无限制？保持为0即可
    int32_t rpm = vel*10;   // 实际发送的数据是实际转速*10
    struct can_frame vel_fame;
    vel_fame.can_id  = 0x400 + nodeId;
	vel_fame.can_dlc = 6;
	std::memset(vel_fame.data, 0, sizeof(vel_fame.data));   // 全部赋值为0
	std::memcpy(vel_fame.data, &rpm, sizeof(rpm));          // 直接使用 memcpy() 可以同时解决字节序的问题
    //std::cout << "set velocity to " << vel << " rad/s" << std::endl;
    socketcanPtr->can_write(vel_fame);                      // 发送
}

/**
 * @brief: 绝对位置控制
 * @param {float} angle 角度，单位是脉冲数量， 每65536个脉冲为一圈
 * @param {int32_t} vel 旋转速度
 * @return {*}
 */
int Motor::set_pos(float angle, int32_t vel)
{
    if (ctrl_mode != MODE_POS)
    {
        std::cout << "ctrl mode: mismatch/NOT set" << std::endl;
        return -1;
    }
    
    // data为8个byte。高4byte为位置指令。低4byte为（速度*10）。 每65536个脉冲为一圈
    int32_t ticks = angle * TICKS_PER_RAD;  // 换算成脉冲数量
    int32_t rpm = vel*10;                   // 发送速度值需要乘以10

    struct can_frame pos_fame;
    pos_fame.can_id  = 0x500 + nodeId;
	pos_fame.can_dlc = 8;
	std::memset(pos_fame.data, 0, sizeof(pos_fame.data));
    std::memcpy(pos_fame.data, &ticks, sizeof(ticks));     // 设置转动的脉冲数
    std::memcpy(pos_fame.data + 4, &rpm, sizeof(rpm));     // 设置旋转的速度
    //std::cout << "set target angle to " << angle << " rad" << std::endl;
    socketcanPtr->can_write(pos_fame);      // 发送目标位置和速度，注意此时并未执行

    execute_posCtrl();  // 执行位置控制指令
}

/**
 * @brief: 执行位置控制。位置数据发送之后，需要再发送执行指令才会开始旋转
 * @return {*}
 */
int Motor::execute_posCtrl()
{
    struct can_frame ctrl_fame;
    ctrl_fame.can_id  = 0x200 + nodeId;
	ctrl_fame.can_dlc = 2;

	std::memset(ctrl_fame.data, 0, sizeof(ctrl_fame.data));
    ctrl_fame.data[0] = 0x3F;
    ctrl_fame.data[1] = 0x00;    //200+(ID) 3F 00
    
    //std::cout<< RED << "motor ID: " << nodeId << " execute_posCtrl" << std::endl;
    socketcanPtr->can_write(ctrl_fame);      // 发送执行绝对位置控制指令
}

/**
 * @brief: 挂起（暂停）动作
 * @return {*}
 */
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

/**
 * @brief: 恢复暂停的动作
 * @return {*}
 */
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

/**
 * @brief: 失能
 * @return {*}
 */
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

/**
 * @brief: 回零，注意，现在的舵轮用的是绝对值编码器，不需要回零。一旦回零会转到电子限位处并把电子限位处设为零点。需要通过485在地面站将零点重新改为0
 * @return {*}
 */
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