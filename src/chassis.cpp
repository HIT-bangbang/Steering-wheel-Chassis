#include "ros/ros.h"
//#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "commen.hpp"
#include "kinematics.h"
#include "motor.h"
#include "socketcan.h"

Kinematics kinematics(WIDTH, LENGTH);

Socketcan *scPtr = new Socketcan("can0");

Motor m_fl_steer(WHEEL_RADIUS, scPtr, M_FL_STEER_ID);   // 左前转向轮
Motor m_fl_drive(WHEEL_RADIUS, scPtr, M_FL_DDRIVE_ID);  // 左前转向轮

void cmd_velCallback(geometry_msgs::TwistConstPtr cmd)
{
  ROS_INFO("vel_x, vel_y, omega_z: [%f],[%f],[%f]", cmd->linear.x,cmd->linear.y,cmd->angular.z);
  Cmd vel = {cmd->linear.x, cmd->linear.y, cmd->angular.z};
  // 运动学逆解
  kinematics.inverse(vel);
  // 发布控制指令
}

// 此函数用于测试使用
void motor_ctrl_Callback(geometry_msgs::TwistConstPtr cmd)
{
    ROS_INFO("motor_ctrl_Callback: set angle to: [%f] rad", cmd->angular.z);
    m_fl_steer.set_pos(cmd->angular.z, 100);
}

int main(int argc, char **argv)
{
    //system("sudo ip link set can0 down");   // 关闭can总线
    //system("sudo ip link set can0 type can bitrate 1000000 loopback on"); // 设置波特率，启用loopback
    //system("sudo ip link set can0 up");     // 开启can总线

    //*舵轮电机初始化*
    m_fl_steer.StartNMT();          // 启动节点
    m_fl_steer.set_mode(MODE_POS);  // 设置为位置控制
    m_fl_steer.enable();            // 使能电机
    m_fl_steer.return_to_zero();    // 回零


    ros::init(argc, argv, "chassis");
    ros::NodeHandle n;
    //ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    ros::Subscriber cmd_vel_sub = n.subscribe("cmd_vel", 1000, cmd_velCallback);
    ros::Subscriber motor_ctrl_sub = n.subscribe("motor_ctrl", 1000, motor_ctrl_Callback);

    // socket can 通信初始化

    ros::spin();
    return 0;
}
