#include "ros/ros.h"
//#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "commen.hpp"
#include "kinematics.h"
#include "motor.h"

Kinematics kinematics(WIDTH, LENGTH);

void cmd_velCallback(geometry_msgs::TwistConstPtr cmd)
{
  ROS_INFO("vel_x, vel_y, omega_z: [%f],[%f],[%f]", cmd->linear.x,cmd->linear.y,cmd->angular.z);
  Cmd vel = {cmd->linear.x, cmd->linear.y, cmd->angular.z};
  // 运动学逆解
  kinematics.inverse(vel);
  // 发布控制指令
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "chassis");
    ros::NodeHandle n;
    //ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    ros::Subscriber sub = n.subscribe("cmd_vel", 1000, cmd_velCallback);

    // socket can 通信初始化

    // 舵轮电机初始化

    ros::spin();
    return 0;
}
