#include "ros/ros.h"
//#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "commen.hpp"
#include "kinematics.h"
#include "motor.h"
#include "socketcan.h"
#include <omp.h>

double steer_start;
int count;

float fl_angle_last = 0.0;
float rr_angle_last = 0.0;
float fl_vel_last = 0;
float rr_vel_last = 0;
bool wait_steer = false; 
float wait_time = 0.0;

float target_omega;
float last_target_omega;


Kinematics kinematics(WIDTH, LENGTH);

Socketcan *scPtr = new Socketcan("can0");

Motor m_fl_steer(WHEEL_RADIUS, scPtr, M_FL_STEER_ID);   // 左前转向电机
Motor m_fl_drive(WHEEL_RADIUS, scPtr, M_FL_DDRIVE_ID);  // 左前行走电机

Motor m_rr_steer(WHEEL_RADIUS, scPtr, M_RR_STEER_ID);   // 右后转向电机
Motor m_rr_drive(WHEEL_RADIUS, scPtr, M_RR_DDRIVE_ID);  // 右后行走电机

void cmd_velCallback(geometry_msgs::TwistConstPtr cmd)
{
    count = 0;
    //ROS_INFO("vel_x, vel_y, omega_z: [%f],[%f],[%f]", cmd->linear.x,cmd->linear.y,cmd->angular.z);
    target_omega = cmd->angular.z;
    Cmd vel = {cmd->linear.x, cmd->linear.y, cmd->angular.z};
    // 运动学逆解
    kinematics.inverse(vel);
}

// 用于接收平滑后的速度
void smooth_cmd_vel_Callback(geometry_msgs::TwistConstPtr cmd)
{
    count = 0;
    //ROS_INFO("vel_x, vel_y, omega_z: [%f],[%f],[%f]", cmd->linear.x,cmd->linear.y,cmd->angular.z);
    target_omega = cmd->angular.z;
    Cmd vel = {cmd->linear.x, cmd->linear.y, cmd->angular.z};
    // 运动学逆解
    kinematics.inverse(vel);
}

int main(int argc, char **argv)
{
    // system("sudo ip link set can0 down");   // 关闭can总线
    // system("sudo ip link set can0 type can bitrate 500000"); // 设置波特率，启用loopback
    // system("sudo ip link set can0 up");     // 开启can总线

    //*舵轮电机初始化*
    // 转向电机
    m_fl_steer.StartNMT();          // 启动节点
    m_fl_steer.set_mode(MODE_POS);  // 设置为位置控制
    m_fl_steer.enable();            // 使能电机
    sleep(1);
    m_rr_steer.StartNMT();          // 启动节点
    m_rr_steer.set_mode(MODE_POS);  // 设置为位置控制
    m_rr_steer.enable();            // 使能电机
    sleep(1);

    // 行走电机
    m_fl_drive.StartNMT();          // 启动节点
    m_fl_drive.set_mode(MODE_VEL);  // 设置为位置控制
    m_fl_drive.enable();            // 使能电机
    sleep(1);
    m_rr_drive.StartNMT();          // 启动节点
    m_rr_drive.set_mode(MODE_VEL);  // 设置为速度控制
    m_rr_drive.enable();            // 使能电机
    sleep(1);

    ros::init(argc, argv, "chassis");
    ros::NodeHandle n;
    //ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    ros::Subscriber cmd_vel_sub = n.subscribe("cmd_vel_", 1000, cmd_velCallback);
    ros::Subscriber motor_ctrl_sub = n.subscribe("smooth_cmd_vel", 1000, smooth_cmd_vel_Callback);
    ros::Rate loop_rate(20);    // 20hz
    while (ros::ok())
    {
        count++ ;
        if (count >= 20)
        {
            count = 21;
            m_fl_drive.set_vel(0,0);
            m_rr_drive.set_vel(0,0);
        }
        else
        {
            // method1.如果任何一个车轮需要旋转，就得停下来先把轮子转到目标位置才能给速度。
            //if (wait_steer == false && (fabs(fl_angle_last - kinematics.fl.angle)>0.01 || fabs(rr_angle_last - kinematics.rr.angle)>0.01))
            // if (wait_steer == false && (fl_angle_last != kinematics.fl.angle || rr_angle_last != kinematics.rr.angle))
            // method2.如果目标车身角速度不为零并且与之前发生了变化，就得停下来先把轮子转到目标位置才能给速度。
            if (wait_steer == false && fabs(target_omega)>0.01 && fabs(last_target_omega - target_omega) >= 0.01)
            {
                wait_steer = true;
                steer_start = omp_get_wtime();
                // 停车
                m_fl_drive.set_vel(0,0);
                m_rr_drive.set_vel(0,0);
                // 求轮子转向的等待的时间
                float wait_time_fl = fabs(kinematics.fl.angle - fl_angle_last) / (STEER_SPEED/STEER_GEAR_RATIO/60*_2PI);
                //std::cout <<"kinematics.fl.angle" << kinematics.fl.angle<< std::endl;
                //std::cout <<"fl_angle_last" << fl_angle_last<< std::endl;

                float wait_time_rr = fabs(kinematics.rr.angle - rr_angle_last) / (STEER_SPEED/STEER_GEAR_RATIO/60*_2PI);
                wait_time = (wait_time_fl > wait_time_rr) ? wait_time_fl : wait_time_rr;
                wait_time = wait_time * 1.2;
                //std::cout <<"wait_time" << wait_time<< std::endl;

                // 记录此次指令
                fl_angle_last = kinematics.fl.angle;
                rr_angle_last = kinematics.rr.angle;
                fl_vel_last = kinematics.fl.speed;
                rr_vel_last = kinematics.rr.speed;

                last_target_omega = target_omega;
            }
            else if(wait_steer == false)
            {
                fl_angle_last = kinematics.fl.angle;
                rr_angle_last = kinematics.rr.angle;
                fl_vel_last = kinematics.fl.speed;
                rr_vel_last = kinematics.rr.speed;
                last_target_omega = target_omega;

                //std::cout <<"walk" << std::endl;
                // 不需要等待转向电机转向的话，直接更新转速即可
                int32_t fl_vel_data =   DRIVE_GEAR_RATIO * kinematics.fl.speed * 60 /(2*_PI*WHEEL_RADIUS);
                int32_t rr_vel_data = - DRIVE_GEAR_RATIO * kinematics.rr.speed * 60 /(2*_PI*WHEEL_RADIUS);
                m_fl_drive.set_vel(fl_vel_data,0);
                m_rr_drive.set_vel(rr_vel_data,0);

                float fl_angle_data = STEER_GEAR_RATIO * kinematics.fl.angle;
                float rr_angle_data = STEER_GEAR_RATIO * kinematics.rr.angle;
                m_fl_steer.set_pos(fl_angle_data,STEER_SPEED);
                m_rr_steer.set_pos(rr_angle_data,STEER_SPEED);
            }

            if (wait_steer == true)
            {
                // 停车
                m_fl_drive.set_vel(0,0);
                m_rr_drive.set_vel(0,0);
                //发布位置
                float fl_angle_data = STEER_GEAR_RATIO * fl_angle_last;
                float rr_angle_data = STEER_GEAR_RATIO * rr_angle_last;
                m_fl_steer.set_pos(fl_angle_data,STEER_SPEED);
                m_rr_steer.set_pos(rr_angle_data,STEER_SPEED);
            }

            // 等待转向完成了，把对应的速度发出去
            if (wait_steer == true && omp_get_wtime()-steer_start >= wait_time)
            {
                int32_t fl_vel_data =   DRIVE_GEAR_RATIO * fl_vel_last * 60 /(2*_PI*WHEEL_RADIUS);
                int32_t rr_vel_data = - DRIVE_GEAR_RATIO * rr_vel_last * 60 /(2*_PI*WHEEL_RADIUS);
                m_fl_drive.set_vel(fl_vel_data,0);
                m_rr_drive.set_vel(rr_vel_data,0);
                wait_steer = false;
            }
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
