#ifndef __TELEOP_DATA_SUB_H_
#define __TELEOP_DATA_SUB_H_

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include "cr10_moveit_class.h"

//遥操作数据存储结构体，存储订阅到的数据
struct DeviceData{
    sensor_msgs::JointState jointState;           //各个关节的角度
    geometry_msgs::PoseStamped endEffectorPose;   //末端位置（xyz加四元数）
    geometry_msgs::Twist twist;                   //各个方向的线速度和角速度
    sensor_msgs::Joy joy;                         //三个轴的位置，三个轴的速度，按钮的状态
    int mode = 0;
};

//实例化一个全局变量，用于存储数据
extern DeviceData deviceData;

//回调函数声明，用于处理接收到数据之后的处理
void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
void twistCallback(const geometry_msgs::Twist::ConstPtr& msg);
void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);


#endif