#ifndef __CR10_MOVEIT_CLASS_H_
#define __CR10_MOVEIT_CLASS_H_

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <tf2/LinearMath/Quaternion.h>
#include <vector>
#include <string>
#include <iostream>

using namespace std;


//定义一个类
class MoveIt_Control
{
public:
	
	//构造函数
	MoveIt_Control(const ros::NodeHandle &nh,moveit::planning_interface::MoveGroupInterface &arm,const string &PLANNING_GROUP);

	void go_home();

	//输入参数是给他六个关节的弧度值
	bool move_j(const vector<double> &joint_group_positions);

	//输入参数是一个位置，用六个参数的位姿来描述，xyzrpy
	bool move_p(const vector<double> &pose);


	//带限制的移动，本例中为目标姿态不变
	bool move_p_with_constrains(const vector<double>& pose);

	//传入参数是一个点的坐标，从当前位置直线前往目标点
	bool move_l(const vector<double>& pose);
	
	//重载，可以传入多个点，走折线
	bool move_l(const vector<vector<double>>& posees);

	//创建环境
	void create_table();
    
	void some_functions_maybe_useful();

	
	~MoveIt_Control();


public:
	
	string reference_frame;
	string end_effector_link;
	ros::NodeHandle nh_;
	moveit::planning_interface::MoveGroupInterface *arm_;
};

#endif