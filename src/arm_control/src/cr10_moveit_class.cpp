#include "arm_control/cr10_moveit_class.h"

using namespace std;


//定义一个类

//构造函数
MoveIt_Control::MoveIt_Control(const ros::NodeHandle &nh,moveit::planning_interface::MoveGroupInterface &arm,const string &PLANNING_GROUP) {
	
	this->arm_ = &arm;
	this->nh_ =nh;
	

	//设置限制,精度
	arm_->setGoalPositionTolerance(0.01);
	arm_->setGoalOrientationTolerance(0.01);
	arm_->setGoalJointTolerance(0.01);

	//调节速度，百分之多少
	arm_->setMaxAccelerationScalingFactor(0.5);
	arm_->setMaxVelocityScalingFactor(0.5);

	const moveit::core::JointModelGroup* joint_model_group =
		arm_->getCurrentState()->getJointModelGroup(PLANNING_GROUP);

	this->end_effector_link = arm_->getEndEffectorLink();

	//参考坐标系
	this->reference_frame = "base_link";
	arm_->setPoseReferenceFrame(reference_frame);
	
	arm_->allowReplanning(true);
	arm_->setPlanningTime(1.0);
	//设定规划算法
	arm_->setPlannerId("RRTConnect");

	//恢复初始状态
	// go_home();

	//创建一个桌子
	create_table();

}	

void MoveIt_Control::go_home() {
	// moveit::planning_interface::MoveGroupInterface arm("manipulator");
	//这里up对应的是设定好的位置的名字
	arm_->setNamedTarget("home");
	arm_->move();
	sleep(0.5);
}

//输入参数是给他六个关节的弧度值
bool MoveIt_Control::move_j(const vector<double> &joint_group_positions) {
	// moveit::planning_interface::MoveGroupInterface arm("manipulator");
	arm_->setJointValueTarget(joint_group_positions);
	arm_->move();
	// sleep(0.5);
	return true;
}

//输入参数是一个位置，用六个参数的位姿来描述，xyzrpy
bool MoveIt_Control::move_p(const vector<double> &pose) {
	// moveit::planning_interface::MoveGroupInterface arm("manipulator");
	// const std::string reference_frame = "base";
	// arm.setPoseReferenceFrame(reference_frame);

	
	geometry_msgs::Pose target_pose;
	target_pose.position.x = pose[0];
	target_pose.position.y = pose[1];
	target_pose.position.z = pose[2];

	
	tf2::Quaternion myQuaternion;
	myQuaternion.setRPY(pose[3], pose[4], pose[5]);
	target_pose.orientation.x = myQuaternion.getX();
	target_pose.orientation.y = myQuaternion.getY();
	target_pose.orientation.z = myQuaternion.getZ();
	target_pose.orientation.w = myQuaternion.getW();

	
	arm_->setStartStateToCurrentState();
	arm_->setPoseTarget(target_pose);

	
	moveit::planning_interface::MoveGroupInterface::Plan plan;
	// moveit::planning_interface::MoveItErrorCode success = arm_->plan(plan);  moveit::planning_interface::MoveItErrorCode被启用，改为core
	moveit::core::MoveItErrorCode success = arm_->plan(plan);

	ROS_INFO("move_p:%s", success ? "SUCCESS" : "FAILED");

	
	if (success) {
		arm_->execute(plan);
		sleep(1);
		return true;
	}
	return false;
}


//带限制的移动，本例中为目标姿态不变
bool MoveIt_Control::move_p_with_constrains(const vector<double>& pose) {

	// moveit::planning_interface::MoveGroupInterface arm("manipulator");
	// const std::string reference_frame = "base";
	// arm.setPoseReferenceFrame(reference_frame);
	// arm.setPlannerId("TRRT");
	arm_->setMaxAccelerationScalingFactor(0.5);
	arm_->setMaxVelocityScalingFactor(0.5);

	geometry_msgs::Pose target_pose;
	target_pose.position.x = pose[0];
	target_pose.position.y = pose[1];
	target_pose.position.z = pose[2];

	tf2::Quaternion myQuaternion;
	myQuaternion.setRPY(pose[3], pose[4], pose[5]);
	target_pose.orientation.x = myQuaternion.getX();
	target_pose.orientation.y = myQuaternion.getY();
	target_pose.orientation.z = myQuaternion.getZ();
	target_pose.orientation.w = myQuaternion.getW();

	// geometry_msgs::PoseStamped current_pose_modified = arm.getCurrentPose(this->end_effector_link);
	// current_pose_modified.header.frame_id = "base";
	// current_pose_modified.pose.position.x = -current_pose_modified.pose.position.x ;
	// current_pose_modified.pose.position.y = -current_pose_modified.pose.position.y ;
	// current_pose_modified.pose.orientation.x = myQuaternion.getX();
	// current_pose_modified.pose.orientation.y = myQuaternion.getY();
	// current_pose_modified.pose.orientation.z = myQuaternion.getZ();
	// current_pose_modified.pose.orientation.w = myQuaternion.getW();
	// arm.setPoseTarget(current_pose_modified.pose);arm.move();
	

	//set constraint 保持末端姿态不变
	moveit_msgs::OrientationConstraint ocm;
	ocm.link_name = "base_link";
	ocm.header.frame_id = "Clamp";
	ocm.orientation.x = myQuaternion.getX();
	ocm.orientation.y = myQuaternion.getY();
	ocm.orientation.z = myQuaternion.getZ();
	ocm.orientation.w = myQuaternion.getW();
	//如果为零，可以导致规划一直失败
	ocm.absolute_x_axis_tolerance = 0.1;
	ocm.absolute_y_axis_tolerance = 0.1;
	ocm.absolute_z_axis_tolerance = 0.1;
	ocm.weight = 1.0;

	// Now, set it as the path constraint for the group.
	moveit_msgs::Constraints test_constraints;
	test_constraints.orientation_constraints.push_back(ocm);
	arm_->setPathConstraints(test_constraints);

	/*moveit::core::RobotState start_state(*move_group_interface.getCurrentState());
	geometry_msgs::Pose start_pose2;
	start_pose2.orientation.w = 1.0;
	start_pose2.position.x = 0.55;
	start_pose2.position.y = -0.05;
	start_pose2.position.z = 0.8;
	start_state.setFromIK(joint_model_group, start_pose2);
	move_group_interface.setStartState(start_state);*/

	// Now we will plan to the earlier pose target from the new
	// start state that we have just created.
	arm_->setStartStateToCurrentState();
	arm_->setPoseTarget(target_pose);

	// Planning with constraints can be slow because every sample must call an inverse kinematics solver.
	// Lets increase the planning time from the default 5 seconds to be sure the planner has enough time to succeed.
	//设定规划时间为10秒，因为此操作相对更复杂，时间长可以更好的规划，后面重新设回来
	arm_->setPlanningTime(10.0);

	moveit::planning_interface::MoveGroupInterface::Plan plan;
	// moveit::planning_interface::MoveItErrorCode success = arm_->plan(plan);
	moveit::core::MoveItErrorCode success = arm_->plan(plan);

	ROS_INFO("move_p_with_constrains :%s", success ? "SUCCESS" : "FAILED");

	arm_->setPlanningTime(5.0);
	arm_->clearPathConstraints();
	if (success) {
		arm_->execute(plan);
		sleep(1);
		return true;
	}
	return false;
}

//传入参数是一个点的坐标，从当前位置直线前往目标点
bool MoveIt_Control::move_l(const vector<double>& pose) {
	// moveit::planning_interface::MoveGroupInterface arm("manipulator");

	vector<geometry_msgs::Pose> waypoints;
	geometry_msgs::Pose target_pose;
	target_pose.position.x = pose[0];
	target_pose.position.y = pose[1];
	target_pose.position.z = pose[2];

	
	tf2::Quaternion myQuaternion;
	myQuaternion.setRPY(pose[3], pose[4], pose[5]);
	target_pose.orientation.x = myQuaternion.getX();
	target_pose.orientation.y = myQuaternion.getY();
	target_pose.orientation.z = myQuaternion.getZ();
	target_pose.orientation.w = myQuaternion.getW();
	waypoints.push_back(target_pose);

	
	moveit_msgs::RobotTrajectory trajectory;
	const double jump_threshold = 0.0;
	const double eef_step = 0.01;
	double fraction = 0.0;
	int maxtries = 100;   
	int attempts = 0;     

	while (fraction < 1.0 && attempts < maxtries)
	{
		// fraction = arm_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);  jump_threshold已被弃用，这个参数是用于控制路径计算的，允许机器人跳跃，目前已被去除，现在会自动处理路径计算，不用手动设置
		fraction = arm_->computeCartesianPath(waypoints, eef_step, trajectory);
		attempts++;
	}

	if (fraction == 1)
	{
		ROS_INFO("Path computed successfully. Moving the arm.");

		
		moveit::planning_interface::MoveGroupInterface::Plan plan;
		plan.trajectory_ = trajectory;

		
		arm_->execute(plan);
		sleep(1);
		return true;
	}
	else
	{
		ROS_INFO("Path planning failed with only %0.6f success after %d attempts.", fraction, maxtries);
		return false;
	}
}
	
//重载，可以传入多个点，走折线
bool MoveIt_Control::move_l(const vector<vector<double>>& posees) {
	// moveit::planning_interface::MoveGroupInterface arm("manipulator");
	vector<geometry_msgs::Pose> waypoints;
	for (int i = 0; i < posees.size(); i++) {
		geometry_msgs::Pose target_pose;
		target_pose.position.x = posees[i][0];
		target_pose.position.y = posees[i][1];
		target_pose.position.z = posees[i][2];

		
		tf2::Quaternion myQuaternion;
		myQuaternion.setRPY(posees[i][3], posees[i][4], posees[i][5]);
		target_pose.orientation.x = myQuaternion.getX();
		target_pose.orientation.y = myQuaternion.getY();
		target_pose.orientation.z = myQuaternion.getZ();
		target_pose.orientation.w = myQuaternion.getW();
		waypoints.push_back(target_pose);
	}

	
	moveit_msgs::RobotTrajectory trajectory;
	const double jump_threshold = 0.0;
	const double eef_step = 0.01;
	double fraction = 0.0;
	int maxtries = 100;   
	int attempts = 0;     

	while (fraction < 1.0 && attempts < maxtries)
	{
		// fraction = arm_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);  jump_threshold已被弃用，这个参数是用于控制路径计算的，允许机器人跳跃，目前已被去除，现在会自动处理路径计算，不用手动设置
		fraction = arm_->computeCartesianPath(waypoints, eef_step, trajectory);
		attempts++;
	}

	if (fraction == 1)
	{
		ROS_INFO("Path computed successfully. Moving the arm.");

		
		moveit::planning_interface::MoveGroupInterface::Plan plan;
		plan.trajectory_ = trajectory;

		
		arm_->execute(plan);
		sleep(1);
		return true;
	}
	else
	{
		ROS_INFO("Path planning failed with only %0.6f success after %d attempts.", fraction, maxtries);
		return false;
	}
}

//创建环境
void MoveIt_Control::create_table() {
	
	// Now let's define a collision object ROS message for the robot to avoid.

	ros::Publisher planning_scene_diff_publisher = nh_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
	ros::WallDuration sleep_t(0.5);
	while (planning_scene_diff_publisher.getNumSubscribers() < 1)
	{
		sleep_t.sleep();
	}
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	moveit_msgs::PlanningScene planning_scene;
	moveit_msgs::CollisionObject collision_object;
	collision_object.header.frame_id = arm_->getPlanningFrame();

	//开始创建
	// The id of the object is used to identify it.
	collision_object.id = "table";

	// Define a box to add to the world.
	shape_msgs::SolidPrimitive primitive;
	primitive.type = primitive.BOX;
	primitive.dimensions.resize(3);
	//定义大小
	primitive.dimensions[primitive.BOX_X] = 1;
	primitive.dimensions[primitive.BOX_Y] = 1;
	primitive.dimensions[primitive.BOX_Z] = 0.7;

	// Define a pose for the box (specified relative to frame_id)
	//定义位姿
	geometry_msgs::Pose box_pose;
	box_pose.orientation.w = 1.0;
	box_pose.position.x = 0.0;
	box_pose.position.y = 0.0;
	box_pose.position.z = -0.7/2-0.02;

	collision_object.primitives.push_back(primitive);
	collision_object.primitive_poses.push_back(box_pose);
	collision_object.operation = collision_object.ADD;


	//创建完成，将其pushback进去
	planning_scene.world.collision_objects.push_back(collision_object);
	planning_scene.is_diff = true;
	planning_scene_diff_publisher.publish(planning_scene);

	ROS_INFO("Added an table into the world");
}
    
void MoveIt_Control::some_functions_maybe_useful(){
	// moveit::planning_interface::MoveGroupInterface arm("manipulator");

	//获取当前位姿，xyz和四元数
	geometry_msgs::PoseStamped current_pose = this->arm_->getCurrentPose(this->end_effector_link);
	ROS_INFO("current pose:x:%f,y:%f,z:%f,Quaternion:[%f,%f,%f,%f]",current_pose.pose.position.x,current_pose.pose.position.y,
	current_pose.pose.position.z,current_pose.pose.orientation.x,current_pose.pose.orientation.y,
	current_pose.pose.orientation.z,current_pose.pose.orientation.w);


	//获取当前关节角度
	std::vector<double> current_joint_values = this->arm_->getCurrentJointValues();
	ROS_INFO("current joint values:%f,%f,%f,%f,%f,%f",current_joint_values[0],current_joint_values[1],current_joint_values[2],
	current_joint_values[3],current_joint_values[4],current_joint_values[5]);


	//获得末端的rpy
	std::vector<double> rpy = this->arm_->getCurrentRPY(this->end_effector_link);
	ROS_INFO("current rpy:%f,%f,%f",rpy[0],rpy[1],rpy[2]);

	//获取当前使用的规划算法，可选择算法在moveit里面的config下的ompl
	string planner = this->arm_->getPlannerId();
	ROS_INFO("current planner:%s",planner.c_str());
	std::cout<<"current planner:"<<planner<<endl;

}

	
MoveIt_Control::~MoveIt_Control() {
	
	ros::shutdown();
}


