#include "ros/ros.h"
#include "arm_control/cr10_moveit_class.h"
#include "arm_control/teleop_data_sub.h"

const double PI = 3.1415;

using namespace std;

int main(int argc, char** argv) {

	//机械臂初始化
	ros::init(argc, argv, "cr10_moveit_control_cpp");
	//创建一个线程，用于处理回调函数，这个就不用ros::spin()了。这里订阅消息的任务简单，一个线程足够了
	ros::AsyncSpinner spinner(1);
	//创建句柄
	ros::NodeHandle nh;
	//启动线程
	spinner.start();
	//需要操作的组 PLANNING_GROUP = "arm"
	static const std::string PLANNING_GROUP = "arm";
	moveit::planning_interface::MoveGroupInterface arm(PLANNING_GROUP);
	
	MoveIt_Control moveit_server(nh,arm,PLANNING_GROUP);

	//遥控器数据订阅
	//参数泛型（例如<sensor_msgs::JointState>）可以不用写，自动推导 
	//第一个参数：话题
	//第二个队列长度，队列存放尚未处理的数据
	//第三个回调函数
	ros::Subscriber jointStateSub = nh.subscribe("Geomagic/joint_states",1000,jointStateCallback);
	// ros::Subscriber poseSub = nh.subscribe("Geomagic/pose",10,poseCallback);
	// ros::Subscriber twistSub = nh.subscribe("Geomagic/twist",10,twistCallback);
	ros::Subscriber joySub = nh.subscribe("Geomagic/joy",1000,joyCallback);

	//设置主循环频率，即通过rate.sleep()控制后面while一次的时间
	ros::Rate rate(1000);

	//接收jointState数据
	vector<double> joints;

	// 定义一个变量来存储上一个关节角度
    vector<double> previousJoints(6, 0.0); // 默认初始化为6个0.0
	
	ros::Duration(1.0).sleep();

	// previousJoints.clear();
	// cout << "deviceData.jointState.positionk is :" << deviceData.jointState.position[1] << endl;
	// previousJoints = deviceData.jointState.position;
	previousJoints[0] = 0.5*(deviceData.jointState.position[0]);
	previousJoints[1] = 0.5*(PI/2 - deviceData.jointState.position[1]);
	previousJoints[2] = PI/2 - deviceData.jointState.position[2];
	previousJoints[3] = deviceData.jointState.position[4] + 2.4;
	previousJoints[4] = deviceData.jointState.position[3] - PI/2 - PI;
	previousJoints[5] = deviceData.jointState.position[5] - 3;


	// 角度变化的容忍度
	const double minAngleChange = 3 * PI / 180;; // 小于3度的变化不发送指令
	const double maxAngleChange = 10 * PI / 180;; // 小于3度的变化不发送指令

	while (ros::ok())
	{
		int currentMode = deviceData.mode;
		if(currentMode == 1){
			// 在每次接收到新的关节状态后，进行差值判断
			bool angAngleBigMin = false; // 用于标记是否需要发送控制指令
			bool warn = false;

			joints.clear();
			joints = deviceData.jointState.position;
			joints[0] = 0.5 * deviceData.jointState.position[0];
            joints[1] = 0.5 * (PI/2 - deviceData.jointState.position[1]);
            joints[2] = PI/2 - (0.5 * deviceData.jointState.position[2]);
            joints[3] = deviceData.jointState.position[4] + 2.4;
            joints[4] = deviceData.jointState.position[3] - PI/2 - PI;
            joints[5] = deviceData.jointState.position[5] - 3;

			for (size_t i = 0; i < joints.size(); ++i) {
				double diff = abs(joints[i] - previousJoints[i]); // 计算关节角度差值
				if (diff > minAngleChange) {
					angAngleBigMin = true;      // 如果某个关节角度差值大于(3度)，则发送控制指令
				}
				// if(diff > maxAngleChange){
				// 	warn = true;
				// }
				ROS_INFO("diff %zu: %f", i+1, diff);
			}

			for(size_t i = 0; i < joints.size(); ++i){
				ROS_INFO("Joint %zu: %f", i+1, joints[i]);
			}

			if(angAngleBigMin && !warn){
				moveit_server.move_j(joints);
				ROS_INFO("Mode 1: Joint states sent to move_j");
				//获取当前时间
				ros::Time currentTime = ros::Time::now();
				ROS_INFO("Time2: %f", currentTime.toSec());

				previousJoints = joints;
			}

		}
	
		// test for move_j
		// cout<<"-----------------------test for move_j----------------------"<<endl;
		// vector<double> joints ={0,0,-1.57,0,0,0};
		// moveit_server.move_j(joints);

		// // test for move_p and move_l(1 point)
		// cout<<"-----------------------test for move_p and move_l---------------------"<<endl;
		// vector<double> xyzrpy={-0.5,-0.5,0.3,-3.1415,0,0};
		// moveit_server.move_p(xyzrpy);
		// xyzrpy[2]=0.2;
		// moveit_server.move_l(xyzrpy);

		// // test for move_l (>=2 points)
		// cout<<"-----------------------test for move_l(more points)----------------------"<<endl;
		// vector<vector<double>> xyzrpys;
		// xyzrpys.push_back(xyzrpy);
		// xyzrpy[1]=0.2;
		// xyzrpys.push_back(xyzrpy);
		// xyzrpy[0]=0.4;
		// moveit_server.move_l(xyzrpys);

		// // test for move_p_with constrains
		// cout<<"-----------------------test for move_p_with_constrains----------------------"<<endl;
		// vector<double> pose1={0.4,0,0.4,0,3.141592/2,0};
		// moveit_server.move_p(pose1);
		// vector<double> pose2={0.4,0.2,0.2,0,3.141592/2,0}; //3.141592/2
		// moveit_server.move_p_with_constrains(pose2);
		// vector<double> pose3={0.0,0.5,0.3,0,3.141592/2,0};
		// moveit_server.move_p_with_constrains(pose3);

		// // test for some useful functions
		// cout<<"-----------------------test for other functions----------------------"<<endl;
		// moveit_server.some_functions_maybe_useful();
		ros::spinOnce();
		rate.sleep();
	}
	
	return 0;
}

