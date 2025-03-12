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

	// test for move_j
	// cout<<"-----------------------test for move_j----------------------"<<endl;
	// vector<double> joints ={0,0,1.74,0,-0.1,0.8};
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
	cout<<"-----------------------test for move_p_with_constrains----------------------"<<endl;
	vector<double> pose1={-0.7,-0.1,0.3,0,3.141592,0};
	moveit_server.move_p(pose1);
	// vector<double> pose2={-0.8,-0.1,0.2,0,3.141592,0}; //3.141592/2
	// moveit_server.move_p_with_constrains(pose2);
	// vector<double> pose3={-0.8,-0.1,0.3,0,3.141592,0};
	// moveit_server.move_p_with_constrains(pose3);
	// vector<double> pose4={-0.7,-0.1,0.3,0,3.141592,0};
	// moveit_server.move_p_with_constrains(pose4);

	vector<double> pose2={-0.8,-0.2,0.3,0,3.141592,0}; //3.141592/2
	moveit_server.move_p(pose2);
	// vector<double> pose3={-0.8,-0.3,0.4,0,3.141592,0};
	// moveit_server.move_p(pose3);
	// vector<double> pose4={-0.7,-0.1,0.3,0,3.141592,0};
	// moveit_server.move_p(pose4);

	// // test for some useful functions
	// cout<<"-----------------------test for other functions----------------------"<<endl;
	// moveit_server.some_functions_maybe_useful();
	
	return 0;
}

