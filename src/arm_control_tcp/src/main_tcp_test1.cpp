//没有单独开线程


#include "arm_control_tcp/rosCRControl.h"
#include "arm_control_tcp/teleop_data_sub_tcp.h"
#include <sensor_msgs/JointState.h>
#include <thread>

const double PI = 3.1415;
using namespace std;

int main(int argc, char** argv)
{
    setlocale(LC_ALL, "");    // 中文乱码
    ros::init(argc, argv, "arm_control_tcp");
    ros::NodeHandle nh("~");
    RosDemoCRV4 serviceHandler(&nh);

	//创建一个线程，用于处理回调函数，这个就不用ros::spin()了。这里订阅消息的任务简单，一个线程足够了
	ros::AsyncSpinner spinner(1);

    ros::NodeHandle nh_sub;

    //启动线程
	spinner.start();

	//遥控器数据订阅
	//参数泛型（例如<sensor_msgs::JointState>）可以不用写，自动推导 
	//第一个参数：话题
	//第二个队列长度，队列存放尚未处理的数据
	//第三个回调函数
	ros::Subscriber jointStateSub_tcp = nh_sub.subscribe("Geomagic/joint_states",100,jointStateCallback);
	// ros::Subscriber poseSub = nh.subscribe("Geomagic/pose",10,poseCallback);
	// ros::Subscriber twistSub = nh.subscribe("Geomagic/twist",10,twistCallback);
	ros::Subscriber joySub_tcp = nh_sub.subscribe("Geomagic/joy",100,joyCallback);

	//设置主循环频率，即通过rate.sleep()控制后面while一次的时间
	ros::Rate rate(50);

    //接收jointState数据
    vector<double> joints = {10, 9, 108, -19, -97, 4};
    // 定义一个变量来存储上一个关节角度
    vector<double> previousJoints = {10, 9, 108, -19, -97, 4}; // 默认初始化为6个0.0

    ros::Duration(1.0).sleep();

    // previousJoints[0] = 0.5*(deviceData.jointState.position[0]);
	// previousJoints[1] = 0.5*(PI/2 - deviceData.jointState.position[1]);
	// previousJoints[2] = PI/2 - deviceData.jointState.position[2];
	// previousJoints[3] = deviceData.jointState.position[4] + 2.4;
	// previousJoints[4] = deviceData.jointState.position[3] - PI/2 - PI;
	// previousJoints[5] = deviceData.jointState.position[5] - 3;

    // 角度变化的容忍度
	const double minAngleChange = 1 * PI / 180;; // 小于3度的变化不发送指令
	const double maxAngleChange = 10 * PI / 180;; // 小于3度的变化不发送指令

	int currentCommandID = 2147483647;
	
    while (ros::ok())
	{      
        int currentMode = deviceData.mode;
		if(currentMode == 1){
            // ROS_INFO("5");
			// 在每次接收到新的关节状态后，进行差值判断
			bool angAngleBigMin = true; // 用于标记是否需要发送控制指令
			bool warn = false;


			joints.clear();
			joints = deviceData.jointState.position;
			joints[0] = 0.5 * deviceData.jointState.position[0];
            joints[1] = 0.5 * (PI/2 - deviceData.jointState.position[1]);
            joints[2] = PI/2 - (0.5 * deviceData.jointState.position[2]);
            joints[3] = deviceData.jointState.position[4] + 2.4;
            joints[4] = deviceData.jointState.position[3] - PI/2 - PI;
            joints[5] = deviceData.jointState.position[5] - 3;
			
            for(size_t i = 0; i < joints.size(); ++i){
				joints[i] = (joints[i] * 180) / PI;
                ROS_INFO("Joint %zu: %f", i+1, joints[i]);
			}

			for (size_t i = 0; i < joints.size(); ++i) {
				double diff = abs(joints[i] - previousJoints[i]); // 计算关节角度差值
				// if (diff > minAngleChange) {
				// 	angAngleBigMin = true;      // 如果某个关节角度差值大于(3度)，则发送控制指令
				// }
				// if(diff > maxAngleChange){
				// 	warn = true;
				// }
				ROS_INFO("diff %zu: %f", i+1, diff);
			}



			if(angAngleBigMin && !warn){

				ros::Time currentTime1 = ros::Time::now();
				// ROS_INFO("Time1: %f", currentTime1.toSec());
				serviceHandler.movePoint(joints, currentCommandID);
            	// ROS_INFO("CurrentCommandID2  %d", currentCommandID);
            	serviceHandler.finishPoint(currentCommandID);
				ROS_INFO("Mode 1: Joint states sent to move_j");
				//获取当前时间
				ros::Time currentTime2 = ros::Time::now();
				ROS_INFO("Time: %f", currentTime2.toSec() - currentTime1.toSec());

				previousJoints = joints;
			}
        }
		ros::spinOnce();
		rate.sleep();
	}
    return 0;
}
