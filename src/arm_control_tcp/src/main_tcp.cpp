#include "arm_control_tcp/rosCRControl.h"
#include "arm_control_tcp/teleop_data_sub_tcp.h"
#include <sensor_msgs/JointState.h>
#include <thread>

const double PI = 3.1415;
using namespace std;


//使用mutex保护数据
mutex dataMutex;

//函数声明


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
	ros::Subscriber jointStateSub_tcp = nh_sub.subscribe("Geomagic/joint_states",200,jointStateCallback);
	// ros::Subscriber poseSub = nh.subscribe("Geomagic/pose",10,poseCallback);
	// ros::Subscriber twistSub = nh.subscribe("Geomagic/twist",10,twistCallback);
	ros::Subscriber joySub_tcp = nh_sub.subscribe("Geomagic/joy",200,joyCallback);

	//设置主循环频率，即通过rate.sleep()控制后面while一次的时间
	ros::Rate rate(200);

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
	// const double minAngleChange = 1 * PI / 180;; // 小于3度的变化不发送指令
	// const double maxAngleChange = 10 * PI / 180;; // 小于3度的变化不发送指令


    ROS_INFO("1");
    // 创建一个新线程，并将obj和data作为引用传递给匿名函数
    std::thread threadMove([&serviceHandler, &joints]() {
        int currentCommandID = 2147483647;    // 初始值  int-max
		ros::Rate rate_robot(33);

        while (ros::ok()) {
			int currentMode = deviceData.mode;
			if(currentMode == 1){
				// ROS_INFO("3---------------------------------------");
				// ros::Time currentTime1 = ros::Time::now();
			    // ROS_INFO("currentTime1: %f", currentTime1.toSec());
            	serviceHandler.moveJoint(joints, currentCommandID);
            	// serviceHandler.finishPoint(currentCommandID);
            	// ROS_INFO("Mode 1: Joint states sent to move_j");
				// ros::Time currentTime2 = ros::Time::now();
			    // ROS_INFO("currentTime2: %f", currentTime2.toSec());
			} else{
				serviceHandler.movePoint(joints, currentCommandID);
    			serviceHandler.finishPoint(currentCommandID);
			}
			rate_robot.sleep();
        }
    });
    threadMove.detach();

    ROS_INFO("4");

    while (ros::ok())
	{      
        int currentMode = deviceData.mode;
		// if(currentMode == 1){
            // ROS_INFO("5");
			// 在每次接收到新的关节状态后，进行差值判断
			bool angAngleBigMin = true; // 用于标记是否需要发送控制指令
			bool warn = false;

            vector<double> jointsTmp;

			jointsTmp.clear();
			jointsTmp = deviceData.jointState.position;
			jointsTmp[0] = deviceData.jointState.position[0];
            jointsTmp[1] = (PI/2 - deviceData.jointState.position[1]);
            jointsTmp[2] = PI/2 - ( deviceData.jointState.position[2]);
            jointsTmp[3] = deviceData.jointState.position[4] + 2.4;
            jointsTmp[4] = deviceData.jointState.position[3] - PI/2 - PI;
            jointsTmp[5] = deviceData.jointState.position[5] - 3;
			
            for(size_t i = 0; i < jointsTmp.size(); ++i){
				jointsTmp[i] = (jointsTmp[i] * 180) / PI;
                ROS_INFO("Joint %zu: %f", i+1, jointsTmp[i]);
			}

			// for (size_t i = 0; i < jointsTmp.size(); ++i) {
			// 	double diff = abs(jointsTmp[i] - previousJoints[i]); // 计算关节角度差值
			// 	// if (diff > minAngleChange) {
			// 	// 	angAngleBigMin = true;      // 如果某个关节角度差值大于(3度)，则发送控制指令
			// 	// }
			// 	// if(diff > maxAngleChange){
			// 	// 	warn = true;
			// 	// }
			// 	ROS_INFO("diff %zu: %f", i+1, diff);
			// }



			if(angAngleBigMin && !warn){
				joints = jointsTmp;
				// ROS_INFO("Mode 1: Joint states sent to move_j");
				//获取当前时间
				// ros::Time currentTime = ros::Time::now();
				// ROS_INFO("Time2: %f", currentTime.toSec());

				previousJoints = joints;
			}
        // }
		ros::spinOnce();
		rate.sleep();
	}
    return 0;
}




// 后期打印数据时使用

// 打印保存的关节角度随时间变化的信息 (前五组) 
void printJointAngles() { 
	std::cout << "Joint Angles vs Time:" << std::endl; 
	for (size_t i = 0; i < 5; ++i) { 
		std::cout << "Time: " << record_timestamps[i] << "s - Angles: ";
		for (const auto& angle : record_joint_angles[i]) { 
			std::cout << angle << " "; 
		} 
		std::cout << std::endl;
	} 
}


// 函数使用
// saveToFile("joint_angles_data.csv"); // 保存为 CSV 文件


// 将数据保存到文件 
void saveToFile(const std::string& filename) { 
	std::ofstream outfile(filename); // 打开文件 
	if (!outfile.is_open()) { 
		std::cerr << "Failed to open file for writing!" << std::endl; 
		return; 
	} 
	
	// 写入表头 
	outfile << "Time(s), Joint 1, Joint 2, Joint 3, ..., Joint N" << std::endl;

	// 写入数据 
	for (size_t i = 0; i < record_joint_angles.size(); ++i){
		// 写入时间戳 
		outfile << record_timestamps[i] << ", "; 

		// 写入关节角度 
		for (size_t j = 0; j < record_joint_angles[i].size(); ++j){ 
			outfile << record_joint_angles[i][j]; 
			if (j < record_joint_angles[i].size() - 1) { 
				outfile << ", "; // 逗号分隔 
			} 
		} 
		outfile << std::endl; // 换行 
	} 
	outfile.close(); // 关闭文件 
	std::cout << "Data saved to " << filename << std::endl; 
}