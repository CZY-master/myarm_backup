#include "arm_control_tcp/teleop_data_sub_tcp.h"

//定义全局变量
DeviceData deviceData;

std::vector<std::vector<double>> record_joint_angles; // 存储关节角度 
std::vector<double> record_timestamps;             // 存储时间戳
ros::Time start_time;                          // 保存第一个时间戳

void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg){
    deviceData.jointState = *msg;
    // for(size_t i = 0; i < deviceData.jointState.position.size(); ++i){
    //     ROS_INFO("Joint %zu: %f", i+1, deviceData.jointState.position[i]);
    // }
    // ROS_INFO("Receive1------");
    // ros::Time currentTime = ros::Time::now();
	// ROS_INFO("Time3: %f", currentTime.toSec());
    if(record_timestamps.empty()){ 
        start_time = msg->header.stamp; // 记录第一次的时间戳 
    }
    record_joint_angles.push_back(msg->position); // 保存关节角度 
    record_timestamps.push_back((msg->header.stamp - start_time).toSec()); // 保存时间戳
}

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    deviceData.endEffectorPose = *msg;
    // ROS_INFO("Receive2");
}

void twistCallback(const geometry_msgs::Twist::ConstPtr& msg){
    deviceData.twist = *msg;
    // ROS_INFO("Receive3");
}

void joyCallback(const sensor_msgs::Joy::ConstPtr& msg){
    //更新数据
    deviceData.joy = *msg;
    
    // 黑色按钮状态获取
    int blackButton = msg->buttons[0];
    // 灰色按钮状态获取
    int grayButton = msg->buttons[1];

    //获取当前时间
    ros::Time currentTime = ros::Time::now();

    // ROS_INFO("Time1: %f", currentTime.toSec());

    //按键状态标志位及上一时刻的时间
    static bool blackButtonPressed = false;
    static bool grayButtonPressed = false;
    static ros::Time lastBlackPressTime;
    static ros::Time lastGrayPressTime;
    static bool finishChoose = false;

    // 判断当前按键组合的模式
    if(blackButton == 1 && !blackButtonPressed && !grayButtonPressed){
        blackButtonPressed = true;
        lastBlackPressTime = currentTime;
        finishChoose = false;
        // 黑色被按下，等待1s；
        ROS_INFO("Black button pressed, waiting for 1 second...........");
    }

    if(grayButton == 1 && !grayButtonPressed && !blackButtonPressed){
        grayButtonPressed = true;
        lastGrayPressTime = currentTime;
        finishChoose = false;
        // 灰色被按下，等待1s；
        ROS_INFO("Gray button pressed, waiting for 1 second...........");
    }

    if(blackButtonPressed){
        if((currentTime - lastBlackPressTime).toSec() <= 1.0){
            if(grayButton == 1 && !finishChoose){
                deviceData.mode = 3;
                ROS_INFO("Current Mode: %d", deviceData.mode);
                finishChoose = true;
            }
        } else {
            if(!finishChoose){
                deviceData.mode = 1;
                ROS_INFO("Current Mode: %d", deviceData.mode);
                finishChoose = true;
            }
            blackButtonPressed = false; 
        }
    }

    if(grayButtonPressed){
        if((currentTime - lastGrayPressTime).toSec() <= 1.0){
            if(blackButton == 1 && !finishChoose){
                deviceData.mode = 3;
                ROS_INFO("Current Mode: %d", deviceData.mode);
                finishChoose = true;
            }
        } else {
            if(!finishChoose){
                deviceData.mode = 2;
                ROS_INFO("Current Mode: %d", deviceData.mode);
                finishChoose = true;
            }
            grayButtonPressed = false; 
        }
    }

    // ROS_INFO("Current Mode: %d", deviceData.mode);
    // ROS_INFO("Receive4");
}

