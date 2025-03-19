/**
 ***********************************************************************************************************************
 *
 * @author ZhangRan
 * @date   2021/08/07
 *
 * <h2><center>&copy; COPYRIGHT 2021 Dobot CORPORATION</center></h2>
 *
 ***********************************************************************************************************************
 */

#include <ros/ros.h>
#include <ros/param.h>
#include <dobot_v4_bringup/cr5_v4_robot.h>
#include <sensor_msgs/JointState.h>
#include <dobot_v4_bringup/ToolVectorActual.h>
#include <iostream>
#include <string>
#include <stdio.h>
#include <stdlib.h>

using namespace std;
int main(int argc, char* argv[])
{
    ros::init(argc, argv, "CRRobot");

    try
    {
        ros::NodeHandle node;   //全局命名空间的句柄
        ros::NodeHandle private_node("~");    //私有命名空间的句柄

        ros::AsyncSpinner async_spinner(1);    //使用异步spinner来处理回调函数，使节点可以在后台执行，提高效率
        async_spinner.start();

        sensor_msgs::JointState joint_state_msg;     //创建JointState消息对象，用于存储机械臂关节状态信息，这里注意，Touch上发布关节角度也是用的这个消息类型。
        ros::Publisher joint_state_pub = private_node.advertise<sensor_msgs::JointState>("/joint_states", 100);   //将信息发布到/joint_states话题下。

        dobot_v4_bringup::RobotStatus robot_status_msg;  //创建RobotStatus消息，这是自己在msg中定义的
        ros::Publisher robot_status_pub = private_node.advertise<dobot_v4_bringup::RobotStatus>("/dobot_v4_bringup/msg/RobotStatus", 100);  //通过/dobot_v4_bringup/msg/RobotStatus发布

        dobot_v4_bringup::ToolVectorActual tool_vector_actual_msg;   //创建ToolVectorActual消息，这是自己在msg中定义的
        ros::Publisher tool_vector_pub =                             //通过/dobot_v4_bringup/msg/ToolVectorActual发布
            private_node.advertise<dobot_v4_bringup::ToolVectorActual>("/dobot_v4_bringup/msg/ToolVectorActual", 100);

        string z ="/";                                               
        const char* robot_type = getenv("DOBOT_TYPE");                //从环境变量中获取机械臂类型           
        string a = robot_type == nullptr ? "cr5" : robot_type;        //如果类型并未被设置，则默认值为cr5
        string b = "_robot/joint_controller/follow_joint_trajectory"; 
        string ss =  z + a + b ;                                      //生成控制器话题的完整路径，并存储。例如"/cr10_robot/joint_controller/follow_joint_trajectory"
        for (uint32_t i = 0; i < 6; i++)                             //初始化joint_state_msg（sensor_msgs::JointState类型）初始值为0.0，名称从joint1~6
        {
            joint_state_msg.position.push_back(0.0);
            joint_state_msg.name.push_back(std::string("joint") + std::to_string(i + 1));
        }

        CRRobot robot(private_node, ss);                       //初始化，ss为路径，CRRobot是一个控制类，用于初始化和控制机械臂

        double rate_vale = private_node.param("JointStatePublishRate", 10);      //发布频率为10HZ，可以通过设置参数服务器中的JointStatePublishRate来修改

        robot.init();                                           //进行初始化，设置rate用于控制主循环频率
        ros::Rate rate(rate_vale);
        double position[6];                                     //定义position用于存放关节状态
        while (ros::ok())
        {
            //
            // publish joint state
            //
            robot.getJointState(position);                       //获取机械臂关节位置，并存储在position中
            joint_state_msg.header.stamp = ros::Time::now();     //设置时间戳
            joint_state_msg.header.frame_id = "dummy_link";      //指定参考坐标系
            for (uint32_t i = 0; i < 6; i++)                     //将关节信息更新到消息中，并发布，使其他节点能够接收更新到的关节位置
                joint_state_msg.position[i] = position[i];
            joint_state_pub.publish(joint_state_msg);

            double val[6];
            robot.getToolVectorActual(val);                      //获取机械臂末端工具的实际位姿，分别是xyz坐标和绕xyz三轴的旋转，并发布
            tool_vector_actual_msg.x = val[0];
            tool_vector_actual_msg.y = val[1];
            tool_vector_actual_msg.z = val[2];
            tool_vector_actual_msg.rx = val[3];
            tool_vector_actual_msg.ry = val[4];
            tool_vector_actual_msg.rz = val[5];
            tool_vector_pub.publish(tool_vector_actual_msg);

            //
            // publish robot status
            //
            robot_status_msg.is_enable = robot.isEnable();      //通过robot_status_msg发布机械臂当前状态
            robot_status_msg.is_connected = robot.isConnected();
            robot_status_pub.publish(robot_status_msg);

            rate.sleep();
        }
    }
    catch (const std::exception& err)
    {
        ROS_ERROR("%s", err.what());
        return -1;
    }

    return 0;
}
