
#pragma once
#include "ros/ros.h"
#include <thread>
#include <nlohmann/json.hpp>
#include <std_msgs/String.h>
#include <memory>
#include <mutex>
#include <fstream>
#include <vector>
#include "arm_control_tcp/EnableRobot.h"
#include "arm_control_tcp/DisableRobot.h"
#include "arm_control_tcp/ClearError.h"
#include "arm_control_tcp/GetErrorID.h"
#include "arm_control_tcp/MovJ.h"
#include "arm_control_tcp/ServoJ.h"

class RosDemoCRV4
{
public:
    RosDemoCRV4(ros::NodeHandle* nh);
    void movePoint(std::vector<double>& pointA, int& id);
    void moveJoint(std::vector<double>& pointA, int& id);
    void finishPoint(int id);

private:
    ros::ServiceClient m_enableRobot;
    ros::ServiceClient m_disableRobot;
    ros::ServiceClient m_clearError;
    ros::ServiceClient m_getErrorID;
    ros::ServiceClient m_movj;
    ros::ServiceClient m_servoj;
    ros::Subscriber subFeedInfo;
    std::mutex m_mutex;
    std::thread threadClearRobotError;
    std::thread threadParseRobotError;
    struct FeedInfo
    {
        int EnableStatus;
        int ErrorStatus;
        int RobotMode;
        int CurrentCommandID;
    };
    FeedInfo feedbackData;
    bool stateFinish{ false };
    nlohmann::json m_JsonDataController;
    nlohmann::json m_JsonDataServo;

private:
    void warmRobotError();
    void getFeedBackInfo(const std_msgs::String::ConstPtr& msg);

    template <typename T>
    bool SendService(ros::ServiceClient serviceClient, T& arg);
    void parseRobotAlarm();
    // Add more service servers if needed
};

// 模板函数 发送service服务
template <typename T>
bool RosDemoCRV4::SendService(ros::ServiceClient serviceClient, T& arg)
{
    // 请求服务
    if (serviceClient.call(arg)) {
        return true;
    }
    return false;
}