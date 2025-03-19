/**
 ***********************************************************************************************************************
 *
 * @author YangXiBo
 * @date   2023/08/18
 *
 * <h2><center>&copy; COPYRIGHT 2021 DOBOT CORPORATION</center></h2>
 *
 ***********************************************************************************************************************
 */

// GetToolDo is not definded

#include <ros/ros.h>
#include <ros/param.h>
#include <dobot_v4_bringup/cr5_v4_robot.h>

// #include <dobot_v4_bringup/parseTool.h>

/**
 ***********************************************************************************************************************
    CCRobot类的构造函数，继承自ActionServer<FollowJointTrajectoryAction>
 ***********************************************************************************************************************
 */
CRRobot::CRRobot(ros::NodeHandle& nh, std::string name)
    : ActionServer<FollowJointTrajectoryAction>(nh, std::move(name), false)
    , goal_{}
    , control_nh_(nh)
    , trajectory_duration_(1.0)
{
    index_ = 0;
    memset(goal_, 0, sizeof(goal_));
}

/**
 ***********************************************************************************************************************
    CCRobot类的析构函数
 ***********************************************************************************************************************
 */

CRRobot::~CRRobot()
{
    ROS_INFO("~CRRobot");
}

/**
 ***********************************************************************************************************************
    CCRobot类的初始化函数
 ***********************************************************************************************************************
 */

void CRRobot::init()
{
    std::string ip = control_nh_.param<std::string>("robot_ip_address", "192.168.5.1");
    trajectory_duration_ = control_nh_.param("trajectory_duration", 0.3);
    ROS_INFO("trajectory_duration : %0.2f", trajectory_duration_);

    int numRobotNodes = control_nh_.param("num_nodes", 1);
    std::string serviceRobotName{ "" };
    std::string serviceProjectName{ "/dobot_v4_bringup/" };
    if (numRobotNodes > 1) {
        serviceRobotName = control_nh_.param<std::string>("robot_node_name", "robot");
        serviceRobotName = serviceRobotName + "/";
    };

    std::string serviceEnableRobot = serviceProjectName + serviceRobotName + "srv/EnableRobot";
    std::string serviceDisableRobot = serviceProjectName + serviceRobotName + "srv/DisableRobot";
    std::string serviceClearError = serviceProjectName + serviceRobotName + "srv/ClearError";
    std::string serviceSpeedFactor = serviceProjectName + serviceRobotName + "srv/SpeedFactor";
    std::string serviceUser = serviceProjectName + serviceRobotName + "srv/User";
    std::string serviceTool = serviceProjectName + serviceRobotName + "srv/Tool";
    std::string serviceRobotMode = serviceProjectName + serviceRobotName + "srv/RobotMode";
    std::string serviceSetPayLoad = serviceProjectName + serviceRobotName + "srv/SetPayLoad";
    std::string serviceDO = serviceProjectName + serviceRobotName + "srv/DO";
    std::string serviceDOInstant = serviceProjectName + serviceRobotName + "srv/DOInstant";
    std::string serviceToolDO = serviceProjectName + serviceRobotName + "srv/ToolDO";
    std::string serviceToolDOInstant = serviceProjectName + serviceRobotName + "srv/ToolDOInstant";
    std::string serviceAO = serviceProjectName + serviceRobotName + "srv/AO";
    std::string serviceAOInstant = serviceProjectName + serviceRobotName + "srv/AOInstant";
    std::string serviceAccJ = serviceProjectName + serviceRobotName + "srv/AccJ";
    std::string serviceAccL = serviceProjectName + serviceRobotName + "srv/AccL";
    std::string serviceVelJ = serviceProjectName + serviceRobotName + "srv/VelJ";
    std::string serviceVelL = serviceProjectName + serviceRobotName + "srv/VelL";
    std::string serviceCP = serviceProjectName + serviceRobotName + "srv/CP";
    std::string servicePowerOn = serviceProjectName + serviceRobotName + "srv/PowerOn";
    std::string serviceRunScript = serviceProjectName + serviceRobotName + "srv/RunScript";
    std::string serviceStop = serviceProjectName + serviceRobotName + "srv/Stop";
    std::string servicePause = serviceProjectName + serviceRobotName + "srv/Pause";
    std::string serviceContinue = serviceProjectName + serviceRobotName + "srv/Continue";

    std::string serviceEnableSafeSkin = serviceProjectName + serviceRobotName + "srv/EnableSafeSkin";
    std::string serviceSetSafeSkin = serviceProjectName + serviceRobotName + "srv/SetSafeSkin";
    std::string serviceGetStartPose = serviceProjectName + serviceRobotName + "srv/GetStartPose";
    std::string serviceStartPath = serviceProjectName + serviceRobotName + "srv/StartPatht";
    std::string servicePositiveKin = serviceProjectName + serviceRobotName + "srv/PositiveKin";
    std::string serviceInverseKin = serviceProjectName + serviceRobotName + "srv/InverseKin";
    std::string serviceGetAngle = serviceProjectName + serviceRobotName + "srv/GetAngle";
    std::string serviceGetPose = serviceProjectName + serviceRobotName + "srv/GetPose";
    std::string serviceSetCollisionLevel = serviceProjectName + serviceRobotName + "srv/SetCollisionLevel";
    std::string serviceEmergencyStop = serviceProjectName + serviceRobotName + "srv/EmergencyStop";
    std::string serviceModbusRTUCreate = serviceProjectName + serviceRobotName + "srv/ModbusRTUCreate";
    std::string serviceModbusCreate = serviceProjectName + serviceRobotName + "srv/ModbusCreate";
    std::string serviceModbusClose = serviceProjectName + serviceRobotName + "srv/ModbusClose";
    std::string serviceGetInBits = serviceProjectName + serviceRobotName + "srv/GetInBits";
    std::string serviceGetInRegs = serviceProjectName + serviceRobotName + "srv/GetInRegs";
    std::string serviceGetCoils = serviceProjectName + serviceRobotName + "srv/GetCoils";
    std::string serviceSetCoils = serviceProjectName + serviceRobotName + "srv/SetCoils";
    std::string serviceGetHoldRegs = serviceProjectName + serviceRobotName + "srv/GetHoldRegs";
    std::string serviceSetHoldRegs = serviceProjectName + serviceRobotName + "srv/SetHoldRegs";

    std::string serviceGetErrorID = serviceProjectName + serviceRobotName + "srv/GetErrorID";
    std::string serviceDI = serviceProjectName + serviceRobotName + "srv/DI";
    std::string serviceToolDI = serviceProjectName + serviceRobotName + "srv/ToolDI";
    std::string serviceAI = serviceProjectName + serviceRobotName + "srv/AI";
    std::string serviceToolAI = serviceProjectName + serviceRobotName + "srv/ToolAI";
    std::string serviceDIGroup = serviceProjectName + serviceRobotName + "srv/DIGroup";
    std::string serviceDOGroup = serviceProjectName + serviceRobotName + "srv/DoGroup";
    std::string serviceBrakeControl = serviceProjectName + serviceRobotName + "srv/BrakeControl";
    std::string serviceStartDrag = serviceProjectName + serviceRobotName + "srv/StartDrag";
    std::string serviceStopDrag = serviceProjectName + serviceRobotName + "srv/StopDrag";
    std::string serviceDragSensivity = serviceProjectName + serviceRobotName + "srv/DragSensivity";
    std::string serviceGetDO = serviceProjectName + serviceRobotName + "srv/GetDO";
    std::string serviceGetAO = serviceProjectName + serviceRobotName + "srv/GetAO";
    std::string serviceGetDOGroup = serviceProjectName + serviceRobotName + "srv/GetDOGroup";
    std::string serviceSetTool485 = serviceProjectName + serviceRobotName + "srv/SetTool485";
    std::string serviceSetSafeWallEnable = serviceProjectName + serviceRobotName + "srv/SetSafeWallEnable";
    std::string serviceSetToolPower = serviceProjectName + serviceRobotName + "srv/SetToolPower";
    std::string serviceSetToolMode = serviceProjectName + serviceRobotName + "srv/SetToolMode";
    std::string serviceSetBackDistance = serviceProjectName + serviceRobotName + "srv/SetBackDistance";
    std::string serviceSetPostCollisionMode = serviceProjectName + serviceRobotName + "srv/SetPostCollisionMode";
    std::string serviceSetUser = serviceProjectName + serviceRobotName + "srv/SetUser";
    std::string serviceSetTool = serviceProjectName + serviceRobotName + "srv/SetTool";
    std::string serviceCalcUser = serviceProjectName + serviceRobotName + "srv/CalcUser";
    std::string serviceCalcTool = serviceProjectName + serviceRobotName + "srv/CalcTool";
    std::string serviceGetInputBool = serviceProjectName + serviceRobotName + "srv/GetInputBool";
    std::string serviceGetInputInt = serviceProjectName + serviceRobotName + "srv/GetInputInt";
    std::string serviceGetInputFloat = serviceProjectName + serviceRobotName + "srv/GetInputFloat";
    std::string serviceGetOutputBool = serviceProjectName + serviceRobotName + "srv/GetOutputBool";
    std::string serviceGetOutputInt = serviceProjectName + serviceRobotName + "srv/GetOutputInt";
    std::string serviceGetOutputFloat = serviceProjectName + serviceRobotName + "srv/GetOutputFloat";
    std::string serviceSetOutputBool = serviceProjectName + serviceRobotName + "srv/SetOutputBool";
    std::string serviceSetOutputInt = serviceProjectName + serviceRobotName + "srv/SetOutputInt";
    std::string serviceSetOutputFloat = serviceProjectName + serviceRobotName + "srv/SetOutputFloat";

    std::string serviceMovJ = serviceProjectName + serviceRobotName + "srv/MovJ";
    std::string serviceMovL = serviceProjectName + serviceRobotName + "srv/MovL";
    std::string serviceMovLIO = serviceProjectName + serviceRobotName + "srv/MovLIO";
    std::string serviceMovJIO = serviceProjectName + serviceRobotName + "srv/MovJIO";
    std::string serviceArc = serviceProjectName + serviceRobotName + "srv/Arc";
    std::string serviceCircle = serviceProjectName + serviceRobotName + "srv/Circle";
    std::string serviceMoveJog = serviceProjectName + serviceRobotName + "srv/MoveJog";
    std::string serviceStopMoveJog = serviceProjectName + serviceRobotName + "srv/StopMoveJog";
    std::string serviceRelMovJTool = serviceProjectName + serviceRobotName + "srv/RelMovJTool";
    std::string serviceRelMovLTool = serviceProjectName + serviceRobotName + "srv/RelMovLTool";
    std::string serviceRelMovJUser = serviceProjectName + serviceRobotName + "srv/RelMovJUser";
    std::string serviceRelMovLUser = serviceProjectName + serviceRobotName + "srv/RelMovLUser";
    std::string serviceRelJointMovJ = serviceProjectName + serviceRobotName + "srv/RelJointMovJ";
    std::string serviceGetCurrentCommandId = serviceProjectName + serviceRobotName + "srv/GetCurrentCommandId";
    std::string serviceServoJ = serviceProjectName + serviceRobotName + "srv/ServoJ";
    std::string serviceServoP = serviceProjectName + serviceRobotName + "srv/ServoP";
    std::string topicFeedInfo = serviceProjectName + serviceRobotName + "/msg/FeedInfo";

    commander_ = std::make_shared<CR5Commander>(ip);
    commander_->init();
    server_tbl_.push_back(control_nh_.advertiseService(serviceEnableRobot, &CRRobot::enableRobot, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceDisableRobot, &CRRobot::disableRobot, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceClearError, &CRRobot::clearError, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceSpeedFactor, &CRRobot::speedFactor, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceUser, &CRRobot::user, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceTool, &CRRobot::tool, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceRobotMode, &CRRobot::robotMode, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceSetPayLoad, &CRRobot::setPayload, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceDO, &CRRobot::DO, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceDOInstant, &CRRobot::DOInstant, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceToolDO, &CRRobot::toolDO, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceToolDOInstant, &CRRobot::toolDOInstant, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceAO, &CRRobot::AO, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceAOInstant, &CRRobot::AOInstant, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceAccJ, &CRRobot::accJ, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceAccL, &CRRobot::accL, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceVelJ, &CRRobot::velJ, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceVelL, &CRRobot::velL, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceCP, &CRRobot::cp, this));
    server_tbl_.push_back(control_nh_.advertiseService(servicePowerOn, &CRRobot::powerOn, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceRunScript, &CRRobot::runScript, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceStop, &CRRobot::stop, this));
    server_tbl_.push_back(control_nh_.advertiseService(servicePause, &CRRobot::pause, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceContinue, &CRRobot::Continue, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceEnableSafeSkin, &CRRobot::enableSafeSkin, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceSetSafeSkin, &CRRobot::setSafeSkin, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceGetStartPose, &CRRobot::getStartPose, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceStartPath, &CRRobot::startPath, this));
    server_tbl_.push_back(control_nh_.advertiseService(servicePositiveKin, &CRRobot::positiveKin, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceInverseKin, &CRRobot::inverseKin, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceGetAngle, &CRRobot::getAngle, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceGetPose, &CRRobot::getPose, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceSetCollisionLevel, &CRRobot::setCollisionLevel, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceEmergencyStop, &CRRobot::emergencyStop, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceModbusRTUCreate, &CRRobot::modbusRTUCreate, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceModbusCreate, &CRRobot::modbusCreate, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceModbusClose, &CRRobot::modbusClose, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceGetInBits, &CRRobot::getInBits, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceGetInRegs, &CRRobot::getInRegs, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceGetCoils, &CRRobot::getCoils, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceSetCoils, &CRRobot::setCoils, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceGetHoldRegs, &CRRobot::getHoldRegs, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceSetHoldRegs, &CRRobot::setHoldRegs, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceGetErrorID, &CRRobot::getErrorID, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceDI, &CRRobot::DI, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceToolDI, &CRRobot::toolDI, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceAI, &CRRobot::AI, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceToolAI, &CRRobot::toolAI, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceDIGroup, &CRRobot::DIGroup, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceDOGroup, &CRRobot::doGroup, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceBrakeControl, &CRRobot::brakeControl, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceStartDrag, &CRRobot::startDrag, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceStopDrag, &CRRobot::stopDrag, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceDragSensivity, &CRRobot::dragSensivity, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceGetDO, &CRRobot::getDO, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceGetAO, &CRRobot::getAO, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceGetDOGroup, &CRRobot::getDOGroup, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceSetTool485, &CRRobot::setTool485, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceSetSafeWallEnable, &CRRobot::setSafeWallEnable, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceSetToolPower, &CRRobot::setToolPower, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceSetToolMode, &CRRobot::setToolMode, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceSetBackDistance, &CRRobot::setBackDistance, this));
    server_tbl_.push_back(
        control_nh_.advertiseService(serviceSetPostCollisionMode, &CRRobot::setPostCollisionMode, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceSetUser, &CRRobot::setUser, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceSetTool, &CRRobot::setTool, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceCalcUser, &CRRobot::calcUser, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceCalcTool, &CRRobot::calcTool, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceGetInputBool, &CRRobot::getInputBool, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceGetInputInt, &CRRobot::getInputInt, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceGetInputFloat, &CRRobot::getInputFloat, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceGetOutputBool, &CRRobot::getOutputBool, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceGetOutputInt, &CRRobot::getOutputInt, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceGetOutputFloat, &CRRobot::getOutputFloat, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceSetOutputBool, &CRRobot::setOutputBool, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceSetOutputInt, &CRRobot::setOutputInt, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceSetOutputFloat, &CRRobot::setOutputFloat, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceMovJ, &CRRobot::movJ, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceMovL, &CRRobot::movL, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceMovLIO, &CRRobot::movLIO, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceMovJIO, &CRRobot::movJIO, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceArc, &CRRobot::arc, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceCircle, &CRRobot::circle, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceMoveJog, &CRRobot::moveJog, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceStopMoveJog, &CRRobot::stopmoveJog, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceRelMovJTool, &CRRobot::relMovJTool, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceRelMovLTool, &CRRobot::relMovLTool, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceRelMovJUser, &CRRobot::relMovJUser, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceRelMovLUser, &CRRobot::relMovLUser, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceRelJointMovJ, &CRRobot::relJointMovJ, this));
    server_tbl_.push_back(
        control_nh_.advertiseService(serviceGetCurrentCommandId, &CRRobot::getCurrentCommandId, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceServoJ, &CRRobot::servoJ, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceServoP, &CRRobot::servoP, this));

    registerGoalCallback(boost::bind(&CRRobot::goalHandle, this, _1));
    registerCancelCallback(boost::bind(&CRRobot::cancelHandle, this, _1));
    backend_task_ = control_nh_.createTimer(ros::Duration(1.5), &CRRobot::backendTask, this);

    pubFeedInfo = control_nh_.advertise<std_msgs::String>(topicFeedInfo, 1000);
    threadPubFeedBackInfo = std::thread(&CRRobot::pubFeedBackInfo, this);
    threadPubFeedBackInfo.detach();
    start();
}

/**
 ***********************************************************************************************************************
    用于实时收集机械臂的各种状态信息并将其以JSON格式发布到ros话题上。
 ***********************************************************************************************************************
 */

void CRRobot::pubFeedBackInfo()
{
    RealTimeData* realTimeData = nullptr;
    // 设置发布频率为10Hz
    ros::Rate rate(100);
    while (ros::ok()) {                                                                  //主循环
        realTimeData = (const_cast<RealTimeData*>(commander_->getRealData()));           //获取实时数据并赋值给realTimeData，const_cast是去除常量限定符，允许realTimeData被修改
        nlohmann::json root;                                                             //创建json对象，将获取到的实时状态添加到json对象root中，下面是一系列的赋值过程
        root["len"] = realTimeData->len;
        root["digital_input_bits"] = realTimeData->digital_input_bits;
        root["digital_outputs"] = realTimeData->digital_outputs;
        root["robot_mode"] = realTimeData->robot_mode;
        root["controller_timer"] = realTimeData->controller_timer;
        root["run_time"] = realTimeData->run_time;
        root["test_value"] = realTimeData->test_value;
        root["safety_mode"] = realTimeData->safety_mode;
        root["speed_scaling"] = realTimeData->speed_scaling;
        root["linear_momentum_norm"] = realTimeData->linear_momentum_norm;
        root["v_main"] = realTimeData->v_main;
        root["v_robot"] = realTimeData->v_robot;
        root["i_robot"] = realTimeData->i_robot;
        root["program_state"] = realTimeData->program_state;
        root["safety_status"] = realTimeData->safety_status;

        std::vector<double> vecTransit;                                                  // vector 中转存取数组类型。针对realTimeData中的数组数据，如关节位置、速度、目标角度等
        for (int i = 0; i < 3; i++) {                                                    // 使用vecTransit作为中转将数据添加到json中
            vecTransit.push_back(realTimeData->tool_accelerometer_values[i]);
        }
        root["tool_accelerometer_values"] = vecTransit;
        vecTransit.clear();

        for (int i = 0; i < 3; i++) {
            vecTransit.push_back(realTimeData->elbow_position[i]);
        }
        root["elbow_position"] = vecTransit;
        vecTransit.clear();

        for (int i = 0; i < 3; i++) {
            vecTransit.push_back(realTimeData->elbow_velocity[i]);
        }
        root["elbow_velocity"] = vecTransit;
        vecTransit.clear();

        for (int i = 0; i < 6; i++) {
            vecTransit.push_back(realTimeData->q_target[i]);
        }
        root["q_target"] = vecTransit;
        vecTransit.clear();

        for (int i = 0; i < 6; i++) {
            vecTransit.push_back(realTimeData->qd_target[i]);
        }
        root["qd_target"] = vecTransit;
        vecTransit.clear();

        for (int i = 0; i < 6; i++) {
            vecTransit.push_back(realTimeData->qdd_target[i]);
        }
        root["qdd_target"] = vecTransit;
        vecTransit.clear();

        for (int i = 0; i < 6; i++) {
            vecTransit.push_back(realTimeData->i_target[i]);
        }
        root["i_target"] = vecTransit;
        vecTransit.clear();

        for (int i = 0; i < 6; i++) {
            vecTransit.push_back(realTimeData->m_target[i]);
        }
        root["m_target"] = vecTransit;
        vecTransit.clear();

        for (int i = 0; i < 6; i++) {
            vecTransit.push_back(realTimeData->q_actual[i]);
        }
        root["q_actual"] = vecTransit;
        vecTransit.clear();

        for (int i = 0; i < 6; i++) {
            vecTransit.push_back(realTimeData->qd_actual[i]);
        }
        root["qd_actual"] = vecTransit;
        vecTransit.clear();

        for (int i = 0; i < 6; i++) {
            vecTransit.push_back(realTimeData->i_actual[i]);
        }
        root["i_actual"] = vecTransit;
        vecTransit.clear();

        for (int i = 0; i < 6; i++) {
            vecTransit.push_back(realTimeData->i_control[i]);
        }
        root["i_control"] = vecTransit;
        vecTransit.clear();

        for (int i = 0; i < 6; i++) {
            vecTransit.push_back(realTimeData->tool_vector_actual[i]);
        }
        root["tool_vector_actual"] = vecTransit;
        vecTransit.clear();

        for (int i = 0; i < 6; i++) {
            vecTransit.push_back(realTimeData->TCP_speed_actual[i]);
        }
        root["TCP_speed_actual"] = vecTransit;
        vecTransit.clear();

        for (int i = 0; i < 6; i++) {
            vecTransit.push_back(realTimeData->TCP_force[i]);
        }
        root["TCP_force"] = vecTransit;
        vecTransit.clear();

        for (int i = 0; i < 6; i++) {
            vecTransit.push_back(realTimeData->tool_vector_target[i]);
        }
        root["tool_vector_target"] = vecTransit;
        vecTransit.clear();

        for (int i = 0; i < 6; i++) {
            vecTransit.push_back(realTimeData->TCP_speed_target[i]);
        }
        root["TCP_speed_target"] = vecTransit;
        vecTransit.clear();

        for (int i = 0; i < 6; i++) {
            vecTransit.push_back(realTimeData->motor_temperatures[i]);
        }
        root["motor_temperatures"] = vecTransit;
        vecTransit.clear();

        for (int i = 0; i < 6; i++) {
            vecTransit.push_back(realTimeData->joint_modes[i]);
        }
        root["joint_modes"] = vecTransit;
        vecTransit.clear();

        for (int i = 0; i < 6; i++) {
            vecTransit.push_back(realTimeData->v_actual[i]);
        }
        root["v_actual"] = vecTransit;
        vecTransit.clear();

        for (int i = 0; i < 4; i++) {
            vecTransit.push_back(realTimeData->handtype[i]);
        }
        root["handtype"] = vecTransit;
        vecTransit.clear();

        root["userCoordinate"] = realTimeData->userCoordinate;
        root["toolCoordinate"] = realTimeData->toolCoordinate;
        root["isRunQueuedCmd"] = realTimeData->isRunQueuedCmd;
        root["isPauseCmdFlag"] = realTimeData->isPauseCmdFlag;
        root["velocityRatio"] = realTimeData->velocityRatio;
        root["accelerationRatio"] = realTimeData->accelerationRatio;
        root["jerkRatio"] = realTimeData->jerkRatio;
        root["xyzVelocityRatio"] = realTimeData->xyzVelocityRatio;
        root["rVelocityRatio"] = realTimeData->rVelocityRatio;
        root["xyzAccelerationRatio"] = realTimeData->xyzAccelerationRatio;
        root["rAccelerationRatio"] = realTimeData->rAccelerationRatio;
        root["xyzJerkRatio"] = realTimeData->xyzJerkRatio;
        root["rJerkRatio"] = realTimeData->rJerkRatio;
        root["BrakeStatus"] = realTimeData->BrakeStatus;
        root["EnableStatus"] = realTimeData->EnableStatus;
        root["DragStatus"] = realTimeData->DragStatus;
        root["RunningStatus"] = realTimeData->RunningStatus;
        root["ErrorStatus"] = realTimeData->ErrorStatus;
        root["JogStatus"] = realTimeData->JogStatus;
        root["RobotType"] = realTimeData->RobotType;
        root["DragButtonSignal"] = realTimeData->DragButtonSignal;
        root["EnableButtonSignal"] = realTimeData->EnableButtonSignal;
        root["RecordButtonSignal"] = realTimeData->RecordButtonSignal;
        root["ReappearButtonSignal"] = realTimeData->ReappearButtonSignal;
        root["JawButtonSignal"] = realTimeData->JawButtonSignal;
        root["SixForceOnline"] = realTimeData->SixForceOnline;
        root["CollisionStates"] = realTimeData->CollisionStates;
        root["ArmApproachState"] = realTimeData->ArmApproachState;
        root["J4ApproachState"] = realTimeData->J4ApproachState;
        root["J5ApproachState"] = realTimeData->J5ApproachState;
        root["J6ApproachState"] = realTimeData->J6ApproachState;
        root["vibrationDisZ"] = realTimeData->vibrationDisZ;
        root["currentCommandId"] = realTimeData->currentCommandId;

        for (int i = 0; i < 6; i++) {
            vecTransit.push_back(realTimeData->m_actual[i]);
        }
        root["m_actual"] = vecTransit;
        vecTransit.clear();

        root["load"] = realTimeData->load;
        root["centerX"] = realTimeData->centerX;
        root["centerY"] = realTimeData->centerY;
        root["centerZ"] = realTimeData->centerZ;

        for (int i = 0; i < 6; i++) {
            vecTransit.push_back(realTimeData->user[i]);
        }
        root["user"] = vecTransit;
        vecTransit.clear();

        for (int i = 0; i < 6; i++) {
            vecTransit.push_back(realTimeData->tool[i]);
        }
        root["tool"] = vecTransit;
        vecTransit.clear();

        root["TraceIndex"] = realTimeData->TraceIndex;    // 1296 ~ 1303 轨迹复现索引 （未实现）

        for (int i = 0; i < 6; i++) {
            vecTransit.push_back(realTimeData->SixForceValue[i]);
        }
        root["SixForceValue"] = vecTransit;
        vecTransit.clear();

        for (int i = 0; i < 4; i++) {
            vecTransit.push_back(realTimeData->TargetQuaternion[i]);
        }
        root["TargetQuaternion"] = vecTransit;
        vecTransit.clear();

        for (int i = 0; i < 4; i++) {
            vecTransit.push_back(realTimeData->ActualQuaternion[i]);
        }
        root["ActualQuaternion"] = vecTransit;
        vecTransit.clear();
        root["AutoManualMode"] = realTimeData->AutoManualMode;                          // 1416 ~ 1417 手自动模式 0: 未开启 1: manual-手动 2:auto-自动

        std::string feedBackVecStr = root.dump();                                       //将root转换为字符串feedBackVecStr

        std_msgs::String msgFeedInfo;                                                   //创建std_msgs::String消息，并发布。发布在话题pubFeedInfo上
        msgFeedInfo.data = feedBackVecStr;
        pubFeedInfo.publish(msgFeedInfo);
        rate.sleep();
    }
}

/**
 ***********************************************************************************************************************
    用于生成并发布机械臂在执行轨迹过程中实时的反馈信息。将当前关节状态与目标状态对比，并反馈给客户端，这里用的是服务通信方式。
 ***********************************************************************************************************************
 */

void CRRobot::feedbackHandle(const ros::TimerEvent& tm,                                                   //tm：ros::TimerEvent 对象，表示触发此反馈处理函数的定时事件。
                             ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle handle)  //handle：GoalHandle 类型，用于管理和控制当前的目标（即客户端发送的 FollowJointTrajectoryAction 目标）。通过它可以发送反馈信息给客户端。
{
    control_msgs::FollowJointTrajectoryFeedback feedback;                                //创建一个 FollowJointTrajectoryFeedback 类型的对象 feedback。这个消息类型用于表示轨迹跟踪的实时反馈，包括机械臂当前的关节状态和目标状态。

    double current_joints[6];
    getJointState(current_joints);                                                       //通过getJointState获取机械臂当前关节位置，并存放在current_joints数组中，六个关节。

    for (uint32_t i = 0; i < 6; i++) {                                                   //遍历每个关节，添加名称，并将当前关节位置添加到反馈 feedback 的实时位置列表中，将目标角度goal_[i]放到feedback的目标位置列表中。
        feedback.joint_names.push_back(std::string("joint") + std::to_string(i + 1));
        feedback.actual.positions.push_back(current_joints[i]);
        feedback.desired.positions.push_back(goal_[i]);
    }

    handle.publishFeedback(feedback);                                                     //通过handle将feedback发布给客户端，客户端可以用该信息监控轨迹执行中的实时状态，了解机械臂实际的位置和目标位置的差异
}

/**
 ***********************************************************************************************************************
    用于在两个轨迹点之间进行插值，生成机械臂关节的中间状态。通过三次多项式插值实现。
 ***********************************************************************************************************************
 */

std::vector<double> CRRobot::sample_traj(const trajectory_msgs::JointTrajectoryPoint& P0, //P0：第一个轨迹点  P1：第二个轨迹点   time_index：插值时的时间点   返回值是一个vector,放time_index时刻各关节的位置
                                         const trajectory_msgs::JointTrajectoryPoint& P1, const double& time_index)
{
    double a, b, c, d;
    double T = P1.time_from_start.toSec() - P0.time_from_start.toSec();                   //时间间隔
    double t = time_index;
    std::vector<double> interp_traj;
    for (int i = 0; i < P0.positions.size(); i++) {                                       //三次多项式插值计算
        a = P0.positions[i];
        b = P0.velocities[i];
        c = (-3.0 * P0.positions[i] + 3.0 * P1.positions[i] - 2.0 * T * P0.velocities[i] - T * P1.velocities[i]) /
            (T * T);
        d = (2.0 * P0.positions[i] - 2.0 * P1.positions[i] + T * P0.velocities[i] + T * P1.velocities[i]) / (T * T * T);
        interp_traj.push_back((a + b * t + c * t * t + d * t * t * t) * 180.0 / M_PI);
    }
    return interp_traj;
}

/**
 ***********************************************************************************************************************
    用于在接收到FollowJointTrajectoryAction后控制机械臂按照指定的轨迹运动。通过分段插值计算每个关节在轨迹中的位置，并使用servoj指令逐步移动关节，以实现平滑的轨迹控制
 ***********************************************************************************************************************
 */

void CRRobot::moveHandle(const ros::TimerEvent& tm,                                        //同上，是个定时事件，用于触发这个函数
                         ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle handle)     //handle 是一个GoalHandle对象，用于管理和控制当前的轨迹跟踪目标，包括发布反馈和完成状态
{
    control_msgs::FollowJointTrajectoryGoalConstPtr goal = handle.getGoal();               //获取目标轨迹handle.getGoal()

    static const double SERVOJ_DURATION = 0.4;                                             //表示每个servoj指令的持续时间为0.4s
    double t = SERVOJ_DURATION * 1.5;                                                      //t为插值的时间控制参数，下面sprintf中用到
    ros::Rate timer(1.0 / SERVOJ_DURATION);                                                // servoj发布频率，持续时间的倒数
    double t0 = ros::Time::now().toSec();

    try {
        for (int i = 0; i < goal->trajectory.points.size() - 1; i++) {                     //遍历轨迹中的每个点 goal->trajectory.points.size()
            trajectory_msgs::JointTrajectoryPoint interp_traj_begin = goal->trajectory.points[i];   //获取当前点和下一个点
            trajectory_msgs::JointTrajectoryPoint interp_traj_end = goal->trajectory.points[i + 1];
            double real_time;                                                              // 实际间隔时间
            double t1;
            t1 = ros::Time::now().toSec();
            real_time = t1 - t0;                                                             //t1是当前时间，t0是函数开始的时间，real_time是函数开始以来过去的时间，interp_traj_end.time_from_start表示interp_traj_end这个目标点相对于轨迹开始的时间
                                                                                             //表示当前时间 real_time 必须小于 interp_traj_end 的目标时间减去 SERVOJ_DURATION。
                                                                                             //换句话说，机械臂需要在距离 interp_traj_end 的目标时间 0.4 秒之前开始插值计算，以确保在达到目标点之前能平滑地过渡到下一个位置。
                                                                                             //原因：这个条件确保了在到达 interp_traj_end 时间前 0.4 秒开始插值，从而在执行过程中留有足够的时间连续平滑地计算和发送插值命令，让机械臂保持稳定的速度和位置控制，避免突然停顿或跳跃。
            while (real_time < interp_traj_end.time_from_start.toSec() - SERVOJ_DURATION) {  //使用上面定义的sample_traj函数，计算当前时间time_index对应的插值位置tmp，并返回。
                double time_index = real_time - interp_traj_begin.time_from_start.toSec();
                std::vector<double> tmp = sample_traj(interp_traj_begin, interp_traj_end, time_index);
                char cmd[100];                                                               //将插值位置tmp 通过sprintf进行格式化，将tmp和t的值的内容存入cmd字符数组中
                sprintf(cmd, "servoj(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,t=%0.3f)", tmp[0], tmp[1], tmp[2], tmp[3],
                        tmp[4], tmp[5], t);
                int32_t err_id;
                commander_->motionDoCmd(cmd, err_id);                                        //调用commander_对象的motionDoCmd方法，将生成的cmd字符串命令发送给机械臂控制器，以执行对应的动作。其中err_id用于捕获命令执行过程中的错误信息，0表示成功，非0表示错误
                timer.sleep();
                t1 = ros::Time::now().toSec();
                real_time = t1 - t0;
            }
        }
        std::vector<double> last_traj;                                                      //获取轨迹的最后一个目标点，将位置转换为角度（弧度转为角度），以便机械臂转准确的停留在最终位置
        int point_num = goal->trajectory.points.size();
        for (int i = 0; i < 6; i++) {
            last_traj.push_back(goal->trajectory.points[point_num - 1].positions[i] * 180 / M_PI);
        }
        char cmd[100];
        sprintf(cmd, "servoj(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,t=%0.3f)", last_traj[0], last_traj[1], last_traj[2],
                last_traj[3], last_traj[4], last_traj[5], t);
        int32_t err_id;
        commander_->motionDoCmd(cmd, err_id);                                               //发布最后一个位置的servoj指令
    } catch (const TcpClientException& err) {                                               //异常处理
        ROS_ERROR("%s", err.what());
        return;
    }

    timer_.stop();
    movj_timer_.stop();
    handle.setSucceeded();                                                                  //通知客户端轨迹执行已完成
}

/**
 ***********************************************************************************************************************
    用于处理ROS Action Server接收到的轨迹跟踪目标请求
    goalHandle 函数的作用是接收并初始化轨迹跟踪目标，设置反馈与运动控制的定时器，启动目标的执行流程。
    通过 feedbackHandle 和 moveHandle 函数，系统能够在执行过程中定时发布状态反馈并控制机械臂逐步跟随指定轨迹。
 ***********************************************************************************************************************
 */

void CRRobot::goalHandle(ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle handle)    //handle用于管理与客户端的通信，传入的是目标值
{
    index_ = 0;
    for (uint32_t i = 0; i < 6; i++) {                                                      //handle.getGoal()获取轨迹目标信息，并提取轨迹中的最后一个点，将各关节的目标位置赋值给goal_[i]
        goal_[i] = handle.getGoal()->trajectory.points[handle.getGoal()->trajectory.points.size() - 1].positions[i];
    }
    timer_ = control_nh_.createTimer(ros::Duration(1.0), boost::bind(&CRRobot::feedbackHandle, this, _1, handle)); //创建定时器，每秒触发一次，当触发时，调用feedbackHandle（上面），通过boost::bind绑定，使其接收触发时间和目标handle
    movj_timer_ = control_nh_.createTimer(ros::Duration(trajectory_duration_),                                     //再创建一个定时器，用于控制机械臂运动，间隔为trajectory_duration_，即控制每段轨迹的时间
                                          boost::bind(&CRRobot::moveHandle, this, _1, handle));                    //每次触发时，调用moveHandle来执行机械臂的移动指令，使其逐步完成目标轨迹
    timer_.start();
    movj_timer_.start();
    handle.setAccepted();                                                                 //handle.setAccepted()接收客户端发送的轨迹跟踪目标了，通知客户端该目标已被接收并开始执行。
}

/**
 ***********************************************************************************************************************
    用于取消当前正在执行的轨迹目标
 ***********************************************************************************************************************
 */

void CRRobot::cancelHandle(ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle handle)
{
    timer_.stop();
    movj_timer_.stop();
    handle.setSucceeded();
}

/**
 ***********************************************************************************************************************
    用于获取机械臂的当前关节状态
 ***********************************************************************************************************************
 */

void CRRobot::getJointState(double* point)
{
    commander_->getCurrentJointStatus(point);
}

/**
 ***********************************************************************************************************************
    用于检查机械臂是否已启用
 ***********************************************************************************************************************
 */

bool CRRobot::isEnable() const
{
    return commander_->isEnable();
}

/**
 ***********************************************************************************************************************
    用于检查机械臂是否已连接
 ***********************************************************************************************************************
 */

bool CRRobot::isConnected() const
{
    return commander_->isConnected();
}

/**
 ***********************************************************************************************************************
    用于获取机械臂末端执行器的实际位置和方向
 ***********************************************************************************************************************
 */

void CRRobot::getToolVectorActual(double* val)
{
    commander_->getToolVectorActual(val);
}

/*
 *----------------------------------------------------------------------------------------------------------------------
 *                                                  dashboard
 *                                           仪表盘，上位机与机械臂的交互
 *----------------------------------------------------------------------------------------------------------------------
 */

bool CRRobot::enableRobot(dobot_v4_bringup::EnableRobot::Request& request,
                          dobot_v4_bringup::EnableRobot::Response& response)
{
    return commander_->callRosService(parseTool::parserenableRobotRequest2String(request), response.res);
}

bool CRRobot::disableRobot(dobot_v4_bringup::DisableRobot::Request& request,
                           dobot_v4_bringup::DisableRobot::Response& response)
{
    return commander_->callRosService(parseTool::parserdisableRobotRequest2String(request), response.res);
}

bool CRRobot::clearError(dobot_v4_bringup::ClearError::Request& request,
                         dobot_v4_bringup::ClearError::Response& response)
{
    return commander_->callRosService(parseTool::parserclearErrorRequest2String(request), response.res);
}

bool CRRobot::speedFactor(dobot_v4_bringup::SpeedFactor::Request& request,
                          dobot_v4_bringup::SpeedFactor::Response& response)
{
    return commander_->callRosService(parseTool::parserspeedFactorRequest2String(request), response.res);
}

bool CRRobot::user(dobot_v4_bringup::User::Request& request, dobot_v4_bringup::User::Response& response)
{
    return commander_->callRosService(parseTool::parseruserRequest2String(request), response.res);
}

bool CRRobot::tool(dobot_v4_bringup::Tool::Request& request, dobot_v4_bringup::Tool::Response& response)
{
    return commander_->callRosService(parseTool::parsertoolRequest2String(request), response.res);
}

bool CRRobot::robotMode(dobot_v4_bringup::RobotMode::Request& request, dobot_v4_bringup::RobotMode::Response& response)
{
    return commander_->callRosService(parseTool::parserrobotModeRequest2String(request), response.res);
}

bool CRRobot::setPayload(dobot_v4_bringup::SetPayload::Request& request,
                         dobot_v4_bringup::SetPayload::Response& response)
{
    return commander_->callRosService(parseTool::parsersetPayloadRequest2String(request), response.res);
}

bool CRRobot::DO(dobot_v4_bringup::DO::Request& request, dobot_v4_bringup::DO::Response& response)
{
    return commander_->callRosService(parseTool::parserDORequest2String(request), response.res);
}

bool CRRobot::DOInstant(dobot_v4_bringup::DOInstant::Request& request, dobot_v4_bringup::DOInstant::Response& response)
{
    return commander_->callRosService(parseTool::parserDOInstantRequest2String(request), response.res);
}

bool CRRobot::toolDO(dobot_v4_bringup::ToolDO::Request& request, dobot_v4_bringup::ToolDO::Response& response)
{
    return commander_->callRosService(parseTool::parsertoolDORequest2String(request), response.res);
}

bool CRRobot::toolDOInstant(dobot_v4_bringup::ToolDOInstant::Request& request,
                            dobot_v4_bringup::ToolDOInstant::Response& response)
{
    return commander_->callRosService(parseTool::parsertoolDOInstantRequest2String(request), response.res);
}

bool CRRobot::AO(dobot_v4_bringup::AO::Request& request, dobot_v4_bringup::AO::Response& response)
{
    return commander_->callRosService(parseTool::parserAORequest2String(request), response.res);
}

bool CRRobot::AOInstant(dobot_v4_bringup::AOInstant::Request& request, dobot_v4_bringup::AOInstant::Response& response)
{
    return commander_->callRosService(parseTool::parserAOInstantRequest2String(request), response.res);
}

bool CRRobot::accJ(dobot_v4_bringup::AccJ::Request& request, dobot_v4_bringup::AccJ::Response& response)
{
    return commander_->callRosService(parseTool::parseraccJRequest2String(request), response.res);
}

bool CRRobot::accL(dobot_v4_bringup::AccL::Request& request, dobot_v4_bringup::AccL::Response& response)
{
    return commander_->callRosService(parseTool::parseraccLRequest2String(request), response.res);
}

bool CRRobot::velJ(dobot_v4_bringup::VelJ::Request& request, dobot_v4_bringup::VelJ::Response& response)
{
    return commander_->callRosService(parseTool::parservelJRequest2String(request), response.res);
}

bool CRRobot::velL(dobot_v4_bringup::VelL::Request& request, dobot_v4_bringup::VelL::Response& response)
{
    return commander_->callRosService(parseTool::parservelLRequest2String(request), response.res);
}

bool CRRobot::cp(dobot_v4_bringup::CP::Request& request, dobot_v4_bringup::CP::Response& response)
{
    return commander_->callRosService(parseTool::parsercpRequest2String(request), response.res);
}

bool CRRobot::powerOn(dobot_v4_bringup::PowerOn::Request& request, dobot_v4_bringup::PowerOn::Response& response)
{
    return commander_->callRosService(parseTool::parserpowerOnRequest2String(request), response.res);
}

bool CRRobot::runScript(dobot_v4_bringup::RunScript::Request& request, dobot_v4_bringup::RunScript::Response& response)
{
    return commander_->callRosService(parseTool::parserrunScriptRequest2String(request), response.res);
}

bool CRRobot::stop(dobot_v4_bringup::Stop::Request& request, dobot_v4_bringup::Stop::Response& response)
{
    return commander_->callRosService(parseTool::parserstopRequest2String(request), response.res);
}

bool CRRobot::pause(dobot_v4_bringup::Pause::Request& request, dobot_v4_bringup::Pause::Response& response)
{
    return commander_->callRosService(parseTool::parserpauseRequest2String(request), response.res);
}

bool CRRobot::Continue(dobot_v4_bringup::Continue::Request& request, dobot_v4_bringup::Continue::Response& response)
{
    return commander_->callRosService(parseTool::parserContinueRequest2String(request), response.res);
}

bool CRRobot::setCollisionLevel(dobot_v4_bringup::SetCollisionLevel::Request& request,
                                dobot_v4_bringup::SetCollisionLevel::Response& response)
{
    return commander_->callRosService(parseTool::parsersetCollisionLevelRequest2String(request), response.res);
}

bool CRRobot::enableSafeSkin(dobot_v4_bringup::EnableSafeSkin::Request& request,
                             dobot_v4_bringup::EnableSafeSkin::Response& response)
{
    return commander_->callRosService(parseTool::parserEnableSafeSkinRequest2String(request), response.res);
}

bool CRRobot::setSafeSkin(dobot_v4_bringup::SetSafeSkin::Request& request,
                          dobot_v4_bringup::SetSafeSkin::Response& response)
{
    return commander_->callRosService(parseTool::parserSetSafeSkinRequest2String(request), response.res);
}

bool CRRobot::getStartPose(dobot_v4_bringup::GetStartPose::Request& request,
                           dobot_v4_bringup::GetStartPose::Response& response)
{
    return commander_->callRosService(parseTool::parserGetStartPoseRequest2String(request), response.res);
}

bool CRRobot::startPath(dobot_v4_bringup::StartPath::Request& request, dobot_v4_bringup::StartPath::Response& response)
{
    return commander_->callRosService(parseTool::parserStartPathRequest2String(request), response.res);
}

bool CRRobot::positiveKin(dobot_v4_bringup::PositiveKin::Request& request,
                          dobot_v4_bringup::PositiveKin::Response& response)
{
    return commander_->callRosService(parseTool::parserPositiveKinRequest2String(request), response.res);
}

bool CRRobot::inverseKin(dobot_v4_bringup::InverseKin::Request& request,
                         dobot_v4_bringup::InverseKin::Response& response)
{
    return commander_->callRosService(parseTool::parserInverseKinRequest2String(request), response.res);
}

bool CRRobot::getAngle(dobot_v4_bringup::GetAngle::Request& request, dobot_v4_bringup::GetAngle::Response& response)
{
    return commander_->callRosService(parseTool::parserGetAngleRequest2String(request), response.res);
}

bool CRRobot::getPose(dobot_v4_bringup::GetPose::Request& request, dobot_v4_bringup::GetPose::Response& response)
{
    return commander_->callRosService(parseTool::parserGetPoseRequest2String(request), response.res);
}

bool CRRobot::emergencyStop(dobot_v4_bringup::EmergencyStop::Request& request,
                            dobot_v4_bringup::EmergencyStop::Response& response)
{
    return commander_->callRosService(parseTool::parserEmergencyStopRequest2String(request), response.res);
}

bool CRRobot::modbusRTUCreate(dobot_v4_bringup::ModbusRTUCreate::Request& request,
                              dobot_v4_bringup::ModbusRTUCreate::Response& response)
{
    return commander_->callRosService(parseTool::parserModbusRTUCreateRequest2String(request), response.res);
}

bool CRRobot::modbusCreate(dobot_v4_bringup::ModbusCreate::Request& request,
                           dobot_v4_bringup::ModbusCreate::Response& response)
{
    return commander_->callRosService(parseTool::parserModbusCreateRequest2String(request), response.res);
}

bool CRRobot::modbusClose(dobot_v4_bringup::ModbusClose::Request& request,
                          dobot_v4_bringup::ModbusClose::Response& response)
{
    return commander_->callRosService(parseTool::parserModbusCloseRequest2String(request), response.res);
}

bool CRRobot::getInBits(dobot_v4_bringup::GetInBits::Request& request, dobot_v4_bringup::GetInBits::Response& response)
{
    return commander_->callRosService(parseTool::parserGetInBitsRequest2String(request), response.res);
}

bool CRRobot::getInRegs(dobot_v4_bringup::GetInRegs::Request& request, dobot_v4_bringup::GetInRegs::Response& response)
{
    return commander_->callRosService(parseTool::parserGetInRegsRequest2String(request), response.res);
}

bool CRRobot::getCoils(dobot_v4_bringup::GetCoils::Request& request, dobot_v4_bringup::GetCoils::Response& response)
{
    return commander_->callRosService(parseTool::parserGetCoilsRequest2String(request), response.res);
}

bool CRRobot::setCoils(dobot_v4_bringup::SetCoils::Request& request, dobot_v4_bringup::SetCoils::Response& response)
{
    return commander_->callRosService(parseTool::parserSetCoilsRequest2String(request), response.res);
}

bool CRRobot::getHoldRegs(dobot_v4_bringup::GetHoldRegs::Request& request,
                          dobot_v4_bringup::GetHoldRegs::Response& response)
{
    return commander_->callRosService(parseTool::parserGetHoldRegsRequest2String(request), response.res);
}

bool CRRobot::setHoldRegs(dobot_v4_bringup::SetHoldRegs::Request& request,
                          dobot_v4_bringup::SetHoldRegs::Response& response)
{
    return commander_->callRosService(parseTool::parserSetHoldRegsRequest2String(request), response.res);
}

bool CRRobot::GetErrorID(dobot_v4_bringup::GetErrorID::Request& request,
                         dobot_v4_bringup::GetErrorID::Response& response)
{
    return getErrorID(request, response);
}

bool CRRobot::DI(dobot_v4_bringup::DI::Request& request, dobot_v4_bringup::DI::Response& response)
{
    return commander_->callRosService(parseTool::parserDIRequest2String(request), response.res);
}

bool CRRobot::toolDI(dobot_v4_bringup::ToolDI::Request& request, dobot_v4_bringup::ToolDI::Response& response)
{
    return commander_->callRosService(parseTool::parserToolDIRequest2String(request), response.res);
}

bool CRRobot::AI(dobot_v4_bringup::AI::Request& request, dobot_v4_bringup::AI::Response& response)
{
    return commander_->callRosService(parseTool::parserAIRequest2String(request), response.res);
}

bool CRRobot::toolAI(dobot_v4_bringup::ToolAI::Request& request, dobot_v4_bringup::ToolAI::Response& response)
{
    return commander_->callRosService(parseTool::parserToolAIRequest2String(request), response.res);
}

bool CRRobot::DIGroup(dobot_v4_bringup::DIGroup::Request& request, dobot_v4_bringup::DIGroup::Response& response)
{
    return commander_->callRosService(parseTool::parserDIGroupRequest2String(request), response.res);
}

bool CRRobot::doGroup(dobot_v4_bringup::DOGroup::Request& request, dobot_v4_bringup::DOGroup::Response& response)
{
    return commander_->callRosService(parseTool::parserdoGroupRequest2String(request), response.res);
}

bool CRRobot::brakeControl(dobot_v4_bringup::BrakeControl::Request& request,
                           dobot_v4_bringup::BrakeControl::Response& response)
{
    return commander_->callRosService(parseTool::parserbrakeControlRequest2String(request), response.res);
}

bool CRRobot::startDrag(dobot_v4_bringup::StartDrag::Request& request, dobot_v4_bringup::StartDrag::Response& response)
{
    return commander_->callRosService(parseTool::parserstartDragRequest2String(request), response.res);
}

bool CRRobot::stopDrag(dobot_v4_bringup::StopDrag::Request& request, dobot_v4_bringup::StopDrag::Response& response)
{
    return commander_->callRosService(parseTool::parserStopDragRequest2String(request), response.res);
}

bool CRRobot::dragSensivity(dobot_v4_bringup::DragSensivity::Request& request,
                            dobot_v4_bringup::DragSensivity::Response& response)
{
    return commander_->callRosService(parseTool::parserDragSensivityRequest2String(request), response.res);
}

bool CRRobot::getDO(dobot_v4_bringup::GetDO::Request& request, dobot_v4_bringup::GetDO::Response& response)
{
    return commander_->callRosService(parseTool::parserGetDORequest2String(request), response.res);
}

bool CRRobot::getAO(dobot_v4_bringup::GetAO::Request& request, dobot_v4_bringup::GetAO::Response& response)
{
    return commander_->callRosService(parseTool::parserGetAORequest2String(request), response.res);
}

bool CRRobot::getDOGroup(dobot_v4_bringup::GetDOGroup::Request& request,
                         dobot_v4_bringup::GetDOGroup::Response& response)
{
    return commander_->callRosService(parseTool::parserGetDOGroupRequest2String(request), response.res);
}

bool CRRobot::setTool485(dobot_v4_bringup::SetTool485::Request& request,
                         dobot_v4_bringup::SetTool485::Response& response)
{
    return commander_->callRosService(parseTool::parserSetTool485Request2String(request), response.res);
}

bool CRRobot::setSafeWallEnable(dobot_v4_bringup::SetSafeWallEnable::Request& request,
                                dobot_v4_bringup::SetSafeWallEnable::Response& response)
{
    return commander_->callRosService(parseTool::parserSetSafeWallEnableRequest2String(request), response.res);
}

bool CRRobot::setToolPower(dobot_v4_bringup::SetToolPower::Request& request,
                           dobot_v4_bringup::SetToolPower::Response& response)
{
    return commander_->callRosService(parseTool::parserSetToolPowerRequest2String(request), response.res);
}

bool CRRobot::setToolMode(dobot_v4_bringup::SetToolMode::Request& request,
                          dobot_v4_bringup::SetToolMode::Response& response)
{
    return commander_->callRosService(parseTool::parserSetToolModeRequest2String(request), response.res);
}

bool CRRobot::setBackDistance(dobot_v4_bringup::SetBackDistance::Request& request,
                              dobot_v4_bringup::SetBackDistance::Response& response)
{
    return commander_->callRosService(parseTool::parserSetBackDistanceRequest2String(request), response.res);
}

bool CRRobot::setPostCollisionMode(dobot_v4_bringup::SetPostCollisionMode::Request& request,
                                   dobot_v4_bringup::SetPostCollisionMode::Response& response)
{
    return commander_->callRosService(parseTool::parserSetPostCollisionModeRequest2String(request), response.res);
}

bool CRRobot::setUser(dobot_v4_bringup::SetUser::Request& request, dobot_v4_bringup::SetUser::Response& response)
{
    return commander_->callRosService(parseTool::parserSetUserRequest2String(request), response.res);
}

bool CRRobot::setTool(dobot_v4_bringup::SetTool::Request& request, dobot_v4_bringup::SetTool::Response& response)
{
    return commander_->callRosService(parseTool::parserSetToolRequest2String(request), response.res);
}

bool CRRobot::calcUser(dobot_v4_bringup::CalcUser::Request& request, dobot_v4_bringup::CalcUser::Response& response)
{
    return commander_->callRosService(parseTool::parserCalcUserRequest2String(request), response.res);
}

bool CRRobot::calcTool(dobot_v4_bringup::CalcTool::Request& request, dobot_v4_bringup::CalcTool::Response& response)
{
    return commander_->callRosService(parseTool::parserCalcToolRequest2String(request), response.res);
}

bool CRRobot::getInputBool(dobot_v4_bringup::GetInputBool::Request& request,
                           dobot_v4_bringup::GetInputBool::Response& response)
{
    return commander_->callRosService(parseTool::parserGetInputboolRequest2String(request), response.res);
}

bool CRRobot::getInputInt(dobot_v4_bringup::GetInputInt::Request& request,
                          dobot_v4_bringup::GetInputInt::Response& response)
{
    return commander_->callRosService(parseTool::parserGetInputIntRequest2String(request), response.res);
}

bool CRRobot::getInputFloat(dobot_v4_bringup::GetInputFloat::Request& request,
                            dobot_v4_bringup::GetInputFloat::Response& response)
{
    return commander_->callRosService(parseTool::parserGetInputFloatRequest2String(request), response.res);
}

bool CRRobot::getOutputBool(dobot_v4_bringup::GetOutputBool::Request& request,
                            dobot_v4_bringup::GetOutputBool::Response& response)
{
    return commander_->callRosService(parseTool::parserGetOutputboolRequest2String(request), response.res);
}

bool CRRobot::getOutputInt(dobot_v4_bringup::GetOutputInt::Request& request,
                           dobot_v4_bringup::GetOutputInt::Response& response)
{
    return commander_->callRosService(parseTool::parserGetOutputIntRequest2String(request), response.res);
}

bool CRRobot::getOutputFloat(dobot_v4_bringup::GetOutputFloat::Request& request,
                             dobot_v4_bringup::GetOutputFloat::Response& response)
{
    return commander_->callRosService(parseTool::parserGetOutputFloatRequest2String(request), response.res);
}

bool CRRobot::setOutputBool(dobot_v4_bringup::SetOutputBool::Request& request,
                            dobot_v4_bringup::SetOutputBool::Response& response)
{
    return commander_->callRosService(parseTool::parserSetOutputboolRequest2String(request), response.res);
}

bool CRRobot::setOutputInt(dobot_v4_bringup::SetOutputInt::Request& request,
                           dobot_v4_bringup::SetOutputInt::Response& response)
{
    return commander_->callRosService(parseTool::parserSetOutputIntRequest2String(request), response.res);
}

bool CRRobot::setOutputFloat(dobot_v4_bringup::SetOutputFloat::Request& request,
                             dobot_v4_bringup::SetOutputFloat::Response& response)
{
    return commander_->callRosService(parseTool::parserSetOutputFloatRequest2String(request), response.res);
}

bool CRRobot::movJ(dobot_v4_bringup::MovJ::Request& request, dobot_v4_bringup::MovJ::Response& response)
{
    return commander_->callRosService(parseTool::parsermovJRequest2String(request), response.res);
}

bool CRRobot::movL(dobot_v4_bringup::MovL::Request& request, dobot_v4_bringup::MovL::Response& response)
{
    return commander_->callRosService(parseTool::parsermovLRequest2String(request), response.res);
}

bool CRRobot::movLIO(dobot_v4_bringup::MovLIO::Request& request, dobot_v4_bringup::MovLIO::Response& response)
{
    return commander_->callRosService(parseTool::parserMovLIORequest2String(request), response.res);
}

bool CRRobot::movJIO(dobot_v4_bringup::MovJIO::Request& request, dobot_v4_bringup::MovJIO::Response& response)
{
    return commander_->callRosService(parseTool::parserMovJIORequest2String(request), response.res);
}

bool CRRobot::arc(dobot_v4_bringup::Arc::Request& request, dobot_v4_bringup::Arc::Response& response)
{
    return commander_->callRosService(parseTool::parserArcRequest2String(request), response.res);
}

bool CRRobot::circle(dobot_v4_bringup::Circle::Request& request, dobot_v4_bringup::Circle::Response& response)
{
    return commander_->callRosService(parseTool::parserCircleRequest2String(request), response.res);
}

bool CRRobot::moveJog(dobot_v4_bringup::MoveJog::Request& request, dobot_v4_bringup::MoveJog::Response& response)
{
    return commander_->callRosService(parseTool::parsermoveJogRequest2String(request), response.res);
}

bool CRRobot::stopmoveJog(dobot_v4_bringup::StopMoveJog::Request& request,
                          dobot_v4_bringup::StopMoveJog::Response& response)
{
    return commander_->callRosService(parseTool::parserStopmoveJogRequest2String(request), response.res);
}

bool CRRobot::relMovJTool(dobot_v4_bringup::RelMovJTool::Request& request,
                          dobot_v4_bringup::RelMovJTool::Response& response)
{
    return commander_->callRosService(parseTool::parserRelMovJToolRequest2String(request), response.res);
}

bool CRRobot::relMovLTool(dobot_v4_bringup::RelMovLTool::Request& request,
                          dobot_v4_bringup::RelMovLTool::Response& response)
{
    return commander_->callRosService(parseTool::parserRelMovLToolRequest2String(request), response.res);
}

bool CRRobot::relMovJUser(dobot_v4_bringup::RelMovJUser::Request& request,
                          dobot_v4_bringup::RelMovJUser::Response& response)
{
    return commander_->callRosService(parseTool::parserRelMovJUserRequest2String(request), response.res);
}

bool CRRobot::relMovLUser(dobot_v4_bringup::RelMovLUser::Request& request,
                          dobot_v4_bringup::RelMovLUser::Response& response)
{
    return commander_->callRosService(parseTool::parserRelMovLUserRequest2String(request), response.res);
}

bool CRRobot::relJointMovJ(dobot_v4_bringup::RelJointMovJ::Request& request,
                           dobot_v4_bringup::RelJointMovJ::Response& response)
{
    return commander_->callRosService(parseTool::parserrelJointMovJRequest2String(request), response.res);
}

bool CRRobot::getCurrentCommandId(dobot_v4_bringup::GetCurrentCommandId::Request& request,
                                  dobot_v4_bringup::GetCurrentCommandId::Response& response)
{
    return commander_->callRosService(parseTool::parserGetCurrentCommandIdRequest2String(request), response.res);
}

bool CRRobot::servoJ(dobot_v4_bringup::ServoJ::Request& request, dobot_v4_bringup::ServoJ::Response& response)
{
    return commander_->callRosService(parseTool::parserServoJRequest2String(request), response.res);
}

bool CRRobot::servoP(dobot_v4_bringup::ServoP::Request& request, dobot_v4_bringup::ServoP::Response& response)
{
    return commander_->callRosService(parseTool::parserServoPRequest2String(request), response.res);
}

/**
 ***********************************************************************************************************************
    提供上位机与机械臂之间的通用命令接口，通过调用TcpDashboard服务将命令发送给机械臂的控制器，并返回执行结果
 ***********************************************************************************************************************
 */

bool CRRobot::tcpDashboard(dobot_v4_bringup::TcpDashboard::Request& request,                  //request：包含要发送到机械臂控制器的命令字符串。
                           dobot_v4_bringup::TcpDashboard::Response& response)                //response：包含机械臂执行命令后的返回结果。   返回值：布尔值，表示服务调用是否成功。true 表示成功，false 表示出现异常。                                                                                           
{
    try {
        std::vector<std::string> result;                                                      
        int res;                                                                             //res 用于接收命令执行的状态码，通常表示执行成功或失败。
        commander_->callRosService(request.command, res, result);                            //定义 result 向量，用于存储命令执行后的输出信息。调用 commander_->callRosService，将 request.command 中的命令发送到机械臂控制器，并将返回结果存储到result。
        if (!result.empty()) {                                                               //处理返回结果，检查result是否非空，若非空则处理返回的多条信息。
            for (int i = 0; i < result.size(); i++) {                                        //遍历result中的字符串，将其拼接成一个字符串并存储到response.result ，用于返回客户端
                if (i == result.size()) {
                    response.result = response.result + result[i];
                    break;
                }
                response.result = response.result + result[i] + " ";                         //如果是最后一个元素，直接添加到response.result，否则添加空格分隔(这部分如果是这样的话，就有点问题，会导致最后多一个空格，后期如果出现问题可以在这解决)
            }
        }
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        return false;
    }
}

/**
 ***********************************************************************************************************************
    用于从机械臂控制器获取当前的错误ID，并将非零错误ID返回给调用者。这是通过调用tcpDashboard服务发送GetErrorID（）命令来实现的。
 ***********************************************************************************************************************
 */

bool CRRobot::getErrorID(dobot_v4_bringup::GetErrorID::Request& request,                     //request：服务请求，包含 GetErrorID 请求参数，但该函数并未直接使用 request。
                         dobot_v4_bringup::GetErrorID::Response& response)                   //response：服务响应，包含从控制器获取的错误 ID 列表（error_id）。   返回值：布尔值，表示是否成功获取错误 ID。true 表示成功，false 表示失败。
{
    dobot_v4_bringup::TcpDashboard::Request req = {};                                        //创建TcpDashboard的请求对象和响应对象
    dobot_v4_bringup::TcpDashboard::Response res = {};
    req.command = "GetErrorID()";                                                            //将请求命令设为GetErrorID()，这是控制器用于获取当前错误ID的指令
    if (tcpDashboard(req, res)) {                                                            //调用命令，发送reaq，并将控制器响应存储在res.result中。返回值是布尔类型，true表示返回成功
        std::stringstream ss(res.result);    // 创建一个字符串流对象，并将输入字符串传入
        int errorid;
        while (ss >> errorid) {    // 使用字符串流读取数字，存到errorid
            if (errorid != 0) {
                response.error_id.push_back(errorid);    // 将读取到的数字添加到容器中
            }
        }

        return true;                                                                          //如果成功获取并处理了错误ID，返回True
    }
    return false;                                                                             //如果调用失败，返回false，表示无法获取错误ID
}

/**
 ***********************************************************************************************************************
    用于解析包含错误ID和返回值的字符串，并将解析后的信息格式化为可读输出。
 ***********************************************************************************************************************
 */

std::string CRRobot::parseString(const std::string& str)                                      //str：需要解析的输入字符串，通常包含错误ID和返回值。返回值是字符串，是格式化后的字符串
{
    std::string returnInfomation = "ErrorID: ";

    std::size_t pos = str.find(',');
    if (pos == std::string::npos)                                                             //如果没找到都好，则抛出逻辑错误异常
        throw std::logic_error(std::string("Has no ',' found : ") + str);
    returnInfomation += str.substr(0, pos);
    returnInfomation += " ReturnValue: ";

    // parse result
    std::size_t start_pos = str.find('{');
    if (start_pos == std::string::npos)
        throw std::logic_error(std::string("Has no '{': ") + str);
    std::size_t end_pos = str.find('}');
    if (end_pos == std::string::npos)
        throw std::logic_error(std::string("Has no '}': ") + str);

    assert(end_pos > start_pos);
    returnInfomation += str.substr(start_pos + 1, end_pos - start_pos - 1);
    return returnInfomation;
}

/**
 ***********************************************************************************************************************
    记录解析后的信息。
 ***********************************************************************************************************************
 */


void CRRobot::Info(const std::string& info)
{
    ROS_INFO("%s", parseString(info).c_str());
}

/**
 ***********************************************************************************************************************
    后台监控机械臂状态，尤其是错误报警，当检测到机械臂进入报警模式，会查询错误ID，并输出详细的错误信息。
 ***********************************************************************************************************************
 */


void CRRobot::backendTask(const ros::TimerEvent& e)                                           //e：定时事件触发参数，表示ROS定时事件，用于定时触发的任务
{
    uint16_t robot_mode = commander_->getRobotMode();                                         //获取机械臂当前的工作模式
    if (robot_mode == 9 && last_robot_mode_ != 9) {                                           //如果当前是9且上一个记录的模式不是9，说明此时机械臂进入了报警模式。
        dobot_v4_bringup::GetErrorID::Request req = {};
        dobot_v4_bringup::GetErrorID::Response res = {};                                      //创建req和res
        bool ok = GetErrorID(req, res);                                                       //调用GetErrorID和获取当前的错误ID列表
        if (ok) {
            for (const auto& i : res.error_id) {                                              //ok为true表示调用成功，则遍历res.error_id，通过ROS_ERROR记录每个非零错误ID，表明当前存在具体的错误
                ROS_ERROR("Robot alarm, error id %d", i);
            }
        } else {
            ROS_ERROR("Robot alarm");                                                         //调用失败则仅输出一条通用的报警信息"Robot alarm"
        }
    }
    last_robot_mode_ = robot_mode;
}
