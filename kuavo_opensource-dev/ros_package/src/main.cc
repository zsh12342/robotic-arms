#include <HighlyDynamicRobot.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"

#include <dynamic_biped/walkCommand.h>
#include <dynamic_biped/srvChangePhases.h>
#include <dynamic_biped/srvClearPositionCMD.h>
#include <dynamic_biped/srvchangeCtlMode.h>
#include <dynamic_biped/changeArmCtrlMode.h>
#include <dynamic_biped/changeAMBACCtrlMode.h>
#include <dynamic_biped/controlEndHand.h>
#include <dynamic_biped/centerOfMass.h>
#include <dynamic_biped/comPosition.h> 
#include <dynamic_biped/getComPosition.h>
#include <dynamic_biped/changeHandArmPosesByConfigName.h>
#include <dynamic_biped/controlJodellClaw.h>

#include <sensor_msgs/JointState.h>
#include <dynamic_biped/robotQVTau.h>
#include <dynamic_biped/robotTorsoState.h>
#include <dynamic_biped/robotPhase.h>
#include <dynamic_biped/robotArmQVVD.h>
#include <dynamic_biped/robotHandPosition.h>
#include <dynamic_biped/robotHeadMotionData.h>
#include <dynamic_biped/armTargetPoses.h>
#include <dynamic_biped/robotImuGyroAcc.h>
#include <dynamic_biped/EndEffectorStateStamped.h>

#include "env_utils.h"

HighlyDynamic::HighlyDynamicRobot *robot_ptr;
RobotState_t HDrobotState;
std::map<std::string, mainPhase_t> phase_map = {
    {"P_stand", P_stand},
    {"P_walk", P_walk},
    {"P_jump", P_jump},
    {"P_squat", P_squat},
    {"P_ERROR", P_ERROR},
    {"P_None", P_None}};
std::map<std::string, subPhase_t> sub_phase_map = {

    // walk
    {"walk_pre", walk_pre},
    {"walking", walking},
    {"walk_stop", walk_stop},

    // jump
    {"jump_pre", jump_pre},
    {"jump_take_off", jump_take_off},
    {"jump_flight", jump_flight},
    {"jump_touch_down", jump_touch_down},
    {"jump_to_ready", jump_to_ready},

    {"sub_phase_none", sub_phase_none},

};

std::map<int, std::string> sub_phase_name_map = {
    {walk_pre, "walk_pre"},
    {walking, "walking"},
    {walk_stop, "walk_stop"},
    {jump_pre, "jump_pre"},
    {jump_take_off, "jump_take_off"},
    {jump_flight, "jump_flight"},
    {jump_touch_down, "jump_touch_down"},
    {jump_to_ready, "jump_to_ready"},
    {squat_normal, "squat_normal"},
    {squat_quick, "squat_quick"},
    {sub_phase_none, "sub_phase_none"}};

std::map<int, std::string> main_phase_name_map = {
    {P_stand, "P_stand"},
    {P_walk, "P_walk"},
    {P_jump, "P_jump"},
    {P_squat, "P_squat"},
    {P_ERROR, "P_ERROR"},
    {P_None, "P_None"}};

static void updateState()
{
    robot_ptr->getRobotState(HDrobotState);
}

class HDrobot_node
{
public:
    ros::Publisher robot_q_v_tau_pub;
    ros::Publisher robot_arm_q_v_vd_pub;
    ros::Publisher robot_torso_state_pub;
    ros::Publisher robot_phase_pub;
    ros::Publisher robot_hand_position_pub;
    ros::Publisher robot_imu_gyro_acc_pub;
    ros::Publisher robot_head_motor_position_pub;
    ros::Publisher robot_end_effector_state_pub;

    HDrobot_node(ros::NodeHandle &nh) : nh_(nh)
    {
        CMD_sub = nh_.subscribe("/walkCommand", 10, &HDrobot_node::walkCommandCallback);

        // Publish timestamp topic at 100hz
        // robot_q_v_tau_pub = nh_.advertise<ros::Time>("/robot_q_v_tau", 100);
        robot_q_v_tau_pub = nh_.advertise<dynamic_biped::robotQVTau>("/robot_q_v_tau", 10);

        robot_arm_q_v_vd_pub = nh_.advertise<dynamic_biped::robotArmQVVD>("/robot_arm_q_v_tau", 10);

        robot_torso_state_pub = nh_.advertise<dynamic_biped::robotTorsoState>("robot_torso_state", 10);

        robot_phase_pub = nh_.advertise<dynamic_biped::robotPhase>("leju_robot_phase", 10);

        robot_hand_position_pub = nh_.advertise<dynamic_biped::robotHandPosition>("robot_hand_position", 10);

        robot_imu_gyro_acc_pub = nh_.advertise<dynamic_biped::robotImuGyroAcc>("robot_imu_gyro_acc", 10);

        robot_end_effector_state_pub = nh_.advertise<dynamic_biped::EndEffectorStateStamped>("robot_end_effector_state", 10);


        // Subscribe to /etherCATJoint/motordata topic
        robot_control_hand_position_sub = nh_.subscribe("/control_robot_hand_position", 10, &HDrobot_node::robotControlHandPositionCallback);

        // Subscribe to /kuavo_arm_traj topic
        joint2_command_desired_sub = nh_.subscribe("/kuavo_arm_traj", 10, &HDrobot_node::joint2CommandDesiredCallback);
        arm_target_poses_sub = nh_.subscribe("/kuavo_arm_target_poses", 10, &HDrobot_node::armTargetPoseCallback);
        robot_head_motion_data_sub = nh_.subscribe("/robot_head_motion_data", 10, &HDrobot_node::robotHeadMotionDataCallback);  
        // Create services and bind callback functions
        static ros::ServiceServer change_phases_service = nh.advertiseService("setPhase", changePhasesCallback);
        static ros::ServiceServer clear_position_cmd_service = nh.advertiseService("clear_position_cmd", clearPositionCMDCallback);
        static ros::ServiceServer change_ctl_mode_service = nh.advertiseService("change_ctl_mode", changeCtlModeCallback);
        static ros::ServiceServer change_arm_ctrl_mode_service = nh.advertiseService("change_arm_ctrl_mode", changeArmCtlModeCallback);
        static ros::ServiceServer change_AMBAC_ctrl_mode_service = nh.advertiseService("change_AMBAC_ctrl_mode", changeAMBACCtlModeCallback);
        static ros::ServiceServer control_end_hand_service = nh.advertiseService("control_end_hand", controlEndHandCallback);

        static ros::ServiceServer update_center_of_mass_position_service = nh.advertiseService("update_center_of_mass_position", updateCenterOfMassPositionCallback);
        static ros::ServiceServer get_center_of_mass_position_service = nh.advertiseService("get_center_of_mass_position", get_center_of_mass_position);

        static ros::ServiceServer change_hand_arm_poses_by_config_name_service = nh.advertiseService("change_hand_arm_poses_by_config_name", changeHandArmPosesByConfigNameCallback);
        
        static ros::ServiceServer robot_control_jodell_claw_service = nh_.advertiseService("control_jodell_claw_position", robotControlJodellClawCallback);

    }

    static bool changeHandArmPosesByConfigNameCallback(dynamic_biped::changeHandArmPosesByConfigName::Request &req,
                                                       dynamic_biped::changeHandArmPosesByConfigName::Response &res)
    {
        std::string config_name = req.config_name;
        robot_ptr->changeHandArmPosesByConfigName(config_name);
        res.result = true;
        return true;
    }

    static bool updateCenterOfMassPositionCallback(dynamic_biped::comPosition::Request  &req,
                                                   dynamic_biped::comPosition::Response &res)
    {
        // Extract the position from the request
        Eigen::Vector3d new_com;
        new_com << req.position.x, req.position.y, req.position.z;

        double acc = req.acc <= 0.0f ? 0.05f : req.acc;
        // Call the function to set the center of mass with the extracted value
        robot_ptr->setCenterOfMass(new_com,req.time,acc);

        // Set the response success flag based on the outcome
        res.success = true;

        return true; // Indicate that the callback executed successfully
    }

    static bool get_center_of_mass_position(dynamic_biped::getComPosition::Request  &req,
                                            dynamic_biped::getComPosition::Response &res)
    {
        Eigen::Vector3d com_eigen;
        robot_ptr->getCenterOfMass(com_eigen);

        geometry_msgs::Vector3 com_value;
        com_value.x = com_eigen[0];
        com_value.y = com_eigen[1];
        com_value.z = com_eigen[2];

        res.com_value = com_value;
        return true;
    }
    //
    static bool changeArmCtlModeCallback(dynamic_biped::changeArmCtrlMode::Request &req,
                                         dynamic_biped::changeArmCtrlMode::Response &res)
    {
        bool control_mode = req.control_mode;
        robot_ptr->switchArmCtrlMode(control_mode);
        res.result = true;
        return true;
    }

    // AMBAC系统（Active Mass Balance Auto-Control）：用于机动战士在太空中的移动和平衡控制。
    static bool changeAMBACCtlModeCallback(dynamic_biped::changeAMBACCtrlMode::Request &req,
                                           dynamic_biped::changeAMBACCtrlMode::Response &res)
    {
        bool control_mode = req.control_mode;
        robot_ptr->setAMBACReady(control_mode);
        res.result = true;
        return true;
    }

    static bool changeCtlModeCallback(dynamic_biped::srvchangeCtlMode::Request &req,
                                      dynamic_biped::srvchangeCtlMode::Response &res)
    {
        // TODO masterID
        robot_ptr->changeCtlMode((controlMode_t)req.control_mode);
        return true;
    }

    static bool clearPositionCMDCallback(dynamic_biped::srvClearPositionCMD::Request &req,
                                         dynamic_biped::srvClearPositionCMD::Response &res)
    {
        robot_ptr->clearPositionCMD();
        return true;
    }

    static bool changePhasesCallback(dynamic_biped::srvChangePhases::Request &req,
                                     dynamic_biped::srvChangePhases::Response &res)
    {
        // TODO: SET masterID
        std::string newphase_str = (!req.stateReq.empty() && phase_map.count(req.stateReq)) ? req.stateReq : "";
        std::string subState_str = (!req.subState.empty() && sub_phase_map.count(req.subState)) ? req.subState : "";

        if (!subState_str.empty() && !newphase_str.empty())
        {
            robot_ptr->changePhases(phase_map[newphase_str], sub_phase_map[subState_str]);
            return true;
        }
        else if (!subState_str.empty())
        {
            robot_ptr->changePhases(P_None, sub_phase_map[subState_str]);
            return true;
        }
        else if (!newphase_str.empty())
        {
            robot_ptr->changePhases(phase_map[newphase_str]);
            return true;
        }
        else
        {
            std::string warn_str = "UNKNOWN phase! req:" + newphase_str + ",subreq:" + subState_str;
            ROS_WARN("%s", warn_str.c_str());
            return false;
        }
    }

    static void walkCommandCallback(const dynamic_biped::walkCommand::ConstPtr &msg)
    {
        updateState();
        if (HDrobotState.phase != P_walk && msg->mode != 2)
        {
            ROS_WARN("NOT IN P_walk STATUS, ignore P_walk Command");
            return;
        }
        int controlmode = (int)robot_ptr->getCtlMode();
        if (controlmode != msg->mode)
        {
            ROS_WARN("Control mode does not match, msg.mode=%d | controll mode=%d", msg->mode, controlmode);
            return;
        }
        if (msg->mode == 0) // 0: position control
        {
            ROS_INFO("Received position control command: [%f, %f, %f]",
                     msg->values[0], msg->values[1], msg->values[2]);
            robot_ptr->positionCommand({msg->values[0], msg->values[1], msg->values[2]});
        }
        else if (msg->mode == 1) // 1: velocity control
        {
            ROS_INFO("Received velocity control command: [%f, %f, %f]",
                     msg->values[0], msg->values[1], msg->values[2]);
            robot_ptr->velocityCommand({msg->values[0], msg->values[1], msg->values[2]});
        }
        else if (msg->mode == 2) // 2: torque control
        {
            if (msg->values.size() != 4)
            {
                ROS_INFO("Received invalid step control command!");
                return;
            }
            else
            {
                ROS_INFO("Received step control command: [%f, %f, %f, %f]",
                         msg->values[0], msg->values[1], msg->values[2], msg->values[3]);
                robot_ptr->stepCommand(msg->values[0], {msg->values[1], msg->values[2], msg->values[3]});
            }
        }
    }

    static void armTargetPoseCallback(const dynamic_biped::armTargetPoses::ConstPtr &msg)
    {
        std::cout << "Received arm target poses" << std::endl;

        if (msg->values.empty() || msg->times.empty() || msg->values.size() != msg->times.size() * HighlyDynamic::NUM_ARM_JOINT)
        {
            ROS_WARN("Invalid armTargetPoses data. Empty values or different sizes.");
            return;
        }

        std::vector<double> times;
        std::vector<Eigen::VectorXd> target_poses;
        for (int i = 0; i < msg->times.size(); i++)
        {
            Eigen::VectorXd pose(HighlyDynamic::NUM_ARM_JOINT);
            for (int j = 0; j < HighlyDynamic::NUM_ARM_JOINT; j++)
            {
                pose[j] = msg->values[i * HighlyDynamic::NUM_ARM_JOINT + j];
            }
            times.push_back(msg->times[i]);
            target_poses.push_back(pose);
        }
        robot_ptr->changeArmPoses(times, target_poses);
        return;
    }

    // 回调函数处理接收到的 /etherCATJoint/motordata 消息
    static void joint2CommandDesiredCallback(const sensor_msgs::JointState::ConstPtr &msg)
    {
        // 处理收到的消息
        // ROS_INFO("Received joint_command_desired: [%f, %f, %f, %f, %f, %f,...]", msg->position[0], msg->position[1], msg->position[2], msg->position[3], msg->position[4], msg->position[5]);

        // 长度为 6
        // 检查 msg 的维度是否符合要求
        if (msg->position.size() == 14)
        {
            // 将 std::vector<double> 转换为 Eigen::VectorXd
            Eigen::VectorXd targetRosPosition(14);

            for (int i = 0; i < msg->position.size(); i++)
            {
                targetRosPosition[i] = msg->position[i];
            }

            // // 左手
            // targetRosPosition[0] = msg->position[0] ;
            // targetRosPosition[1] = msg->position[1] ;   // moveit电机输出轴方向和机器人Kuavo输出轴方向相反
            // targetRosPosition[2] = msg->position[2] ;   // moveit电机输出轴方向和机器人Kuavo输出轴方向相反

            // // 右手
            // targetRosPosition[3] = msg->position[3] ;
            // targetRosPosition[4] = msg->position[4] ;
            // targetRosPosition[5] = msg->position[5] ;

            // 调用 setROSArmPose
            robot_ptr->setROSArmPose(targetRosPosition);
        }
        else
        {
            ROS_WARN("Invalid arm_command_desired data. Expected 6 elements, but received %lu elements.", msg->position.size());
        }
    }

    static void robotHeadMotionDataCallback(const dynamic_biped::robotHeadMotionData::ConstPtr &msg)
    {
        // Check if the message has the correct number of elements
        if (msg->joint_data.size() == 2)
        {
            if (msg->joint_data[0] < -30 || msg->joint_data[0] > 30 || msg->joint_data[1] < -25 || msg->joint_data[1] > 25)
            {
                ROS_WARN("Invalid robot head motion data. Joint data must be in the range [-30, 30] and [-25, 25].");
                return;
            }
            ROS_INFO("Received robot head motion data joint_data: [%f, %f]", msg->joint_data[0], msg->joint_data[1]);

            robot_ptr->setHeadJointData(msg->joint_data);
        }
        else
        {
            ROS_WARN("Invalid robot head motion data. Expected 2 elements, but received %lu elements.", msg->joint_data.size());
        }
    }

    static void robotControlHandPositionCallback(const dynamic_biped::robotHandPosition::ConstPtr &msg)
    {
        // Check if the message has the correct number of elements
        if (msg->left_hand_position.size() != 6 || msg->right_hand_position.size() != 6)
        {
            ROS_WARN("Invalid hand positions size. Both left and right hand positions must have size 6.");
            return;
        }

        Eigen::VectorXd left_right_pos(12);
        for (int i = 0; i < 6; i++) {
            left_right_pos[i] = std::move(msg->left_hand_position[i]);
            left_right_pos[i + 6] = std::move(msg->right_hand_position[i]);
        }

        robot_ptr->setEndhand(left_right_pos);
    }

    static bool  robotControlJodellClawCallback(dynamic_biped::controlJodellClaw::Request &req,
                                                        dynamic_biped::controlJodellClaw::Response &res)
    {
        Eigen::VectorXd left_right_pos(12);
        left_right_pos.setZero();
        left_right_pos[0] = req.left_claw_position;
        left_right_pos[6] = req.right_claw_position;
        
        ROS_INFO("[ROS Service] Control Jodell Claw, left_claw_position:%d, right_claw_position:%d", 
            req.left_claw_position, req.right_claw_position);

        robot_ptr->setEndhand(left_right_pos);
        res.result = true;

        return true;
    }

    static bool controlEndHandCallback(dynamic_biped::controlEndHand::Request &req,
                                       dynamic_biped::controlEndHand::Response &res)
    {
        if (req.left_hand_position.size() != 6 || req.right_hand_position.size() != 6)
        {
            ROS_ERROR("Invalid hand positions size. Both left and right hand positions must have size 6.");
            res.result = false;
            return false;
        }

        auto isInRange = [](const std::vector<uint8_t> &positions)
        {
            return std::all_of(positions.begin(), positions.end(), [](uint8_t pos)
                               { return pos >= 0 && pos <= 100; });
        };

        if (!isInRange(req.left_hand_position) || !isInRange(req.right_hand_position))
        {
            ROS_ERROR("Invalid hand positions value. All positions value must be in the range [0, 100].");
            res.result = false;
            return false;
        }

        Eigen::VectorXd left_right_pos(12);
        for (int i = 0; i < 6; i++)
        {
            left_right_pos[i] = req.left_hand_position[i];
            left_right_pos[i + 6] = req.right_hand_position[i];
        }

        robot_ptr->setEndhand(left_right_pos);

        res.result = true;
        return true;
    }

    template<class MessageT>
    void buildPublisher(ros::Publisher &publisher, const std::string& topic, uint32_t queue_size, bool latch = false) 
    {
        publisher = nh_.advertise<MessageT>(topic, queue_size, latch);
    }

    void setUpROSParam(HighlyDynamic::JSONConfigReader &JSONConfigReader)
    {
        // Get the ROS parameters from the JSON configuration file
        try
        {   
            std::vector<std::string> end_effector_type = JSONConfigReader["EndEffectorType"];
            nh_.setParam("end_effector_type", end_effector_type);
            // ...
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }
        
    }
private:
    ros::NodeHandle nh_;
    ros::Subscriber CMD_sub;
    ros::Subscriber joint2_command_desired_sub;
    ros::Subscriber arm_target_poses_sub;
    ros::Subscriber robot_head_motion_data_sub;
    ros::Subscriber robot_control_hand_position_sub;
    ros::Time current_time_;

};

void resizeAndCopyVector(const Eigen::VectorXd &source, std::vector<double> &destination)
{
    destination.resize(source.size());
    for (int i = 0; i < source.size(); i++)
    {
        destination[i] = source[i];
    }
}

std::string getPhaseName(const int phase, const std::map<int, std::string> &phase_name_map)
{
    std::string est_main_phase_name = "Unknown";
    auto phase_name = phase_name_map.find(phase);
    if (phase_name != phase_name_map.end())
    {
        est_main_phase_name = phase_name->second;
    }
    return est_main_phase_name;
}

void ros_publish_robot_arm_q_v_vd(const RobotState_t &state_est, const ros::Publisher &robot_ros_publisher)
{
    dynamic_biped::robotArmQVVD msg;
    resizeAndCopyVector(state_est.arm_q, msg.q);
    resizeAndCopyVector(state_est.arm_v, msg.v);
    resizeAndCopyVector(state_est.arm_vd, msg.vd);

    robot_ros_publisher.publish(msg);
}

void ros_publish_robot_q_v_tau(const RobotState_t &state_est, const ros::Publisher &robot_ros_publisher)
{
    dynamic_biped::robotQVTau msg;

    resizeAndCopyVector(state_est.q, msg.q);
    resizeAndCopyVector(state_est.v, msg.v);
    resizeAndCopyVector(state_est.tau, msg.tau);

    robot_ros_publisher.publish(msg);
}

void publishRobotTorsoState(const RobotState_t &state_est, const ros::Publisher &publisher)
{
    dynamic_biped::robotTorsoState robot_torsor_state_msg;

    robot_torsor_state_msg.torsoR.x = state_est.torsoR.x();
    robot_torsor_state_msg.torsoR.y = state_est.torsoR.y();
    robot_torsor_state_msg.torsoR.z = state_est.torsoR.z();

    robot_torsor_state_msg.torsoRd.x = state_est.torsoRd.x();
    robot_torsor_state_msg.torsoRd.y = state_est.torsoRd.y();
    robot_torsor_state_msg.torsoRd.z = state_est.torsoRd.z();

    robot_torsor_state_msg.torsoRdd.x = state_est.torsoRdd.x();
    robot_torsor_state_msg.torsoRdd.y = state_est.torsoRdd.y();
    robot_torsor_state_msg.torsoRdd.z = state_est.torsoRdd.z();

    robot_torsor_state_msg.r.x = state_est.r.x();
    robot_torsor_state_msg.r.y = state_est.r.y();
    robot_torsor_state_msg.r.z = state_est.r.z();

    robot_torsor_state_msg.rd.x = state_est.rd.x();
    robot_torsor_state_msg.rd.y = state_est.rd.y();
    robot_torsor_state_msg.rd.z = state_est.rd.z();

    robot_torsor_state_msg.rdd.x = state_est.rdd.x();
    robot_torsor_state_msg.rdd.y = state_est.rdd.y();
    robot_torsor_state_msg.rdd.z = state_est.rdd.z();

    publisher.publish(robot_torsor_state_msg);
}

void publishRobotPhase(const RobotState_t &state_des, ros::Publisher &robot_phase_pub)
{
    dynamic_biped::robotPhase robot_phase_msg;
    robot_phase_msg.mainPhase = state_des.phase;
    robot_phase_msg.subPhase = state_des.sub_phase;

    robot_phase_pub.publish(robot_phase_msg);
}

void publishRobotHandPosition(ros::Publisher &robot_hand_position_pub)
{
    dynamic_biped::robotHandPosition robot_hand_position_msg;
    robot_hand_position_msg.left_hand_position.resize(6);
    robot_hand_position_msg.right_hand_position.resize(6);

    auto left_right_pos = robot_ptr->getEndhand();

    for (int i = 0; i < 6; i++)
    {
        robot_hand_position_msg.left_hand_position[i] = left_right_pos[i];
        robot_hand_position_msg.right_hand_position[i] = left_right_pos[i + 6];
    }

    robot_hand_position_msg.header.stamp = ros::Time::now();
    robot_hand_position_pub.publish(robot_hand_position_msg);
}

void ros_publish_robot_imu_gyro_acc(const ros::Publisher &publisher)
{
    if(robot_ptr == nullptr) {
        return;
    }

    dynamic_biped::robotImuGyroAcc msg;
    SensorData_t sensor_data = robot_ptr->getSensorData();

    msg.gyro.x = sensor_data.gyro.x();
    msg.gyro.y = sensor_data.gyro.y();
    msg.gyro.z = sensor_data.gyro.z();

    msg.acc.x = sensor_data.acc.x();
    msg.acc.y = sensor_data.acc.y();
    msg.acc.z = sensor_data.acc.z();

    publisher.publish(msg);
}

void ros_publish_robot_head_motor_position(const RobotState_t &state_est, const ros::Publisher &publisher)
{
    if(robot_ptr == nullptr) {
        return;
    }

    dynamic_biped::robotHeadMotionData msg;
    msg.joint_data.resize(state_est.head_joint_q.size());

    for (int i = 0; i<state_est.head_joint_q.size(); i++)    
        msg.joint_data[i] = state_est.head_joint_q[i];

    publisher.publish(msg);
}

template<typename T>
std::vector<T> extract_any_vector(const std::vector<std::any>& any_vector)
{
    std::vector<T> result;
    result.reserve(any_vector.size());  // Reserve space to avoid multiple allocations
    for (const auto& any_value : any_vector) {
        result.push_back(std::any_cast<T>(any_value));
    }
    return result;
}

void ros_publish_end_effector_state(const ros::Publisher &publisher)
{

    dynamic_biped::EndEffectorStateStamped msg;
    msg.header.stamp = ros::Time::now();

    auto state_ptr = robot_ptr->getEndEffectorStatePtr();
    if (state_ptr && !state_ptr->empty())
    {
        const auto& state_map = *state_ptr;

        try {
            msg.names = extract_any_vector<std::string>(state_map.at("names"));
            msg.positions = extract_any_vector<double>(state_map.at("positions"));
            msg.velocities = extract_any_vector<double>(state_map.at("velocities"));
            msg.torques = extract_any_vector<double>(state_map.at("torques"));
            msg.voltages = extract_any_vector<double>(state_map.at("voltages"));
            msg.status = extract_any_vector<double>(state_map.at("status"));
        } 
        catch (const std::bad_any_cast& e) {
            std::cerr << "Error: " << e.what() << std::endl;
        }
    }

    publisher.publish(msg);
}


int main(int argc, char *argv[])
{
    // Check Environment
    HighlyDynamic::env_utils::Environment robot_env;
    if(!robot_env.Init()) {
      return -1;
    }

    // Load Config
    std::string config_path = HighlyDynamic::env_utils::GetConfigFilePath();
    if(!HighlyDynamic::JSONConfigReader::getInstance().init(config_path)) {
      return -1;
    }

    ros::init(argc, argv, "HDrobot_node");
    ros::NodeHandle nh;
    bool cali_flag = false;
    bool setzero_flag = false;

    // Loop through command line arguments
    for (int i = 1; i < argc; ++i) { 
        if (std::strcmp(argv[i], "--cali") == 0) {
            cali_flag = true;
        } else if (std::strcmp(argv[i], "--setzero") == 0) {
            setzero_flag = true;
            // avoid the argument being passed to the robot
            argc -= 1;
            argv[i] = nullptr;
        }
    }
    
    if (cali_flag && setzero_flag && ROBOT_VERSION_INT >= 40)
    {
        auto result = call_Setzero(HighlyDynamic::env_utils::GetSourceRootPath(), HighlyDynamic::Setzero_script);
        if (result != 0)
        {
            std::cerr << "Failed to call Setzero script. Exiting..." << std::endl;
            return -1;
        }else
        {
            std::cout << "Setzero script called successfully. wait 10 seconds." << std::endl;
            sleep(10);
        }
    }

    HighlyDynamic::HighlyDynamicRobot robot;
    robot_ptr = &robot;
    // robot.traj_ptr->setRosArmTrue();
    // robot.listening_keyboard = false;
    HDrobot_node node(nh);
    node.setUpROSParam(HighlyDynamic::JSONConfigReader::getInstance());
    int ret = 0;
    robot.doMainAsync(argc, argv);
    // robot.switchArmCtrlMode(true);
    if (ret != 0)
    {
        std::cout << "robot start failed!\n";
        return -1;
    }
    ros::Rate rate(100);

    uint64_t low_rate_count = 0;
    

    bool head_joint_exist = robot.hasHeadJoint();

    if(head_joint_exist)
    { 
        // Create Topic Publisher: `robot_head_motor_position`
        node.buildPublisher<dynamic_biped::robotHeadMotionData>(
            node.robot_head_motor_position_pub,
            "robot_head_motor_position", 
            10);

        std::cout << "robot head joint exist, publish topic : robot_head_motor_position \n";    
    }
    else {
        std::cout << "robot head joint isn't exist, doesn't need to publish topic : robot_head_motor_position \n";    
    }

    while (ros::ok())
    {
        if (low_rate_count++ % 100 == 0)
        {
            RobotData robotData = robot.queryNewestRobotStates();
            RobotState_t state_des = robotData.state_des;
            RobotState_t state_est = robotData.state_est;

            ros_publish_robot_q_v_tau(state_est, node.robot_q_v_tau_pub);

            ros_publish_robot_arm_q_v_vd(state_est, node.robot_arm_q_v_vd_pub);

            publishRobotTorsoState(state_est, node.robot_torso_state_pub);

            publishRobotPhase(state_des, node.robot_phase_pub);

            publishRobotHandPosition(node.robot_hand_position_pub);

            ros_publish_robot_imu_gyro_acc(node.robot_imu_gyro_acc_pub);


            if(head_joint_exist) 
            {
                ros_publish_robot_head_motor_position(state_est, node.robot_head_motor_position_pub);
            }
        }

        ros_publish_end_effector_state(node.robot_end_effector_state_pub);

        ros::spinOnce();
        rate.sleep();
    }
}
