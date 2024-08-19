#pragma once

#include <iostream>
#include <array>
#include <variant>
#include <unordered_map>
#include "drake/systems/framework/leaf_system.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "actuators_interface.h"
#include "kalman_estimate.h"
#include <lcm/lcm-cpp.hpp>
#include "sensor_data.h"
#include "robot_state.h"
#include "imu_receiver.h"
#include "config.h"
#include "ruierman_actuator.h"
#include "ruiwo_actuator.h"
#include "jodell_claw_driver.h"
#include "dynamixel_interface.h"
#include "hand_sdk.h"

typedef lcm::LCM lcm_cpp;
namespace HighlyDynamic
{

#define MOTOR_OFFSET_L (-15 * M_PI / 180)
#define MOTOR_OFFSET_S (-15 * M_PI / 180)

  class HardwarePlant
  {  
    struct RuiWoJointData
      {
        std::vector<double> pos;
        std::vector<double> vel;
        std::vector<double> torque;
      };
  public:
    HardwarePlant(drake::multibody::MultibodyPlant<double> *plant,
                  double dt = 1e-3,
                  uint8_t control_mode = MOTOR_CONTROL_MODE_TORQUE,
                  uint16_t num_actuated = 0,
                  uint16_t nq_f = 7, uint16_t nv_f = 6);

    std::vector<double> head_joint_data_;
    void Update(RobotState_t state_des, Eigen::VectorXd actuation);
    void joint2motor(const RobotState_t &state_des_, const Eigen::VectorXd &actuation, Eigen::VectorXd &cmd_out);
    void motor2joint(SensorData_t sensor_data_motor, SensorData_t &sensor_data_joint);
    void GetC2Tcoeff(double *ret_c2t_coeff);
    bool readSensor(SensorData_t &sensor_data);
    bool HWPlantCheck();
    int8_t HWPlantInit(lcm_cpp &lcm_obj);
    SensorData_t sensorsInit();
    bool sensorsCheck();
    static void HWPlantDeInit();
    void jointMoveTo(std::vector<double> goal_pos, double speed, double dt);
    void qv_joint_to_motor(Eigen::VectorXd &no_arm_state, Eigen::VectorXd &with_arm_state, uint32_t nq_with_arm, uint32_t nq_no_arm);
    int8_t PDInitialize(Eigen::VectorXd &q0);
    void writeCommand(Eigen::VectorXd cmd_r, uint32_t na_r, uint8_t control_mode);
    void endEffectorCommand(std::vector<EndEffectorInfo> &end_effector_cmd);
    // void endhandEffectorCommand(std::vector<EndhandEffectorInfo> &end_hand_effector_cmd);
    void writeCommandbyLCM(Eigen::VectorXd cmd, uint32_t na, uint8_t control_mode);
    bool checkJointPos(JointParam_t *joint_data);
    void jointFiltering(JointParam_t *joint_data, double dt);
    inline void SetJointVelocity(const std::vector<uint8_t> &joint_ids, std::vector<JointParam_t> &joint_data);
    inline void SetJointTorque(const std::vector<uint8_t> &joint_ids, std::vector<JointParam_t> &joint_data);
    inline void SetJointPosition(const std::vector<uint8_t> &joint_ids, std::vector<JointParam_t> &joint_data);
    inline void GetJointData(const std::vector<uint8_t> &joint_ids, std::vector<JointParam_t> &joint_data);
    void reloadZeroOffsetConfig(std::vector<double> goal_pos);
    bool checkAndCreateTempOffsetPath();
    bool checkAndCopyOffsetFile();

    static bool hasHeadJoint();

    std::shared_ptr<std::unordered_map<std::string, std::vector<std::any>>> getEndEffectorStatePtr();

  private:
    bool initEndEffector();
  private:
    drake::multibody::MultibodyPlant<double> *plant_;
    std::unique_ptr<drake::systems::Context<double>> plant_context_;
    int32_t na_;
    int32_t nq_;
    int32_t nv_;
    uint32_t nq_f_;
    uint32_t nv_f_;
    double dt_;
    const drake::systems::CacheEntry *cache_actuation_{};
    const drake::systems::CacheEntry *cache_state_des_{};
    std::vector<std::string> end_frames_name_;
    uint8_t control_mode_;
    std::unique_ptr<KalmanEstimate> filter;
    bool Uncalibration_IMU = true;

    SensorData_t sensor_data_;
    Eigen::Vector3d free_acc_;
    Decoupled_states decoup_states;
    bool ori_init_flag;

    RobotState_t state_est_, prev_state_est_;
    RobotState_t state_des_, prev_state_des_;
    Eigen::Vector3d p_landing_;

    RobotState_t raw_state_est_, raw_prev_state_est_;
    Eigen::Vector3d raw_p_landing_;
    Eigen::Vector4d ankle_motor_offset;
    Eigen::Vector4d arm_ankle_motor_offset_;
    Eigen::VectorXd qv_;
    std::vector<JointParam_t> joint_data;
    std::vector<JointParam_t> joint_data_old;
    std::vector<JointParam_t> joint_cmd;
    std::vector<JointParam_t> joint_cmd_old;
    std::vector<uint8_t> joint_ids;
    std::vector<uint8_t> joint_tau_ids;
    std::vector<uint8_t> joint_vel_ids;
    std::vector<uint8_t> joint_pos_ids;
    std::vector<JointParam_t> joint_tau_cmd;
    std::vector<JointParam_t> joint_vel_cmd;
    std::vector<JointParam_t> joint_pos_cmd;
    std::vector<double> c2t_coeff;
    std::vector<double_t> min_joint_position_limits;
    std::vector<double_t> max_joint_position_limits;
    // std::vector<int> qiangnao_port;

    uint32_t num_joint;
    bool has_end_effectors{false};
    
    std::array<EndEffectorType, 2> end_effector_types_{EndEffectorType::none, EndEffectorType::none};
  };
  void Invt_imudate(SensorData_t &sensor_data);

} // namespace HighlyDynamic
