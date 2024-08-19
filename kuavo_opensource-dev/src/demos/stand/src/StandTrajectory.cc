#include <StandTrajectory.h>
DECLARE_double(powerscale);

namespace HighlyDynamic
{
  StandTrajectory::StandTrajectory(multibody::MultibodyPlant<double> *plant, std::vector<std::string> end_frames_name,
                                   uint32_t nq_f, uint32_t nv_f) 
    : Trajectory(plant, end_frames_name, nq_f, nv_f)
  {
    p_x_des = 0;
    p_y_des = 0;
    L_y_T_des = 0;
    comH_pre = com_z;
    StepDuration = RobotConfig.getValue<double>("StepDuration");
    param_loader_ = env_utils::GetConfigFileParentPath().append("/param.csv");
    stablize_detecter = RobotConfig.getValue<int>("walk_stablizer_count");

    // RobotConfig.getValue<double> 是一个配置文件，根据当前机器人的型号即 ROBOT_VERSION 的值到 src/biped_v2/config/ 里面寻找对应的 kuavo.json
    // 这个 kuavo.json 是储存机器人控制参数的配置文件，比如 torsoY 就是在文件里面的一个配置值

    torsoY = RobotConfig.getValue<double>("torsoY") * TO_RADIAN;
    Yaw_des_W = torsoY; //躯干在 Yaw 角上的旋转角度

    torsoP = RobotConfig.getValue<double>("torsoP") * TO_RADIAN; // 躯干在 Pitch 角上的旋转角度

    com_z = RobotConfig.getValue<double>("com_z");  // 初始高度，质心距离地面的高度

    com_z = 0.78;
  }


  void StandTrajectory::planStand(RobotState_t &state_des, RobotState_t &state_est)
  {

    state_des.cm.setZero();
    double alpha = 0.1;
    Eigen::Vector3d diff_angle = (Eigen::Vector3d(0, torsoP, Yaw_des_W) - state_des.torsoR);
    diff_angle[2] = normalize_angle(diff_angle[2]);
    state_des.torsoRd = diff_angle * 1;
    state_des.torsoR += state_des.torsoRd * dt_;
    Eigen::Vector3d pre_rd = state_des.rd;
    state_des.rd = 0.5 * (Eigen::Vector3d(anchor_pos[0], anchor_pos[1], (anchor_pos[2] + com_z)) - state_des.r);
    state_des.rdd.setZero();
    state_des.r = state_des.r + state_des.rd * dt_;
    state_des.lfv.setZero();
    state_des.rfv.setZero();
    lfvel_ref.setZero();
    rfvel_ref.setZero();
    state_des.walk_contact = Double_contact;
    state_des.tau_ratio.segment(0, 6) << 1, 1, 1, 1, 1, 1;
    state_des.tau_ratio.segment(6, 6) << 1, 1, 1, 1, 1, 1;
  }

  void StandTrajectory::UpdateStand(RobotState_t &state_des, RobotState_t &state_est)
  {
    prev_state_des_ = state_des_;
    this->planStand(state_des_, state_est);
    this->doIK(state_des_);
    this->planArm(state_des_, state_est);
    this->planEndEffectors(state_des_, state_est);
    this->CalTau(state_des_, state_est);
    this->safeCheck(state_des_);
    state_des = state_des_;
    this->pubLCM(state_des_);
  }
  void StandTrajectory::CalTau(RobotState_t &state_des_, RobotState_t &state_est)
  {
    state_des_.control_modes[3] = MOTOR_CONTROL_MODE_POSITION;
    state_des_.control_modes[9] = MOTOR_CONTROL_MODE_POSITION;
    state_des_.control_modes[4] = MOTOR_CONTROL_MODE_POSITION;
    state_des_.control_modes[5] = MOTOR_CONTROL_MODE_POSITION;
    state_des_.control_modes[10] = MOTOR_CONTROL_MODE_POSITION;
    state_des_.control_modes[11] = MOTOR_CONTROL_MODE_POSITION;

    state_des_.tau_max[4] = motor_info.max_current[4];
    state_des_.tau_max[5] = motor_info.max_current[5];
    state_des_.tau_max[10] = motor_info.max_current[10];
    state_des_.tau_max[11] = motor_info.max_current[11];
    state_des_.tau_max[3] = motor_info.max_current[3];
    state_des_.tau_max[9] = motor_info.max_current[9];

  }
  void StandTrajectory::planArm(RobotState_t &state_des, RobotState_t &state_est)
  {
    Eigen::VectorXd desire_arm_q(NUM_ARM_JOINT);
    mtx_pose.lock();
    desire_arm_q << current_arm_pose;
    mtx_pose.unlock();
    desire_arm_q *= TO_RADIAN;
    state_des.arm_v = (desire_arm_q - state_des.arm_q) * 1;
    state_des.arm_q += state_des.arm_v * dt_;
  }
  void StandTrajectory::planEndEffectors(RobotState_t &state_des, RobotState_t &state_est)
  {
    if (!has_end_effectors)
      return;
    mtx_ef_data.lock();
    state_des.end_effectors = end_effectors_data;
    mtx_ef_data.unlock();
  }

}
