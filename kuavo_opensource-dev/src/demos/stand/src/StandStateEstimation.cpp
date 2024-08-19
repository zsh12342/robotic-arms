#include "StandStateEstimation.h"
#include <unistd.h>
#include <gflags/gflags.h>
#include "utils.h"
#include "lcm_publish.h"
#include "lcm_std_msgs/Float64MultiArray.hpp"
#include "lcm_sensor_msgs/Imu.hpp"
#include "lcm_sensor_msgs/TimeReference.hpp"
#include "lcm_sensor_msgs/Vector3Stamped.hpp"
#include "lcm_sensor_msgs/JointState.hpp"
#include "lcm_sensor_msgs/JointCommand.hpp"
#include "hardware_plant.h"

DECLARE_double(dt);
DECLARE_double(realtime);
DECLARE_double(simulation_time);
DECLARE_bool(pub);
DECLARE_bool(real);
DECLARE_bool(cali);
DECLARE_uint32(traj);
DECLARE_bool(play_back_mode);

DECLARE_bool(rm_est);
DECLARE_bool(pub_real);
DECLARE_bool(cal_time);
DECLARE_bool(use_motion_cal);
DECLARE_bool(record_motion);

static lcm::LCM *lcm_ptr;

namespace HighlyDynamic
{
    StandStateEstimation::StandStateEstimation(drake::multibody::MultibodyPlant<double> *plant, drake::multibody::MultibodyPlant<double> *plant_with_arm, HighlyDynamic::HardwarePlant *hardware_ptr)
      : StateEstimation(plant, plant_with_arm, hardware_ptr)
    {
      hw_ptr = hardware_ptr;
      stablize_detecter = 500;
      lcm_ptr = &lc_instance;
    }

    StandStateEstimation::~StandStateEstimation()
    {
    }

    void StandStateEstimation::UpdateStateWithQV(RobotState_t &state)
    {
        Eigen::VectorXd qv(nq_ + nv_);
        qv << state.q, state.v;
        plant_->SetPositionsAndVelocities(plant_context_.get(), qv);
        state.torsoR = drake::math::RollPitchYawd(Eigen::Quaterniond(qv[0], qv[1], qv[2], qv[3])).vector();
        state.torsoRd = plant_->GetFrameByName(end_frames_name[END_FRAMES_TORSO]).CalcSpatialVelocityInWorld(*plant_context_.get()).rotational();

        state.r = plant_->CalcCenterOfMassPositionInWorld(*plant_context_);
        state.rd = plant_->CalcCenterOfMassTranslationalVelocityInWorld(*plant_context_);
        state.rdd = plant_->CalcBiasCenterOfMassTranslationalAcceleration(*plant_context_, drake::multibody::JacobianWrtVariable::kV,
                                                                          plant_->world_frame(),
                                                                          plant_->world_frame());

        drake::math::RigidTransformd foot_in_w;
        drake::multibody::SpatialVelocity<double> footv_in_w;
        foot_in_w = plant_->GetFrameByName(end_frames_name[END_FRAMES_L_FOOT_SOLE]).CalcPose(*plant_context_.get(), plant_->world_frame());
        footv_in_w = plant_->GetFrameByName(end_frames_name[END_FRAMES_L_FOOT_SOLE]).CalcSpatialVelocityInWorld(*plant_context_.get());
        state.lf << drake::math::RollPitchYawd(foot_in_w.rotation()).vector(), foot_in_w.translation();
        state.lfv = footv_in_w.get_coeffs();
        foot_in_w = plant_->GetFrameByName(end_frames_name[END_FRAMES_R_FOOT_SOLE]).CalcPose(*plant_context_.get(), plant_->world_frame());
        footv_in_w = plant_->GetFrameByName(end_frames_name[END_FRAMES_R_FOOT_SOLE]).CalcSpatialVelocityInWorld(*plant_context_.get());
        state.rf << drake::math::RollPitchYawd(foot_in_w.rotation()).vector(), foot_in_w.translation();
        state.rfv = footv_in_w.get_coeffs();

        drake::multibody::SpatialMomentum<double> L_WScm_W;
        L_WScm_W = plant_->CalcSpatialMomentumInWorldAboutPoint(*plant_context_, state.r);
        state.cm << L_WScm_W.rotational(), L_WScm_W.translational();
        L_WScm_W = plant_->CalcSpatialMomentumInWorldAboutPoint(*plant_context_, state.lf.segment(3, 3));
        state.lf_sm << L_WScm_W.rotational(), L_WScm_W.translational();
    }
    /**
     * @brief
     *
     * @param sensor_data
     * @param state_est
     * @param prev_state_est
     * @param free_acc
     * @param anchor_pos
     *
     * @note 先将IMU中的数据进行从imu device到base frame的变换。
     * 再融合关节角度和IMU角度数据，结合接触点绝对位置，获得机器人浮动基状态估计
     */
    void StandStateEstimation::UpdateQV(const SensorData_t &sensor_data, RobotState_t &state_est, const RobotState_t &prev_state_est,
                                   Eigen::Vector3d &anchor_pos)
    {
        state_est.vd.segment(0, 3) = (sensor_data.gyro_W - state_est.v.segment(0, 3)) / dt_;
        state_est.vd.segment(3, 3) = Eigen::Vector3d(sensor_data.free_acc_W[0], 0, sensor_data.free_acc_W[2]);
        state_est.q.segment(0, 4) = sensor_data.quat_W;
        state_est.v.segment(0, 3) = sensor_data.gyro_W;

        state_est.q.segment(7, na_) = sensor_data.joint_q.segment(0, na_);
        state_est.v.segment(6, na_) = sensor_data.joint_v.segment(0, na_);
        state_est.vd.segment(6, na_) = sensor_data.joint_vd.segment(0, na_);

        state_est.arm_q = sensor_data.joint_q.segment(na_, NUM_ARM_JOINT);
        state_est.arm_v = sensor_data.joint_v.segment(na_, NUM_ARM_JOINT);
        state_est.arm_vd = sensor_data.joint_vd.segment(na_, NUM_ARM_JOINT);

        Eigen::VectorXd qv(nq_ + nv_);
        qv << state_est.q, state_est.v;

        plant_->SetPositionsAndVelocities(plant_context_.get(), qv);

        drake::math::RigidTransformd lf_in_w = plant_->GetFrameByName(end_frames_name[END_FRAMES_L_FOOT_SOLE]).CalcPose(*plant_context_.get(), plant_->world_frame());
        drake::math::RigidTransformd rf_in_w = plant_->GetFrameByName(end_frames_name[END_FRAMES_R_FOOT_SOLE]).CalcPose(*plant_context_.get(), plant_->world_frame());

        drake::math::RigidTransformd base_in_lf = plant_->GetFrameByName(end_frames_name[END_FRAMES_TORSO]).CalcPose(*plant_context_.get(), plant_->GetFrameByName(end_frames_name[END_FRAMES_L_FOOT_SOLE]));
        drake::math::RigidTransformd base_in_rf = plant_->GetFrameByName(end_frames_name[END_FRAMES_TORSO]).CalcPose(*plant_context_.get(), plant_->GetFrameByName(end_frames_name[END_FRAMES_R_FOOT_SOLE]));

        Eigen::Vector3d base_in_w;

        state_est.v.segment(3, 3) = (base_in_w - state_est.q.segment(4, 3)) / dt_;
        state_est.q.segment(4, 3) = base_in_w;

        drake::math::RigidTransformd lf_in_base = plant_->GetFrameByName(end_frames_name[END_FRAMES_L_FOOT_SOLE]).CalcPose(*plant_context_.get(), plant_->GetFrameByName(end_frames_name[END_FRAMES_TORSO]));
        drake::math::RigidTransformd rf_in_base = plant_->GetFrameByName(end_frames_name[END_FRAMES_R_FOOT_SOLE]).CalcPose(*plant_context_.get(), plant_->GetFrameByName(end_frames_name[END_FRAMES_TORSO]));

        Eigen::Matrix<double, 6, 1> foot_in;
        Mat3 rBody = sensori_data_.rBody;
        foot_in << rBody.transpose() * lf_in_base.translation(), rBody.transpose() * rf_in_base.translation();
        lcmPublishVector(lcm_ptr, "state/est/base_in_foot", foot_in);

        // this qv base filter init in init @warning sensor_data_joint.acc 在 updateqv()被旋转需要注意
        qv << state_est.q, state_est.v;

        decoup_states = filter->baseFiltering(qv, sensor_data_joint.acc, dt_); // use quat, | one on one off ->qv
        state_est.q = qv.segment(0, nq_);
        state_est.v = qv.segment(nq_, nv_);
    }

    /**
     * @brief 相位更新
     *
     * @param state_des 预期值
     * @param state_est 估计值 被更新
     */
    void StandStateEstimation::UpdatePhase(const RobotState_t state_des, RobotState_t &state_est)
    {
      if (state_des_.phase != state_est.phase || state_est.sub_phase != state_des.sub_phase)
      {
        state_est.phase = state_des_.phase;
        state_est.sub_phase = state_des.sub_phase;
        state_est.phase_time = 0;
      }
    }
    void StandStateEstimation::Update(RobotState_t state_des, RobotState_t &state_est, SensorData_t sensor_data_motor, const drake::systems::Context<double> &g_plant_context)
    {
        state_with_arm = plant_with_arm_->GetPositionsAndVelocities(g_plant_context);

        this->Update(state_des, state_est, sensor_data_motor);
    }

    /**
     * @brief 状态估计 通过预期值和传感器值，根据上一次状态估计值输出当前状态估计值
     *
     * @param state_des 输入 预期值
     * @param state_est 输入 上一次估计值 输出 当前估计值
     * @param sensor_data 传感器数据 IMU + 关节信息
     */
    void StandStateEstimation::Update(RobotState_t state_des, RobotState_t &state_est, SensorData_t sensor_data_motor)
    {

        static struct timespec t0, t1, last_time;
        clock_gettime(CLOCK_MONOTONIC, &t0);
        hw_ptr->motor2joint(sensor_data_motor, sensor_data_joint);
        // save state_des_
        prev_state_des_ = state_des_;
        state_des_ = state_des;

        prev_state_est_ = state_est_;
        preProcessIMU(sensor_data_joint);
        JointPosCompensating(state_est_, sensor_data_joint);

        Eigen::VectorXd mhiproll(3);
        Eigen::VectorXd hiproll(3);
        mhiproll << sensor_data_joint.joint_q[1], sensor_data_joint.joint_v[1], 0;
        double v_torso = state_est_.v[4];
        double v_foot = 0;
        double l = 0.5;
        filter->HipRollFiltering(mhiproll, hiproll, v_torso, v_foot, l, dt_);

        UpdatePhase(state_des, state_est_);

        // 先将IMU中的数据进行从imu device到base frame的变换。再融合关节角度和IMU角度数据，结合接触点绝对位置，获得机器人浮动基状态估计。
        UpdateQV(sensor_data_joint, state_est_, prev_state_est_, p_landing_);

        // 根据state_est_中qpos和qvel，借助drake utilities进行动力学计算，计算state_est_中其余部分
        UpdateStateWithQV(state_est_);

        // 滤波函数
        Eigen::VectorXd r_vec_est(3 * 2);
        Eigen::VectorXd r_vec_(3 * 2);
        Eigen::Vector3d p_st(3);
        r_vec_ << state_est_.r, state_est_.rd;

        p_st = (state_est_.lf.segment(3, 3) + state_est_.rf.segment(3, 3)) / 2;

        filter->comFiltering(r_vec_est, r_vec_, p_st, dt_);
        state_est_.r_est = r_vec_est.segment(0, 3);
        state_est_.rd_est = r_vec_est.segment(3, 3);

        if (sensor_data_joint.end_effectors_data.size() > 0)
        {
            state_est_.end_effectors = sensor_data_joint.end_effectors_data;
        }
        state_est = state_est_;

        pubLCM(state_est_);
    }

    void StandStateEstimation::pubLCM(RobotState_t &state_est_pub, std::string str_pre)
    {
        lcmPublishState(lcm_ptr, str_pre, state_est_.q, state_est_.v, state_est_.vd);
        Eigen::Matrix<double, 6, 1> x_com, x_com_est;
        x_com << state_est_.r, state_est_.rd;
        x_com_est << state_est_.r_est, state_est_.rd_est;
        lcmPublishVector(lcm_ptr, str_pre + "/arms_q", state_est_.arm_q);
        lcmPublishVector(lcm_ptr, str_pre + "/arms_v", state_est_.arm_v);
        lcmPublishVector(lcm_ptr, str_pre + "/com/x", x_com);
        lcmPublishVector(lcm_ptr, str_pre + "/com/x_est", x_com_est);
        lcmPublishVector(lcm_ptr, str_pre + "/torsoR", state_est_.torsoR);
        lcmPublishVector(lcm_ptr, str_pre + "/torsoRd", state_est_.torsoRd);
        lcmPublishVector(lcm_ptr, str_pre + "/lfoot", state_est_.lf);
        lcmPublishVector(lcm_ptr, str_pre + "/lfootv", state_est_.lfv);
        lcmPublishVector(lcm_ptr, str_pre + "/rfoot", state_est_.rf);
        lcmPublishVector(lcm_ptr, str_pre + "/rfootv", state_est_.rfv);
        lcmPublishVector(lcm_ptr, str_pre + "/L/com", state_est_.cm);
        lcmPublishValue(lcm_ptr, str_pre + "/phase", state_est_.phase);
        lcmPublishValue(lcm_ptr, str_pre + "/subphase", state_est_.sub_phase);
        lcmPublishValue(lcm_ptr, str_pre + "/phase_time", state_est_.phase_time);
    }

    /**
     * @brief 接触力推测
     *
     * @param state
     * @param sensor_data
     * @param c2t_coeff 电流和力系数
     */
    void StandStateEstimation::JointPosCompensating(RobotState_t &state, SensorData_t sensor_data)
    {

        Eigen::VectorXd tau(na_);
        tau << sensor_data.joint_current.segment(0, na_);
        for (int i = 0; i < na_; i++)
        {
            tau[i] = sensor_data.joint_current[i] * motor_info.c2t_coeff[i];
        }
        JointPosCompensating(state, tau);
    }

    void StandStateEstimation::JointPosCompensating(RobotState_t &state, Eigen::VectorXd &tau)
    {
        StateEstimation::JointPosCompensating(state, tau);
        Eigen::MatrixXd contact_force = state.contact_force;
        lcmPublishVector(lcm_ptr, "state/est/contact_force", contact_force);
    }

} // namespace HighlyDynamic
