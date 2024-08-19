#ifndef Traj_H
#define Traj_H
#include <ncurses.h>
#include <thread>
#include <chrono>
#include <future>
#include <iostream>
#include <unistd.h>
#include <signal.h>
#include <time.h>
#include <fstream>
#include <vector>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <gflags/gflags.h>
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/geometry/scene_graph.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/lcm/drake_lcm.h"
#include <drake/systems/lcm/lcm_interface_system.h>
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/controllers/linear_quadratic_regulator.h"
#include "drake/systems/primitives/vector_log_sink.h"
#include "drake/common/eigen_types.h"
#include "Predict_Fp.hpp"
// #include "state_estimation.h"
#include "utils.h"
#include "forceDisturber.h"
#include "plantIK.h"
#include "pb_controller.h"
#include "common_sys.h"
#include "traj.h"
#include "controller.h"
#include "imu_receiver.h"
#include <queue>
#include <fstream>
#include <sstream>
#include "csv_reader.h"
// #include "hardware_plant.h"
#include "config.h"
#include "robotStateTransform.h"
#include "Interpolator.h"
DECLARE_double(dt);
DECLARE_double(realtime);
DECLARE_double(simulation_time);
DECLARE_bool(pub);
DECLARE_bool(real);
DECLARE_uint32(traj);

namespace HighlyDynamic
{
    struct MPC_DATA
    {
        planner_to_mpc_data to_mpc_update;
        mpc_to_planner_data to_planner;
        int mpc_flag;
        bool send_flag;
        bool change_foot_flag;
    };

    // for jump
    static double et2f_vz_comk = 0;

    struct PositionDelta
    {
        double dx;
        double dy;
        double vyaw;
        friend std::ostream &operator<<(std::ostream &out, const PositionDelta &p)
        {
            out << "dx: " << p.dx << " dy: " << p.dy << " vyaw: " << p.vyaw;
            return out;
        }
    };
    struct VelocityData
    {
        double vx;
        double vy;
        double vyaw;
        friend std::ostream &operator<<(std::ostream &out, const VelocityData &p)
        {
            out << "vx: " << p.vx << " vy: " << p.vy << " vyaw: " << p.vyaw;
            return out;
        }
    };
    struct StepCmd
    {
        uint32_t num_step;
        Eigen::Vector3d step_cmd;
        friend std::ostream &operator<<(std::ostream &out, const StepCmd &p)
        {
            out << "num_step: " << p.num_step << " step_cmd: " << p.step_cmd;
            return out;
        }
    };

    class Trajectory
    {
    public:
        Trajectory(multibody::MultibodyPlant<double> *plant, std::vector<std::string> end_frames_name,
                   uint32_t nq_f = 7, uint32_t nv_f = 6);
        ~Trajectory()
        {
            th_runing = false;
        }

        void Initialize(RobotState_t &state_des, RobotState_t &state_est, const Eigen::VectorXd q0);

        void UpdateStateWithQV(RobotState_t &state);
        void UpdateStatePositionsWithQV(RobotState_t &state);

        static void Update_fp(MPC_DATA &mpc_data);
        void VelocityPositionControl(RobotState_t &state_est, RobotState_t &state_des, Eigen::Vector3d &Vel_des);
        void changePhases(mainPhase_t new_phase = P_None, subPhase_t new_sub_phase = sub_phase_none);
        void positionCommand(PositionDelta positionDelta);              // 接收位置控制差值输入
        void stepCommand(uint32_t num_step, Eigen::Vector3d &step_cmd); // 接收步态控制输入

        void clearPositionCMD();              // 清除当前位置差异、停止行走
        void changeCtlMode(controlMode_t cm); // 更改控制模式
        controlMode_t getCtlMode();           // 获取控制模式
        VelocityData velocityCommand(VelocityData velocityData);   // 接收速度控制输入

        void UpdateStand(RobotState_t &state_des, RobotState_t &state_est);
        void UpdateWalk(RobotState_t &state_des, RobotState_t &state_est);
        void UpdateSquat(RobotState_t &state_des, RobotState_t &state_est);
        void UpdateJump(RobotState_t &state_des, RobotState_t &state_est);

        void doIK(RobotState_t &state_des_);

        bool armCoMIK(Eigen::VectorXd &arm_xyzabc, Eigen::VectorXd &all_q);
        Eigen::VectorXd armFK();

        void doIK_st(RobotState_t &state_des_st);
        void do_cm_anlyIK(RobotState_t &state_des_);
        void do_cm_anlyIK_st(RobotState_t &state_des_st_);
        void CalTau(RobotState_t &state_des_, RobotState_t &state_est);
        void safeCheck(RobotState_t &state_des_);
        void pubLCM(RobotState_t &state_des, const std::string &prefix_str = "desire");
        // 从文件读取手臂姿态
        void changeArmPose(int index);

        // 通用接口，设置手臂姿态的接口，组合动作的序列, 自动插值
        void changeArmPoses(const std::vector<double> &times, const std::vector<Eigen::VectorXd> &targets);

        // 从自带配置文件读取和设置手臂和手指姿态,一个动作
        void changeHandArmPose(int index);
        
        // 从文件读取一系列手臂姿态,组合动作
        void changeHandArmPoses(std::string filename);

        void setCenterOfMass(const Eigen::Vector3d& new_com, double time = 0.0f, double acc = 0.05f);

        // ROS位置设置，不插值
        void setROSArmPose(Eigen::VectorXd targetRosPosition);
        void setRosArmTrue();
        void setRosArmFalse();
        void setEndEffectors(Eigen::Vector2d left_right_pos);
        // 通用接口，设置手指姿态的接口
        void setEndhand(Eigen::VectorXd left_right_pos);
        void getEndhand(Eigen::VectorXd &left_right_pos);
        bool ifOpenRosArm = false; // 默认关闭ROS手臂规划
        void updateTransform(const RobotState_t &state_W_des, const RobotState_t &state_W_est);

        void processErrors(const RobotState_t &state_est);
        mainPhase_t prasePhase(const RobotState_t &state_est);
        
        EndEffectorType getEndEffectorType();
        
        Eigen::Vector3d position_cmd;
        Eigen::Vector3d vel_cmd;
        StepCmd step_cmd;
        MPC_DATA mpc_data_global;
        RobotStateTransform *stateTransformer;
    private:
        // 静态互斥锁，用于同步对共享资源的访问
        static std::mutex mtx_Update_fp;
    protected:
        bool th_runing{true};
        void planStand(RobotState_t &state_des, RobotState_t &state_est);
        void planSquatQuick(RobotState_t &state_des, RobotState_t &state_est);
        void planSquat(RobotState_t &state_des, RobotState_t &state_est);
        void planWalk(RobotState_t &state_des, RobotState_t &state_est);
        void planJump(RobotState_t &state_des, RobotState_t &state_est);
        void getWalkCmd(int step_num, double &Vx_cmd, double &Yaw_cmd);
        void planArm(RobotState_t &state_des_, RobotState_t &state_est);
        void planEndEffectors(RobotState_t &state_des, RobotState_t &state_est);
        void planEndhandEffectors(RobotState_t &state_des, RobotState_t &state_est);
        void MPC_thread_func();
        void getXDesire(const RobotState_t &state_des, Eigen::VectorXd &x_d, Eigen::VectorXd &xd_d, Eigen::VectorXd &xdd_d);
        inline void getQvInput(const RobotState_t &prev_state_des, Eigen::VectorXd &q_in, Eigen::VectorXd &v_in)
        {
            const int n_q_fb = plant_full_body_->num_positions();
            const int n_v_fb = plant_full_body_->num_velocities();
            const int n_q = plant_->num_positions();
            const int n_v = plant_->num_velocities();
            const int arm_dof = n_q_fb - n_q;

            // auto qv_fb = plant_full_body_->GetPositionsAndVelocities(*plant_full_body_context_.get());

            q_in.segment(0, n_q) = prev_state_des.q;
            // q_in.segment(n_q, arm_dof) = qv_fb.segment(n_q, arm_dof);
            q_in.segment(n_q, arm_dof) = prev_state_des.arm_q;
            v_in.segment(0, n_v) = prev_state_des.v;
            // v_in.segment(n_v, arm_dof) = qv_fb.tail(arm_dof);
            v_in.segment(n_v, arm_dof) = prev_state_des.arm_v;
        }

        inline void setFullBodyQV(const RobotState_t &state_des)
        {
            // 使用目标值更新为上肢的状态
            auto qv = plant_->GetPositionsAndVelocities(*plant_context_.get());
            const int nq = plant_->num_positions();
            const int nv = plant_->num_velocities();
            const int nq_fb = plant_full_body_->num_positions();
            const int nv_fb = plant_full_body_->num_velocities();
            const int arm_dof = nq_fb - nq;

            Eigen::VectorXd qv_full_body = Eigen::VectorXd::Zero(nq_fb + nv_fb);
            qv_full_body.segment(0, nq) = qv.segment(0, nq);                  // foot
            qv_full_body.segment(nq_fb - arm_dof, arm_dof) = state_des.arm_q; // arm
            qv_full_body.segment(nq_fb, nv) = qv.segment(nq, nv);
            qv_full_body.tail(arm_dof) = state_des.arm_v;
            plant_full_body_->SetPositionsAndVelocities(plant_full_body_context_.get(), qv_full_body);
        }

        PIDController Postion_Controller_;
        char Walk_Command;
        multibody::MultibodyPlant<double> *plant_;
        std::unique_ptr<systems::Context<double>> plant_context_;
        multibody::MultibodyPlant<double> *plant_full_body_;
        std::unique_ptr<systems::Context<double>> plant_full_body_context_;
        int32_t na_;
        int32_t nq_;
        int32_t nv_;
        uint32_t nq_f_;
        uint32_t nv_f_;
        double dt_;
        std::vector<std::string> end_frames_name_;
        std::vector<std::string> end_frames_name_full_body_;
        Eigen::Vector3d gravity_vec_;
        double total_mass_;
        double iota;
        // ThreadPool update_fp_thread_pool;
        std::shared_ptr<CoMIK> com_ik_;
        std::shared_ptr<CoMVIK> comv_ik_;
        std::shared_ptr<AnalyticalIK> anly_ik_;
        std::shared_ptr<AnalyticalIK> anly_ik_full_body_;
        double comH_pre;
        Eigen::Vector3d r_pre, rd_pre, lf_pre, rf_pre, lf_ik_pre, rf_ik_pre;
        Eigen::Vector3d lf_contact, rf_contact, lf_contact_des, rf_contact_des, rf_contact_des_W, lf_contact_des_W;
        double At_x, At_y, Qt, Cov_rpx_stToe_stTD0;
        double Lx_stToe_kf_stTD0, Ly_stToe_kf_stTD0, sigma_Ly_stToe_stTD0, sigma_Lx_stToe_stTD0;
        bool flag_LFst_err, flag_RFst_err;
        Eigen::Vector3d LFst_err, RFst_err, st_pos_des;

        trajectories::PiecewisePolynomial<double> q_pp_, v_pp_, vd_pp_;
        double time_{0};
        RobotState_t state_des_, state_des_st, state_est_st, prev_state_des_, prev_state_des_st_, state_des_4cons;
        double L_x_T_des_k1, L_y_T_des_k1, p_sol, t_sol, L_y_T_des, V_y_T_des_k1;
        Eigen::Vector3d P_y_des, p_t_ref, p0_t_ref, v0_t_ref, v_t_ref, a_t_ref;
        double StepDuration, t_remain;
        int step_num;
        double walk_swing_arm_degree{5};
        double x_x_T_est_w, x_y_T_est_w, p_x_w_des, p_y_w_des, x_y_T_est;
        double p_y_des, p_x_des, p_y_des_ref;
        double ankle_tau_p, ankle_tau_r;
        Eigen::Vector3d rfvel_ref, lfvel_ref, rfrotvel_ref, lfrotvel_ref, rf_ref, lf_ref, rfrot_ref, lfrot_ref;
        Eigen::Vector3d sw_vel_des, sw_vd_des, st_vel_des, st_vd_des;
        Eigen::Vector3d st_pos_ini, st_pos_ini_W;
        double sw_yaw_init, st_yaw_init, torso_yaw_init, sw_yaw_init_W, st_yaw_init_W, torso_yaw_init_W;
        Eigen::VectorXd stance_foot_pose;
        multibody::SpatialMomentum<double> L_t_est, L_t_smoothed;

        Eigen::Vector3d V_des_next, Width_des_next;
        double Vx_des, Vy_des, Yaw_des_st, Yaw_des_W, Vx_cmd, Vy_cmd, Yaw_cmd;
        planner_to_mpc_data to_mpc_update;
        mpc_to_planner_data to_planner;
        std::shared_ptr<Predict_Fp> Predict_fp;
        std::mutex mtx_cmd, mtx_phase, mtx_pose, mtx_walk_pose, mtx_hand_arm_pose;
        CSVParamLoader param_loader_;
        Eigen::Vector3d VelocityLimit;
        Eigen::Vector3d Vel_des;
        Eigen::Vector3d current_position_;
        int stop_status = -1;
        std::vector<std::vector<double>> lf_vels; // 存储左脚两次落地间的速度值
        std::vector<std::vector<double>> rf_vels;
        Eigen::Vector3d avg_vel_;
        mainPhase_t phase_des;
        subPhase_t sub_phase_des;
        controlMode_t controlMode;
        Eigen::Vector3d target_pos_w;
        Vec2 lip_ufp_wrt_st;
        bool time_run = false;
        bool isExitingWalk = false;
        // for jump
        Eigen::Vector3d torso_rot_des_;
        Eigen::Vector3d rdd_cmd_;
        Eigen::Vector3d rd_takeoff_des_;
        double theta_des_;
        double ll_, lu_, lu_des_;
        uint8_t flags;
        uint32_t count_jump_{0};
        double robot_total_mass_;
        double V_touch_down;
        double V_takeoff_;
        double leg_length{0.68};
        PeriodicData stablize_detecter;

        Eigen::Vector3d anchor_pos, anchor_pos_st; // 落足点
        Eigen::Vector3d anchor_lf;                 // 落地左脚
        Eigen::Vector3d anchor_rf;                 // 落地左脚
        Eigen::VectorXd tmp_vec;
        Eigen::Vector2d prev_mpc_output;
        bool mpc_succ_flag{false};

        Eigen::VectorXd pbc_kp;
        Eigen::VectorXd pbc_kd;
        CSVParamLoader *static_arm_hand_pose_ptr, *pose_hand_arm_param_ptr, *pose_hand_arm_delay_ptr;
        Eigen::VectorXd current_arm_pose_degree, stand_arm_init_pose_, walk_arm_pose, walk_arm_init_pose_, walk_hand_arm_pose, walk_hand_arm_delay_pose;

        uint16_t nq_f{7}, nv_f{6};
        double torsoP;
        double StepH;
        double com_x_ini{0.0};
        double com_y_ini{0.0};
        double com_z;      // 初始高度
        double com_time;
        double com_acc;
        double com_x = 0.0;
        double com_y = 0.0;
        bool is_com_values_updated = false;

        double com_z_jump; // 跳跃初始高度
        double StepWith;
        double torsoY;
        double Yawcmd_lim{170 * deg2rad};
        double dYaw_lim{2 * deg2rad};
        bool swing_arm{false};

        double walk_stablizer_threshold;
        std::thread mpc_thread;
        std::mutex mtx_mpcdata;
        bool has_end_effectors{false};
        EndEffectorType end_effectors_type_{EndEffectorType::none};
        // bool has_endhand_effectors{false};
        std::vector<EndEffectorInfo> end_effectors_data;
        std::map<double, std::vector<EndEffectorInfo>> end_effectors_data_map;
        // std::vector<EndhandEffectorInfo> endhand_effectors_data;
        std::mutex mtx_ef_data;
        SmoothInterpolator *com_interpolator;
        SmoothInterpolator *arm_interpolator;
        bool is_arm_pose_updated{true};
        std::vector<Eigen::VectorXd> multi_arm_poses;
        target_pose_data_t target_arm_pose_data_;
        int MPC_solver_num;
    };
    constexpr double last_arm_pose_frame_duration = 1.0;
}
#endif
