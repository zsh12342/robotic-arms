#ifndef _STATE_ESTIMATION_H_
#define _STATE_ESTIMATION_H_

#include "drake/systems/framework/leaf_system.h"
#include "drake/multibody/plant/multibody_plant.h"

#include "hardware_plant.h"
#include "robot_state.h"
#include "kalman_estimate.h"
#include "sensor_data.h"
#include "config.h"
#include "lcm/lcm-cpp.hpp"
#include "motionClient.h"
#define UPDATE_DEBUG true

#define END_FRAMES_TORSO 0
#define END_FRAMES_L_FOOT_SOLE 1
#define END_FRAMES_R_FOOT_SOLE 2

namespace HighlyDynamic
{

    class StateEstimation
    {
    protected:
        // drake::systems::Context<double> *plant_context_;
        drake::multibody::MultibodyPlant<double> *plant_;
        drake::multibody::MultibodyPlant<double> *plant_with_arm_;
        std::unique_ptr<drake::systems::Context<double>> plant_context_;
        HighlyDynamic::HardwarePlant *hw_ptr;

        std::vector<std::string> end_frames_name{"torso", "l_foot_sole", "r_foot_sole"};

        bool ori_init_flag;
        uint8_t initial_count{0};
        Eigen::Vector4d _quat_ini_inv;
        Eigen::Vector4d stand_ini_yaw_quat_;
        _SensorOrientationData sensori_data_;
        int32_t na_, nq_, nv_, na_with_arm, nq_with_arm, nv_with_arm;
        double dt_;
        double total_mass_;
        double V_takeoff_;
        Eigen::VectorXd v_prev;
        Eigen::VectorXd v_arm_prev;
        Eigen::VectorXd state_with_arm;
        Eigen::VectorXd base_Q,base_R;

        RobotState_t state_est_, prev_state_est_;
        RobotState_t state_des_, prev_state_des_;
        bool Uncalibration_IMU = false;
        std::unique_ptr<KalmanEstimate> filter; // 滤波器
        std::unique_ptr<KalmanEstimate> filter2;

        PeriodicData stablize_detecter;

        Decoupled_states decoup_states;

        Eigen::Vector3d p_landing_;
        SensorData_t sensor_data_joint;
        MotionCaptureClient *motionCapture_ptr;

#if UPDATE_DEBUG
        /* debug values */
        Eigen::VectorXd check_q;
        int error_count = 0;
#endif
        /* update function */
        void UpdatePhase(const RobotState_t state_des, RobotState_t &state_est);
        void UpdateQV(const SensorData_t &sensor_data, RobotState_t &state_est, const RobotState_t &prev_state_est,
                      Eigen::Vector3d &anchor_pos);
        void UpdateQVwithMotion(const SensorData_t &sensor_data, RobotState_t &state_est, const RobotState_t &prev_state_est,
                                Eigen::Vector3d &anchor_pos);
        void UpdateStateWithQV(RobotState_t &state);

    public:
        void updateStateWithSimQV(drake::multibody::MultibodyPlant<double> *plant, Eigen::VectorXd &state_with_arm, RobotState_t &state_est_, RobotState_t &state_des_);
        void Update(RobotState_t state_des, RobotState_t &state_est, SensorData_t sensor_data_motor);
        void Update(RobotState_t state_des, RobotState_t &state_est, SensorData_t sensor_data_motor, const drake::systems::Context<double> &g_plant_context);

        void Initialize(RobotState_t &state_est, SensorData_t sensor_data_motor, const Eigen::VectorXd q0);
        void JointPosCompensating(RobotState_t &state, SensorData_t sensor_data);
        void JointPosCompensating(RobotState_t &state, Eigen::VectorXd &tau);
        void JointPosCompensating(RobotState_t state, SensorData_t sensor_data, double *c2t_coeff);
        void preProcessIMU(SensorData_t &sensor_data);
        void pubLCM(RobotState_t &state_est_pub, std::string str_pre = "state");
        StateEstimation(drake::multibody::MultibodyPlant<double> *plant, drake::multibody::MultibodyPlant<double> *plant_with_arm, HighlyDynamic::HardwarePlant *hardware_ptr);
        ~StateEstimation();
    };

}

#endif // _STATE_ESTIMATION_H_
