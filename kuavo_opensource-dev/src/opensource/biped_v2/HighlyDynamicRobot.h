#ifndef HighlyDynamicRobot_H
#define HighlyDynamicRobot_H
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
#include "orientation_tools.h"
#include "savedata.h"
#include "utils.h"
#include "forceDisturber.h"
#include "plantIK.h"
#include "common_sys.h"
#include "traj.h"
#include "controller.h"
#include "hardware_plant.h"
#include "Trajectory.h"
#include "sim_sensor.h"
#include "state_estimation.h"

#include "FSMState.h"
#include "FSMStateJump.h"
#include "FSMStateSquat.h"
#include "FSMStateStand.h"
#include "FSMStateWalk.h"
#include "FSMStateError.h"
#include "config.h"
#include "robotStateStorage.h"
#define WHIT_THREAD_NUM 3

namespace HighlyDynamic
{
    void sigintHandler(int sig);
    static Eigen::VectorXd q_initial;
    static const std::string traj_file("data/biped_v2_qv_traj.csv");
    static std::vector<double> initial_joint_pos{0.4, 0.4};
    static std::vector<std::string> initial_joint_name{"l_knee_pitch_joint", "r_knee_pitch_joint"};

    class HighlyDynamicRobot
    {
    public:
        bool th_runing = false;
        bool listening_keyboard = true;
        bool AMBAC_ready = false;

        RobotState_t state_des;
        RobotState_t state_est;

        HighlyDynamicRobot();
        void initialState(multibody::MultibodyPlant<double> *plant, systems::Context<double> *plant_context,
                          std::vector<std::string> &initial_joint_name, std::vector<double> &initial_joint_pos);
        void buildMultibodyPlant();
        void initialFSM();
        void initialRobot();
        void real_init_wait();
        int initializeSimulatorAndThreads();
        int doMain(int argc, char *argv[]);
        void simStep(RobotState_t &state_des_, Eigen::VectorXd &actuation_);
        virtual int doMainAsync(int argc, char *argv[]);
        void getRobotState(RobotState_t &robotState);

        void setCenterOfMass(const Eigen::Vector3d& new_com, double time = 0.0f, double acc = 0.05f);
        void getCenterOfMass(Eigen::Vector3d& com_out) const;

        void updateHeadJointData();
        void changePhases(mainPhase_t new_phase = P_None, subPhase_t new_sub_phase = sub_phase_none)
        {
            traj_ptr->changePhases(new_phase, new_sub_phase);
        }
        void positionCommand(PositionDelta positionDelta)
        {
            traj_ptr->positionCommand(positionDelta);
        }
        void clearPositionCMD()
        {
            traj_ptr->clearPositionCMD();
        }
        void velocityCommand(VelocityData velocityData)
        {
            traj_ptr->velocityCommand(velocityData);
        }
        void stepCommand(uint32_t num_step, Eigen::Vector3d step_cmd)
        {
            traj_ptr->stepCommand(num_step, step_cmd);
        }
        void changeCtlMode(controlMode_t cm)
        {
            traj_ptr->changeCtlMode(cm);
            std::cout << "control_mode: " << controlMode_name_map[cm] << std::endl;
        }

        void setROSArmPose(Eigen::VectorXd targetRosPosition)
        {
            traj_ptr->setROSArmPose(targetRosPosition);
        }

        controlMode_t getCtlMode()
        {
            return traj_ptr->getCtlMode();
        }

        void setEndhand(Eigen::VectorXd left_right_pos)
        {
            traj_ptr->setEndhand(left_right_pos);
        }

        Eigen::VectorXd getEndhand()
        {
            Eigen::VectorXd left_right_pos(12);
            traj_ptr->getEndhand(left_right_pos);
            return left_right_pos;
        }
        void setHeadJointData(std::vector<double> head_joint_data)
        {
            head_joint_data_ = head_joint_data;
        }
        inline std::shared_ptr<std::unordered_map<std::string, std::vector<std::any>>> getEndEffectorStatePtr()
        {
            return hw_ptr->getEndEffectorStatePtr();
        }
        void changeArmPoses(const std::vector<double> &times, const std::vector<Eigen::VectorXd> &targets)
        {
            traj_ptr->changeArmPoses(times, targets);
        }
        void changeHandArmPosesByConfigName(const std::string &config_name)
        {
            traj_ptr->changeHandArmPoses(config_name);
        }
        void armCoMIK(Eigen::VectorXd &arm_xyzabc, Eigen::VectorXd &arm_joint_state)
        {
            // q:  0, 70, 0, 0, 0, 0, 0    rpyxyz: 1.22, 0, 0, 0, 0.76768, 0.13135,   -1.22, 0, 0, 0, -0.76768, 0.13135;
            // q: -40, 0, 0, 0, 0, 0, 0     rpyxyz: 0, -0.698, 0, 0.34, 0.264, 0.655,   0, -0.698, 0, 0.34, -0.264, 0.655;
            // q: -90, 0, 0, 0, 0, 0, 0     rpyxyz: 0, -1.57, 0, 0.518, 0.2645, 1.071,  0, -1.57, 0, 0.518, -0.2645, 1.071;
            // q: -90, 0, 0, -90, 0, 0, 0   rpyxyz : -3.14, 0, 3.14, 0.173, 0.264, 1.424,   -3.14159, 0, 3.14159, 0.173, -0.264, 1.423

            Eigen::VectorXd all_q(33);
            auto bool_ik = traj_ptr->armCoMIK(arm_xyzabc, all_q);
            if (bool_ik)
            {
                arm_joint_state = all_q.segment(19, 14);
            }
        }

        Eigen::VectorXd armFK()
        {
            Eigen::VectorXd arm_rpyxyz(12);
            arm_rpyxyz = traj_ptr->armFK();
            return arm_rpyxyz;
        }

        RobotData queryNewestRobotStates();
        void switchArmCtrlMode(bool rosArmMode);
        void setAMBACReady(bool value);
        void SetsetEndEffectors(Eigen::Vector2d target_left_right_pos);

        const SensorData_t& getSensorData();

        bool hasHeadJoint();

    protected:
        virtual void state_thread_func();
        virtual void control_thread_func();
        virtual void plan_thread_func();
        virtual void keyboard_thread_func();
        void MPC_thread_func();
        static void sigintHandler(int sig);
        std::thread state_thread;
        std::thread control_thread;
        std::thread plan_thread;

        std::thread keyboard_thread;
        HighlyDynamic::Trajectory *traj_ptr;
        HighlyDynamic::WholeBodyController *wbc_ptr;
        HighlyDynamic::HardwarePlant *hw_ptr;
        drake::multibody::MultibodyPlant<double> *g_plant;
        drake::multibody::MultibodyPlant<double> *g_plant_with_arm;
        drake::systems::Context<double> *g_plant_context;
        drake::systems::Context<double> *g_plant_context_with_arm;
        drake::systems::Simulator<double> *g_simulator;
        systems::DiagramBuilder<double> builder;
        std::unique_ptr<drake::systems::Diagram<double>> diagram;
        std::unique_ptr<systems::Context<double>> diagram_context;
        systems::lcm::LcmInterfaceSystem *lcm;
        geometry::SceneGraph<double> *scene_graph;

        SimSensor *sim_sensors_ptr;
        StateEstimation *Estimate_ptr;

        struct timespec next_time;
        std::mutex mtx_est;
        std::mutex mtx_des;
        std::mutex mtx_cv;
        std::mutex mtx_fsm;
        std::condition_variable cv;
        uint16_t thread_cnt = 0;
        uint16_t thread_all_cnt = 0;

        Eigen::VectorXd actuation, actuation_sim;

        uint32_t step_count = 0;
        char Walk_Command;
        Eigen::VectorXd q_initial;

        uint32_t na_, nq_, nv_, na_with_arm, nq_with_arm, nv_with_arm;

        FSMState *fsm_ptr;
        std::map<mainPhase_t, FSMState *> FSMStateMap;
        CSVParamLoader *hand_param_ptr;
        Eigen::VectorXd arms_init_pos;
        ThreadSync start_sync_, end_sync_;

        ThreadSafeDataStorage *robot_state_storge;

        std::vector<double> head_joint_data_;

        SensorData_t    sensor_data_;
        std::mutex      mtx_sensor_data_;
    
    };

    extern HighlyDynamicRobot *global_robot_ptr;

}
#endif
