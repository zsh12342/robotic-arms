#pragma once

#include <iomanip>
#include <iostream>
#include "drake/systems/framework/leaf_system.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/solvers/solve.h"

#include <lcm/lcm-cpp.hpp>
#include "lcm_std_msgs/Float64.hpp"
#include "lcm_std_msgs/Float64MultiArray.hpp"
#include "drake/systems/controllers/linear_quadratic_regulator.h"

#include "utils.h"
#include "lcm_publish.h"
#include "robot_state.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
#include <csv_reader.h>
#include "config.h"
#include "robotStateTransform.h"

DECLARE_bool(real);

extern lcm::LCM lc_instance;
namespace HighlyDynamic
{
  extern std::vector<std::string> end_frames_name;
  static const std::vector<std::string> end_frames_name_full = {"torso", "l_foot_sole", "r_foot_sole", "l_hand_sole", "r_hand_sole"};

  using namespace drake;

  class PIDController
  {
  public:
    PIDController(const Eigen::VectorXd &Kp, const Eigen::VectorXd &Ki, const Eigen::VectorXd &Kd, double dt);
    explicit PIDController(double dt = 0.01);
    Eigen::VectorXd update(const Eigen::VectorXd &error);
    void setCoefficients(const Eigen::VectorXd &Kp, const Eigen::VectorXd &Ki, const Eigen::VectorXd &Kd);
    static Eigen::VectorXd readParamFromCSV(const std::string &filename, const std::string &paramName);
    void reset();

  private:
    Eigen::VectorXd Kp_;         // Proportional coefficients
    Eigen::VectorXd Ki_;         // Integral coefficients
    Eigen::VectorXd Kd_;         // Derivative coefficients
    double dt_;                  // Sampling time interval
    Eigen::VectorXd integral_;   // Integral term
    Eigen::VectorXd derivative_; // Derivative term
    Eigen::VectorXd prev_error_; // Previous error
  };
  class WholeBodyController
  {
  public:
    WholeBodyController(multibody::MultibodyPlant<double> *plant, multibody::MultibodyPlant<double> *plant_with_arm,
                        std::vector<std::string> contact_frames_name,
                        std::string model_instance_name = "none");
    void calArmsTau(RobotState_t &state_est_, RobotState_t &state_des_, Eigen::VectorXd &tau);
    void UpdateTau(RobotState_t &state_des, RobotState_t &state_est, Eigen::VectorXd &tau);

  private:
    Eigen::VectorXd qddDesire(const Eigen::VectorXd &qv,
                              const Eigen::VectorXd &q_des, const Eigen::VectorXd &v_des, const Eigen::VectorXd &vd_des,
                              Eigen::VectorXd kp, Eigen::VectorXd ki, Eigen::VectorXd kd);
    void UpdateMat(const Eigen::VectorXd &qv, const Contact_states contact_des);
    Eigen::VectorXd Tau(Eigen::VectorXd &qdd, Eigen::VectorXd &lambda);
    drake::symbolic::Expression V(Eigen::Matrix<double, 6, 1> &x_com, solvers::VectorXDecisionVariable &u_com,
                           Eigen::Vector3d &r_des, Eigen::Vector3d &rd_des, Eigen::Vector3d &rdd_des);
    void CreateQP(const Eigen::VectorXd &qv,
                  Eigen::Vector3d &r, Eigen::Vector3d &rd,
                  Eigen::VectorXd &q_des, Eigen::VectorXd &v_des, Eigen::VectorXd &vd_des,
                  Eigen::Vector3d &r_des, Eigen::Vector3d &rd_des, Eigen::Vector3d &rdd_des,
                  solvers::VectorXDecisionVariable &qdd,
                  solvers::VectorXDecisionVariable &lambda,
                  solvers::MatrixXDecisionVariable &beta,
                  solvers::VectorXDecisionVariable &u_com,
                  solvers::MathematicalProgram &prog);
    bool IsTracking(const Eigen::VectorXd &qv, const Eigen::VectorXd &qv_des, double threshold) const;

    // Member variables are omitted for brevity
    const double mu_ = 1.0;  // Coefficient of friction
    const uint32_t N_d_ = 4; // friction cone approximated as a i-pyramid
    const uint32_t N_f_ = 3; // contact force dimension
    const uint32_t com_dim_ = 3;
    const double epsilon = 1.0e-8;
    uint32_t N_c_, N_eq_; // N_c_:支撑脚的frame个数(0,2,4),N_eq_:非支撑脚着地frame个数/2

    multibody::MultibodyPlant<double> *plant_;
    multibody::MultibodyPlant<double> *plant_with_arm_;
    multibody::MultibodyPlant<double> *plant_arm_dyn_;
    multibody::MultibodyPlant<double> *full_body_plant_;
    std::unique_ptr<systems::Context<double>> plant_context_;
    std::unique_ptr<systems::Context<double>> plant_context_with_arm;
    std::unique_ptr<systems::Context<double>> plant_context_arm_dyn_;
    std::unique_ptr<systems::Context<double>> plant_context_full_body_;
    std::unique_ptr<systems::Context<double>> plant_context_full_body_des_;
    int32_t na_, nq_, nv_, na_with_arm, nq_with_arm, nv_with_arm;
    uint32_t nq_f_;
    uint32_t nv_f_;
    double dt_;
    std::vector<multibody::JointIndex> joint_indices_;
    std::vector<std::string> contact_frames_name_;
    Eigen::VectorXd joint_effort_limits_;

    Eigen::VectorXd kp_, ki_, kd_;
    double w_qdd_, w_V_, w_xdd_, w_rdd_;

    Eigen::MatrixXd A_, B_;
    Eigen::MatrixXd Q_, R_;
    Eigen::MatrixXd K_, S_;

    Eigen::MatrixXd Ba_;
    Eigen::MatrixXd Ba_inv_;
    Eigen::MatrixXd phi_, phi_T_f_, phi_T_a_;
    Eigen::MatrixXd H_, Hf_, Ha_;
    Eigen::MatrixXd C_, Cf_, Ca_;
    Eigen::MatrixXd Js_v_Ccm_;

    Eigen::Vector3d r_, rd_, rdd_;
    Eigen::Vector3d prev_rd_;

    Eigen::VectorXd tau_sol_;
    Eigen::VectorXd lambda_sol_;
    Eigen::VectorXd qdd_sol_ ;
    Eigen::VectorXd u_com_sol_;
    Eigen::MatrixXd beta_sol_;
    Eigen::VectorXd cost_value_;
    bool is_tracking_ = true;
    bool isParallelArm{false};
    Eigen::VectorXd q_failed_tracking;
    RobotState_t state_est_, state_est_st_;
    RobotState_t state_des_, state_des_st_;

    Eigen::VectorXd hand_kp;
    Eigen::VectorXd hand_ki;
    Eigen::VectorXd hand_kd;
    PIDController *hands_Controller_;
    RobotStateTransform *state_transform;
  };
  ;
} // namespace HighlyDynamic
