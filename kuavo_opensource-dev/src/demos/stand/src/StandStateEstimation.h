#ifndef _STANDSTATEESTIMATION_H_
#define _STANDSTATEESTIMATION_H_

#include <state_estimation.h>

namespace HighlyDynamic
{

  class StandStateEstimation : public StateEstimation
  {
  public:
    StandStateEstimation(drake::multibody::MultibodyPlant<double> *plant, drake::multibody::MultibodyPlant<double> *plant_with_arm, HighlyDynamic::HardwarePlant *hardware_ptr);
    ~StandStateEstimation();

    void UpdatePhase(const RobotState_t state_des, RobotState_t &state_est);
    void UpdateQV(const SensorData_t &sensor_data, RobotState_t &state_est, const RobotState_t &prev_state_est,
                  Eigen::Vector3d &anchor_pos);
    void UpdateQVwithMotion(const SensorData_t &sensor_data, RobotState_t &state_est, const RobotState_t &prev_state_est,
                            Eigen::Vector3d &anchor_pos);
    void UpdateStateWithQV(RobotState_t &state);

    void updateStateWithSimQV(drake::multibody::MultibodyPlant<double> *plant, Eigen::VectorXd &state_with_arm, RobotState_t &state_est_, RobotState_t &state_des_);
    void Update(RobotState_t state_des, RobotState_t &state_est, SensorData_t sensor_data_motor);
    void Update(RobotState_t state_des, RobotState_t &state_est, SensorData_t sensor_data_motor, const drake::systems::Context<double> &g_plant_context);

    void JointPosCompensating(RobotState_t &state, SensorData_t sensor_data);
    void JointPosCompensating(RobotState_t &state, Eigen::VectorXd &tau);
    void pubLCM(RobotState_t &state_est_pub, std::string str_pre = "state");

  };

}

#endif // _STATE_ESTIMATION_H_
