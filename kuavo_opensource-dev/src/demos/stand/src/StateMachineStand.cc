#include <StateMachineStand.h>

void StateMachineStand::onPlan(RobotState_t &state_des, RobotState_t &state_est)
{
  traj_ptr_->UpdateStand(state_des, state_est);
}

void StateMachineStand::onCalTau(RobotState_t &state_des, RobotState_t &state_est, Eigen::VectorXd &actuation)
{
  wbc_ptr_->UpdateTau(state_des, state_est, state_des.tau);
  for (uint32_t i = 0; i < state_des.tau.size(); i++)
  {
    actuation[i] = state_des.tau[i];
  }
}
