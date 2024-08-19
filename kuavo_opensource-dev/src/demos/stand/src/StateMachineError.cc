#include <StateMachineError.h>
// 状态规划
void StateMachineError::onPlan(RobotState_t &state_des, RobotState_t &state_est)
{
  usleep(500000);
  std::raise(SIGINT);
  // TODO: need do something while in error state...
}
void StateMachineError::onCalTau(RobotState_t &state_des, RobotState_t &state_est, Eigen::VectorXd &actuation)
{
  // wbc_ptr_->UpdateTau(state_des, state_est, state_des.tau);
  state_des.tau.setZero();
  state_des.tau_max.setZero();
  actuation << state_des.tau;
}
