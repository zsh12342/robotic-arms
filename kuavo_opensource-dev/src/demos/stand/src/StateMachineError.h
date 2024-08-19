#pragma once
#include <FSMState.h>
#include <csignal>

class StateMachineError : public FSMState
{
public:
    StateMachineError(std::map<mainPhase_t, FSMState *> &fsm_map,
                  HighlyDynamic::StateEstimation *Estimate_ptr,
                  HighlyDynamic::Trajectory *traj_ptr,
                  HighlyDynamic::WholeBodyController *wbc_ptr) : FSMState(fsm_map, Estimate_ptr, traj_ptr, wbc_ptr)
    {
        m_current_phase = P_ERROR;
    }
    ~StateMachineError() {}

    // 状态规划
  void onPlan(RobotState_t &state_des, RobotState_t &state_est) override;
  void onCalTau(RobotState_t &state_des, RobotState_t &state_est, Eigen::VectorXd &actuation) override;
};
