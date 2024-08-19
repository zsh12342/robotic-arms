#pragma once
#include <FSMState.h>
#include <csignal>

class FSMStateError : public FSMState
{
public:
    FSMStateError(std::map<mainPhase_t, FSMState *> &fsm_map,
                  HighlyDynamic::StateEstimation *Estimate_ptr,
                  HighlyDynamic::Trajectory *traj_ptr,
                  HighlyDynamic::WholeBodyController *wbc_ptr) : FSMState(fsm_map, Estimate_ptr, traj_ptr, wbc_ptr)
    {
        m_current_phase = P_ERROR;
    }
    ~FSMStateError() {}

    // 状态规划
    void onPlan(RobotState_t &state_des, RobotState_t &state_est) override
    {
        usleep(500000);
        std::raise(SIGINT);
        // TODO: need do something while in error state...
    }
    void onCalTau(RobotState_t &state_des, RobotState_t &state_est, Eigen::VectorXd &actuation) override
    {
        // wbc_ptr_->UpdateTau(state_des, state_est, state_des.tau);
        state_des.tau.setZero();
        state_des.tau_max.setZero();
        actuation << state_des.tau;
    }
};
