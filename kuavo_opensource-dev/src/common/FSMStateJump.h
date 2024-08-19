#pragma once
#include <FSMState.h>

class FSMStateJump : public FSMState
{
public:
    FSMStateJump(std::map<mainPhase_t, FSMState *> &fsm_map,
                 HighlyDynamic::StateEstimation *Estimate_ptr,
                 HighlyDynamic::Trajectory *traj_ptr,
                 HighlyDynamic::WholeBodyController *wbc_ptr) : FSMState(fsm_map, Estimate_ptr, traj_ptr, wbc_ptr)
    {
        m_current_phase = P_jump;
    }
    ~FSMStateJump() {}

    // 状态规划
    void onPlan(RobotState_t &state_des, RobotState_t &state_est) override
    {
        traj_ptr_->UpdateJump(state_des, state_est);
    }
};
