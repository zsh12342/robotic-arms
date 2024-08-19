#pragma once
#include <FSMState.h>

class FSMStateSquat : public FSMState
{
public:
    FSMStateSquat(std::map<mainPhase_t, FSMState *> &fsm_map,
                  HighlyDynamic::StateEstimation *Estimate_ptr,
                  HighlyDynamic::Trajectory *traj_ptr,
                  HighlyDynamic::WholeBodyController *wbc_ptr) : FSMState(fsm_map, Estimate_ptr, traj_ptr, wbc_ptr)
    {
        m_current_phase = P_squat;
    }
    ~FSMStateSquat() {}

    // 状态规划
    void onPlan(RobotState_t &state_des, RobotState_t &state_est) override
    {
        traj_ptr_->UpdateSquat(state_des, state_est);
    }
};
