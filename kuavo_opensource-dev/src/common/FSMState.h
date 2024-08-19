#pragma once
#include <iostream>
#include <map>
#include <memory>
#include <robot_state.h>
#include "sensor_data.h"
#include "Trajectory.h"
#include "state_estimation.h"

enum FSMstatus
{
    FSM_init,
    FSM_running,
    FSM_error,
    FSM_exiting
};

// 有限状态机基类
class FSMState
{
public:
    FSMState(std::map<mainPhase_t, FSMState *> &fsm_map,
             HighlyDynamic::StateEstimation *Estimate_ptr,
             HighlyDynamic::Trajectory *traj_ptr,
             HighlyDynamic::WholeBodyController *wbc_ptr) : m_fsm_map(fsm_map), Estimate_ptr_(Estimate_ptr), traj_ptr_(traj_ptr), wbc_ptr_(wbc_ptr) {}
    ~FSMState() {}

    // 进入状态处理
    virtual void Enter(RobotState_t &state_est)
    {
        m_requested_phase = P_None;
    }

    // 状态规划
    virtual void onPlan(RobotState_t &state_des, RobotState_t &state_est)
    {
    }

    // 状态估计
    virtual void onEstimation(RobotState_t state_des, RobotState_t &state_est, SensorData_t sensor_data)
    {
    }

    // 力矩计算
    virtual void onCalTau(RobotState_t &state_des, RobotState_t &state_est, Eigen::VectorXd &actuation)
    {
        wbc_ptr_->UpdateTau(state_des, state_est, state_des.tau);
        for (uint32_t i = 0; i < state_des.tau.size(); i++)
        {
            actuation[i] = state_des.tau[i];
        }
    }

    // 退出处理
    virtual void Exit() {}

    // 错误处理
    virtual void onError(RobotState_t &state_des, RobotState_t &state_est)
    {
        m_requested_phase = P_ERROR;
    }

    // 检查状态机是否需要修改
    virtual bool checkFsmChange(RobotState_t &state_est, FSMState *&next_FSM_ptr)
    {
        mainPhase_t new_phase = traj_ptr_->prasePhase(state_est);
        m_requested_phase = new_phase;
        // do something to change phase, if need...
        // ...
        if (m_current_phase != m_requested_phase)
        {
            std::cout << "\033[34mChange main phase to \033[3;1m" << phase_name_map[m_requested_phase] << "\033[0m\n";
            next_FSM_ptr = m_fsm_map[m_requested_phase];
            this->Exit();
            if (next_FSM_ptr != nullptr)
                next_FSM_ptr->Enter(state_est);
            return true;
        }
        return false;
    }

    // 直接修改状态机, 用于回放功能
    virtual bool DirectChange(mainPhase_t &new_phase, FSMState *&next_FSM_ptr)
    {
        m_requested_phase = new_phase;
        // do something to change phase, if need...
        // ...
        if (m_current_phase != m_requested_phase)
        {
            std::cout << "\033[34mChange main phase to \033[3;1m" << phase_name_map[m_requested_phase] << "\033[0m\n";
            next_FSM_ptr = m_fsm_map[m_requested_phase];
            this->Exit();
            // if (next_FSM_ptr != nullptr)
            //     next_FSM_ptr->Enter(state_est);
            return true;
        }
        return false;
    }

protected:
    mainPhase_t m_requested_phase = P_None;
    mainPhase_t m_current_phase = P_None;
    FSMstatus m_fsm_status = FSM_running;
    std::map<mainPhase_t, FSMState *> &m_fsm_map;
    RobotState_t m_state_des, m_state_est;

    HighlyDynamic::StateEstimation *Estimate_ptr_;
    HighlyDynamic::Trajectory *traj_ptr_;
    HighlyDynamic::WholeBodyController *wbc_ptr_;
};
