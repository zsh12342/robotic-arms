#ifndef STANDTRAJECTORY_H
#define STANDTRAJECTORY_H
#include <Trajectory.h>
DECLARE_double(dt);
DECLARE_double(realtime);
DECLARE_double(simulation_time);
DECLARE_bool(pub);
DECLARE_bool(real);
DECLARE_uint32(traj);

namespace HighlyDynamic
{
  class StandTrajectory : public Trajectory
  {
  public:
    StandTrajectory(multibody::MultibodyPlant<double> *plant, std::vector<std::string> end_frames_name,
               uint32_t nq_f = 7, uint32_t nv_f = 6);
    ~StandTrajectory()
    {
      th_runing = false;
    }
  private:
    void planStand(RobotState_t &state_des, RobotState_t &state_est);
    void UpdateStand(RobotState_t &state_des, RobotState_t &state_est);
    void CalTau(RobotState_t &state_des_, RobotState_t &state_est);
    void planArm(RobotState_t &state_des, RobotState_t &state_est);
    void planEndEffectors(RobotState_t &state_des, RobotState_t &state_est) ;
  };
}
#endif
