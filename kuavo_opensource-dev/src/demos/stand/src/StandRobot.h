#ifndef ROBANSTAND_H
#define ROBANSTAND_H
#include <HighlyDynamicRobot.h>
#include <StateMachineStand.h>
#include <StateMachineError.h>
#include <StandTrajectory.h>
#include <StandStateEstimation.h>

#define WHIT_THREAD_NUM 3

namespace HighlyDynamic
{
  class StandRobot : public HighlyDynamicRobot
  {
  public:
    StandRobot();
    void initialFSM();
    void plan_thread_func();
    void simStep(RobotState_t&, Eigen::VectorXd&);
    void state_thread_func();
    void control_thread_func();
    int initializeSimulatorAndThreads();
    void real_init_wait();
    void initialState(multibody::MultibodyPlant<double> *plant, systems::Context<double> *plant_context,
                                  std::vector<std::string> &initial_joint_name, std::vector<double> &initial_joint_pos);

    int doMainAsync(int, char**);
    int doMain(int argc, char *argv[]);

  protected:
    ThreadSync start_sync_, end_sync_;
  };
}
#endif
