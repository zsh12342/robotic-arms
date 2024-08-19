#ifndef _common_sys_h_
#define _common_sys_h_

#include <iostream>
#include <queue>

#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "lcm_std_msgs/Float64MultiArray.hpp"

namespace drake
{
  class VectorSender : public systems::LeafSystem<double>
  {
  public:
    DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(VectorSender)

    VectorSender(uint32_t size);

  private:
    void Output(const systems::Context<double> &context, lcm_std_msgs::Float64MultiArray *output) const;

    uint32_t _size;
  };
} // namespace drake

namespace drake
{
  class Delayer : public systems::LeafSystem<double>
  {
  public:
    DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Delayer)

    Delayer(uint32_t size, uint32_t delayPeriod);

  private:
    void Output(const systems::Context<double> &context, systems::BasicVector<double> *output) const;

    uint32_t _size;
    uint32_t _delayPeriod;
    mutable std::queue<Eigen::VectorXd> _queue;
  };
} // namespace drake

namespace drake
{
  class PidController : public systems::LeafSystem<double>
  {
  public:
    DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PidController)

    PidController(multibody::MultibodyPlant<double> *plant,
                  Eigen::VectorXd kp,
                  Eigen::VectorXd ki,
                  Eigen::VectorXd kd,
                  uint32_t nqf = 7,
                  uint32_t nvf = 6);

    const systems::InputPort<double> &get_state_input_port() const
    {
      return systems::LeafSystem<double>::get_input_port(0);
    }

    const systems::InputPort<double> &get_desire_input_port() const
    {
      return systems::LeafSystem<double>::get_input_port(1);
    }

    const systems::OutputPort<double> &get_control_output_port() const
    {
      return systems::LeafSystem<double>::get_output_port(0);
    }

    void setGain(Eigen::VectorXd kp, Eigen::VectorXd ki, Eigen::VectorXd kd);

    void setIntegralValue(systems::Context<double> &context, const Eigen::Ref<const Eigen::VectorXd> &value) const;

  private:
    void DoCalcTimeDerivatives(const systems::Context<double> &context,
                               systems::ContinuousState<double> *derivatives) const override;
    void Output(const systems::Context<double> &context, systems::BasicVector<double> *output) const;

    int32_t na_;
    int32_t nq_;
    int32_t nv_;
    multibody::MultibodyPlant<double> *_plant;
    Eigen::MatrixXd Ba;
    Eigen::VectorXd _kp;
    Eigen::VectorXd _ki;
    Eigen::VectorXd _kd;
    uint32_t nq_f_;
    uint32_t nv_f_;
  };
} // namespace drake

#endif
