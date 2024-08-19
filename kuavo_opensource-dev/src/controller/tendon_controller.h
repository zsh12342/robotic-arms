#pragma once
#include "Eigen/Core"

#include <drake/systems/framework/leaf_system.h>
#include <drake/systems/framework/context.h>
#include <drake/multibody/plant/multibody_plant.h>

using namespace Eigen;
using namespace drake;

namespace lower_leg
{

  class TendonController : public drake::systems::LeafSystem<double>
  {
  public:
    DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TendonController)

    TendonController(multibody::MultibodyPlant<double> &plant);
    void CalcTau(Eigen::Vector4d &ankle_q, Eigen::Vector2d &lower_tau, Eigen::Vector2d &ankle_motor_tau) const;
    void CalcLambda(Eigen::VectorXd &lower_q, Eigen::Vector2d &lower_tau) const;

  private:
    int nq_;
    int nv_;
    int na_;
    int i_j_l_arm_;
    int i_j_r_arm_;
    int i_roll_;
    int i_pitch_;
    double l_Lkleft_;
    double l_Lkright_;
    int in_roll_;
    int in_pitch_;
    int in_l_rocker_arm_;
    int in_r_rocker_arm_;
    const drake::multibody::MultibodyPlant<double> &plant_;
    std::unique_ptr<drake::systems::Context<double>> plant_context_;
  };

} // namespace lower_leg
