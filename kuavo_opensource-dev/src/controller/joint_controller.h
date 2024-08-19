#pragma once
#include "Eigen/Core"

#include <drake/systems/framework/leaf_system.h>
#include <drake/systems/framework/context.h>
#include <drake/multibody/plant/multibody_plant.h>

using namespace Eigen;
using namespace drake;

namespace lower_leg
{

  class JointController2 : public drake::systems::LeafSystem<double>
  {
  public:
    DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(JointController2)

    JointController2(multibody::MultibodyPlant<double> &plant);
    void CalcArmState(Eigen::Vector4d &lower_qv, Eigen::Vector4d &ankle_motor_qv) const;
    void CalcArmStateQv(Eigen::Vector4d &lower_qv, Eigen::VectorXd &ankle_motor_qv) const;

  private:
    int nq_;
    int nv_;
    int i_j_l_arm_;
    int i_j_r_arm_;
    int i_roll_;
    int i_pitch_;
    int i_j_r_link,i_j_l_link;
    double l_Lkleft_;
    double l_Lkright_;
    double l_Armleft_;
    double l_Armright_;
    double qO_Armleft_; // this is "capital ou", not "zero"
    double qO_Armright_;
    double init_l_link_xz,init_l_link_yz,init_r_link_xz,init_r_link_yz;
    multibody::MultibodyPlant<double> &plant_;
    std::unique_ptr<systems::Context<double>> plant_context_;
  };
} // namespace lower_leg
