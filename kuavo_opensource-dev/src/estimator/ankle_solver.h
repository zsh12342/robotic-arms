#pragma once

#include <iostream>
#include <string>

#include <drake/common/eigen_types.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/multibody/inverse_kinematics/inverse_kinematics.h>
#include <drake/multibody/tree/linear_bushing_roll_pitch_yaw.h>
#include <drake/solvers/solve.h>
#include <drake/solvers/snopt_solver.h>

using namespace Eigen;
using namespace drake;

#define LEFT_ANKLE 0
#define RIGHT_ANKLE 1

namespace lower_leg
{

  class AnkleSolverSystem2 : public drake::systems::LeafSystem<double>
  {
  public:
    DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(AnkleSolverSystem2)

    AnkleSolverSystem2(const drake::multibody::MultibodyPlant<double> &plant);

    void CalcQ(VectorXd &q) const;
    void CalcState(Vector4d qv_ankle_arm, VectorXd &ankle_qv);

  private:
    int nq_;
    int nv_;
    int i_j_l_arm_;
    int i_j_r_arm_;
    int i_roll_;
    int i_pitch_;
    double l_Lkleft_;
    double l_Lkright_;
    const drake::multibody::MultibodyPlant<double> &plant_;
    std::unique_ptr<drake::systems::Context<double>> plant_context_;
  };

} // namespace lower_leg