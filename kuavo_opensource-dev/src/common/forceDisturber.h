#ifndef __forceDisturber_h_
#define __forceDisturber_h_

#include "unistd.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/multibody/plant/externally_applied_spatial_force.h"

namespace drake
{
  class ForceDisturber : public systems::LeafSystem<double>
  {
  public:
    DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ForceDisturber)

    ForceDisturber(multibody::BodyIndex bodyIndex, Eigen::Matrix<double, 6, 1> forceVec,
                   double startTime = 10, double disturbDuration = 0.1, double disturbPeriod = 4);

  private:
    void Output(const systems::Context<double> &context,
                std::vector<multibody::ExternallyAppliedSpatialForce<double>> *result) const;

    multibody::BodyIndex _bodyIndex;
    Eigen::Matrix<double, 6, 1> _forceVec;
    double _startTime, _disturbDuration, _disturbPeriod;
  };
} // namespace drake

#endif
