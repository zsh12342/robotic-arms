#pragma once

#include <iostream>
#include "drake/systems/framework/leaf_system.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/common/trajectories/piecewise_polynomial.h"

namespace drake
{
  void epp2qvpp(multibody::MultibodyPlant<double> *plant, systems::Context<double> *plant_context,
                   std::vector<std::string> &end_frames_name,
                   trajectories::PiecewisePolynomial<double> &e_pp,
                   trajectories::PiecewisePolynomial<double> &q_pp,
                   trajectories::PiecewisePolynomial<double> &v_pp,
                   double dt = 1.0e-2,
                   double tol = 1.0e-7, double solver_tol = 1.0e-7);

  void qvpp2csv(const char *fine_name,
                   uint32_t nq, uint32_t nv, double dt,
                   trajectories::PiecewisePolynomial<double> &q_pp,
                   trajectories::PiecewisePolynomial<double> &v_pp);

  void qvpp2csvOfJoint(const char *fine_name,
                          uint32_t nq, uint32_t nv, double dt,
                          trajectories::PiecewisePolynomial<double> &q_pp,
                          trajectories::PiecewisePolynomial<double> &v_pp);

  void csv2pp(const std::string fine_name, double dt, bool skip_header,
              trajectories::PiecewisePolynomial<double> &pp);

  void csv2epp(const std::string fine_name, uint32_t n_ep, double dt, bool skip_header,
               trajectories::PiecewisePolynomial<double> &e_pp);

  void csv2qvpp(const std::string fine_name,
                   uint32_t nq, uint32_t nv, double dt,
                   trajectories::PiecewisePolynomial<double> &q_pp,
                   trajectories::PiecewisePolynomial<double> &v_pp);

} // namespace drake
