#pragma once

#include <iostream>
#include <vector>

#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/geometry/scene_graph.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/lcm/drake_lcm.h"
#include <drake/systems/lcm/lcm_interface_system.h>
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/controllers/linear_quadratic_regulator.h"
#include "drake/systems/primitives/vector_log_sink.h"
#include "drake/common/eigen_types.h"
#include "robot_state.h"
#include "orientation_tools.h"

namespace drake
{
    void getTorqueFromPBC(drake::multibody::MultibodyPlant<double> *plant_,
                          std::vector<std::string> end_frames_name,
                          const EstimateState_t &state_est, const EstimateState_t &state_des,
                          Eigen::VectorXd &kp_gain, Eigen::VectorXd &kd_gain, Eigen::VectorXd &joint_index,
                          Eigen::VectorXd &u, int32_t na_, int32_t nq_, int32_t nv_);

} // namespace drake
