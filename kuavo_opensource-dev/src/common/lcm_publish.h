#pragma once

#include <iostream>
#include <utils.h>
#include <gflags/gflags.h>
#include <lcm/lcm-cpp.hpp>
#include "EcDemoApp.h"

using JointParam_t = MotorParam_t;

DECLARE_bool(log);
DECLARE_bool(real);
DECLARE_bool(pub);
extern lcm::LCM lc_instance;
void lcmPublishValue(lcm::LCM *lc, std::string name, const double value);
void lcmPublishVector(lcm::LCM *lc, std::string name, const Eigen::VectorXd &vec);
void lcmPublishState(lcm::LCM *lc, std::string prefix,
                     const Eigen::VectorXd &q, const Eigen::VectorXd &v, const Eigen::VectorXd &vdot,
                     bool to_rpy = false);
void jointPublish(lcm::LCM *lcm_ptr, std::string prefix, JointParam_t *joint_data, size_t num_joint);
