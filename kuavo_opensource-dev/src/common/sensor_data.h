#pragma once

#include <iostream>
#include "Eigen/Core"
#include "robot_state.h"

typedef struct
{
  Eigen::VectorXd joint_q;
  Eigen::VectorXd joint_v;
  Eigen::VectorXd joint_vd;
  Eigen::VectorXd joint_current;
  Eigen::Vector3d gyro;
  Eigen::Vector3d acc;
  Eigen::Vector3d free_acc;
  Eigen::Vector4d quat;
  Eigen::Vector3d gyro_W;
  Eigen::Vector3d acc_W;
  Eigen::Vector3d free_acc_W;
  Eigen::Vector4d quat_W;
  std::vector<EndEffectorInfo> end_effectors_data;
  struct timespec timestamp;
  struct timespec acc_timestamp;
  struct timespec gyro_timestamp;
  struct timespec quat_timestamp;
  // std::vector<EndEffectorInfo> endhand_effectors_data;
  Eigen::Vector2d head_joint_q;
  
  void resizeJoint(size_t n)
  {
    joint_q.resize(n);
    joint_v.resize(n);
    joint_vd.resize(n);
    joint_current.resize(n);
  }
} SensorData_t;

typedef struct
{
  Eigen::Vector3d omegaBody;
  Eigen::Vector3d omegaWorld;
  Eigen::Vector3d aBody;
  Eigen::Vector3d aWorld;
  Eigen::Vector3d Angu_xyz;
  Eigen::Vector3d VelBody;
  Eigen::Vector3d VelWorld;
  Eigen::Vector3d PosBody;
  Eigen::Vector3d PosWorld;
  Eigen::Vector4d quat;
  Eigen::Matrix3d rBody;
} _SensorOrientationData;
