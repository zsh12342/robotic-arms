#ifndef VELOCITY_SMOOTH_INTERPOLATOR_H
#define VELOCITY_SMOOTH_INTERPOLATOR_H
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
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "utils.h"
#include <Eigen/Dense>
// #include <trajectories/piecewise_polynomial.h>
#include <iostream>
#include <mutex>

enum InterpolationType
{
    ACCELERATION = 0,
    TIME_SERIES = 1
};

struct target_pose_data_t
{
    std::vector<double> times;
    std::vector<Eigen::VectorXd> positions; // 统一使用radian
    bool is_updated_ = false;
    std::mutex mtx;
    bool isUpdated()
    {
        std::lock_guard<std::mutex> lock(mtx);
        return is_updated_;
    }
    target_pose_data_t &operator=(const target_pose_data_t &other)
    {
        std::lock_guard<std::mutex> lock(mtx);

        times = other.times;
        positions = other.positions;
        is_updated_ = other.is_updated_;

        return *this;
    }

    void setPosition(const Eigen::VectorXd &newPosition)
    {
        std::lock_guard<std::mutex> lock(mtx);
        positions.clear();
        positions.push_back(newPosition);
        is_updated_ = true;
    }

    void setTime(double newTime)
    {
        std::lock_guard<std::mutex> lock(mtx);
        times.clear();
        times.push_back(newTime);
        is_updated_ = true;
    }
    void setTimePosition(const double newTimes, const Eigen::VectorXd &newPositions)
    {
        std::lock_guard<std::mutex> lock(mtx);
        times.clear();
        times.push_back(newTimes);
        positions.clear();
        positions.push_back(newPositions);
        is_updated_ = true;
    }

    void setValue(const std::vector<double> &newTimes, const std::vector<Eigen::VectorXd> &newPositions)
    {
        std::lock_guard<std::mutex> lock(mtx);
        times = newTimes;
        positions = newPositions;
        is_updated_ = true;
    }

    void getValues(std::vector<double> &outTimes, std::vector<Eigen::VectorXd> &outPositions)
    {
        std::lock_guard<std::mutex> lock(mtx);
        outTimes = times;
        outPositions = positions;
        is_updated_ = false;
    }

    std::vector<double> getTimes()
    {
        std::lock_guard<std::mutex> lock(mtx);
        is_updated_ = false;
        return times;
    }
    std::vector<Eigen::VectorXd> getPoses()
    {
        std::lock_guard<std::mutex> lock(mtx);
        is_updated_ = false;
        return positions;
    }
};

class SmoothInterpolator
{
public:
    SmoothInterpolator(const Eigen::VectorXd &start_pos,
                       const Eigen::VectorXd &start_vel,
                       const Eigen::VectorXd &target_pos,
                       double acceleration = 0.05,
                       double dt = 0.001,
                       const std::string& name = "default");
    double getCurrentTime();
    // void get(double t_step, Eigen::VectorXd& pos, Eigen::VectorXd& vel);
    void get(double t_step, Eigen::VectorXd *pos = nullptr, Eigen::VectorXd *vel = nullptr, Eigen::VectorXd *acc = nullptr);

    // 按顺序获取pos, vel, acc
    void get(Eigen::VectorXd *pos = nullptr, Eigen::VectorXd *vel = nullptr, Eigen::VectorXd *acc = nullptr);

    // 更新初始速度和目标位置
    void update(const Eigen::VectorXd &start_pos,
                const Eigen::VectorXd &start_vel,
                const std::vector<Eigen::VectorXd> &target_pos,
                double acceleration = 0.05);
    void update(const Eigen::VectorXd &start_pos,
                const Eigen::VectorXd &start_vel,
                const Eigen::VectorXd &target_pos,
                double acceleration = 0.05);
    void update(const Eigen::VectorXd &target_pos,
                double acceleration = 0.05);
    void update(const Eigen::VectorXd &start_pos,
                const Eigen::VectorXd &start_vel,
                const std::vector<double> &times,
                const std::vector<Eigen::VectorXd> &target_pos);

private:
    Eigen::VectorXd start_pos_;
    Eigen::VectorXd start_vel_;
    std::vector<Eigen::VectorXd> target_pos_;
    std::vector<double> times_;
    double dt_;
    double step_;
    double current_traj_time_;
    double acceleration_;
    drake::trajectories::PiecewisePolynomial<double> pos_traj_;
    drake::trajectories::PiecewisePolynomial<double> vel_traj_;
    drake::trajectories::PiecewisePolynomial<double> acc_traj_;

    std::string name_;

    void updateTrajectory(InterpolationType mode);
};

#endif // VELOCITY_SMOOTH_INTERPOLATOR_H
