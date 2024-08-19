#ifndef _sim_sensor_h_
#define _sim_sensor_h_

#include <map>
#include <string>
#include <vector>

#include "drake/systems/framework/diagram.h"
#include "drake/multibody/plant/multibody_plant.h"

#include "drake/systems/sensors/accelerometer.h"
#include "drake/systems/sensors/gyroscope.h"

#include "sensor_data.h"
#include "lcm_publish.h"
// #include "actuators_interface.h"
#include "config.h"
/// SimSensor is a block that gathers controller input,
/// gyroscope, accelerometer and plant state, and packs them into
namespace HighlyDynamic
{
  class SimSensor
  {
  public:
    SimSensor(drake::multibody::MultibodyPlant<double> *plant, double dt_);

    /// Add simulated gyroscope and accelerometer along with sensor aggregator,
    /// which creates and publishes a simulated lcmt_cassie_out LCM message.
    /// @param builder The diagram builder
    /// @param plant The Cassie plant
    /// @param sensor_aggregator The sensor aggregator
    void AddImu(
        drake::systems::DiagramBuilder<double> *builder,
        const drake::multibody::MultibodyPlant<double> &plant);
    void SetContext(drake::systems::Diagram<double> *diagram,
                    drake::systems::Context<double> *diagram_context);
    void UpdateImu();
    void readSensor(SensorData_t &sensorData, const drake::systems::Context<double> &g_plant_context);
    void printImuData();

    void SetIMUquat(Eigen::Vector4d quat);

    void GetSensorData_t(SensorData_t &sensorData);

    const drake::systems::Context<double> *accel_context;
    const drake::systems::Context<double> *free_accel_context;
    const drake::systems::Context<double> *gyro_context;
    const drake::systems::sensors::Gyroscope<double> *gyroscope;
    const drake::systems::sensors::Accelerometer<double> *accelerometer;
    const drake::systems::sensors::Accelerometer<double> *accelerometer_no_gravity;

  private:
    Eigen::Vector3d accel_step;
    Eigen::Vector3d free_acc_step;
    Eigen::Vector3d gyro_step;
    Eigen::Vector4d quat_step; // [w,x,y,z] @warning Eigen::Quaterniond is order [x,y,z,w]
    drake::multibody::MultibodyPlant<double> *plant_;
    int32_t na_;
    int32_t nq_;
    int32_t nv_;
    Eigen::VectorXd state;
    Eigen::VectorXd v_prev;
    Eigen::VectorXd state_prev;
    std::vector<JointParam_t> joint_data;
    std::vector<JointParam_t> joint_data_old;
    double dt;
    bool isParallelArm{false};
  };
}
#endif // _sim_sensor_h
