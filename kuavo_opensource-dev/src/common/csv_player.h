#ifndef CSV_CSV_PLAYER_H
#define CSV_CSV_PLAYER_H

#include "csv_reader.h"
#include "sensor_data.h"
#include "robot_state.h"

class CsvPlayer
{
public:
    CsvPlayer(const std::string &filename, bool issim = false, double dt = 0.001);
    void readSensor(uint32_t step, SensorData_t &sensor_data);
    void readStateEst(uint32_t step, RobotState_t &state_est);
    void readStateDes(uint32_t step, RobotState_t &state_des);
    void readTau(uint32_t step, Eigen::VectorXd &tau);
    uint64_t getMaxIndex();
    bool isSim;

private:
    CsvLogLoader log_loader;
    // RobotState_t state_est_, state_des_;
    // SensorData_t sensor_data_;
    uint64_t max_index;
    double dt_;
};

#endif
