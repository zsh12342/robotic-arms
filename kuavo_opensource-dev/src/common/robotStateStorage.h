#include <mutex>
#include <unordered_map>
#include "utils.h"
#include "robot_state.h"
#include <unistd.h>

struct RobotData
{
    Eigen::VectorXd actuation;
    RobotState_t state_des;
    RobotState_t state_est;
    bool actuation_updated;
    bool state_des_updated;
    bool state_est_updated;
    RobotData(const Eigen::VectorXd &act, const RobotState_t &des, const RobotState_t &est)
        : actuation(act), state_des(des), state_est(est),
          actuation_updated(true), state_des_updated(true), state_est_updated(true) {}
    RobotData() : actuation_updated(false), state_des_updated(false), state_est_updated(false) {}
    void update_actuation(Eigen::VectorXd &new_actuation)
    {
        actuation = new_actuation;
        actuation_updated = true;
    }
    void update_state_des(RobotState_t &new_state_des)
    {
        state_des = new_state_des;
        state_des_updated = true;
    }
    void update_state_est(RobotState_t &new_state_est)
    {
        state_est = new_state_est;
        state_est_updated = true;
    }
};

class ThreadSafeDataStorage
{
public:
    void update(const uint64_t &index, RobotData &new_transfer_data);
    void update_actuation(const uint64_t &index, Eigen::VectorXd &new_actuation);
    void update_state_des(const uint64_t &index, RobotState_t &new_state_des);
    void update_state_est(const uint64_t &index, RobotState_t &new_state_est);

    RobotData &check_data(const uint64_t &index);
    bool check_data(const uint64_t &index, RobotData &data_slot);
    void wait_for_data(const uint64_t &index);
    RobotData newestData();

    RobotData &get(const uint64_t &index, bool get_state_des = true, bool get_state_est = true, bool get_actuation = true);
    size_t size();
    void prune_data();

private:
    std::mutex mutex_, mutex_est, mutex_des, mutex_act;
    std::condition_variable cv_;
    uint64_t max_index_{0}, prune_index_{0};
    std::unordered_map<uint64_t, RobotData> data_;
};
