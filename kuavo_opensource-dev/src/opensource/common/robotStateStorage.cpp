#include "robotStateStorage.h"
void ThreadSafeDataStorage::update(const uint64_t &index, RobotData &new_transfer_data)
{
    RobotData &data_slot = check_data(index);
    if (new_transfer_data.actuation_updated)
    {
        mutex_act.lock();
        data_slot.update_actuation(new_transfer_data.actuation);
        mutex_act.unlock();
    }
    if (new_transfer_data.state_des_updated)
    {
        mutex_des.lock();
        data_slot.update_state_des(new_transfer_data.state_des);
        mutex_des.unlock();
    }
    if (new_transfer_data.state_est_updated)
    {
        mutex_est.lock();
        data_slot.update_state_est(new_transfer_data.state_est);
        mutex_est.unlock();
    }
}
void ThreadSafeDataStorage::update_actuation(const uint64_t &index, Eigen::VectorXd &new_actuation)
{
    RobotData &data_slot = check_data(index);
    std::lock_guard<std::mutex> lock(mutex_act);
    data_slot.update_actuation(new_actuation);
}
void ThreadSafeDataStorage::update_state_des(const uint64_t &index, RobotState_t &new_state_des)
{

    RobotData &data_slot = check_data(index);
    std::lock_guard<std::mutex> lock(mutex_des);
    data_slot.update_state_des(new_state_des);
}
void ThreadSafeDataStorage::update_state_est(const uint64_t &index, RobotState_t &new_state_est)
{
    RobotData &data_slot = check_data(index);
    std::lock_guard<std::mutex> lock(mutex_est);
    data_slot.update_state_est(new_state_est);
}

RobotData &ThreadSafeDataStorage::check_data(const uint64_t &index)
{
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = data_.find(index);
    if (it == data_.end())
    {
        data_[index] = RobotData();
        max_index_++;
        prune_data();
    }
    cv_.notify_all();
    return data_[index];
}
void ThreadSafeDataStorage::prune_data()
{
    if (data_.size() > 1000)
    {
        size_t erased = data_.erase(prune_index_);
        prune_index_++;
    }
}
size_t ThreadSafeDataStorage::size()
{
    return max_index_;
}
bool ThreadSafeDataStorage::check_data(const uint64_t &index, RobotData &data_slot)
{
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = data_.find(index);
    if (it == data_.end())
    {
        return false;
    }
    data_slot = data_[index];
    return true;
}
RobotData ThreadSafeDataStorage::newestData()
{
    std::unique_lock<std::mutex> lock(mutex_);
    uint64_t newest_id = max_index_;
    cv_.wait(lock, [&]
             { auto it = data_.find(newest_id);
                    return it != data_.end() &&
                        it->second.state_des_updated &&
                        it->second.state_est_updated&&
                        it->second.actuation_updated; });
    return data_[newest_id];
}

RobotData &ThreadSafeDataStorage::get(const uint64_t &index, bool get_state_des, bool get_state_est, bool get_actuation)
{
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait(lock, [&]
             { auto it = data_.find(index);
                    return it != data_.end() &&
                        (it->second.state_des_updated || !get_state_des) &&
                        (it->second.state_est_updated || !get_state_est)&&
                        (it->second.actuation_updated || !get_actuation); });
    return data_[index];
}
