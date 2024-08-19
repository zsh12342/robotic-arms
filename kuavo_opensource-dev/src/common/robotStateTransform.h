#pragma once
#include "utils.h"
#include "robot_state.h"

class RobotStateTransform
{
private:
    Eigen::Matrix4d R_transformation; // transformation from body to world
    Eigen::Matrix4d R_transformation_inverse;
    Eigen::Matrix3d R_rotationMatrix;
    Eigen::Matrix3d R_rotationMatrix_inverse;

public:
    RobotStateTransform()
    {
        // Initialize with an identity matrix as the default transformation
        R_transformation = R_transformation_inverse = Eigen::Matrix4d::Identity();
        R_rotationMatrix = R_rotationMatrix_inverse = Eigen::Matrix3d::Identity();
        // update(Eigen::Vector3d(1, 1, 0), Eigen::Vector3d(0, 0, M_PI * 5 / 6));
        // tests();
    }
    void world2stance(const RobotState_t &state_W, RobotState_t &state_Stance)
    {
        state_Stance = state_W;
        state_Stance.q.segment(0, 4) = R_WB_quat(state_Stance.q.segment(0, 4));
        state_Stance.q.segment(4, 3) = R_WB_pos(state_Stance.q.segment(4, 3));
        state_Stance.v.segment(0, 3) = R_WB_vel(state_Stance.v.segment(0, 3));
        state_Stance.v.segment(3, 3) = R_WB_vel(state_Stance.v.segment(3, 3));
        state_Stance.vd.segment(0, 3) = R_WB_vel(state_Stance.vd.segment(0, 3));
        state_Stance.vd.segment(3, 3) = R_WB_vel(state_Stance.vd.segment(3, 3));
        state_Stance.r = R_WB_pos(state_Stance.r);
        state_Stance.rd = R_WB_vel(state_Stance.rd);
        state_Stance.rdd = R_WB_vel(state_Stance.rdd);
        state_Stance.r_est = R_WB_pos(state_Stance.r_est);
        state_Stance.rd_est = R_WB_vel(state_Stance.rd_est);

        state_Stance.torsoR = R_WB_euler(state_Stance.torsoR);
        state_Stance.torsoRd = R_WB_vel(state_Stance.torsoRd);
        state_Stance.torsoRdd = R_WB_vel(state_Stance.torsoRdd);

        state_Stance.lf.segment(0, 3) = R_WB_euler(state_Stance.lf.segment(0, 3));
        state_Stance.lf.segment(3, 3) = R_WB_pos(state_Stance.lf.segment(3, 3));
        state_Stance.lfv.segment(0, 3) = R_WB_vel(state_Stance.lfv.segment(0, 3));
        state_Stance.lfv.segment(3, 3) = R_WB_vel(state_Stance.lfv.segment(3, 3));
        state_Stance.lfvd.segment(0, 3) = R_WB_vel(state_Stance.lfvd.segment(0, 3));
        state_Stance.lfvd.segment(3, 3) = R_WB_vel(state_Stance.lfvd.segment(3, 3));
        state_Stance.rf.segment(0, 3) = R_WB_euler(state_Stance.rf.segment(0, 3));
        state_Stance.rf.segment(3, 3) = R_WB_pos(state_Stance.rf.segment(3, 3));
        state_Stance.rfv.segment(0, 3) = R_WB_vel(state_Stance.rfv.segment(0, 3));
        state_Stance.rfv.segment(3, 3) = R_WB_vel(state_Stance.rfv.segment(3, 3));
        state_Stance.rfvd.segment(0, 3) = R_WB_vel(state_Stance.rfvd.segment(0, 3));
        state_Stance.rfvd.segment(3, 3) = R_WB_vel(state_Stance.rfvd.segment(3, 3));

        state_Stance.cm.segment(0, 3) = R_WB_vel(state_Stance.cm.segment(0, 3));
        state_Stance.cm.segment(3, 3) = R_WB_vel(state_Stance.cm.segment(3, 3));
        state_Stance.lf_sm.segment(0, 3) = R_WB_vel(state_Stance.lf_sm.segment(0, 3));
        state_Stance.lf_sm.segment(3, 3) = R_WB_vel(state_Stance.lf_sm.segment(3, 3));
        state_Stance.rf_sm.segment(0, 3) = R_WB_vel(state_Stance.rf_sm.segment(0, 3));
        state_Stance.rf_sm.segment(3, 3) = R_WB_vel(state_Stance.rf_sm.segment(3, 3));
    }
    void stance2world(const RobotState_t &state_Stance, RobotState_t &state_W)
    {
        state_W = state_Stance;
        state_W.q.segment(0, 4) = R_BW_quat(state_W.q.segment(0, 4));
        state_W.q.segment(4, 3) = R_BW_pos(state_W.q.segment(4, 3));
        state_W.v.segment(0, 3) = R_BW_vel(state_W.v.segment(0, 3));
        state_W.v.segment(3, 3) = R_BW_vel(state_W.v.segment(3, 3));
        state_W.vd.segment(0, 3) = R_BW_vel(state_W.vd.segment(0, 3));
        state_W.vd.segment(3, 3) = R_BW_vel(state_W.vd.segment(3, 3));
        state_W.r = R_BW_pos(state_W.r);
        state_W.rd = R_BW_vel(state_W.rd);
        state_W.rdd = R_BW_vel(state_W.rdd);
        state_W.r_est = R_BW_pos(state_W.r_est);
        state_W.rd_est = R_BW_vel(state_W.rd_est);

        state_W.torsoR = R_BW_euler(state_W.torsoR);
        state_W.torsoRd = R_BW_vel(state_W.torsoRd);
        state_W.torsoRdd = R_BW_vel(state_W.torsoRdd);

        state_W.lf.segment(0, 3) = R_BW_euler(state_W.lf.segment(0, 3));
        state_W.lf.segment(3, 3) = R_BW_pos(state_W.lf.segment(3, 3));
        state_W.lfv.segment(0, 3) = R_BW_vel(state_W.lfv.segment(0, 3));
        state_W.lfv.segment(3, 3) = R_BW_vel(state_W.lfv.segment(3, 3));
        state_W.lfvd.segment(0, 3) = R_BW_vel(state_W.lfvd.segment(0, 3));
        state_W.lfvd.segment(3, 3) = R_BW_vel(state_W.lfvd.segment(3, 3));
        state_W.rf.segment(0, 3) = R_BW_euler(state_W.rf.segment(0, 3));
        state_W.rf.segment(3, 3) = R_BW_pos(state_W.rf.segment(3, 3));
        state_W.rfv.segment(0, 3) = R_BW_vel(state_W.rfv.segment(0, 3));
        state_W.rfv.segment(3, 3) = R_BW_vel(state_W.rfv.segment(3, 3));
        state_W.rfvd.segment(0, 3) = R_BW_vel(state_W.rfvd.segment(0, 3));
        state_W.rfvd.segment(3, 3) = R_BW_vel(state_W.rfvd.segment(3, 3));

        state_W.cm.segment(0, 3) = R_BW_vel(state_W.cm.segment(0, 3));
        state_W.cm.segment(3, 3) = R_BW_vel(state_W.cm.segment(3, 3));
        state_W.lf_sm.segment(0, 3) = R_BW_vel(state_W.lf_sm.segment(0, 3));
        state_W.lf_sm.segment(3, 3) = R_BW_vel(state_W.lf_sm.segment(3, 3));
        state_W.rf_sm.segment(0, 3) = R_BW_vel(state_W.rf_sm.segment(0, 3));
        state_W.rf_sm.segment(3, 3) = R_BW_vel(state_W.rf_sm.segment(3, 3));
    }
     void update(const RobotState_t &state_W_des, const RobotState_t &state_W_est)
    {
        Eigen::Vector3d stance_foot_pos, stance_foot_rot;
        
        if (state_W_est.walk_touch.walk_contact == R_Contact)
        {
            stance_foot_pos = state_W_est.rf.segment(3, 3); // 支撑脚位置
            stance_foot_rot = {0, 0, state_W_est.rf[2]};
        }
        else
        {
            stance_foot_pos = state_W_est.lf.segment(3, 3);
            stance_foot_rot = {0, 0, state_W_est.lf[2]};
        }
        this->update(stance_foot_pos, stance_foot_rot);
    }

    // 使用世界坐标系下局部坐标系的位置和旋转更新转换关系
    void update(const Eigen::Vector3d &position, const Eigen::Vector3d &rotation)
    {

        double yaw = rotation[2];
        R_rotationMatrix << cos(yaw), -sin(yaw), 0,
            sin(yaw), cos(yaw), 0,
            0, 0, 1.0;
        R_rotationMatrix_inverse = R_rotationMatrix.inverse();
        // Create a transformation matrix by combining rotation and translation for yaw and position only
        R_transformation.block<3, 3>(0, 0) = R_rotationMatrix;
        R_transformation.block<3, 1>(0, 3) = position;
        R_transformation_inverse = R_transformation.inverse();
    }

    inline Eigen::Vector3d R_BW_pos(const Eigen::Vector3d &position)
    {
        Eigen::Vector4d pose(position(0), position(1), position(2), 1.0);
        Eigen::Vector4d transformedPose = R_transformation * pose;
        return transformedPose.head<3>();
    }
    inline double R_BW_euler(const double &angle_yaw)
    {
        return R_BW_euler(Eigen::Vector3d(0, 0, angle_yaw))[2];
    }

    inline Eigen::Vector3d R_BW_euler(const Eigen::Vector3d &angle)
    {
        drake::math::RollPitchYaw<double> rpy(angle);
        auto quat = R_BW_quat(rpy.ToQuaternion());
        auto res = drake::math::RollPitchYawd(quat).vector();
        return res;
    }

    inline Eigen::Vector4d R_BW_quat(const Eigen::Vector4d &quaternionVector)
    {
        Eigen::Quaterniond quaternion(quaternionVector[0], quaternionVector[1], quaternionVector[2], quaternionVector[3]);
        quaternion = R_BW_quat(quaternion);
        return {quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z()};
    }

    inline Eigen::Quaterniond R_BW_quat(const Eigen::Quaterniond &quaternion)
    {
        Eigen::Quaterniond initialQuaternion(R_rotationMatrix);
        initialQuaternion *= quaternion;
        initialQuaternion.normalize();
        return initialQuaternion;
    }

    inline Eigen::Vector3d R_BW_vel(const Eigen::Vector3d &velocity)
    {
        return R_rotationMatrix * velocity;
    }

    inline Eigen::Vector3d R_WB_pos(const Eigen::Vector3d &position)
    {
        Eigen::Vector4d pose(position(0), position(1), position(2), 1.0);
        Eigen::Vector4d transformedPose = R_transformation_inverse * pose;
        return transformedPose.head<3>();
    }
    inline double R_WB_euler(const double &angle_yaw)
    {
        return R_WB_euler(Eigen::Vector3d(0, 0, angle_yaw))[2];
    }
    inline Eigen::Vector3d R_WB_euler(const Eigen::Vector3d &angle)
    {
        drake::math::RollPitchYaw<double> rpy(angle);
        auto quat = R_WB_quat(rpy.ToQuaternion());
        auto res = drake::math::RollPitchYawd(quat).vector();

        return res;
    }
    inline Eigen::Vector4d R_WB_quat(const Eigen::Vector4d &quaternionVector)
    {
        Eigen::Quaterniond quaternion(quaternionVector[0], quaternionVector[1], quaternionVector[2], quaternionVector[3]);
        quaternion = R_WB_quat(quaternion);
        return {quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z()};
    }
    // Apply a transformation to a given quaternion
    inline Eigen::Quaterniond R_WB_quat(const Eigen::Quaterniond &quaternion)
    {
        Eigen::Quaterniond initialQuaternion(R_rotationMatrix_inverse);
        initialQuaternion *= quaternion;
        initialQuaternion.normalize();
        return initialQuaternion;
    }

    // Apply a transformation to a given linear velocity
    inline Eigen::Vector3d R_WB_vel(const Eigen::Vector3d &velocity)
    {
        return R_rotationMatrix_inverse * velocity;
    }
};
