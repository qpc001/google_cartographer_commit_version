/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CARTOGRAPHER_MAPPING_IMU_TRACKER_H_
#define CARTOGRAPHER_MAPPING_IMU_TRACKER_H_

#include "Eigen/Geometry"
#include "cartographer/common/time.h"

namespace cartographer {
namespace mapping {

// Keeps track of the orientation using angular velocities and linear
// accelerations from an IMU. Because averaged linear acceleration (assuming
// slow movement) is a direct measurement of gravity, roll/pitch does not drift,
// though yaw does.
////////////////////////////////////////////////////////////////////////////////
/// \brief IMU角度跟踪器
/// \brief 使用IMU的线加速度和角速度来跟踪旋转,
///         因为可以使用重力分量来求出 roll和pitch两个不会漂移的维度, 但是YAW会漂移
////////////////////////////////////////////////////////////////////////////////
class ImuTracker {
public:
    // 初始化一个IMU跟踪器
    ImuTracker(double imu_gravity_time_constant, common::Time time);

    // Advances to the given 'time' and updates the orientation to reflect this.
    // 根据给定时间与上一次时间的时间差, 以及imu角速度数据, 更新旋转量和重力方向矢量
    void Advance(common::Time time);

    // Updates from an IMU reading (in the IMU frame).
    // 根据线加速度修正旋转量
    void AddImuLinearAccelerationObservation(
            const Eigen::Vector3d& imu_linear_acceleration);
    // 更新角速度
    void AddImuAngularVelocityObservation(
            const Eigen::Vector3d& imu_angular_velocity);

    // Query the current time.
    common::Time time() const { return time_; }

    // Query the current orientation estimate.
    // 从载体坐标系到参考坐标系的旋转
    Eigen::Quaterniond orientation() const { return orientation_; }

private:
    const double imu_gravity_time_constant_;
    common::Time time_;
    common::Time last_linear_acceleration_time_;
    Eigen::Quaterniond orientation_;
    Eigen::Vector3d gravity_vector_;
    Eigen::Vector3d imu_angular_velocity_;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_IMU_TRACKER_H_
