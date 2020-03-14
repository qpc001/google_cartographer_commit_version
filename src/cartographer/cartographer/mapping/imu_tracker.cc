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

#include "cartographer/mapping/imu_tracker.h"

#include <cmath>
#include <limits>

#include "cartographer/common/math.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

ImuTracker::ImuTracker(const double imu_gravity_time_constant,
                       const common::Time time)
    : imu_gravity_time_constant_(imu_gravity_time_constant),    //设置重力常数
      time_(time),                                              //设置时间
      last_linear_acceleration_time_(common::Time::min()),      //设置上一次记录时间
      orientation_(Eigen::Quaterniond::Identity()),             //设置初始旋转量
      gravity_vector_(Eigen::Vector3d::UnitZ()),                //重力方向向量
      imu_angular_velocity_(Eigen::Vector3d::Zero())            //设置初始角速度
{}

// 根据给定时间与上一次时间的时间差, 以及imu角速度数据, 更新旋转量和重力方向矢量
void ImuTracker::Advance(const common::Time time) {
    CHECK_LE(time_, time);
    // 求给定时间与上一次计算时间的时间差delta t
    const double delta_t = common::ToSeconds(time - time_);
    // 给定绕angle-axis旋转，返回四元数
    const Eigen::Quaterniond rotation =
            transform::AngleAxisVectorToRotationQuaternion(
                Eigen::Vector3d(imu_angular_velocity_ * delta_t));  //轴-角旋转变化量= 角速度*delta t
    // 四元数更新
    orientation_ = (orientation_ * rotation).normalized();
    // 更新重力方向矢量
    gravity_vector_ = rotation.conjugate() * gravity_vector_; // 重力矢量变化方向刚好与载体坐标系变化方向相反,所以反向更新
    time_ = time;
}

// 根据线加速度修正旋转量
void ImuTracker::AddImuLinearAccelerationObservation(
        const Eigen::Vector3d& imu_linear_acceleration) {
    // Update the 'gravity_vector_' with an exponential moving average using the
    // 'imu_gravity_time_constant'.
    // 根据上一次更新线加速度的时间, 求delta t
    const double delta_t =
            last_linear_acceleration_time_ > common::Time::min()
            ? common::ToSeconds(time_ - last_linear_acceleration_time_)
            : std::numeric_limits<double>::infinity();
    last_linear_acceleration_time_ = time_;
    // 使用加权的方式修正重力方向矢量
    const double alpha = 1. - std::exp(-delta_t / imu_gravity_time_constant_);
    gravity_vector_ =
            (1. - alpha) * gravity_vector_ + alpha * imu_linear_acceleration;

    // Change the 'orientation_' so that it agrees with the current
    // 'gravity_vector_'.
    // 根据修正的重力方向矢量来更新旋转量
    // 1. 先根据旧的旋转量求出 从上面修正之后的(新重力方向矢量)到(旧的重力方向矢量)的旋转变化量
    const Eigen::Quaterniond rotation = Eigen::Quaterniond::FromTwoVectors(
                gravity_vector_, orientation_.conjugate() * Eigen::Vector3d::UnitZ());
    // 2. 利用旋转变化量来更新旋转量
    // orientation_: 从旧的重力矢量方向到载体的旋转?
    orientation_ = (orientation_ * rotation).normalized();
    CHECK_GT((orientation_ * gravity_vector_).z(), 0.);
    CHECK_GT((orientation_ * gravity_vector_).normalized().z(), 0.99);
}

// 更新角速度
void ImuTracker::AddImuAngularVelocityObservation(
        const Eigen::Vector3d& imu_angular_velocity) {
    imu_angular_velocity_ = imu_angular_velocity;
}

}  // namespace mapping
}  // namespace cartographer
