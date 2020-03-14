/*
 * Copyright 2017 The Cartographer Authors
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

#include "cartographer/mapping/pose_extrapolator.h"

#include <algorithm>

#include "cartographer/common/make_unique.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

//////////////////////////姿态外推器还没有认真看/////////////////////////////////////////

PoseExtrapolator::PoseExtrapolator(const common::Duration pose_queue_duration,
                                   double imu_gravity_time_constant)
    : pose_queue_duration_(pose_queue_duration),                // 推算姿态间隔
      gravity_time_constant_(imu_gravity_time_constant),        // 重力常数
      cached_extrapolated_pose_{common::Time::min(),            // 推算姿态缓存
                                transform::Rigid3d::Identity()} {}

//
std::unique_ptr<PoseExtrapolator> PoseExtrapolator::InitializeWithImu(
        const common::Duration pose_queue_duration,     // 推算姿态间隔
        const double imu_gravity_time_constant,         // 重力常数
        const sensor::ImuData& imu_data) {              // imu数据
    // 先初始化一个姿态外推器PoseExtrapolator
    auto extrapolator = common::make_unique<PoseExtrapolator>(
                pose_queue_duration, imu_gravity_time_constant);
    // 添加imu数据到imu_data_ ,然后根据时间戳等条件剔除无效imu数据
    extrapolator->AddImuData(imu_data);
    // 构造一个IMU跟踪器ImuTracker, 然后保存到上面new出来的姿态外推器
    extrapolator->imu_tracker_ =
            common::make_unique<ImuTracker>(imu_gravity_time_constant, imu_data.time);
    // 向ImuTracker更新线加速度, 同时利用重力矢量修正旋转量
    extrapolator->imu_tracker_->AddImuLinearAccelerationObservation(
                imu_data.linear_acceleration);
    // 更新角速度
    extrapolator->imu_tracker_->AddImuAngularVelocityObservation(
                imu_data.angular_velocity);
    // 推算一次 旋转量, 只推算旋转,不推算位置
    extrapolator->imu_tracker_->Advance(imu_data.time);
    //
    extrapolator->AddPose(
                imu_data.time,
                transform::Rigid3d::Rotation(extrapolator->imu_tracker_->orientation()));
    return extrapolator;
}

common::Time PoseExtrapolator::GetLastPoseTime() const {
    if (timed_pose_queue_.empty()) {
        return common::Time::min();
    }
    return timed_pose_queue_.back().time;
}

common::Time PoseExtrapolator::GetLastExtrapolatedTime() const {
    if (!extrapolation_imu_tracker_) {
        return common::Time::min();
    }
    return extrapolation_imu_tracker_->time();
}

// 添加一次位姿观测 (由scanMatching获得)
void PoseExtrapolator::AddPose(const common::Time time,
                               const transform::Rigid3d& pose) {
    // 检查ImuTracker
    if (imu_tracker_ == nullptr) {
        // 如果ImuTracker为空
        common::Time tracker_start = time;
        // 如果有imu数据
        if (!imu_data_.empty()) {
            // 取imu数据第一个元素的时间戳和当前时间戳 , 哪个小取哪个
            tracker_start = std::min(tracker_start, imu_data_.front().time);
        }
        // 创建一个新的ImuTracker
        imu_tracker_ =
                common::make_unique<ImuTracker>(gravity_time_constant_, tracker_start);
    }
    // 向位姿队列添加传进来的参数 pose
    timed_pose_queue_.push_back(TimedPose{time, pose});
    // 弹出位姿队列一些早期的pose
    while (timed_pose_queue_.size() > 2 &&
           timed_pose_queue_[1].time <= time - pose_queue_duration_) {
        timed_pose_queue_.pop_front();
    }
    // 从pose队列里面取首尾两个pose,计算角速度和线速度
    UpdateVelocitiesFromPoses();
    // 遍历imu数据队列, 使用imu_tracker进行更新,直到赶上当前的时间或者遍历完imu数据队列
    // 赶上当前时间之后, 在推算一次,作为估计
    AdvanceImuTracker(time, imu_tracker_.get());
    // 裁剪IMU数据(剔除一些不合格的数据)
    TrimImuData();
    // 裁剪里程计数据(剔除一些不合格的数据)
    TrimOdometryData();
    // 如果有新创建的imu_tracker,则保存
    odometry_imu_tracker_ = common::make_unique<ImuTracker>(*imu_tracker_);
    extrapolation_imu_tracker_ = common::make_unique<ImuTracker>(*imu_tracker_);
}

// 添加imu数据到imu_data_ ,然后根据时间戳等条件剔除无效imu数据
void PoseExtrapolator::AddImuData(const sensor::ImuData& imu_data) {
    CHECK(timed_pose_queue_.empty() ||
          imu_data.time >= timed_pose_queue_.back().time);
    imu_data_.push_back(imu_data);
    // 裁剪IMU数据(剔除一些不合格的数据)
    TrimImuData();
}

void PoseExtrapolator::AddOdometryData(
        const sensor::OdometryData& odometry_data) {
    CHECK(timed_pose_queue_.empty() ||
          odometry_data.time >= timed_pose_queue_.back().time);
    // push
    odometry_data_.push_back(odometry_data);
    // 裁剪里程计数据
    TrimOdometryData();
    if (odometry_data_.size() < 2) {
        return;
    }
    // TODO(whess): Improve by using more than just the last two odometry poses.
    // Compute extrapolation in the tracking frame.
    // 取里程计数据首尾两个数据
    const sensor::OdometryData& odometry_data_oldest = odometry_data_.front();
    const sensor::OdometryData& odometry_data_newest = odometry_data_.back();
    // 时间差
    const double odometry_time_delta =
            common::ToSeconds(odometry_data_oldest.time - odometry_data_newest.time);
    // odometry_data_oldest.pose: (最旧的) 里程计坐标系到载体坐标系的变换, 即载体在里程计坐标系的坐标
    // odometry_data_newest.pose: (最新的) 里程计坐标系到载体坐标系的变换, 即载体在里程计坐标系的坐标
    // odometry_pose_delta: (最旧的) 里程计坐标系 到 (最新的)载体坐标系的变换 , 即 (最新的)载体坐标系在里程计坐标系的坐标
    const transform::Rigid3d odometry_pose_delta =
            odometry_data_newest.pose.inverse() * odometry_data_oldest.pose;
    // 计算角速度
    angular_velocity_from_odometry_ =
            transform::RotationQuaternionToAngleAxisVector(
                odometry_pose_delta.rotation()) /
            odometry_time_delta;
    if (timed_pose_queue_.empty()) {
        return;
    }
    // 最新里程计时间戳下的 在tracking_frame的线速度
    const Eigen::Vector3d
            linear_velocity_in_tracking_frame_at_newest_odometry_time =
            odometry_pose_delta.translation() / odometry_time_delta;
    // timed_pose_queue_.back().pose.rotation(): 姿态队列最后一个元素的旋转量: 从参考坐标系到载体坐标系的旋转变换
    const Eigen::Quaterniond orientation_at_newest_odometry_time =
            timed_pose_queue_.back().pose.rotation() *          // timed_pose_queue_.back().pose.rotation():
            ExtrapolateRotation(odometry_data_newest.time,      // 这个函数返回: 旧的载体坐标系到新的载体坐标系的变换
                                odometry_imu_tracker_.get());
    linear_velocity_from_odometry_ =
            orientation_at_newest_odometry_time *
            linear_velocity_in_tracking_frame_at_newest_odometry_time;
}
// 计算没有重力对齐的位姿预测
transform::Rigid3d PoseExtrapolator::ExtrapolatePose(const common::Time time) {
    const TimedPose& newest_timed_pose = timed_pose_queue_.back();
    CHECK_GE(time, newest_timed_pose.time);
    if (cached_extrapolated_pose_.time != time) {
        const Eigen::Vector3d translation =
                ExtrapolateTranslation(time) + newest_timed_pose.pose.translation();
        const Eigen::Quaterniond rotation =
                newest_timed_pose.pose.rotation() *
                ExtrapolateRotation(time, extrapolation_imu_tracker_.get());
        cached_extrapolated_pose_ =
                TimedPose{time, transform::Rigid3d{translation, rotation}};
    }
    return cached_extrapolated_pose_.pose;
}

Eigen::Quaterniond PoseExtrapolator::EstimateGravityOrientation(
        const common::Time time) {
    ImuTracker imu_tracker = *imu_tracker_;
    AdvanceImuTracker(time, &imu_tracker);
    return imu_tracker.orientation();
}

// 从pose队列里面取首尾两个pose,计算角速度和线速度
void PoseExtrapolator::UpdateVelocitiesFromPoses() {
    // 至少需要两个pose才能计算速度
    if (timed_pose_queue_.size() < 2) {
        // We need two poses to estimate velocities.
        return;
    }
    CHECK(!timed_pose_queue_.empty());
    // 取最新的pose的时间戳
    const TimedPose& newest_timed_pose = timed_pose_queue_.back();
    const auto newest_time = newest_timed_pose.time;
    // 取队列最第一个pose的时间戳
    const TimedPose& oldest_timed_pose = timed_pose_queue_.front();
    const auto oldest_time = oldest_timed_pose.time;
    // 计算时间间隔
    const double queue_delta = common::ToSeconds(newest_time - oldest_time);
    // 如果时间间隔<1毫秒
    if (queue_delta < 0.001) {  // 1 ms
        // 表示pose队列太短了
        LOG(WARNING) << "Queue too short for velocity estimation. Queue duration: "
                     << queue_delta << " ms";
        return;
    }
    // 取两个pose
    const transform::Rigid3d& newest_pose = newest_timed_pose.pose;
    const transform::Rigid3d& oldest_pose = oldest_timed_pose.pose;
    // 线速度= 两个pose的平移量之差/时间
    linear_velocity_from_poses_ =
            (newest_pose.translation() - oldest_pose.translation()) / queue_delta;
    // 角速度 = 两个pose之间的旋转量/时间
    angular_velocity_from_poses_ =
            transform::RotationQuaternionToAngleAxisVector( //根据四元数返回绕轴-角旋转的向量
                oldest_pose.rotation().inverse() * newest_pose.rotation()) /
            queue_delta;
}

// 裁剪IMU数据(剔除一些不合格的数据)
void PoseExtrapolator::TrimImuData() {
    // imu数据长度>1
    // 姿态队列非空
    // imu数据时间戳比姿态队列最后一个元素的时间戳还早
    // 符合上面3个条件的imu数据都是无效数据
    while (imu_data_.size() > 1 && !timed_pose_queue_.empty() &&
           imu_data_[1].time <= timed_pose_queue_.back().time) {
        imu_data_.pop_front();
    }
}

// 裁剪里程计数据(剔除一些不合格的数据)
void PoseExtrapolator::TrimOdometryData() {
    // 里程计数据>2
    // pose队列非空
    // 里程计第一个数据比姿态队列最后一个元素的时间戳还早
    // 符合上面条件的里程计数据都是无效数据
    while (odometry_data_.size() > 2 && !timed_pose_queue_.empty() &&
           odometry_data_[1].time <= timed_pose_queue_.back().time) {
        odometry_data_.pop_front();
    }
}
// 遍历imu数据队列, 使用imu_tracker进行更新,直到赶上当前的时间或者遍历完imu数据队列
// 赶上当前时间之后, 在推算一次,作为估计
void PoseExtrapolator::AdvanceImuTracker(const common::Time time,
                                         ImuTracker* const imu_tracker) const {
    CHECK_GE(time, imu_tracker->time());
    // 如果imu数据为空,或者当前时间比imu数据时间早
    if (imu_data_.empty() || time < imu_data_.front().time) {
        // There is no IMU data until 'time', so we advance the ImuTracker and use
        // the angular velocities from poses and fake gravity to help 2D stability.
        // 根据当前时间进行一次推算
        imu_tracker->Advance(time);
        // 更新线加速度以及修正旋转
        imu_tracker->AddImuLinearAccelerationObservation(Eigen::Vector3d::UnitZ());
        // 如果里程计数据不足2个,则使用来自pose计算得到的角速度来更新
        // 如果里程计数据大于2个,则使用里程计数据来更新角速度
        imu_tracker->AddImuAngularVelocityObservation(
                    odometry_data_.size() < 2 ? angular_velocity_from_poses_
                                              : angular_velocity_from_odometry_);
        return;
    }
    // 如果imu_tracker的时间 比 imu数据时间早, 表示有新的imu数据来了
    if (imu_tracker->time() < imu_data_.front().time) {
        // Advance to the beginning of 'imu_data_'.
        // 从imu数据的最先的元素开始进行推算
        imu_tracker->Advance(imu_data_.front().time);
    }
    // imu数据迭代器 : 返回 比当前时间早的某个imu数据
    auto it = std::lower_bound(
                imu_data_.begin(), imu_data_.end(), imu_tracker->time(),
                [](const sensor::ImuData& imu_data, const common::Time& time) {
        return imu_data.time < time;
    });
    // 遍历imu数据队列, 使用imu_tracker进行更新,直到赶上当前的时间或者遍历完imu数据队列
    while (it != imu_data_.end() && it->time < time) {
        imu_tracker->Advance(it->time);
        imu_tracker->AddImuLinearAccelerationObservation(it->linear_acceleration);
        imu_tracker->AddImuAngularVelocityObservation(it->angular_velocity);
        ++it;
    }
    // 赶上当前时间之后, 在推算一次,作为估计
    imu_tracker->Advance(time);
}

Eigen::Quaterniond PoseExtrapolator::ExtrapolateRotation(
        const common::Time time, ImuTracker* const imu_tracker) const {
    CHECK_GE(time, imu_tracker->time());
    // 给定当前时间, 启动imu_tracker向前推算, 直到赶上时间time
    // 然后再向前推算一步
    AdvanceImuTracker(time, imu_tracker);

    const Eigen::Quaterniond last_orientation = imu_tracker_->orientation();
    // imu_tracker->orientation() : 载体坐标系到参考坐标系的旋转
    // last_orientation.inverse() : 参考坐标系到旧的载体坐标系的旋转
    // 返回: 新的载体坐标系到 旧的载体坐标系的旋转
    return last_orientation.inverse() * imu_tracker->orientation();
}

Eigen::Vector3d PoseExtrapolator::ExtrapolateTranslation(common::Time time) {
    const TimedPose& newest_timed_pose = timed_pose_queue_.back();
    const double extrapolation_delta =
            common::ToSeconds(time - newest_timed_pose.time);
    if (odometry_data_.size() < 2) {
        return extrapolation_delta * linear_velocity_from_poses_;
    }
    return extrapolation_delta * linear_velocity_from_odometry_;
}

}  // namespace mapping
}  // namespace cartographer
