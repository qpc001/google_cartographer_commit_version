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

#include "cartographer_ros/sensor_bridge.h"

#include "cartographer/common/make_unique.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/time_conversion.h"

namespace cartographer_ros {

namespace carto = ::cartographer;

using carto::transform::Rigid3d;

namespace {

// 字符串检查,检查第一个字符是否为'/'
const std::string& CheckNoLeadingSlash(const std::string& frame_id) {
    if (frame_id.size() > 0) {
        // frame_id的第1个字符不应该为'/'
        CHECK_NE(frame_id[0], '/') << "The frame_id " << frame_id
                                   << " should not start with a /. See 1.7 in "
                                      "http://wiki.ros.org/tf2/Migration.";
    }
    return frame_id;
}

}  // namespace

SensorBridge::SensorBridge(
        const int num_subdivisions_per_laser_scan,
        const std::string& tracking_frame,
        const double lookup_transform_timeout_sec, tf2_ros::Buffer* const tf_buffer,
        carto::mapping::TrajectoryBuilderInterface* const trajectory_builder)
    : num_subdivisions_per_laser_scan_(num_subdivisions_per_laser_scan),
      tf_bridge_(tracking_frame, lookup_transform_timeout_sec, tf_buffer),
      trajectory_builder_(trajectory_builder) {}

// 将ros的里程计msg转换为自定义的carto::sensor::OdometryData
// 数据内容为: tracking坐标系原点在 里程计坐标系的坐标
std::unique_ptr<carto::sensor::OdometryData> SensorBridge::ToOdometryData(
        const nav_msgs::Odometry::ConstPtr& msg) {
    // ros msg的时间类型转换为自定义的时间类型
    const carto::common::Time time = FromRos(msg->header.stamp);
    // 查找tf变换: 从数据msg所在的坐标系转换到 tracking坐标系的变换
    const auto sensor_to_tracking = tf_bridge_.LookupToTracking(
                time, CheckNoLeadingSlash(msg->child_frame_id));
    // 找不到,则直接返回空
    if (sensor_to_tracking == nullptr) {
        return nullptr;
    }
    //
    return carto::common::make_unique<carto::sensor::OdometryData>(
                // msg->pose.pose: 当前机器人坐标系在里程计起点的坐标,即 传感器坐标系到里程计坐标系的变换
                // sensor_to_tracking->inverse(): tracking坐标系到 传感器坐标系的变换
                // 构造: tracking坐标系 到 里程计坐标系的变换 , 也就是tracking坐标系原点在 里程计坐标系的坐标
                carto::sensor::OdometryData{
                    time, ToRigid3d(msg->pose.pose) * sensor_to_tracking->inverse()});
}

// 调用trajectory_builder_,在对应轨迹添加里程计数据
void SensorBridge::HandleOdometryMessage(
        const std::string& sensor_id, const nav_msgs::Odometry::ConstPtr& msg) {
    std::unique_ptr<carto::sensor::OdometryData> odometry_data =
            ToOdometryData(msg);
    // 调用trajectory_builder_,在对应轨迹添加里程计数据
    if (odometry_data != nullptr) {
        trajectory_builder_->AddSensorData(
                    sensor_id,
                    carto::sensor::OdometryData{odometry_data->time, odometry_data->pose});
    }
}

// 处理gps数据,并添加到轨迹
// 得到gps接收器坐标系到local_frame坐标系的变换, 也就是gps接收器在local_frame坐标系的坐标
void SensorBridge::HandleNavSatFixMessage(
        const std::string& sensor_id, const sensor_msgs::NavSatFix::ConstPtr& msg) {
    const carto::common::Time time = FromRos(msg->header.stamp);
    // 检查gps,是否为fix状态
    if (msg->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX) {
        // 不是FIX状态 , 调用trajectory_builder_ 添加一个空的变换
        trajectory_builder_->AddSensorData(
                    sensor_id, carto::sensor::FixedFramePoseData{
                        time, carto::common::optional<Rigid3d>()});
        return;
    }

    // 检查: ecef坐标系到local_frame 的变换是否存在
    if (!ecef_to_local_frame_.has_value()) {
        // 如果不存在,则进行计算
        // 计算ECEF坐标系到 LocalFram的变换
        // LocalFram坐标系则 z轴的负方向指向地心, y轴正方向指向东边, x轴正方向指向南极(那一侧,并不是真的指向南极)
        ecef_to_local_frame_ =
                ComputeLocalFrameFromLatLong(msg->latitude, msg->longitude);
        LOG(INFO) << "Using NavSatFix. Setting ecef_to_local_frame with lat = "
                  << msg->latitude << ", long = " << msg->longitude << ".";
    }

    // LatLongAltToEcef(msg->latitude, msg->longitude,msg->altitude): GPS接收器在ECEF下的坐标, GPS接收器坐标系到ECEF坐标系的变换
    // ecef_to_local_frame_.value() *LatLongAltToEcef(msg->latitude, msg->longitude,msg->altitude) :
    // 所以,得到了 gps接收器坐标系到local_frame坐标系的变换, 也就是gps接收器在local_frame坐标系的坐标
    // 调用trajectory_builder_来添加这个数据
    trajectory_builder_->AddSensorData(
                sensor_id,
                carto::sensor::FixedFramePoseData{
                    time, carto::common::optional<Rigid3d>(Rigid3d::Translation(
                    ecef_to_local_frame_.value() *
                    LatLongAltToEcef(msg->latitude, msg->longitude,
                    msg->altitude)))});
}

// 回调,处理路标消息
void SensorBridge::HandleLandmarkMessage(
        const std::string& sensor_id,
        const cartographer_ros_msgs::LandmarkList::ConstPtr& msg) {

    // ToLandmarkData: 遍历landmark_list, 转换并返回LandmarkData结构的数据
    // 再调用trajectory_builder_->AddSensorData添加LandmarkData结构的数据
    trajectory_builder_->AddSensorData(sensor_id, ToLandmarkData(*msg));
}
// 读取数据, 并返回转换到'tracking_frame_'坐标系之后的加速度和角速度
std::unique_ptr<carto::sensor::ImuData> SensorBridge::ToImuData(
        const sensor_msgs::Imu::ConstPtr& msg) {
    // 检查协方差, 如果协方差不等于-1, 则继续往下, 否则,抛出异常
    CHECK_NE(msg->linear_acceleration_covariance[0], -1)
            << "Your IMU data claims to not contain linear acceleration measurements "
               "by setting linear_acceleration_covariance[0] to -1. Cartographer "
               "requires this data to work. See "
               "http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html.";
    CHECK_NE(msg->angular_velocity_covariance[0], -1)
            << "Your IMU data claims to not contain angular velocity measurements "
               "by setting angular_velocity_covariance[0] to -1. Cartographer "
               "requires this data to work. See "
               "http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html.";

    // 时间转换
    const carto::common::Time time = FromRos(msg->header.stamp);
    // 调用tf_bridge_  查找 从给定"frame_id"坐标系到'tracking_frame_'坐标系的变换, 并返回Rigid3d类型的变换
    const auto sensor_to_tracking = tf_bridge_.LookupToTracking(
                time, CheckNoLeadingSlash(msg->header.frame_id));
    if (sensor_to_tracking == nullptr) {
        return nullptr;
    }
    // 检查平移量
    CHECK(sensor_to_tracking->translation().norm() < 1e-5)
            << "The IMU frame must be colocated with the tracking frame. "
               "Transforming linear acceleration into the tracking frame will "
               "otherwise be imprecise.";
    // 返回carto::sensor::ImuData 类型的imu数据
    // 数据内容是: 将加速度,角速度 从传感器坐标系转换到 'tracking_frame_'坐标系之后的加速度和角速度
    return carto::common::make_unique<carto::sensor::ImuData>(
                carto::sensor::ImuData{
                    time,
                    sensor_to_tracking->rotation() * ToEigen(msg->linear_acceleration),
                    sensor_to_tracking->rotation() * ToEigen(msg->angular_velocity)});
}
// 回调:处理IMU数据
// 向trajectory_builder_添加: 转换到'tracking_frame_'坐标系之后的加速度和角速度
void SensorBridge::HandleImuMessage(const std::string& sensor_id,
                                    const sensor_msgs::Imu::ConstPtr& msg) {
    // 读取数据, 并返回转换到'tracking_frame_'坐标系之后的加速度和角速度
    std::unique_ptr<carto::sensor::ImuData> imu_data = ToImuData(msg);
    if (imu_data != nullptr) {
        // 向轨迹添加IMU数据
        trajectory_builder_->AddSensorData(
                    sensor_id,
                    carto::sensor::ImuData{imu_data->time, imu_data->linear_acceleration,
                                           imu_data->angular_velocity});
    }
}
// 回调: 处理激光扫描数据
void SensorBridge::HandleLaserScanMessage(
        const std::string& sensor_id, const sensor_msgs::LaserScan::ConstPtr& msg) {
    carto::sensor::PointCloudWithIntensities point_cloud;
    carto::common::Time time;
    // 调用pcl解析ros的点云消息 , 然后返回元组类型数据<带强度的点云,时间戳>
    std::tie(point_cloud, time) = ToPointCloudWithIntensities(*msg);
    // HandleLaserScan:
    // 1.对点云进行细分
    // 2.对细分的点云进行坐标系变换 :将点云从激光雷达坐标系转换到'tracking_frame_'坐标系
    // 3.然后调用trajectory_builder_ 添加传感器数据
    HandleLaserScan(sensor_id, time, msg->header.frame_id, point_cloud);
}

// 同上
void SensorBridge::HandleMultiEchoLaserScanMessage(
        const std::string& sensor_id,
        const sensor_msgs::MultiEchoLaserScan::ConstPtr& msg) {
    carto::sensor::PointCloudWithIntensities point_cloud;
    carto::common::Time time;
    std::tie(point_cloud, time) = ToPointCloudWithIntensities(*msg);
    HandleLaserScan(sensor_id, time, msg->header.frame_id, point_cloud);
}

// 对于sensor_msgs::PointCloud2的数据
// 直接调用pcl解析,然后将解析的点push进carto::sensor::TimedPointCloud类型
// 然后坐标系转换, 添加传感器数据
void SensorBridge::HandlePointCloud2Message(
        const std::string& sensor_id,
        const sensor_msgs::PointCloud2::ConstPtr& msg) {
    pcl::PointCloud<pcl::PointXYZ> pcl_point_cloud;
    pcl::fromROSMsg(*msg, pcl_point_cloud);
    carto::sensor::TimedPointCloud point_cloud;
    for (const auto& point : pcl_point_cloud) {
        point_cloud.emplace_back(point.x, point.y, point.z, 0.f);
    }
    //将点云从激光雷达坐标系转换到'tracking_frame_'坐标系
    // 然后调用trajectory_builder_ 添加传感器数据
    HandleRangefinder(sensor_id, FromRos(msg->header.stamp), msg->header.frame_id,
                      point_cloud);
}

// 返回成员变量 tf_bridge_
const TfBridge& SensorBridge::tf_bridge() const { return tf_bridge_; }

// 对点云进行细分
// 对细分的点云进行坐标系变换 :将点云从激光雷达坐标系转换到'tracking_frame_'坐标系
// 然后调用trajectory_builder_ 添加传感器数据
void SensorBridge::HandleLaserScan(
        const std::string& sensor_id, const carto::common::Time time,
        const std::string& frame_id,
        const carto::sensor::PointCloudWithIntensities& points) {   //带强度数据的点云
    if (points.points.empty()) {
        return;
    }
    // 检查点云最后一个点的第4维: 相对测量时间 , 小于0,则抛出异常, 应该要等于0才正常
    CHECK_LE(points.points.back()[3], 0);
    // TODO(gaschler): Use per-point time instead of subdivisions.
    // 遍历点云,对点云进行细分,然后发送
    for (int i = 0; i != num_subdivisions_per_laser_scan_; ++i) {
        // 计算细分索引
        const size_t start_index =
                points.points.size() * i / num_subdivisions_per_laser_scan_;
        const size_t end_index =
                points.points.size() * (i + 1) / num_subdivisions_per_laser_scan_;
        // 根据细分的索引进行切割, 裁剪点云
        carto::sensor::TimedPointCloud subdivision(
                    points.points.begin() + start_index, points.points.begin() + end_index);
        if (start_index == end_index) {
            continue;
        }
        // 获取裁剪之后的点云最后一个点的相对测量时间
        const double time_to_subdivision_end = subdivision.back()[3];
        // `subdivision_time` is the end of the measurement so sensor::Collator will
        // send all other sensor data first.
        const carto::common::Time subdivision_time =
                time + carto::common::FromSeconds(time_to_subdivision_end); //这是最后发送的

        // 对std::map<std::string, cartographer::common::Time>进行查找
        // 也就是上一小片细分点云的传感器id以及时间
        auto it = sensor_to_previous_subdivision_time_.find(sensor_id);
        // 如果找到, 但是同时 对应的时间比subdivision_time 大
        if (it != sensor_to_previous_subdivision_time_.end() &&
                it->second >= subdivision_time) {
            // 则跳过这小片细分的点云
            LOG(WARNING) << "Ignored subdivision of a LaserScan message from sensor "
                         << sensor_id << " because previous subdivision time "
                         << it->second << " is not before current subdivision time "
                         << subdivision_time;
            continue;
        }
        // 设置sensor_to_previous_subdivision_time_
        sensor_to_previous_subdivision_time_[sensor_id] = subdivision_time;
        // 遍历细分点云
        for (Eigen::Vector4f& point : subdivision) {
            // 每个点的时间都减去这片细分点云最后一个点的时间
            point[3] -= time_to_subdivision_end;
        }
        CHECK_EQ(subdivision.back()[3], 0);
        // 对细分的点云进行坐标系变换 :将点云从激光雷达坐标系转换到'tracking_frame_'坐标系
        // 然后调用trajectory_builder_ 添加传感器数据
        HandleRangefinder(sensor_id, subdivision_time, frame_id, subdivision);
    }
}

// 对细分的点云进行坐标系变换 :将点云从激光雷达坐标系转换到'tracking_frame_'坐标系
// 然后调用trajectory_builder_ 添加传感器数据
void SensorBridge::HandleRangefinder(
        const std::string& sensor_id, const carto::common::Time time,
        const std::string& frame_id, const carto::sensor::TimedPointCloud& ranges) {
    // 查找 从给定"frame_id"坐标系到'tracking_frame_'坐标系的变换, 并返回Rigid3d类型的变换
    // 也就是获取激光雷达坐标系到'tracking_frame_'坐标系的变换
    const auto sensor_to_tracking =
            tf_bridge_.LookupToTracking(time, CheckNoLeadingSlash(frame_id));
    if (sensor_to_tracking != nullptr) {
        // 调用trajectory_builder_,添加carto::sensor::TimedPointCloudData类型数据
        // 其中, 数据内容为<传感器id, <时间戳,激光雷达坐标系在'tracking_frame_'坐标系的坐标, 'tracking_frame_'坐标系的点云>>
        trajectory_builder_->AddSensorData(
                    sensor_id, carto::sensor::TimedPointCloudData{
                        time,
                        sensor_to_tracking->translation().cast<float>(),  //激光雷达坐标系在'tracking_frame_'坐标系的坐标
                        carto::sensor::TransformTimedPointCloud(    //将点云从激光雷达坐标系转换到'tracking_frame_'坐标系
                        ranges, sensor_to_tracking->cast<float>())});
    }
}

}  // namespace cartographer_ros
