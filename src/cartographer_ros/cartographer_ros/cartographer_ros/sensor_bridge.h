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

#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_SENSOR_BRIDGE_H
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_SENSOR_BRIDGE_H

#include <memory>

#include "cartographer/common/optional.h"
#include "cartographer/mapping/trajectory_builder_interface.h"
#include "cartographer/sensor/imu_data.h"
#include "cartographer/sensor/odometry_data.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
#include "cartographer_ros/tf_bridge.h"
#include "cartographer_ros_msgs/LandmarkList.h"
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/MultiEchoLaserScan.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/PointCloud2.h"

namespace cartographer_ros {

// Converts ROS messages into SensorData in tracking frame for the MapBuilder.
// 为MapBuilder 将ROS的订阅传感器topic的消息 转换到 tracking frame坐标系 (同时转到cartographer的消息类型)
class SensorBridge {
public:
    // 构造函数
    explicit SensorBridge(
            int num_subdivisions_per_laser_scan, const std::string& tracking_frame,
            double lookup_transform_timeout_sec, tf2_ros::Buffer* tf_buffer,
            ::cartographer::mapping::TrajectoryBuilderInterface* trajectory_builder);

    SensorBridge(const SensorBridge&) = delete;
    SensorBridge& operator=(const SensorBridge&) = delete;

    // 将ros的里程计msg转换为自定义的carto::sensor::OdometryData
    // 数据内容为: tracking坐标系原点在 里程计坐标系的坐标
    std::unique_ptr<::cartographer::sensor::OdometryData> ToOdometryData(
            const nav_msgs::Odometry::ConstPtr& msg);
    // 调用trajectory_builder_,在对应轨迹添加里程计数据
    void HandleOdometryMessage(const std::string& sensor_id,
                               const nav_msgs::Odometry::ConstPtr& msg);

    // 处理gps数据,并添加到轨迹
    // 得到gps接收器坐标系到local_frame坐标系的变换, 也就是gps接收器在local_frame坐标系的坐标
    void HandleNavSatFixMessage(const std::string& sensor_id,
                                const sensor_msgs::NavSatFix::ConstPtr& msg);

    // ToLandmarkData: 遍历landmark_list, 转换并返回LandmarkData结构的数据
    // 再调用trajectory_builder_->AddSensorData添加LandmarkData结构的数据
    void HandleLandmarkMessage(
            const std::string& sensor_id,
            const cartographer_ros_msgs::LandmarkList::ConstPtr& msg);

    // 读取数据, 并返回转换到'tracking_frame_'坐标系之后的加速度和角速度
    std::unique_ptr<::cartographer::sensor::ImuData> ToImuData(
            const sensor_msgs::Imu::ConstPtr& msg);
    // 回调:处理IMU数据
    // 向trajectory_builder_添加: 转换到'tracking_frame_'坐标系之后的加速度和角速度
    void HandleImuMessage(const std::string& sensor_id,
                          const sensor_msgs::Imu::ConstPtr& msg);

    // 回调: 处理激光扫描数据
    // 调用了HandleLaserScan
    void HandleLaserScanMessage(const std::string& sensor_id,
                                const sensor_msgs::LaserScan::ConstPtr& msg);
    // 同上
    void HandleMultiEchoLaserScanMessage(
            const std::string& sensor_id,
            const sensor_msgs::MultiEchoLaserScan::ConstPtr& msg);
    // 对于sensor_msgs::PointCloud2的数据
    // 直接调用pcl解析,然后将解析的点push进carto::sensor::TimedPointCloud类型
    // 然后坐标系转换, 添加传感器数据
    void HandlePointCloud2Message(const std::string& sensor_id,
                                  const sensor_msgs::PointCloud2::ConstPtr& msg);

    const TfBridge& tf_bridge() const;

private:
    // 对点云进行细分,然后调用HandleRangefinder
    // 对细分的点云进行坐标系变换 :将点云从激光雷达坐标系转换到'tracking_frame_'坐标系
    // 然后调用trajectory_builder_ 添加传感器数据
    void HandleLaserScan(
            const std::string& sensor_id, ::cartographer::common::Time start_time,
            const std::string& frame_id,
            const ::cartographer::sensor::PointCloudWithIntensities& points);
    // 对细分的点云进行坐标系变换 :将点云从激光雷达坐标系转换到'tracking_frame_'坐标系
    // 然后调用trajectory_builder_ 添加传感器数据
    void HandleRangefinder(const std::string& sensor_id,
                           ::cartographer::common::Time time,
                           const std::string& frame_id,
                           const ::cartographer::sensor::TimedPointCloud& ranges);

    // 点云细分数
    const int num_subdivisions_per_laser_scan_;
    // 上一份细分点云的时间
    std::map<std::string, cartographer::common::Time>
    sensor_to_previous_subdivision_time_;

    // tf变换接口
    const TfBridge tf_bridge_;

    // 实际上是子类<CollatedTrajectoryBuilder>
    ::cartographer::mapping::TrajectoryBuilderInterface* const
    trajectory_builder_;

    // 从ecef坐标系到local_frame的坐标系
    ::cartographer::common::optional<::cartographer::transform::Rigid3d>
    ecef_to_local_frame_;
};

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_SENSOR_BRIDGE_H
