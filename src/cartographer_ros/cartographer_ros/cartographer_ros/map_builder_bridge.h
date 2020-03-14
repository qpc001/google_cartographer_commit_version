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

#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_MAP_BUILDER_BRIDGE_H
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_MAP_BUILDER_BRIDGE_H

#include <memory>
#include <set>
#include <string>
#include <unordered_map>

#include "cartographer/common/mutex.h"
#include "cartographer/mapping/map_builder_interface.h"
#include "cartographer/mapping/proto/trajectory_builder_options.pb.h"
#include "cartographer/mapping/trajectory_builder_interface.h"
#include "cartographer_ros/node_options.h"
#include "cartographer_ros/sensor_bridge.h"
#include "cartographer_ros/tf_bridge.h"
#include "cartographer_ros/trajectory_options.h"
#include "cartographer_ros_msgs/SubmapEntry.h"
#include "cartographer_ros_msgs/SubmapList.h"
#include "cartographer_ros_msgs/SubmapQuery.h"
#include "nav_msgs/OccupancyGrid.h"
#include "visualization_msgs/MarkerArray.h"

namespace cartographer_ros {

// 中间件,基本由node类来调用
class MapBuilderBridge {
public:
    // 轨迹状态结构体
    struct TrajectoryState {
        // Contains the trajectory state data received from local SLAM, after
        // it had processed accumulated 'range_data_in_local' and estimated
        // current 'local_pose' at 'time'.
        // 包含了来自 local SLAM的轨迹状态，在local SLAM 在"time"时刻处理完range_data_in_local和当前的local_pose之后
        struct LocalSlamData {
            ::cartographer::common::Time time;
            ::cartographer::transform::Rigid3d local_pose;
            ::cartographer::sensor::RangeData range_data_in_local;
        };
        std::shared_ptr<const LocalSlamData> local_slam_data;   //上面定义的结构体
        cartographer::transform::Rigid3d local_to_map;          // local 到 map的3D变换
        std::unique_ptr<cartographer::transform::Rigid3d> published_to_tracking;    //published到tracking坐标系的变换
        TrajectoryOptions trajectory_options;
    };

    // MapBuilderBridge构造函数
    MapBuilderBridge(
            const NodeOptions& node_options,
            std::unique_ptr<cartographer::mapping::MapBuilderInterface> map_builder,
            tf2_ros::Buffer* tf_buffer);

    MapBuilderBridge(const MapBuilderBridge&) = delete;
    MapBuilderBridge& operator=(const MapBuilderBridge&) = delete;

    // 从文件读取状态
    void LoadState(const std::string& state_filename, bool load_frozen_state);
    // 添加轨迹,(SensorId结构体集合，轨迹选项)
    int AddTrajectory(
            const std::set<
            ::cartographer::mapping::TrajectoryBuilderInterface::SensorId>&
            expected_sensor_ids,
            const TrajectoryOptions& trajectory_options);
    // 完成轨迹(轨迹id)
    void FinishTrajectory(int trajectory_id);
    // 调用最后的全局BA
    void RunFinalOptimization();
    // 序列化
    bool SerializeState(const std::string& filename);
    // 回调： 处理子图查询
    void HandleSubmapQuery(
            cartographer_ros_msgs::SubmapQuery::Request& request,
            cartographer_ros_msgs::SubmapQuery::Response& response);

    // 获取冻结的轨迹id集合
    std::set<int> GetFrozenTrajectoryIds();
    // 获取子图列表
    cartographer_ros_msgs::SubmapList GetSubmapList();
    // 获取轨迹状态
    std::unordered_map<int, TrajectoryState> GetTrajectoryStates()
    EXCLUDES(mutex_);
    // 可视化msg
    visualization_msgs::MarkerArray GetTrajectoryNodeList();
    visualization_msgs::MarkerArray GetLandmarkPosesList();
    visualization_msgs::MarkerArray GetConstraintList();
    // 当Node节点订阅传感器并执行相应回调函数时,通过这个函数获取给定轨迹ID的sensor_bridge对象
    SensorBridge* sensor_bridge(int trajectory_id);

private:
    // 传给Cartographer C++的map_builder对象的回调
    void OnLocalSlamResult(
            const int trajectory_id, const ::cartographer::common::Time time,
            const ::cartographer::transform::Rigid3d local_pose,
            ::cartographer::sensor::RangeData range_data_in_local,
            const std::unique_ptr<const ::cartographer::mapping::
            TrajectoryBuilderInterface::InsertionResult>
            insertion_result) EXCLUDES(mutex_);

    cartographer::common::Mutex mutex_;
    const NodeOptions node_options_;

    //轨迹状态数据，由线程锁守护
    std::unordered_map<int, std::shared_ptr<const TrajectoryState::LocalSlamData>>
    trajectory_state_data_ GUARDED_BY(mutex_);

    // node_main.cc 传进来的map_builder_
    std::unique_ptr<cartographer::mapping::MapBuilderInterface> map_builder_;

    // node_main.cc 传进来的tf变换缓冲区
    tf2_ros::Buffer* const tf_buffer_;

    std::unordered_map<std::string /* landmark ID */, int> landmark_to_index_;

    // These are keyed with 'trajectory_id'.
    // <轨迹id , TrajectoryOptions>
    std::unordered_map<int, TrajectoryOptions> trajectory_options_;
    // <轨迹id, SensorBridge>
    std::unordered_map<int, std::unique_ptr<SensorBridge>> sensor_bridges_;
    // <轨迹id, 轨迹中编号最高的标记的id>
    std::unordered_map<int, size_t> trajectory_to_highest_marker_id_;
};

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_MAP_BUILDER_BRIDGE_H
