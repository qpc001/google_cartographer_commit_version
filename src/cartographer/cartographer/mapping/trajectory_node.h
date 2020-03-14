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

#ifndef CARTOGRAPHER_MAPPING_TRAJECTORY_NODE_H_
#define CARTOGRAPHER_MAPPING_TRAJECTORY_NODE_H_

#include <memory>
#include <vector>

#include "Eigen/Core"
#include "cartographer/common/optional.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/proto/trajectory_node_data.pb.h"
#include "cartographer/sensor/range_data.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace mapping {

// 轨迹节点位姿结构体
struct TrajectoryNodePose {
    // 常量位姿结构体
    struct ConstantPoseData {
        common::Time time;
        transform::Rigid3d local_pose;
    };
    // The node pose in the global SLAM frame.
    // 在全局SLAM坐标系中的 节点位姿
    transform::Rigid3d global_pose;

    // 常量local_pose
    common::optional<ConstantPoseData> constant_pose_data;
};

// 轨迹节点结构体(重力对齐四元数, 重力对齐并且滤波后的点云 , {} {} {} ,
//              局部SLAM的节点位姿(scanMatching位姿估计: 机器人在子图坐标系的坐标) )
struct TrajectoryNode {
    struct Data {
        common::Time time;

        // Transform to approximately gravity align the tracking frame as
        // determined by local SLAM.
        // 重力对齐
        Eigen::Quaterniond gravity_alignment;

        // Used for loop closure in 2D: voxel filtered returns in the
        // 'gravity_alignment' frame.
        // 将点云进行重力对齐 (在2D的闭环中使用)
        sensor::PointCloud filtered_gravity_aligned_point_cloud;

        // Used for loop closure in 3D.
        // 这是在3D使用的
        sensor::PointCloud high_resolution_point_cloud;
        sensor::PointCloud low_resolution_point_cloud;
        Eigen::VectorXf rotational_scan_matcher_histogram;

        // The node pose in the local SLAM frame.
        // 局部SLAM的节点位姿(节点相对于local map的位姿 ,即 节点坐标系到local map坐标系的变换)
        transform::Rigid3d local_pose;
    };

    // 常量时间
    common::Time time() const { return constant_data->time; }

    // This must be a shared_ptr. If the data is used for visualization while the
    // node is being trimmed, it must survive until all use finishes.
    // 这个常量数据必须是share的,需要维护到最后使用完成
    std::shared_ptr<const Data> constant_data;

    // The node pose in the global SLAM frame.
    // 节点的全局位姿
    transform::Rigid3d global_pose;
};

proto::TrajectoryNodeData ToProto(const TrajectoryNode::Data& constant_data);
TrajectoryNode::Data FromProto(const proto::TrajectoryNodeData& proto);

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_TRAJECTORY_NODE_H_
