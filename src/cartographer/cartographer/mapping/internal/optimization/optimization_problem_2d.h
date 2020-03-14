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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_OPTIMIZATION_OPTIMIZATION_PROBLEM_2D_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_OPTIMIZATION_OPTIMIZATION_PROBLEM_2D_H_

#include <array>
#include <deque>
#include <map>
#include <set>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/common/port.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/id.h"
#include "cartographer/mapping/internal/optimization/optimization_problem_interface.h"
#include "cartographer/mapping/pose_graph_interface.h"
#include "cartographer/mapping/proto/pose_graph/optimization_problem_options.pb.h"
#include "cartographer/sensor/imu_data.h"
#include "cartographer/sensor/map_by_time.h"
#include "cartographer/sensor/odometry_data.h"
#include "cartographer/transform/timestamped_transform.h"

namespace cartographer {
namespace mapping {
namespace optimization {

// (时间戳, 局部位姿, 全局位姿, 重力方向)
struct NodeSpec2D {
    common::Time time;                      //时间戳
    transform::Rigid2d local_pose_2d;       //局部位姿 (节点在local map的坐标)
    transform::Rigid2d global_pose_2d;      //全局位姿 (节点在全局坐标系的坐标)
    Eigen::Quaterniond gravity_alignment;   //重力方向
};

// 记录子图的全局位姿
struct SubmapSpec2D {
    transform::Rigid2d global_pose;
};

class OptimizationProblem2D
        : public OptimizationProblemInterface<NodeSpec2D, SubmapSpec2D,
        transform::Rigid2d> {
public:
    // 2D优化问题构造函数: 使用配置选项构造
    explicit OptimizationProblem2D(
            const optimization::proto::OptimizationProblemOptions& options);
    ~OptimizationProblem2D();

    OptimizationProblem2D(const OptimizationProblem2D&) = delete;
    OptimizationProblem2D& operator=(const OptimizationProblem2D&) = delete;

    // 将imu数据添加到imu_data_ 成员变量 (按时间戳映射的imu数据容器)
    void AddImuData(int trajectory_id, const sensor::ImuData& imu_data) override;

    // 将里程计数据添加到odometry_data_ , (按时间戳映射的里程计数据容器)
    void AddOdometryData(int trajectory_id,
                         const sensor::OdometryData& odometry_data) override;

    // 将节点数据(时间戳, 局部位姿, 全局位姿, 重力方向) [追加]到node_data_
    // node_data_ (按NodeId{轨迹id , 节点索引}映射==>节点数据的容器)
    void AddTrajectoryNode(int trajectory_id,
                           const NodeSpec2D& node_data) override;
    // 将节点数据(时间戳, 局部位姿, 全局位姿, 重力方向) [插入]到node_data_
    // node_data_ (按NodeId{轨迹id , 节点索引}映射==>节点数据的容器)
    void InsertTrajectoryNode(const NodeId& node_id,
                              const NodeSpec2D& node_data) override;

    // 移除`node_id`节点及其数据
    void TrimTrajectoryNode(const NodeId& node_id) override;

    void AddSubmap(int trajectory_id,
                   const transform::Rigid2d& global_submap_pose) override;
    void InsertSubmap(const SubmapId& submap_id,
                      const transform::Rigid2d& global_submap_pose) override;
    void TrimSubmap(const SubmapId& submap_id) override;
    void SetMaxNumIterations(int32 max_num_iterations) override;

    void Solve(
            const std::vector<Constraint>& constraints,
            const std::set<int>& frozen_trajectories,
            const std::map<std::string, LandmarkNode>& landmark_nodes) override;

    const MapById<NodeId, NodeSpec2D>& node_data() const override {
        return node_data_;
    }
    const MapById<SubmapId, SubmapSpec2D>& submap_data() const override {
        return submap_data_;
    }
    const std::map<std::string, transform::Rigid3d>& landmark_data()
    const override {
        return landmark_data_;
    }
    const sensor::MapByTime<sensor::ImuData>& imu_data() const override {
        return imu_data_;
    }
    const sensor::MapByTime<sensor::OdometryData>& odometry_data()
    const override {
        return odometry_data_;
    }

private:
    std::unique_ptr<transform::Rigid3d> InterpolateOdometry(
            int trajectory_id, common::Time time) const;
    // Computes the relative pose between two nodes based on odometry data.
    std::unique_ptr<transform::Rigid3d> CalculateOdometryBetweenNodes(
            int trajectory_id, const NodeSpec2D& first_node_data,
            const NodeSpec2D& second_node_data) const;

    // 配置选项
    optimization::proto::OptimizationProblemOptions options_;
    /// MapById 相当于std::map的封装
    // 按NodeId{轨迹id , 节点索引}映射==>节点数据(时间戳, 局部位姿, 全局位姿, 重力方向)的容器
    MapById<NodeId, NodeSpec2D> node_data_;
    // 按SubmapId{轨迹id , 子图索引}映射==>子图全局坐标的容器
    MapById<SubmapId, SubmapSpec2D> submap_data_;
    // 路标点数据映射 <路标id , ???>
    std::map<std::string, transform::Rigid3d> landmark_data_;
    sensor::MapByTime<sensor::ImuData> imu_data_;               //(按时间戳映射的imu数据容器)
    sensor::MapByTime<sensor::OdometryData> odometry_data_;     //(按时间戳映射的里程计数据容器)
};

}  // namespace optimization
}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_OPTIMIZATION_OPTIMIZATION_PROBLEM_2D_H_
