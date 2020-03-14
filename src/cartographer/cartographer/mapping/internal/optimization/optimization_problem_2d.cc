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

#include "cartographer/mapping/internal/optimization/optimization_problem_2d.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "cartographer/common/ceres_solver_options.h"
#include "cartographer/common/histogram.h"
#include "cartographer/common/math.h"
#include "cartographer/mapping/internal/optimization/ceres_pose.h"
#include "cartographer/mapping/internal/optimization/cost_functions/landmark_cost_function_2d.h"
#include "cartographer/mapping/internal/optimization/cost_functions/spa_cost_function_2d.h"
#include "cartographer/sensor/odometry_data.h"
#include "cartographer/transform/transform.h"
#include "ceres/ceres.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace optimization {
namespace {

using ::cartographer::mapping::optimization::CeresPose;
using LandmarkNode = ::cartographer::mapping::PoseGraphInterface::LandmarkNode;

// Converts a pose into the 3 optimization variable format used for Ceres:
// translation in x and y, followed by the rotation angle representing the
// orientation.
std::array<double, 3> FromPose(const transform::Rigid2d& pose) {
    return {{pose.translation().x(), pose.translation().y(),
                    pose.normalized_angle()}};
}

// Converts a pose as represented for Ceres back to an transform::Rigid2d pose.
transform::Rigid2d ToPose(const std::array<double, 3>& values) {
    return transform::Rigid2d({values[0], values[1]}, values[2]);
}

// Selects a trajectory node closest in time to the landmark observation and
// applies a relative transform from it.
transform::Rigid3d GetInitialLandmarkPose(
        const LandmarkNode::LandmarkObservation& observation,
        const NodeSpec2D& prev_node, const NodeSpec2D& next_node,
        const std::array<double, 3>& prev_node_pose,
        const std::array<double, 3>& next_node_pose) {
    const double interpolation_parameter =
            common::ToSeconds(observation.time - prev_node.time) /
            common::ToSeconds(next_node.time - prev_node.time);

    const std::tuple<std::array<double, 4>, std::array<double, 3>>
            rotation_and_translation =
            InterpolateNodes2D(prev_node_pose.data(), prev_node.gravity_alignment,
                               next_node_pose.data(), next_node.gravity_alignment,
                               interpolation_parameter);
    return transform::Rigid3d::FromArrays(std::get<0>(rotation_and_translation),
                                          std::get<1>(rotation_and_translation)) *
            observation.landmark_to_tracking_transform;
}

void AddLandmarkCostFunctions(
        const std::map<std::string, LandmarkNode>& landmark_nodes,
        bool freeze_landmarks, const MapById<NodeId, NodeSpec2D>& node_data,
        MapById<NodeId, std::array<double, 3>>* C_nodes,
        std::map<std::string, CeresPose>* C_landmarks, ceres::Problem* problem) {
    for (const auto& landmark_node : landmark_nodes) {
        // Do not use landmarks that were not optimized for localization.
        if (!landmark_node.second.global_landmark_pose.has_value() &&
                freeze_landmarks) {
            continue;
        }
        for (const auto& observation : landmark_node.second.landmark_observations) {
            const std::string& landmark_id = landmark_node.first;
            const auto& begin_of_trajectory =
                    node_data.BeginOfTrajectory(observation.trajectory_id);
            // The landmark observation was made before the trajectory was created.
            if (observation.time < begin_of_trajectory->data.time) {
                continue;
            }
            // Find the trajectory nodes before and after the landmark observation.
            auto next =
                    node_data.lower_bound(observation.trajectory_id, observation.time);
            // The landmark observation was made, but the next trajectory node has
            // not been added yet.
            if (next == node_data.EndOfTrajectory(observation.trajectory_id)) {
                continue;
            }
            if (next == begin_of_trajectory) {
                next = std::next(next);
            }
            auto prev = std::prev(next);
            // Add parameter blocks for the landmark ID if they were not added before.
            std::array<double, 3>* prev_node_pose = &C_nodes->at(prev->id);
            std::array<double, 3>* next_node_pose = &C_nodes->at(next->id);
            if (!C_landmarks->count(landmark_id)) {
                const transform::Rigid3d starting_point =
                        landmark_node.second.global_landmark_pose.has_value()
                        ? landmark_node.second.global_landmark_pose.value()
                        : GetInitialLandmarkPose(observation, prev->data, next->data,
                                                 *prev_node_pose, *next_node_pose);
                C_landmarks->emplace(
                            landmark_id,
                            CeresPose(starting_point, nullptr /* translation_parametrization */,
                                      common::make_unique<ceres::QuaternionParameterization>(),
                                      problem));
                if (freeze_landmarks) {
                    problem->SetParameterBlockConstant(
                                C_landmarks->at(landmark_id).translation());
                    problem->SetParameterBlockConstant(
                                C_landmarks->at(landmark_id).rotation());
                }
            }
            problem->AddResidualBlock(
                        LandmarkCostFunction2D::CreateAutoDiffCostFunction(
                            observation, prev->data, next->data),
                        nullptr /* loss function */, prev_node_pose->data(),
                        next_node_pose->data(), C_landmarks->at(landmark_id).rotation(),
                        C_landmarks->at(landmark_id).translation());
        }
    }
}

}  // namespace

OptimizationProblem2D::OptimizationProblem2D(
        const proto::OptimizationProblemOptions& options)
    : options_(options) {}

OptimizationProblem2D::~OptimizationProblem2D() {}

// 将imu数据添加到imu_data_ 成员变量 (按时间戳映射的imu数据容器)
// 尼玛的, 2D模式下IMU在全局BA优化没有起作用, 加进来做莫子?
void OptimizationProblem2D::AddImuData(const int trajectory_id,
                                       const sensor::ImuData& imu_data) {
    //sensor::MapByTime<sensor::ImuData> imu_data_ : 按时间戳映射的imu数据容器
    imu_data_.Append(trajectory_id, imu_data);
}

// 将里程计数据添加到odometry_data_ , (按时间戳映射的里程计数据容器)
void OptimizationProblem2D::AddOdometryData(
        const int trajectory_id, const sensor::OdometryData& odometry_data) {
    odometry_data_.Append(trajectory_id, odometry_data);
}

// 将节点数据(时间戳, 局部位姿, 全局位姿, 重力方向) [追加]到node_data_
// node_data_ (按NodeId{轨迹id , 节点索引}映射==>节点数据的容器)
void OptimizationProblem2D::AddTrajectoryNode(const int trajectory_id,
                                              const NodeSpec2D& node_data) {
    node_data_.Append(trajectory_id, node_data);
}

// 将节点数据(时间戳, 局部位姿, 全局位姿, 重力方向) [插入]到node_data_
// node_data_ (按NodeId{轨迹id , 节点索引}映射==>节点数据的容器)
void OptimizationProblem2D::InsertTrajectoryNode(const NodeId& node_id,
                                                 const NodeSpec2D& node_data) {
    node_data_.Insert(node_id, node_data);
}

// 移除`node_id`节点及其数据
void OptimizationProblem2D::TrimTrajectoryNode(const NodeId& node_id) {
    // 一旦`node_id`从 nodes容器中移除的时候
    // 同时移除不再需要的数据
    imu_data_.Trim(node_data_, node_id);
    odometry_data_.Trim(node_data_, node_id);
    node_data_.Trim(node_id);
}

// 给定轨迹id ,和子图全局位姿 , [追加]子图全局位姿到submap_data_
void OptimizationProblem2D::AddSubmap(
        const int trajectory_id,                        //轨迹id
        const transform::Rigid2d& global_submap_pose) { //子图的全局位姿
    // submap_data_ 按SubmapId{轨迹id , 子图索引}映射==>子图全局坐标的容器
    // .Append() : 根据轨迹id 来[追加]数据 (即自动分配子图索引)
    submap_data_.Append(trajectory_id, SubmapSpec2D{global_submap_pose});
}

// 给定轨迹id ,和子图全局位姿 , [插入]子图全局位姿到submap_data_
void OptimizationProblem2D::InsertSubmap(
        const SubmapId& submap_id,                      //SubmapId{轨迹id , 子图索引}
        const transform::Rigid2d& global_submap_pose) { //子图全局位姿
    submap_data_.Insert(submap_id, SubmapSpec2D{global_submap_pose});
}

// 从submap_data_移除 数据
void OptimizationProblem2D::TrimSubmap(const SubmapId& submap_id) {
    submap_data_.Trim(submap_id);
}

// 设置最大迭代次数
void OptimizationProblem2D::SetMaxNumIterations(
        const int32 max_num_iterations) {
    options_.mutable_ceres_solver_options()->set_max_num_iterations(
                max_num_iterations);
}

void OptimizationProblem2D::Solve(
        const std::vector<Constraint>& constraints,                 //约束
        const std::set<int>& frozen_trajectories,                   //冻结的轨迹
        const std::map<std::string, LandmarkNode>& landmark_nodes) {//路标
    // 如果没有数据,则返回
    if (node_data_.empty()) {
        // Nothing to optimize.
        return;
    }

    // 构造problem
    ceres::Problem::Options problem_options;
    ceres::Problem problem(problem_options);

    // Set the starting point.
    // TODO(hrapp): Move ceres data into SubmapSpec.
    // 设置起点
    MapById<SubmapId, std::array<double, 3>> C_submaps;     //要优化的子图位姿(全局位姿)
    MapById<NodeId, std::array<double, 3>> C_nodes;         //要优化的节点位姿(全局位姿)
    std::map<std::string, CeresPose> C_landmarks;           //要优化的landmark位姿(全局位姿)
    bool first_submap = true;                               //标志位 , 第一帧子图, 则不参与位姿优化,固定位姿[防止零空间漂移]
    bool freeze_landmarks = !frozen_trajectories.empty();
    // 遍历MapById<SubmapId, SubmapSpec2D> submap_data_ 由pose_graph2d插进来的子图数据?
    for (const auto& submap_id_data : submap_data_) {
        // 检查该子图所在轨迹id是否存在于需要冻结的列表
        const bool frozen =
                frozen_trajectories.count(submap_id_data.id.trajectory_id) != 0;
        // 取出当前遍历的子图对象{SubmapId , 子图的全局位姿}
        // 插入到C_submaps容器
        C_submaps.Insert(submap_id_data.id,
                         FromPose(submap_id_data.data.global_pose));
        // 向problem添加需要求解的参数
        // .AddParameterBlock(参数变量, 参数维度)
        problem.AddParameterBlock(C_submaps.at(submap_id_data.id).data(), 3);
        // 如果该子图所在的轨迹id存在于冻结列表 或者 该子图是第一帧子图
        // 则固定其全局位姿, 不参与优化
        if (first_submap || frozen) {
            first_submap = false;   //恢复标志位
            // Fix the pose of the first submap or all submaps of a frozen
            // trajectory.
            // 固定位姿: problem.SetParameterBlockConstant(需要固定的参数)
            problem.SetParameterBlockConstant(C_submaps.at(submap_id_data.id).data());
        }
    }
    // 遍历MapById<NodeId, NodeSpec2D> node_data_
    for (const auto& node_id_data : node_data_) {
        // 检查该节点所在轨迹id是否存在于需要冻结的列表
        const bool frozen =
                frozen_trajectories.count(node_id_data.id.trajectory_id) != 0;
        // 取出当前遍历节点元素{NodeId , 节点的全局位姿}
        // 插入到C_nodes容器
        C_nodes.Insert(node_id_data.id, FromPose(node_id_data.data.global_pose_2d));
        // 向problem添加需要求解的参数
        // .AddParameterBlock(参数变量, 参数维度)
        problem.AddParameterBlock(C_nodes.at(node_id_data.id).data(), 3);
        // 如果该节点所在轨迹是需要冻结的轨迹
        if (frozen) {
            // 则固定对应的参数: problem.SetParameterBlockConstant(需要固定的参数)
            problem.SetParameterBlockConstant(C_nodes.at(node_id_data.id).data());
        }
    }
    // Add cost functions for intra- and inter-submap constraints.
    /// 添加约束到cost function
    //  每个约束包含了(1. 子图i的{轨迹id , 子图索引} ,
    //              2. 节点j的{轨迹id,节点索引} ,
    //              3. 相对位姿和权重{从j坐标系到i坐标系的变换T_{i<-j} , 平移量权重 , 旋转量权重 })
    // 遍历约束 (来自形参) , 根据约束计算残差
    for (const Constraint& constraint : constraints) {
        // 添加残差项
        problem.AddResidualBlock(
                    CreateAutoDiffSpaCostFunction(constraint.pose),     //取约束的相对位姿和权重,构造cost func
                    // Only loop closure constraints should have a loss function.
                    constraint.tag == Constraint::INTER_SUBMAP          //根据标志(表明该约束是否回环生成的)是否使用核函数
                    ? new ceres::HuberLoss(options_.huber_scale())      //~
                    : nullptr,                                          //~
                    C_submaps.at(constraint.submap_id).data(),    //(待优化参数: 子图位姿) 按约束的submap_id取C_submaps容器对应的数据
                    C_nodes.at(constraint.node_id).data());       //(待优化参数: 节点位姿) 按约束的node_id取C_nodes容器对应的数据
    }
    // Add cost functions for landmarks.
    // 如果有路标, 把路标的cost func 也加入
    /// 暂时不考虑
    AddLandmarkCostFunctions(landmark_nodes, freeze_landmarks, node_data_,
                             &C_nodes, &C_landmarks, &problem);

    // Add penalties for violating odometry or changes between consecutive nodes
    // if odometry is not available.
    // 遍历MapById<NodeId, NodeSpec2D> node_data_
    for (auto node_it = node_data_.begin(); node_it != node_data_.end();) {
        // 取节点对应的轨迹ID
        const int trajectory_id = node_it->id.trajectory_id;
        // 
        const auto trajectory_end = node_data_.EndOfTrajectory(trajectory_id);
        // 如果这个轨迹被冻结
        if (frozen_trajectories.count(trajectory_id) != 0) {
            // 将node_it指向下一条轨迹的开头 , 用来下一个for
            // 其实啥也没干
            node_it = trajectory_end;
            continue;
        }

        // 否则,到这里就表示轨迹没有被冻结, 开始遍历这条轨迹上的每两个相邻节点
        // node_it为下的一条轨迹的起点
        auto prev_node_it = node_it;
        for (++node_it; node_it != trajectory_end; ++node_it) {
            //遍历这个被冻结的轨迹上的每个节点
            const NodeId first_node_id = prev_node_it->id;          //轨迹起点节点
            const NodeSpec2D& first_node_data = prev_node_it->data; //起点数据
            // 指向下一个
            prev_node_it = node_it;
            const NodeId second_node_id = node_it->id;              //相邻的下一个节点
            const NodeSpec2D& second_node_data = node_it->data;     //下一个节点数据

            // 检查是否是相邻的
            if (second_node_id.node_index != first_node_id.node_index + 1) {
                continue;
            }

            // Add a relative pose constraint based on the odometry (if available).
            // CalculateOdometryBetweenNodes : 利用里程计数据计算节点之间的相对位姿
            // relative_odometry : 第二个节点到第一个节点的坐标系变换
            std::unique_ptr<transform::Rigid3d> relative_odometry =
                    CalculateOdometryBetweenNodes(trajectory_id,    //轨迹ID
                                                  first_node_data,  //前一个节点
                                                  second_node_data);//下一个节点
            // 如果上面的里程计相对位姿不为空
            if (relative_odometry != nullptr) {
                // 添加残差项
                problem.AddResidualBlock(
                            CreateAutoDiffSpaCostFunction(                      //cost function
                                Constraint::Pose{
                                    *relative_odometry,
                                    options_.odometry_translation_weight(),
                                    options_.odometry_rotation_weight()}),
                            nullptr /* loss function */,                        //核函数
                            C_nodes.at(first_node_id).data(),                   //待优化参数(第一个节点的位姿数据)
                            C_nodes.at(second_node_id).data());                 //待优化参数
            }

            // Add a relative pose constraint based on consecutive local SLAM poses.
            // 再根据local SLAM计算出来的local位姿, 再计算一个相对位姿
            // relative_local_slam_pose: 第二个节点到第一个节点的变换
            const transform::Rigid3d relative_local_slam_pose =
                    transform::Embed3D(first_node_data.local_pose_2d.inverse() *    // (local map坐标系到第一个节点的变换)
                                       second_node_data.local_pose_2d);             // 第二个节点到local map坐标系的变换

            problem.AddResidualBlock(
                        CreateAutoDiffSpaCostFunction(                          //cost function
                            Constraint::Pose{
                                relative_local_slam_pose,
                                options_.local_slam_pose_translation_weight(),
                                options_.local_slam_pose_rotation_weight()}),
                        nullptr /* loss function */,
                        C_nodes.at(first_node_id).data(),                       //待优化参数(第一个节点的位姿数据)
                        C_nodes.at(second_node_id).data());                     //待优化参数
        }
    }

    // Solve.
    // 开始求解
    ceres::Solver::Summary summary;
    ceres::Solve(
                common::CreateCeresSolverOptions(options_.ceres_solver_options()),
                &problem, &summary);
    // 日志输出
    if (options_.log_solver_summary()) {
        LOG(INFO) << summary.FullReport();
    }

    // Store the result.
    // 储存所有优化结果到成员变量
    for (const auto& C_submap_id_data : C_submaps) {
        // 子图全局位姿
        submap_data_.at(C_submap_id_data.id).global_pose =
                ToPose(C_submap_id_data.data);
    }
    for (const auto& C_node_id_data : C_nodes) {
        // 节点全局位姿
        node_data_.at(C_node_id_data.id).global_pose_2d =
                ToPose(C_node_id_data.data);
    }
    for (const auto& C_landmark : C_landmarks) {
        // landmark全局位姿
        landmark_data_[C_landmark.first] = C_landmark.second.ToRigid();
    }
}

// 按时间戳得到里程计数据插值 , 返回该时间戳的机器人位姿估计
std::unique_ptr<transform::Rigid3d> OptimizationProblem2D::InterpolateOdometry(
        const int trajectory_id /*轨迹ID*/, const common::Time time /*时间戳*/) const {
    // 取该轨迹数据中的时间>给定参数`time` 的第一个元素
    const auto it = odometry_data_.lower_bound(trajectory_id, time);
    // 如果并没有
    if (it == odometry_data_.EndOfTrajectory(trajectory_id)) {
        // 则返回空
        return nullptr;
    }
    // 如果刚好是轨迹的起点
    if (it == odometry_data_.BeginOfTrajectory(trajectory_id)) {
        // 并且时间戳刚好等于输入参数
        if (it->time == time) {
            // 直接返回该节点的全局位姿
            return common::make_unique<transform::Rigid3d>(it->pose);
        }
        // 否则返回空
        return nullptr;
    }
    // it既不是起点也不是终点
    // 再取it的前一个元素
    const auto prev_it = std::prev(it);
    // 根据输入的时间戳`time` , 在这两个元素的全局位姿之间进行插值
    return common::make_unique<transform::Rigid3d>(
                Interpolate(transform::TimestampedTransform{prev_it->time, prev_it->pose},
                            transform::TimestampedTransform{it->time, it->pose}, time)
                .transform);
}

// 利用里程计数据计算节点之间的相对位姿
// 返回 : 第二个节点到第一个节点的坐标系变换
std::unique_ptr<transform::Rigid3d>
OptimizationProblem2D::CalculateOdometryBetweenNodes(
        const int trajectory_id,                        //轨迹ID
        const NodeSpec2D& first_node_data,              //第一个节点(时间戳, 局部位姿, 全局位姿, 重力方向)
        const NodeSpec2D& second_node_data) const {     //下一个节点
    // 如果里程计数据中有 这个轨迹ID的数据
    if (odometry_data_.HasTrajectory(trajectory_id)) {
        // 按时间戳得到里程计数据插值 , 返回该时间戳的机器人位姿估计
        const std::unique_ptr<transform::Rigid3d> first_node_odometry =
                InterpolateOdometry(trajectory_id, first_node_data.time);
        const std::unique_ptr<transform::Rigid3d> second_node_odometry =
                InterpolateOdometry(trajectory_id, second_node_data.time);
        // 如果两个节点都有对应的插值数据
        if (first_node_odometry != nullptr && second_node_odometry != nullptr) {
            // 计算两个节点的相对位姿
            // 得到 : 第二个节点到第一个节点的坐标系变换
            transform::Rigid3d relative_odometry =
                    transform::Rigid3d::Rotation(first_node_data.gravity_alignment) *
                    first_node_odometry->inverse() *                        // 第一个节点的位姿逆(世界坐标系到第一个节点的变换)
                    (*second_node_odometry) *                               // 第二个节点的位姿 (第二个节点到世界坐标系的变换)
                    transform::Rigid3d::Rotation(
                        second_node_data.gravity_alignment.inverse());
            // 返回
            return common::make_unique<transform::Rigid3d>(relative_odometry);
        }
    }
    return nullptr;
}

}  // namespace optimization
}  // namespace mapping
}  // namespace cartographer
