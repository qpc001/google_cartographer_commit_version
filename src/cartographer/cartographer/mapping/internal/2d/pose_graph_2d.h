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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_2D_POSE_GRAPH_2D_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_2D_POSE_GRAPH_2D_H_

#include <deque>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <unordered_map>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/common/fixed_ratio_sampler.h"
#include "cartographer/common/mutex.h"
#include "cartographer/common/thread_pool.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/mapping/internal/constraints/constraint_builder_2d.h"
#include "cartographer/mapping/internal/optimization/optimization_problem_2d.h"
#include "cartographer/mapping/internal/trajectory_connectivity_state.h"
#include "cartographer/mapping/pose_graph.h"
#include "cartographer/mapping/pose_graph_trimmer.h"
#include "cartographer/sensor/fixed_frame_pose_data.h"
#include "cartographer/sensor/landmark_data.h"
#include "cartographer/sensor/odometry_data.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"

namespace cartographer {
namespace mapping {

// Implements the loop closure method called Sparse Pose Adjustment (SPA) from
// Konolige, Kurt, et al. "Efficient sparse pose adjustment for 2d mapping."
// Intelligent Robots and Systems (IROS), 2010 IEEE/RSJ International Conference
// on (pp. 22--29). IEEE, 2010.
//
// It is extended for submapping:
// Each node has been matched against one or more submaps (adding a constraint
// for each match), both poses of nodes and of submaps are to be optimized.
// All constraints are between a submap i and a node j.
// 每一个节点(位姿)都与一张或多张子图进行匹配(即它们之间添加了约束) , 节点位姿和子图都将被优化
// 所有的约束都是 子图i 和节点j 之间的
class PoseGraph2D : public PoseGraph {
public:
    // 构造函数, 使用传进来的参数对成员变量进行初始化(配置选项 ,优化问题, 线程池 )
    PoseGraph2D(
            const proto::PoseGraphOptions& options,
            std::unique_ptr<optimization::OptimizationProblem2D> optimization_problem,
            common::ThreadPool* thread_pool);
    ~PoseGraph2D() override;

    PoseGraph2D(const PoseGraph2D&) = delete;
    PoseGraph2D& operator=(const PoseGraph2D&) = delete;

    // Adds a new node with 'constant_data'. Its 'constant_data->local_pose' was
    // determined by scan matching against 'insertion_submaps.front()' and the
    // node data was inserted into the 'insertion_submaps'. If
    // 'insertion_submaps.front().finished()' is 'true', data was inserted into
    // this submap for the last time.

    // AddNode的主要工作是把一个节点的数据加入到PoseGraph维护的trajectory_nodes_这个容器中。并返回加入的节点的Node
    NodeId AddNode(
            std::shared_ptr<const TrajectoryNode::Data> constant_data,
            int trajectory_id,
            const std::vector<std::shared_ptr<const Submap2D>>& insertion_submaps)
    EXCLUDES(mutex_);

    // 向优化问题添加imu数据
    void AddImuData(int trajectory_id, const sensor::ImuData& imu_data) override
    EXCLUDES(mutex_);
    void AddOdometryData(int trajectory_id,
                         const sensor::OdometryData& odometry_data) override
    EXCLUDES(mutex_);
    void AddFixedFramePoseData(
            int trajectory_id,
            const sensor::FixedFramePoseData& fixed_frame_pose_data) override
    EXCLUDES(mutex_);

    // 遍历landmark数据的landmark_observations
    // 将数据储存到landmark_nodes_ 容器中
    void AddLandmarkData(int trajectory_id,
                         const sensor::LandmarkData& landmark_data) override
    EXCLUDES(mutex_);

    void FinishTrajectory(int trajectory_id) override;
    bool IsTrajectoryFinished(int trajectory_id) const override REQUIRES(mutex_);
    void FreezeTrajectory(int trajectory_id) override;
    bool IsTrajectoryFrozen(int trajectory_id) const override REQUIRES(mutex_);
    void AddSubmapFromProto(const transform::Rigid3d& global_submap_pose,
                            const proto::Submap& submap) override;
    void AddNodeFromProto(const transform::Rigid3d& global_pose,
                          const proto::Node& node) override;
    void SetTrajectoryDataFromProto(const proto::TrajectoryData& data) override;
    void AddNodeToSubmap(const NodeId& node_id,
                         const SubmapId& submap_id) override;
    void AddSerializedConstraints(
            const std::vector<Constraint>& constraints) override;
    void AddTrimmer(std::unique_ptr<PoseGraphTrimmer> trimmer) override;
    void RunFinalOptimization() override;
    std::vector<std::vector<int>> GetConnectedTrajectories() const override;
    PoseGraphInterface::SubmapData GetSubmapData(const SubmapId& submap_id) const
    EXCLUDES(mutex_) override;
    MapById<SubmapId, PoseGraphInterface::SubmapData> GetAllSubmapData() const
    EXCLUDES(mutex_) override;
    MapById<SubmapId, SubmapPose> GetAllSubmapPoses() const
    EXCLUDES(mutex_) override;
    transform::Rigid3d GetLocalToGlobalTransform(int trajectory_id) const
    EXCLUDES(mutex_) override;
    MapById<NodeId, TrajectoryNode> GetTrajectoryNodes() const override
    EXCLUDES(mutex_);
    MapById<NodeId, TrajectoryNodePose> GetTrajectoryNodePoses() const override
    EXCLUDES(mutex_);
    std::map<std::string, transform::Rigid3d> GetLandmarkPoses() const override
    EXCLUDES(mutex_);
    void SetLandmarkPose(const std::string& landmark_id,
                         const transform::Rigid3d& global_pose) override
    EXCLUDES(mutex_);
    sensor::MapByTime<sensor::ImuData> GetImuData() const override
    EXCLUDES(mutex_);
    sensor::MapByTime<sensor::OdometryData> GetOdometryData() const override
    EXCLUDES(mutex_);
    sensor::MapByTime<sensor::FixedFramePoseData> GetFixedFramePoseData()
    const override EXCLUDES(mutex_);
    std::map<std::string /* landmark ID */, PoseGraph::LandmarkNode>
    GetLandmarkNodes() const override EXCLUDES(mutex_);
    std::map<int, TrajectoryData> GetTrajectoryData() const override
    EXCLUDES(mutex_);
    std::vector<Constraint> constraints() const override EXCLUDES(mutex_);
    void SetInitialTrajectoryPose(int from_trajectory_id, int to_trajectory_id,
                                  const transform::Rigid3d& pose,
                                  const common::Time time) override
    EXCLUDES(mutex_);
    void SetGlobalSlamOptimizationCallback(
            PoseGraphInterface::GlobalSlamOptimizationCallback callback) override;

    // 根据时间戳进行插值,返回给定轨迹和时间戳 经过插值后的全局位姿
    transform::Rigid3d GetInterpolatedGlobalTrajectoryPose(
            int trajectory_id, const common::Time time) const REQUIRES(mutex_);

private:
    // The current state of the submap in the background threads. When this
    // transitions to kFinished, all nodes are tried to match against this submap.
    // Likewise, all new nodes are matched against submaps which are finished.
    // 描述了子图的当前状态, 当状态变成`kFinished` 时, 表示子图完成
    // 那么所有的节点都会尝试与这个子图进行匹配
    enum class SubmapState { kActive, kFinished };  //子图状态结构体
    struct InternalSubmapData {
        std::shared_ptr<const Submap2D> submap; //2D子图

        // IDs of the nodes that were inserted into this map together with
        // constraints for them. They are not to be matched again when this submap
        // becomes 'finished'.
        // 节点的ID集合
        std::set<NodeId> node_ids;

        // 子图状态
        SubmapState state = SubmapState::kActive;
    };

    MapById<SubmapId, PoseGraphInterface::SubmapData> GetSubmapDataUnderLock()
    const REQUIRES(mutex_);

    // Handles a new work item.
    void AddWorkItem(const std::function<void()>& work_item) REQUIRES(mutex_);

    // Adds connectivity and sampler for a trajectory if it does not exist.
    void AddTrajectoryIfNeeded(int trajectory_id) REQUIRES(mutex_);

    // Grows the optimization problem to have an entry for every element of
    // 'insertion_submaps'. Returns the IDs for the 'insertion_submaps'.
    // 该函数的主要工作就是指定一个trajectory_id的情况下，返回当前正处于活跃状态下的submap的id
    // 也就是系统正在维护的insertion_submaps的两个submap的id
    // insertion_submaps可能为空  也可能当前只有一个元素，也可能已经有两个元素了
    std::vector<SubmapId> InitializeGlobalSubmapPoses(
            int trajectory_id, const common::Time time,
            const std::vector<std::shared_ptr<const Submap2D>>& insertion_submaps)
    REQUIRES(mutex_);

    // Adds constraints for a node, and starts scan matching in the background.
    void ComputeConstraintsForNode(
            const NodeId& node_id,
            std::vector<std::shared_ptr<const Submap2D>> insertion_submaps,
            bool newly_finished_submap) REQUIRES(mutex_);

    // Computes constraints for a node and submap pair.
    void ComputeConstraint(const NodeId& node_id, const SubmapId& submap_id)
    REQUIRES(mutex_);

    // Adds constraints for older nodes whenever a new submap is finished.
    void ComputeConstraintsForOldNodes(const SubmapId& submap_id)
    REQUIRES(mutex_);

    // Runs the optimization, executes the trimmers and processes the work queue.
    void HandleWorkQueue(const constraints::ConstraintBuilder2D::Result& result)
    REQUIRES(mutex_);

    // Waits until we caught up (i.e. nothing is waiting to be scheduled), and
    // all computations have finished.
    void WaitForAllComputations() EXCLUDES(mutex_);

    // Runs the optimization. Callers have to make sure, that there is only one
    // optimization being run at a time.
    // 运行全局优化 , 获取优化结果,
    // 并且对新加进来的节点(指在优化过程中新加进来的节点: 这些节点没有得到优化)进行修正
    void RunOptimization() EXCLUDES(mutex_);

    // Computes the local to global map frame transform based on the given
    // 'global_submap_poses'.
    // 计算local 到全局地图的变换
    transform::Rigid3d ComputeLocalToGlobalTransform(
            const MapById<SubmapId, optimization::SubmapSpec2D>& global_submap_poses,
            int trajectory_id) const REQUIRES(mutex_);

    // 获取子图数据{子图 , 子图的全局位姿}
    SubmapData GetSubmapDataUnderLock(const SubmapId& submap_id) const
    REQUIRES(mutex_);

    common::Time GetLatestNodeTime(const NodeId& node_id,
                                   const SubmapId& submap_id) const
    REQUIRES(mutex_);

    // Updates the trajectory connectivity structure with a new constraint.
    void UpdateTrajectoryConnectivity(const Constraint& constraint)
    REQUIRES(mutex_);

    const proto::PoseGraphOptions options_;                                     //位姿图的各种配置
    GlobalSlamOptimizationCallback global_slam_optimization_callback_;          //完成全局优化后的回调函数
    mutable common::Mutex mutex_;                                               //线程锁

    // If it exists, further work items must be added to this queue, and will be
    // considered later.
    // 这是一个智能指针形式的工作队列，用于记录将要完成的任务
    std::unique_ptr<std::deque<std::function<void()>>> work_queue_              //工作队列
                                                     GUARDED_BY(mutex_);

    // How our various trajectories are related.
    // 描述不同轨迹之间的 连接状态
    TrajectoryConnectivityState trajectory_connectivity_state_;

    // We globally localize a fraction of the nodes from each trajectory.
    // 这应该是一个以trajectory_id为索引的字典，用于对各个轨迹上的部分节点进行全局定位
    std::unordered_map<int, std::unique_ptr<common::FixedRatioSampler>>
    global_localization_samplers_ GUARDED_BY(mutex_);

    // Number of nodes added since last loop closure.
    // 一个计数器，记录了自从上次闭环检测之后新增的节点数量
    int num_nodes_since_last_loop_closure_ GUARDED_BY(mutex_) = 0;

    // Whether the optimization has to be run before more data is added.
    // 在更多的数据插入之前, 是否需要进行一次优化
    bool run_loop_closure_ GUARDED_BY(mutex_) = false;

    // Schedules optimization (i.e. loop closure) to run.
    void DispatchOptimization() REQUIRES(mutex_);

    // Current optimization problem.
    // 描述当前优化问题的对象，应该是PoseGraph2D的核心
    std::unique_ptr<optimization::OptimizationProblem2D> optimization_problem_;
    // 约束构造器，用于异步的计算约束
    constraints::ConstraintBuilder2D constraint_builder_ GUARDED_BY(mutex_);
    // 记录了位姿图中的所有约束
    std::vector<Constraint> constraints_ GUARDED_BY(mutex_);

    // Submaps get assigned an ID and state as soon as they are seen, even
    // before they take part in the background computations.
    // 子图一旦被观测到,就分配了ID和状态 , 尽管它们还没有参与后端优化
    // 该容器记录了所有的子图数据及其内部节点，
    // 其中MapById是对std::map的一个封装，InternalSubmapData除了描述了子图的数据之外还记录了所有内部的节点
    MapById<SubmapId, InternalSubmapData> submap_data_ GUARDED_BY(mutex_);

    // Data that are currently being shown.
    // 记录轨迹节点的容器
    MapById<NodeId, TrajectoryNode> trajectory_nodes_ GUARDED_BY(mutex_);
    // 轨迹节点数量
    int num_trajectory_nodes_ GUARDED_BY(mutex_) = 0;

    // Global submap poses currently used for displaying data. (用于可视化)
    // 记录优化过的子图全局位姿
    MapById<SubmapId, optimization::SubmapSpec2D> global_submap_poses_
    GUARDED_BY(mutex_);

    // Global landmark poses with all observations.
    // 记录路标点的容器
    std::map<std::string /* landmark ID */, PoseGraph::LandmarkNode>
    landmark_nodes_ GUARDED_BY(mutex_);

    // List of all trimmers to consult when optimizations finish.
    // 用于指导修正地图的修饰器 , 当优化完成之后使用
    std::vector<std::unique_ptr<PoseGraphTrimmer>> trimmers_ GUARDED_BY(mutex_);

    // Set of all frozen trajectories not being optimized.
    // 记录所有当前冻结(不参与优化)的轨迹
    std::set<int> frozen_trajectories_ GUARDED_BY(mutex_);

    // Set of all finished trajectories.
    // 记录所有已经完结的轨迹
    std::set<int> finished_trajectories_ GUARDED_BY(mutex_);

    // Set of all initial trajectory poses.
    // 记录所有轨迹的初始位姿
    std::map<int, InitialTrajectoryPose> initial_trajectory_poses_
    GUARDED_BY(mutex_);

    // Allows querying and manipulating the pose graph by the 'trimmers_'. The
    // 'mutex_' of the pose graph is held while this class is used.
    // 允许 'trimmers_' 对位姿图进行查询和操作
    class TrimmingHandle : public Trimmable {
    public:
        //构造函数
        TrimmingHandle(PoseGraph2D* parent);
        ~TrimmingHandle() override {}

        //返回子图数量
        int num_submaps(int trajectory_id) const override;
        //返回子图id
        std::vector<SubmapId> GetSubmapIds(int trajectory_id) const override;
        // 返回优化之后的子图数据
        MapById<SubmapId, SubmapData> GetOptimizedSubmapData() const override
        REQUIRES(parent_->mutex_);
        // 返回父对象的trajectory_nodes_ (记录轨迹节点的容器)
        const MapById<NodeId, TrajectoryNode>& GetTrajectoryNodes() const override
        REQUIRES(parent_->mutex_);
        // 返回父对象的constraints_ (记录约束的容器)
        const std::vector<Constraint>& GetConstraints() const override
        REQUIRES(parent_->mutex_);

        // 标记某个子图需要被trim掉
        // 对父对象的constraints_ , submap_data_ , constraint_builder_ , optimization_problem_
        // 进行修整 ,去除与这个子图有关的数据 , 以及与这个子图构成约束的节点 的相关数据
        void MarkSubmapAsTrimmed(const SubmapId& submap_id)
        REQUIRES(parent_->mutex_) override;

        // 返回 给定轨迹ID的轨迹是否结束
        bool IsFinished(int trajectory_id) const override REQUIRES(parent_->mutex_);

    private:
        PoseGraph2D* const parent_; //父对象
    };
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_2D_POSE_GRAPH_2D_H_
