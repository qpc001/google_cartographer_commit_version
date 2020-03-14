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

#include "cartographer/mapping/internal/2d/pose_graph_2d.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <memory>
#include <sstream>
#include <string>

#include "Eigen/Eigenvalues"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/math.h"
#include "cartographer/mapping/proto/pose_graph/constraint_builder_options.pb.h"
#include "cartographer/sensor/compressed_point_cloud.h"
#include "cartographer/sensor/internal/voxel_filter.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

// 构造函数, 使用传进来的参数对成员变量进行初始化(配置选项 ,优化问题, 线程池 )
PoseGraph2D::PoseGraph2D(
        const proto::PoseGraphOptions& options,                                     //配置选项
        std::unique_ptr<optimization::OptimizationProblem2D> optimization_problem,  //优化问题
        common::ThreadPool* thread_pool)                                            //线程池
    : //初始化成员变量
      options_(options),
      optimization_problem_(std::move(optimization_problem)),                       //优化问题
      constraint_builder_(options_.constraint_builder_options(), thread_pool) {}    //约束生成器

PoseGraph2D::~PoseGraph2D() {
    WaitForAllComputations();
    common::MutexLocker locker(&mutex_);
    CHECK(work_queue_ == nullptr);
}

// 1. 该函数的主要工作就是指定一个trajectory_id的情况下，返回当前正处于活跃状态下的submap的id
// 也就是系统正在维护的insertion_submaps的两个submap的id
// insertion_submaps可能为空  也可能当前只有一个元素，也可能已经有两个元素了
// 2. 同时向优化问题optimization_problem_添加子图数据{轨迹id, 子图的全局位姿}  [只有在这个函数有添加子图到优化问题的操作]
std::vector<SubmapId> PoseGraph2D::InitializeGlobalSubmapPoses(
        const int trajectory_id,                                                //轨迹ID
        const common::Time time,                                                //时间戳
        const std::vector<std::shared_ptr<const Submap2D>>& insertion_submaps) {//局部SLAM当前的子图(一个或两个甚至0个)
    CHECK(!insertion_submaps.empty());

    // 从优化问题取优化过的子图数据 (主要是子图的全局位姿)
    const auto& submap_data = optimization_problem_->submap_data();
    //如果局部SLAM当前的子图size为1 , 表示这条轨迹刚刚初始化
    if (insertion_submaps.size() == 1) {
        // If we don't already have an entry for the first submap, add one.
        // SizeOfTrajectoryOrZero: 返回一个指定Id的元素（这里的元素是SubmapSpec2D）的数据的size，如果该id不存在，则返回0
        // 如果判断指定id的submap_data的size为0，说明该trajectory_id上还没有submap数据，那么就需要建立一个submap
        if (submap_data.SizeOfTrajectoryOrZero(trajectory_id) == 0) {
            //检查这个轨迹有没有初始位姿
            if (initial_trajectory_poses_.count(trajectory_id) > 0) {
                // 有初始位姿, 则基于初始位姿数据中的`to_trajectory_id`
                // 更新轨迹之间的连接关系 (因为从前面的判断知道了这条轨迹是新的轨迹)
                trajectory_connectivity_state_.Connect(
                            trajectory_id,
                            initial_trajectory_poses_.at(trajectory_id).to_trajectory_id, time);
            }
            // 向优化问题添加子图信息{轨迹id, 子图的全局位姿}
            optimization_problem_->AddSubmap(
                        trajectory_id,
                        transform::Project2D(ComputeLocalToGlobalTransform(             //local map坐标系到全局坐标系的变换
                                                 global_submap_poses_, trajectory_id) *
                                             insertion_submaps[0]->local_pose())); //子图坐标系原点在 local map坐标系的坐标 (子图到local map坐标系的变换)
        }
        CHECK_EQ(1, submap_data.SizeOfTrajectoryOrZero(trajectory_id));
        // {轨迹id , 子图索引}
        const SubmapId submap_id{trajectory_id, 0};
        CHECK(submap_data_.at(submap_id).submap == insertion_submaps.front());
        // 返回{轨迹id , 子图索引=0}
        return {submap_id};
    }
    CHECK_EQ(2, insertion_submaps.size());

    // 获取submap_data的末尾trajectory_id(实际上不是末尾,而是下一个ID的轨迹开头)
    const auto end_it = submap_data.EndOfTrajectory(trajectory_id);
    // 检查开头是否不等于末尾。如果等于，说明这个容器里没有一个元素
    CHECK(submap_data.BeginOfTrajectory(trajectory_id) != end_it);

    // end_it的前的一个submap的id才是submap_data中的最后一个元素
    const SubmapId last_submap_id = std::prev(end_it)->id;
    // 如果是等于insertion_submaps的第一个元素，说明insertion_submaps的第二个元素还没有分配了id
    if (submap_data_.at(last_submap_id).submap == insertion_submaps.front()) {
        // In this case, 'last_submap_id' is the ID of
        // 'insertions_submaps.front()' and 'insertions_submaps.back()' is new.
        // 这种情况下，要给新的submap分配id，并把它加到OptimizationProblem的submap_data_这个容器中
        const auto& first_submap_pose = submap_data.at(last_submap_id).global_pose;
        // 向优化问题添加子图信息{轨迹id, 子图的全局位姿}
        optimization_problem_->AddSubmap(
                    trajectory_id,
                    first_submap_pose *
                    constraints::ComputeSubmapPose(*insertion_submaps[0]).inverse() *
                constraints::ComputeSubmapPose(*insertion_submaps[1]));
        // 返回一个包含两个Submap的向量
        return {last_submap_id,
                    SubmapId{trajectory_id, last_submap_id.submap_index + 1}};
    }
    // 如果是等于insertion_submaps的第二个元素，
    // 说明insertion_submaps的第二个元素也已经分配了id并加入到了OptimizationProblem的submap_data_中
    CHECK(submap_data_.at(last_submap_id).submap == insertion_submaps.back());
    // 生成一个submap的Id, 这个id是submap_data_的倒数第二个
    const SubmapId front_submap_id{trajectory_id,
                last_submap_id.submap_index - 1};
    CHECK(submap_data_.at(front_submap_id).submap == insertion_submaps.front());
    // 把这两个submap的id返回
    return {front_submap_id, last_submap_id};
}

// AddNode的主要工作是把一个节点的数据加入到PoseGraph维护的trajectory_nodes_这个容器中并,返回加入的节点的Node
// 其次,还检查 local SLAM当前维护的子图中的 font子图是否完成, 如果子图完成,则更新一下约束关系
// 每一个子图完成, 都会尝试与所有的节点进行 约束的生成
NodeId PoseGraph2D::AddNode(
        std::shared_ptr<const TrajectoryNode::Data> constant_data,              //某个节点数据
        const int trajectory_id,                                                //轨迹ID
        const std::vector<std::shared_ptr<const Submap2D>>& insertion_submaps) {//正在维护的子图(一个或两个甚至0个)
    // 得到 节点在世界坐标系的坐标
    const transform::Rigid3d optimized_pose(
                GetLocalToGlobalTransform(trajectory_id) *  // local map 到全局坐标系的变换
                constant_data->local_pose);                 // 节点(某个时刻的机器人)坐标系到local mao的变换

    common::MutexLocker locker(&mutex_);
    // 向connectivity_state_增加这个轨迹id的记录
    AddTrajectoryIfNeeded(trajectory_id);
    // 把该节点[追加]到PoseGraph2D维护的trajectory_nodes_这个容器中,返回节点ID
    const NodeId node_id = trajectory_nodes_.Append(
                trajectory_id, TrajectoryNode{constant_data, optimized_pose});
    // 节点数加1.
    ++num_trajectory_nodes_;

    // Test if the 'insertion_submap.back()' is one we never saw before.
    // 如果 该ID的轨迹还没有子图 或者
    // submap_data_容器中记录的轨迹的最后一个子图 不是 'insertion_submap.back()'
    // 则表明(insertion_submap.back()所对应的子图是新的 ,还没加入到容器)
    if (submap_data_.SizeOfTrajectoryOrZero(trajectory_id) == 0 ||
            std::prev(submap_data_.EndOfTrajectory(trajectory_id))->data.submap !=
            insertion_submaps.back()) {
        // We grow 'submap_data_' as needed. This code assumes that the first
        // time we see a new submap is as 'insertion_submaps.back()'.

        // 记录insertion_submap.back()所对应的子图到容器中
        // 把insertion_submaps的第二个元素加到submap_data_中 , 返回子图ID
        const SubmapId submap_id =
                submap_data_.Append(trajectory_id, InternalSubmapData());
        submap_data_.at(submap_id).submap = insertion_submaps.back();
    }

    // We have to check this here, because it might have changed by the time we
    // execute the lambda.
    // 检查insertion_submaps的第一个submap是否已经被finished了。
    // 如果被finished，那么我们需要计算一下约束并且进行一下全局优化了。
    // 这里是把这个工作交到了workItem中等待处理
    const bool newly_finished_submap = insertion_submaps.front()->finished();
    AddWorkItem([=]() REQUIRES(mutex_) {
        // 准备更新约束关系
        // 计算新添加的这个节点与所有Submap之间的约束关系 , 如果有子图完成, 也计算子图与所有节点之间的约束关系
        ComputeConstraintsForNode(node_id, insertion_submaps,
                                  newly_finished_submap);
    });
    // 上述都做完了，返回node_id.
    return node_id;
}

// 把一个函数地址加到工作队列中
void PoseGraph2D::AddWorkItem(const std::function<void()>& work_item) {
    if (work_queue_ == nullptr) {// 如果工作队列为空，那么直接执行该函数即可
        work_item();
    } else {
        // 否则的话暂时加入工作队列，等待处理
        work_queue_->push_back(work_item);
    }
}

// 直接添加一条轨迹
void PoseGraph2D::AddTrajectoryIfNeeded(const int trajectory_id) {
    // 增加一条trajectory的话就添加到TrajectoryConnectivityState中
    trajectory_connectivity_state_.Add(trajectory_id);
    // Make sure we have a sampler for this trajectory.
    // 确保这条轨迹有全局位姿采样器
    if (!global_localization_samplers_[trajectory_id]) {
        // 如果没有,就创建一个
        global_localization_samplers_[trajectory_id] =
                common::make_unique<common::FixedRatioSampler>(
                    options_.global_sampling_ratio());
    }
}

// 向优化问题添加imu数据
void PoseGraph2D::AddImuData(const int trajectory_id,
                             const sensor::ImuData& imu_data) {
    common::MutexLocker locker(&mutex_);
    AddWorkItem([=]() REQUIRES(mutex_) {
        optimization_problem_->AddImuData(trajectory_id, imu_data);
    });
}

// 向优化问题添加里程计数据
void PoseGraph2D::AddOdometryData(const int trajectory_id,
                                  const sensor::OdometryData& odometry_data) {
    common::MutexLocker locker(&mutex_);
    AddWorkItem([=]() REQUIRES(mutex_) {
        optimization_problem_->AddOdometryData(trajectory_id, odometry_data);
    });
}

// 向优化问题添加gps数据 (空函数,还没实现)
void PoseGraph2D::AddFixedFramePoseData(
        const int trajectory_id,
        const sensor::FixedFramePoseData& fixed_frame_pose_data) {
    LOG(FATAL) << "Not yet implemented for 2D.";
}

// 遍历landmark数据的landmark_observations
// 将数据储存到landmark_nodes_ 容器中
void PoseGraph2D::AddLandmarkData(int trajectory_id,
                                  const sensor::LandmarkData& landmark_data)
EXCLUDES(mutex_) {
    common::MutexLocker locker(&mutex_);
    AddWorkItem([=]() REQUIRES(mutex_) {
        // 遍历landmark数据的landmark_observations
        for (const auto& observation : landmark_data.landmark_observations) {
            // 按landmark的id , 将landmark_observations 数据 储存到landmark_nodes_ 容器中
            landmark_nodes_[observation.id].landmark_observations.emplace_back(
                        LandmarkNode::LandmarkObservation{
                            trajectory_id,                              //轨迹id
                            landmark_data.time,                         //数据时间戳
                            observation.landmark_to_tracking_transform, //landmark到tracking坐标系的变换
                            observation.translation_weight, observation.rotation_weight});  //权重
        }
    });
}

//计算一个Node和一个submap之间的约束
//细节: 这里只调用了constraint_builder_来计算约束, 这些约束还储存在constraint_builder_.constraint容器里面
//      还没有拿出来, 最终在PoseGraph2D::HandleWorkQueue()函数才从constraint_builder_取出这些约束
void PoseGraph2D::ComputeConstraint(const NodeId& node_id,          //节点ID
                                    const SubmapId& submap_id) {    //子图ID
    // 检查子图是否完成
    CHECK(submap_data_.at(submap_id).state == SubmapState::kFinished);

    // 取节点和子图两者之间最新的时间
    const common::Time node_time = GetLatestNodeTime(node_id, submap_id);
    // 取这两条轨迹上一次的连接时间
    const common::Time last_connection_time =
            trajectory_connectivity_state_.LastConnectionTime(
                node_id.trajectory_id, submap_id.trajectory_id);
    // 如果节点和子图都在同一条轨迹上 或者 (节点和子图两者之间最新的时间) 比 (所在轨迹的上次连接时间+ 搜索延时) 还早
    // 表示可以尝试使用一个初始位姿来计算节点和子图之间的约束 (简单说就是还没跑太远)
    if (node_id.trajectory_id == submap_id.trajectory_id ||
            node_time <
            last_connection_time +
            common::FromSeconds(
                options_.global_constraint_search_after_n_seconds())) {
        // If the node and the submap belong to the same trajectory or if there
        // has been a recent global constraint that ties that node's trajectory to
        // the submap's trajectory, it suffices to do a match constrained to a
        // local search window.

        // 尝试使用一个初始位姿来计算节点和子图之间的约束
        // initial_relative_pose : 节点到子图坐标系的变换
        const transform::Rigid2d initial_relative_pose =
                optimization_problem_->submap_data().at(submap_id).global_pose.inverse() *  //全局坐标系到子图坐标系的变换
                optimization_problem_->node_data().at(node_id).global_pose_2d;  //节点到全局坐标系的变换
        // 调用约束生成器 , 调用 分枝定界+ceres位姿优化,
        // 看能不能匹配上, 尝试生成约束
        constraint_builder_.MaybeAddConstraint( //(用try不好吗? 神TM maybe)
                    submap_id,
                    submap_data_.at(submap_id).submap.get(),
                    node_id,
                    trajectory_nodes_.at(node_id).constant_data.get(),
                    initial_relative_pose);
    } else if (global_localization_samplers_[node_id.trajectory_id]->Pulse()) { //节点进行了全局位姿采样(即该节点有全局位姿?)
        // 如果跑太远了, 即有初始位姿也没用 (或者不是在同一条轨迹的)
        // 尝试在 子图的全局范围内进行匹配,  从而生成约束
        constraint_builder_.MaybeAddGlobalConstraint(
                    submap_id,
                    submap_data_.at(submap_id).submap.get(),
                    node_id,
                    trajectory_nodes_.at(node_id).constant_data.get());
    }
}

//对一个刚刚被finished的submap，遍历所有的Node, 并建立该submap与Node之间的约束
void PoseGraph2D::ComputeConstraintsForOldNodes(const SubmapId& submap_id) {
    // 取子图数据
    const auto& submap_data = submap_data_.at(submap_id);
    // 遍历节点
    for (const auto& node_id_data : optimization_problem_->node_data()) {
        const NodeId& node_id = node_id_data.id;
        //调用ComputeConstraint,计算一个Node和这个submap之间的约束
        if (submap_data.node_ids.count(node_id) == 0) {
            ComputeConstraint(node_id, submap_id);
        }
    }
}

// 计算新添加的这个节点与所有Submap之间的约束关系
// 约束指的就是一个节点和submap之间的相对位姿
void PoseGraph2D::ComputeConstraintsForNode(
        const NodeId& node_id,                                              //刚加入的节点ID
        std::vector<std::shared_ptr<const Submap2D>> insertion_submaps,     //Local Slam返回的insertion_submaps(也就是Local SLAM正在维护的子图)
        const bool newly_finished_submap) {                                 //是否有新finished的submap的判断标志

    //////////////////////这部分生成新添加的节点与Local Slam 当前的子图的约束////////////////////////
    // 获取节点数据 (在PoseGraph2D::AddNode()的时候插入的数据)
    const auto& constant_data = trajectory_nodes_.at(node_id).constant_data;
    // 根据节点数据的时间获取最新的submap的id
    const std::vector<SubmapId> submap_ids = InitializeGlobalSubmapPoses(
                node_id.trajectory_id, constant_data->time, insertion_submaps);
    CHECK_EQ(submap_ids.size(), insertion_submaps.size());
    // 获取这两个submap中前一个的id (即前面用来扫描匹配的子图)
    const SubmapId matching_id = submap_ids.front();
    // 计算该Node经过重力align后的相对位姿，即在local map中的位姿
    const transform::Rigid2d local_pose_2d = transform::Project2D(
                constant_data->local_pose *
                transform::Rigid3d::Rotation(constant_data->gravity_alignment.inverse()));
    // 计算该Node在世界坐标系中的绝对位姿
    const transform::Rigid2d global_pose_2d =
            optimization_problem_->submap_data().at(matching_id).global_pose *  // 子图坐标系到全局坐标系的变换
            constraints::ComputeSubmapPose(*insertion_submaps.front()).inverse() *  // local map坐标系到子图坐标系的变换
            local_pose_2d;  // 节点到local map的变换
    // 向优化问题添加节点数据 {轨迹id, (时间戳, 局部位姿, 全局位姿, 重力方向)}
    optimization_problem_->AddTrajectoryNode(
                matching_id.trajectory_id,
                optimization::NodeSpec2D{constant_data->time, local_pose_2d,
                                         global_pose_2d,
                                         constant_data->gravity_alignment});
    // 遍历Local Slam返回的insertion_submaps
    for (size_t i = 0; i < insertion_submaps.size(); ++i) {
        const SubmapId submap_id = submap_ids[i];
        // Even if this was the last node added to 'submap_id', the submap will
        // only be marked as finished in 'submap_data_' further below.
        // 检查指定id是否是kActive
        CHECK(submap_data_.at(submap_id).state == SubmapState::kActive);
        // 每个子图都对应许多节点,不是一对一的
        // 向这个子图所对应的节点集合node_ids加入 当前这个{node_id}
        submap_data_.at(submap_id).node_ids.emplace(node_id);
        /// 为什么这里可以直接得到约束? 因为当前这个node在Local Slam部分已经取得了与当前子图的相对位姿关系 (就是已经进行过scanMatching了)
        // 计算约束 (节点到子图坐标系的变换)
        const transform::Rigid2d constraint_transform =
                constraints::ComputeSubmapPose(*insertion_submaps[i]).inverse() * // local map坐标系到子图坐标系的变换
                local_pose_2d;  // 节点到local map的变换
        // 构造约束 {submap_id ,node_id , (从j坐标系到i坐标系的变换T_{i<-j} , 平移量权重 , 旋转量权重) , 约束类型}
        // 将构造的约束储存到constraints_
        constraints_.push_back(Constraint{submap_id,
                                          node_id,
                                          {transform::Embed3D(constraint_transform),
                                           options_.matcher_translation_weight(),
                                           options_.matcher_rotation_weight()},
                                          Constraint::INTRA_SUBMAP});
    }

    //////////////////////下面计算新添加的节点与之前的所有子图之间的约束////////////////////////////

    // 遍历所有的子图数据及其内部节点
    for (const auto& submap_id_data : submap_data_) {
        // 如果子图状态为kFinished
        if (submap_id_data.data.state == SubmapState::kFinished) {
            CHECK_EQ(submap_id_data.data.node_ids.count(node_id), 0);
            //计算一个Node和一个submap之间的约束
            //细节: 这里只调用了constraint_builder_来计算约束, 这些约束还储存在constraint_builder_.constraint容器里面
            //      还没有拿出来, 最终在PoseGraph2D::HandleWorkQueue()函数才从constraint_builder_取出这些约束
            ComputeConstraint(node_id, submap_id_data.id);
        }
    }

    //////////////////////下面是检查当前维护的子图的第一张是否完成////////////////////////////////
    //////////////////////如果完成,就遍历所有的Node, 建立与新完成子图的约束///////////////////////
    // 如果有新的刚被finished的submap
    if (newly_finished_submap) {
        //insertion_maps中的第一个是Old的那个，如果有刚被finished的submap，那一定是他
        const SubmapId finished_submap_id = submap_ids.front();
        // 获取该submap的数据
        InternalSubmapData& finished_submap_data =
                submap_data_.at(finished_submap_id);
        CHECK(finished_submap_data.state == SubmapState::kActive);
        // 把它设置成finished
        finished_submap_data.state = SubmapState::kFinished;
        // We have a new completed submap, so we look into adding constraints for
        // old nodes.
        //细节: 这里只调用了constraint_builder_来计算约束, 这些约束还储存在constraint_builder_.constraint容器里面
        //      还没有拿出来, 最终在PoseGraph2D::HandleWorkQueue()函数才从constraint_builder_取出这些约束
        //对一个刚刚被finished的submap，遍历所有的Node, 并建立其该submap与Node之间的约束
        ComputeConstraintsForOldNodes(finished_submap_id);
    }

    // 结束构建约束
    constraint_builder_.NotifyEndOfNode();
    // 记录自从上次闭环检测之后新增的节点数量 的计数器加1
    ++num_nodes_since_last_loop_closure_;
    CHECK(!run_loop_closure_);

    ///////////////////////这里检查是不是该进行全局BA优化了/////////////////////////////////
    // options_.optimize_every_n_nodes()每增加多少个节点进行一次优化 (设置为0,则不进行全局优化)
    // 如果自从上次闭环检测之后新增的节点数量 > 上面的这个值, 则进行一次优化
    if (options_.optimize_every_n_nodes() > 0 &&
            num_nodes_since_last_loop_closure_ > options_.optimize_every_n_nodes()) {
        // 分配优化任务
        DispatchOptimization();
    }
}

// 分配优化任务
void PoseGraph2D::DispatchOptimization() {
    run_loop_closure_ = true;
    // If there is a 'work_queue_' already, some other thread will take care.
    // 如果工作队列为空
    if (work_queue_ == nullptr) {
        // 创建一个
        work_queue_ = common::make_unique<std::deque<std::function<void()>>>();
        // 绑定回调 (在constraint_builder_处理完所有约束的生成之后,在线程池中处理这个回调)
        constraint_builder_.WhenDone(
                    std::bind(&PoseGraph2D::HandleWorkQueue, this, std::placeholders::_1));
    }
}
common::Time PoseGraph2D::GetLatestNodeTime(const NodeId& node_id,
                                            const SubmapId& submap_id) const {
    common::Time time = trajectory_nodes_.at(node_id).constant_data->time;
    const InternalSubmapData& submap_data = submap_data_.at(submap_id);
    if (!submap_data.node_ids.empty()) {
        const NodeId last_submap_node_id =
                *submap_data_.at(submap_id).node_ids.rbegin();
        time = std::max(
                    time, trajectory_nodes_.at(last_submap_node_id).constant_data->time);
    }
    return time;
}

// 根据新建立的约束, 更新连接关系
void PoseGraph2D::UpdateTrajectoryConnectivity(const Constraint& constraint) {  //给定某条约束
    CHECK_EQ(constraint.tag, Constraint::INTER_SUBMAP);
    // 取约束上的 节点和子图 两者之间最新的时间
    const common::Time time =
            GetLatestNodeTime(constraint.node_id, constraint.submap_id);
    // 在trajectory_connectivity_state_ 添加连接关系
    trajectory_connectivity_state_.Connect(constraint.node_id.trajectory_id,
                                           constraint.submap_id.trajectory_id,
                                           time);
}

// 这个回调, 由ConstraintBuilder2D中的线程池来执行
void PoseGraph2D::HandleWorkQueue(
        const constraints::ConstraintBuilder2D::Result& result) {   //由ConstraintBuilder2D建立约束之后的结果, 这些约束用完一次就丢了
    {
        // 线程锁
        common::MutexLocker locker(&mutex_);
        // 细节: 在函数PoseGraph2D::ComputeConstraint()里面,调用constraint_builder_计算的约束还储存在constraint_builder_中
        //      现在把这些约束拿出来, 也就是`result`储存的东西
        constraints_.insert(constraints_.end(), result.begin(), result.end());
    }
    // 运行全局优化 , 获取优化结果, 并且对新加进来的节点(指在优化过程中新加进来的节点: 这些节点没有得到优化)进行修正
    RunOptimization();

    //如果完成全局优化后的回调函数存在
    if (global_slam_optimization_callback_) {
        //
        std::map<int, NodeId> trajectory_id_to_last_optimized_node_id;
        std::map<int, SubmapId> trajectory_id_to_last_optimized_submap_id;
        {
            // 取优化结果
            common::MutexLocker locker(&mutex_);
            const auto& submap_data = optimization_problem_->submap_data();
            const auto& node_data = optimization_problem_->node_data();
            // 遍历优化之后的每条轨迹
            for (const int trajectory_id : node_data.trajectory_ids()) {
                //取该轨迹上的最后一个优化过的节点
                trajectory_id_to_last_optimized_node_id[trajectory_id] =
                        std::prev(node_data.EndOfTrajectory(trajectory_id))->id;
                //取该轨迹上的最后一个优化过的子图
                trajectory_id_to_last_optimized_submap_id[trajectory_id] =
                        std::prev(submap_data.EndOfTrajectory(trajectory_id))->id;
            }
        }
        // 将上面取得的{最后一个优化过的节点 , 最后一个优化过的子图}作为参数
        // 调用回调函数
        global_slam_optimization_callback_(
                    trajectory_id_to_last_optimized_submap_id,
                    trajectory_id_to_last_optimized_node_id);
    }

    // 线程锁
    common::MutexLocker locker(&mutex_);
    // 遍历由ConstraintBuilder2D建立约束之后的结果
    for (const Constraint& constraint : result) {
        // 根据新建立的约束, 更新连接关系
        UpdateTrajectoryConnectivity(constraint);
    }
    //
    TrimmingHandle trimming_handle(this);
    for (auto& trimmer : trimmers_) {
        trimmer->Trim(&trimming_handle);
    }
    trimmers_.erase(
                std::remove_if(trimmers_.begin(), trimmers_.end(),
                               [](std::unique_ptr<PoseGraphTrimmer>& trimmer) {
                    return trimmer->IsFinished();
                }),
            trimmers_.end());

    num_nodes_since_last_loop_closure_ = 0;
    run_loop_closure_ = false;
    while (!run_loop_closure_) {
        if (work_queue_->empty()) {
            work_queue_.reset();
            return;
        }
        work_queue_->front()();
        work_queue_->pop_front();
    }
    LOG(INFO) << "Remaining work items in queue: " << work_queue_->size();
    // We have to optimize again.
    constraint_builder_.WhenDone(
                std::bind(&PoseGraph2D::HandleWorkQueue, this, std::placeholders::_1));
}

void PoseGraph2D::WaitForAllComputations() {
    bool notification = false;
    common::MutexLocker locker(&mutex_);
    const int num_finished_nodes_at_start =
            constraint_builder_.GetNumFinishedNodes();
    while (!locker.AwaitWithTimeout(
               [this]() REQUIRES(mutex_) {
               return ((constraint_builder_.GetNumFinishedNodes() ==
                        num_trajectory_nodes_) &&
                       !work_queue_);
},
               common::FromSeconds(1.))) {
        std::ostringstream progress_info;
        progress_info << "Optimizing: " << std::fixed << std::setprecision(1)
                      << 100. *
                         (constraint_builder_.GetNumFinishedNodes() -
                          num_finished_nodes_at_start) /
                         (num_trajectory_nodes_ - num_finished_nodes_at_start)
                      << "%...";
        std::cout << "\r\x1b[K" << progress_info.str() << std::flush;
    }
    std::cout << "\r\x1b[KOptimizing: Done.     " << std::endl;
    constraint_builder_.WhenDone(
                [this,
                &notification](const constraints::ConstraintBuilder2D::Result& result) {
        common::MutexLocker locker(&mutex_);
        constraints_.insert(constraints_.end(), result.begin(), result.end());
        notification = true;
    });
    locker.Await([&notification]() { return notification; });
}

void PoseGraph2D::FinishTrajectory(const int trajectory_id) {
    common::MutexLocker locker(&mutex_);
    AddWorkItem([this, trajectory_id]() REQUIRES(mutex_) {
        CHECK_EQ(finished_trajectories_.count(trajectory_id), 0);
        finished_trajectories_.insert(trajectory_id);

        for (const auto& submap : submap_data_.trajectory(trajectory_id)) {
            submap_data_.at(submap.id).state = SubmapState::kFinished;
        }
        CHECK(!run_loop_closure_);
        DispatchOptimization();
    });
}

bool PoseGraph2D::IsTrajectoryFinished(const int trajectory_id) const {
    return finished_trajectories_.count(trajectory_id) > 0;
}

void PoseGraph2D::FreezeTrajectory(const int trajectory_id) {
    common::MutexLocker locker(&mutex_);
    trajectory_connectivity_state_.Add(trajectory_id);
    AddWorkItem([this, trajectory_id]() REQUIRES(mutex_) {
        CHECK_EQ(frozen_trajectories_.count(trajectory_id), 0);
        frozen_trajectories_.insert(trajectory_id);
    });
}

bool PoseGraph2D::IsTrajectoryFrozen(const int trajectory_id) const {
    return frozen_trajectories_.count(trajectory_id) > 0;
}

void PoseGraph2D::AddSubmapFromProto(
        const transform::Rigid3d& global_submap_pose, const proto::Submap& submap) {
    if (!submap.has_submap_2d()) {
        return;
    }

    const SubmapId submap_id = {submap.submap_id().trajectory_id(),
                                submap.submap_id().submap_index()};
    std::shared_ptr<const Submap2D> submap_ptr =
            std::make_shared<const Submap2D>(submap.submap_2d());
    const transform::Rigid2d global_submap_pose_2d =
            transform::Project2D(global_submap_pose);

    common::MutexLocker locker(&mutex_);
    AddTrajectoryIfNeeded(submap_id.trajectory_id);
    submap_data_.Insert(submap_id, InternalSubmapData());
    submap_data_.at(submap_id).submap = submap_ptr;
    // Immediately show the submap at the 'global_submap_pose'.
    global_submap_poses_.Insert(
                submap_id, optimization::SubmapSpec2D{global_submap_pose_2d});
    AddWorkItem([this, submap_id, global_submap_pose_2d]() REQUIRES(mutex_) {
        submap_data_.at(submap_id).state = SubmapState::kFinished;
        optimization_problem_->InsertSubmap(submap_id, global_submap_pose_2d);
    });
}

void PoseGraph2D::AddNodeFromProto(const transform::Rigid3d& global_pose,
                                   const proto::Node& node) {
    const NodeId node_id = {node.node_id().trajectory_id(),
                            node.node_id().node_index()};
    std::shared_ptr<const TrajectoryNode::Data> constant_data =
            std::make_shared<const TrajectoryNode::Data>(FromProto(node.node_data()));

    common::MutexLocker locker(&mutex_);
    AddTrajectoryIfNeeded(node_id.trajectory_id);
    trajectory_nodes_.Insert(node_id, TrajectoryNode{constant_data, global_pose});

    AddWorkItem([this, node_id, global_pose]() REQUIRES(mutex_) {
        const auto& constant_data = trajectory_nodes_.at(node_id).constant_data;
        const auto gravity_alignment_inverse = transform::Rigid3d::Rotation(
                    constant_data->gravity_alignment.inverse());
        optimization_problem_->InsertTrajectoryNode(
                    node_id,
                    optimization::NodeSpec2D{
                        constant_data->time,
                        transform::Project2D(constant_data->local_pose *
                        gravity_alignment_inverse),
                        transform::Project2D(global_pose * gravity_alignment_inverse),
                        constant_data->gravity_alignment});
    });
}

void PoseGraph2D::SetTrajectoryDataFromProto(
        const proto::TrajectoryData& data) {
    LOG(ERROR) << "not implemented";
}

void PoseGraph2D::AddNodeToSubmap(const NodeId& node_id,
                                  const SubmapId& submap_id) {
    common::MutexLocker locker(&mutex_);
    AddWorkItem([this, node_id, submap_id]() REQUIRES(mutex_) {
        submap_data_.at(submap_id).node_ids.insert(node_id);
    });
}

void PoseGraph2D::AddSerializedConstraints(
        const std::vector<Constraint>& constraints) {
    common::MutexLocker locker(&mutex_);
    AddWorkItem([this, constraints]() REQUIRES(mutex_) {
        for (const auto& constraint : constraints) {
            CHECK(trajectory_nodes_.Contains(constraint.node_id));
            CHECK(submap_data_.Contains(constraint.submap_id));
            CHECK(trajectory_nodes_.at(constraint.node_id).constant_data != nullptr);
            CHECK(submap_data_.at(constraint.submap_id).submap != nullptr);
            switch (constraint.tag) {
            case Constraint::Tag::INTRA_SUBMAP:
                CHECK(submap_data_.at(constraint.submap_id)
                      .node_ids.emplace(constraint.node_id)
                      .second);
                break;
            case Constraint::Tag::INTER_SUBMAP:
                UpdateTrajectoryConnectivity(constraint);
                break;
            }
            const Constraint::Pose pose = {
                constraint.pose.zbar_ij *
                transform::Rigid3d::Rotation(
                trajectory_nodes_.at(constraint.node_id)
                .constant_data->gravity_alignment.inverse()),
                constraint.pose.translation_weight, constraint.pose.rotation_weight};
            constraints_.push_back(Constraint{
                                       constraint.submap_id, constraint.node_id, pose, constraint.tag});
        }
        LOG(INFO) << "Loaded " << constraints.size() << " constraints.";
    });
}

void PoseGraph2D::AddTrimmer(std::unique_ptr<PoseGraphTrimmer> trimmer) {
    common::MutexLocker locker(&mutex_);
    // C++11 does not allow us to move a unique_ptr into a lambda.
    PoseGraphTrimmer* const trimmer_ptr = trimmer.release();
    AddWorkItem([this, trimmer_ptr]()
                REQUIRES(mutex_) { trimmers_.emplace_back(trimmer_ptr); });
}

void PoseGraph2D::RunFinalOptimization() {
    {
        common::MutexLocker locker(&mutex_);
        AddWorkItem([this]() REQUIRES(mutex_) {
                        optimization_problem_->SetMaxNumIterations(
                        options_.max_num_final_iterations());
                        DispatchOptimization();
                    });
        AddWorkItem([this]() REQUIRES(mutex_) {
                        optimization_problem_->SetMaxNumIterations(
                        options_.optimization_problem_options()
                        .ceres_solver_options()
                        .max_num_iterations());
                    });
    }
    WaitForAllComputations();
}

// 运行全局优化 , 获取优化结果, 
// 并且对新加进来的节点(指在优化过程中新加进来的节点: 这些节点没有得到优化)进行修正
void PoseGraph2D::RunOptimization() {
    if (optimization_problem_->submap_data().empty()) {
        return;
    }

    // No other thread is accessing the optimization_problem_, constraints_,
    // frozen_trajectories_ and landmark_nodes_ when executing the Solve. Solve is
    // time consuming, so not taking the mutex before Solve to avoid blocking
    // foreground processing.
    // 当执行optimization_problem_->Solve时 , 没有其他线程有权对
    // optimization_problem_ , constraints_ , frozen_trajectories_ , landmark_nodes_ 进行操作

    // 求解
    // 残差项有如下:
    //              1. 回环约束产生的节点与子图的相对位姿
    //              2. landmark
    //              3. 同一条轨迹上:里程计数据获取到的节点与节点之间的相对位姿
    //              4. 同一条轨迹上:local SLAM得到的节点local位姿可构成相对位姿
    optimization_problem_->Solve(constraints_, frozen_trajectories_,
                                 landmark_nodes_);
    common::MutexLocker locker(&mutex_);

    // 取优化之后的结果
    const auto& submap_data = optimization_problem_->submap_data();
    const auto& node_data = optimization_problem_->node_data();
    //遍历每条轨迹
    for (const int trajectory_id : node_data.trajectory_ids()) {
        //遍历每条轨迹
        for (const auto& node : node_data.trajectory(trajectory_id)) {
            // 设置每条轨迹每个节点的全局位姿
            auto& mutable_trajectory_node = trajectory_nodes_.at(node.id);
            mutable_trajectory_node.global_pose =
                    transform::Embed3D(node.data.global_pose_2d) *
                    transform::Rigid3d::Rotation(
                        mutable_trajectory_node.constant_data->gravity_alignment);
        }

        // Extrapolate all point cloud poses that were not included in the
        // 'optimization_problem_' yet.
        // 计算新的 从local map坐标系到全局坐标系的变换
        const auto local_to_new_global =
                ComputeLocalToGlobalTransform(submap_data, trajectory_id);
        // 计算旧的 从local map坐标系到全局坐标系的变换
        const auto local_to_old_global =
                ComputeLocalToGlobalTransform(global_submap_poses_, trajectory_id);
        // 得到旧的全局坐标系到新的全局坐标系的变换
        const transform::Rigid3d old_global_to_new_global =
                local_to_new_global * local_to_old_global.inverse();

        /// 下面的部分是优化结果进行传播, 因为优化的时候,同时会有新的节点加入,需要将优化的结果向后传播
        //取最后一个经过优化的节点id
        const NodeId last_optimized_node_id =
                std::prev(node_data.EndOfTrajectory(trajectory_id))->id;
        //从这个最后一个优化过的节点开始, 取下一个节点
        auto node_it = std::next(trajectory_nodes_.find(last_optimized_node_id));
        //从这个最后一个优化过的节点开始,向后遍历节点
        for (; node_it != trajectory_nodes_.EndOfTrajectory(trajectory_id);
             ++node_it) {
            auto& mutable_trajectory_node = trajectory_nodes_.at(node_it->id);
            // 修正这些没有经过优化的节点的全局位姿
            mutable_trajectory_node.global_pose =
                    old_global_to_new_global * mutable_trajectory_node.global_pose;
        }
    }
    // 遍历所有landmark
    for (const auto& landmark : optimization_problem_->landmark_data()) {
        // 更新landmark的全局坐标系坐标
        landmark_nodes_[landmark.first].global_landmark_pose = landmark.second;
    }
    // 记录优化之后的子图全局位姿
    global_submap_poses_ = submap_data;
}

// 获取轨迹节点
MapById<NodeId, TrajectoryNode> PoseGraph2D::GetTrajectoryNodes() const {
    common::MutexLocker locker(&mutex_);
    return trajectory_nodes_;
}

// 返回有数据的节点位姿
MapById<NodeId, TrajectoryNodePose> PoseGraph2D::GetTrajectoryNodePoses()
const {
    MapById<NodeId, TrajectoryNodePose> node_poses;
    common::MutexLocker locker(&mutex_);
    // 遍历所有轨迹及节点
    for (const auto& node_id_data : trajectory_nodes_) {
        //
        common::optional<TrajectoryNodePose::ConstantPoseData> constant_pose_data;
        // 节点是否有数据
        if (node_id_data.data.constant_data != nullptr) {
            //有数据, 则拷贝到临时变量constant_pose_data
            constant_pose_data = TrajectoryNodePose::ConstantPoseData{
                    node_id_data.data.constant_data->time,
                    node_id_data.data.constant_data->local_pose};
        }
        // 插入到准备输出的容器
        node_poses.Insert(
                    node_id_data.id,
                    TrajectoryNodePose{node_id_data.data.global_pose, constant_pose_data});
    }
    // 返回<NodeId, TrajectoryNodePose>类型的节点数据
    return node_poses;
}

// 返回landmark的全局位姿
std::map<std::string, transform::Rigid3d> PoseGraph2D::GetLandmarkPoses()
const {
    std::map<std::string, transform::Rigid3d> landmark_poses;
    common::MutexLocker locker(&mutex_);
    for (const auto& landmark : landmark_nodes_) {
        // Landmark without value has not been optimized yet.
        if (!landmark.second.global_landmark_pose.has_value()) continue;
        landmark_poses[landmark.first] =
                landmark.second.global_landmark_pose.value();
    }
    return landmark_poses;
}

// 设置landmark全局位姿
void PoseGraph2D::SetLandmarkPose(const std::string& landmark_id,
                                  const transform::Rigid3d& global_pose) {
    common::MutexLocker locker(&mutex_);
    AddWorkItem([=]() REQUIRES(mutex_) {
        landmark_nodes_[landmark_id].global_landmark_pose = global_pose;
    });
}

// 取IMU数据
sensor::MapByTime<sensor::ImuData> PoseGraph2D::GetImuData() const {
    common::MutexLocker locker(&mutex_);
    return optimization_problem_->imu_data();
}

// 取里程计数据
sensor::MapByTime<sensor::OdometryData> PoseGraph2D::GetOdometryData() const {
    common::MutexLocker locker(&mutex_);
    return optimization_problem_->odometry_data();
}

// 取所有路标点
std::map<std::string /* landmark ID */, PoseGraphInterface::LandmarkNode>
PoseGraph2D::GetLandmarkNodes() const {
    common::MutexLocker locker(&mutex_);
    return landmark_nodes_;
}

// 空
std::map<int, PoseGraphInterface::TrajectoryData>
PoseGraph2D::GetTrajectoryData() const {
    return {};  // Not implemented yet in 2D.
}

// 空
sensor::MapByTime<sensor::FixedFramePoseData>
PoseGraph2D::GetFixedFramePoseData() const {
    return {};  // Not implemented yet in 2D.
}

// 返回约束
std::vector<PoseGraphInterface::Constraint> PoseGraph2D::constraints() const {
    std::vector<PoseGraphInterface::Constraint> result;
    common::MutexLocker locker(&mutex_);
    for (const Constraint& constraint : constraints_) {
        result.push_back(Constraint{
                             constraint.submap_id, constraint.node_id,
                             Constraint::Pose{constraint.pose.zbar_ij *
                                              transform::Rigid3d::Rotation(
                                              trajectory_nodes_.at(constraint.node_id)
                                              .constant_data->gravity_alignment),
                                              constraint.pose.translation_weight,
                                              constraint.pose.rotation_weight},
                             constraint.tag});
    }
    return result;
}

// 设置???
void PoseGraph2D::SetInitialTrajectoryPose(const int from_trajectory_id,
                                           const int to_trajectory_id,
                                           const transform::Rigid3d& pose,
                                           const common::Time time) {
    common::MutexLocker locker(&mutex_);
    initial_trajectory_poses_[from_trajectory_id] =
            InitialTrajectoryPose{to_trajectory_id, pose, time};
}

// 根据时间戳进行插值,返回给定轨迹和时间戳 经过插值后的全局位姿
transform::Rigid3d PoseGraph2D::GetInterpolatedGlobalTrajectoryPose(
        const int trajectory_id, const common::Time time) const {
    CHECK_GT(trajectory_nodes_.SizeOfTrajectoryOrZero(trajectory_id), 0);
    // MapById<NodeId, TrajectoryNode> trajectory_nodes_ 记录轨迹节点的容器
    // 返回第一个属于这个轨迹id的节点, 并且该节点的时间戳 > 输入的参数`time`
    const auto it = trajectory_nodes_.lower_bound(trajectory_id, time);
    // 如果这个节点是这条轨迹的起点
    if (it == trajectory_nodes_.BeginOfTrajectory(trajectory_id)) {
        // 直接返回节点的全局位姿
        return trajectory_nodes_.BeginOfTrajectory(trajectory_id)->data.global_pose;
    }
    // 如果这个节点是这条轨迹的终点
    if (it == trajectory_nodes_.EndOfTrajectory(trajectory_id)) {
        // 返回节点的全局位姿
        return std::prev(trajectory_nodes_.EndOfTrajectory(trajectory_id))
                ->data.global_pose;
    }
    // 否则: 利用这个节点的前一个节点 和本节点,得到两个全局位姿, 根据时间戳,进行插值
    // 返回插值之后的 全局位姿
    return transform::Interpolate(
                transform::TimestampedTransform{std::prev(it)->data.time(),
                                                std::prev(it)->data.global_pose},
                transform::TimestampedTransform{it->data.time(),
                                                it->data.global_pose},
                time)
            .transform;
}

// 获取当前local map到全局坐标系的变换(利用优化过的子图全局位姿)
transform::Rigid3d PoseGraph2D::GetLocalToGlobalTransform(
        const int trajectory_id) const {
    common::MutexLocker locker(&mutex_);
    // 计算local map到全局坐标系的变换
    return ComputeLocalToGlobalTransform(global_submap_poses_, trajectory_id);
}

// 获取轨迹之间的连接关系
std::vector<std::vector<int>> PoseGraph2D::GetConnectedTrajectories() const {
    return trajectory_connectivity_state_.Components();
}

// 获取指定子图数据{子图 , 子图的全局位姿}
PoseGraphInterface::SubmapData PoseGraph2D::GetSubmapData(
        const SubmapId& submap_id) const {
    common::MutexLocker locker(&mutex_);
    return GetSubmapDataUnderLock(submap_id);
}

// 获取所有子图数据
MapById<SubmapId, PoseGraphInterface::SubmapData>
PoseGraph2D::GetAllSubmapData() const {
    common::MutexLocker locker(&mutex_);
    return GetSubmapDataUnderLock();
}

// 获取所有子图位姿
MapById<SubmapId, PoseGraphInterface::SubmapPose>
PoseGraph2D::GetAllSubmapPoses() const {
    common::MutexLocker locker(&mutex_);
    MapById<SubmapId, SubmapPose> submap_poses;
    for (const auto& submap_id_data : submap_data_) {
        auto submap_data = GetSubmapDataUnderLock(submap_id_data.id);
        submap_poses.Insert(
                    submap_id_data.id,
                    PoseGraph::SubmapPose{submap_data.submap->num_range_data(),
                                          submap_data.pose});
    }
    return submap_poses;
}

// 计算local map到全局坐标系的变换
transform::Rigid3d PoseGraph2D::ComputeLocalToGlobalTransform(
        const MapById<SubmapId, optimization::SubmapSpec2D>& global_submap_poses,   //映射<{轨迹id , 子图索引} , 子图的全局位姿>
        const int trajectory_id) const {                                            //轨迹id

    // 取给定轨迹上的 第一个子图数据{SubmapId, optimization::SubmapSpec2D}
    auto begin_it = global_submap_poses.BeginOfTrajectory(trajectory_id);
    // 取下一条轨迹上的 第一个子图数据{SubmapId, optimization::SubmapSpec2D}
    auto end_it = global_submap_poses.EndOfTrajectory(trajectory_id);
    // 如果等于，说明这个容器里没有一个元素,即还没有子图?
    if (begin_it == end_it) {
        //initial_trajectory_poses_ : 记录着所有轨迹的轨迹初始位姿 (轨迹id , 相对位姿, 时间戳)
        // it : 这个轨迹id的轨迹初始位姿 (轨迹id , 相对位姿, 时间戳)
        const auto it = initial_trajectory_poses_.find(trajectory_id);
        if (it != initial_trajectory_poses_.end()) {
            // 找到了,
            // GetInterpolatedGlobalTrajectoryPose() 根据时间戳进行插值,给定id的轨迹上的给定时间戳上的全局位姿
            // it->second.to_trajectory_id : 与这条轨迹关联的轨迹id
            // it->second.relative_pose : 这条轨迹与关联轨迹的相对位姿
            return GetInterpolatedGlobalTrajectoryPose(it->second.to_trajectory_id,it->second.time) *   // 这条轨迹起点的全局位姿
                    it->second.relative_pose;   //  local map到轨迹初始位姿的变换
        } else {
            // 如果轨迹还没有初始位姿
            // 那么认为 local map 与 全局坐标系重合
            return transform::Rigid3d::Identity();
        }
    }
    // 取轨迹上最后一个优化过的子图
    const SubmapId last_optimized_submap_id = std::prev(end_it)->id;
    // Accessing 'local_pose' in Submap is okay, since the member is const.
    // 返回 local map坐标系到全局坐标系的变换
    return transform::Embed3D(
                //最后一个优化过的子图的全局位姿(子图到全局坐标系的变换)
                global_submap_poses.at(last_optimized_submap_id).global_pose) *
                //最后一个优化过的子图的local map位姿的逆 (local map到该子图坐标系的变换)
                submap_data_.at(last_optimized_submap_id).submap->local_pose().inverse();
}

// 获取指定ID的子图数据{子图 , 子图的全局位姿}
PoseGraphInterface::SubmapData PoseGraph2D::GetSubmapDataUnderLock(
        const SubmapId& submap_id) const {      //子图ID
    // 从submap_data_容器中查找 对应ID的数据记录
    const auto it = submap_data_.find(submap_id);
    if (it == submap_data_.end()) {
        return {};
    }
    // 取记录的2D子图<const Submap2D>
    auto submap = it->data.submap;
    // 如果子图的全局位姿记录器包含 这个id的子图 全局位姿信息
    if (global_submap_poses_.Contains(submap_id)) {
        // We already have an optimized pose.
        // 表示这个子图的位姿是经过了BA优化的
        // 返回{子图 , 子图的全局位姿}
        return {submap,
                    transform::Embed3D(global_submap_poses_.at(submap_id).global_pose)};
    }
    // We have to extrapolate.
    // 否则,表示这个子图没有经过优化 , 其全局位姿需要经过推算
    // 返回 {子图 , 子图的全局位姿(推算出来的)}
    return {submap,
                ComputeLocalToGlobalTransform(global_submap_poses_,submap_id.trajectory_id) *   //local map坐标系到全局坐标系的变换
                submap->local_pose()};      //子图在local map的位姿
}

// 获取所有子图数据
MapById<SubmapId, PoseGraphInterface::SubmapData>
PoseGraph2D::GetSubmapDataUnderLock() const {

    MapById<SubmapId, PoseGraphInterface::SubmapData> submaps;
    // 遍历submap_data_
    for (const auto& submap_id_data : submap_data_) {
        // 取{轨迹id , 子图索引}
        submaps.Insert(submap_id_data.id,
                       GetSubmapDataUnderLock(submap_id_data.id));
    }
    return submaps;
}

PoseGraph2D::TrimmingHandle::TrimmingHandle(PoseGraph2D* const parent)
    : parent_(parent) {}

// 获取子图数量
int PoseGraph2D::TrimmingHandle::num_submaps(const int trajectory_id) const {
    const auto& submap_data = parent_->optimization_problem_->submap_data();
    return submap_data.SizeOfTrajectoryOrZero(trajectory_id);
}

// 返回优化之后的子图数据
MapById<SubmapId, PoseGraphInterface::SubmapData>
PoseGraph2D::TrimmingHandle::GetOptimizedSubmapData() const {
    //这是要返回的结果
    MapById<SubmapId, PoseGraphInterface::SubmapData> submaps;
    // parent_ : 就是PoseGraph2D对象
    // 遍历submap_data_
    for (const auto& submap_id_data : parent_->submap_data_) {
        // 如果子图还没有完成 或者 global_submap_poses_ 不包含这个子图的数据
        if (submap_id_data.data.state != SubmapState::kFinished ||
                !parent_->global_submap_poses_.Contains(submap_id_data.id)) {
            // 则跳过
            continue;
        }
        // 否则
        // 把数据插入到要返回的结果中
        submaps.Insert(submap_id_data.id,
                       SubmapData{submap_id_data.data.submap,
                                  transform::Embed3D(parent_->global_submap_poses_
                                  .at(submap_id_data.id)
                                  .global_pose)});
    }
    //返回结果
    return submaps;
}

// 获取指定轨迹的所有子图ID
std::vector<SubmapId> PoseGraph2D::TrimmingHandle::GetSubmapIds(
        int trajectory_id) const {
    std::vector<SubmapId> submap_ids;
    const auto& submap_data = parent_->optimization_problem_->submap_data();
    for (const auto& it : submap_data.trajectory(trajectory_id)) {
        submap_ids.push_back(it.id);
    }
    return submap_ids;
}

// 返回父对象的trajectory_nodes_ (记录轨迹节点的容器)
const MapById<NodeId, TrajectoryNode>&
PoseGraph2D::TrimmingHandle::GetTrajectoryNodes() const {
    return parent_->trajectory_nodes_;
}

// 返回父对象的constraints_ (记录约束的容器)
const std::vector<PoseGraphInterface::Constraint>&
PoseGraph2D::TrimmingHandle::GetConstraints() const {
    return parent_->constraints_;
}

// 返回 给定轨迹ID的轨迹是否结束
bool PoseGraph2D::TrimmingHandle::IsFinished(const int trajectory_id) const {
    return parent_->IsTrajectoryFinished(trajectory_id);
}

// 标记某个子图需要被trim掉
// 对父对象的constraints_ , submap_data_ , constraint_builder_ , optimization_problem_
// 进行修整 ,去除与这个子图有关的数据 , 以及与这个子图构成约束的节点 的相关数据
void PoseGraph2D::TrimmingHandle::MarkSubmapAsTrimmed(
        const SubmapId& submap_id) {    //输入子图id
    // TODO(hrapp): We have to make sure that the trajectory has been finished
    // if we want to delete the last submaps.
    // 检查子图是否完成状态
    CHECK(parent_->submap_data_.at(submap_id).state == SubmapState::kFinished);

    // Compile all nodes that are still INTRA_SUBMAP constrained once the submap
    // with 'submap_id' is gone.

    std::set<NodeId> nodes_to_retain;   //要保留的节点
    // 遍历父对象的所有约束
    for (const Constraint& constraint : parent_->constraints_) {
        // 如果约束的标签为INTRA_SUBMAP : (表示节点插入到约束对应的子图)
        // 同时 , 约束的子图ID 与 输入的参数 子图id 不一致
        if (constraint.tag == Constraint::Tag::INTRA_SUBMAP &&
                constraint.submap_id != submap_id) {
            // 则保留这个约束对应的节点
            nodes_to_retain.insert(constraint.node_id);
        }
    }
    // Remove all 'constraints_' related to 'submap_id'.
    // 去除所有与输入参数submap_id相关的约束

    std::set<NodeId> nodes_to_remove;   //这是要去除的节点
    {
        std::vector<Constraint> constraints;
        // 遍历约束
        for (const Constraint& constraint : parent_->constraints_) {
            //如果约束中的子图id 与输入参数一样
            if (constraint.submap_id == submap_id) {
                // 如果约束的标签为 INTRA_SUBMAP (表示这个节点插入到了 这个被标记为要去除的子图)
                // 同时上面记录的要保留的节点中不包含这个约束的节点
                if (constraint.tag == Constraint::Tag::INTRA_SUBMAP &&
                        nodes_to_retain.count(constraint.node_id) == 0) {
                    // This node will no longer be INTRA_SUBMAP contrained and has to be
                    // removed.
                    // 这个约束对应的节点需要被去除
                    nodes_to_remove.insert(constraint.node_id);
                }
            } else {
                // 保留不包含这个要去除的子图的约束
                constraints.push_back(constraint);
            }
        }
        // 用保留下来的约束 , 替换掉父对象的约束
        parent_->constraints_ = std::move(constraints);
    }
    // Remove all 'constraints_' related to 'nodes_to_remove'.
    // 再次遍历父对象约束 , 这时候需要把 与上面标记为 'nodes_to_remove' 的相关的约束也去掉
    {
        std::vector<Constraint> constraints;
        // 遍历约束
        for (const Constraint& constraint : parent_->constraints_) {
            // 如果约束的节点不包含在`nodes_to_remove`里面
            if (nodes_to_remove.count(constraint.node_id) == 0) {
                // 保留这个约束
                constraints.push_back(constraint);
            }
        }
        // 再次替换父对象约束
        parent_->constraints_ = std::move(constraints);
    }

    // Mark the submap with 'submap_id' as trimmed and remove its data.
    // 使用子图ID,对这个子图进行标记为 要去除的
    CHECK(parent_->submap_data_.at(submap_id).state == SubmapState::kFinished);
    // 父对象的submap_data_ ,去除 与这个子图ID相关的东西
    parent_->submap_data_.Trim(submap_id);
    // 约束构造器 : 去除这个子图的扫描匹配器
    parent_->constraint_builder_.DeleteScanMatcher(submap_id);
    // 优化问题: 去除这个子图的数据
    parent_->optimization_problem_->TrimSubmap(submap_id);

    // Remove the 'nodes_to_remove' from the pose graph and the optimization
    // problem.
    // 遍历需要去除的节点, 根据节点ID去除
    for (const NodeId& node_id : nodes_to_remove) {
        // 父对象trajectory_nodes_容器 , 去除节点
        parent_->trajectory_nodes_.Trim(node_id);
        // 优化问题: 去除节点
        parent_->optimization_problem_->TrimTrajectoryNode(node_id);
    }
}

// 设置全局BA优化结束之后需要调用的回调函数
void PoseGraph2D::SetGlobalSlamOptimizationCallback(
        PoseGraphInterface::GlobalSlamOptimizationCallback callback) {
    global_slam_optimization_callback_ = callback;
}

}  // namespace mapping
}  // namespace cartographer
