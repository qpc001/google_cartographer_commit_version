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

#include "cartographer/mapping/internal/constraints/constraint_builder_2d.h"

#include <cmath>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>

#include "Eigen/Eigenvalues"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/math.h"
#include "cartographer/common/thread_pool.h"
#include "cartographer/mapping/proto/scan_matching//ceres_scan_matcher_options_2d.pb.h"
#include "cartographer/mapping/proto/scan_matching//fast_correlative_scan_matcher_options_2d.pb.h"
#include "cartographer/metrics/counter.h"
#include "cartographer/metrics/gauge.h"
#include "cartographer/metrics/histogram.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace constraints {

static auto* kConstraintsSearchedMetric = metrics::Counter::Null();
static auto* kConstraintsFoundMetric = metrics::Counter::Null();
static auto* kGlobalConstraintsSearchedMetric = metrics::Counter::Null();
static auto* kGlobalConstraintsFoundMetric = metrics::Counter::Null();
static auto* kQueueLengthMetric = metrics::Gauge::Null();
static auto* kConstraintScoresMetric = metrics::Histogram::Null();
static auto* kGlobalConstraintScoresMetric = metrics::Histogram::Null();

// 计算子图坐标系原点在 local map坐标系的坐标
transform::Rigid2d ComputeSubmapPose(const Submap2D& submap) {
    return transform::Project2D(submap.local_pose());
}

// 构造函数
ConstraintBuilder2D::ConstraintBuilder2D(
        const constraints::proto::ConstraintBuilderOptions& options,        //配置选项
        common::ThreadPoolInterface* const thread_pool)                     //线程池 (从哪里传进来的?)
    : // 初始化成员对象
      options_(options),
      thread_pool_(thread_pool),
      finish_node_task_(common::make_unique<common::Task>()),
      when_done_task_(common::make_unique<common::Task>()),
      sampler_(options.sampling_ratio()),
      ceres_scan_matcher_(options.ceres_scan_matcher_options()) {}

ConstraintBuilder2D::~ConstraintBuilder2D() {
    common::MutexLocker locker(&mutex_);
    CHECK_EQ(finish_node_task_->GetState(), common::Task::NEW);
    CHECK_EQ(when_done_task_->GetState(), common::Task::NEW);
    CHECK_EQ(constraints_.size(), 0) << "WhenDone() was not called";
    CHECK_EQ(num_started_nodes_, num_finished_nodes_);
    CHECK(when_done_ == nullptr);
}

// 给定子图和节点,以及初始化相对位姿{T_{sub<--node}} , 尝试在给定的pair{子图,节点}之间建立约束
void ConstraintBuilder2D::MaybeAddConstraint(
        const SubmapId& submap_id,                          //子图id
        const Submap2D* const submap,                       //子图
        const NodeId& node_id,                              //节点id
        const TrajectoryNode::Data* const constant_data,    //节点数据
        const transform::Rigid2d& initial_relative_pose) {  //初始化相对位姿
    // 检查给定的初始相对位姿的平移量是否大于配置中的 最大约束距离
    if (initial_relative_pose.translation().norm() >
            options_.max_constraint_distance()) {
        //大于,则返回
        return;
    }
    // 检查是否该采样了
    if (!sampler_.Pulse()) return;  //还没到采样的时候

    common::MutexLocker locker(&mutex_);
    // 标志位, 表示 WhenDone()函数正在运行
    if (when_done_) {
        //输出日志
        LOG(WARNING)
                << "MaybeAddConstraint was called while WhenDone was scheduled.";
    }
    // 推入一个空元素
    constraints_.emplace_back();
    kQueueLengthMetric->Set(constraints_.size());
    // 取指针
    auto* const constraint = &constraints_.back();
    // 取(或创建)该子图的匹配器, 返回该子图匹配器的指针
    const auto* scan_matcher =
            DispatchScanMatcherConstruction(submap_id, submap->grid());
    // 创建一个任务
    auto constraint_task = common::make_unique<common::Task>();
    // 设置任务内容 :
    constraint_task->SetWorkItem([=]() EXCLUDES(mutex_) {
        // 任务内容为 计算子图和节点之间的约束
        ComputeConstraint(submap_id, submap, node_id, false, /* match_full_submap */
                          constant_data, initial_relative_pose, *scan_matcher,
                          constraint);
    });
    // 添加独立性????
    constraint_task->AddDependency(scan_matcher->creation_task_handle);
    // 把这个任务加到线程池 , 返回constraint_task_handle
    auto constraint_task_handle =
            thread_pool_->Schedule(std::move(constraint_task));
    // ???
    finish_node_task_->AddDependency(constraint_task_handle);
}

// 给定子图和节点 (这里与上面的唯一不同是, 这里没有给定初始相对位姿) , 尝试在给定的pair{整个子图,节点}之间建立约束
void ConstraintBuilder2D::MaybeAddGlobalConstraint(
        const SubmapId& submap_id,                          //子图id
        const Submap2D* const submap,                       //子图
        const NodeId& node_id,                              //节点ID
        const TrajectoryNode::Data* const constant_data) {  //节点数据
    common::MutexLocker locker(&mutex_);
    // 标志位, 检查 WhenDone()函数是否正在运行
    if (when_done_) {
        LOG(WARNING)
                << "MaybeAddGlobalConstraint was called while WhenDone was scheduled.";
    }
    // 在约束容器添加一个空元素
    constraints_.emplace_back();
    kQueueLengthMetric->Set(constraints_.size());
    // 取约束容器新的最后一个空元素
    auto* const constraint = &constraints_.back();
    // 取(或创建)该子图的匹配器, 返回该子图匹配器的指针
    const auto* scan_matcher =
            DispatchScanMatcherConstruction(submap_id, submap->grid());
    // 创建一个任务
    auto constraint_task = common::make_unique<common::Task>();
    // 设置任务内容 :
    constraint_task->SetWorkItem([=]() EXCLUDES(mutex_) {
        // 任务内容为 计算子图和节点之间的约束
        ComputeConstraint(submap_id, submap, node_id, true, /* match_full_submap */
                          constant_data, transform::Rigid2d::Identity(), //注意: 这里给定了初始位姿为I, 即没有初始位姿
                          *scan_matcher, constraint);
    });
    // 添加独立性????
    constraint_task->AddDependency(scan_matcher->creation_task_handle);
    // 把这个任务加到线程池 , 返回constraint_task_handle
    auto constraint_task_handle =
            thread_pool_->Schedule(std::move(constraint_task));
    finish_node_task_->AddDependency(constraint_task_handle);
}

// 在PoseGraph2D::ComputeConstraintsForNode()函数中被调用
// 当一个节点进行了与所有子图尝试建立约束之后, 这个函数需要被调用, 用来记录这个节点已经完成过约束的生成
void ConstraintBuilder2D::NotifyEndOfNode() {
    common::MutexLocker locker(&mutex_);
    CHECK(finish_node_task_ != nullptr);
    // 记录完成约束建立的节点数
    finish_node_task_->SetWorkItem([this] {
        common::MutexLocker locker(&mutex_);
        ++num_finished_nodes_;
    });
    auto finish_node_task_handle =
            thread_pool_->Schedule(std::move(finish_node_task_));
    finish_node_task_ = common::make_unique<common::Task>();
    //
    when_done_task_->AddDependency(finish_node_task_handle);
    ++num_started_nodes_;
}

// 在PoseGraph2D::DispatchOptimization()被调用
// 回调函数内容是 : PoseGraph2D::HandleWorkQueue
void ConstraintBuilder2D::WhenDone(
        const std::function<void(const ConstraintBuilder2D::Result&)>& callback) {
    common::MutexLocker locker(&mutex_);
    CHECK(when_done_ == nullptr);
    // TODO(gaschler): Consider using just std::function, it can also be empty.
    // 把这个回调函数保存下来
    when_done_ =
            common::make_unique<std::function<void(const Result&)>>(callback);
    CHECK(when_done_task_ != nullptr);
    // 将回调作为任务
    // //因为RunWhenDoneCallback()函数是ConstraintBuilder2D类的成员函数, 而发起者却是PoseGraph2D类对象,所以要加 this
    when_done_task_->SetWorkItem([this] { RunWhenDoneCallback(); });
    // 添加到线程池工作队列
    thread_pool_->Schedule(std::move(when_done_task_));
    when_done_task_ = common::make_unique<common::Task>();
}

// 1. 检查这个子图是否有子图匹配器,有,则返回
// 2. 没有, 则创建一个, 然后把`创建一个 fast_correlative_scan_matcher的任务`添加到Task里面
//    把这个创建任务交给线程池来处理(可能是要生成多分辨率网格地图)
// 3. 返回这个子图匹配器
const ConstraintBuilder2D::SubmapScanMatcher*
ConstraintBuilder2D::DispatchScanMatcherConstruction(const SubmapId& submap_id,     //子图ID
                                                     const Grid2D* const grid) {    //概率地图
    // 检查这个子图是否已经有匹配器
    // <SubmapId, SubmapScanMatcher> submap_scan_matchers_
    if (submap_scan_matchers_.count(submap_id) != 0) {
        // 有, 则直接返回对应的子图匹配器
        return &submap_scan_matchers_.at(submap_id);
    }
    // 没有,则创建一个
    auto& submap_scan_matcher = submap_scan_matchers_[submap_id];
    // 设置子图匹配器的网格地图
    submap_scan_matcher.grid = grid;
    // 配置选项
    auto& scan_matcher_options = options_.fast_correlative_scan_matcher_options();
    // 创建一个Task
    auto scan_matcher_task = common::make_unique<common::Task>();
    // 将`创建一个 fast_correlative_scan_matcher的任务`添加到Task里面
    // 准备把这个创建任务交给线程池来处理(可能是要生成多分辨率网格地图)
    scan_matcher_task->SetWorkItem(
                // lambda函数表达式
                // 参数(上面创建的子图匹配器, 配置选项)
                [&submap_scan_matcher, &scan_matcher_options]() {
        // 为子图匹配器创建一个 `fast_correlative_scan_matcher`
        submap_scan_matcher.fast_correlative_scan_matcher =
                // 使用 <概率地图, 配置选项> 来构造
                common::make_unique<scan_matching::FastCorrelativeScanMatcher2D>(
                    *submap_scan_matcher.grid, scan_matcher_options);
    });
    // 将Task添加到线程池
    // 返回task句柄, 并保存到子图匹配器的`creation_task_handle`成员变量
    submap_scan_matcher.creation_task_handle =
            thread_pool_->Schedule(std::move(scan_matcher_task));
    // 返回这个子图匹配器
    return &submap_scan_matchers_.at(submap_id);
}

// 三步计算位姿估计:
// 1. 使用'分枝定界'快速估计
// 2. 如果分数太低,则丢弃
// 3. 如果分数OK, 则继续优化位姿 (使用ceres优化)
// 得到节点在local map的位姿之后, 计算出节点与子图的相对位姿T_{sub<--node}
// 最后生成约束
void ConstraintBuilder2D::ComputeConstraint(
        const SubmapId& submap_id,                                          //子图ID
        const Submap2D* const submap,                                       //子图
        const NodeId& node_id,                                              //节点ID
        bool match_full_submap,                                             //标志位(是否全子图匹配)
        const TrajectoryNode::Data* const constant_data,                    //节点数据
        const transform::Rigid2d& initial_relative_pose,                    //初始的相对位姿
        const SubmapScanMatcher& submap_scan_matcher,                       //子图匹配器
        std::unique_ptr<ConstraintBuilder2D::Constraint>* constraint) {     //输出: 生成的约束
    //ComputeSubmapPose(*submap) : 子图坐标系原点在 local map坐标系的坐标
    //initial_relative_pose : 节点node与子图的相对位姿, 即 节点坐标系到子图的坐标系变换
    // initial_pose : 得到节点node到local map坐标系的变换
    const transform::Rigid2d initial_pose =
            ComputeSubmapPose(*submap) * initial_relative_pose;

    // The 'constraint_transform' (submap i <- node j) is computed from:
    // - a 'filtered_gravity_aligned_point_cloud' in node j,
    // - the initial guess 'initial_pose' for (map <- node j),
    // - the result 'pose_estimate' of Match() (map <- node j).
    // - the ComputeSubmapPose() (map <- submap i)

    float score = 0.;   //匹配得分
    transform::Rigid2d pose_estimate = transform::Rigid2d::Identity();

    // Compute 'pose_estimate' in three stages:
    // 1. Fast estimate using the fast correlative scan matcher.
    // 2. Prune if the score is too low.
    // 3. Refine.
    // 三步计算位姿估计:
    // 1. 使用'分枝定界'快速估计
    // 2. 如果分数太低,则丢弃
    // 3. 如果分数OK, 则继续优化位姿 (使用ceres优化)

    // 标志位: 是否全子图匹配
    if (match_full_submap) {
        // 是 , 表明这次调用是由 MaybeAddGlobalConstraint() 发起的 , 没有给定初始位姿
        kGlobalConstraintsSearchedMetric->Increment();
        // 调用 子图匹配器.fast_correlative_scan_matcher.MatchFullSubmap()进行全子图匹配
        if (submap_scan_matcher.fast_correlative_scan_matcher->MatchFullSubmap(
                    constant_data->filtered_gravity_aligned_point_cloud,    //滤波后的重力对齐的点云(该节点的数据)
                    options_.global_localization_min_score(),               //全局定位的最低分数
                    &score,                                                 //输出:得分
                    &pose_estimate)) {                                      //输出:位姿估计(节点在该local map的位姿)
            // 如果调用成功, 则记录下分数
            CHECK_GT(score, options_.global_localization_min_score());
            CHECK_GE(node_id.trajectory_id, 0);
            CHECK_GE(submap_id.trajectory_id, 0);
            kGlobalConstraintsFoundMetric->Increment();
            kGlobalConstraintScoresMetric->Observe(score);
        } else {
            return;
        }
    } else {
        // 不是进行全子图匹配 , 则这次调用是由 MaybeAddConstraint() 发起的, 具有初始位姿
        kConstraintsSearchedMetric->Increment();
        // 调用 子图匹配器.fast_correlative_scan_matcher->Match() 进行匹配
        if (submap_scan_matcher.fast_correlative_scan_matcher->Match(
                    initial_pose,                                           //输入初始位姿
                    constant_data->filtered_gravity_aligned_point_cloud,    //经过滤波的重力对齐点云(该节点的数据)
                    options_.min_score(),                                   //配置选项的最低分数阈值
                    &score,                                                 //输出:得分
                    &pose_estimate)) {                                      //输出:位姿估计(节点在该local map的位姿)
            // We've reported a successful local match.
            // 如果成功匹配, 输出日志
            CHECK_GT(score, options_.min_score());
            kConstraintsFoundMetric->Increment();
            kConstraintScoresMetric->Observe(score);
        } else {
            return;
        }
    }
    // 到这里,表明匹配成功
    {
        // 记录分数到 分数直方图
        common::MutexLocker locker(&mutex_);
        score_histogram_.Add(score);
    }

    // Use the CSM estimate as both the initial and previous pose. This has the
    // effect that, in the absence of better information, we prefer the original
    // CSM estimate.

    // 使用ceres进一步调优
    ceres::Solver::Summary unused_summary;
    ceres_scan_matcher_.Match(pose_estimate.translation(),                          //上面使用分枝定界搜索得到的位姿的平移量
                              pose_estimate,                                        //上面分枝定界搜索的粗位姿
                              constant_data->filtered_gravity_aligned_point_cloud,  //经过滤波和重力对齐的点云(节点附带的数据)
                              *submap_scan_matcher.grid,                            //子图的概率地图
                              &pose_estimate,                                       //输出:位姿估计(节点在该local map的位姿)
                              &unused_summary);

    // pose_estimate: 上面经过 分枝定界和非线性优化之后得到的节点位姿(local map坐标系下)
    // ComputeSubmapPose(*submap).inverse() : local map坐标系到子图坐标系的变换
    // 所以这里得到了 : 节点到子图坐标系的变换 T_{sub<--node}=T_{i<--j}
    const transform::Rigid2d constraint_transform =
            ComputeSubmapPose(*submap).inverse() * pose_estimate;

    // 设置要输出的约束
    constraint->reset(new Constraint{submap_id,                                     //子图ID
                                     node_id,                                       //节点ID
                                     {transform::Embed3D(constraint_transform),     //约束: 节点到子图坐标系的变换 T_{sub<--node}=T_{i<--j}
                                      options_.loop_closure_translation_weight(),   //权重
                                      options_.loop_closure_rotation_weight()},
                                     Constraint::INTER_SUBMAP});                    //约束类型

    // 下面是日志的输出
    if (options_.log_matches()) {
        std::ostringstream info;
        info << "Node " << node_id << " with "
             << constant_data->filtered_gravity_aligned_point_cloud.size()
             << " points on submap " << submap_id << std::fixed;
        if (match_full_submap) {
            info << " matches";
        } else {
            const transform::Rigid2d difference =
                    initial_pose.inverse() * pose_estimate;
            info << " differs by translation " << std::setprecision(2)
                 << difference.translation().norm() << " rotation "
                 << std::setprecision(3) << std::abs(difference.normalized_angle());
        }
        info << " with score " << std::setprecision(1) << 100. * score << "%.";
        LOG(INFO) << info.str();
    }
}

void ConstraintBuilder2D::RunWhenDoneCallback() {

    Result result;
    // 回调函数
    std::unique_ptr<std::function<void(const Result&)>> callback;
    {
        common::MutexLocker locker(&mutex_);
        CHECK(when_done_ != nullptr);
        // 遍历约束容器
        for (const std::unique_ptr<Constraint>& constraint : constraints_) {
            // 某个元素为空, 跳过
            if (constraint == nullptr) continue;
            // 将非空的约束push到result
            result.push_back(*constraint);
        }
        // 日志输出
        if (options_.log_matches()) {
            LOG(INFO) << constraints_.size() << " computations resulted in "
                      << result.size() << " additional constraints.";
            LOG(INFO) << "Score histogram:\n" << score_histogram_.ToString(10);
        }
        // 清空约束容器 ???
        constraints_.clear();
        // 将when_done_的回调函数指针, 传给callback
        callback = std::move(when_done_);
        when_done_.reset();
        kQueueLengthMetric->Set(constraints_.size());
    }
    // 调用when_done_传过来的回调函数 (将约束作为参数传进这个回调里面)
    (*callback)(result);
}

// 获取完成节点数?
int ConstraintBuilder2D::GetNumFinishedNodes() {
    common::MutexLocker locker(&mutex_);
    return num_finished_nodes_;
}

// 删除某个子图匹配器
void ConstraintBuilder2D::DeleteScanMatcher(const SubmapId& submap_id) {
    common::MutexLocker locker(&mutex_);
    if (when_done_) {
        LOG(WARNING)
                << "DeleteScanMatcher was called while WhenDone was scheduled.";
    }
    submap_scan_matchers_.erase(submap_id);
}

// 配准评价(暂时不知道干嘛的)
void ConstraintBuilder2D::RegisterMetrics(metrics::FamilyFactory* factory) {
    auto* counts = factory->NewCounterFamily(
                "mapping_internal_constraints_constraint_builder_2d_constraints",
                "Constraints computed");
    kConstraintsSearchedMetric =
            counts->Add({{"search_region", "local"}, {"matcher", "searched"}});
    kConstraintsFoundMetric =
            counts->Add({{"search_region", "local"}, {"matcher", "found"}});
    kGlobalConstraintsSearchedMetric =
            counts->Add({{"search_region", "global"}, {"matcher", "searched"}});
    kGlobalConstraintsFoundMetric =
            counts->Add({{"search_region", "global"}, {"matcher", "found"}});
    auto* queue_length = factory->NewGaugeFamily(
                "mapping_internal_constraints_constraint_builder_2d_queue_length",
                "Queue length");
    kQueueLengthMetric = queue_length->Add({});
    auto boundaries = metrics::Histogram::FixedWidth(0.05, 20);
    auto* scores = factory->NewHistogramFamily(
                "mapping_internal_constraints_constraint_builder_2d_scores",
                "Constraint scores built", boundaries);
    kConstraintScoresMetric = scores->Add({{"search_region", "local"}});
    kGlobalConstraintScoresMetric = scores->Add({{"search_region", "global"}});
}

}  // namespace constraints
}  // namespace mapping
}  // namespace cartographer
