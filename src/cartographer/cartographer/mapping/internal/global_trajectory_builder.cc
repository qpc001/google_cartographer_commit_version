/*
 * Copyright 2018 The Cartographer Authors
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

#include "cartographer/mapping/internal/global_trajectory_builder.h"

#include <memory>

#include "cartographer/common/make_unique.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/local_slam_result_data.h"
#include "cartographer/metrics/family_factory.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace {

static auto* kLocalSlamMatchingResults = metrics::Counter::Null();
static auto* kLocalSlamInsertionResults = metrics::Counter::Null();

/** class GlobalTrajectoryBuilder : public mapping::TrajectoryBuilderInterface
 * @brief 连接前端与后端的桥梁
 * @brief 两个重要的成员 LocalTrajectoryBuilder 和 PoseGraph
 * @brief
 */
template <typename LocalTrajectoryBuilder, typename PoseGraph>
class GlobalTrajectoryBuilder : public mapping::TrajectoryBuilderInterface {
public:
    // Passing a 'nullptr' for 'local_trajectory_builder' is acceptable, but no
    // 'TimedPointCloudData' may be added in that case.

    //构造函数
    GlobalTrajectoryBuilder(
            std::unique_ptr<LocalTrajectoryBuilder> local_trajectory_builder,
            const int trajectory_id,
            PoseGraph* const pose_graph,
            const LocalSlamResultCallback& local_slam_result_callback)
        : //初始化成员变量
          trajectory_id_(trajectory_id),                                      //轨迹ID
          pose_graph_(pose_graph),                                            //位姿图对象
          local_trajectory_builder_(std::move(local_trajectory_builder)),     //local SLAM
          local_slam_result_callback_(local_slam_result_callback) {}          //local SLAM完成后的回调函数
    ~GlobalTrajectoryBuilder() override {}

    GlobalTrajectoryBuilder(const GlobalTrajectoryBuilder&) = delete;
    GlobalTrajectoryBuilder& operator=(const GlobalTrajectoryBuilder&) = delete;

    /**
    * 下面有几个AddSensorData()的重载
    * @brief 主要功能是:
    *        1. 将数据交给 local_trajectory_builder_
    *        2. 将数据交给 pose_graph_
    */

    // 激光点云数据
    void AddSensorData(
            const std::string& sensor_id,                                             //传感器id
            const sensor::TimedPointCloudData& timed_point_cloud_data) override {     //点云数据

        CHECK(local_trajectory_builder_)
                << "Cannot add TimedPointCloudData without a LocalTrajectoryBuilder.";

        /// 以2D为例 : 在这里铺开 AddRangeData()函数的作用
        // 1. 添加激光数据 , 然后将Cartographer系统中原始的激光点云数据转换成占用栅格和插入器需要的RangeData类型的数据
        // 2. 然后调用函数AddAccumulatedRangeData() , 完成scanMatching 和 激光插入子图
        // 3. 最后返回匹配结果结构体 <MatchingResult> (时间戳 , 位姿估计(机器人在local map坐标系的坐标) , 插入到子图的激光数据 , 插入结果)
        std::unique_ptr<typename LocalTrajectoryBuilder::MatchingResult>
                matching_result = local_trajectory_builder_->AddRangeData(
                    sensor_id, timed_point_cloud_data);
        // 检查匹配结果
        if (matching_result == nullptr) {
            // The range data has not been fully accumulated yet.
            return;
        }
        // 这句没什么卵用
        kLocalSlamMatchingResults->Increment();

        std::unique_ptr<InsertionResult> insertion_result;
        // 如果匹配结果不为空
        if (matching_result->insertion_result != nullptr) {
            kLocalSlamInsertionResults->Increment();    // 这句没什么卵用
            // 向pose_graph_添加一个节点
            // AddNode的主要工作是把一个节点的数据加入到PoseGraph维护的trajectory_nodes_这个容器中, 并返回加入的节点的Node
            auto node_id = pose_graph_->AddNode(
                        matching_result->insertion_result->constant_data,       //节点数据 <const TrajectoryNode::Data> constant_data
                        trajectory_id_,                                         //轨迹id
                        matching_result->insertion_result->insertion_submaps);  //子图, active_submaps_的副本
            CHECK_EQ(node_id.trajectory_id, trajectory_id_);
            // 构造一个<InsertionResult>对象
            insertion_result = common::make_unique<InsertionResult>(InsertionResult{
                            node_id, matching_result->insertion_result->constant_data,      //节点数据
                            std::vector<std::shared_ptr<const Submap>>(                     //子图, active_submaps_的副本
                                matching_result->insertion_result->insertion_submaps.begin(),
                                matching_result->insertion_result->insertion_submaps.end())});
        }
        // 如果存在local SLAM 完成的回调函数
        if (local_slam_result_callback_) {
            // 调用回调函数
            local_slam_result_callback_(
                        trajectory_id_,                                 //轨迹id
                        matching_result->time,                          //匹配结果时间戳
                        matching_result->local_pose,                    //匹配结果(节点在local map的位姿)
                        std::move(matching_result->range_data_in_local),//转换到local map坐标系下的激光扫描
                        std::move(insertion_result));                   //上面构造的<InsertionResult>对象
        }
    }

    // IMU数据
    void AddSensorData(const std::string& sensor_id,
                       const sensor::ImuData& imu_data) override {
        if (local_trajectory_builder_) {
            // 把IMU数据交给local_trajectory_builder_
            local_trajectory_builder_->AddImuData(imu_data);
        }
        // 把IMU数据交给pose_graph_
        pose_graph_->AddImuData(trajectory_id_, imu_data);
    }

    // 里程计数据
    void AddSensorData(const std::string& sensor_id,
                       const sensor::OdometryData& odometry_data) override {
        CHECK(odometry_data.pose.IsValid()) << odometry_data.pose;
        if (local_trajectory_builder_) {
            // 把数据交给local_trajectory_builder_
            local_trajectory_builder_->AddOdometryData(odometry_data);
        }
        //把数据交给pose_graph_
        pose_graph_->AddOdometryData(trajectory_id_, odometry_data);
    }

    // gps数据 [2D 情况下没有用,还没实现]
    void AddSensorData(
            const std::string& sensor_id,
            const sensor::FixedFramePoseData& fixed_frame_pose) override {
        if (fixed_frame_pose.pose.has_value()) {
            CHECK(fixed_frame_pose.pose.value().IsValid())
                    << fixed_frame_pose.pose.value();
        }
        //只把数据交给pose_graph_ , 因为local_trajectory_builder_没法对gps数据进行处理
        //2D: pose_graph_->AddFixedFramePoseData (空函数,还没实现)
        pose_graph_->AddFixedFramePoseData(trajectory_id_, fixed_frame_pose);
    }

    // landmark数据
    void AddSensorData(const std::string& sensor_id,
                       const sensor::LandmarkData& landmark_data) override {
        //只把数据交给pose_graph_
        // pose_graph_->AddLandmarkData():
        // 1.遍历landmark数据的landmark_observations
        //  将数据储存到landmark_nodes_ 容器中
        pose_graph_->AddLandmarkData(trajectory_id_, landmark_data);
    }

    // 暂时还没看到有使用这个函数
    void AddLocalSlamResultData(std::unique_ptr<mapping::LocalSlamResultData>
                                local_slam_result_data) override {
        CHECK(!local_trajectory_builder_) << "Can't add LocalSlamResultData with "
                                             "local_trajectory_builder_ present.";
        local_slam_result_data->AddToPoseGraph(trajectory_id_, pose_graph_);
    }

private:
    const int trajectory_id_;
    PoseGraph* const pose_graph_;
    std::unique_ptr<LocalTrajectoryBuilder> local_trajectory_builder_;    //2D : LocalTrajectoryBuilder2D

    //构造函数传进来的local SLAM 完成回调函数
    //回调函数的形参(轨迹ID, 时间戳, scanMatching得到的位姿估计(机器人在子图的坐标) ,
    //            子图坐标系下的激光扫描 ,
    //            InsertionResult{节点ID,
    //                            TrajectoryNode::Data[重力对齐四元数, 重力对齐并且滤波后的点云 , {} {} {} ,
    //                                                 局部SLAM的节点位姿(scanMatching位姿估计: 机器人在子图坐标系的坐标)]
    //                            子图列表}
    //            )
    LocalSlamResultCallback local_slam_result_callback_;
};

}  // namespace

/// 主要看2D
// 创建全局2D建图器
std::unique_ptr<TrajectoryBuilderInterface> CreateGlobalTrajectoryBuilder2D(
        std::unique_ptr<LocalTrajectoryBuilder2D> local_trajectory_builder,         //2D local SLAM 对象
        const int trajectory_id,                                                    //轨迹ID
        mapping::PoseGraph2D* const pose_graph,                                     //位姿图(用于全局BA优化)
        const TrajectoryBuilderInterface::LocalSlamResultCallback&                  //local SLAM结果回调函数
        local_slam_result_callback) {

    // 返回GlobalTrajectoryBuilder<LocalTrajectoryBuilder2D, mapping::PoseGraph2D>>对象
    return common::make_unique<
            GlobalTrajectoryBuilder<LocalTrajectoryBuilder2D, mapping::PoseGraph2D>>(
                 std::move(local_trajectory_builder),
                 trajectory_id,
                 pose_graph,
                 local_slam_result_callback);
}

/////////////////下面的可先不看//////////////////////////////////////////////////
///3D 先不看
// 创建全局3D建图器
std::unique_ptr<TrajectoryBuilderInterface> CreateGlobalTrajectoryBuilder3D(
        std::unique_ptr<LocalTrajectoryBuilder3D> local_trajectory_builder,     //局部建图器
        const int trajectory_id, mapping::PoseGraph3D* const pose_graph,        //轨迹id, 位姿图
        const TrajectoryBuilderInterface::LocalSlamResultCallback&              //局部SLAM结果回调
        local_slam_result_callback) {
    //返回GlobalTrajectoryBuilder<LocalTrajectoryBuilder3D, mapping::PoseGraph3D>>
    return common::make_unique<
            GlobalTrajectoryBuilder<LocalTrajectoryBuilder3D, mapping::PoseGraph3D>>(
                                                                                        std::move(local_trajectory_builder), trajectory_id, pose_graph,
                                                                                        local_slam_result_callback);
}

void GlobalTrajectoryBuilderRegisterMetrics(metrics::FamilyFactory* factory) {
    auto* results = factory->NewCounterFamily(
                "mapping_internal_global_trajectory_builder_local_slam_results",
                "Local SLAM results");
    kLocalSlamMatchingResults = results->Add({{"type", "MatchingResult"}});
    kLocalSlamInsertionResults = results->Add({{"type", "InsertionResult"}});
}

}  // namespace mapping
}  // namespace cartographer
