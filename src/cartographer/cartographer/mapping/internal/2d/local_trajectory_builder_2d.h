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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_2D_LOCAL_TRAJECTORY_BUILDER_2D_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_2D_LOCAL_TRAJECTORY_BUILDER_2D_H_

#include <chrono>
#include <memory>

#include "cartographer/common/time.h"
#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/ceres_scan_matcher_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/real_time_correlative_scan_matcher_2d.h"
#include "cartographer/mapping/internal/motion_filter.h"
#include "cartographer/mapping/internal/range_data_collator.h"
#include "cartographer/mapping/pose_extrapolator.h"
#include "cartographer/mapping/proto/2d/local_trajectory_builder_options_2d.pb.h"
#include "cartographer/metrics/family_factory.h"
#include "cartographer/sensor/imu_data.h"
#include "cartographer/sensor/internal/voxel_filter.h"
#include "cartographer/sensor/odometry_data.h"
#include "cartographer/sensor/range_data.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace mapping {

// Wires up the local SLAM stack (i.e. pose extrapolator, scan matching, etc.)
// without loop closure.
// local SLAM的操作都在这里 (但是没有回环)
// 1. 航迹推算
// 2. 扫描匹配
// 3. 激光插入子图
class LocalTrajectoryBuilder2D {
public:
    // 子图插入结果
    // InsertIntoSubmap()函数返回:
    // 插入结果<InsertionResult> (轨迹节点数据[时间戳,重力方向向量,没有重力对齐的滤波点云,{} {} {},
    //                                  scanMatching位姿估计] , active_submaps_的副本  )
    struct InsertionResult {
        std::shared_ptr<const TrajectoryNode::Data> constant_data;      //轨迹节点数据(点云数据,节点位姿[节点相对于轨迹的位姿,即 节点坐标系到local map坐标系的变换])
        std::vector<std::shared_ptr<const Submap2D>> insertion_submaps; //active_submaps_的副本
    };
    // 匹配结果
    struct MatchingResult {
        common::Time time;                                  // 时间戳
        transform::Rigid3d local_pose;                      // 匹配的位姿估计(机器人在local map的位姿)
        sensor::RangeData range_data_in_local;              // 转换到local map坐标系下的激光点云
        // 'nullptr' if dropped by the motion filter.
        std::unique_ptr<const InsertionResult> insertion_result;    //插入结果
    };

    // 构造函数
    explicit LocalTrajectoryBuilder2D(
            const proto::LocalTrajectoryBuilderOptions2D& options,
            const std::vector<std::string>& expected_range_sensor_ids);
    ~LocalTrajectoryBuilder2D();

    LocalTrajectoryBuilder2D(const LocalTrajectoryBuilder2D&) = delete;
    LocalTrajectoryBuilder2D& operator=(const LocalTrajectoryBuilder2D&) = delete;

    // Returns 'MatchingResult' when range data accumulation completed,
    // otherwise 'nullptr'. Range data must be approximately horizontal
    // for 2D SLAM. `TimedPointCloudData::time` is when the last point in
    // `range_data` was acquired, `TimedPointCloudData::ranges` contains the
    // relative time of point with respect to `TimedPointCloudData::time`.

    /// 这是这个类的主要方法 , 其中会调用其他函数
    // 添加激光数据 , 然后将Cartographer系统中原始的激光点云数据转换成占用栅格和插入器需要的RangeData类型的数据
    // 然后调用函数AddAccumulatedRangeData() , 完成scanMatching 和 激光插入子图
    // 最后返回匹配结果结构体 <MatchingResult> (时间戳 , 位姿估计 , 插入到子图的激光数据 , 插入结果)
    std::unique_ptr<MatchingResult> AddRangeData(
            const std::string& sensor_id,
            const sensor::TimedPointCloudData& range_data);

    // 添加IMU数据
    // 1. 利用IMU数据的时间戳来初始化姿态外推器
    // 2. 调用姿态外推器extrapolator_->AddImuData()
    void AddImuData(const sensor::ImuData& imu_data);
    // 添加里程计数据
    // 1. 如果姿态外推器还没有被初始化,则返回
    // 2. 否则调用extrapolator_->AddOdometryData() 添加里程计数据
    void AddOdometryData(const sensor::OdometryData& odometry_data);
    // 配准评价
    static void RegisterMetrics(metrics::FamilyFactory* family_factory);

private:
    ///////////////////////////////////////////////////////////////////////////
    ///下面4个函数都被 AddRangeData()来调用
    // 添加累积的激光数据
    std::unique_ptr<MatchingResult> AddAccumulatedRangeData(
            common::Time time, const sensor::RangeData& gravity_aligned_range_data,
            const transform::Rigid3d& gravity_alignment);
    // 对激光数据进行对齐重力坐标系以及进行过滤
    sensor::RangeData TransformToGravityAlignedFrameAndFilter(
            const transform::Rigid3f& transform_to_gravity_aligned_frame,
            const sensor::RangeData& range_data) const;

    // 激光数据插入到子图
    std::unique_ptr<InsertionResult> InsertIntoSubmap(
            common::Time time, const sensor::RangeData& range_data_in_local,
            const sensor::RangeData& gravity_aligned_range_data,
            const transform::Rigid3d& pose_estimate,
            const Eigen::Quaterniond& gravity_alignment);

    // Scan matches 'gravity_aligned_range_data' and returns the observed pose,
    // or nullptr on failure.
    // 进行ScanMatch 如果成功,则返回 2D的变换
    std::unique_ptr<transform::Rigid2d> ScanMatch(
            common::Time time, const transform::Rigid2d& pose_prediction,
            const sensor::RangeData& gravity_aligned_range_data);
    ///////////////////////////////////////////////////////////////////////////

    // Lazily constructs a PoseExtrapolator.
    // 延迟构造一个位姿外推器
    void InitializeExtrapolator(common::Time time);

    // LocalTrajectoryBuilder的配置选项
    const proto::LocalTrajectoryBuilderOptions2D options_;

    // 维护的子图 (两张) <旧的用于扫描匹配 , 新的用于激光数据插入>
    ActiveSubmaps2D active_submaps_;

    // 运动滤波器
    MotionFilter motion_filter_;
    // 2D ScanMatch, 相关性匹配器 [算法"Real-Time Correlative Scan Matching"的实现]
    scan_matching::RealTimeCorrelativeScanMatcher2D
    real_time_correlative_scan_matcher_;
    // ceres的ScanMatcher ,位姿优化器
    scan_matching::CeresScanMatcher2D ceres_scan_matcher_;

    // 姿态外推器
    std::unique_ptr<PoseExtrapolator> extrapolator_;

    int num_accumulated_ = 0;
    // 累积的激光
    sensor::RangeData accumulated_range_data_;
    // 开始累积数据的时间，也是开始跟踪轨迹的时间
    std::chrono::steady_clock::time_point accumulation_started_;
    // 激光数据收集器
    RangeDataCollator range_data_collator_;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_2D_LOCAL_TRAJECTORY_BUILDER_2D_H_
