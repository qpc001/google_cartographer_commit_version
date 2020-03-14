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

#include "cartographer/mapping/internal/2d/local_trajectory_builder_2d.h"

#include <limits>
#include <memory>

#include "cartographer/common/make_unique.h"
#include "cartographer/metrics/family_factory.h"
#include "cartographer/sensor/range_data.h"

namespace cartographer {
namespace mapping {

static auto* kLocalSlamLatencyMetric = metrics::Gauge::Null();
static auto* kFastCorrelativeScanMatcherScoreMetric =
        metrics::Histogram::Null();
static auto* kCeresScanMatcherCostMetric = metrics::Histogram::Null();
static auto* kScanMatcherResidualDistanceMetric = metrics::Histogram::Null();
static auto* kScanMatcherResidualAngleMetric = metrics::Histogram::Null();

// 构造函数 : 初始化了一堆成员变量 ,但是唯独姿态外推器没有初始化
LocalTrajectoryBuilder2D::LocalTrajectoryBuilder2D(
        const proto::LocalTrajectoryBuilderOptions2D& options,      //配置选项
        const std::vector<std::string>& expected_range_sensor_ids)  //激光传感器id
    : options_(options),                                            // 初始化
      active_submaps_(options.submaps_options()),                   // 初始化
      motion_filter_(options_.motion_filter_options()),             // 初始化
      real_time_correlative_scan_matcher_(                          // 初始化
          options_.real_time_correlative_scan_matcher_options()),   // 初始化
      ceres_scan_matcher_(options_.ceres_scan_matcher_options()),   // 初始化
      range_data_collator_(expected_range_sensor_ids) {}            // 初始化

LocalTrajectoryBuilder2D::~LocalTrajectoryBuilder2D() {}

// 将激光数据转换到重力对齐的坐标系,并且进行滤波
sensor::RangeData
LocalTrajectoryBuilder2D::TransformToGravityAlignedFrameAndFilter(
        const transform::Rigid3f& transform_to_gravity_aligned_frame,       //转换到重力对齐坐标系的变换
        const sensor::RangeData& range_data) const {                        //激光数据
    // 1. 根据3D变换(转换到重力对齐坐标系的变换)，对点云进行变换
    // 2. 根据给定z轴范围(配置选项获取)，裁剪点云 , 返回裁剪之后的点云
    const sensor::RangeData cropped =
            sensor::CropRangeData(sensor::TransformRangeData(
                                      range_data, transform_to_gravity_aligned_frame),
                                  options_.min_z(), options_.max_z());
    // 3. 对点云使用体素滤波器 , 返回滤波之后的点云
    return sensor::RangeData{
        cropped.origin,
                sensor::VoxelFilter(options_.voxel_filter_size()).Filter(cropped.returns),
                sensor::VoxelFilter(options_.voxel_filter_size()).Filter(cropped.misses)};
}

//
std::unique_ptr<transform::Rigid2d> LocalTrajectoryBuilder2D::ScanMatch(
        const common::Time time,                                //时间戳
        const transform::Rigid2d& pose_prediction,              //初始位姿(粗位姿)
        const sensor::RangeData& gravity_aligned_range_data) {  //重力对齐的激光数据
    // 取active_submaps_维护的两张子图中的旧子图 作为匹配用
    std::shared_ptr<const Submap2D> matching_submap =
            active_submaps_.submaps().front();
    // The online correlative scan matcher will refine the initial estimate for
    // the Ceres scan matcher.
    // 设置初始位姿
    transform::Rigid2d initial_ceres_pose = pose_prediction;
    // 创建自适应体素滤波器
    sensor::AdaptiveVoxelFilter adaptive_voxel_filter(
                options_.adaptive_voxel_filter_options());
    // 对重力对齐的激光数据再次使用自适应体素滤波器
    const sensor::PointCloud filtered_gravity_aligned_point_cloud =
            adaptive_voxel_filter.Filter(gravity_aligned_range_data.returns);
    if (filtered_gravity_aligned_point_cloud.empty()) {
        return nullptr;
    }
    ///////////////////相关性匹配/////////////////////////////////
    // 检查配置选项, 是否使用相关性匹配 进一步优化初始值
    if (options_.use_online_correlative_scan_matching()) {
        CHECK_EQ(options_.submaps_options().grid_options_2d().grid_type(),
                 proto::GridOptions2D_GridType_PROBABILITY_GRID);
        // 是, 则调用real_time_correlative_scan_matcher_.Match()
        double score = real_time_correlative_scan_matcher_.Match(
                    pose_prediction, filtered_gravity_aligned_point_cloud,
                    *static_cast<const ProbabilityGrid*>(matching_submap->grid()),
                    &initial_ceres_pose);
        // 记录分数
        kFastCorrelativeScanMatcherScoreMetric->Observe(score);
    }

    // 这是要输出的结果
    auto pose_observation = common::make_unique<transform::Rigid2d>();

    // 准备ceres优化器
    ceres::Solver::Summary summary;
    ceres_scan_matcher_.Match(pose_prediction.translation(), initial_ceres_pose,
                              filtered_gravity_aligned_point_cloud,
                              *matching_submap->grid(), pose_observation.get(),
                              &summary);
    //
    if (pose_observation) {
        // 储存优化信息和结果
        kCeresScanMatcherCostMetric->Observe(summary.final_cost);
        double residual_distance =
                (pose_observation->translation() - pose_prediction.translation())
                .norm();
        kScanMatcherResidualDistanceMetric->Observe(residual_distance);
        double residual_angle = std::abs(pose_observation->rotation().angle() -
                                         pose_prediction.rotation().angle());
        kScanMatcherResidualAngleMetric->Observe(residual_angle);
    }
    // 返回scanMatching的结果
    return pose_observation;
}

/// 这是这个类的主要方法 , 其中会调用其他函数
// 将Cartographer系统中原始的激光点云数据转换成占用栅格和插入器需要的RangeData类型的数据
// 然后调用函数AddAccumulatedRangeData() , 完成scanMatching 和 激光插入子图
// 最后返回匹配结果结构体 <MatchingResult> (时间戳 , 位姿估计 , 插入到子图的激光数据 , 插入结果)
std::unique_ptr<LocalTrajectoryBuilder2D::MatchingResult>
LocalTrajectoryBuilder2D::AddRangeData(
        const std::string& sensor_id,                                   //传感器ID
        const sensor::TimedPointCloudData& unsynchronized_data) {       //带时间戳的点云

    // 调用激光数据同步器 range_data_collator_ 获取同步的点云数据
    auto synchronized_data =
            range_data_collator_.AddRangeData(sensor_id, unsynchronized_data);
    if (synchronized_data.ranges.empty()) {
        LOG(INFO) << "Range data collator filling buffer.";
        return nullptr;
    }

    // 获取同步的时间戳
    const common::Time& time = synchronized_data.time;
    // Initialize extrapolator now if we do not ever use an IMU.
    // 如果没有IMU或者IMU没有初始化姿态外推器, 在这里初始化姿态外推器
    if (!options_.use_imu_data()) {
        InitializeExtrapolator(time);
    }

    if (extrapolator_ == nullptr) {
        // Until we've initialized the extrapolator with our first IMU message, we
        // cannot compute the orientation of the rangefinder.
        LOG(INFO) << "Extrapolator not yet initialized.";
        return nullptr;
    }

    CHECK(!synchronized_data.ranges.empty());
    // TODO(gaschler): Check if this can strictly be 0.
    // 检查扫描数据的时间确保最后一个数据的时间偏移量大于等于0
    CHECK_LE(synchronized_data.ranges.back().point_time[3], 0.f);

    // 获取第一个测量点的时间戳(绝对时间戳)
    const common::Time time_first_point =
            time +
            common::FromSeconds(synchronized_data.ranges.front().point_time[3]);
    // 与姿态外推器的第一个pose的时间戳进行比较
    if (time_first_point < extrapolator_->GetLastPoseTime()) {
        // 如果姿态外推器的上一次添加pose的时间 > 激光数据的第一个测量点时间戳
        // 则表明姿态外推器还在初始化
        LOG(INFO) << "Extrapolator is still initializing.";
        return nullptr;
    }
    // 如果历史没有累积数据，就用成员变量accumulation_started_记录下当前的时间，作为跟踪轨迹的开始时刻
    if (num_accumulated_ == 0) {
        accumulation_started_ = std::chrono::steady_clock::now();
    }

    // 创建临时的容器range_data_poses : 用于记录各个扫描点所对应的机器人位姿
    std::vector<transform::Rigid3f> range_data_poses;
    range_data_poses.reserve(synchronized_data.ranges.size());
    bool warned = false;
    // 遍历同步的激光数据
    for (const auto& range : synchronized_data.ranges) {
        // 取激光数据点的绝对时间戳
        common::Time time_point = time + common::FromSeconds(range.point_time[3]);
        // 如果姿态外推器的上一次 '推算' 的时间 > 激光数据的第一个测量点时间戳
        if (time_point < extrapolator_->GetLastExtrapolatedTime()) {
            // 激光数据时间滞后了 , 报警提示(标志位)
            if (!warned) {
                LOG(ERROR)
                        << "Timestamp of individual range data point jumps backwards from "
                        << extrapolator_->GetLastExtrapolatedTime() << " to " << time_point;
                warned = true;
            }
            // 取姿态外推器的上一次 '推算' 的时间
            time_point = extrapolator_->GetLastExtrapolatedTime();
        }
        // 给定时间'time_point' , 利用姿态外推器推算该时刻机器人的姿态
        range_data_poses.push_back(
                    extrapolator_->ExtrapolatePose(time_point).cast<float>());
    }
    //如果我们还没有累积过扫描数据，就重置对象accumulated_range_data_
    if (num_accumulated_ == 0) {
        // 'accumulated_range_data_.origin' is uninitialized until the last
        // accumulation.
        accumulated_range_data_ = sensor::RangeData{{}, {}, {}};
    }

    // Drop any returns below the minimum range and convert returns beyond the
    // maximum range into misses.
    // 抛弃掉所有测量距离小于配置min_range的hit点
    // 并把那些超过配置max_range的测量点划归miss点
    for (size_t i = 0; i < synchronized_data.ranges.size(); ++i) {
        // 遍历同步的激光数据
        // 取一个激光点
        const Eigen::Vector4f& hit = synchronized_data.ranges[i].point_time;
        /////////////////Attention!!!////////////////////////////////////////////////
        // 利用推算出来该时刻机器人的位姿range_data_poses[i] : 机器人坐标系到local 坐标系的变换
        // 将激光点的坐标系原点转换到 local 坐标系下 (注意: 是local坐标系下 , 不是机器人坐标系下)
        const Eigen::Vector3f origin_in_local =
                range_data_poses[i] *                           //这是机器人坐标系到local 坐标系的变换
                synchronized_data.origins.at(synchronized_data.ranges[i].origin_index); //这是激光点坐标系原点在机器人坐标系的坐标
        // 利用推算出来该时刻机器人的位姿range_data_poses[i] : 机器人坐标系到local 坐标系的变换
        // 将hit中的激光点转换到 local 坐标系下
        const Eigen::Vector3f hit_in_local =
                range_data_poses[i] *   //机器人坐标系到local 坐标系的变换
                hit.head<3>();          //这是激光点坐标系原点在机器人坐标系的坐标
        // 求的hit点与激光点坐标原点在 local坐标系下的 差向量
        const Eigen::Vector3f delta = hit_in_local - origin_in_local;
        // 求模长
        const float range = delta.norm();
        // 检查是否大于配置的最小距离
        if (range >= options_.min_range()) {
            // 检查是否超过配置的最大距离
            if (range <= options_.max_range()) {
                // 都符合, 则将在local坐标系下的hit点 推入到accumulated_range_data_.returns
                accumulated_range_data_.returns.push_back(hit_in_local);
            } else {
                // 如果超过配置的最大距离
                // 则认为是miss点 ,
                // 在该miss点方向上设定距离的某个点作为miss点, 推入到accumulated_range_data_
                accumulated_range_data_.misses.push_back(
                            origin_in_local +
                            options_.missing_data_ray_length() / range * delta);
            }
        }
    }
    // 累加计数num_accumulated_
    ++num_accumulated_;

    /////////上面有一个重要操作就是将激光点从机器人坐标系转换到local坐标系//////////////////////
    /////////即 `accumulated_range_data_` 是在local坐标系下的激光点//////////////////////
    ////////因此,后面进行的scanMatching 得到的位姿估计都是local坐标系下的位姿///////////////

    // 每当累积的传感器数据数量超过了配置值num_accumulated_range_data
    if (num_accumulated_ >= options_.num_accumulated_range_data()) {
        // 对累积数据值清零
        num_accumulated_ = 0;
        // 从姿态外推器获取给定时间的重力方向向量
        const transform::Rigid3d gravity_alignment = transform::Rigid3d::Rotation(
                    extrapolator_->EstimateGravityOrientation(time));
        // TODO(gaschler): This assumes that 'range_data_poses.back()' is at time
        // 'time'.

        // 设置累积激光数据的原点 = 最后一个激光点的时间戳的时刻所对应的机器人在local坐标系的位姿  [平移量]
        accumulated_range_data_.origin = range_data_poses.back().translation();
        /** 调用函数AddAccumulatedRangeData (时间戳 , 对齐到重力坐标系并且经过体素滤波的点云 , 重力方向向量)
        // (1)调用ScanMatch 进行扫描匹配
        // (2)返回位姿估计 pose_estimate_2d = T_{local<--body} 机器人在local坐标系的坐标 , 即机器人(body)坐标系(tracking 坐标系)到local坐标系的变换
        // (3)将激光点云转换到local map坐标系下
        // (4)将该帧激光扫描数据插入到子图(更新概率地图)
        // (5)返回 : 匹配结果结构体 <MatchingResult> (时间戳 , 位姿估计 , 插入到子图的激光数据 , 插入结果)
        */
        return AddAccumulatedRangeData(
                    time,
                    TransformToGravityAlignedFrameAndFilter(    // 将激光数据转换到重力对齐的坐标系,并且重新转回机器人坐标系,并且进行滤波
                        gravity_alignment.cast<float>() *
                       range_data_poses.back().inverse(),   // local坐标系到机器人坐标系的变换
                        accumulated_range_data_),   //在local坐标系下的激光点 , 但是在这里又被重新转回机器人坐标系
                    gravity_alignment);
    }
    return nullptr;
}

// 调用ScanMatch 进行扫描匹配
// 返回位姿估计 pose_estimate_2d = T_{local<--body} 机器人在local坐标系的坐标 , 即机器人(body)坐标系(tracking 坐标系)到local坐标系的变换
// 将激光点云转换到local map坐标系下
// 将该帧激光扫描数据插入到子图(更新概率地图)
// 返回 : 匹配结果结构体 <MatchingResult> (时间戳 , 位姿估计 , 插入到子图的激光数据 , 插入结果)
std::unique_ptr<LocalTrajectoryBuilder2D::MatchingResult>
LocalTrajectoryBuilder2D::AddAccumulatedRangeData(
        const common::Time time,                                //时间戳
        const sensor::RangeData& gravity_aligned_range_data,    //重力对齐的点云数据(机器人坐标系下)
        const transform::Rigid3d& gravity_alignment) {          //重力方向向量
    // 检查点云数据
    if (gravity_aligned_range_data.returns.empty()) {
        LOG(WARNING) << "Dropped empty horizontal range data.";
        return nullptr;
    }

    // Computes a gravity aligned pose prediction.
    ///这里是利用姿态外推器, 得到位姿粗值
    // 计算没有重力对齐的位姿预测
    const transform::Rigid3d non_gravity_aligned_pose_prediction =
            extrapolator_->ExtrapolatePose(time);
    // 利用重力方向向量修正位姿预测
    // 然后将3D的位姿变换 转成 2D的, 即抛弃z轴平移量 , 旋转只保留yaw轴旋转
    const transform::Rigid2d pose_prediction = transform::Project2D(
                non_gravity_aligned_pose_prediction * gravity_alignment.inverse());

    // local map frame <- gravity-aligned frame
    // 调用ScanMatch 进行扫描匹配 , 输入(时间戳 , 位姿初始值 , 重力对齐的激光数据(机器人坐标系下))
    // 返回位姿估计 pose_estimate_2d = T_{local<--body} :
    // 机器人在local map坐标系的坐标 , 即机器人(body)坐标系(tracking 坐标系)到local map坐标系的变换
    // P_{local} = R_{local<--body}* P_{body} + t_{local<--body}
    // 这是因为 子图的cell index 都是由 local坐标系的激光点构成的 , 即子图是在local map坐标系下的
    // 因此, 通过scanMatching 得到的位姿, 也是local map坐标系下的
    std::unique_ptr<transform::Rigid2d> pose_estimate_2d =
            ScanMatch(time, pose_prediction, gravity_aligned_range_data);
    // 检查返回结果
    if (pose_estimate_2d == nullptr) {
        LOG(WARNING) << "Scan matching failed.";
        return nullptr;
    }

    // 对匹配结果使用重力矫正, 然后重新变成3D的位姿估计
    const transform::Rigid3d pose_estimate =
            transform::Embed3D(*pose_estimate_2d) * gravity_alignment;

    // 向姿态外推器添加这次观测
    extrapolator_->AddPose(time, pose_estimate);

    ///qpc: bug  ===> gravity_aligned_range_data本来就是在local map坐标系下的激光扫描
    // 将激光点云转换到local map坐标系下
    sensor::RangeData range_data_in_local =
            TransformRangeData(gravity_aligned_range_data,        //机器人坐标系下的local map
                               transform::Embed3D(pose_estimate_2d->cast<float>()));    //pose_estimate_2d : 机器人坐标系到local map坐标系的变换

    // 使用local map坐标系下的激光扫描更新子图, 即 子图也是local map坐标系下的
    // 将该帧激光扫描数据插入到子图(更新概率地图) , 并返回插入结果 <InsertionResult>
    std::unique_ptr<InsertionResult> insertion_result =
            InsertIntoSubmap(time, range_data_in_local, gravity_aligned_range_data,
                             pose_estimate, gravity_alignment.rotation());

    // 获取(当前时间) 距离 (开始累积数据的时间，也是开始跟踪轨迹的时间) 已经多久了
    auto duration = std::chrono::steady_clock::now() - accumulation_started_;
    // 用于评价
    kLocalSlamLatencyMetric->Set(
                std::chrono::duration_cast<std::chrono::seconds>(duration).count());

    // 返回 : 匹配结果结构体 <MatchingResult> (时间戳 , 位姿估计 , 插入到子图的激光数据 , 插入结果)
    return common::make_unique<MatchingResult>(
                MatchingResult{time, pose_estimate, std::move(range_data_in_local),
                               std::move(insertion_result)});
}

// 将子图指针保存到insertion_submaps
// 子图插入激光数据(更新子图概率地图)
std::unique_ptr<LocalTrajectoryBuilder2D::InsertionResult>
LocalTrajectoryBuilder2D::InsertIntoSubmap(
        const common::Time time,                                    //时间戳
        const sensor::RangeData& range_data_in_local,               //在local map坐标系下的激光数据
        const sensor::RangeData& gravity_aligned_range_data,        //没有进行坐标转换的激光点云
        const transform::Rigid3d& pose_estimate,                    //scanMatching得到的位姿估计(local map坐标系下的位姿)
        const Eigen::Quaterniond& gravity_alignment) {              //重力对齐向量
    // 根据运动滤波器,检查当前位姿与之前的位姿是否差别不大
    if (motion_filter_.IsSimilar(time, pose_estimate)) {
        // 是,则返回
        return nullptr;
    }

    // Querying the active submaps must be done here before calling
    // InsertRangeData() since the queried values are valid for next insertion.
    //
    std::vector<std::shared_ptr<const Submap2D>> insertion_submaps;
    // 遍历active_submaps_ 中的子图 ,进行保存 (这一步要在InsertRangeData()之前执行)
    for (const std::shared_ptr<Submap2D>& submap : active_submaps_.submaps()) {
        // 将子图保存到insertion_submaps
        insertion_submaps.push_back(submap);
    }
    // 插入激光数据(更新子图概率地图)
    active_submaps_.InsertRangeData(range_data_in_local);

    // 对没有进行坐标转换的激光点云进行体素滤波
    sensor::AdaptiveVoxelFilter adaptive_voxel_filter(
                options_.loop_closure_adaptive_voxel_filter_options());
    const sensor::PointCloud filtered_gravity_aligned_point_cloud =
            adaptive_voxel_filter.Filter(gravity_aligned_range_data.returns);

    // 返回: 插入结果<InsertionResult> (轨迹节点数据[时间戳,重力方向向量,没有重力对齐的滤波点云,{} {} {},
    //                                  scanMatching位姿估计] , active_submaps_的副本  )
    return common::make_unique<InsertionResult>(InsertionResult{
                                                    std::make_shared<const TrajectoryNode::Data>(TrajectoryNode::Data{
                                                        time,
                                                        gravity_alignment,
                                                        filtered_gravity_aligned_point_cloud,
                                                        {},  // 'high_resolution_point_cloud' is only used in 3D.
                                                        {},  // 'low_resolution_point_cloud' is only used in 3D.
                                                        {},  // 'rotational_scan_matcher_histogram' is only used in 3D.
                                                        pose_estimate}),
                                                    std::move(insertion_submaps)});
}

// 1. 利用IMU数据的时间戳来初始化姿态外推器
// 2. 调用姿态外推器extrapolator_->AddImuData()
void LocalTrajectoryBuilder2D::AddImuData(const sensor::ImuData& imu_data) {
    // 首先检查配置项是否要求使用IMU数据
    CHECK(options_.use_imu_data()) << "An unexpected IMU packet was added.";
    // 利用IMU数据的时间戳来初始化姿态外推器
    InitializeExtrapolator(imu_data.time);
    // AddImuData(): 添加imu数据到姿态外推器的成员变量'imu_data_', 然后根据时间戳等条件剔除无效imu数据
    extrapolator_->AddImuData(imu_data);
}

// 1. 如果姿态外推器还没有被初始化,则返回
// 2. 否则调用extrapolator_->AddOdometryData() 添加里程计数据
void LocalTrajectoryBuilder2D::AddOdometryData(
        const sensor::OdometryData& odometry_data) {
    // 如果还没有初始化姿态外推器, 则不能添加里程计数据
    if (extrapolator_ == nullptr) {
        // Until we've initialized the extrapolator we cannot add odometry data.
        LOG(INFO) << "Extrapolator not yet initialized.";
        return;
    }
    // 调用extrapolator_->AddOdometryData()
    /// 这里还没看
    extrapolator_->AddOdometryData(odometry_data);
}

// 姿态外推器的延迟初始化
void LocalTrajectoryBuilder2D::InitializeExtrapolator(const common::Time time) {
    if (extrapolator_ != nullptr) {
        return;
    }
    // We derive velocities from poses which are at least 1 ms apart for numerical
    // stability. Usually poses known to the extrapolator will be further apart
    // in time and thus the last two are used.
    // 为了数值稳定性，我们从至少相距1毫秒的位姿中推导出速度, 不过通常实际上都会大于这个时间

    // 使用 (1ms , 配置中的重力加速度) 作为参数, 初始化姿态外推器
    constexpr double kExtrapolationEstimationTimeSec = 0.001;
    // TODO(gaschler): Consider using InitializeWithImu as 3D does.
    extrapolator_ = common::make_unique<PoseExtrapolator>(
                ::cartographer::common::FromSeconds(kExtrapolationEstimationTimeSec),
                options_.imu_gravity_time_constant());
    // 姿态外推器添加第一个pose (时间戳 , T[I|0])
    extrapolator_->AddPose(time, transform::Rigid3d::Identity());
}

// 配准评价
void LocalTrajectoryBuilder2D::RegisterMetrics(
        metrics::FamilyFactory* family_factory) {
    auto* latency = family_factory->NewGaugeFamily(
                "mapping_internal_2d_local_trajectory_builder_latency",
                "Duration from first incoming point cloud in accumulation to local slam "
                "result");
    kLocalSlamLatencyMetric = latency->Add({});
    auto score_boundaries = metrics::Histogram::FixedWidth(0.05, 20);
    auto* scores = family_factory->NewHistogramFamily(
                "mapping_internal_2d_local_trajectory_builder_scores",
                "Local scan matcher scores", score_boundaries);
    kFastCorrelativeScanMatcherScoreMetric =
            scores->Add({{"scan_matcher", "fast_correlative"}});
    auto cost_boundaries = metrics::Histogram::ScaledPowersOf(2, 0.01, 100);
    auto* costs = family_factory->NewHistogramFamily(
                "mapping_internal_2d_local_trajectory_builder_costs",
                "Local scan matcher costs", cost_boundaries);
    kCeresScanMatcherCostMetric = costs->Add({{"scan_matcher", "ceres"}});
    auto distance_boundaries = metrics::Histogram::ScaledPowersOf(2, 0.01, 10);
    auto* residuals = family_factory->NewHistogramFamily(
                "mapping_internal_2d_local_trajectory_builder_residuals",
                "Local scan matcher residuals", distance_boundaries);
    kScanMatcherResidualDistanceMetric =
            residuals->Add({{"component", "distance"}});
    kScanMatcherResidualAngleMetric = residuals->Add({{"component", "angle"}});
}

}  // namespace mapping
}  // namespace cartographer
