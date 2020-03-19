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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_COLLATED_TRAJECTORY_BUILDER_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_COLLATED_TRAJECTORY_BUILDER_H_

#include <chrono>
#include <map>
#include <memory>
#include <set>
#include <string>

#include "cartographer/common/port.h"
#include "cartographer/common/rate_timer.h"
#include "cartographer/mapping/local_slam_result_data.h"
#include "cartographer/mapping/submaps.h"
#include "cartographer/mapping/trajectory_builder_interface.h"
#include "cartographer/sensor/collator_interface.h"
#include "cartographer/sensor/internal/dispatchable.h"

namespace cartographer {
namespace mapping {

// Collates sensor data using a sensor::CollatorInterface, then passes it on to
// a mapping::TrajectoryBuilderInterface which is common for 2D and 3D.

/// 这TM是个壳
/// 只是为了兼容2D和3D
///
class CollatedTrajectoryBuilder : public TrajectoryBuilderInterface {
public:
    using SensorId = TrajectoryBuilderInterface::SensorId;

    // 构造函数
    CollatedTrajectoryBuilder(
            sensor::CollatorInterface* sensor_collator,
            int trajectory_id,
            const std::set<SensorId>& expected_sensor_ids,
            std::unique_ptr<TrajectoryBuilderInterface> wrapped_trajectory_builder);
    ~CollatedTrajectoryBuilder() override;

    CollatedTrajectoryBuilder(const CollatedTrajectoryBuilder&) = delete;
    CollatedTrajectoryBuilder& operator=(const CollatedTrajectoryBuilder&) =
            delete;

    /// 传感器消息的处理
    //  都是通过AddData()函数处理
    void AddSensorData(
            const std::string& sensor_id,
            const sensor::TimedPointCloudData& timed_point_cloud_data) override {
        AddData(sensor::MakeDispatchable(sensor_id, timed_point_cloud_data));
    }

    void AddSensorData(const std::string& sensor_id,
                       const sensor::ImuData& imu_data) override {
        AddData(sensor::MakeDispatchable(sensor_id, imu_data));
    }

    void AddSensorData(const std::string& sensor_id,
                       const sensor::OdometryData& odometry_data) override {
        AddData(sensor::MakeDispatchable(sensor_id, odometry_data));
    }

    void AddSensorData(
            const std::string& sensor_id,
            const sensor::FixedFramePoseData& fixed_frame_pose_data) override {
        AddData(sensor::MakeDispatchable(sensor_id, fixed_frame_pose_data));
    }

    void AddSensorData(const std::string& sensor_id,
                       const sensor::LandmarkData& landmark_data) override {
        AddData(sensor::MakeDispatchable(sensor_id, landmark_data));
    }

    void AddLocalSlamResultData(std::unique_ptr<mapping::LocalSlamResultData>
                                local_slam_result_data) override {
        AddData(std::move(local_slam_result_data));
    }

private:
    // 调用sensor_collator_,向特定传感器ID的数据队列添加数据,
    // 之后会自动执行回调函数CollatedTrajectoryBuilder::HandleCollatedSensorData()
    void AddData(std::unique_ptr<sensor::Data> data);

    // 回调
    void HandleCollatedSensorData(const std::string& sensor_id,
                                  std::unique_ptr<sensor::Data> data);

    // 传感器收集器
    sensor::CollatorInterface* const sensor_collator_;

    // 轨迹ID
    const int trajectory_id_;

    // 2D : GlobalTrajectoryBuilder<LocalTrajectoryBuilder2D, mapping::PoseGraph2D>>
    std::unique_ptr<TrajectoryBuilderInterface> wrapped_trajectory_builder_;

    // Time at which we last logged the rates of incoming sensor data.
    // 最后一次记录传入传感器数据速率的时间
    std::chrono::steady_clock::time_point last_logging_time_;
    std::map<std::string, common::RateTimer<>> rate_timers_;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_COLLATED_TRAJECTORY_BUILDER_H_
