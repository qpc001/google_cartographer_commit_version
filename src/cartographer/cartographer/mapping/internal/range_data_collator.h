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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_RANGE_DATA_COLLATOR_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_RANGE_DATA_COLLATOR_H_

#include <memory>

#include "cartographer/common/make_unique.h"
#include "cartographer/sensor/timed_point_cloud_data.h"

namespace cartographer {
namespace mapping {

// Synchronizes TimedPointCloudData from different sensors. Input needs only be
// monotonous in 'TimedPointCloudData::time', output is monotonous in per-point
// timing. Up to one message per sensor is buffered, so a delay of the period of
// the slowest sensor may be introduced, which can be alleviated by passing
// subdivisions.

// 从不同的传感器中对带有时间戳的点云数据进行时间同步
// 每个传感器最多缓冲一条消息，因此可能会因为最慢的传感器引起周期性延迟，这可以通过传送 细分的点云数据来缓解
/// 复杂,还没看
class RangeDataCollator {
public:
    // 构造激光数据收集器
    explicit RangeDataCollator(
            const std::vector<std::string>& expected_range_sensor_ids)  //传感器ID
        : expected_sensor_ids_(expected_range_sensor_ids.begin(),       // 初始化成员变量expected_sensor_ids_
                               expected_range_sensor_ids.end()) {}

    //
    sensor::TimedPointCloudOriginData AddRangeData(
            const std::string& sensor_id,
            const sensor::TimedPointCloudData& timed_point_cloud_data);

private:
    sensor::TimedPointCloudOriginData CropAndMerge();

    // 储存传感器id的集合
    const std::set<std::string> expected_sensor_ids_;
    // Store at most one message for each sensor.
    // 为每个传感器储存最多1帧的消息
    std::map<std::string, sensor::TimedPointCloudData> id_to_pending_data_;

    // 当前数据起始时间?
    common::Time current_start_ = common::Time::min();
    common::Time current_end_ = common::Time::min();
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_RANGE_DATA_COLLATOR_H_
