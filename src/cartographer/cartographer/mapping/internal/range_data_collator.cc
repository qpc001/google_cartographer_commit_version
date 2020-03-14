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

#include "cartographer/mapping/internal/range_data_collator.h"

#include <memory>

#include "cartographer/common/make_unique.h"
#include "cartographer/mapping/local_slam_result_data.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

sensor::TimedPointCloudOriginData RangeDataCollator::AddRangeData(
        const std::string& sensor_id,                                   //传感器ID
        const sensor::TimedPointCloudData& timed_point_cloud_data) {    //带时间戳的点云数据
    CHECK_NE(expected_sensor_ids_.count(sensor_id), 0);

    // 如果'id_to_pending_data_' 这个映射里面存在这个 ID的传感器
    if (id_to_pending_data_.count(sensor_id) != 0) {
        // ?
        current_start_ = current_end_;
        // When we have two messages of the same sensor, move forward the older of
        // the two (do not send out current).
        // 如果同一个传感器有两条数据 ,
        current_end_ = id_to_pending_data_.at(sensor_id).time;
        auto result = CropAndMerge();
        id_to_pending_data_.emplace(sensor_id, timed_point_cloud_data);
        return result;
    }
    id_to_pending_data_.emplace(sensor_id, timed_point_cloud_data);
    if (expected_sensor_ids_.size() != id_to_pending_data_.size()) {
        return {};
    }
    current_start_ = current_end_;
    // We have messages from all sensors, move forward to oldest.
    common::Time oldest_timestamp = common::Time::max();
    for (const auto& pair : id_to_pending_data_) {
        oldest_timestamp = std::min(oldest_timestamp, pair.second.time);
    }
    current_end_ = oldest_timestamp;
    return CropAndMerge();
}

sensor::TimedPointCloudOriginData RangeDataCollator::CropAndMerge() {
    // 这是要返回的东西TimedPointCloudOriginData(时间, 原点 , 测量数据)
    sensor::TimedPointCloudOriginData result{current_end_, {}, {}};
    //
    bool warned_for_dropped_points = false;
    // 遍历id_to_pending_data_ 里面的映射<传感器ID,点云数据>
    for (auto it = id_to_pending_data_.begin();it != id_to_pending_data_.end();) {
        // TimedPointCloudData: [时间戳+原点+激光数据(点云数据)]
        sensor::TimedPointCloudData& data = it->second;
        // TimedPointCloud : 储存(x,y,z,相对测量时间)
        sensor::TimedPointCloud& ranges = it->second.ranges;

        // overlap_begin : 取点云中的第一个点
        auto overlap_begin = ranges.begin();
        // 移动迭代器overlap_begin(相当于取后面的点) , 直到取得的某个点的时间 大于等于 current_start_
        while (overlap_begin < ranges.end() &&
               data.time + common::FromSeconds((*overlap_begin)[3]) <
               current_start_) {
            ++overlap_begin;
        }
        //
        auto overlap_end = overlap_begin;
        // 移动迭代器overlap_end(相当与继续取后面的点) , 直到后面的某个点的时间 大于 current_end_
        while (overlap_end < ranges.end() &&
               data.time + common::FromSeconds((*overlap_end)[3]) <= current_end_) {
            ++overlap_end;
        }

        if (ranges.begin() < overlap_begin && !warned_for_dropped_points) {
            LOG(WARNING) << "Dropped " << std::distance(ranges.begin(), overlap_begin)
                         << " earlier points.";
            warned_for_dropped_points = true;
        }

        // 复制重叠的
        if (overlap_begin < overlap_end) {
            std::size_t origin_index = result.origins.size();
            // 设置点云原点
            result.origins.push_back(data.origin);
            // 时间修正
            double time_correction = common::ToSeconds(data.time - current_end_);
            // 遍历中间的部分
            for (auto overlap_it = overlap_begin; overlap_it != overlap_end;
                 ++overlap_it) {
                // RangeMeasurement (带有时间的点 [4维] , 原点索引 )
                sensor::TimedPointCloudOriginData::RangeMeasurement point{*overlap_it,
                            origin_index};
                // current_end_ + point_time[3]_after == in_timestamp +
                // point_time[3]_before
                point.point_time[3] += time_correction;
                result.ranges.push_back(point);
            }
        }

        // Drop buffered points until overlap_end.
        if (overlap_end == ranges.end()) {
            it = id_to_pending_data_.erase(it);
        } else if (overlap_end == ranges.begin()) {
            ++it;
        } else {
            data = sensor::TimedPointCloudData{
                    data.time, data.origin,
                    sensor::TimedPointCloud(overlap_end, ranges.end())};
            ++it;
        }
    }

    std::sort(result.ranges.begin(), result.ranges.end(),
              [](const sensor::TimedPointCloudOriginData::RangeMeasurement& a,
              const sensor::TimedPointCloudOriginData::RangeMeasurement& b) {
        return a.point_time[3] < b.point_time[3];
    });
    return result;
}

}  // namespace mapping
}  // namespace cartographer
