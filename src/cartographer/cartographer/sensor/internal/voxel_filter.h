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

#ifndef CARTOGRAPHER_SENSOR_INTERNAL_VOXEL_FILTER_H_
#define CARTOGRAPHER_SENSOR_INTERNAL_VOXEL_FILTER_H_

#include <bitset>
#include <unordered_set>

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/sensor/proto/adaptive_voxel_filter_options.pb.h"
#include "cartographer/sensor/timed_point_cloud_data.h"

namespace cartographer {
namespace sensor {

// Voxel filter for point clouds. For each voxel, the assembled point cloud
// contains the first point that fell into it from any of the inserted point
// clouds.
// 体素滤波器
class VoxelFilter {
 public:
  // 'size' is the length of a voxel edge.
  // 构造函数，边长构造
  explicit VoxelFilter(float size) : resolution_(size) {}

  VoxelFilter(const VoxelFilter&) = delete;
  VoxelFilter& operator=(const VoxelFilter&) = delete;

  // Returns a voxel filtered copy of 'point_cloud'.
  // 返回滤波之后的点云
  PointCloud Filter(const PointCloud& point_cloud);

  // Same for TimedPointCloud.
  // 返回滤波之后的点云
  TimedPointCloud Filter(const TimedPointCloud& timed_point_cloud);

  // Same for RangeMeasurement.
  // 对RangeMeasurement进行滤波
  std::vector<sensor::TimedPointCloudOriginData::RangeMeasurement> Filter(
      const std::vector<sensor::TimedPointCloudOriginData::RangeMeasurement>&
          range_measurements);

 private:
  using KeyType = std::bitset<3 * 32>;

  //cell索引转keyType类型
  static KeyType IndexToKey(const Eigen::Array3i& index);

  //输入某个点，获取属于的某个cell索引
  Eigen::Array3i GetCellIndex(const Eigen::Vector3f& point) const;

  //分辨率
  float resolution_;
  std::unordered_set<KeyType> voxel_set_;
};

// 自适应体素滤波器
proto::AdaptiveVoxelFilterOptions CreateAdaptiveVoxelFilterOptions(
    common::LuaParameterDictionary* const parameter_dictionary);

class AdaptiveVoxelFilter {
 public:
  explicit AdaptiveVoxelFilter(
      const proto::AdaptiveVoxelFilterOptions& options);

  AdaptiveVoxelFilter(const AdaptiveVoxelFilter&) = delete;
  AdaptiveVoxelFilter& operator=(const AdaptiveVoxelFilter&) = delete;

  PointCloud Filter(const PointCloud& point_cloud) const;

 private:
  const proto::AdaptiveVoxelFilterOptions options_;
};

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_SENSOR_INTERNAL_VOXEL_FILTER_H_
