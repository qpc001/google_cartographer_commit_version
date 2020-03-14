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

#ifndef CARTOGRAPHER_SENSOR_RANGE_DATA_H_
#define CARTOGRAPHER_SENSOR_RANGE_DATA_H_

#include "cartographer/common/port.h"
#include "cartographer/sensor/compressed_point_cloud.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/sensor/proto/sensor.pb.h"

namespace cartographer {
namespace sensor {

/*
RangeData:
数据成员包括
1),原始位置,{x0,y0,z0}
2),返回点云,{x,y,z}
3),缺失点云,标识free space.
*/

// Rays begin at 'origin'. 'returns' are the points where obstructions were
// detected. 'misses' are points in the direction of rays for which no return
// was detected, and were inserted at a configured distance. It is assumed that
// between the 'origin' and 'misses' is free space.
// 射线始于“原点”。“返回”检测到障碍物的位置
// 如果某个方向的射线没有返回，该点就丢失，使用一个给定的距离插入

// 原始Rangedata
struct RangeData {
  Eigen::Vector3f origin;   //原点
  PointCloud returns;       //有返回的
  PointCloud misses;        //miss点(无反射)
};

// Like 'RangeData', but with 'TimedPointClouds'.
// 点云带有时间戳的RangeData
struct TimedRangeData {
  Eigen::Vector3f origin;
  TimedPointCloud returns;
  TimedPointCloud misses;
};

// 根据3D变换，对点云进行变换
RangeData TransformRangeData(const RangeData& range_data,
                             const transform::Rigid3f& transform);
// 根据3D变换，对点云进行变换
TimedRangeData TransformTimedRangeData(const TimedRangeData& range_data,
                                       const transform::Rigid3f& transform);

// Crops 'range_data' according to the region defined by 'min_z' and 'max_z'.
// 给定z轴范围，裁剪点云
RangeData CropRangeData(const RangeData& range_data, float min_z, float max_z);

// Crops 'range_data' according to the region defined by 'min_z' and 'max_z'.
// 给定z轴范围，裁剪点云
TimedRangeData CropTimedRangeData(const TimedRangeData& range_data, float min_z,
                                  float max_z);

// Converts 'range_data' to a proto::RangeData.
// 转换到proto
proto::RangeData ToProto(const RangeData& range_data);

// Converts 'proto' to RangeData.
// 从proto转换
RangeData FromProto(const proto::RangeData& proto);

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_SENSOR_RANGE_DATA_H_
