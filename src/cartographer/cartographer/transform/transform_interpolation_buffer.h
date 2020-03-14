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

#ifndef CARTOGRAPHER_TRANSFORM_TRANSFORM_INTERPOLATION_BUFFER_H_
#define CARTOGRAPHER_TRANSFORM_TRANSFORM_INTERPOLATION_BUFFER_H_

#include <vector>

#include "cartographer/common/time.h"
#include "cartographer/mapping/proto/trajectory.pb.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/timestamped_transform.h"

/*
基于时间有序的变换,支持在队列中按时间顺序查找,即使变换不存在于队列中，任然支持相邻时间内的插值变换进行近似。
作用与ROS的tf2函数族类似。

数据成员：
1,deque_;
成员函数：
1,Push()
2,Has()
3,Lookup()
4,earliest_time()
5,latest_time()
6,empty()
*/

namespace cartographer {
namespace transform {

// A time-ordered buffer of transforms that supports interpolated lookups.
class TransformInterpolationBuffer {
 public:
  TransformInterpolationBuffer() = default;
  // 用来构成轨迹， 返回trajectory对象指针
  explicit TransformInterpolationBuffer(
      const mapping::proto::Trajectory& trajectory);

  // Adds a new transform to the buffer and removes the oldest transform if the
  // buffer size limit is exceeded.
  ///添加变换到队列尾部,当缓冲区已满时,删除队首元素
  void Push(common::Time time, const transform::Rigid3d& transform);

  // Returns true if an interpolated transfrom can be computed at 'time'.
  bool Has(common::Time time) const;

  // Returns an interpolated transform at 'time'. CHECK()s that a transform at
  // 'time' is available.
  /// 查找给定时间的变换
  transform::Rigid3d Lookup(common::Time time) const;

  // Returns the timestamp of the earliest transform in the buffer or 0 if the
  // buffer is empty.
  /// 返回队列缓冲区内变换的最早时间，也就是队首元素
  common::Time earliest_time() const;

  // Returns the timestamp of the earliest transform in the buffer or 0 if the
  // buffer is empty.
  /// 最晚时间，也就是队尾元素
  common::Time latest_time() const;

  // Returns true if the buffer is empty.
  bool empty() const;

 private:
  //队列，元素是带时间戳的变换,存储了一段时间内的变换矩阵信息
  std::vector<TimestampedTransform> timestamped_transforms_;
};

}  // namespace transform
}  // namespace cartographer

#endif  // CARTOGRAPHER_TRANSFORM_TRANSFORM_INTERPOLATION_BUFFER_H_
