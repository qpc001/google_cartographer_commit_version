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

#ifndef CARTOGRAPHER_SENSOR_COLLATOR_INTERFACE_H_
#define CARTOGRAPHER_SENSOR_COLLATOR_INTERFACE_H_

#include <functional>
#include <memory>
#include <unordered_set>
#include <vector>

#include "cartographer/common/optional.h"
#include "cartographer/sensor/data.h"

namespace cartographer {
namespace sensor {

//Collator的父类
class CollatorInterface {
 public:
  using Callback =
      std::function<void(const std::string&, std::unique_ptr<Data>)>;

  CollatorInterface() {}
  virtual ~CollatorInterface() {}
  CollatorInterface(const CollatorInterface&) = delete;
  CollatorInterface& operator=(const CollatorInterface&) = delete;

  /// 下面都是虚函数,由子类重载

  // Adds a trajectory to produce sorted sensor output for. Calls 'callback'
  // for each collated sensor data.
  // 添加一条轨线, 为每种传感器数据调用回调函数
  virtual void AddTrajectory(
      int trajectory_id,
      const std::unordered_set<std::string>& expected_sensor_ids,
      const Callback& callback) = 0;

  // Marks 'trajectory_id' as finished.
  // 结束轨迹
  virtual void FinishTrajectory(int trajectory_id) = 0;

  // Adds 'data' for 'trajectory_id' to be collated. 'data' must contain valid
  // sensor data. Sensor packets with matching 'data.sensor_id_' must be added
  // in time order.
  // 添加某条轨迹上的数据, 带有'data.sensor_id_'的数据包必须按时间顺序排列
  virtual void AddSensorData(int trajectory_id, std::unique_ptr<Data> data) = 0;

  // Dispatches all queued sensor packets. May only be called once.
  // AddSensorData may not be called after Flush.
  // ?
  virtual void Flush() = 0;

  // Must only be called if at least one unfinished trajectory exists. Returns
  // the ID of the trajectory that needs more data before CollatorInterface is
  // unblocked. Returns 'nullopt' for implementations that do not wait for a
  // particular trajectory.
  // 只有在至少有一个未完成的轨迹存在时才能调用, 返回阻塞中的轨迹id(即需要更多数据的轨迹)
  virtual common::optional<int> GetBlockingTrajectoryId() const = 0;
};

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_SENSOR_COLLATOR_INTERFACE_H_
