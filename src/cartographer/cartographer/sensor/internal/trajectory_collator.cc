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

#include "cartographer/sensor/internal/trajectory_collator.h"

namespace cartographer {
namespace sensor {

// 为指定的某个传感器的消息队列设置回调函数
void TrajectoryCollator::AddTrajectory(
    const int trajectory_id,                                    //轨迹id
    const std::unordered_set<std::string>& expected_sensor_ids, //传感器id(字符串)
    const Callback& callback) {                                 //回调
  CHECK_EQ(trajectory_to_queue_.count(trajectory_id), 0);
  // 遍历传感器id(字符串)
  for (const auto& sensor_id : expected_sensor_ids) {
    // {轨迹id, 传感器id}为每个传感器构成 独一无二的QueueKey
    const auto queue_key = QueueKey{trajectory_id, sensor_id};
    // 使用QueueKey增加一个之前不存在的新的队列，当有数据的时候，执行回调函数
    // 即: 为指定的某个传感器的消息队列设置回调函数
    trajectory_to_queue_[trajectory_id].AddQueue(
        queue_key, [callback, sensor_id](std::unique_ptr<Data> data) {
          callback(sensor_id, std::move(data));
        });
    // 储存queue_key
    trajectory_to_queue_keys_[trajectory_id].push_back(queue_key);
  }
}

// 结束轨迹
void TrajectoryCollator::FinishTrajectory(const int trajectory_id) {
    // 遍历trajectory_to_queue_keys_[trajectory_id]中的queue_key
  for (const auto& queue_key : trajectory_to_queue_keys_[trajectory_id]) {
      // 标记该轨迹中的所有queue已经结束，不能再添加数据
    trajectory_to_queue_.at(trajectory_id).MarkQueueAsFinished(queue_key);
  }
}

// 添加传感器数据
void TrajectoryCollator::AddSensorData(const int trajectory_id,
                                       std::unique_ptr<Data> data) {
    // 根据轨迹id,传感器id , 得到queue_key
  QueueKey queue_key{trajectory_id, data->GetSensorId()};
  // 根据轨迹id, 向对应的数据队列添加 (queue_key,传感器数据)
  trajectory_to_queue_.at(trajectory_id)
      .Add(std::move(queue_key), std::move(data));
}

void TrajectoryCollator::Flush() {
  for (auto& it : trajectory_to_queue_) {
    it.second.Flush();
  }
}

//  空
common::optional<int> TrajectoryCollator::GetBlockingTrajectoryId() const {
  return common::optional<int>();
}

}  // namespace sensor
}  // namespace cartographer
