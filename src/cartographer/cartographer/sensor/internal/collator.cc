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

#include "cartographer/sensor/internal/collator.h"

namespace cartographer {
namespace sensor {

// 主要作用: 为指定的某个传感器的消息队列设置回调函数
void Collator::AddTrajectory(
        const int trajectory_id,                                        //轨迹ID
        const std::unordered_set<std::string>& expected_sensor_ids,     //传感器id (字符串)
        const Callback& callback) {                                     //回调函数
    // 遍历所有传感器id
    for (const auto& sensor_id : expected_sensor_ids) {
        // {轨迹ID,传感器ID} 构成独一无二的 QueueKey
        const auto queue_key = QueueKey{trajectory_id, sensor_id};
        // 为指定的某个传感器的消息队列设置回调函数
        queue_.AddQueue(queue_key,                                          //queue_key 确定一个传感器
                        [callback, sensor_id](std::unique_ptr<Data> data) { //消息来了之后的回调函数
            callback(sensor_id, std::move(data));
        });
        //以轨迹ID为索引, 将queue_key 储存到queue_keys_
        queue_keys_[trajectory_id].push_back(queue_key);
    }
}
/*队列不再接收数据*/
void Collator::FinishTrajectory(const int trajectory_id) {
    for (const auto& queue_key : queue_keys_[trajectory_id]) {
        queue_.MarkQueueAsFinished(queue_key);
    }
}
//主要的操作,添加传感器数据,数据形式是:key+data
void Collator::AddSensorData(const int trajectory_id,
                             std::unique_ptr<Data> data) {
    //根据 轨迹id和传感器id，生成queueKey
    QueueKey queue_key{trajectory_id, data->GetSensorId()};
    // 将queueKey和data添加到OrderedMultiQueue类型的队列
    queue_.Add(std::move(queue_key), std::move(data));
}

void Collator::Flush() { queue_.Flush(); }

common::optional<int> Collator::GetBlockingTrajectoryId() const {
    return common::optional<int>(queue_.GetBlocker().trajectory_id);
}

}  // namespace sensor
}  // namespace cartographer
