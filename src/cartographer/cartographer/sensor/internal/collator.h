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

#ifndef CARTOGRAPHER_SENSOR_INTERNAL_COLLATOR_H_
#define CARTOGRAPHER_SENSOR_INTERNAL_COLLATOR_H_

#include <functional>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "cartographer/sensor/collator_interface.h"
#include "cartographer/sensor/data.h"
#include "cartographer/sensor/internal/ordered_multi_queue.h"
/*
Collator,采集者，校对者，整理者
将多传感器采集的数据归并到轨迹上。

Collator类:不可拷贝,不可赋值.
只有一个默认构造函数.
有2个数据成员

1,OrderedMultiQueue queue_; key是pair<轨迹线id,传感器id>.
一般情况下，对于已有的bag文件，轨迹id等于0.

2,std::unordered_map<int, std::vector<QueueKey>> queue_keys_
轨迹线和队列key组成的hash表,1：N模式

Collator类主要提供三个操作:
1,AddTrajectory() 添加一个轨迹线,
2,FinishTrajectory() 标记轨迹线已经采集完成
3,AddSensorData()接收传感器数据
4,Flush()刷新
*/

namespace cartographer {
namespace sensor {

class Collator : public CollatorInterface {
 public:
  Collator() {}

  Collator(const Collator&) = delete;
  Collator& operator=(const Collator&) = delete;

  /*添加一个轨迹线,接受有序的传感器数据,并使用callback回调处理data。
    expected_sensor_ids: 预期的传感器id集合，添加轨迹的时候会进行遍历
  */
  void AddTrajectory(int trajectory_id,
                     const std::unordered_set<std::string>& expected_sensor_ids,
                     const Callback& callback) override;

  // Marks 'trajectory_id' as finished.标记轨迹线已经完成采样.
  void FinishTrajectory(int trajectory_id) override;

  /*添加一个传感器id对应的数据data,数据必须按时间排序，*/
  void AddSensorData(int trajectory_id, std::unique_ptr<Data> data) override;

  //刷新
  void Flush() override;

  //返回阻塞的轨迹id,
  //此种情况多见于某一传感器持久未采集data，造成ordered_multi_queue阻塞。
  common::optional<int> GetBlockingTrajectoryId() const override;

 private:
  // Queue keys are a pair of trajectory ID and sensor identifier.
  // 多个key构成的多队列
  OrderedMultiQueue queue_;

  //int为传感器id,vector是id+sensor组成的QueueKey
  // Map of trajectory ID to all associated QueueKeys.
  std::unordered_map<int, std::vector<QueueKey>> queue_keys_;
};

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_SENSOR_INTERNAL_COLLATOR_H_
