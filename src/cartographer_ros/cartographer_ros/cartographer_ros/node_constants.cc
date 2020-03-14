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

#include "cartographer_ros/node_constants.h"

#include "glog/logging.h"

namespace cartographer_ros {

// 根据传感器数量,返回预期的topic字符串, 如果只有1个传感器,那么就返回原来的topic字符串, 否则, 就在原来的字符串加上编号,得到新的topic
std::vector<std::string> ComputeRepeatedTopicNames(const std::string& topic,
                                                   const int num_topics) {
  CHECK_GE(num_topics, 0);
  // 如果topic数量为1,则直接返回传进来的topic字符串
  if (num_topics == 1) {
    return {topic};
  }
  // 如果传感器数量>1,
  std::vector<std::string> topics;
  topics.reserve(num_topics);
  for (int i = 0; i < num_topics; ++i) {
      //则将原始的topic 字符串进行修改
      // string topic ==> topic_第几个传感器   举例:  imu ====>  imu_1 , imu_2 ...
    topics.emplace_back(topic + "_" + std::to_string(i + 1));
  }
  return topics;
}

}  // namespace cartographer_ros
