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

#include "cartographer/mapping/internal/collated_trajectory_builder.h"

#include "cartographer/common/time.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

namespace {

constexpr double kSensorDataRatesLoggingPeriodSeconds = 15.;

}  // namespace

// 构造函数
// 对轨迹所需的传感器消息队列进行初始化, 并设置对应的回调函数
// 后面当调用CollatedTrajectoryBuilder::AddData()向某个传感器数据队列添加数据时, 会自动执行所设置的回调函数
CollatedTrajectoryBuilder::CollatedTrajectoryBuilder(
        sensor::CollatorInterface* const sensor_collator,                           //传感器数据收集器
        const int trajectory_id,                                                    //轨迹id
        const std::set<SensorId>& expected_sensor_ids,                              //预期的传感器{传感器id , 传感器类型}
        std::unique_ptr<TrajectoryBuilderInterface> wrapped_trajectory_builder)     //这里写的是基类, 实际上传进来的是
        //2D : GlobalTrajectoryBuilder<LocalTrajectoryBuilder2D, mapping::PoseGraph2D>>
    : //成员变量初始化
      sensor_collator_(sensor_collator),
      trajectory_id_(trajectory_id),
      wrapped_trajectory_builder_(std::move(wrapped_trajectory_builder)),
      last_logging_time_(std::chrono::steady_clock::now()) {

    // 遍历传感器{传感器id , 传感器类型}
    std::unordered_set<std::string> expected_sensor_id_strings;
    for (const auto& sensor_id : expected_sensor_ids) {
        // 将传感器id号(string类型) , 插入到expected_sensor_id_strings 容器
        expected_sensor_id_strings.insert(sensor_id.id);
    }
    // 调用传感器数据收集器
    // 为指定的某个传感器的消息队列设置回调函数
    sensor_collator_->AddTrajectory(
                trajectory_id,                  //轨迹ID
                expected_sensor_id_strings,     //传感器ID
                //回调函数, 当调用CollatedTrajectoryBuilder::AddData()向 某个传感器数据队列添加数据时, 会自动执行下面的回调函数
                [this](const std::string& sensor_id, std::unique_ptr<sensor::Data> data)
    {
        /// 回调内容
        // 处理传感器数据
        // 将数据交给wrapped_trajectory_builder_
        // 也就是GlobalTrajectoryBuilder<LocalTrajectoryBuilder2D, mapping::PoseGraph2D>> 类的对象
        HandleCollatedSensorData(sensor_id, std::move(data));
    });
}

CollatedTrajectoryBuilder::~CollatedTrajectoryBuilder() {}

// 调用sensor_collator_,向特定传感器ID的数据队列添加数据 , 之后会自动执行回调函数CollatedTrajectoryBuilder::HandleCollatedSensorData()
void CollatedTrajectoryBuilder::AddData(std::unique_ptr<sensor::Data> data) {
    sensor_collator_->AddSensorData(trajectory_id_, std::move(data));
}

//
void CollatedTrajectoryBuilder::HandleCollatedSensorData(
        const std::string& sensor_id,           //传感器ID
        std::unique_ptr<sensor::Data> data) {   //传感器数据
    //==========================================================================
    // 框起来的这块没什么卵用,只是用来打印的 , 整个函数只有最后一句话有用
    // 检查这个传感器有没有RateTimer
    auto it = rate_timers_.find(sensor_id);
    // 如果还没有
    if (it == rate_timers_.end()) {
        // 则创建一个, 然后储存到rate_timers_
        // 然后it指向这个新创建的RateTimer
        it = rate_timers_
                .emplace(
                    std::piecewise_construct, std::forward_as_tuple(sensor_id),
                    std::forward_as_tuple(
                        common::FromSeconds(kSensorDataRatesLoggingPeriodSeconds)))
                .first;
    }
    // ???
    it->second.Pulse(data->GetTime());

    // 如果 当前时间- 最后一次打印传感器数据速率的时间 > 15 秒
    if (std::chrono::steady_clock::now() - last_logging_time_ >
            common::FromSeconds(kSensorDataRatesLoggingPeriodSeconds)) {
        //遍历rate_timers_ , 输出传感器速率
        for (const auto& pair : rate_timers_) {
            LOG(INFO) << pair.first << " rate: " << pair.second.DebugString();
        }
        //记录上一次打印传感器速率的事件
        last_logging_time_ = std::chrono::steady_clock::now();
    }
    //====================================================================================

    ///整个函数就这里有用
    /// 将数据交给wrapped_trajectory_builder_
    /// 也就是GlobalTrajectoryBuilder<LocalTrajectoryBuilder2D, mapping::PoseGraph2D>> 类的对象
    data->AddToTrajectoryBuilder(wrapped_trajectory_builder_.get());
}

}  // namespace mapping
}  // namespace cartographer
