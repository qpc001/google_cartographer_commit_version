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

#ifndef CARTOGRAPHER_MAPPING_MAP_BUILDER_H_
#define CARTOGRAPHER_MAPPING_MAP_BUILDER_H_

#include "cartographer/mapping/map_builder_interface.h"

#include <memory>

#include "cartographer/common/thread_pool.h"
#include "cartographer/mapping/pose_graph.h"
#include "cartographer/mapping/proto/map_builder_options.pb.h"
#include "cartographer/sensor/collator_interface.h"

namespace cartographer {
namespace mapping {

proto::MapBuilderOptions CreateMapBuilderOptions(
        common::LuaParameterDictionary *const parameter_dictionary);
/**
 * @brief 一些坐标系说明
 *          1. global frame :   全局坐标系
 *          2. tracking frame : 机器人本体坐标系
 *          3. Local map :      每个子图所在的坐标系, 但子图原点不是这个坐标系的原点
 *                               (scanMatching 得到的就是机器人在这个坐标系的位姿)
 *          4. 子图坐标系:       基本没用到
 */


// Wires up the complete SLAM stack with TrajectoryBuilders (for local submaps)
// and a PoseGraph for loop closure.
// 是MapBuilderInterface的子类(核心类)
// 涵盖整个salm过程

/** MapBuilder
 * @brief 这是整个cartographer C++库最顶层的封装
 * @brief 继承自接口类  `MapBuilderInterface`
 * @brief 这个类里面有两个重要的成员 : pose_graph_ 和 trajectory_builders_
 * @param class <CollatedTrajectoryBuilder> trajectory_builders_ :  []
 *          包含了: 1. 前端核心:LocalTrajectoryBuilder2D
 *                 2. 后端核心:PoseGraph2D
 * @param pose_graph_ : 全局闭环优化
 * @param thread_pool_: 多线程管理
 **/
class MapBuilder : public MapBuilderInterface {
public:
    // 构造函数
    explicit MapBuilder(const proto::MapBuilderOptions &options);
    ~MapBuilder() override {}

    MapBuilder(const MapBuilder &) = delete;
    MapBuilder &operator=(const MapBuilder &) = delete;

    /// 下面几个函数重载父类的虚函数

    // 增加一个TrajectoryBuilder
    int AddTrajectoryBuilder(
            const std::set<SensorId> &expected_sensor_ids,
            const proto::TrajectoryBuilderOptions &trajectory_options,
            LocalSlamResultCallback local_slam_result_callback) override;

    // 结束轨迹
    void FinishTrajectory(int trajectory_id) override;

    //(非重点)从proto读取
    int AddTrajectoryForDeserialization(
            const proto::TrajectoryBuilderOptionsWithSensorIds
            &options_with_sensor_ids_proto) override;

    //(非重点)子图转proto
    std::string SubmapToProto(const SubmapId &submap_id,
                              proto::SubmapQuery::Response *response) override;

    //(非重点)序列化输出
    void SerializeState(io::ProtoStreamWriterInterface *writer) override;
    //(非重点)从proto读取状态
    void LoadState(io::ProtoStreamReaderInterface *reader,
                   bool load_frozen_state) override;

    // 返回位姿图
    mapping::PoseGraphInterface *pose_graph() override {
        return pose_graph_.get();
    }

    // 返回trajectory_builders的数量
    int num_trajectory_builders() const override {
        return trajectory_builders_.size();
    }

    // 给定轨迹id, 返回该轨迹的trajectory_builder
    mapping::TrajectoryBuilderInterface *GetTrajectoryBuilder(
            int trajectory_id) const override {
        return trajectory_builders_.at(trajectory_id).get();
    }

    // 返回配置选项
    const std::vector<proto::TrajectoryBuilderOptionsWithSensorIds>
    &GetAllTrajectoryBuilderOptions() const override {
        return all_trajectory_builder_options_;
    }

private:
    //配置选项
    const proto::MapBuilderOptions options_;                            //顺序不能换，必须放第一
    //线程池
    common::ThreadPool thread_pool_;

    //传感器收集器
    std::unique_ptr<sensor::CollatorInterface> sensor_collator_;

    //轨迹构建器: TrajectoryBuilderInterface是基类
    //实际上用的是子类: CollatedTrajectoryBuilder
    std::vector<std::unique_ptr<mapping::TrajectoryBuilderInterface>>
    trajectory_builders_;

    //位姿图
    std::unique_ptr<PoseGraph> pose_graph_;

    //轨迹构建器的配置选项
    std::vector<proto::TrajectoryBuilderOptionsWithSensorIds>
    all_trajectory_builder_options_;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_MAP_BUILDER_H_
