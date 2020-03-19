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

#include "cartographer/mapping/map_builder.h"
#include "cartographer_ros/node.h"
#include "cartographer_ros/node_options.h"
#include "cartographer_ros/ros_log_sink.h"
#include "gflags/gflags.h"
#include "tf2_ros/transform_listener.h"

DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
DEFINE_string(configuration_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");
DEFINE_string(load_state_filename, "",
              "If non-empty, filename of a .pbstream file to load, containing "
              "a saved SLAM state.");
DEFINE_bool(load_frozen_state, true,
            "Load the saved state as frozen (non-optimized) trajectories.");
DEFINE_bool(
        start_trajectory_with_default_topics, true,
        "Enable to immediately start the first trajectory with default topics.");
DEFINE_string(
        save_state_filename, "",
        "If non-empty, serialize state and write it to disk before shutting down.");

namespace cartographer_ros {
namespace {

void Run() {
    // 初始化tf2 buffer
    constexpr double kTfBufferCacheTimeInSeconds = 10.;
    tf2_ros::Buffer tf_buffer{::ros::Duration(kTfBufferCacheTimeInSeconds)};
    // 创建tf变换监听器
    tf2_ros::TransformListener tf(tf_buffer);
    // 读取配置文件
    NodeOptions node_options;
    TrajectoryOptions trajectory_options;
    std::tie(node_options, trajectory_options) =
            LoadOptions(FLAGS_configuration_directory, FLAGS_configuration_basename);
    // 创建MapBuilder对象，返回对象指针，给map_builder
    auto map_builder =
            cartographer::common::make_unique<cartographer::mapping::MapBuilder>(
                node_options.map_builder_options);
    // 构造的Node节点
    Node node(node_options, std::move(map_builder), &tf_buffer);

    // 从文件读取状态(如果标志位存在)
    if (!FLAGS_load_state_filename.empty()) {
        node.LoadState(FLAGS_load_state_filename, FLAGS_load_frozen_state);
    }

    // node :　使用默认的topic开始一条轨迹
    if (FLAGS_start_trajectory_with_default_topics) {
        node.StartTrajectoryWithDefaultTopics(trajectory_options);
    }

    // ros spin等待
    ::ros::spin();

    //结束轨迹
    node.FinishAllTrajectories();
    //调用最后一次BA优化
    node.RunFinalOptimization();

    //根据标志是否进行序列化
    if (!FLAGS_save_state_filename.empty()) {
        node.SerializeState(FLAGS_save_state_filename);
    }
}

}  // namespace
}  // namespace cartographer_ros

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);

    CHECK(!FLAGS_configuration_directory.empty())
            << "-configuration_directory is missing.";
    CHECK(!FLAGS_configuration_basename.empty())
            << "-configuration_basename is missing.";

    ::ros::init(argc, argv, "cartographer_node");
    ::ros::start();

    cartographer_ros::ScopedRosLogSink ros_log_sink;  // 使得Google 的日志输出变成使用ROS 日志输出
    cartographer_ros::Run();
    ::ros::shutdown();
}
