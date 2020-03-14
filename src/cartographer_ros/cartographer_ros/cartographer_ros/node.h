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

#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_NODE_H
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_NODE_H

#include <map>
#include <memory>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "cartographer/common/fixed_ratio_sampler.h"
#include "cartographer/common/mutex.h"
#include "cartographer/mapping/map_builder_interface.h"
#include "cartographer/mapping/pose_extrapolator.h"
#include "cartographer_ros/map_builder_bridge.h"
#include "cartographer_ros/node_constants.h"
#include "cartographer_ros/node_options.h"
#include "cartographer_ros/trajectory_options.h"
#include "cartographer_ros_msgs/FinishTrajectory.h"
#include "cartographer_ros_msgs/SensorTopics.h"
#include "cartographer_ros_msgs/StartTrajectory.h"
#include "cartographer_ros_msgs/StatusResponse.h"
#include "cartographer_ros_msgs/SubmapEntry.h"
#include "cartographer_ros_msgs/SubmapList.h"
#include "cartographer_ros_msgs/SubmapQuery.h"
#include "cartographer_ros_msgs/TrajectoryOptions.h"
#include "cartographer_ros_msgs/WriteState.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/MultiEchoLaserScan.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf2_ros/transform_broadcaster.h"

namespace cartographer_ros {

// Wires up ROS topics to SLAM.
class Node {
public:
    //构造函数， 传入 节点配置，map_builder指针，tf变换的缓冲区tf_buffer
    Node(const NodeOptions& node_options,
         std::unique_ptr<cartographer::mapping::MapBuilderInterface> map_builder,
         tf2_ros::Buffer* tf_buffer);
    ~Node();

    Node(const Node&) = delete;
    Node& operator=(const Node&) = delete;

    // Finishes all yet active trajectories.
    // 结束现在正在活动的轨迹
    void FinishAllTrajectories();
    // Finishes a single given trajectory. Returns false if the trajectory did not
    // exist or was already finished.
    // 完成给定id的轨迹，如果轨迹已经完成或者不存在，返回false
    bool FinishTrajectory(int trajectory_id);

    // Runs final optimization. All trajectories have to be finished when calling.
    // 调用最后的BA (通过map_builder_bridge_来调用)
    void RunFinalOptimization();

    // Starts the first trajectory with the default topics.
    // 使用默认的Topic开始第一条轨迹
    void StartTrajectoryWithDefaultTopics(const TrajectoryOptions& options);

    // Returns unique SensorIds for multiple input bag files based on
    // their TrajectoryOptions.
    // 'SensorId::id' is the expected ROS topic name.
    std::vector<
    std::set<::cartographer::mapping::TrajectoryBuilderInterface::SensorId>>
    ComputeDefaultSensorIdsForMultipleBags(
            const std::vector<TrajectoryOptions>& bags_options) const;

    // Adds a trajectory for offline processing, i.e. not listening to topics.
    // 添加离线轨迹(相当于直接解包)
    int AddOfflineTrajectory(
            const std::set<
            cartographer::mapping::TrajectoryBuilderInterface::SensorId>&
            expected_sensor_ids,
            const TrajectoryOptions& options);

    // The following functions handle adding sensor data to a trajectory.
    // 接下来的几个函数是回调函数,处理传感器数据,然后加到trajectory
    void HandleOdometryMessage(int trajectory_id, const std::string& sensor_id,
                               const nav_msgs::Odometry::ConstPtr& msg);
    void HandleNavSatFixMessage(int trajectory_id, const std::string& sensor_id,
                                const sensor_msgs::NavSatFix::ConstPtr& msg);
    void HandleLandmarkMessage(
            int trajectory_id, const std::string& sensor_id,
            const cartographer_ros_msgs::LandmarkList::ConstPtr& msg);
    void HandleImuMessage(int trajectory_id, const std::string& sensor_id,
                          const sensor_msgs::Imu::ConstPtr& msg);
    void HandleLaserScanMessage(int trajectory_id, const std::string& sensor_id,
                                const sensor_msgs::LaserScan::ConstPtr& msg);
    void HandleMultiEchoLaserScanMessage(
            int trajectory_id, const std::string& sensor_id,
            const sensor_msgs::MultiEchoLaserScan::ConstPtr& msg);
    void HandlePointCloud2Message(int trajectory_id, const std::string& sensor_id,
                                  const sensor_msgs::PointCloud2::ConstPtr& msg);

    // Serializes the complete Node state.
    // 序列化, 调用map_builder_bridge_ 实现, 将数据储存到filename
    void SerializeState(const std::string& filename);

    // Loads a serialized SLAM state from a .pbstream file.
    // 读取一个经过序列化生成的SLAM状态, 如一个`.pbstream`文件
    void LoadState(const std::string& state_filename, bool load_frozen_state);

    // ROS的节点类 , 返回private对象
    ::ros::NodeHandle* node_handle();

private:
    // 订阅器结构体, 包括ros的订阅器对象和topic
    struct Subscriber {
        ::ros::Subscriber subscriber;

        // ::ros::Subscriber::getTopic() does not necessarily return the same
        // std::string
        // it was given in its constructor. Since we rely on the topic name as the
        // unique identifier of a subscriber, we remember it ourselves.
        std::string topic;
    };

    // 下面几个函数是 服务的回调

    // 1. 查询子图服务回调
    bool HandleSubmapQuery(
            cartographer_ros_msgs::SubmapQuery::Request& request,
            cartographer_ros_msgs::SubmapQuery::Response& response);
    // 2. 开始轨迹服务回调
    bool HandleStartTrajectory(
            cartographer_ros_msgs::StartTrajectory::Request& request,
            cartographer_ros_msgs::StartTrajectory::Response& response);
    // 3. 结束轨迹回调
    bool HandleFinishTrajectory(
            cartographer_ros_msgs::FinishTrajectory::Request& request,
            cartographer_ros_msgs::FinishTrajectory::Response& response);
    // 4. 记录状态回调
    bool HandleWriteState(cartographer_ros_msgs::WriteState::Request& request,
                          cartographer_ros_msgs::WriteState::Response& response);

    // Returns the set of SensorIds expected for a trajectory.
    // 'SensorId::id' is the expected ROS topic name.
    // 返回某个轨迹的传感器ID的集合 , SensorId::id也就是topic
    std::set<::cartographer::mapping::TrajectoryBuilderInterface::SensorId>
    ComputeExpectedSensorIds(
            const TrajectoryOptions& options,
            const cartographer_ros_msgs::SensorTopics& topics) const;
    // 调用map_builder_bridge_ ,传入 预期传感器id以及 选项 ,添加轨迹
    int AddTrajectory(const TrajectoryOptions& options,
                      const cartographer_ros_msgs::SensorTopics& topics);
    // 启动订阅, 调用ROS的节点句柄_node_handle订阅各个传感器的topic
    void LaunchSubscribers(const TrajectoryOptions& options,
                           const cartographer_ros_msgs::SensorTopics& topics,
                           int trajectory_id);
    // 子图发布器, 获取子图列表并发布
    void PublishSubmapList(const ::ros::WallTimerEvent& timer_event);
    // 给某个轨迹添加一个姿态外推器(航迹推算器)
    void AddExtrapolator(int trajectory_id, const TrajectoryOptions& options);
    // 根据options设置某个轨迹的sensor_samplers_参数
    void AddSensorSamplers(int trajectory_id, const TrajectoryOptions& options);

    /// 下面是几个发布器发布数据的函数
    // 发布轨迹状态?
    void PublishTrajectoryStates(const ::ros::WallTimerEvent& timer_event);
    // 发布轨迹节点列表
    void PublishTrajectoryNodeList(const ::ros::WallTimerEvent& timer_event);
    // 发布路标Pose列表
    void PublishLandmarkPosesList(const ::ros::WallTimerEvent& timer_event);
    // 发布约束列表
    void PublishConstraintList(const ::ros::WallTimerEvent& timer_event);
    void SpinOccupancyGridThreadForever();  //空的,没用到?
    // 根据标志位判断是2D模式还是3D模式, 返回轨迹选项参数
    bool ValidateTrajectoryOptions(const TrajectoryOptions& options);
    // 检查topic是否被占用
    bool ValidateTopicNames(const ::cartographer_ros_msgs::SensorTopics& topics,
                            const TrajectoryOptions& options);

    // 检查轨迹可否被结束,如果可以,则结束轨迹
    cartographer_ros_msgs::StatusResponse FinishTrajectoryUnderLock(
            int trajectory_id) REQUIRES(mutex_);

    // 节点参数, 构造函数的时候被赋值, 也就是由node_main.cc传值进来
    const NodeOptions node_options_;

    // 封装的线程锁
    cartographer::common::Mutex mutex_;

    ///[important]
    // map_builder_bridge_ : ros与cartographer c++核心库交互的接口
    MapBuilderBridge map_builder_bridge_ GUARDED_BY(mutex_);

    /// ros相关
    ::ros::NodeHandle node_handle_;                     //节点句柄
    ::ros::Publisher submap_list_publisher_;            //子图列表发布器
    ::ros::Publisher trajectory_node_list_publisher_;   //轨迹节点列表发布器
    ::ros::Publisher landmark_poses_list_publisher_;    //路标位姿列表发布器
    ::ros::Publisher constraint_list_publisher_;        //约束列表发布器

    // These ros::ServiceServers need to live for the lifetime of the node.
    // 下面的::ros::ServiceServer需要一直存在
    std::vector<::ros::ServiceServer> service_servers_;
    // scan_matched点云发布器
    ::ros::Publisher scan_matched_point_cloud_publisher_;
    // TF变换发布器
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    // 一条轨迹上的传感器采样器
    struct TrajectorySensorSamplers {
        // 这个结构体相当于为轨迹上的传感器都创建一个采样器
        // 采样时间到,就从数据流中获取数据

        // 构造函数, 相当于初始化下面的各种传感器的采样器
        TrajectorySensorSamplers(const double rangefinder_sampling_ratio,
                                 const double odometry_sampling_ratio,
                                 const double fixed_frame_pose_sampling_ratio,
                                 const double imu_sampling_ratio,
                                 const double landmark_sampling_ratio)
            : rangefinder_sampler(rangefinder_sampling_ratio),
              odometry_sampler(odometry_sampling_ratio),
              fixed_frame_pose_sampler(fixed_frame_pose_sampling_ratio),
              imu_sampler(imu_sampling_ratio),
              landmark_sampler(landmark_sampling_ratio) {}

        // 固定频率的采样器
        ::cartographer::common::FixedRatioSampler rangefinder_sampler;      //激光雷达
        ::cartographer::common::FixedRatioSampler odometry_sampler;         //里程计
        ::cartographer::common::FixedRatioSampler fixed_frame_pose_sampler; //gps
        ::cartographer::common::FixedRatioSampler imu_sampler;              //imu
        ::cartographer::common::FixedRatioSampler landmark_sampler;         //路标
    };

    ///下面的都是每条轨迹对应的
    // These are keyed with 'trajectory_id'.
    // 下面的映射 <轨迹id, 映射内容>
    //姿态外推器列表
    std::map<int, ::cartographer::mapping::PoseExtrapolator> extrapolators_;
    //传感器数据采样器
    std::unordered_map<int, TrajectorySensorSamplers> sensor_samplers_;
    //订阅器
    std::unordered_map<int, std::vector<Subscriber>> subscribers_;
    //订阅topic列表
    std::unordered_set<std::string> subscribed_topics_;
    //正在运行的轨迹列表
    std::unordered_map<int, bool> is_active_trajectory_ GUARDED_BY(mutex_);

    // We have to keep the timer handles of ::ros::WallTimers around, otherwise
    // they do not fire.
    // 必须保留::ros:: walltimer的定时器句柄，否则它们不会触发
    std::vector<::ros::WallTimer> wall_timers_;
};

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_NODE_H
