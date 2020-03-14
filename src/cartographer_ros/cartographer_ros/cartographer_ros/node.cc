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

#include "cartographer_ros/node.h"

#include <chrono>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/port.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/proto/submap_visualization.pb.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/sensor_bridge.h"
#include "cartographer_ros/tf_bridge.h"
#include "cartographer_ros/time_conversion.h"
#include "cartographer_ros_msgs/StatusCode.h"
#include "cartographer_ros_msgs/StatusResponse.h"
#include "glog/logging.h"
#include "nav_msgs/Odometry.h"
#include "ros/serialization.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf2_eigen/tf2_eigen.h"
#include "visualization_msgs/MarkerArray.h"

namespace cartographer_ros {

namespace {

// 获取默认的传感器topic
cartographer_ros_msgs::SensorTopics DefaultSensorTopics() {
    cartographer_ros_msgs::SensorTopics topics;
    topics.laser_scan_topic = kLaserScanTopic;
    topics.multi_echo_laser_scan_topic = kMultiEchoLaserScanTopic;
    topics.point_cloud2_topic = kPointCloud2Topic;
    topics.imu_topic = kImuTopic;
    topics.odometry_topic = kOdometryTopic;
    topics.nav_sat_fix_topic = kNavSatFixTopic;
    topics.landmark_topic = kLandmarkTopic;
    return topics;
}

// Subscribes to the 'topic' for 'trajectory_id' using the 'node_handle' and
// calls 'handler' on the 'node' to handle messages. Returns the subscriber.
// 调用ROS的节点句柄_node_handle订阅topic, 传入参数如下
// (订阅的回调函数, 轨迹id , topic , ROS的节点句柄_node_handle )
template <typename MessageType>
::ros::Subscriber SubscribeWithHandler(
        void (Node::*handler)(int, const std::string&,
                              const typename MessageType::ConstPtr&),   //回调函数
                              const int trajectory_id,                  //轨迹ID
                              const std::string& topic,                 //topic
        ::ros::NodeHandle* const node_handle, Node* const node) {       //ROS的节点句柄_node_handle

    // 调用ROS的节点句柄_node_handle订阅topic, 传入给定的回调函数
    return node_handle->subscribe<MessageType>(
                topic, kInfiniteSubscriberQueueSize,
                boost::function<void(const typename MessageType::ConstPtr&)>(
                    [node, handler, trajectory_id,topic]
                    (const typename MessageType::ConstPtr& msg) {
        (node->*handler)(trajectory_id, topic, msg);
    }));
}

}  // namespace

namespace carto = ::cartographer;

using carto::transform::Rigid3d;

Node::Node(
        const NodeOptions& node_options,
        std::unique_ptr<cartographer::mapping::MapBuilderInterface> map_builder,
        tf2_ros::Buffer* const tf_buffer)
    : node_options_(node_options),
      map_builder_bridge_(node_options_, std::move(map_builder), tf_buffer) {   //将三个输入又传给了map_builder_bridge_
    //线程锁
    carto::common::MutexLocker lock(&mutex_);
    // 一些发布器
    submap_list_publisher_ =
            node_handle_.advertise<::cartographer_ros_msgs::SubmapList>(
                kSubmapListTopic, kLatestOnlyPublisherQueueSize);
    trajectory_node_list_publisher_ =
            node_handle_.advertise<::visualization_msgs::MarkerArray>(
                kTrajectoryNodeListTopic, kLatestOnlyPublisherQueueSize);
    landmark_poses_list_publisher_ =
            node_handle_.advertise<::visualization_msgs::MarkerArray>(
                kLandmarkPosesListTopic, kLatestOnlyPublisherQueueSize);
    constraint_list_publisher_ =
            node_handle_.advertise<::visualization_msgs::MarkerArray>(
                kConstraintListTopic, kLatestOnlyPublisherQueueSize);
    // 一些服务
    service_servers_.push_back(node_handle_.advertiseService(
                                   kSubmapQueryServiceName, &Node::HandleSubmapQuery, this));
    service_servers_.push_back(node_handle_.advertiseService(
                                   kStartTrajectoryServiceName, &Node::HandleStartTrajectory, this));
    service_servers_.push_back(node_handle_.advertiseService(
                                   kFinishTrajectoryServiceName, &Node::HandleFinishTrajectory, this));
    service_servers_.push_back(node_handle_.advertiseService(
                                   kWriteStateServiceName, &Node::HandleWriteState, this));

    //scan_matched点云发布器
    scan_matched_point_cloud_publisher_ =
            node_handle_.advertise<sensor_msgs::PointCloud2>(
                kScanMatchedPointCloudTopic, kLatestOnlyPublisherQueueSize);

    wall_timers_.push_back(node_handle_.createWallTimer(
                               ::ros::WallDuration(node_options_.submap_publish_period_sec),
                               &Node::PublishSubmapList, this));
    wall_timers_.push_back(node_handle_.createWallTimer(
                               ::ros::WallDuration(node_options_.pose_publish_period_sec),
                               &Node::PublishTrajectoryStates, this));
    wall_timers_.push_back(node_handle_.createWallTimer(
                               ::ros::WallDuration(node_options_.trajectory_publish_period_sec),
                               &Node::PublishTrajectoryNodeList, this));
    wall_timers_.push_back(node_handle_.createWallTimer(
                               ::ros::WallDuration(node_options_.trajectory_publish_period_sec),
                               &Node::PublishLandmarkPosesList, this));
    wall_timers_.push_back(node_handle_.createWallTimer(
                               ::ros::WallDuration(kConstraintPublishPeriodSec),
                               &Node::PublishConstraintList, this));
}

Node::~Node() { FinishAllTrajectories(); }

// 返回private对象
::ros::NodeHandle* Node::node_handle() { return &node_handle_; }

// 调用map_builder_bridge_,处理 查询子图的请求 , 这是个回调函数
bool Node::HandleSubmapQuery(
        ::cartographer_ros_msgs::SubmapQuery::Request& request,
        ::cartographer_ros_msgs::SubmapQuery::Response& response) {
    carto::common::MutexLocker lock(&mutex_);
    map_builder_bridge_.HandleSubmapQuery(request, response);
    return true;
}

// 子图发布器, 获取子图列表并发布
void Node::PublishSubmapList(const ::ros::WallTimerEvent& unused_timer_event) {
    carto::common::MutexLocker lock(&mutex_);
    submap_list_publisher_.publish(map_builder_bridge_.GetSubmapList());
}
// 为对应轨迹添加姿态外推器, 航迹推算器
void Node::AddExtrapolator(const int trajectory_id,
                           const TrajectoryOptions& options) {
    //
    constexpr double kExtrapolationEstimationTimeSec = 0.001;  // 1 ms
    // 确保这个id的轨迹还没有姿态外推器, 航迹推算器
    CHECK(extrapolators_.count(trajectory_id) == 0);
    // 判断是3DM模式还是2D模式,从配置选项中选择对应的 重力加速度常数
    const double gravity_time_constant =
            node_options_.map_builder_options.use_trajectory_builder_3d()
            ? options.trajectory_builder_options.trajectory_builder_3d_options()
              .imu_gravity_time_constant()
            : options.trajectory_builder_options.trajectory_builder_2d_options()
              .imu_gravity_time_constant();
    // 构造一个::cartographer::mapping::PoseExtrapolator(推算时间间隔, 重力常数)
    // 然后构成pair<轨迹id , 姿态外推器>,存到extrapolators_
    extrapolators_.emplace(
                std::piecewise_construct, std::forward_as_tuple(trajectory_id),
                std::forward_as_tuple(
                    ::cartographer::common::FromSeconds(kExtrapolationEstimationTimeSec),
                    gravity_time_constant));
}

// 根据options设置某个轨迹的sensor_samplers_参数
void Node::AddSensorSamplers(const int trajectory_id,
                             const TrajectoryOptions& options) {
    // 检查sensor_samplers_ 是否有 这个id的轨迹
    CHECK(sensor_samplers_.count(trajectory_id) == 0);
    // make_tuple: 对于多个元素的映射关系时,使用make_tuple进行打包
    // std::piecewise_construct: 结合使用make_tuple进行打包用于解决歧义,
    // <key, 数据 >= <trajectory_id , (下面的5个参数)>
    sensor_samplers_.emplace(
                std::piecewise_construct, std::forward_as_tuple(trajectory_id),
                std::forward_as_tuple(
                    options.rangefinder_sampling_ratio, options.odometry_sampling_ratio,
                    options.fixed_frame_pose_sampling_ratio, options.imu_sampling_ratio,
                    options.landmarks_sampling_ratio));
}

void Node::PublishTrajectoryStates(const ::ros::WallTimerEvent& timer_event) {
    carto::common::MutexLocker lock(&mutex_);
    for (const auto& entry : map_builder_bridge_.GetTrajectoryStates()) {
        const auto& trajectory_state = entry.second;

        auto& extrapolator = extrapolators_.at(entry.first);
        // We only publish a point cloud if it has changed. It is not needed at high
        // frequency, and republishing it would be computationally wasteful.
        if (trajectory_state.local_slam_data->time !=
                extrapolator.GetLastPoseTime()) {
            if (scan_matched_point_cloud_publisher_.getNumSubscribers() > 0) {
                // TODO(gaschler): Consider using other message without time
                // information.
                carto::sensor::TimedPointCloud point_cloud;
                point_cloud.reserve(trajectory_state.local_slam_data
                                    ->range_data_in_local.returns.size());
                for (const Eigen::Vector3f point :
                     trajectory_state.local_slam_data->range_data_in_local.returns) {
                    Eigen::Vector4f point_time;
                    point_time << point, 0.f;
                    point_cloud.push_back(point_time);
                }
                scan_matched_point_cloud_publisher_.publish(ToPointCloud2Message(
                                                                carto::common::ToUniversal(trajectory_state.local_slam_data->time),
                                                                node_options_.map_frame,
                                                                carto::sensor::TransformTimedPointCloud(
                                                                    point_cloud, trajectory_state.local_to_map.cast<float>())));
            }
            extrapolator.AddPose(trajectory_state.local_slam_data->time,
                                 trajectory_state.local_slam_data->local_pose);
        }

        geometry_msgs::TransformStamped stamped_transform;
        // If we do not publish a new point cloud, we still allow time of the
        // published poses to advance. If we already know a newer pose, we use its
        // time instead. Since tf knows how to interpolate, providing newer
        // information is better.
        const ::cartographer::common::Time now = std::max(
                    FromRos(ros::Time::now()), extrapolator.GetLastExtrapolatedTime());
        stamped_transform.header.stamp = ToRos(now);

        const Rigid3d tracking_to_local = [&] {
            if (trajectory_state.trajectory_options.publish_frame_projected_to_2d) {
                return carto::transform::Embed3D(
                            carto::transform::Project2D(extrapolator.ExtrapolatePose(now)));
            }
            return extrapolator.ExtrapolatePose(now);
        }();

        const Rigid3d tracking_to_map =
                trajectory_state.local_to_map * tracking_to_local;

        if (trajectory_state.published_to_tracking != nullptr) {
            if (trajectory_state.trajectory_options.provide_odom_frame) {
                std::vector<geometry_msgs::TransformStamped> stamped_transforms;

                stamped_transform.header.frame_id = node_options_.map_frame;
                stamped_transform.child_frame_id =
                        trajectory_state.trajectory_options.odom_frame;
                stamped_transform.transform =
                        ToGeometryMsgTransform(trajectory_state.local_to_map);
                stamped_transforms.push_back(stamped_transform);

                stamped_transform.header.frame_id =
                        trajectory_state.trajectory_options.odom_frame;
                stamped_transform.child_frame_id =
                        trajectory_state.trajectory_options.published_frame;
                stamped_transform.transform = ToGeometryMsgTransform(
                            tracking_to_local * (*trajectory_state.published_to_tracking));
                stamped_transforms.push_back(stamped_transform);

                tf_broadcaster_.sendTransform(stamped_transforms);
            } else {
                stamped_transform.header.frame_id = node_options_.map_frame;
                stamped_transform.child_frame_id =
                        trajectory_state.trajectory_options.published_frame;
                stamped_transform.transform = ToGeometryMsgTransform(
                            tracking_to_map * (*trajectory_state.published_to_tracking));
                tf_broadcaster_.sendTransform(stamped_transform);
            }
        }
    }
}

// 发布轨迹节点列表
void Node::PublishTrajectoryNodeList(
        const ::ros::WallTimerEvent& unused_timer_event) {
    if (trajectory_node_list_publisher_.getNumSubscribers() > 0) {
        carto::common::MutexLocker lock(&mutex_);
        trajectory_node_list_publisher_.publish(
                    map_builder_bridge_.GetTrajectoryNodeList());
    }
}

// 发布路标Pose列表
void Node::PublishLandmarkPosesList(
        const ::ros::WallTimerEvent& unused_timer_event) {
    if (landmark_poses_list_publisher_.getNumSubscribers() > 0) {
        carto::common::MutexLocker lock(&mutex_);
        landmark_poses_list_publisher_.publish(
                    map_builder_bridge_.GetLandmarkPosesList());
    }
}

// 发布约束列表
void Node::PublishConstraintList(
        const ::ros::WallTimerEvent& unused_timer_event) {
    if (constraint_list_publisher_.getNumSubscribers() > 0) {
        carto::common::MutexLocker lock(&mutex_);
        constraint_list_publisher_.publish(map_builder_bridge_.GetConstraintList());
    }
}
// 根据轨迹选项(订阅了哪些topic),返回预期的传感器id
std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>
Node::ComputeExpectedSensorIds(
        const TrajectoryOptions& options,
        const cartographer_ros_msgs::SensorTopics& topics) const {
    using SensorId = cartographer::mapping::TrajectoryBuilderInterface::SensorId;
    using SensorType = SensorId::SensorType;

    // 根据选项,储存预期的topic
    std::set<SensorId> expected_topics;
    // Subscribe to all laser scan, multi echo laser scan, and point cloud topics.
    // 订阅所有激光扫描topic
    for (const std::string& topic : ComputeRepeatedTopicNames(
             topics.laser_scan_topic, options.num_laser_scans)) {
        expected_topics.insert(SensorId{SensorType::RANGE, topic});
    }
    for (const std::string& topic :
         ComputeRepeatedTopicNames(topics.multi_echo_laser_scan_topic,
                                   options.num_multi_echo_laser_scans)) {
        expected_topics.insert(SensorId{SensorType::RANGE, topic});
    }
    for (const std::string& topic : ComputeRepeatedTopicNames(
             topics.point_cloud2_topic, options.num_point_clouds)) {
        expected_topics.insert(SensorId{SensorType::RANGE, topic});
    }
    // For 2D SLAM, subscribe to the IMU if we expect it. For 3D SLAM, the IMU is
    // required.
    // 对于2D SLAM, 如果需要IMU,才进行订阅, 对于3D, 则IMU必须有
    if (node_options_.map_builder_options.use_trajectory_builder_3d() ||
            (node_options_.map_builder_options.use_trajectory_builder_2d() &&
             options.trajectory_builder_options.trajectory_builder_2d_options()
             .use_imu_data())) {
        expected_topics.insert(SensorId{SensorType::IMU, topics.imu_topic});
    }
    // Odometry is optional.
    // 里程计是可选的topic
    if (options.use_odometry) {
        expected_topics.insert(
                    SensorId{SensorType::ODOMETRY, topics.odometry_topic});
    }
    // NavSatFix is optional.
    // gps 可选
    if (options.use_nav_sat) {
        expected_topics.insert(
                    SensorId{SensorType::FIXED_FRAME_POSE, topics.nav_sat_fix_topic});
    }
    // Landmark is optional.
    // 路标topic,可选
    if (options.use_landmarks) {
        expected_topics.insert(SensorId{SensorType::LANDMARK, kLandmarkTopic});
    }
    return expected_topics;
}

// 调用map_builder_bridge_ ,传入 预期传感器id以及 选项 ,添加轨迹
int Node::AddTrajectory(const TrajectoryOptions& options,
                        const cartographer_ros_msgs::SensorTopics& topics) {
    // ComputeExpectedSensorIds(): 根据轨迹选项(订阅了哪些topic),返回预期的传感器id和topic
    const std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>
            expected_sensor_ids = ComputeExpectedSensorIds(options, topics);
    // 调用map_builder_bridge_ ,传入 预期传感器id以及 选项 ,添加轨迹
    // 返回轨迹id
    const int trajectory_id =
            map_builder_bridge_.AddTrajectory(expected_sensor_ids, options);
    // 为对应轨迹添加姿态外推器, 航迹推算器
    AddExtrapolator(trajectory_id, options);
    // 根据options设置某个轨迹的sensor_samplers_参数
    AddSensorSamplers(trajectory_id, options);
    // 加载订阅器,订阅传感器topic,并设置相关回调
    LaunchSubscribers(options, topics, trajectory_id);
    // 轨迹激活标志位设置
    is_active_trajectory_[trajectory_id] = true;
    // 遍历预期的传感器id和topic
    // 将订阅过的topic记录下来
    for (const auto& sensor_id : expected_sensor_ids) {
        subscribed_topics_.insert(sensor_id.id);
    }
    return trajectory_id;
}

// 启动订阅, 调用ROS的节点句柄_node_handle订阅各个传感器的topic
void Node::LaunchSubscribers(const TrajectoryOptions& options,
                             const cartographer_ros_msgs::SensorTopics& topics,
                             const int trajectory_id) {
    //ComputeRepeatedTopicNames (topic字符串,传感器数量)
    // 根据传感器数量,返回预期的topic字符串, 如果只有1个传感器,那么就返回原来的topic字符串, 否则, 就在原来的字符串加上编号,得到新的topic
    for (const std::string& topic : ComputeRepeatedTopicNames(
             topics.laser_scan_topic, options.num_laser_scans)) {

        // SubscribeWithHandler: 调用ROS的节点句柄_node_handle订阅topic, 传入参数如下
        // (订阅的回调函数, 轨迹id , topic , ROS的节点句柄_node_handle )
        // 返回对应的订阅器, 存到subscribers_
        subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<sensor_msgs::LaserScan>(
         &Node::HandleLaserScanMessage, trajectory_id, topic, &node_handle_,
         this),
         topic});
    }
    for (const std::string& topic :
         ComputeRepeatedTopicNames(topics.multi_echo_laser_scan_topic,
                                   options.num_multi_echo_laser_scans)) {
        subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<sensor_msgs::MultiEchoLaserScan>(
         &Node::HandleMultiEchoLaserScanMessage, trajectory_id, topic,
         &node_handle_, this),
         topic});
    }
    for (const std::string& topic : ComputeRepeatedTopicNames(
             topics.point_cloud2_topic, options.num_point_clouds)) {
        subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<sensor_msgs::PointCloud2>(
         &Node::HandlePointCloud2Message, trajectory_id, topic,
         &node_handle_, this),
         topic});
    }

    // For 2D SLAM, subscribe to the IMU if we expect it. For 3D SLAM, the IMU is
    // required.
    if (node_options_.map_builder_options.use_trajectory_builder_3d() ||
            (node_options_.map_builder_options.use_trajectory_builder_2d() &&
             options.trajectory_builder_options.trajectory_builder_2d_options()
             .use_imu_data())) {
        std::string topic = topics.imu_topic;
        subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<sensor_msgs::Imu>(&Node::HandleImuMessage,
         trajectory_id, topic,
         &node_handle_, this),
         topic});
    }

    if (options.use_odometry) {
        std::string topic = topics.odometry_topic;
        subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<nav_msgs::Odometry>(&Node::HandleOdometryMessage,
         trajectory_id, topic,
         &node_handle_, this),
         topic});
    }
    if (options.use_nav_sat) {
        std::string topic = topics.nav_sat_fix_topic;
        subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<sensor_msgs::NavSatFix>(
         &Node::HandleNavSatFixMessage, trajectory_id, topic, &node_handle_,
         this),
         topic});
    }
    if (options.use_landmarks) {
        std::string topic = topics.landmark_topic;
        subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<cartographer_ros_msgs::LandmarkList>(
         &Node::HandleLandmarkMessage, trajectory_id, topic, &node_handle_,
         this),
         topic});
    }
}
// 根据标志位判断是2D模式还是3D模式, 返回轨迹选项参数
bool Node::ValidateTrajectoryOptions(const TrajectoryOptions& options) {
    // 根据标志位判断是2D模式还是3D模式
    if (node_options_.map_builder_options.use_trajectory_builder_2d()) {
        // 返回对应的参数
        return options.trajectory_builder_options
                .has_trajectory_builder_2d_options();
    }
    if (node_options_.map_builder_options.use_trajectory_builder_3d()) {
        return options.trajectory_builder_options
                .has_trajectory_builder_3d_options();
    }
    return false;
}
// 检查topic是否被占用
bool Node::ValidateTopicNames(
        const ::cartographer_ros_msgs::SensorTopics& topics,
        const TrajectoryOptions& options) {
    // ComputeExpectedSensorIds: 根据轨迹选项(订阅了哪些topic),返回预期的传感器id(topic)
    for (const auto& sensor_id : ComputeExpectedSensorIds(options, topics)) {
        const std::string& topic = sensor_id.id;
        // 检查topic是否被占用
        if (subscribed_topics_.count(topic) > 0) {
            LOG(ERROR) << "Topic name [" << topic << "] is already used.";
            return false;
        }
    }
    return true;
}
// 检查轨迹可否被结束,如果可以,则结束轨迹
cartographer_ros_msgs::StatusResponse Node::FinishTrajectoryUnderLock(
        const int trajectory_id) {
    cartographer_ros_msgs::StatusResponse status_response;

    // First, check if we can actually finish the trajectory.
    // 首先检查是否真的可以结束这个id的轨迹
    if (map_builder_bridge_.GetFrozenTrajectoryIds().count(trajectory_id)) {
        // 轨迹被冻结
        const std::string error =
                "Trajectory " + std::to_string(trajectory_id) + " is frozen.";
        LOG(ERROR) << error;
        status_response.code = cartographer_ros_msgs::StatusCode::INVALID_ARGUMENT;
        status_response.message = error;
        return status_response;
    }
    if (is_active_trajectory_.count(trajectory_id) == 0) {
        // 如果这个id的轨迹不是在活动中, 则不需要结束
        const std::string error =
                "Trajectory " + std::to_string(trajectory_id) + " is not created yet.";
        LOG(ERROR) << error;
        status_response.code = cartographer_ros_msgs::StatusCode::NOT_FOUND;
        status_response.message = error;
        return status_response;
    }
    if (!is_active_trajectory_[trajectory_id]) {
        // 轨迹已经被结束了,则不需要重复结束
        const std::string error = "Trajectory " + std::to_string(trajectory_id) +
                " has already been finished.";
        LOG(ERROR) << error;
        status_response.code =
                cartographer_ros_msgs::StatusCode::RESOURCE_EXHAUSTED;
        status_response.message = error;
        return status_response;
    }

    // Shutdown the subscribers of this trajectory.
    // 首先停止订阅
    for (auto& entry : subscribers_[trajectory_id]) {
        entry.subscriber.shutdown();
        subscribed_topics_.erase(entry.topic);
        LOG(INFO) << "Shutdown the subscriber of [" << entry.topic << "]";
    }
    CHECK_EQ(subscribers_.erase(trajectory_id), 1);
    CHECK(is_active_trajectory_.at(trajectory_id));
    // 调用map_builder_bridge_ 来结束轨迹
    map_builder_bridge_.FinishTrajectory(trajectory_id);
    // 标志设置
    is_active_trajectory_[trajectory_id] = false;
    const std::string message =
            "Finished trajectory " + std::to_string(trajectory_id) + ".";
    status_response.code = cartographer_ros_msgs::StatusCode::OK;
    status_response.message = message;
    return status_response;
}

// ros服务的回调处理
// 开始轨迹
bool Node::HandleStartTrajectory(
        ::cartographer_ros_msgs::StartTrajectory::Request& request,
        ::cartographer_ros_msgs::StartTrajectory::Response& response) {
    carto::common::MutexLocker lock(&mutex_);
    TrajectoryOptions options;
    // 如果无法正常解析 或者 轨迹选项参数无效
    if (!FromRosMessage(request.options, &options) ||
            !ValidateTrajectoryOptions(options)) {
        const std::string error = "Invalid trajectory options.";
        LOG(ERROR) << error;
        response.status.code = cartographer_ros_msgs::StatusCode::INVALID_ARGUMENT;
        response.status.message = error;
    } else if (!ValidateTopicNames(request.topics, options)) {  // 检查topic是否被占用
        const std::string error = "Invalid topics.";
        LOG(ERROR) << error;
        response.status.code = cartographer_ros_msgs::StatusCode::INVALID_ARGUMENT;
        response.status.message = error;
    } else {
        // 检查通过 , 添加轨迹
        response.trajectory_id = AddTrajectory(options, request.topics);
        response.status.code = cartographer_ros_msgs::StatusCode::OK;
        response.status.message = "Success.";
    }
    return true;
}

// 使用默认topic开始轨迹
void Node::StartTrajectoryWithDefaultTopics(const TrajectoryOptions& options) {
    carto::common::MutexLocker lock(&mutex_);
    CHECK(ValidateTrajectoryOptions(options));  //确认轨迹选项
    AddTrajectory(options, DefaultSensorTopics());  // 添加轨迹
}

std::vector<
std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>>
Node::ComputeDefaultSensorIdsForMultipleBags(
        const std::vector<TrajectoryOptions>& bags_options) const {
    using SensorId = cartographer::mapping::TrajectoryBuilderInterface::SensorId;
    std::vector<std::set<SensorId>> bags_sensor_ids;
    for (size_t i = 0; i < bags_options.size(); ++i) {
        std::string prefix;
        if (bags_options.size() > 1) {
            prefix = "bag_" + std::to_string(i + 1) + "_";
        }
        std::set<SensorId> unique_sensor_ids;
        for (const auto& sensor_id :
             ComputeExpectedSensorIds(bags_options.at(i), DefaultSensorTopics())) {
            unique_sensor_ids.insert(SensorId{sensor_id.type, prefix + sensor_id.id});
        }
        bags_sensor_ids.push_back(unique_sensor_ids);
    }
    return bags_sensor_ids;
}

// 添加离线轨迹
int Node::AddOfflineTrajectory(
        const std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>&
        expected_sensor_ids,
        const TrajectoryOptions& options) {
    carto::common::MutexLocker lock(&mutex_);
    const int trajectory_id =
            map_builder_bridge_.AddTrajectory(expected_sensor_ids, options);
    AddExtrapolator(trajectory_id, options);
    AddSensorSamplers(trajectory_id, options);
    is_active_trajectory_[trajectory_id] = true;
    return trajectory_id;
}

// 服务: 结束轨迹, 回调
bool Node::HandleFinishTrajectory(
        ::cartographer_ros_msgs::FinishTrajectory::Request& request,
        ::cartographer_ros_msgs::FinishTrajectory::Response& response) {
    carto::common::MutexLocker lock(&mutex_);
    response.status = FinishTrajectoryUnderLock(request.trajectory_id);
    return true;
}

// 调用map_builder_bridge_ , 数据序列化,写入文件
bool Node::HandleWriteState(
        ::cartographer_ros_msgs::WriteState::Request& request,
        ::cartographer_ros_msgs::WriteState::Response& response) {
    carto::common::MutexLocker lock(&mutex_);
    if (map_builder_bridge_.SerializeState(request.filename)) {
        response.status.code = cartographer_ros_msgs::StatusCode::OK;
        response.status.message = "State written to '" + request.filename + "'.";
    } else {
        response.status.code = cartographer_ros_msgs::StatusCode::INVALID_ARGUMENT;
        response.status.message = "Failed to write '" + request.filename + "'.";
    }
    return true;
}

// 结束所有轨迹
void Node::FinishAllTrajectories() {
    carto::common::MutexLocker lock(&mutex_);
    // 遍历正在活动的轨迹
    for (auto& entry : is_active_trajectory_) {
        const int trajectory_id = entry.first;
        if (entry.second) {
            //CHECK_EQ: 相当于assert(val1 == val2)  release下可用,如果检测为true，则返回NULL，
            //否则就会返回一个有明确提示信息的字符串指针，并输出该信息，然后是程序宕掉
            CHECK_EQ(FinishTrajectoryUnderLock(trajectory_id).code,
                     cartographer_ros_msgs::StatusCode::OK);
        }
    }
}

// 结束指定id的轨迹
bool Node::FinishTrajectory(const int trajectory_id) {
    carto::common::MutexLocker lock(&mutex_);
    return FinishTrajectoryUnderLock(trajectory_id).code ==
            cartographer_ros_msgs::StatusCode::OK;
}

// 调用map_builder_bridge_,来实现最后的优化
void Node::RunFinalOptimization() {
    {
        carto::common::MutexLocker lock(&mutex_);
        // 检查轨迹是否都已经结束了
        for (const auto& entry : is_active_trajectory_) {
            CHECK(!entry.second);
        }
    }
    // Assuming we are not adding new data anymore, the final optimization
    // can be performed without holding the mutex.
    map_builder_bridge_.RunFinalOptimization();
}

// 回调: 处理里程计数据
void Node::HandleOdometryMessage(const int trajectory_id,
                                 const std::string& sensor_id,
                                 const nav_msgs::Odometry::ConstPtr& msg) {
    carto::common::MutexLocker lock(&mutex_);
    // 检查数据是否由采样器采样产生
    // 采样一次,Pluse返回true,表明数据确实由采样器采样得到的
    if (!sensor_samplers_.at(trajectory_id).odometry_sampler.Pulse()) {
        return;
    }
    // 调用map_builder_bridge_ ,获取该轨迹的sensor_bridge
    auto sensor_bridge_ptr = map_builder_bridge_.sensor_bridge(trajectory_id);
    // 将ros的里程计msg转换为自定义的carto::sensor::OdometryData, 并且
    // odometry_data_ptr 表示 tracking坐标系 到 里程计坐标系的变换 , 也就是tracking坐标系原点在 里程计坐标系的坐标
    auto odometry_data_ptr = sensor_bridge_ptr->ToOdometryData(msg);

    // extrapolators_ 姿态外推器,在对应轨迹上添加里程计的数据
    if (odometry_data_ptr != nullptr) {
        extrapolators_.at(trajectory_id).AddOdometryData(*odometry_data_ptr);   //还没进去看
    }
    // 调用sensor_bridge_ptr类中的成员trajectory_builder_,在对应轨迹添加里程计数据
    sensor_bridge_ptr->HandleOdometryMessage(sensor_id, msg);
}

// 回调: 处理GPS数据
void Node::HandleNavSatFixMessage(const int trajectory_id,
                                  const std::string& sensor_id,
                                  const sensor_msgs::NavSatFix::ConstPtr& msg) {
    carto::common::MutexLocker lock(&mutex_);
    // 检查数据是否由采样器采样产生
    if (!sensor_samplers_.at(trajectory_id).fixed_frame_pose_sampler.Pulse()) {
        return;
    }
    //调用map_builder_bridge_中对应轨迹的sensor_bridge, 来进行回调函数的调用
    map_builder_bridge_.sensor_bridge(trajectory_id)
            ->HandleNavSatFixMessage(sensor_id, msg);
}

// 回调:处理路标数据
void Node::HandleLandmarkMessage(
        const int trajectory_id, const std::string& sensor_id,
        const cartographer_ros_msgs::LandmarkList::ConstPtr& msg) {
    carto::common::MutexLocker lock(&mutex_);
    // 检查数据是否由采样器采样产生
    if (!sensor_samplers_.at(trajectory_id).landmark_sampler.Pulse()) {
        return;
    }
    //调用map_builder_bridge_ -> sensor_bridge -> trajectory_builder_ -->
    map_builder_bridge_.sensor_bridge(trajectory_id)
            ->HandleLandmarkMessage(sensor_id, msg);
}
// 回调:处理imu数据
void Node::HandleImuMessage(const int trajectory_id,
                            const std::string& sensor_id,
                            const sensor_msgs::Imu::ConstPtr& msg) {
    carto::common::MutexLocker lock(&mutex_);
    // 检查数据是否由采样器采样产生
    if (!sensor_samplers_.at(trajectory_id).imu_sampler.Pulse()) {
        return;
    }
    // 调用map_builder_bridge_ ,获取该轨迹的sensor_bridge
    auto sensor_bridge_ptr = map_builder_bridge_.sensor_bridge(trajectory_id);
    // 读取数据, 并返回转换到'tracking_frame_'坐标系之后的加速度和角速度
    auto imu_data_ptr = sensor_bridge_ptr->ToImuData(msg);
    if (imu_data_ptr != nullptr) {
        //调用姿态外推器(航迹推算), 添加IMU数据
        extrapolators_.at(trajectory_id).AddImuData(*imu_data_ptr); //还没进去看
    }
    // 向trajectory_builder_添加: 转换到'tracking_frame_'坐标系之后的加速度和角速度
    sensor_bridge_ptr->HandleImuMessage(sensor_id, msg);
}

// 回调: 处理激光数据
void Node::HandleLaserScanMessage(const int trajectory_id,
                                  const std::string& sensor_id,
                                  const sensor_msgs::LaserScan::ConstPtr& msg) {
    carto::common::MutexLocker lock(&mutex_);
    if (!sensor_samplers_.at(trajectory_id).rangefinder_sampler.Pulse()) {
        return;
    }
    ///处理流程
    // 0.调用pcl解析ros的点云消息 , 得到元组类型数据<带强度的点云,时间戳>
    // 1.对点云进行细分
    // 2.对细分的点云进行坐标系变换 :将点云从激光雷达坐标系转换到'tracking_frame_'坐标系
    // 3.然后调用trajectory_builder_ 添加传感器数据
    map_builder_bridge_.sensor_bridge(trajectory_id)
            ->HandleLaserScanMessage(sensor_id, msg);
}

// 同上
void Node::HandleMultiEchoLaserScanMessage(
        const int trajectory_id, const std::string& sensor_id,
        const sensor_msgs::MultiEchoLaserScan::ConstPtr& msg) {
    carto::common::MutexLocker lock(&mutex_);
    if (!sensor_samplers_.at(trajectory_id).rangefinder_sampler.Pulse()) {
        return;
    }
    map_builder_bridge_.sensor_bridge(trajectory_id)
            ->HandleMultiEchoLaserScanMessage(sensor_id, msg);
}

// 对于sensor_msgs::PointCloud2的数据
// 直接调用pcl解析,然后将解析的点push进carto::sensor::TimedPointCloud类型
// 然后坐标系转换, 添加传感器数据
void Node::HandlePointCloud2Message(
        const int trajectory_id, const std::string& sensor_id,
        const sensor_msgs::PointCloud2::ConstPtr& msg) {
    carto::common::MutexLocker lock(&mutex_);
    if (!sensor_samplers_.at(trajectory_id).rangefinder_sampler.Pulse()) {
        return;
    }
    map_builder_bridge_.sensor_bridge(trajectory_id)
            ->HandlePointCloud2Message(sensor_id, msg);
}

// 序列化, 调用map_builder_bridge_ 实现, 将数据储存到filename
void Node::SerializeState(const std::string& filename) {
    carto::common::MutexLocker lock(&mutex_);
    CHECK(map_builder_bridge_.SerializeState(filename))
            << "Could not write state.";
}

// 读取序列化的文件, 调用map_builder_bridge_加载状态
void Node::LoadState(const std::string& state_filename,
                     const bool load_frozen_state) {
    carto::common::MutexLocker lock(&mutex_);
    map_builder_bridge_.LoadState(state_filename, load_frozen_state);
}

}  // namespace cartographer_ros
