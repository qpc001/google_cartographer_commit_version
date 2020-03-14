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

#ifndef CARTOGRAPHER_MAPPING_TRAJECTORY_BUILDER_INTERFACE_H_
#define CARTOGRAPHER_MAPPING_TRAJECTORY_BUILDER_INTERFACE_H_

#include <functional>
#include <memory>
#include <string>

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/port.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/proto/trajectory_builder_options.pb.h"
#include "cartographer/mapping/submaps.h"
#include "cartographer/sensor/fixed_frame_pose_data.h"
#include "cartographer/sensor/imu_data.h"
#include "cartographer/sensor/landmark_data.h"
#include "cartographer/sensor/odometry_data.h"
#include "cartographer/sensor/timed_point_cloud_data.h"

namespace cartographer {
namespace mapping {

proto::TrajectoryBuilderOptions CreateTrajectoryBuilderOptions(
        common::LuaParameterDictionary* const parameter_dictionary);

class LocalSlamResultData;

// This interface is used for both 2D and 3D SLAM. Implementations wire up a
// global SLAM stack, i.e. local SLAM for initial pose estimates, scan matching
// to detect loop closure, and a sparse pose graph optimization to compute
// optimized pose estimates.
// 这个接口用于2D/3D通用

///////////////////////////////////////////////////////////////////////////////////////
/// \brief The TrajectoryBuilderInterface class
/// @brief 这是个基类
/// @brief 有以下几个子类:
///         1.class GlobalTrajectoryBuilder : public mapping::TrajectoryBuilderInterface
///         2.class CollatedTrajectoryBuilder : public TrajectoryBuilderInterface
////////////////////////////////////////////////////////////////////////////////////////
class TrajectoryBuilderInterface {
public:
    // 子图插入结果
    struct InsertionResult {
        //节点ID
        NodeId node_id;
        // 轨迹节点结构体(重力对齐四元数, 重力对齐并且滤波后的点云 , {} {} {} ,
        //              局部SLAM的节点位姿(scanMatching位姿估计: 机器人在子图坐标系的坐标) )
        std::shared_ptr<const TrajectoryNode::Data> constant_data;
        // 子图列表
        std::vector<std::shared_ptr<const Submap>> insertion_submaps;
    };

    // A callback which is called after local SLAM processes an accumulated
    // 'sensor::RangeData'. If the data was inserted into a submap, reports the
    // assigned 'NodeId', otherwise 'nullptr' if the data was filtered out.
    // 在local SLAM处理完积累的激光扫描数据之后的回调 , 即调用完AddRangeData之后的回调?

    // 回调函数, 可以传入的参数有这些
    using LocalSlamResultCallback =
    std::function<void(int            /* trajectory ID */,
    common::Time,          /*时间戳*/
    transform::Rigid3d  , /* local pose estimate : scanMatching得到的位姿估计(机器人在子图的坐标) */
    sensor::RangeData  , /* in local frame 子图坐标系下的激光扫描 */
    std::unique_ptr<const InsertionResult>)>;   /* 上面定义的结构体 子图插入结果 */

    // SensorId是个结构体
    // 元素有:  传感器id , 传感器类型
    struct SensorId {
        enum class SensorType {
            RANGE = 0,
            IMU,
            ODOMETRY,
            FIXED_FRAME_POSE,
            LANDMARK,
            LOCAL_SLAM_RESULT
        };

        SensorType type;
        std::string id;

        bool operator==(const SensorId& other) const {
            return std::forward_as_tuple(type, id) ==
                    std::forward_as_tuple(other.type, other.id);
        }

        bool operator<(const SensorId& other) const {
            return std::forward_as_tuple(type, id) <
                    std::forward_as_tuple(other.type, other.id);
        }
    };

    // 构造函数
    TrajectoryBuilderInterface() {}
    virtual ~TrajectoryBuilderInterface() {}

    TrajectoryBuilderInterface(const TrajectoryBuilderInterface&) = delete;     //这个delete表示禁用这个函数
    TrajectoryBuilderInterface& operator=(const TrajectoryBuilderInterface&) =  //这个delete表示禁用这个函数
            delete;

    /// 虚函数,等待子类重载
    // 点云数据
    virtual void AddSensorData(
            const std::string& sensor_id,
            const sensor::TimedPointCloudData& timed_point_cloud_data) = 0;
    // IMU数据
    virtual void AddSensorData(const std::string& sensor_id,
                               const sensor::ImuData& imu_data) = 0;
    // 里程计数据
    virtual void AddSensorData(const std::string& sensor_id,
                               const sensor::OdometryData& odometry_data) = 0;
    // gps数据
    virtual void AddSensorData(
            const std::string& sensor_id,
            const sensor::FixedFramePoseData& fixed_frame_pose) = 0;
    // landmark数据
    virtual void AddSensorData(const std::string& sensor_id,
                               const sensor::LandmarkData& landmark_data) = 0;


    // Allows to directly add local SLAM results to the 'PoseGraph'. Note that it
    // is invalid to add local SLAM results for a trajectory that has a
    // 'LocalTrajectoryBuilder2D/3D'.
    // 允许直接添加 local SLAM的结果到位子图中
    // 注意: 添加local SLAM结果到一个拥有'LocalTrajectoryBuilder2D/3D'的轨迹是无效的
    virtual void AddLocalSlamResultData(
            std::unique_ptr<mapping::LocalSlamResultData> local_slam_result_data) = 0;
};

// proto之间的转换
proto::SensorId ToProto(const TrajectoryBuilderInterface::SensorId& sensor_id);
TrajectoryBuilderInterface::SensorId FromProto(
        const proto::SensorId& sensor_id_proto);

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_TRAJECTORY_BUILDER_INTERFACE_H_
