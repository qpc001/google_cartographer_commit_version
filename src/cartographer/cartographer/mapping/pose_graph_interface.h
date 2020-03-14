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

#ifndef CARTOGRAPHER_MAPPING_POSE_GRAPH_INTERFACE_H_
#define CARTOGRAPHER_MAPPING_POSE_GRAPH_INTERFACE_H_

#include <vector>

#include "cartographer/common/optional.h"
#include "cartographer/mapping/id.h"
#include "cartographer/mapping/submaps.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace mapping {

////////////////////////////////////////////////////////////////////////////////
/// \brief The PoseGraphInterface class
/// \brief 位姿图类接口-----基类
/// \brief 子类有:
///             1.class PoseGraph : public PoseGraphInterface
///             2.class PoseGraph2D : public PoseGraph
///             3.class PoseGraph3D : public PoseGraph
////////////////////////////////////////////////////////////////////////////////
class PoseGraphInterface {
public:
    // A "constraint" as in the paper by Konolige, Kurt, et al. "Efficient sparse
    // pose adjustment for 2d mapping." Intelligent Robots and Systems (IROS),
    // 2010 IEEE/RSJ International Conference on (pp. 22--29). IEEE, 2010.
    ///////////////////////////////////////////////////////////////////////////////
    /// \brief Constraint
    /// \brief 在class ConstraintBuilder2D 用到这个结构体
    ///////////////////////////////////////////////////////////////////////////////
    struct Constraint {
        // 结构体 {从j坐标系到i坐标系的变换T_i<-j , 平移量权重 , 旋转量权重 }
        struct Pose {
            transform::Rigid3d zbar_ij;
            double translation_weight;
            double rotation_weight;
        };

        // 子图i的{轨迹id , 子图索引}
        SubmapId submap_id;  // 'i' in the paper.
        // 节点j的{轨迹id,节点索引}
        NodeId node_id;      // 'j' in the paper.

        // Pose of the node 'j' relative to submap 'i'.
        // 上面的pose结构体 (从j坐标系到i坐标系的变换T_{i<-j} , 平移量权重 , 旋转量权重 )
        Pose pose;

        // Differentiates between intra-submap (where node 'j' was inserted into
        // submap 'i') and inter-submap constraints (where node 'j' was not inserted
        // into submap 'i').
        // 因为约束生成有两种: 1. 节点中的激光插入到子图中, 生成第一种约束
        //                  2. 节点刚好路过这个子图附近, 通过分枝定界+ceres优化匹配上了, 可以生成第二种约束
        // 标签: 表明该约束是否回环生成的
        //      INTRA_SUBMAP: 回环生成的约束,也就是上面所说的第二种约束
        //      INTER_SUBMAP: 第一种约束
        enum Tag { INTRA_SUBMAP, INTER_SUBMAP } tag;
    };

    // 路标点
    struct LandmarkNode {
        // 路标观测
        struct LandmarkObservation {
            int trajectory_id;                                  //轨迹id
            common::Time time;                                  //时间戳
            transform::Rigid3d landmark_to_tracking_transform;  //landmark到机器人本体坐标系的变换
            double translation_weight;                          //平移权重
            double rotation_weight;                             //旋转量权重
        };
        std::vector<LandmarkObservation> landmark_observations;     //上面定义的结构体
        common::optional<transform::Rigid3d> global_landmark_pose;  //landmark的全局位姿
    };

    struct SubmapPose {
        int version;
        transform::Rigid3d pose;
    };

    struct SubmapData {
        std::shared_ptr<const Submap> submap;
        transform::Rigid3d pose;
    };

    struct TrajectoryData {
        double gravity_constant = 9.8;
        std::array<double, 4> imu_calibration{{1., 0., 0., 0.}};
        common::optional<transform::Rigid3d> fixed_frame_origin_in_map;
    };

    using GlobalSlamOptimizationCallback =
    std::function<void(const std::map<int /* trajectory_id */, SubmapId>&,
    const std::map<int /* trajectory_id */, NodeId>&)>;

    PoseGraphInterface() {}
    virtual ~PoseGraphInterface() {}

    PoseGraphInterface(const PoseGraphInterface&) = delete;
    PoseGraphInterface& operator=(const PoseGraphInterface&) = delete;

    // Waits for all computations to finish and computes optimized poses.
    virtual void RunFinalOptimization() = 0;

    // Returns data for all submaps.
    virtual MapById<SubmapId, SubmapData> GetAllSubmapData() const = 0;

    // Returns the global poses for all submaps.
    virtual MapById<SubmapId, SubmapPose> GetAllSubmapPoses() const = 0;

    // Returns the transform converting data in the local map frame (i.e. the
    // continuous, non-loop-closed frame) into the global map frame (i.e. the
    // discontinuous, loop-closed frame).
    virtual transform::Rigid3d GetLocalToGlobalTransform(
            int trajectory_id) const = 0;

    // Returns the current optimized trajectories.
    virtual MapById<NodeId, TrajectoryNode> GetTrajectoryNodes() const = 0;

    // Returns the current optimized trajectory poses.
    virtual MapById<NodeId, TrajectoryNodePose> GetTrajectoryNodePoses()
    const = 0;

    // Returns the current optimized landmark poses.
    virtual std::map<std::string, transform::Rigid3d> GetLandmarkPoses()
    const = 0;

    // Sets global pose of landmark 'landmark_id' to given 'global_pose'.
    virtual void SetLandmarkPose(const std::string& landmark_id,
                                 const transform::Rigid3d& global_pose) = 0;

    // Checks if the given trajectory is finished.
    virtual bool IsTrajectoryFinished(int trajectory_id) const = 0;

    // Checks if the given trajectory is frozen.
    virtual bool IsTrajectoryFrozen(int trajectory_id) const = 0;

    // Returns the trajectory data.
    virtual std::map<int, TrajectoryData> GetTrajectoryData() const = 0;

    // Returns the collection of constraints.
    virtual std::vector<Constraint> constraints() const = 0;

    // Serializes the constraints and trajectories.
    virtual proto::PoseGraph ToProto() const = 0;

    // Sets the callback function that is invoked whenever the global optimization
    // problem is solved.
    virtual void SetGlobalSlamOptimizationCallback(
            GlobalSlamOptimizationCallback callback) = 0;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_POSE_GRAPH_INTERFACE_H_
