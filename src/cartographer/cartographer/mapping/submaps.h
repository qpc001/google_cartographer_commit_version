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

#ifndef CARTOGRAPHER_MAPPING_SUBMAPS_H_
#define CARTOGRAPHER_MAPPING_SUBMAPS_H_

#include <memory>
#include <vector>

#include "Eigen/Geometry"
#include "cartographer/common/math.h"
#include "cartographer/common/port.h"
#include "cartographer/mapping/id.h"
#include "cartographer/mapping/probability_values.h"
#include "cartographer/mapping/proto/serialization.pb.h"
#include "cartographer/mapping/proto/submap_visualization.pb.h"
#include "cartographer/mapping/trajectory_node.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

// Converts the given probability to log odds.
// 给定占据概率, 先求Odds, 再求log
// 即返回 log(Odds)
inline float Logit(float probability) {
    return std::log(probability / (1.f - probability));
}

const float kMaxLogOdds = Logit(kMaxProbability);   //最大占据概率对应的 log(Odds_max)
const float kMinLogOdds = Logit(kMinProbability);   //最小占据概率对应的 log(Odds_min)

// Converts a probability to a log odds integer. 0 means unknown, [kMinLogOdds,
// kMaxLogOdds] is mapped to [1, 255].
// 将输入概率(may be [0.1,0.9]) , 映射为[1-255]之间的整数
// 1. 对输入的概率求log_p
// 2. (log_p - log_min) / (log_max - log_min) *254 + 1
inline uint8 ProbabilityToLogOddsInteger(const float probability) {
    const int value = common::RoundToInt((Logit(probability) - kMinLogOdds) *
                                         254.f / (kMaxLogOdds - kMinLogOdds)) +
            1;
    CHECK_LE(1, value);
    CHECK_GE(255, value);
    return value;
}

// An individual submap, which has a 'local_pose' in the local map frame, keeps
// track of how many range data were inserted into it, and sets the
// 'finished_probability_grid' to be used for loop closing once the map no
// longer changes.

///////////////////////////////////////////////////////////////////////////
/// \brief The Submap class
/// \brief 这是 子图的基类
///////////////////////////////////////////////////////////////////////////
class Submap {
public:
    // 构造函数 : 初始化
    // local_pose_: 子图的坐标系原点在全局坐标系的坐标?
    Submap(const transform::Rigid3d& local_submap_pose)
        : local_pose_(local_submap_pose) {}
    virtual ~Submap() {}

    virtual void ToProto(proto::Submap* proto,
                         bool include_probability_grid_data) const = 0;
    virtual void UpdateFromProto(const proto::Submap& proto) = 0;

    // Fills data into the 'response'.
    // 转proto用的,不是重点
    virtual void ToResponseProto(
            const transform::Rigid3d& global_submap_pose,
            proto::SubmapQuery::Response* response) const = 0;

    // Pose of this submap in the local map frame.
    // 返回子图坐标系原点在 local map坐标系的坐标
    transform::Rigid3d local_pose() const { return local_pose_; }

    // Number of RangeData inserted.
    // 已经插入的激光数据 数量的set和get
    int num_range_data() const { return num_range_data_; }
    void set_num_range_data(const int num_range_data) {
        num_range_data_ = num_range_data;
    }

    // Whether the submap is finished or not.
    // 子图完成标志位的set和get
    bool finished() const { return finished_; }
    void set_finished(bool finished) { finished_ = finished; }

private:
    const transform::Rigid3d local_pose_;   //子图的坐标系原点在local map坐标系的坐标
    int num_range_data_ = 0;                //已经插入的激光扫描帧数据
    bool finished_ = false;                 //子图是否完成标志位
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_SUBMAPS_H_
