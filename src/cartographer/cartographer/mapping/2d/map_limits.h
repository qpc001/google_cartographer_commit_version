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

#ifndef CARTOGRAPHER_MAPPING_2D_MAP_LIMITS_H_
#define CARTOGRAPHER_MAPPING_2D_MAP_LIMITS_H_

#include <utility>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/common/math.h"
#include "cartographer/mapping/2d/xy_index.h"
#include "cartographer/mapping/proto/2d/map_limits.pb.h"
#include "cartographer/mapping/trajectory_node.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/sensor/range_data.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

// Defines the limits of a grid map. This class must remain inlined for
// performance reasons.
// 定义了栅格地图的范围限制
class MapLimits {
public:

    // CellLimits={x轴最大cell数, y轴最大cell数}

    //构造(分辨率,最大值,cell限制)
    MapLimits(const double resolution, const Eigen::Vector2d& max,
              const CellLimits& cell_limits)
        : resolution_(resolution), max_(max), cell_limits_(cell_limits) {
        CHECK_GT(resolution_, 0.);
        CHECK_GT(cell_limits.num_x_cells, 0.);
        CHECK_GT(cell_limits.num_y_cells, 0.);
    }

    explicit MapLimits(const proto::MapLimits& map_limits)
        : resolution_(map_limits.resolution()),
          max_(transform::ToEigen(map_limits.max())),
          cell_limits_(map_limits.cell_limits()) {}

    // Returns the cell size in meters. All cells are square and the resolution is
    // the length of one side.
    // 返回每个cell的size,单位是米
    double resolution() const { return resolution_; }

    // Returns the corner of the limits, i.e., all pixels have positions with
    // smaller coordinates.
    // max_ 是指真实距离, 也就是现实中的'米'
    const Eigen::Vector2d& max() const { return max_; }

    // Returns the limits of the grid in number of cells.
    // CellLimits={x轴最大cell数, y轴最大cell数}
    // 返回栅格地图中cell网格的限制数量
    const CellLimits& cell_limits() const { return cell_limits_; }

    // Returns the index of the cell containing the 'point' which may be outside
    // the map, i.e., negative or too large indices that will return false for
    // Contains().
    // 输入某个2D点(真实距离),返回其所在的栅格
    Eigen::Array2i GetCellIndex(const Eigen::Vector2f& point) const {
        // Index values are row major and the top left has Eigen::Array2i::Zero()
        // and contains (centered_max_x, centered_max_y). We need to flip and
        // rotate.
        // 这里的2D点也是真实距离,也就是现实中的'米'
        // 下面转换为像素,然后向下取整
        return Eigen::Array2i(
                    common::RoundToInt((max_.y() - point.y()) / resolution_ - 0.5),
                    common::RoundToInt((max_.x() - point.x()) / resolution_ - 0.5));
    }

    // Returns true if the ProbabilityGrid contains 'cell_index'.
    // 如果输入的cell_index在限制范围之内,则返回true
    bool Contains(const Eigen::Array2i& cell_index) const {
        return (Eigen::Array2i(0, 0) <= cell_index).all() &&        // .all() 表示每个元素
                (cell_index <
                 Eigen::Array2i(cell_limits_.num_x_cells, cell_limits_.num_y_cells))
                .all();
    }

private:
    double resolution_;     //分辨率 (cell/米)
    Eigen::Vector2d max_;   //最大的距离 (米)
    CellLimits cell_limits_;//网格限制{最大x, 最大y}个网格
};

inline proto::MapLimits ToProto(const MapLimits& map_limits) {
    proto::MapLimits result;
    result.set_resolution(map_limits.resolution());
    *result.mutable_max() = transform::ToProto(map_limits.max());
    *result.mutable_cell_limits() = ToProto(map_limits.cell_limits());
    return result;
}

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_MAP_LIMITS_H_
