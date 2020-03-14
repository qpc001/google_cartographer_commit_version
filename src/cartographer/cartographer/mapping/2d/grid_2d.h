/*
 * Copyright 2018 The Cartographer Authors
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

#ifndef CARTOGRAPHER_MAPPING_2D_GRID_2D_H_
#define CARTOGRAPHER_MAPPING_2D_GRID_2D_H_

#include <vector>

#include "cartographer/mapping/2d/map_limits.h"
#include "cartographer/mapping/grid_interface.h"
#include "cartographer/mapping/proto/2d/grid_2d.pb.h"
#include "cartographer/mapping/proto/2d/submaps_options_2d.pb.h"
#include "cartographer/mapping/proto/submap_visualization.pb.h"

namespace cartographer {
namespace mapping {

proto::GridOptions2D CreateGridOptions2D(
        common::LuaParameterDictionary* const parameter_dictionary);

// 2D栅格地图
class Grid2D : public GridInterface {
public:
    //MapLimits(分辨率(cell/米) , 最大值 , cell数量限制{x_max,y_max} )
    explicit Grid2D(const MapLimits& limits,
                    float min_correspondence_cost,      //最小空闲概率
                    float max_correspondence_cost);     //最大空闲概率

    explicit Grid2D(const proto::Grid2D& proto);

    // Returns the limits of this Grid2D.
    const MapLimits& limits() const { return limits_; }

    // Finishes the update sequence.
    void FinishUpdate();

    // Returns the correspondence cost of the cell with 'cell_index'.
    // 给定[x,y]索引,返回空闲概率的float型
    float GetCorrespondenceCost(const Eigen::Array2i& cell_index) const;

    // Returns the minimum possible correspondence cost.
    float GetMinCorrespondenceCost() const { return min_correspondence_cost_; }

    // Returns the maximum possible correspondence cost.
    float GetMaxCorrespondenceCost() const { return max_correspondence_cost_; }

    // 给定索引, 如果在范围内, 并且栅格单元的空闲概率不等于0(表示该栅格不是未知状态), 则返回true
    bool IsKnown(const Eigen::Array2i& cell_index) const;

    // 裁剪子图 (就是对offset 和limits进行赋值)
    // offset = 已知有概率值的网格最小索引 [x,y]
    // limits = 已知有概率值的网格地图的尺寸(索引最大最小值相减) [x_max-x_min , y_max - y_min]
    void ComputeCroppedLimits(Eigen::Array2i* const offset,
                              CellLimits* const limits) const;

    // 如果输入的点不在范围内,表示需要扩展地图, 应该在FinishUpdate()函数之后立即调用 ('ApplyLookupTable()' 查表之前)
    // 这里会改变limits_ , correspondence_cost_cells_ , known_cells_box_ 3个成员变量
    virtual void GrowLimits(const Eigen::Vector2f& point);

    virtual std::unique_ptr<Grid2D> ComputeCroppedGrid() const = 0;

    virtual proto::Grid2D ToProto() const;

    virtual bool DrawToSubmapTexture(
            proto::SubmapQuery::Response::SubmapTexture* const texture,
            transform::Rigid3d local_pose) const = 0;

protected:
    // 返回记录各个栅格单元的空闲概率的列表
    const std::vector<uint16>& correspondence_cost_cells() const {
        return correspondence_cost_cells_;
    }
    const std::vector<int>& update_indices() const { return update_indices_; }
    const Eigen::AlignedBox2i& known_cells_box() const {
        return known_cells_box_;
    }

    std::vector<uint16>* mutable_correspondence_cost_cells() {
        return &correspondence_cost_cells_;
    }
    std::vector<int>* mutable_update_indices() { return &update_indices_; }
    Eigen::AlignedBox2i* mutable_known_cells_box() { return &known_cells_box_; }

    // Converts a 'cell_index' into an index into 'cells_'.
    // 输入[x,y]索引, 返回[y*num_x+x],也就是1个整数来表达索引
    int ToFlatIndex(const Eigen::Array2i& cell_index) const;

private:
    MapLimits limits_;
    //记录各个栅格单元的空闲概率
    //0表示对应栅格概率未知 , [1, 32767]表示空闲概率
    //可以通过一对儿函数CorrespondenceCostToValue和ValueToCorrespondenceCost相互转换
    std::vector<uint16> correspondence_cost_cells_; //记录各个栅格单元的空闲概率的列表
    float min_correspondence_cost_;     //	pfree的最小值(空闲概率的最小值)
    float max_correspondence_cost_;     //	pfree的最大值(空闲概率最大值)
    std::vector<int> update_indices_;   //  记录更新过的栅格单元的存储索引

    Eigen::AlignedBox2i known_cells_box_;   // 一个用于记录哪些栅格单元中有值的数据结构
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_GRID_2D_H_
