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
#include "cartographer/mapping/2d/grid_2d.h"

#include "cartographer/mapping/probability_values.h"

namespace cartographer {
namespace mapping {

proto::GridOptions2D CreateGridOptions2D(
        common::LuaParameterDictionary* const parameter_dictionary) {
    proto::GridOptions2D options;
    const std::string grid_type_string =
            parameter_dictionary->GetString("grid_type");
    proto::GridOptions2D_GridType grid_type;
    CHECK(proto::GridOptions2D_GridType_Parse(grid_type_string, &grid_type))
            << "Unknown GridOptions2D_GridType kind: " << grid_type_string;
    options.set_grid_type(grid_type);
    options.set_resolution(parameter_dictionary->GetDouble("resolution"));
    return options;
}

Grid2D::Grid2D(const MapLimits& limits,         //MapLimits(分辨率(cell/米) , 最大值 , cell数量限制{x_max,y_max} )
               float min_correspondence_cost,   //最小空闲概率
               float max_correspondence_cost)   //最大空闲概率
    : limits_(limits),
      correspondence_cost_cells_(               //初始化一个数组 [0~最大栅格数]= 0 (表示未知空闲概率)
                                                limits_.cell_limits().num_x_cells * limits_.cell_limits().num_y_cells,
                                                kUnknownCorrespondenceValue),
      min_correspondence_cost_(min_correspondence_cost),
      max_correspondence_cost_(max_correspondence_cost) {
    CHECK_LT(min_correspondence_cost_, max_correspondence_cost_);
}

Grid2D::Grid2D(const proto::Grid2D& proto)
    : limits_(proto.limits()), correspondence_cost_cells_() {
    if (proto.has_known_cells_box()) {
        const auto& box = proto.known_cells_box();
        known_cells_box_ =
                Eigen::AlignedBox2i(Eigen::Vector2i(box.min_x(), box.min_y()),
                                    Eigen::Vector2i(box.max_x(), box.max_y()));
    }
    correspondence_cost_cells_.reserve(proto.cells_size());
    for (const auto& cell : proto.cells()) {
        CHECK_LE(cell, std::numeric_limits<uint16>::max());
        correspondence_cost_cells_.push_back(cell);
    }
    if (proto.min_correspondence_cost() == 0.f &&
            proto.max_correspondence_cost() == 0.f) {
        LOG(WARNING)
                << "proto::Grid2D: max_correspondence_cost and min_correspondence_cost "
                   "are initialized with 0 indicating an older version of the "
                   "protobuf format. Loading default values.";
        min_correspondence_cost_ = kMinCorrespondenceCost;
        max_correspondence_cost_ = kMaxCorrespondenceCost;
    } else {
        min_correspondence_cost_ = proto.min_correspondence_cost();
        max_correspondence_cost_ = proto.max_correspondence_cost();
    }
    CHECK_LT(min_correspondence_cost_, max_correspondence_cost_);
}

// Finishes the update sequence.
// 完成更新操作的第二步: 对有变动的栅格的空闲概率进行修正 (就是对通过查表更新过的栅格单元的空闲概率整数值进行修正)
void Grid2D::FinishUpdate() {
    // update_indices_ 记录有变化的栅格单元的索引
    // 如果更新序列还有元素,则
    while (!update_indices_.empty()) {
        DCHECK_GE(correspondence_cost_cells_[update_indices_.back()],
                kUpdateMarker);
        //update_indices_.back() : 有变化的栅格索引
        //这个操作就是:  对于有变化的栅格 , 其空闲概率整数都减去(32768) ?
        // 原因是:在初始化查询表ComputeLookupTableToApplyOdds()的时候,在每个栅格对应的概率整数上都加了这么个整数
        correspondence_cost_cells_[update_indices_.back()] -= kUpdateMarker;
        update_indices_.pop_back();
    }
}

// Returns the correspondence cost of the cell with 'cell_index'.
// 给定[x,y]索引,返回空闲概率的float型
float Grid2D::GetCorrespondenceCost(const Eigen::Array2i& cell_index) const {
    // 如果索引不在范围内, 则返回 空闲概率最大值
    if (!limits().Contains(cell_index)) return kMaxCorrespondenceCost;
    // 否则, 返回空闲概率的float型
    return ValueToCorrespondenceCost(   //这个操作是将correspondence_cost_cells里面的空闲概率整数映射到浮点数空间
                correspondence_cost_cells()[ToFlatIndex(cell_index)]);
    //correspondence_cost_cells()[ToFlatIndex(cell_index)] : 得到了给定索引的栅格的空闲概率(uint_16)
}

// Returns true if the correspondence cost at the specified index is known.
// 给定索引, 如果在范围内, 并且栅格单元的空闲概率不等于0(表示该栅格不是未知状态), 则返回true
bool Grid2D::IsKnown(const Eigen::Array2i& cell_index) const {
    return limits_.Contains(cell_index) &&
            correspondence_cost_cells_[ToFlatIndex(cell_index)] !=
            kUnknownCorrespondenceValue;
}

// 裁剪子图 (就是对offset 和limits进行赋值)
// offset = 已知有概率值的网格最小索引 [x,y]
// limits = 已知有概率值的网格地图的尺寸(索引最大最小值相减) [x_max-x_min , y_max - y_min]
void Grid2D::ComputeCroppedLimits(Eigen::Array2i* const offset,
                                  CellLimits* const limits) const {
    if (known_cells_box_.isEmpty()) {
        *offset = Eigen::Array2i::Zero();
        *limits = CellLimits(1, 1);
        return;
    }
    *offset = known_cells_box_.min().array();
    *limits = CellLimits(known_cells_box_.sizes().x() + 1,
                         known_cells_box_.sizes().y() + 1);
}

// Grows the map as necessary to include 'point'. This changes the meaning of
// these coordinates going forward. This method must be called immediately
// after 'FinishUpdate', before any calls to 'ApplyLookupTable'.
// 如果输入的点不在范围内,表示需要扩展地图, 应该在FinishUpdate()函数之后立即调用 ('ApplyLookupTable()' 查表之前)
// 这里会改变limits_ , correspondence_cost_cells_ , known_cells_box_ 3个成员变量
void Grid2D::GrowLimits(const Eigen::Vector2f& point) {
    CHECK(update_indices_.empty());
    while (!limits_.Contains(limits_.GetCellIndex(point))) {
        const int x_offset = limits_.cell_limits().num_x_cells / 2;
        const int y_offset = limits_.cell_limits().num_y_cells / 2;
        const MapLimits new_limits(
                    limits_.resolution(),
                    limits_.max() +
                    limits_.resolution() * Eigen::Vector2d(y_offset, x_offset),
                    CellLimits(2 * limits_.cell_limits().num_x_cells,
                               2 * limits_.cell_limits().num_y_cells));
        const int stride = new_limits.cell_limits().num_x_cells;
        const int offset = x_offset + stride * y_offset;
        const int new_size = new_limits.cell_limits().num_x_cells *
                new_limits.cell_limits().num_y_cells;
        std::vector<uint16> new_cells(new_size, kUnknownCorrespondenceValue);
        for (int i = 0; i < limits_.cell_limits().num_y_cells; ++i) {
            for (int j = 0; j < limits_.cell_limits().num_x_cells; ++j) {
                new_cells[offset + j + i * stride] =
                        correspondence_cost_cells_[j +
                        i * limits_.cell_limits().num_x_cells];
            }
        }
        // 得到新的栅格单元的空闲概率序列
        correspondence_cost_cells_ = new_cells;
        // 扩展limit
        limits_ = new_limits;
        // 对known_cells_box_进行平移
        if (!known_cells_box_.isEmpty()) {
            known_cells_box_.translate(Eigen::Vector2i(x_offset, y_offset));
        }
    }
}

proto::Grid2D Grid2D::ToProto() const {
    proto::Grid2D result;
    *result.mutable_limits() = mapping::ToProto(limits_);
    result.mutable_cells()->Reserve(correspondence_cost_cells_.size());
    for (const auto& cell : correspondence_cost_cells_) {
        result.mutable_cells()->Add(cell);
    }
    CHECK(update_indices().empty()) << "Serializing a grid during an update is "
                                       "not supported. Finish the update first.";
    if (!known_cells_box().isEmpty()) {
        auto* const box = result.mutable_known_cells_box();
        box->set_max_x(known_cells_box().max().x());
        box->set_max_y(known_cells_box().max().y());
        box->set_min_x(known_cells_box().min().x());
        box->set_min_y(known_cells_box().min().y());
    }
    result.set_min_correspondence_cost(min_correspondence_cost_);
    result.set_max_correspondence_cost(max_correspondence_cost_);
    return result;
}

// 输入[x,y]索引, 返回[y*num_x+x],也就是1个整数来表达索引
int Grid2D::ToFlatIndex(const Eigen::Array2i& cell_index) const {
    CHECK(limits_.Contains(cell_index)) << cell_index;
    return limits_.cell_limits().num_x_cells * cell_index.y() + cell_index.x();
}

}  // namespace mapping
}  // namespace cartographer
