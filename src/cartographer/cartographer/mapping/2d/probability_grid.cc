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
#include "cartographer/mapping/2d/probability_grid.h"

#include <limits>

#include "cartographer/common/make_unique.h"
#include "cartographer/mapping/probability_values.h"
#include "cartographer/mapping/submaps.h"

namespace cartographer {
namespace mapping {

// MapLimits:  定义了栅格地图的范围限制(分辨率,最大值,cell限制)
// 构造父类 (MapLimits , 空闲概率最小值 ,空闲概率最大值 )
ProbabilityGrid::ProbabilityGrid(const MapLimits& limits)
    : Grid2D(limits, kMinCorrespondenceCost, kMaxCorrespondenceCost) {}

ProbabilityGrid::ProbabilityGrid(const proto::Grid2D& proto) : Grid2D(proto) {
    CHECK(proto.has_probability_grid_2d());
}

// Sets the probability of the cell at 'cell_index' to the given
// 'probability'. Only allowed if the cell was unknown before.
// 设置 给定索引[x,y]的栅格 的占据概率 (只允许对未知状态的栅格赋值)
void ProbabilityGrid::SetProbability(const Eigen::Array2i& cell_index,
                                     const float probability) {
    // 取指针 : 父类的栅格单元的空闲概率的列表: correspondence_cost_cells_
    // 然后根据索引指向某个元素
    uint16& cell =
            (*mutable_correspondence_cost_cells())[ToFlatIndex(cell_index)];
    // 检查该栅格是否为未知状态
    CHECK_EQ(cell, kUnknownProbabilityValue);
    // 对元素进行赋值
    cell =
            CorrespondenceCostToValue(ProbabilityToCorrespondenceCost(probability));
    // 根据cell索引对父类的known_cells_box_ 成员进行扩展
    mutable_known_cells_box()->extend(cell_index.matrix());
}

// Applies the 'odds' specified when calling ComputeLookupTableToApplyOdds()
// to the probability of the cell at 'cell_index' if the cell has not already
// been updated. Multiple updates of the same cell will be ignored until
// FinishUpdate() is called. Returns true if the cell was updated.
//
// If this is the first call to ApplyOdds() for the specified cell, its value
// will be set to probability corresponding to 'odds'.
// 通过查表来更新给定索引[x,y]栅格单元的占用概率
bool ProbabilityGrid::ApplyLookupTable(const Eigen::Array2i& cell_index,
                                       const std::vector<uint16>& table) {
    // 在函数的一开始的时候，先检查一下查找表的大小。
    DCHECK_EQ(table.size(), kUpdateMarker);
    // 然后通过cell_index计算栅格单元的存储索引
    const int flat_index = ToFlatIndex(cell_index);
    // 获取该cell在空闲概率整数列表的指针
    uint16* cell = &(*mutable_correspondence_cost_cells())[flat_index];
    // 检查空闲概率整数值没有超过table数量上限(下面需要根据这个值来查表)
    if (*cell >= kUpdateMarker) {
        return false;
    }
    // 将这个 要更新的cell索引push到父类的 update_indices_列表,以作为记录
    mutable_update_indices()->push_back(flat_index);
    // 查表, 对该cell的空闲概率整数值进行更新
    *cell = table[*cell];
    DCHECK_GE(*cell, kUpdateMarker);
    // 根据cell索引对父类的known_cells_box_ 成员进行扩展 , 或者进行标记,标记该cell对应的空闲概率已知
    mutable_known_cells_box()->extend(cell_index.matrix());
    return true;
}

// Returns the probability of the cell with 'cell_index'.
// 返回给定索引的占据概率[0~1]浮点数, 如果在范围以外,则返回最大占用概率
float ProbabilityGrid::GetProbability(const Eigen::Array2i& cell_index) const {
    if (!limits().Contains(cell_index)) return kMinProbability;
    return CorrespondenceCostToProbability(ValueToCorrespondenceCost(
                                               correspondence_cost_cells()[ToFlatIndex(cell_index)]));
}

// 此外ProbabilityGrid还重载了两个虚函数ToProto和DrawToSubmapTexture，它们主要用于对象的序列化，不是重点不管它
proto::Grid2D ProbabilityGrid::ToProto() const {
    proto::Grid2D result;
    result = Grid2D::ToProto();
    result.mutable_probability_grid_2d();
    return result;
}

// 裁剪概率地图,返回新的概率地图
std::unique_ptr<Grid2D> ProbabilityGrid::ComputeCroppedGrid() const {

    Eigen::Array2i offset;
    CellLimits cell_limits;
    // 调用父类裁剪子图函数 (就是对offset 和limits进行赋值)
    // offset = 已知有概率值的网格最小索引 [x,y]
    // limits = 已知有概率值的网格地图的尺寸(索引最大最小值相减) [x_max-x_min , y_max - y_min]
    ComputeCroppedLimits(&offset, &cell_limits);
    // 取分辨率
    const double resolution = limits().resolution();
    // 更新max值 (实际中的距离,单位为米)
    const Eigen::Vector2d max =
            limits().max() - resolution * Eigen::Vector2d(offset.y(), offset.x());
    // 重新构造一个ProbabilityGrid,作为裁剪之后的概率地图
    std::unique_ptr<ProbabilityGrid> cropped_grid =
            common::make_unique<ProbabilityGrid>(
                MapLimits(resolution, max, cell_limits));
    // 对之前已知概率的cell进行重新赋值
    for (const Eigen::Array2i& xy_index : XYIndexRangeIterator(cell_limits)) {
        if (!IsKnown(xy_index + offset)) continue;
        cropped_grid->SetProbability(xy_index, GetProbability(xy_index + offset));
    }

    // 返回裁剪后的概率地图
    return std::unique_ptr<Grid2D>(cropped_grid.release());
}

// 此外ProbabilityGrid还重载了两个虚函数ToProto和DrawToSubmapTexture，它们主要用于对象的序列化，不是重点不管它
bool ProbabilityGrid::DrawToSubmapTexture(
        proto::SubmapQuery::Response::SubmapTexture* const texture,
        transform::Rigid3d local_pose) const {
    Eigen::Array2i offset;
    CellLimits cell_limits;
    ComputeCroppedLimits(&offset, &cell_limits);

    std::string cells;
    for (const Eigen::Array2i& xy_index : XYIndexRangeIterator(cell_limits)) {
        if (!IsKnown(xy_index + offset)) {
            cells.push_back(0 /* unknown log odds value */);
            cells.push_back(0 /* alpha */);
            continue;
        }
        // We would like to add 'delta' but this is not possible using a value and
        // alpha. We use premultiplied alpha, so when 'delta' is positive we can
        // add it by setting 'alpha' to zero. If it is negative, we set 'value' to
        // zero, and use 'alpha' to subtract. This is only correct when the pixel
        // is currently white, so walls will look too gray. This should be hard to
        // detect visually for the user, though.
        const int delta =
                128 - ProbabilityToLogOddsInteger(GetProbability(xy_index + offset));
        const uint8 alpha = delta > 0 ? 0 : -delta;
        const uint8 value = delta > 0 ? delta : 0;
        cells.push_back(value);
        cells.push_back((value || alpha) ? alpha : 1);
    }

    common::FastGzipString(cells, texture->mutable_cells());
    texture->set_width(cell_limits.num_x_cells);
    texture->set_height(cell_limits.num_y_cells);
    const double resolution = limits().resolution();
    texture->set_resolution(resolution);
    const double max_x = limits().max().x() - resolution * offset.y();
    const double max_y = limits().max().y() - resolution * offset.x();
    *texture->mutable_slice_pose() = transform::ToProto(
                local_pose.inverse() *
                transform::Rigid3d::Translation(Eigen::Vector3d(max_x, max_y, 0.)));

    return true;
}

}  // namespace mapping
}  // namespace cartographer
