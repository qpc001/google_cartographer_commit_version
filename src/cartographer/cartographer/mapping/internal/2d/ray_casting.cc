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

#include "cartographer/mapping/internal/2d/ray_casting.h"

namespace cartographer {
namespace mapping {
namespace {

// Factor for subpixel accuracy of start and end point.
// 把每个cell继续细分成 kSubpixelScale x kSubpixelScale 个小块
constexpr int kSubpixelScale = 1000;

// We divide each pixel in kSubpixelScale x kSubpixelScale subpixels. 'begin'
// and 'end' are coordinates at subpixel precision. We compute all pixels in
// which some part of the line segment connecting 'begin' and 'end' lies.

// 处理射线起点到hit点之间的栅格，把它们都看做是发生了miss事件的栅格，查找miss_table更新占用概率。
// 但是需要注意这里的begin和end都是精细栅格下的索引。
/// 暂不细看
void CastRay(const Eigen::Array2i& begin, const Eigen::Array2i& end,
             const std::vector<uint16>& miss_table,
             ProbabilityGrid* const probability_grid) {
    // For simplicity, we order 'begin' and 'end' by their x coordinate.
    // 如果射线起点的x值 大于 击中点的x值, 则交换起点和终点
    if (begin.x() > end.x()) {
        CastRay(end, begin, miss_table, probability_grid);
        return;
    }
    // 一些检查
    CHECK_GE(begin.x(), 0);
    CHECK_GE(begin.y(), 0);
    CHECK_GE(end.y(), 0);

    // Special case: We have to draw a vertical line in full pixels, as 'begin'
    // and 'end' have the same full pixel x coordinate.
    if (begin.x() / kSubpixelScale == end.x() / kSubpixelScale) {
        Eigen::Array2i current(begin.x() / kSubpixelScale,
                               std::min(begin.y(), end.y()) / kSubpixelScale);
        const int end_y = std::max(begin.y(), end.y()) / kSubpixelScale;
        for (; current.y() <= end_y; ++current.y()) {
            probability_grid->ApplyLookupTable(current, miss_table);
        }
        return;
    }

    const int64 dx = end.x() - begin.x();
    const int64 dy = end.y() - begin.y();
    const int64 denominator = 2 * kSubpixelScale * dx;

    // The current full pixel coordinates. We begin at 'begin'.
    Eigen::Array2i current = begin / kSubpixelScale;

    // To represent subpixel centers, we use a factor of 2 * 'kSubpixelScale' in
    // the denominator.
    // +-+-+-+ -- 1 = (2 * kSubpixelScale) / (2 * kSubpixelScale)
    // | | | |
    // +-+-+-+
    // | | | |
    // +-+-+-+ -- top edge of first subpixel = 2 / (2 * kSubpixelScale)
    // | | | | -- center of first subpixel = 1 / (2 * kSubpixelScale)
    // +-+-+-+ -- 0 = 0 / (2 * kSubpixelScale)

    // The center of the subpixel part of 'begin.y()' assuming the
    // 'denominator', i.e., sub_y / denominator is in (0, 1).
    int64 sub_y = (2 * (begin.y() % kSubpixelScale) + 1) * dx;

    // The distance from the from 'begin' to the right pixel border, to be divided
    // by 2 * 'kSubpixelScale'.
    const int first_pixel =
            2 * kSubpixelScale - 2 * (begin.x() % kSubpixelScale) - 1;
    // The same from the left pixel border to 'end'.
    const int last_pixel = 2 * (end.x() % kSubpixelScale) + 1;

    // The full pixel x coordinate of 'end'.
    const int end_x = std::max(begin.x(), end.x()) / kSubpixelScale;

    // Move from 'begin' to the next pixel border to the right.
    sub_y += dy * first_pixel;
    if (dy > 0) {
        while (true) {
            probability_grid->ApplyLookupTable(current, miss_table);
            while (sub_y > denominator) {
                sub_y -= denominator;
                ++current.y();
                probability_grid->ApplyLookupTable(current, miss_table);
            }
            ++current.x();
            if (sub_y == denominator) {
                sub_y -= denominator;
                ++current.y();
            }
            if (current.x() == end_x) {
                break;
            }
            // Move from one pixel border to the next.
            sub_y += dy * 2 * kSubpixelScale;
        }
        // Move from the pixel border on the right to 'end'.
        sub_y += dy * last_pixel;
        probability_grid->ApplyLookupTable(current, miss_table);
        while (sub_y > denominator) {
            sub_y -= denominator;
            ++current.y();
            probability_grid->ApplyLookupTable(current, miss_table);
        }
        CHECK_NE(sub_y, denominator);
        CHECK_EQ(current.y(), end.y() / kSubpixelScale);
        return;
    }

    // Same for lines non-ascending in y coordinates.
    while (true) {
        probability_grid->ApplyLookupTable(current, miss_table);
        while (sub_y < 0) {
            sub_y += denominator;
            --current.y();
            probability_grid->ApplyLookupTable(current, miss_table);
        }
        ++current.x();
        if (sub_y == 0) {
            sub_y += denominator;
            --current.y();
        }
        if (current.x() == end_x) {
            break;
        }
        sub_y += dy * 2 * kSubpixelScale;
    }
    sub_y += dy * last_pixel;
    probability_grid->ApplyLookupTable(current, miss_table);
    while (sub_y < 0) {
        sub_y += denominator;
        --current.y();
        probability_grid->ApplyLookupTable(current, miss_table);
    }
    CHECK_NE(sub_y, 0);
    CHECK_EQ(current.y(), end.y() / kSubpixelScale);
}

void GrowAsNeeded(const sensor::RangeData& range_data,
                  ProbabilityGrid* const probability_grid) {
    Eigen::AlignedBox2f bounding_box(range_data.origin.head<2>());
    constexpr float kPadding = 1e-6f;
    for (const Eigen::Vector3f& hit : range_data.returns) {
        bounding_box.extend(hit.head<2>());
    }
    for (const Eigen::Vector3f& miss : range_data.misses) {
        bounding_box.extend(miss.head<2>());
    }
    probability_grid->GrowLimits(bounding_box.min() -
                                 kPadding * Eigen::Vector2f::Ones());
    probability_grid->GrowLimits(bounding_box.max() +
                                 kPadding * Eigen::Vector2f::Ones());
}

}  // namespace

// 根据激光扫描, 对hit中的栅格进行空闲概率更新(相当于更新占用概率)
// 如果需要, 则根据标志位对 射线上的没有击中的栅格也进行空闲概率更新
void CastRays(const sensor::RangeData& range_data,          // 将要插入的扫描数据
              const std::vector<uint16>& hit_table,         // 预先计算的hit_table
              const std::vector<uint16>& miss_table,        // 预先计算的miss_table
              const bool insert_free_space,
              ProbabilityGrid* const probability_grid) {
    // 扩展概率地图,如果需要
    GrowAsNeeded(range_data, probability_grid);

    // 获取概率地图limit (分辨率(cell/米) , 最大值 , cell数量限制{x_max,y_max} )
    const MapLimits& limits = probability_grid->limits();
    // 把每个cell继续细分成 kSubpixelScale x kSubpixelScale 个小块
    // 计算每个小块的真实长度, (米) , 得到新的分辨率
    const double superscaled_resolution = limits.resolution() / kSubpixelScale;
    // 细分之后的超分辨率MapLimits (每个小块的真实长度:(米) , max ,  换成小块之后的小块栅格数量[x_num, y_num])
    const MapLimits superscaled_limits(
                superscaled_resolution, limits.max(),
                CellLimits(limits.cell_limits().num_x_cells * kSubpixelScale,
                           limits.cell_limits().num_y_cells * kSubpixelScale));
    // 获取激光雷达原点所在的cell的index, 即射线的起点
    // MapLimits::GetCellIndex: 输入某个2D点(真实距离),返回其所在的栅格
    // range_data.origin.head<2>() : 激光坐标系原点在tracking坐标系的坐标
    const Eigen::Array2i begin =
            superscaled_limits.GetCellIndex(range_data.origin.head<2>());

    std::vector<Eigen::Array2i> ends;
    ends.reserve(range_data.returns.size());
    // 遍历激光有返回的点
    for (const Eigen::Vector3f& hit : range_data.returns) {
        // MapLimits::GetCellIndex: 输入某个2D点(真实距离),返回其所在的栅格
        // 获取激光打到的真实点所在的cell的index
        // 将该index 推入ends列表
        ends.push_back(superscaled_limits.GetCellIndex(hit.head<2>()));

        // ProbabilityGrid::ApplyLookupTable : 通过查表来更新给定索引[x,y]栅格单元的空闲概率
        // 即使用 给定索引的栅格的 hit_table[给定索引的栅格]
        // 这里的hit_table是初始化的时候预计算的
        // 这里输入的索引是恢复尺度的索引, 不是细分的索引
        probability_grid->ApplyLookupTable(ends.back() / kSubpixelScale, hit_table);
    }

    // 标志位,是否对那些没有击中的点进行更新, 更新它们的空闲概率?
    if (!insert_free_space) {
        return;
    }

    // 处理射线起点到hit点之间的栅格，把它们都看做是发生了miss事件的栅格，查找miss_table更新占用概率。
    // 但是需要注意这里的begin和end都是精细栅格下的索引。
    for (const Eigen::Array2i& end : ends) {
        CastRay(begin, end, miss_table, probability_grid);
    }

    // 最后处理miss点的情况，我们同样认为射线起点到miss点之间的栅格发生的都是miss事件
    // 所谓miss点: 就是激光打出去,但是没有数值返回的,
    // 需要对整条射线上的栅格进行空闲概率更新
    for (const Eigen::Vector3f& missing_echo : range_data.misses) {
        CastRay(begin, superscaled_limits.GetCellIndex(missing_echo.head<2>()),
                miss_table, probability_grid);
    }
}

}  // namespace mapping
}  // namespace cartographer
