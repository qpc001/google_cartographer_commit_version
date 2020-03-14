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

#include "cartographer/mapping/2d/probability_grid_range_data_inserter_2d.h"

#include <cstdlib>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/mapping/2d/xy_index.h"
#include "cartographer/mapping/internal/2d/ray_casting.h"
#include "cartographer/mapping/probability_values.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

proto::ProbabilityGridRangeDataInserterOptions2D
CreateProbabilityGridRangeDataInserterOptions2D(
        common::LuaParameterDictionary* parameter_dictionary) {
    proto::ProbabilityGridRangeDataInserterOptions2D options;
    options.set_hit_probability(
                parameter_dictionary->GetDouble("hit_probability"));
    options.set_miss_probability(
                parameter_dictionary->GetDouble("miss_probability"));
    options.set_insert_free_space(
                parameter_dictionary->HasKey("insert_free_space")
                ? parameter_dictionary->GetBool("insert_free_space")
                : true);
    CHECK_GT(options.hit_probability(), 0.5);
    CHECK_LT(options.miss_probability(), 0.5);
    return options;
}

ProbabilityGridRangeDataInserter2D::ProbabilityGridRangeDataInserter2D(
        const proto::ProbabilityGridRangeDataInserterOptions2D& options)
    : options_(options),
      hit_table_(ComputeLookupTableToApplyCorrespondenceCostOdds(       // 初始化hit_table (初始化使用的Odds采用配置选项中的参数)
                                                                        Odds(options.hit_probability()))),
      miss_table_(ComputeLookupTableToApplyCorrespondenceCostOdds(      // 初始化miss_table (初始化使用的Odds采用配置选项中的参数)
                                                                        Odds(options.miss_probability()))) {}
// 根据激光扫描, 对hit中的栅格进行空闲概率更新(相当于更新占用概率)
// 如果需要, 则根据标志位对 射线上的没有击中的栅格也进行空闲概率更新
// 最后: 调用修正函数Grid2D::FinishUpdate(), 完成空闲概率的更新(实际上就是减去查表的时候,表格中所附带的一个偏移值kUpdateMarker)
void ProbabilityGridRangeDataInserter2D::Insert(
        const sensor::RangeData& range_data,    //激光数据
        GridInterface* const grid) const {      //传入概率地图
    // 类型转换, 实际上传进来的本就是概率地图
    ProbabilityGrid* const probability_grid = static_cast<ProbabilityGrid*>(grid);

    // By not finishing the update after hits are inserted, we give hits priority
    // (i.e. no hits will be ignored because of a miss in the same cell).
    // 根据激光扫描, 对hit中的栅格进行空闲概率更新(相当于更新占用概率)
    // 如果需要, 则根据标志位对 射线上的没有击中的栅格也进行空闲概率更新
    CastRays(range_data, hit_table_, miss_table_, options_.insert_free_space(),
             CHECK_NOTNULL(probability_grid));
    // 调用最后的修正,完成空闲概率的更新(实际上就是减去查表的时候,表格中所附带的一个偏移值kUpdateMarker)
    probability_grid->FinishUpdate();
}

}  // namespace mapping
}  // namespace cartographer
