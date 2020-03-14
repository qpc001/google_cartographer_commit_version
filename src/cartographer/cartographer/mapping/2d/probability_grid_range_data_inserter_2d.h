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

#ifndef CARTOGRAPHER_MAPPING_2D_RANGE_DATA_INSERTER_2D_PROBABILITY_GRID_H_
#define CARTOGRAPHER_MAPPING_2D_RANGE_DATA_INSERTER_2D_PROBABILITY_GRID_H_

#include <utility>
#include <vector>

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/port.h"
#include "cartographer/mapping/2d/probability_grid.h"
#include "cartographer/mapping/2d/xy_index.h"
#include "cartographer/mapping/proto/2d/probability_grid_range_data_inserter_options_2d.pb.h"
#include "cartographer/mapping/range_data_inserter_interface.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/sensor/range_data.h"

namespace cartographer {
namespace mapping {

proto::ProbabilityGridRangeDataInserterOptions2D
CreateProbabilityGridRangeDataInserterOptions2D(
        common::LuaParameterDictionary* parameter_dictionary);

// 子类, 2D栅格地图 激光数据插入器
class ProbabilityGridRangeDataInserter2D : public RangeDataInserterInterface {
public:
    // 构造函数 : 基于配置选项构建
    // 同时:
    // 1. 初始化hit_table : hit_table_[原来的空闲概率整数值(uint16)] = 再次击中之后新的空闲概率整数值(uint16)
    // 2. 初始化mis_table : miss_table_[原来的空闲概率整数值(uint16)] = 再次空闲(没有被击中)之后新的空闲概率整数值(uint16)
    explicit ProbabilityGridRangeDataInserter2D(
            const proto::ProbabilityGridRangeDataInserterOptions2D& options);

    ProbabilityGridRangeDataInserter2D(
            const ProbabilityGridRangeDataInserter2D&) = delete;
    ProbabilityGridRangeDataInserter2D& operator=(
            const ProbabilityGridRangeDataInserter2D&) = delete;

    // Inserts 'range_data' into 'probability_grid'.
    // 重载父类虚函数 : 插入激光雷达数据到概率地图
    // 1. 根据激光扫描, 对hit中的栅格进行空闲概率更新(相当于更新占用概率)
    // 2. 如果需要, 则根据标志位对 射线上的没有击中的栅格也进行空闲概率更新
    // 3. 最后: 调用修正函数Grid2D::FinishUpdate(), 完成空闲概率的更新(实际上就是减去查表的时候,表格中所附带的一个偏移值kUpdateMarker)
    virtual void Insert(const sensor::RangeData& range_data,
                        GridInterface* grid) const override;

private:
    const proto::ProbabilityGridRangeDataInserterOptions2D options_;
    /** @brief 所谓的 hit_table_
     * 0. 就是说 某个栅格, 原来有着一个占用概率<=====>占用概率整数值
     * 1. 当激光击中这个栅格的时候 , 不是需要对这个栅格的占用概率进行更新吗 ?
     * 2. 这时候这个 hit_table_就发挥作用了 :
     * 3. hit_table_[0~32767] = [0~32767] ====> hit_table_ 实际上就是一个整数的映射表, 继续看下面
     * 4. 给定某个栅格的占用概率整数值(uint16) , 如果该栅格再次被激光击中了 , 那么它新的占用概率值 , 其实是可以预先计算出来的
     * 5. hit_table_ 里面的东西, 存的就是: 栅格原有的占用概率整数值(uint16) =====> 再次击中之后新的占用概率整数值(uint16)
     * 7. (划重点): hit_table_[原有的占用概率整数值(uint16)] = 再次击中之后新的占用概率整数值(uint16)
     * 8. 这就不难解释为什么 hit_table_[0~32767] = [0~32767]
     **/

    // 不过这里的hit_table_不是真的hit_table , 是 hit完之后对应的miss_table,即
    // hit_table_[原来的空闲概率整数值(uint16)] = 再次击中之后新的空闲概率整数值(uint16)
    const std::vector<uint16> hit_table_;
    // miss_table_[原来的空闲概率整数值(uint16)] = 再次空闲(没有被击中)之后新的空闲概率整数值(uint16)
    const std::vector<uint16> miss_table_;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_RANGE_DATA_INSERTER_2D_PROBABILITY_GRID_H_
