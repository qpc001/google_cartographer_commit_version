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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_2D_RAY_CASTING_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_2D_RAY_CASTING_H_

#include <vector>

#include "cartographer/common/port.h"
#include "cartographer/mapping/2d/probability_grid.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/sensor/range_data.h"
#include "cartographer/transform/transform.h"

namespace cartographer {
namespace mapping {

///栅格地图是按照一定的分辨率把坐标空间划分称为一个个的方格子，障碍点一定会落在某个方格内。
/// 如右图中的灰色带叉的区域，我们说在这些栅格上观测到了一次hit事件，称之为hit栅格。
/// 而从传感器所在的栅格到hit栅格之间所有栅格就应该是空闲的，如右图中的灰色区域，
/// 在这些栅格上观测到了一次miss事件，称之为miss栅格。图中白色的区域表示在这次扫描过程中，
/// 对应的扇区方向上没有发现障碍，并不意味着这些栅格就是空闲的，所以称为未知栅格。
/// 我把这种从扫描得到的距离信息转换为栅格的hit或者miss事件的过程称为RayCasting

/** @brief 所谓的 hit_table_
 * 0. 就是说 某个栅格, 原来有着一个占用概率<=====>占用概率整数值
 * 1. 当激光击中这个栅格的时候 , 不是需要对这个栅格的占用概率进行更新吗 ?
 * 2. 这时候这个 hit_table_就发挥作用了 :
 * 3. hit_table_[0~32767] = [0~32767]之间的某个数值 ====> hit_table_ 实际上就是一个整数的映射表, 继续看下面
 * 4. 给定某个栅格的占用概率整数值(uint16) , 如果该栅格再次被激光击中了 , 那么它新的占用概率值 , 其实是可以预先计算出来的
 * 5. hit_table_ 里面的东西, 就是: 栅格原有的占用概率整数值(uint16) =====> 再次击中之后新的占用概率整数值(uint16)
 * 7. (划重点): hit_table_[原有的占用概率整数值(uint16)] = 再次击中之后新的占用概率整数值(uint16)
 * 8. 这就不难解释为什么 hit_table_[0~32767] = [0~32767]之间的某个数值
 **/

// 对于每一条射线, 计算hit 和miss  ,先计算 hits
// 根据激光扫描, 对hit中的栅格进行空闲概率更新(相当于更新占用概率)
// 如果需要, 则根据标志位对 射线上的没有击中的栅格也进行空闲概率更新
void CastRays(const sensor::RangeData& range_data,          //激光数据
              const std::vector<uint16>& hit_table,         //击中的table
              const std::vector<uint16>& miss_table,        //没击中的table
              bool insert_free_space,                       //是否对射线上的没有击中的栅格也进行空闲概率更新
              ProbabilityGrid* probability_grid);           //概率地图

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_2D_RAY_CASTING_H_
