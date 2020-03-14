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

#include "cartographer/mapping/probability_values.h"

#include "cartographer/common/make_unique.h"

namespace cartographer {
namespace mapping {

namespace {

// 0 is unknown, [1, 32767] maps to [lower_bound, upper_bound].
// 把1_32767的整数值映射到0~1的概率值
float SlowValueToBoundedFloat(const uint16 value, const uint16 unknown_value,
                              const float unknown_result,
                              const float lower_bound,
                              const float upper_bound) {
  CHECK_GE(value, 0);
  CHECK_LE(value, 32767);
  if (value == unknown_value) return unknown_result;
  const float kScale = (upper_bound - lower_bound) / 32766.f;
  return value * kScale + (lower_bound - kScale);
}

// 把[1,32767]之间的所有value预先计算出来其映射到[lower_bound, upper_bound]这个区间
// 的对应浮点值，存到一个浮点型向量中：
std::unique_ptr<std::vector<float>> PrecomputeValueToBoundedFloat(
    const uint16 unknown_value, const float unknown_result,
    const float lower_bound, const float upper_bound) {
  auto result = common::make_unique<std::vector<float>>();
  // Repeat two times, so that both values with and without the update marker
  // can be converted to a probability.
  for (int repeat = 0; repeat != 2; ++repeat) {
    for (int value = 0; value != 32768; ++value) {
      result->push_back(SlowValueToBoundedFloat(
          value, unknown_value, unknown_result, lower_bound, upper_bound));
    }
  }
  return result;
}

// 把[1,32767]之间的所有value预先计算出来其映射到[lower_bound, upper_bound]这个区间的对应浮点值，存到一个浮点型向量中：
// 这里是把占据概率对应的整数值 预先计算, 映射到[kMinProbability,kMaxProbability]之间
// 即返回一个数组, 一共[32768]个元素, 每个元素对应一个占据概率值, 值区间为[kMinProbability,kMaxProbability]
std::unique_ptr<std::vector<float>> PrecomputeValueToProbability() {
  return PrecomputeValueToBoundedFloat(kUnknownProbabilityValue,
                                       kMinProbability, kMinProbability,
                                       kMaxProbability);
}

// 这里是把空闲概率对应的整数值 预先计算, 映射到[kMinCorrespondenceCost,kMaxCorrespondenceCost]之间
// 即返回一个数组, 一共[32768]个元素, 每个元素对应一个空闲概率值, 值区间为[kMinCorrespondenceCost,kMaxCorrespondenceCost]
std::unique_ptr<std::vector<float>> PrecomputeValueToCorrespondenceCost() {
  return PrecomputeValueToBoundedFloat(
      kUnknownCorrespondenceValue, kMaxCorrespondenceCost,
      kMinCorrespondenceCost, kMaxCorrespondenceCost);
}

}  // namespace

// 一个数组, 一共[32768]个元素, 每个元素对应一个占据概率值, 值区间为[kMinProbability,kMaxProbability]
const std::vector<float>* const kValueToProbability =
    PrecomputeValueToProbability().release();

// 一个数组, 一共[32768]个元素, 每个元素对应一个空闲概率值, 值区间为[kMinCorrespondenceCost,kMaxCorrespondenceCost]
const std::vector<float>* const kValueToCorrespondenceCost =
    PrecomputeValueToCorrespondenceCost().release();

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
// 这个函数暂时只在初始化table的时候用到
// 调用时, odds为配置选项的参数 (把它看成1或者0.9好了)
// 所以会返回一个初始化的hit_table
std::vector<uint16> ComputeLookupTableToApplyOdds(const float odds) {
  std::vector<uint16> result;
  // ProbabilityFromOdds(odds) : odd 转 占据概率
  // ProbabilityToValue(ProbabilityFromOdds(odds)): 占据概率转uint16值
  // 占据概率的整数空间 : [0]表示未知状态, 单独进行更新, 然后push到 输出列表
  result.push_back(ProbabilityToValue(ProbabilityFromOdds(odds)) +
                   kUpdateMarker);
  // 下面开始遍历[1,32768] , 计算出新的odds之后, 反求新的占据概率, 再映射回整数空间
  for (int cell = 1; cell != 32768; ++cell) {
    result.push_back(ProbabilityToValue(ProbabilityFromOdds(
                         odds * Odds((*kValueToProbability)[cell]))) +  //odds * Odds((*kValueToProbability)[cell]) 得到新的Odds
                     kUpdateMarker);
    //这里加的`kUpdateMarker`在函数Grid2D::FinishUpdate()会重新减掉
  }
  // 这样,得到一个新的数组result[0~32768]= uint16值   // 有什么用????
  return result;
}

// 同上 : 调用时, odds为配置选项的参数 (把它看成1或者0.9好了) 选项文件:probability_grid_range_data_inserter_options_2d.proto
// 所以会返回一个初始化的miss_table
std::vector<uint16> ComputeLookupTableToApplyCorrespondenceCostOdds(
    float odds) {
  std::vector<uint16> result;
  result.push_back(CorrespondenceCostToValue(ProbabilityToCorrespondenceCost(
                       ProbabilityFromOdds(odds))) +
                   kUpdateMarker);
  for (int cell = 1; cell != 32768; ++cell) {
    result.push_back(
        CorrespondenceCostToValue(
            ProbabilityToCorrespondenceCost(ProbabilityFromOdds(
                odds * Odds(CorrespondenceCostToProbability(
                           (*kValueToCorrespondenceCost)[cell]))))) +
        kUpdateMarker);
  }
  return result;
}

}  // namespace mapping
}  // namespace cartographer
