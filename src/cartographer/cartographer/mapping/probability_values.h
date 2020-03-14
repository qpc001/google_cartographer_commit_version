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

#ifndef CARTOGRAPHER_MAPPING_PROBABILITY_VALUES_H_
#define CARTOGRAPHER_MAPPING_PROBABILITY_VALUES_H_

#include <cmath>
#include <vector>

#include "cartographer/common/math.h"
#include "cartographer/common/port.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

//////////////////////////////////////////////////////////////////////////////////
/// \brief 关于一些概率和整数空间的映射和计算
/// \brief 需要重点掌握
//////////////////////////////////////////////////////////////////////////////////
namespace {

// 将float类型转换为uint16类型
inline uint16 BoundedFloatToValue(const float float_value,
                                  const float lower_bound,
                                  const float upper_bound) {
    // Clamp: 将val截取到区间min至max中.
    const int value =
            common::RoundToInt(
                (common::Clamp(float_value, lower_bound, upper_bound) - lower_bound) *
                (32766.f / (upper_bound - lower_bound))) +
            1;
    // DCHECK for performance.
    DCHECK_GE(value, 1);
    DCHECK_LE(value, 32767);
    return value;
}

}  // namespace

// 根据占据概率计算出Odds的值  odds = 占据概率/(1-占据概率) = 占据概率/空闲概率
inline float Odds(float probability) {
    return probability / (1.f - probability);
}

// 从odds反求占据概率
inline float ProbabilityFromOdds(const float odds) {
    return odds / (odds + 1.f);
}

// 空闲概率
inline float ProbabilityToCorrespondenceCost(const float probability) {
    return 1.f - probability;
}

// 占据概率
inline float CorrespondenceCostToProbability(const float correspondence_cost) {
    return 1.f - correspondence_cost;
}

constexpr float kMinProbability = 0.1f;                         //占据概率最小值
constexpr float kMaxProbability = 1.f - kMinProbability;        //占据概率最大值
constexpr float kMinCorrespondenceCost = 1.f - kMaxProbability; //空闲概率最小值
constexpr float kMaxCorrespondenceCost = 1.f - kMinProbability; //空闲概率最大值

// Clamps probability to be in the range [kMinProbability, kMaxProbability].
// 对占据概率进行截断
inline float ClampProbability(const float probability) {
    return common::Clamp(probability, kMinProbability, kMaxProbability);
}
// Clamps correspondece cost to be in the range [kMinCorrespondenceCost,
// kMaxCorrespondenceCost].
// 对空闲概率进行截断
inline float ClampCorrespondenceCost(const float correspondence_cost) {
    return common::Clamp(correspondence_cost, kMinCorrespondenceCost,
                         kMaxCorrespondenceCost);
}

constexpr uint16 kUnknownProbabilityValue = 0;                              //未知占据概率的值 0
constexpr uint16 kUnknownCorrespondenceValue = kUnknownProbabilityValue;    //未知空闲概率的值 0
constexpr uint16 kUpdateMarker = 1u << 15;                              //无符号整形1, 左移15位=32768

// Converts a correspondence_cost to a uint16 in the [1, 32767] range.
// 将float类型转换为uint16类型 : 将空闲概率向整数区间映射
inline uint16 CorrespondenceCostToValue(const float correspondence_cost) {
    return BoundedFloatToValue(correspondence_cost, kMinCorrespondenceCost,
                               kMaxCorrespondenceCost);
}

// Converts a probability to a uint16 in the [1, 32767] range.
// 将float类型转换为uint16类型 : 将占据概率向整数区间映射
inline uint16 ProbabilityToValue(const float probability) {
    return BoundedFloatToValue(probability, kMinProbability, kMaxProbability);
}

extern const std::vector<float>* const kValueToProbability;
extern const std::vector<float>* const kValueToCorrespondenceCost;

// Converts a uint16 (which may or may not have the update marker set) to a
// probability in the range [kMinProbability, kMaxProbability].
// 整数区间value向浮点数[占据概率]映射
inline float ValueToProbability(const uint16 value) {
    return (*kValueToProbability)[value];
}

// Converts a uint16 (which may or may not have the update marker set) to a
// correspondence cost in the range [kMinCorrespondenceCost,
// kMaxCorrespondenceCost].
// 整数区间value向浮点数[空闲概率]映射
inline float ValueToCorrespondenceCost(const uint16 value) {
    return (*kValueToCorrespondenceCost)[value];
}

// 输入占据概率的uint16整数值, 返回 空闲概率的uint16整数值
inline uint16 ProbabilityValueToCorrespondenceCostValue(
        uint16 probability_value) {
    //如果是Unknown值还返回unknown值。Probability和CorrespondenceCost的Unknown值都是0
    if (probability_value == kUnknownProbabilityValue) {
        return kUnknownCorrespondenceValue;
    }
    bool update_carry = false;
    //如果该值超过最大范围：
    if (probability_value > kUpdateMarker) {
        probability_value -= kUpdateMarker; //防止溢出范围
        update_carry = true;    //如果存在过超出范围的行为，则将update_carry置为true
    }
    // 完成转换
    uint16 result = CorrespondenceCostToValue(
                ProbabilityToCorrespondenceCost(ValueToProbability(probability_value)));
    // 原先由于(超出范围)减去过一个最大范围，现在再加回来
    if (update_carry) result += kUpdateMarker;
    return result;
}

// 输入空闲概率的uint16整数值, 返回 占据概率的uint16整数值
inline uint16 CorrespondenceCostValueToProbabilityValue(
        uint16 correspondence_cost_value) {
    if (correspondence_cost_value == kUnknownCorrespondenceValue)
        return kUnknownProbabilityValue;
    bool update_carry = false;
    if (correspondence_cost_value > kUpdateMarker) {
        correspondence_cost_value -= kUpdateMarker;
        update_carry = true;
    }
    uint16 result = ProbabilityToValue(CorrespondenceCostToProbability(
                                           ValueToCorrespondenceCost(correspondence_cost_value)));
    if (update_carry) result += kUpdateMarker;
    return result;
}

std::vector<uint16> ComputeLookupTableToApplyOdds(float odds);
std::vector<uint16> ComputeLookupTableToApplyCorrespondenceCostOdds(float odds);

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_PROBABILITY_VALUES_H_
