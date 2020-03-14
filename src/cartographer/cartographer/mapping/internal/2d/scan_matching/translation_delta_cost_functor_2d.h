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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_TRANSLATION_DELTA_COST_FUNCTOR_2D_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_TRANSLATION_DELTA_COST_FUNCTOR_2D_H_

#include "Eigen/Core"
#include "ceres/ceres.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {

// Computes the cost of translating 'pose' to 'target_translation'.
// Cost increases with the solution's distance from 'target_translation'.
class TranslationDeltaCostFunctor2D {
public:
    static ceres::CostFunction* CreateAutoDiffCostFunction(
            const double scaling_factor,                    //权重因子
            const Eigen::Vector2d& target_translation) {    //初始位姿的平移量
        return new ceres::AutoDiffCostFunction<TranslationDeltaCostFunctor2D,
                2 /* residuals 残差维度*/,
                3 /* pose variables 待优化参数维度*/>(
                    // 利用传进来的参数构造 TranslationDeltaCostFunctor2D类 , 主要是进行成员变量初始化
                    new TranslationDeltaCostFunctor2D(scaling_factor, target_translation));
    }

    template <typename T>
    bool operator()(const T* const pose, T* residual) const {
        // pose: 待优化变量
        // residual: 残差 = 待优化变量 [x,y] - 初始位姿的平移量[x,y] ===> (就是约束,不能跑太偏了)
        residual[0] = scaling_factor_ * (pose[0] - x_);
        residual[1] = scaling_factor_ * (pose[1] - y_);
        return true;
    }

private:
    // Constructs a new TranslationDeltaCostFunctor2D from the given
    // 'target_translation' (x, y).
    // 构造函数 , 由static ceres::CostFunction* CreateAutoDiffCostFunction 静态构造
    explicit TranslationDeltaCostFunctor2D(
            const double scaling_factor,                //权重尺度
            const Eigen::Vector2d& target_translation)  //初始位姿的平移量
        : // 初始化成员变量
          scaling_factor_(scaling_factor),
          x_(target_translation.x()),
          y_(target_translation.y()) {}

    TranslationDeltaCostFunctor2D(const TranslationDeltaCostFunctor2D&) = delete;
    TranslationDeltaCostFunctor2D& operator=(
            const TranslationDeltaCostFunctor2D&) = delete;

    const double scaling_factor_;   //权重尺度
    const double x_;                //初始位姿的x平移量
    const double y_;                //初始位姿的y平移量
};

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_TRANSLATION_DELTA_COST_FUNCTOR_2D_H_
