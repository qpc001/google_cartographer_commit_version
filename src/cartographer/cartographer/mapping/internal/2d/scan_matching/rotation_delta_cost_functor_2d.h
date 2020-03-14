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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_ROTATION_DELTA_COST_FUNCTOR_2D_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_ROTATION_DELTA_COST_FUNCTOR_2D_H_

#include "Eigen/Core"
#include "ceres/ceres.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {

// Computes the cost of rotating 'pose' to 'target_angle'. Cost increases with
// the solution's distance from 'target_angle'.
class RotationDeltaCostFunctor2D {
public:
    static ceres::CostFunction* CreateAutoDiffCostFunction(
            const double scaling_factor,        //权重因子
            const double target_angle) {        //初始位姿的角度
        return new ceres::AutoDiffCostFunction<
                RotationDeltaCostFunctor2D, 1 /* residuals 误差维度 */, 3 /* pose variables 待优化变量维度 */>(
                    // 创建RotationDeltaCostFunctor2D类对象 主要是进行 成员变量初始化
                    new RotationDeltaCostFunctor2D(scaling_factor, target_angle));
    }

    template <typename T>
    bool operator()(const T* const pose, T* residual) const {
        // pose: 待优化位姿
        // residual 残差 =  待优化位姿的角度 - 初始位姿的角度  (就是约束,不能跑太偏了)
        residual[0] = scaling_factor_ * (pose[2] - angle_);
        return true;
    }

private:
    explicit RotationDeltaCostFunctor2D(const double scaling_factor,
                                        const double target_angle)
        : scaling_factor_(scaling_factor), angle_(target_angle) {}

    RotationDeltaCostFunctor2D(const RotationDeltaCostFunctor2D&) = delete;
    RotationDeltaCostFunctor2D& operator=(const RotationDeltaCostFunctor2D&) =
            delete;

    const double scaling_factor_;   //权重因子
    const double angle_;            //初始位姿的角度
};

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_ROTATION_DELTA_COST_FUNCTOR_2D_H_
