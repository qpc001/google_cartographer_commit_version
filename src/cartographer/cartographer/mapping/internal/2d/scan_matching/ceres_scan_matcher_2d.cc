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

#include "cartographer/mapping/internal/2d/scan_matching/ceres_scan_matcher_2d.h"

#include <utility>
#include <vector>

#include "Eigen/Core"
#include "cartographer/common/ceres_solver_options.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/mapping/2d/grid_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/occupied_space_cost_function_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/rotation_delta_cost_functor_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/translation_delta_cost_functor_2d.h"
#include "cartographer/transform/transform.h"
#include "ceres/ceres.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {

proto::CeresScanMatcherOptions2D CreateCeresScanMatcherOptions2D(
        common::LuaParameterDictionary* const parameter_dictionary) {
    proto::CeresScanMatcherOptions2D options;
    options.set_occupied_space_weight(
                parameter_dictionary->GetDouble("occupied_space_weight"));
    options.set_translation_weight(
                parameter_dictionary->GetDouble("translation_weight"));
    options.set_rotation_weight(
                parameter_dictionary->GetDouble("rotation_weight"));
    *options.mutable_ceres_solver_options() =
            common::CreateCeresSolverOptionsProto(
                parameter_dictionary->GetDictionary("ceres_solver_options").get());
    return options;
}

// 构造函数
CeresScanMatcher2D::CeresScanMatcher2D(
        const proto::CeresScanMatcherOptions2D& options)    //配置选项
    : //成员变量初始化
      options_(options),
      ceres_solver_options_(
          common::CreateCeresSolverOptions(options.ceres_solver_options())) {
    ceres_solver_options_.linear_solver_type = ceres::DENSE_QR;   //设置ceres求解方式
}

CeresScanMatcher2D::~CeresScanMatcher2D() {}

void CeresScanMatcher2D::Match(const Eigen::Vector2d& target_translation,           //初始位姿的平移
                               const transform::Rigid2d& initial_pose_estimate,     //初始位姿估计(来自姿态外推器或者相关性匹配)
                               const sensor::PointCloud& point_cloud,               //激光扫描点云
                               const Grid2D& grid,                                  //概率地图
                               transform::Rigid2d* const pose_estimate,             //要输出的位姿估计
                               ceres::Solver::Summary* const summary) const {       //ceres 优化报告
    // 初始化一个ceres使用的待优化位姿
    double ceres_pose_estimate[3] = {initial_pose_estimate.translation().x(),
                                     initial_pose_estimate.translation().y(),
                                     initial_pose_estimate.rotation().angle()};
    // 构造ceres优化问题
    ceres::Problem problem;
    CHECK_GT(options_.occupied_space_weight(), 0.);
    // 向problem添加残差项
    /// 1. 激光扫描与概率地图的残差项
    problem.AddResidualBlock(
                //(1)使用自动求导 cost Func
                OccupiedSpaceCostFunction2D::CreateAutoDiffCostFunction(
                    options_.occupied_space_weight() /std::sqrt(static_cast<double>(point_cloud.size())),   //权重尺度
                    point_cloud, grid), // 激光扫描 和 概率地图
                //(2)loss function 是否使用核函数
                nullptr ,
                //(3)待优化参数
                ceres_pose_estimate);
    CHECK_GT(options_.translation_weight(), 0.);

    /// 2. 平移量的残差项
    problem.AddResidualBlock(
                //(1)使用自动求导 cost Func
                TranslationDeltaCostFunctor2D::CreateAutoDiffCostFunction(
                    options_.translation_weight(), target_translation),
                //(2)loss function 是否使用核函数
                nullptr /* loss function */,
                //(3)待优化参数
                ceres_pose_estimate);
    CHECK_GT(options_.rotation_weight(), 0.);

    /// 3. 旋转量的残差项
    problem.AddResidualBlock(
                //(1)使用自动求导 cost Func
                RotationDeltaCostFunctor2D::CreateAutoDiffCostFunction(
                    options_.rotation_weight(), ceres_pose_estimate[2]),
                //(2)loss function 是否使用核函数
                nullptr /* loss function */,
                //(3)待优化参数
                ceres_pose_estimate);

    //开始优化
    ceres::Solve(ceres_solver_options_, &problem, summary);

    //取优化之后的位姿估计
    *pose_estimate = transform::Rigid2d(
    {ceres_pose_estimate[0], ceres_pose_estimate[1]}, ceres_pose_estimate[2]);
}

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer
