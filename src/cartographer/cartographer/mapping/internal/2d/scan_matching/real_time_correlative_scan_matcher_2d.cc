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

#include "cartographer/mapping/internal/2d/scan_matching/real_time_correlative_scan_matcher_2d.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>

#include "Eigen/Geometry"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/math.h"
#include "cartographer/mapping/2d/probability_grid.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {

// 构造函数  : 利用配置选项创建 RealTimeCorrelativeScanMatcher2D
RealTimeCorrelativeScanMatcher2D::RealTimeCorrelativeScanMatcher2D(
        const proto::RealTimeCorrelativeScanMatcherOptions& options)
    : options_(options) {}

// 给定搜索参数,生成所有的搜索候选 (也就是所有可能的解)
std::vector<Candidate2D>
RealTimeCorrelativeScanMatcher2D::GenerateExhaustiveSearchCandidates(
        const SearchParameters& search_parameters) const {                  //搜索参数
    // 候选数量
    int num_candidates = 0;
    // 遍历
    for (int scan_index = 0; scan_index != search_parameters.num_scans;++scan_index) {
        // 搜索窗口x方向总步数
        const int num_linear_x_candidates =
                (search_parameters.linear_bounds[scan_index].max_x -
                 search_parameters.linear_bounds[scan_index].min_x + 1);
        // 搜索窗口y方向总补数
        const int num_linear_y_candidates =
                (search_parameters.linear_bounds[scan_index].max_y -
                 search_parameters.linear_bounds[scan_index].min_y + 1);
        // 全部角度 全部搜索窗口 总步数
        num_candidates += num_linear_x_candidates * num_linear_y_candidates;
    }
    //
    std::vector<Candidate2D> candidates;
    candidates.reserve(num_candidates);
    // 遍历
    for (int scan_index = 0; scan_index != search_parameters.num_scans;++scan_index) {
        // x方向遍历
        for (int x_index_offset = search_parameters.linear_bounds[scan_index].min_x;
             x_index_offset <= search_parameters.linear_bounds[scan_index].max_x;
             ++x_index_offset) {
            // y方向遍历
            for (int y_index_offset =
                 search_parameters.linear_bounds[scan_index].min_y;
                 y_index_offset <= search_parameters.linear_bounds[scan_index].max_y;
                 ++y_index_offset) {
                // scan_index : 搜索角度的索引
                // x_index_offset :  与给定初始位姿 x方向上的偏移 [这是cell上的偏移]
                // y_index_offset :  与给定初始位姿 y方向上的偏移 [这是cell上的偏移]
                // 这些参数构成 1个候选的解
                // 保存到 candidates
                candidates.emplace_back(scan_index, x_index_offset, y_index_offset,
                                        search_parameters);
            }
        }
    }
    CHECK_EQ(candidates.size(), num_candidates);
    // 返回所有可能的解
    return candidates;
}

// 输入初始位姿(机器人在local map坐标系的坐标) , 激光扫描 , 子图概率地图
// 进行旋转\平移匹配, 得到得分最高的候选解, 输出 位姿估计pose_estimate
double RealTimeCorrelativeScanMatcher2D::Match(
        const transform::Rigid2d& initial_pose_estimate,        //初始位姿(机器人在子图的坐标,真实距离,(单位:米))
        const sensor::PointCloud& point_cloud,                  //点云
        const ProbabilityGrid& probability_grid,                //概率地图 (是来自子图的吗?)
        transform::Rigid2d* pose_estimate) const {              //输出的位姿估计
    CHECK_NOTNULL(pose_estimate);

    // 取给定初始位姿的旋转
    const Eigen::Rotation2Dd initial_rotation = initial_pose_estimate.rotation();
    // 先对点云进行旋转
    const sensor::PointCloud rotated_point_cloud = sensor::TransformPointCloud(
                point_cloud,
                transform::Rigid3f::Rotation(Eigen::AngleAxisf(
                                                 initial_rotation.cast<float>().angle(), Eigen::Vector3f::UnitZ())));
    // 生成搜索参数 (角度搜索的总步数 , 角度搜索步长 , 分辨率 , 每个角度下的平移搜索窗口)
    const SearchParameters search_parameters(
                options_.linear_search_window(), options_.angular_search_window(),
                rotated_point_cloud, probability_grid.limits().resolution());

    // 根据搜索参数 , 对激光扫描进行旋转变换
    // 返回 各个搜索角度下的激光扫描数据 (实际上就是对激光扫描进行旋转, 在角度搜索范围内旋转)
    const std::vector<sensor::PointCloud> rotated_scans =
            GenerateRotatedScans(rotated_point_cloud, search_parameters);
    // 对上面各个搜索角度下的激光扫描数据进行离散化处理
    // DiscretizeScans() : 遍历输入的多帧激光扫描数据
    // 对每一帧: 遍历该帧扫描的每一个激光点 , 对每一个点进行平移 , 然后计算其所在cell的索引
    // 返回离散化的激光扫描: 即每一帧点云的每个激光点所在的cell的索引
    const std::vector<DiscreteScan2D> discrete_scans = DiscretizeScans(
                probability_grid.limits(), rotated_scans,
                Eigen::Translation2f(initial_pose_estimate.translation().x(),
                                     initial_pose_estimate.translation().y()));
    // 根据搜索参数, 生成所有的候选解
    std::vector<Candidate2D> candidates =
            GenerateExhaustiveSearchCandidates(search_parameters);
    // 遍历候选解, 得到根据概率地图计算候选解的得分
    ScoreCandidates(probability_grid, discrete_scans, search_parameters,
                    &candidates);

    // 取得分最高的候选解
    const Candidate2D& best_candidate =
            *std::max_element(candidates.begin(), candidates.end());

    // 根据候选解 与给定初始位姿的平移量偏移(真实距离,米) 以及 角度量偏移
    // 计算出基于最优候选解的最优机器人位姿 , 赋值到pose_estimate
    *pose_estimate = transform::Rigid2d(
    {initial_pose_estimate.translation().x() + best_candidate.x,
     initial_pose_estimate.translation().y() + best_candidate.y},
                initial_rotation * Eigen::Rotation2Dd(best_candidate.orientation));
    // 输出最优候选解的得分
    return best_candidate.score;
}

// 遍历候选解 , 计算候选解的得分
void RealTimeCorrelativeScanMatcher2D::ScoreCandidates(
        const ProbabilityGrid& probability_grid,                //概率地图
        const std::vector<DiscreteScan2D>& discrete_scans,      //离散化2D扫描
        const SearchParameters& search_parameters,              //搜索参数
        std::vector<Candidate2D>* const candidates) const {     //所有候选解

    // 遍历候选解 , 计算候选解的得分
    for (Candidate2D& candidate : *candidates) {
        candidate.score = 0.f;
        // 取其中一帧角度索引为scan_index的离散化2D激光扫描数据
        // 遍历该帧的每一个激光点
        for (const Eigen::Array2i& xy_index : discrete_scans[candidate.scan_index]) {
            // 该激光点 + 上候选解candidate 所携带的偏移量
            // 得到 以候选解为坐标原点(点云中心) 的激光扫描点的 cell索引
            const Eigen::Array2i proposed_xy_index(
                        xy_index.x() + candidate.x_index_offset,
                        xy_index.y() + candidate.y_index_offset);
            // 根据该激光点的cell索引 , 获取概率地图上对应cell的占据概率[0~1]
            const float probability =
                    probability_grid.GetProbability(proposed_xy_index);
            // 更新该候选解的得分
            candidate.score += probability;
        }
        // 取平均
        candidate.score /=
                static_cast<float>(discrete_scans[candidate.scan_index].size());
        // std::hypot =sqrt(x*x + y*y) 这里用来计算 候选解与给定初始位姿的平移模长 (真实距离 (米))
        // options_.translation_delta_cost_weight() : 平移量 cost 权重 ,
        // 如果权重越大 , 或者平移量偏移 旋转量偏移越大, 那么这个候选解的总得分越小
        // score= score * [-(平移量偏移*权重1 + 旋转量*权重2)^2]
        candidate.score *=
                std::exp(-common::Pow2(std::hypot(candidate.x, candidate.y) *
                                       options_.translation_delta_cost_weight() +
                                       std::abs(candidate.orientation) *
                                       options_.rotation_delta_cost_weight()));
        CHECK_GT(candidate.score, 0.f);
    }
}

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer
