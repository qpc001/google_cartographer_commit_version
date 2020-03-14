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

#include "cartographer/mapping/internal/2d/scan_matching/correlative_scan_matcher_2d.h"

#include <cmath>

#include "cartographer/common/math.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {

SearchParameters::SearchParameters(const double linear_search_window,               //平移部分搜索窗口(真实距离,米)
                                   const double angular_search_window,              //角度搜索窗口
                                   const sensor::PointCloud& point_cloud,           //点云
                                   const double resolution)                         //分辨率
    : resolution(resolution) {                                                      // 设置成员变量: 分辨率
    // We set this value to something on the order of resolution to make sure that
    // the std::acos() below is defined.
    // 我们将这个值设置为按分辨率排序的值，以确保下面的std::acos()有定义

    // 先设置最远的激光距离 为 3倍分辨率
    float max_scan_range = 3.f * resolution;
    // 遍历点云
    for (const Eigen::Vector3f& point : point_cloud) {
        // 计算点云距离激光传感器的距离, 如果有比前面的3倍分辨率更大的, 则取最大值
        const float range = point.head<2>().norm();
        max_scan_range = std::max(range, max_scan_range);
    }
    // 安全偏移?
    const double kSafetyMargin = 1. - 1e-3;

    // 按照论文公式 计算角度步长
    angular_perturbation_step_size =
            kSafetyMargin * std::acos(1. - common::Pow2(resolution) /
                                      (2. * common::Pow2(max_scan_range)));
    // 计算角度搜索的单侧步数 = 向上取整[(单侧角度搜索范围)/(步长)]
    num_angular_perturbations =
            std::ceil(angular_search_window / angular_perturbation_step_size);

    // 由于角度搜索向左右两侧展开, 即 angular_search_window 只是总的角度搜索范围一半
    // 所以, 需要将激光扫描旋转 '2 * num_angular_perturbations + 1' 个角度
    num_scans = 2 * num_angular_perturbations + 1;

    // 计算平移搜索的一侧的步数 : 向上取整[(单侧平移搜索窗口)/ 分辨率]  (得到的是以cell为单位的)
    const int num_linear_perturbations =
            std::ceil(linear_search_window / resolution);

    // 每平移一步, 都应用1个完整的角度搜索
    // 所以 linear_bounds 储存 '2 * num_angular_perturbations + 1' 个角度 的平移搜索窗口
    linear_bounds.reserve(num_scans);
    for (int i = 0; i != num_scans; ++i) {
        linear_bounds.push_back(
                    LinearBounds{-num_linear_perturbations, num_linear_perturbations,
                                 -num_linear_perturbations, num_linear_perturbations});
    }
}

SearchParameters::SearchParameters(const int num_linear_perturbations,
                                   const int num_angular_perturbations,
                                   const double angular_perturbation_step_size,
                                   const double resolution)
    : num_angular_perturbations(num_angular_perturbations),
      angular_perturbation_step_size(angular_perturbation_step_size),
      resolution(resolution),
      num_scans(2 * num_angular_perturbations + 1) {
    linear_bounds.reserve(num_scans);
    for (int i = 0; i != num_scans; ++i) {
        linear_bounds.push_back(
                    LinearBounds{-num_linear_perturbations, num_linear_perturbations,
                                 -num_linear_perturbations, num_linear_perturbations});
    }
}

// 遍历各个角度下的激光数据, 对平移搜索窗口进行裁剪
void SearchParameters::ShrinkToFit(const std::vector<DiscreteScan2D>& scans,    //2D激光数据 vector<Eigen::Array2i> = DiscreteScan2D
                                   const CellLimits& cell_limits) {             // CellLimits={x轴最大cell数, y轴最大cell数}
    CHECK_EQ(scans.size(), num_scans);
    CHECK_EQ(linear_bounds.size(), num_scans);
    // 遍历角度
    for (int i = 0; i != num_scans; ++i) {
        //
        Eigen::Array2i min_bound = Eigen::Array2i::Zero();
        Eigen::Array2i max_bound = Eigen::Array2i::Zero();
        // 遍历激光数据
        for (const Eigen::Array2i& xy_index : scans[i]) {
            // 最小搜索窗口边界[x,y]取 激光点[x,y]最小的
            min_bound = min_bound.min(-xy_index);
            // 最大搜索窗口边界[x,y]取 {celllimits[x轴最大cell数-1, y轴最大cell数-1] - 激光点[x,y]}的最大值
            max_bound = max_bound.max(Eigen::Array2i(cell_limits.num_x_cells - 1,
                                                     cell_limits.num_y_cells - 1) -
                                      xy_index);
        }
        // 更新第i个角度下的平移搜索窗口的边界
        linear_bounds[i].min_x = std::max(linear_bounds[i].min_x, min_bound.x());
        linear_bounds[i].max_x = std::min(linear_bounds[i].max_x, max_bound.x());
        linear_bounds[i].min_y = std::max(linear_bounds[i].min_y, min_bound.y());
        linear_bounds[i].max_y = std::min(linear_bounds[i].max_y, max_bound.y());
    }
}

// 对激光扫描进行旋转变换
// 返回储存着 num_scans 个角度下的激光扫描数据的vector
std::vector<sensor::PointCloud> GenerateRotatedScans(
        const sensor::PointCloud& point_cloud,          //点云
        const SearchParameters& search_parameters) {    //搜索参数
    // 这是要输出的结果
    std::vector<sensor::PointCloud> rotated_scans;
    rotated_scans.reserve(search_parameters.num_scans);

    // 将角度置到最左 : 即 如果角度总搜索范围是[-a , a] , 那么就把delta_theta设置为-a
    double delta_theta = -search_parameters.num_angular_perturbations *
            search_parameters.angular_perturbation_step_size;
    // 根据步长进行角度遍历 , delta_theta每次+一个步长
    for (int scan_index = 0; scan_index < search_parameters.num_scans;
         ++scan_index,delta_theta += search_parameters.angular_perturbation_step_size) {
        // delta_theta是yaw角, 在3D的情况下就是沿z轴旋转 delta_theta 度
        // 储存旋转之后的点云到rotated_scans
        rotated_scans.push_back(sensor::TransformPointCloud(
                                    point_cloud, transform::Rigid3f::Rotation(Eigen::AngleAxisf(
                                                                                  delta_theta, Eigen::Vector3f::UnitZ()))));
    }
    // 返回容器 : 储存着 num_scans 个角度下的激光扫描数据
    return rotated_scans;
}

// 遍历输入的多帧激光扫描数据
// 对每一帧: 遍历该帧扫描的每一个激光点 , 对每一个点进行平移 , 然后计算其所在cell的索引
// 返回离散化的激光扫描: 即每一帧点云的每个激光点所在的cell的索引
std::vector<DiscreteScan2D> DiscretizeScans(
        const MapLimits& map_limits,                        //MapLimits (分辨率,最大值,cell数量限制)
        const std::vector<sensor::PointCloud>& scans,       //多帧激光点云数据
        const Eigen::Translation2f& initial_translation) {  //给定初始位姿的平移 (初始位姿: 机器人在局部子图的坐标,真实距离,米)
    // 这是要输出的东西
    std::vector<DiscreteScan2D> discrete_scans;
    discrete_scans.reserve(scans.size());
    // 遍历每一帧扫描
    for (const sensor::PointCloud& scan : scans) {
        // 在discrete_scans末尾增加一个空元素 , 该元素是std::vector<Eigen::Array2i> DiscreteScan2D 类型
        discrete_scans.emplace_back();
        discrete_scans.back().reserve(scan.size());
        for (const Eigen::Vector3f& point : scan) {
            // 遍历该帧扫描的每一个激光点
            // Eigen::Affine2f : eigen的变换函数, 相当于2D的变换矩阵T
            // MapLimits::GetCellIndex : 输入某个2D点(真实距离),返回其所在的栅格
            // 对每一个点进行平移 , 然后计算其所在cell的索引
            // 储存该激光点所在的cell的索引
            const Eigen::Vector2f translated_point =
                    Eigen::Affine2f(initial_translation) * point.head<2>();
            discrete_scans.back().push_back(
                        map_limits.GetCellIndex(translated_point));
        }
    }
    return discrete_scans;
}

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer
