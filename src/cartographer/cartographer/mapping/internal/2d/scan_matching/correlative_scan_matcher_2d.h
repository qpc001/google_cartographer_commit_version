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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_CORRELATIVE_SCAN_MATCHER_2D_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_CORRELATIVE_SCAN_MATCHER_2D_H_

#include <vector>

#include "Eigen/Core"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/mapping/2d/map_limits.h"
#include "cartographer/mapping/2d/xy_index.h"
#include "cartographer/sensor/point_cloud.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {

typedef std::vector<Eigen::Array2i> DiscreteScan2D;

// Describes the search space.
// 搜索空间的描述
struct SearchParameters {
    // Linear search window in pixel offsets; bounds are inclusive.
    // 搜索窗口 , 包含边界
    struct LinearBounds {
        int min_x;
        int max_x;
        int min_y;
        int max_y;
    };

    // 生成搜索参数 (角度搜索的总步数 , 角度搜索步长 , 分辨率 , 每个角度下的平移搜索窗口)
    SearchParameters(double linear_search_window, double angular_search_window,
                     const sensor::PointCloud& point_cloud, double resolution);

    // For testing. (不看)
    SearchParameters(int num_linear_perturbations, int num_angular_perturbations,
                     double angular_perturbation_step_size, double resolution);

    // Tightens the search window as much as possible.
    void ShrinkToFit(const std::vector<DiscreteScan2D>& scans,
                     const CellLimits& cell_limits);

    int num_angular_perturbations;          // 计算角度的总步数 = 向上取整[(角度搜索范围)/(步长)]
    double angular_perturbation_step_size;  // 按照论文公式 的角度搜索步长
    double resolution;                      // 分辨率
    int num_scans;                          // 总的角度搜索步数
    std::vector<LinearBounds> linear_bounds;  // Per rotated scans. 每个角度下的平移搜索窗口
};

// Generates a collection of rotated scans.
// 对激光扫描进行旋转变换
// 返回储存着 num_scans 个角度下的激光扫描数据的vector
std::vector<sensor::PointCloud> GenerateRotatedScans(
        const sensor::PointCloud& point_cloud,
        const SearchParameters& search_parameters);

// Translates and discretizes the rotated scans into a vector of integer
// indices.
// 遍历输入的多帧激光扫描数据
// 对每一帧: 遍历该帧扫描的每一个激光点 , 对每一个点进行平移 , 然后计算其所在cell的索引
// 返回离散化的激光扫描: 即每一帧点云的每个激光点所在的cell的索引
std::vector<DiscreteScan2D> DiscretizeScans(
        const MapLimits& map_limits, const std::vector<sensor::PointCloud>& scans,
        const Eigen::Translation2f& initial_translation);

// A possible solution.
// 一个可能的解
struct Candidate2D {
    Candidate2D(const int init_scan_index,                      //对应哪一个旋转角度的索引
                const int init_x_index_offset,                  //相对于初始值的x偏移量
                const int init_y_index_offset,                  //相对于初始值的y偏移量
                const SearchParameters& search_parameters)      //搜索空间
        : // 初始化成员变量
          scan_index(init_scan_index),
          x_index_offset(init_x_index_offset),                  // 与给定初始值的x平移偏移量 [这是cell上的偏移]
          y_index_offset(init_y_index_offset),                  // 与给定初始值的y平移偏移量 [这是cell上的偏移]
          x(-y_index_offset * search_parameters.resolution),    // 与初始位姿的相对关系 [这是真实距离,单位是米]
          y(-x_index_offset * search_parameters.resolution),    // 负号以及xy翻转 是图像与真实空间的转换
          orientation((scan_index - search_parameters.num_angular_perturbations) *
                      search_parameters.angular_perturbation_step_size) {}

    // Index into the rotated scans vector.
    // 表明这个解 对应哪一个旋转角度
    int scan_index = 0;

    // Linear offset from the initial pose.
    // 与给定初始值的平移偏移量 [这是cell上的偏移]
    int x_index_offset = 0;
    int y_index_offset = 0;

    // Pose of this Candidate2D relative to the initial pose.
    // 与 初始化位姿 (粗位姿) 的相对关系 [这是真实距离,单位是米]
    double x = 0.;
    double y = 0.;
    double orientation = 0.;

    // Score, higher is better.
    // 这个解对应的得分
    float score = 0.f;

    // 操作符 , 比较得分
    bool operator<(const Candidate2D& other) const { return score < other.score; }
    bool operator>(const Candidate2D& other) const { return score > other.score; }
};

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_CORRELATIVE_SCAN_MATCHER_2D_H_
