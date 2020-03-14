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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_OCCUPIED_SPACE_COST_FUNCTION_2D_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_OCCUPIED_SPACE_COST_FUNCTION_2D_H_

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/mapping/2d/grid_2d.h"
#include "cartographer/mapping/probability_values.h"
#include "cartographer/sensor/point_cloud.h"
#include "ceres/ceres.h"
#include "ceres/cubic_interpolation.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {

// Computes a cost for matching the 'point_cloud' to the 'grid' with
// a 'pose'. The cost increases with poorer correspondence of the grid and the
// point observation (e.g. points falling into less occupied space).
/// 计算 : 使用候选解位姿对激光扫描和栅格地图进行匹配的 cost
/// 如果匹配不好,那么cost会增加
class OccupiedSpaceCostFunction2D {
public:
    ////////////////////////////ceres残差项////////////////////////////////////////
    static ceres::CostFunction* CreateAutoDiffCostFunction(
            const double scaling_factor,                //输入 : 尺度因子
            const sensor::PointCloud& point_cloud,      //      激光扫描
            const Grid2D& grid) {                       //      概率地图

        //AutoDiffCostFunction<残差类型 , 输出维度(误差维度) , 输入维度>
        return new ceres::AutoDiffCostFunction<
                OccupiedSpaceCostFunction2D,    // 残差类型
                ceres::DYNAMIC ,                // residuals 输出维度(误差维度)
                3 >(                            // pose variables 输入维度
                    new OccupiedSpaceCostFunction2D(scaling_factor, point_cloud, grid),     //构造函数, 初始化成员变量
                    point_cloud.size());
    }

    // 计算残差
    template <typename T>
    bool operator()(const T* const pose, T* residual) const {
        // pose : 待优化参数
        // residual : 残差
        Eigen::Matrix<T, 2, 1> translation(pose[0], pose[1]);   //平移量 (真实距离,单位:米)
        Eigen::Rotation2D<T> rotation(pose[2]);                 //旋转量
        Eigen::Matrix<T, 2, 2> rotation_matrix = rotation.toRotationMatrix();   //旋转量 ==>旋转矩阵
        Eigen::Matrix<T, 3, 3> transform;                       //构成3x3的 2D变换矩阵T_{sub<--body}
        transform << rotation_matrix, translation, T(0.), T(0.), T(1.);

        const GridArrayAdapter adapter(grid_);
        // 对概率地图进行双三次插值 , 使得可以微分
        ceres::BiCubicInterpolator<GridArrayAdapter> interpolator(adapter);
        // 获取概率地图的MapLimits(分辨率,最大值,cell限制)
        const MapLimits& limits = grid_.limits();

        // 遍历点云
        for (size_t i = 0; i < point_cloud_.size(); ++i) {
            // Note that this is a 2D point. The third component is a scaling factor.
            // 这是一个2D激光点, 第3维是尺度因子
            const Eigen::Matrix<T, 3, 1> point((T(point_cloud_[i].x())),
                                               (T(point_cloud_[i].y())), T(1.));
            // 根据给定的初始位姿 , 将2D激光点转换到子图坐标系
            const Eigen::Matrix<T, 3, 1> world = transform * point;
            /// 这里还没仔细看: 大概是 根据激光点所在的padding之后的栅格地图的坐标 ,
            ///           返回双三次插值之后的概率地图的对应的插值函数, 即概率地图该处的空闲概率
            ///
            /// 残差residual[i] 其实就是 该栅格的空闲概率 , 空闲概率越大,表示激光击中的可能越小, 残差越大
            interpolator.Evaluate(
                        (limits.max().x() - world[0]) / limits.resolution() - 0.5 +
                    static_cast<double>(kPadding),
                    (limits.max().y() - world[1]) / limits.resolution() - 0.5 +
                    static_cast<double>(kPadding),
                    &residual[i]);

            // 残差乘以权重因子
            residual[i] = scaling_factor_ * residual[i];
        }
        return true;
    }
    /////////////////////////////////////////////////////////////////////////////////
private:
    static constexpr int kPadding = INT_MAX / 4;
    class GridArrayAdapter {
    ///GridArrayAdapter 就是对原来的栅格地图进行padding填补, 填补的层数为kPadding
    /// 得到一个假想的新地图 (grid_x_num+kPadding)x(grid_y_numk+Padding)
    public:
        enum { DATA_DIMENSION = 1 };

        // 输入概率地图 , 构造GridArrayAdapter
        explicit GridArrayAdapter(const Grid2D& grid) : grid_(grid) {}

        /// 双三次插值,需要对栅格地图进行padding ,就是在周围补上几层

        // 给定cell 索引(row,column), 需要判断是否落在padding补上的区域
        void GetValue(const int row, const int column, double* const value) const {
            // 对给定范围进行padding

            if (row < kPadding || column < kPadding || row >= NumRows() - kPadding ||
                    column >= NumCols() - kPadding) {
                // 如果给定的 (row,column) 在padding区域, (不是中心那一片)
                // 则直接对 vlaue 赋值= 空闲概率最大值
                *value = kMaxCorrespondenceCost;
            } else {
                // 否则 , 落在原来概率地图的区域
                // 返回对应cell的 '空闲概率'的float型 [0~1]
                *value = static_cast<double>(grid_.GetCorrespondenceCost(
                                                 Eigen::Array2i(column - kPadding, row - kPadding)));
            }
        }

        // 返回padding之后的行数
        int NumRows() const {
            return grid_.limits().cell_limits().num_y_cells + 2 * kPadding;
        }
        // 返回padding之后的列数
        int NumCols() const {
            return grid_.limits().cell_limits().num_x_cells + 2 * kPadding;
        }

    private:
        const Grid2D& grid_;        //传进来的概率地图
    };

    // 构造函数 : 仅仅对成员变量进行初始化
    OccupiedSpaceCostFunction2D(const double scaling_factor,
                                const sensor::PointCloud& point_cloud,
                                const Grid2D& grid)
        : scaling_factor_(scaling_factor),              //尺度因子
          point_cloud_(point_cloud),                    //激光扫描
          grid_(grid) {}                                //概率地图

    OccupiedSpaceCostFunction2D(const OccupiedSpaceCostFunction2D&) = delete;
    OccupiedSpaceCostFunction2D& operator=(const OccupiedSpaceCostFunction2D&) =
            delete;

    const double scaling_factor_;           //尺度因子
    const sensor::PointCloud& point_cloud_; //激光扫描
    const Grid2D& grid_;                    //概率地图
};

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_OCCUPIED_SPACE_COST_FUNCTION_2D_H_
