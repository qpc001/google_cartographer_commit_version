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

#ifndef CARTOGRAPHER_MAPPING_2D_SUBMAP_2D_H_
#define CARTOGRAPHER_MAPPING_2D_SUBMAP_2D_H_

#include <memory>
#include <vector>

#include "Eigen/Core"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/mapping/2d/grid_2d.h"
#include "cartographer/mapping/2d/map_limits.h"
#include "cartographer/mapping/proto/2d/submaps_options_2d.pb.h"
#include "cartographer/mapping/proto/serialization.pb.h"
#include "cartographer/mapping/proto/submap_visualization.pb.h"
#include "cartographer/mapping/range_data_inserter_interface.h"
#include "cartographer/mapping/submaps.h"
#include "cartographer/mapping/trajectory_node.h"
#include "cartographer/sensor/range_data.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace mapping {

proto::SubmapsOptions2D CreateSubmapsOptions2D(
        common::LuaParameterDictionary* parameter_dictionary);

// 2D子图类
class Submap2D : public Submap {
public:
    /// Submap2D 基本上就是父类Submap ,
    /// 新增了InsertRangeData()插入激光数据函数
    /// 新增了Finish()
    /// 新增成员变量: 概率地图 grid_

    // 构造函数(子图原点(在全局坐标系的坐标?), 概率地图)
    Submap2D(const Eigen::Vector2f& origin, std::unique_ptr<Grid2D> grid);

    // (不是重点)从proto中读取子图
    explicit Submap2D(const proto::Submap2D& proto);
    // (不是重点)转换到proto
    void ToProto(proto::Submap* proto,
                 bool include_probability_grid_data) const override;
    // (不是重点)从proto中更新
    void UpdateFromProto(const proto::Submap& proto) override;
    // (不是重点)
    void ToResponseProto(const transform::Rigid3d& global_submap_pose,
                         proto::SubmapQuery::Response* response) const override;

    // 返回概率地图
    const Grid2D* grid() const { return grid_.get(); }

    // Insert 'range_data' into this submap using 'range_data_inserter'. The
    // submap must not be finished yet.
    // 使用range_data_inserter 插入激光数据
    void InsertRangeData(const sensor::RangeData& range_data,
                         const RangeDataInserterInterface* range_data_inserter);
    // 结束
    void Finish();

private:
    std::unique_ptr<Grid2D> grid_;
};

// Except during initialization when only a single submap exists, there are
// always two submaps into which range data is inserted: an old submap that is
// used for matching, and a new one, which will be used for matching next, that
// is being initialized.
// 除了在初始化的时候只有单个子图, 否则一般情况下将维护两张子图
// 1. 一份旧的子图用于扫描匹配
// 2. 新的一份子图(也就是新插入激光数据的)用于下一次扫描匹配
//
// Once a certain number of range data have been inserted, the new submap is
// considered initialized: the old submap is no longer changed, the "new" submap
// is now the "old" submap and is used for scan-to-map matching. Moreover, a
// "new" submap gets created. The "old" submap is forgotten by this object.
// 一旦插入了一定数量的激光扫描, 新的子图被考虑使用来初始化: 即替换掉旧的子图, 用于' scan-to-map ' 匹配
// 另外又会重新创建一份更新的子图

/// 在类LocalTrajectoryBuilder2D中，通过对象active_submaps_来维护子图，
/// 它是一个ActiveSubmaps2D类型的数据。
/// 除了刚开始构建该对象的时候，只有一个子图(Submap2D)，其他时候它都维护着两个子图对象。
/// 一个子图用于进行扫描匹配，称为旧图。另一个子图被称为新图用于插入扫描数据。
/// 当新图中插入一定数量的数据完成了初始化操作之后，它就会被当作旧图，用于扫描匹配。
/// 对象active_submaps_将抛弃原来的旧图，并重新构建一个新图。
class ActiveSubmaps2D {
public:
    // 激活的2D子图
    explicit ActiveSubmaps2D(const proto::SubmapsOptions2D& options);

    ActiveSubmaps2D(const ActiveSubmaps2D&) = delete;
    ActiveSubmaps2D& operator=(const ActiveSubmaps2D&) = delete;

    // Returns the index of the newest initialized Submap which can be
    // used for scan-to-map matching.
    // 返回用于 (scan-to-map的匹配)的子图 索引, 也就是旧的子图索引
    int matching_index() const;

    // Inserts 'range_data' into the Submap collection.
    // 插入激光数据
    void InsertRangeData(const sensor::RangeData& range_data);

    // 返回子图
    std::vector<std::shared_ptr<Submap2D>> submaps() const;

private:
    // 创建激光数据插入器
    std::unique_ptr<RangeDataInserterInterface> CreateRangeDataInserter();
    // 给定子图坐标系原点, 创建一个新的概率地图
    std::unique_ptr<GridInterface> CreateGrid(const Eigen::Vector2f& origin);
    // FinishSubmap(): submaps_丢掉一个旧的子图指针(不一定把整个子图丢弃了), 空出一个新的位置
    void FinishSubmap();
    // 添加子图 (子图的原点是: 激光坐标系在tracking 坐标系下的坐标)
    void AddSubmap(const Eigen::Vector2f& origin);

    // 2D子图选项
    const proto::SubmapsOptions2D options_;
    // 用于 (scan-to-map的匹配)的子图 索引, 也就是旧的子图索引
    int matching_submap_index_ = 0;
    // 子图列表
    std::vector<std::shared_ptr<Submap2D>> submaps_;
    // 激光插入器
    std::unique_ptr<RangeDataInserterInterface> range_data_inserter_;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_SUBMAP_2D_H_
