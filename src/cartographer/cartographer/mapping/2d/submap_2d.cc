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

#include "cartographer/mapping/2d/submap_2d.h"

#include <cinttypes>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <limits>

#include "Eigen/Geometry"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/port.h"
#include "cartographer/mapping/2d/probability_grid_range_data_inserter_2d.h"
#include "cartographer/mapping/range_data_inserter_interface.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

proto::SubmapsOptions2D CreateSubmapsOptions2D(
        common::LuaParameterDictionary* const parameter_dictionary) {
    proto::SubmapsOptions2D options;
    options.set_num_range_data(
                parameter_dictionary->GetNonNegativeInt("num_range_data"));
    *options.mutable_grid_options_2d() = CreateGridOptions2D(
                parameter_dictionary->GetDictionary("grid_options_2d").get());
    *options.mutable_range_data_inserter_options() =
            CreateRangeDataInserterOptions(
                parameter_dictionary->GetDictionary("range_data_inserter").get());

    bool valid_range_data_inserter_grid_combination = false;
    const proto::GridOptions2D_GridType& grid_type =
            options.grid_options_2d().grid_type();
    const proto::RangeDataInserterOptions_RangeDataInserterType&
            range_data_inserter_type =
            options.range_data_inserter_options().range_data_inserter_type();
    if (grid_type == proto::GridOptions2D::PROBABILITY_GRID &&
            range_data_inserter_type ==
            proto::RangeDataInserterOptions::PROBABILITY_GRID_INSERTER_2D) {
        valid_range_data_inserter_grid_combination = true;
    }
    CHECK(valid_range_data_inserter_grid_combination)
            << "Invalid combination grid_type " << grid_type
            << " with range_data_inserter_type " << range_data_inserter_type;
    CHECK_GT(options.num_range_data(), 0);
    return options;
}

// 构造子图(原点, 栅格)
Submap2D::Submap2D(const Eigen::Vector2f& origin, std::unique_ptr<Grid2D> grid)
    : Submap(transform::Rigid3d::Translation(Eigen::Vector3d(origin.x(), origin.y(), 0.))) //初始化父类, 子图的坐标系原点在全局坐标系的坐标
{
    // 概率网格传值
    grid_ = std::move(grid);
}
////////////////非重点////////////////////////////////
Submap2D::Submap2D(const proto::Submap2D& proto)
    : Submap(transform::ToRigid3(proto.local_pose())) {
    if (proto.has_grid()) {
        CHECK(proto.grid().has_probability_grid_2d());
        grid_ = common::make_unique<ProbabilityGrid>(proto.grid());
    }
    set_num_range_data(proto.num_range_data());
    set_finished(proto.finished());
}

void Submap2D::ToProto(proto::Submap* const proto,
                       bool include_probability_grid_data) const {
    auto* const submap_2d = proto->mutable_submap_2d();
    *submap_2d->mutable_local_pose() = transform::ToProto(local_pose());
    submap_2d->set_num_range_data(num_range_data());
    submap_2d->set_finished(finished());
    if (include_probability_grid_data) {
        CHECK(grid_);
        *submap_2d->mutable_grid() = grid_->ToProto();
    }
}

void Submap2D::UpdateFromProto(const proto::Submap& proto) {
    CHECK(proto.has_submap_2d());
    const auto& submap_2d = proto.submap_2d();
    set_num_range_data(submap_2d.num_range_data());
    set_finished(submap_2d.finished());
    if (proto.submap_2d().has_grid()) {
        CHECK(proto.submap_2d().grid().has_probability_grid_2d());
        grid_ = common::make_unique<ProbabilityGrid>(submap_2d.grid());
    }
}

void Submap2D::ToResponseProto(
        const transform::Rigid3d&,
        proto::SubmapQuery::Response* const response) const {
    if (!grid_) return;
    response->set_submap_version(num_range_data());
    proto::SubmapQuery::Response::SubmapTexture* const texture =
            response->add_textures();
    grid()->DrawToSubmapTexture(texture, local_pose());
}
//////////////////////////////////////////////////////////

// 插入激光数据(更新子图概率地图)
void Submap2D::InsertRangeData(
        const sensor::RangeData& range_data,                        //激光扫描
        const RangeDataInserterInterface* range_data_inserter) {    //虽然这里是父类,但是实际传进来的是子类:ProbabilityGridRangeDataInserter2D
    CHECK(grid_);
    CHECK(!finished());
    // ProbabilityGridRangeDataInserter2D::Insert 插入激光
    range_data_inserter->Insert(range_data, grid_.get());
    //设置激光数据数量
    set_num_range_data(num_range_data() + 1);
}

// 完成子图
void Submap2D::Finish() {
    CHECK(grid_);
    CHECK(!finished()); //检查子图是否已经完成,不需要重复完成
    // 裁剪子图
    grid_ = grid_->ComputeCroppedGrid();
    // 设置标志位
    set_finished(true);
}

// 构造函数
ActiveSubmaps2D::ActiveSubmaps2D(const proto::SubmapsOptions2D& options)
    : options_(options),                                            //配置文件选项
      range_data_inserter_(std::move(CreateRangeDataInserter())) {  //基于配置文件创建ProbabilityGridRangeDataInserter2D 激光插入器

    // We always want to have at least one likelihood field which we can return,
    // and will create it at the origin in absence of a better choice.
    // 我们总是希望至少有一个我们可以返回的可能域，并且在没有更好选择的情况下在原点创建它
    // 初始化: 在tracking坐标系原点添加一个子图
    AddSubmap(Eigen::Vector2f::Zero());
}

std::vector<std::shared_ptr<Submap2D>> ActiveSubmaps2D::submaps() const {
    return submaps_;
}

// 返回扫描匹配所使用的子图的索引
int ActiveSubmaps2D::matching_index() const { return matching_submap_index_; }

// 插入激光数据
// 如果子图列表最后一个元素插入了足够的激光数据,(表示可以用来作为扫描匹配了), 则进行子图新旧替换过程
void ActiveSubmaps2D::InsertRangeData(const sensor::RangeData& range_data) {
    // 遍历submaps_列表中的子图 (不是说只在新的子图插入激光数据吗? 为什么要遍历)
    for (auto& submap : submaps_) {
        // 插入激光数据(更新子图概率地图)
        // 如果这些激光数据都是在local 坐标系下的, 那么子图也是local map坐标系下的
        submap->InsertRangeData(range_data, range_data_inserter_.get());
    }
    // 如果子图列表最后一个元素插入了足够的激光数据,(表示可以用来作为扫描匹配了)
    // 那么往submaps_ 添加新的子图 ,  使用激光数据的原点作为新的子图的原点
    if (submaps_.back()->num_range_data() == options_.num_range_data()) {
        // range_data.origin.head<2>() : 激光坐标系到 local 坐标系的转换的平移 ,作为子图的原点
        AddSubmap(range_data.origin.head<2>());
    }
}

// 创建激光扫描插入器
std::unique_ptr<RangeDataInserterInterface>
ActiveSubmaps2D::CreateRangeDataInserter() {
    return common::make_unique<ProbabilityGridRangeDataInserter2D>(
                options_.range_data_inserter_options()
                .probability_grid_range_data_inserter_options_2d());
}

// 给定子图坐标系原点, 创建一个新的概率地图
std::unique_ptr<GridInterface> ActiveSubmaps2D::CreateGrid(
        const Eigen::Vector2f& origin) {
    // 给定一个初始的cell数量限制
    constexpr int kInitialSubmapSize = 100;
    float resolution = options_.grid_options_2d().resolution(); //从配置文件获取分辨率

    // MapLimits (分辨率,最大值(单位:米),cell数量限制)
    return common::make_unique<ProbabilityGrid>(
                MapLimits(resolution,
                          origin.cast<double>() + 0.5 * kInitialSubmapSize * resolution *Eigen::Vector2d::Ones(),
                          CellLimits(kInitialSubmapSize, kInitialSubmapSize)));
}

// FinishSubmap(): submaps_丢掉一个旧的子图指针(不一定把整个子图丢弃了), 空出一个新的位置
void ActiveSubmaps2D::FinishSubmap() {
    // 获取子图队列的第一个子图
    Submap2D* submap = submaps_.front().get();
    // 裁剪这个子图
    submap->Finish();
    // 增加matching_submap_index_记录当前新图索引
    ++matching_submap_index_;
    // std::vector<std::shared_ptr<Submap2D>> submaps_;
    // 由于这个列表储存的是shared_ptr<Submap2D> , 所以erace()只是把这个指针从这个队列移除, 但是可能还有其他地方正在维护这个子图指针
    // 所以, 这只是丢掉这个子图指针, 不是丢掉整个子图
    submaps_.erase(submaps_.begin());
}

// 添加子图 (给定子图的坐标系原点)
void ActiveSubmaps2D::AddSubmap(const Eigen::Vector2f& origin) {
    // 如果正在维护的子图数量>1 , 那么在添加新的子图之前
    if (submaps_.size() > 1) {
        // This will crop the finished Submap before inserting a new Submap to
        // reduce peak memory usage a bit.
        // FinishSubmap(): submaps_丢掉一个旧的子图指针(不一定把整个子图丢弃了), 空出一个新的位置
        FinishSubmap();
    }
    // CreateGrid(origin) : 给定子图坐标系原点, 创建一个新的概率地图
    // 最终产生 <Submap2D> , 然后push到子图列表
    submaps_.push_back(common::make_unique<Submap2D>(
                           origin, std::unique_ptr<Grid2D>(
                               static_cast<Grid2D*>(CreateGrid(origin).release()))));
    LOG(INFO) << "Added submap " << matching_submap_index_ + submaps_.size();
}

}  // namespace mapping
}  // namespace cartographer
