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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_MOTION_FILTER_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_MOTION_FILTER_H_

#include <limits>

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/proto/motion_filter_options.pb.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace mapping {

proto::MotionFilterOptions CreateMotionFilterOptions(
        common::LuaParameterDictionary* parameter_dictionary);

// Takes poses as input and filters them to get fewer poses.
////////////////////////////////////////////////////////////////////
/// \brief The MotionFilter class
/// \brief 主要用于判断是否与上个位姿足够接近
/// ///////////////////////////////////////////////////////////////
class MotionFilter {
public:
    explicit MotionFilter(const proto::MotionFilterOptions& options);

    // If the accumulated motion (linear, rotational, or time) is above the
    // threshold, returns false. Otherwise the relative motion is accumulated and
    // true is returned.

    // 判断是否与上个位姿足够接近:
    // (1)如果不是第一次调用 &&
    // (2)(给定时间戳-上次调用的时间)< 配置选项中设定的最大时间  &&
    // (3)(给定位姿的平移量 - 上一个位姿的平移量)的模长 < 配置选项中的最大距离(米) &&
    // (4)前后两次的旋转量之差 < 配置选项中的最大角度
    // 如果都满足,  表示与上一个位姿比较接近 ===>返回: true
    //
    // 否则, 记录输入进来的时间戳和pose 作为上一个位姿的记录 , 然后返回 false , 表示输入的位姿与上个位姿相差较大
    bool IsSimilar(common::Time time, const transform::Rigid3d& pose);

private:
    const proto::MotionFilterOptions options_;
    int num_total_ = 0;
    int num_different_ = 0;
    common::Time last_time_;
    transform::Rigid3d last_pose_;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_MOTION_FILTER_H_
