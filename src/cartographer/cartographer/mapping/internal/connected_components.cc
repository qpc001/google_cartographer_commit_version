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

#include "cartographer/mapping/internal/connected_components.h"

#include <algorithm>
#include <unordered_set>

#include "cartographer/mapping/proto/connected_components.pb.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

ConnectedComponents::ConnectedComponents()
    : lock_(), forest_(), connection_map_() {}

void ConnectedComponents::Add(const int trajectory_id) {
    // 线程锁
    common::MutexLocker locker(&lock_);
    // 很好理解,直接push进去,作为自身的初始化
    // std::map<int, int> forest_
    // 如果key已经有记录,那么这个操作不会生效 , 只有使用索引[]= 才能达到替换的效果
    forest_.emplace(trajectory_id, trajectory_id);
}

void ConnectedComponents::Connect(const int trajectory_id_a,
                                  const int trajectory_id_b) {

    common::MutexLocker locker(&lock_);
    // 建立两个轨迹ID的连接
    Union(trajectory_id_a, trajectory_id_b);
    // 排序,小的id放前面 构成pair<id_min, id_bigger>
    auto sorted_pair = std::minmax(trajectory_id_a, trajectory_id_b);
    // 连接计数+1
    ++connection_map_[sorted_pair];
}

void ConnectedComponents::Union(const int trajectory_id_a,
                                const int trajectory_id_b) {
    // 向连接森林推入两项(a,b) (b,a)
    // 如果key已经有记录,那么这个操作不会生效 , 只有使用索引[]= 才能达到替换的效果
    forest_.emplace(trajectory_id_a, trajectory_id_a);
    forest_.emplace(trajectory_id_b, trajectory_id_b);
    // 取一个能最接近轨迹a的节点
    // 下面这个操作正是因为上面的那个原因
    const int representative_a = FindSet(trajectory_id_a);
    const int representative_b = FindSet(trajectory_id_b);
    // 将两片叶子节点连接起来
    forest_[representative_a] = representative_b;
}

/// 疑问: 两个节点, 其中连接关系为: [a]=a [b]=b [a]=b [b]=a 会不会死循环?
/// 应该不会, 因为 如果key已经有记录,那么这个emplace()操作不会生效 , 只有使用索引[]= 才能达到替换的效果
///          所以,如果已经有[a]=a [b]=b
///          再使用emplace(a,b) 或者emplace(b,a), 都不会使得记录发生改变
///          此时调用FindSet(a), 会得到a
///          上面的Union()最后一步 forest_[a] = FindSet(b); 才是真正的起作用了(这种情况是两个ID都已经有记录的情况)
// 返回与trajectory_id所连接起来的树的最后一个节点(该节点还没有任何连接)
int ConnectedComponents::FindSet(const int trajectory_id) {
    // 给定轨迹id
    // 在forest_ 查找key为该ID的元素
    auto it = forest_.find(trajectory_id);
    CHECK(it != forest_.end());
    // 如果 这个元素的连接关系不是指向该轨迹本身 , 就说明有其他轨迹跟这个id的轨迹相连
    if (it->first != it->second) {
        // 取与这条轨迹相连的轨迹ID
        // 然后再次调用这个函数,一直迭代,直到找到某个还没有任何连接的轨迹ID
        // Path compression for efficiency.
        it->second = FindSet(it->second);
    }
    // 返回
    return it->second;
}

bool ConnectedComponents::TransitivelyConnected(const int trajectory_id_a,
                                                const int trajectory_id_b) {
    // 如果两条轨迹ID一样, 返回true
    if (trajectory_id_a == trajectory_id_b) {
        return true;
    }

    common::MutexLocker locker(&lock_);
    // 检查森林里面是否两条轨迹的ID都存在
    if (forest_.count(trajectory_id_a) == 0 ||
            forest_.count(trajectory_id_b) == 0) {
        // 任意一个没有,都false
        return false;
    }
    //
    return FindSet(trajectory_id_a) == FindSet(trajectory_id_b);
}

std::vector<std::vector<int>> ConnectedComponents::Components() {
    // Map from cluster exemplar -> growing cluster.
    std::unordered_map<int, std::vector<int>> map;
    common::MutexLocker locker(&lock_);
    // 遍历连接森林
    for (const auto& trajectory_id_entry : forest_) {
        // 取其中一条映射关系x
        // map[最接近(x的key)的节点].push(x的key)
        map[FindSet(trajectory_id_entry.first)].push_back(
                    trajectory_id_entry.first);
    }

    // map转vector
    std::vector<std::vector<int>> result;
    result.reserve(map.size());
    for (auto& pair : map) {
        result.emplace_back(std::move(pair.second));
    }
    return result;
}

std::vector<int> ConnectedComponents::GetComponent(const int trajectory_id) {
    common::MutexLocker locker(&lock_);
    const int set_id = FindSet(trajectory_id);
    std::vector<int> trajectory_ids;
    for (const auto& entry : forest_) {
        if (FindSet(entry.first) == set_id) {
            trajectory_ids.push_back(entry.first);
        }
    }
    return trajectory_ids;
}

int ConnectedComponents::ConnectionCount(const int trajectory_id_a,
                                         const int trajectory_id_b) {
    common::MutexLocker locker(&lock_);
    const auto it =
            connection_map_.find(std::minmax(trajectory_id_a, trajectory_id_b));
    return it != connection_map_.end() ? it->second : 0;
}

proto::ConnectedComponents ToProto(
        std::vector<std::vector<int>> connected_components) {
    proto::ConnectedComponents proto;
    for (auto& connected_component : connected_components) {
        std::sort(connected_component.begin(), connected_component.end());
    }
    std::sort(connected_components.begin(), connected_components.end());
    for (const auto& connected_component : connected_components) {
        auto* proto_connected_component = proto.add_connected_component();
        for (const int trajectory_id : connected_component) {
            proto_connected_component->add_trajectory_id(trajectory_id);
        }
    }
    return proto;
}

}  // namespace mapping
}  // namespace cartographer
