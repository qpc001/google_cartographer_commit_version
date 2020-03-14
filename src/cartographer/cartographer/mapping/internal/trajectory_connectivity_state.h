/*
 * Copyright 2017 The Cartographer Authors
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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_TRAJECTORY_CONNECTIVITY_STATE_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_TRAJECTORY_CONNECTIVITY_STATE_H_

#include "cartographer/common/time.h"
#include "cartographer/mapping/internal/connected_components.h"

namespace cartographer {
namespace mapping {

// A class that tracks the connectivity state between trajectories. Compared to
// ConnectedComponents it tracks additionally the last time that a global
// constraint connected to trajectories.
//
// This class is thread-compatible.
/////////////////////////////////////////////////////////////////////////////
/// \brief The TrajectoryConnectivityState class
/// \brief 用于跟踪轨迹之间的连接关系,
///        与 `ConnectedComponents`类相比, 这个还跟踪上一次连接到轨迹的全局约束
/// @brief 这个类是线程安全的
/// @brief 好像没怎么用这个...
/// /////////////////////////////////////////////////////////////////////////
class TrajectoryConnectivityState {
 public:
  TrajectoryConnectivityState() {}

  TrajectoryConnectivityState(const TrajectoryConnectivityState&) = delete;
  TrajectoryConnectivityState& operator=(const TrajectoryConnectivityState&) =
      delete;

  // Add a trajectory which is initially connected to only itself.
  // 添加一条 起始只与自己本身相连接的轨迹
  // 在pose_graph_2d.cc用到
  void Add(int trajectory_id);

  // Connect two trajectories. If either trajectory is untracked, it will be
  // tracked. This function is invariant to the order of its arguments. Repeated
  // calls to Connect increment the connectivity count and update the last
  // connected time.
  // 连接两条轨迹, 如果其中一条轨迹没有在跟踪,那么它也会被跟踪, 函数对参数顺序不敏感
  // 重复的调用增加连接计数，并更新最后的连接时间
  // 在pose_graph_2d.cc用到
  void Connect(int trajectory_id_a, int trajectory_id_b, common::Time time);

  // Determines if two trajectories have been (transitively) connected. If
  // either trajectory is not being tracked, returns false, except when it is
  // the same trajectory, where it returns true. This function is invariant to
  // the order of its arguments.
  // 检查?两条轨迹是否直接(或间接)地连接起来, 如果其中一条轨迹没有被跟踪,直接返回false,
  // 除非是同一条轨迹, 那么会返回true, 函数对参数顺序不敏感
  /// 没见到在哪里用到这个函数
  bool TransitivelyConnected(int trajectory_id_a, int trajectory_id_b) const;

  // The trajectory IDs, grouped by connectivity.
  /// 没见到在哪里用到这个函数
  std::vector<std::vector<int>> Components() const;

  // Return the last connection count between the two trajectories. If either of
  // the trajectories is untracked or they have never been connected returns the
  // beginning of time.
  // 返回两条轨迹之间的连接计数, 如果其中一条没有被跟踪或者它们之间从没有连接,则返回 开始的时间?
  /// 没见到在哪里用到这个函数
  common::Time LastConnectionTime(int trajectory_id_a, int trajectory_id_b);

 private:
  // ConnectedComponents are thread safe.
  mutable ConnectedComponents connected_components_;

  // Tracks the last time a direct connection between two trajectories has
  // been added. The exception is when a connection between two trajectories
  // connects two formerly unconnected connected components. In this case all
  // bipartite trajectories entries for these components are updated with the
  // new connection time.

  // 储存最后一次添加两个轨迹之间的直接连接
  std::map<std::pair<int, int>, common::Time> last_connection_time_map_;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_TRAJECTORY_CONNECTIVITY_STATE_H_
