/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Square Robot, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Square Robot, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
#include "octomap_server/SquareOcTreeStamped.h"

namespace octomap {
std::shared_ptr<unsigned int> SquareOcTreeNodeStamped::time = std::make_shared<unsigned int>(0);

SquareOcTreeStamped::SquareOcTreeStamped(double in_resolution)
  : OccupancyOcTreeBase<SquareOcTreeNodeStamped>(in_resolution) {
  SquareOcTreeStampedMemberInit.ensureLinking();
}

unsigned int SquareOcTreeStamped::getLastUpdateTime() {
  return time_last_updated;
}

void SquareOcTreeStamped::updateTime(unsigned int t) {
  time_last_updated = t;
  if (this->root) {
    root->setTime(t);
  }
}

void SquareOcTreeStamped::degradeOutdatedNodes(unsigned int time_thres, unsigned int current_time) {
  for (leaf_iterator it = this->begin_leafs(), end = this->end_leafs(); it != end; ++it) {
    if (this->isNodeOccupied(*it) && ((current_time - it->getTimestamp()) > time_thres)) {
      OccupancyOcTreeBase<SquareOcTreeNodeStamped>::updateNodeLogOdds(&*it, prob_miss_log);
    }
  }
}

void SquareOcTreeStamped::removeStaleNodes(unsigned int epoch) {
  std::deque<OcTreeKey> keys_to_remove;
  for (leaf_iterator it = this->begin_leafs(); it != this->end_leafs(); ++it) {
    if (it->getTimestamp() < epoch) {
      keys_to_remove.push_back(it.getKey());
    }
  }
  for (auto k : keys_to_remove) {
    this->deleteNode(k);
  }
}

void SquareOcTreeStamped::updateNodeLogOdds(SquareOcTreeNodeStamped* node, const float& update) const {
  OccupancyOcTreeBase<SquareOcTreeNodeStamped>::updateNodeLogOdds(node, update);
  node->updateTimestamp();
}

SquareOcTreeStamped::StaticMemberInitializer SquareOcTreeStamped::SquareOcTreeStampedMemberInit;
}  // namespace octomap