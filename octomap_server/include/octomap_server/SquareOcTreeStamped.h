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

#ifndef SRC_EXTERNAL_OCTOMAP_MAPPING_OCTOMAP_SERVER_INCLUDE_OCTOMAP_SERVER_SQUAREOCTREESTAMPED_H_
#define SRC_EXTERNAL_OCTOMAP_MAPPING_OCTOMAP_SERVER_INCLUDE_OCTOMAP_SERVER_SQUAREOCTREESTAMPED_H_

#include <octomap/OcTreeNode.h>
#include <octomap/OccupancyOcTreeBase.h>
#include <memory>

namespace octomap {

// node definition
class SquareOcTreeNodeStamped : public OcTreeNode {
public:
  SquareOcTreeNodeStamped() : OcTreeNode(), timestamp(0) {
  }

  SquareOcTreeNodeStamped(const SquareOcTreeNodeStamped& rhs) : OcTreeNode(rhs), timestamp(rhs.timestamp) {
  }

  bool operator==(const SquareOcTreeNodeStamped& rhs) const {
    return (rhs.value == value && rhs.timestamp == timestamp);
  }

  void copyData(const SquareOcTreeNodeStamped& from) {
    OcTreeNode::copyData(from);
    timestamp = from.getTimestamp();
  }

  // timestamp
  inline uint32_t getTimestamp() const {
    return timestamp;
  }

  inline void updateTimestamp() {
    timestamp = time;
  }

  inline void setTimestamp(uint32_t t) {
    timestamp = t;
  }

  inline void setTime(uint32_t t) {
    time = t;
  }

  inline void updateOccupancyChildren() {
    this->setLogOdds(this->getMaxChildLogOdds());  // conservative
    updateTimestamp();
  }

protected:
  uint32_t timestamp;
  static uint32_t time;
};

// tree definition
class SquareOcTreeStamped : public OccupancyOcTreeBase<SquareOcTreeNodeStamped> {
private:
  uint32_t time_last_updated;

public:
  // Default Constructor
  explicit SquareOcTreeStamped(double resolution);

  // Virtual Constructor
  SquareOcTreeStamped* create() const {
    return new SquareOcTreeStamped(resolution);
  }

  std::string getTreeType() const {
    return "SquareOcTreeStamped";
  }

  uint32_t getLastUpdateTime();
  void updateTime(uint32_t t);

  // Probabilistically degrades all occupied nodes last updated more than time_thresh seconds ago
  void degradeOutdatedNodes(uint32_t time_thresh, uint32_t current_time);
  // Removes all nodes last updated before the given epoch
  void removeStaleNodes(uint32_t epoch);
  // Updates the log odds of a node by update
  void updateNodeLogOdds(SquareOcTreeNodeStamped* node, const float& update) const override;

protected:
  /**
   * Static member object which ensures that this OcTree's prototype
   * ends up in the classIDMapping only once. You need this as a
   * static member in any derived octree class in order to read .ot
   * files through the AbstractOcTree factory. You should also call
   * ensureLinking() once from the constructor.
   */
  class StaticMemberInitializer {
  public:
    StaticMemberInitializer() {
      SquareOcTreeStamped* tree = new SquareOcTreeStamped(0.1);
      tree->clearKeyRays();
      AbstractOcTree::registerTreeType(tree);
    }
    void ensureLinking() {
    }
  };
  /// to ensure static initialization (only once)
  static StaticMemberInitializer SquareOcTreeStampedMemberInit;
};

}  // namespace octomap

#endif  // SRC_EXTERNAL_OCTOMAP_MAPPING_OCTOMAP_SERVER_INCLUDE_OCTOMAP_SERVER_SQUAREOCTREESTAMPED_H_