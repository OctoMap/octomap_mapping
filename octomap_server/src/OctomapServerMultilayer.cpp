/**
* octomap_server: A Tool to serve 3D OctoMaps in ROS (binary and as visualization)
* (inspired by the ROS map_saver)
* @author A. Hornung, University of Freiburg, Copyright (C) 2010-2011.
* @see http://octomap.sourceforge.net/
* License: BSD
*/

/*
 * Copyright (c) 2010-2011, A. Hornung, University of Freiburg
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <octomap_server/OctomapServerMultilayer.h>

namespace octomap{

	OctomapServerMultilayer::OctomapServerMultilayer(const std::string& filename)
	  : OctomapServer(filename),
	    m_haveAttachedObject(false)
	{

	  m_attachedObjectsSub = m_nh.subscribe("attached_collision_object", 1, &OctomapServerMultilayer::attachedCallback, this);

	}

	OctomapServerMultilayer::~OctomapServerMultilayer(){

	}

	void OctomapServerMultilayer::attachedCallback(const arm_navigation_msgs::AttachedCollisionObjectConstPtr& msg){
	  ROS_DEBUG("AttachedCollisionObjects received");
	  m_haveAttachedObject = (msg->object.operation.operation == arm_navigation_msgs::CollisionObjectOperation::ATTACH_AND_REMOVE_AS_OBJECT);
	  if(m_haveAttachedObject){
	    m_attachedFrame = msg->link_name;
	    m_attachedMaxOffset = msg->object.poses.back().position.z + msg->object.shapes.back().dimensions[1];
	    m_attachedMinOffset = msg->object.poses.back().position.z - msg->object.shapes.back().dimensions[1];
	  }
	}

	void OctomapServerMultilayer::handlePreNodeTraversal(const ros::Time& rostime){

	  // TODO: set up 2D map
	}

	void OctomapServerMultilayer::handlePostNodeTraversal(const ros::Time& rostime){

	  // TODO: publish multilayer maps

	}

	void OctomapServerMultilayer::handleOccupiedNode(const OcTreeROS::OcTreeType::iterator& it){

	  // TODO: set map pixel in layers

	}

	void OctomapServerMultilayer::handleFreeNode(const OcTreeROS::OcTreeType::iterator& it){

	  // TODO: set map pixel in layers

	}
}



