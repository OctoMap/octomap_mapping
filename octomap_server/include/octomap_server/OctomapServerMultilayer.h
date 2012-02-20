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


#include <octomap_server/OctomapServer.h>
#include <arm_navigation_msgs/AttachedCollisionObject.h>

namespace octomap {
	class OctomapServerMultilayer : public OctomapServer{

	public:
		OctomapServerMultilayer(const std::string& filename= "");
		virtual ~OctomapServerMultilayer();
		void attachedCallback(const arm_navigation_msgs::AttachedCollisionObjectConstPtr& msg);

	protected:

		/// hook that is called after traversing all nodes
		void handlePreNodeTraversal(const ros::Time& rostime);

		/// hook that is called when traversing all nodes of the updated Octree (does nothing here)
		void handleNode(const OcTreeROS::OcTreeType::iterator& it) {};

		/// hook that is called when traversing occupied nodes of the updated Octree (updates 2D map projection here)
		void handleOccupiedNode(const OcTreeROS::OcTreeType::iterator& it);

		/// hook that is called when traversing free nodes of the updated Octree (updates 2D map projection here)
		void handleFreeNode(const OcTreeROS::OcTreeType::iterator& it);

		/// hook that is called after traversing all nodes
		void handlePostNodeTraversal(const ros::Time& rostime);

		ros::Subscriber m_attachedObjectsSub;

    std::string m_attachedFrame;
    double m_attachedMaxOffset;
    double m_attachedMinOffset;
		bool m_haveAttachedObject;


	};
}

