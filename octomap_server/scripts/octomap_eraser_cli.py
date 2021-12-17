#!/usr/bin/env python

# Copyright 2011, A. Hornung, University of Freiburg. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Willow Garage nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Clear a region specified by a global axis-aligned bounding box in stored OctoMap."""

import sys

from geometry_msgs.msg import Point
from octomap_msgs.srv import BoundingBoxQuery
import rclpy
from rclpy.node import Node


class OctomapEraserCli(Node):

    def __init__(self):
        super().__init__('octomap_erase_cli')
        self.cli = self.create_client(BoundingBoxQuery, '/octomap_server/clear_bbox')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = BoundingBoxQuery.Request()
        min_point = Point(*[float(x) for x in sys.argv[1:4]])
        max_point = Point(*[float(x) for x in sys.argv[4:7]])
        self.req.min = min_point
        self.req.max = max_point
        self.future = self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    cli = OctomapEraserCli()

    while rclpy.ok():
        rclpy.spin_once(cli)
        if cli.future.done():
            if cli.future.result() is not None:
                cli.get_logger().info('Connect to service')
            else:
                cli.get_logger().error(
                    'Exception while calling service'
                )
            break

    cli.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
