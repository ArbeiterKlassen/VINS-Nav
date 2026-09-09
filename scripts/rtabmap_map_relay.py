#!/usr/bin/env python3
"""
rtabmap_map_relay.py -- Bridge RTAB-Map's live grid_map to /map + /static_map.

RTAB-Map publishes /rtabmap/grid_map (nav_msgs/OccupancyGrid) but does NOT
provide a /static_map service.  map_to_pc2 (from ego-planner) needs /static_map
to fetch the map and convert it to PointCloud2 for the ESDF constructor.

This node:
  1. Subscribes to /rtabmap/grid_map
  2. Republishes to /map (latched, for RViz and ego_planner's GridMap)
  3. Advertises a /static_map service that returns the latest cached map

This enables the ONE-SESSION workflow:
  Launch VIO+RTAB-Map+ego-planner → drive to map → set navigation goals
  -- all without saving/restarting.
"""

import rospy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap, GetMapResponse


class RTABMapRelay:
    def __init__(self):
        rospy.init_node("rtabmap_map_relay")

        self.latest_map = None

        # Subscribe to RTAB-Map's live grid map
        rospy.Subscriber("/rtabmap/grid_map", OccupancyGrid, self.map_cb, queue_size=1)

        # Republich to the standard /map topic (latched, for RViz and GridMap)
        self.map_pub = rospy.Publisher("/map", OccupancyGrid, queue_size=1, latch=True)

        # Advertise /static_map service (used by map_to_pc2)
        self.srv = rospy.Service("/static_map", GetMap, self.handle_static_map)

        rospy.loginfo("RTAB-Map Relay ready: /rtabmap/grid_map -> /map + /static_map")

    def map_cb(self, msg):
        self.latest_map = msg
        self.map_pub.publish(msg)

    def handle_static_map(self, req):
        if self.latest_map is None:
            rospy.logwarn_throttle(10, "No map received from RTAB-Map yet")
            return GetMapResponse()
        return GetMapResponse(self.latest_map)


if __name__ == "__main__":
    try:
        RTABMapRelay()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
