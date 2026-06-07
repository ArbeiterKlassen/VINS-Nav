#!/usr/bin/env python3
"""Frontier-based auto-exploration. No sklearn needed - uses cv2.connectedComponents."""
import rospy
import numpy as np
import cv2
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray
import tf2_ros
import math

class FrontierExplorer:
    def __init__(self):
        self.grid_map = None
        self.goal_active = False
        self.tf_buf = tf2_ros.Buffer(rospy.Duration(10))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf)

        rospy.Subscriber('/rtabmap/grid_map', OccupancyGrid, self.map_cb)
        rospy.Subscriber('/move_base/status', GoalStatusArray, self.status_cb)
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)

        self.visited_goals = set()
        self.min_frontier_size = 8
        rospy.loginfo("Frontier Explorer ready (no sklearn)")
        rospy.sleep(5)  # wait for move_base + TF to come up

    def map_cb(self, msg):
        self.grid_map = msg

    def status_cb(self, msg):
        if msg.status_list:
            s = msg.status_list[-1].status
            self.goal_active = (s in [1, 3])

    def get_robot_pose(self):
        try:
            t = self.tf_buf.lookup_transform('map', 'base_footprint',
                                              rospy.Time(0), rospy.Duration(2.0))
            return t.transform.translation.x, t.transform.translation.y
        except:
            return None

    def find_frontiers(self):
        if self.grid_map is None:
            return []
        data = np.array(self.grid_map.data, dtype=np.int8).reshape(
            self.grid_map.info.height, self.grid_map.info.width)
        res = self.grid_map.info.resolution
        ox = self.grid_map.info.origin.position.x
        oy = self.grid_map.info.origin.position.y

        free = (data == 0)
        unknown = (data == -1)
        kernel = np.ones((3, 3), np.uint8)
        unknown_dilated = cv2.dilate(unknown.astype(np.uint8), kernel)
        frontier_mask = (free & (unknown_dilated > 0)).astype(np.uint8)

        if frontier_mask.sum() < self.min_frontier_size:
            return []

        n_labels, labels = cv2.connectedComponents(frontier_mask, connectivity=8)
        clusters = []
        for lbl in range(1, n_labels):
            ys, xs = np.where(labels == lbl)
            if len(ys) < self.min_frontier_size:
                continue
            cx = np.mean(xs) * res + ox
            cy = np.mean(ys) * res + oy
            # Shrink goal slightly into free space so move_base can reach it
            rx, ry = self.get_robot_pose() or (0, 0)
            dx, dy = cx - rx, cy - ry
            dist = math.hypot(dx, dy) or 1.0
            shrink = min(0.5, dist * 0.3)  # shrink 30% toward robot, max 0.5m
            cx -= dx / dist * shrink
            cy -= dy / dist * shrink
            clusters.append({'x': cx, 'y': cy, 'size': len(ys)})
        return clusters

    def score_clusters(self, clusters, rx, ry):
        scored = []
        for c in clusters:
            dist = math.sqrt((c['x']-rx)**2 + (c['y']-ry)**2)
            score = c['size'] / (1.0 + dist)
            key = (int(c['x']*10), int(c['y']*10))
            if key in self.visited_goals:
                score *= 0.1
            scored.append((score, c))
        scored.sort(key=lambda x: x[0], reverse=True)
        return [c for _, c in scored]

    def send_goal(self, x, y):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)
        self.visited_goals.add((int(x*10), int(y*10)))

    def run(self):
        rate = rospy.Rate(0.5)
        max_goals = rospy.get_param('~max_goals', 30)
        goal_count = 0

        while not rospy.is_shutdown() and goal_count < max_goals:
            robot = self.get_robot_pose()
            if robot is None:
                rospy.logwarn_throttle(10, "Waiting for robot pose (TF map->base_footprint)...")
                rate.sleep()
                continue
            rx, ry = robot

            if self.goal_active:
                rate.sleep()
                continue

            clusters = self.find_frontiers()
            if not clusters:
                rospy.loginfo_throttle(5, "No frontiers found - waiting for map to expand...")
                rate.sleep()
                continue

            scored = self.score_clusters(clusters, rx, ry)
            best = scored[0]
            rospy.loginfo("Goal %d/%d: (%.2f,%.2f) size=%d dist=%.1fm",
                          goal_count+1, max_goals, best['x'], best['y'],
                          best['size'], math.hypot(best['x']-rx, best['y']-ry))
            self.send_goal(best['x'], best['y'])
            goal_count += 1
            rate.sleep()

        rospy.loginfo("Exploration done. %d goals sent.", goal_count)

if __name__ == '__main__':
    rospy.init_node('frontier_explore')
    FrontierExplorer().run()
