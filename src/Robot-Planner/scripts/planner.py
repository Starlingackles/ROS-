#!/usr/bin/env python
import rospy
import numpy as np
import math
import random
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Float32MultiArray
from tf.transformations import euler_from_quaternion

def calc_distance(p_1, p_2):
    return math.sqrt((p_2[0]-p_1[0])**2 + (p_2[1]-p_1[1])**2)

def point_to_line(p_1, p_2, p_3):
    dist = math.sqrt((p_2[0] - p_1[0])**2 + (p_2[1] - p_1[1])**2)

    # determine intersection ratio u
    # for three points A, B with a line between them and a third point C, the tangent to the line AB
    # passing through C intersects the line AB a distance along its length equal to u*|AB|
    r_u = ((p_3[0] - p_1[0])*(p_2[0] - p_1[0]) + (p_3[1] - p_1[1])*(p_2[1] - p_1[1]))/(dist**2)

    # intersection point
    p_i = (p_1[0] + r_u*(p_2[0] - p_1[0]), p_1[1] + r_u*(p_2[1] - p_1[1]))

    # distance from P3 to intersection point
    tan_len = calc_distance(p_i, p_3)

    return r_u, tan_len

class MapProcessor:
    def __init__(self):
        self.map_data = None
        self.map_info = None
        self.obstacles = []
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)

    def map_callback(self, msg):
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_info = msg.info
        self.extract_obstacles()
        rospy.loginfo("Map loaded with resolution %.3f at origin (%.2f, %.2f)" % (
            self.map_info.resolution, 
            self.map_info.origin.position.x,
            self.map_info.origin.position.y))

    def extract_obstacles(self):
        self.obstacles = []
        for y in range(self.map_info.height):
            for x in range(self.map_info.width):
                if self.map_data[y][x] > 50:  # 障碍物阈值
                    world_x = x * self.map_info.resolution + self.map_info.origin.position.x
                    world_y = y * self.map_info.resolution + self.map_info.origin.position.y
                    self.obstacles.append((world_x, world_y))

    def world_to_map(self, point):
        x = int((point.x - self.map_info.origin.position.x) / self.map_info.resolution)
        y = int((point.y - self.map_info.origin.position.y) / self.map_info.resolution)
        return (x, y)

    def is_collision(self, p1, p2):
        # ... todo

class RRTStarPlanner:
    def __init__(self):
        self.mp = MapProcessor()
        self.start = None
        self.goal = None
        self.nodes = []
        
        # 双路径发布器
        self.vis_path_pub = rospy.Publisher('/rrt_path', Path, queue_size=10)  # 可视化用
        self.ctrl_path_pub = rospy.Publisher('/path', Float32MultiArray, queue_size=10)  # 控制用
        self.tree_pub = rospy.Publisher('/rrt_tree', MarkerArray, queue_size=10)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        
        rospy.wait_for_message('/map', OccupancyGrid)
        rospy.loginfo("Planner initialized, waiting for goal...")

    class Node:
        def __init__(self, point, parent=None):
            self.point = point
            self.parent = parent
            self.cost = 0.0 if parent is None else parent.cost + math.hypot(
                point.x - parent.point.x, point.y - parent.point.y)

    def goal_callback(self, msg):
        self.goal = msg.pose.position
        if self.mp.map_info is not None:
            self.start = Point(0, 0, 0)  # 强制起点为(0,0)以匹配控制器
            self.plan_path()

    def plan_path(self):
        reached=False
        
        # ...todo
        



    


    def publish_path(self, goal_node):
        # 可视化Path构建
        vis_path = Path()
        vis_path.header.frame_id = "map"
        vis_path.header.stamp = rospy.Time.now()
        
        # 控制用路径数据构建
        ctrl_path = Float32MultiArray()
        path_points = []
        
        # 收集路径点（从终点到起点）
        current = goal_node
        while current is not None:
            # 可视化Path的点
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position = current.point
            vis_path.poses.append(pose)
            
            # 控制用路径点
            path_points.append((current.point.x, current.point.y))
            current = current.parent
        
        # 反转路径顺序（起点到终点）
        vis_path.poses.reverse()
        path_points.reverse()
        
        # 填充控制路径数据
        for point in path_points:
            ctrl_path.data.extend([point[0], point[1]])
        
        # 同时发布两种格式
        self.vis_path_pub.publish(vis_path)
        self.ctrl_path_pub.publish(ctrl_path)
        rospy.loginfo("Published both path formats:\n" +
                     f"- Visual Path ({len(vis_path.poses)} poses)\n" +
                     f"- Control Path ({len(path_points)} points)")
        
        self.publish_tree()

    def publish_tree(self):
        marker_array = MarkerArray()
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "rrt_tree"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.03
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.5
        
        for node in self.nodes:
            if node.parent:
                marker.points.append(node.parent.point)
                marker.points.append(node.point)
        
        marker_array.markers.append(marker)
        self.tree_pub.publish(marker_array)

if __name__ == '__main__':
    rospy.init_node('rrt_star_planner')
    planner = RRTStarPlanner()
    rospy.spin()