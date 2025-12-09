import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import numpy as np
import threading
import math

from nav_msgs.msg import OccupancyGrid, Path, Odometry
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

# OR-Tools
try:
    from ortools.constraint_solver import routing_enums_pb2
    from ortools.constraint_solver import pywrapcp
    ORTOOLS_AVAILABLE = True
except ImportError:
    ORTOOLS_AVAILABLE = False
    print("Error: OR-Tools not installed. Please run 'pip install ortools'")

class Point2D:
    def __init__(self, x, y):
        self.x = int(x)
        self.y = int(y)
    def __repr__(self):
        return f"({self.x}, {self.y})"

class TSPExplorer(Node):
    def __init__(self):
        super().__init__('tsp_explorer_node')
        
        self.lock = threading.Lock()
        
        self.declare_parameter("replan_interval", 2.0)
        self.declare_parameter("cluster_size", 10)
        self.declare_parameter("robot_name", "av1")
        
        self.replan_interval = self.get_parameter("replan_interval").value
        self.cluster_size = self.get_parameter("cluster_size").value
        robot_name = self.get_parameter("robot_name").value

        self.create_subscription(OccupancyGrid, "/visible_map", self.map_callback, 10)
        self.create_subscription(Odometry, f"/{robot_name}/odom", self.odom_callback, 10)

        self.pcl_pub = self.create_publisher(PointCloud2, "/frontiers_cloud", 10)
        self.path_pub = self.create_publisher(Path, f"/{robot_name}/tsp_path", 10)

        self.map_data = None
        self.width = 0
        self.height = 0
        self.resolution = 0.05
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.origin_z = 0.0
        self.map_header = None
        self.map_received = False

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.odom_received = False

        self.timer = self.create_timer(self.replan_interval, self.run_pipeline)
        
        if not ORTOOLS_AVAILABLE:
            self.get_logger().error("OR-Tools library is missing! TSP will not work.")
        else:
            self.get_logger().info("TSP Explorer initialized with OR-Tools.")

    def map_callback(self, msg):
        with self.lock:
            self.width = msg.info.width
            self.height = msg.info.height
            self.resolution = msg.info.resolution
            self.origin_x = msg.info.origin.position.x
            self.origin_y = msg.info.origin.position.y
            self.origin_z = msg.info.origin.position.z
            self.map_header = msg.header
            self.map_data = np.array(msg.data, dtype=np.int8).reshape((self.height, self.width))
            self.map_received = True

    def odom_callback(self, msg):
        with self.lock:
            self.robot_x = msg.pose.pose.position.x
            self.robot_y = msg.pose.pose.position.y
            self.odom_received = True

    def run_pipeline(self):
        with self.lock:
            if not self.map_received or not self.odom_received:
                self.get_logger().info("Waiting for Map or Odom...", throttle_duration_sec=2.0)
                return

            raw_frontiers = self.find_frontiers()

            clustered_frontiers = self.cluster_frontiers(raw_frontiers)

            self.publish_frontier_cloud(clustered_frontiers)

            if len(clustered_frontiers) > 0 and ORTOOLS_AVAILABLE:
                ordered_points = self.solve_tsp(clustered_frontiers)
                self.publish_path(ordered_points)

    def find_frontiers(self):
        frontiers = []
        free_y, free_x = np.where(self.map_data == 0)
        min_x, max_x = 1, self.width - 2
        min_y, max_y = 1, self.height - 2
        
        for y, x in zip(free_y, free_x):
            if not (min_x <= x <= max_x and min_y <= y <= max_y): continue
            window = self.map_data[y-1:y+2, x-1:x+2]
            if np.any(window == -1):
                frontiers.append(Point2D(x, y))
        return frontiers

    def cluster_frontiers(self, raw_frontiers):
        if not raw_frontiers: return []
        clusters = {}
        for pt in raw_frontiers:
            k = (pt.x // self.cluster_size, pt.y // self.cluster_size)
            if k not in clusters: clusters[k] = []
            clusters[k].append(pt)
            
        final_points = []
        for points in clusters.values():
            sum_x = sum(p.x for p in points)
            sum_y = sum(p.y for p in points)
            mean_x, mean_y = sum_x / len(points), sum_y / len(points)
            best = min(points, key=lambda p: (p.x - mean_x)**2 + (p.y - mean_y)**2)
            final_points.append(best)
        return final_points

    def solve_tsp(self, frontiers):
        nodes_world = [(self.robot_x, self.robot_y)]
        for pt in frontiers:
            wx, wy = self.grid_to_world(pt.x, pt.y)
            nodes_world.append((wx, wy))
            
        num_locations = len(nodes_world)
        if num_locations < 2:
            return []

        distance_matrix = {}
        for i in range(num_locations):
            distance_matrix[i] = {}
            for j in range(num_locations):
                if i == j:
                    distance_matrix[i][j] = 0
                else:
                    dist = math.hypot(nodes_world[i][0] - nodes_world[j][0],
                                      nodes_world[i][1] - nodes_world[j][1])
                    distance_matrix[i][j] = int(dist * 1000)

        manager = pywrapcp.RoutingIndexManager(num_locations, 1, 0)
        routing = pywrapcp.RoutingModel(manager)

        def distance_callback(from_index, to_index):
            from_node = manager.IndexToNode(from_index)
            to_node = manager.IndexToNode(to_index)
            return distance_matrix[from_node][to_node]

        transit_callback_index = routing.RegisterTransitCallback(distance_callback)
        routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

        search_parameters = pywrapcp.DefaultRoutingSearchParameters()
        search_parameters.first_solution_strategy = (
            routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
        search_parameters.time_limit.seconds = 1

        solution = routing.SolveWithParameters(search_parameters)

        ordered_coords = []
        if solution:
            index = routing.Start(0)
            while not routing.IsEnd(index):
                node_index = manager.IndexToNode(index)
                ordered_coords.append(nodes_world[node_index])
                index = solution.Value(routing.NextVar(index))
        else:
            self.get_logger().warn("TSP Solution not found!")
            
        return ordered_coords

    def publish_path(self, ordered_coords):
        path_msg = Path()
        path_msg.header = self.map_header
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for (wx, wy) in ordered_coords:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            pose.pose.position.z = self.origin_z
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
            
        self.path_pub.publish(path_msg)
        self.get_logger().info(f"Published TSP Path with {len(ordered_coords)} waypoints", throttle_duration_sec=2.0)

    def publish_frontier_cloud(self, frontiers):
        if not frontiers: return
        points = []
        for pt in frontiers:
            wx, wy = self.grid_to_world(pt.x, pt.y)
            points.append([wx, wy, self.origin_z])
            
        header = self.map_header
        header.stamp = self.get_clock().now().to_msg()
        pc_msg = point_cloud2.create_cloud_xyz32(header, points)
        self.pcl_pub.publish(pc_msg)

    def grid_to_world(self, gx, gy):
        wx = self.origin_x + (gx + 0.5) * self.resolution
        wy = self.origin_y + (gy + 0.5) * self.resolution
        return wx, wy

def main(args=None):
    rclpy.init(args=args)
    node = TSPExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()