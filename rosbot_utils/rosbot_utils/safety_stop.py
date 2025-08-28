#!/usr/bin/env python3

import math
from enum import Enum
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, Point
from visualization_msgs.msg import Marker, MarkerArray

class State(Enum):
    FREE = 0
    WARNING = 1
    DANGER = 2

class SafetyStop(Node):
    def __init__(self):
        super().__init__("safety_stop_node")

        # Zone parameters from diagram
        self.declare_parameter("inner_free_radius", 0.1)   # 0.1m free zone
        self.declare_parameter("danger_radius", 0.3)      # 0.3m danger start
        self.declare_parameter("warning_radius", 0.6)       # 0.6m warning boundary
        
        # Get parameters
        self.inner_radius = self.get_parameter("inner_free_radius").value
        self.warning_radius = self.get_parameter("warning_radius").value
        self.danger_radius = self.get_parameter("danger_radius").value

        # ROS interfaces (unchanged from original)
        self.laser_sub = self.create_subscription(LaserScan, "scan", self.laser_callback, 10)
        self.safety_stop_pub = self.create_publisher(Bool, "safety_stop", 10)
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.zone_pub = self.create_publisher(MarkerArray, "zone", 10)

        # Visualization markers for all zones
        self.zones = MarkerArray()
        self.create_zone_markers()

        self.state = State.FREE
        self.prev_state = State.FREE

    def create_zone_markers(self):
        """Create concentric zone markers matching the diagram"""
        # warning Zone (0.3-0.6m) - Red
        warning_marker = Marker()
        warning_marker.ns = "warning_zone"                   
        warning_marker.id = 0
        warning_marker.type = Marker.LINE_STRIP
        warning_marker.action = Marker.ADD
        warning_marker.scale.x = 0.03
        warning_marker.color.r = 1.0
        warning_marker.color.g = 1.0
        warning_marker.color.a = 0.7 
        warning_marker.points = self.generate_circle_points(self.warning_radius)

        # danger Zone (0.1-0.3m) - Yellow
        danger_marker = Marker()
        danger_marker.ns = "danger_zone"
        danger_marker.id = 1
        danger_marker.type = Marker.LINE_STRIP
        danger_marker.action = Marker.ADD
        danger_marker.scale.x = 0.05  # Line thickness
        danger_marker.color.r = 1.0
        danger_marker.color.a = 0.7
        danger_marker.points = self.generate_circle_points(self.danger_radius)

        # Inner Free Zone (0-0.1m) - Green
        free_marker = Marker()
        free_marker.ns = "free_zone"
        free_marker.id = 2
        free_marker.type = Marker.LINE_STRIP
        free_marker.action = Marker.ADD
        free_marker.scale.x = 0.02
        free_marker.color.g = 1.0
        free_marker.color.a = 0.7
        free_marker.points = self.generate_circle_points(self.inner_radius)

        self.zones.markers = [warning_marker, danger_marker, free_marker]

    def generate_circle_points(self, radius, angle_range=(-math.pi/2, math.pi/2)):
        """Generate points for a 180-degree front arc"""
        points = []
        steps = 30  # Resolution of the arc
        for i in range(steps + 1):
            angle = angle_range[0] + (angle_range[1]-angle_range[0]) * i/steps
            x = radius * math.cos(angle)
            y = radius * math.sin(angle)
            points.append(Point(x=x, y=y, z=0.0))
        return points

    def laser_callback(self, msg: LaserScan):
        """Check all scan points in front 180-degree arc"""
        self.state = State.FREE
        closest_obstacle = float('inf')

        # Scan processing
        for i, range_val in enumerate(msg.ranges):
            # Only consider front 180 degrees
            angle = msg.angle_min + i * msg.angle_increment
            if not (-math.pi/2 <= angle <= math.pi/2):
                continue

            if math.isinf(range_val) or range_val < msg.range_min:
                continue

            closest_obstacle = min(closest_obstacle, range_val)

        # State determination
        if closest_obstacle <= self.danger_radius:
            self.state = State.DANGER
        elif closest_obstacle <= self.warning_radius:
            self.state = State.WARNING
        else:
            self.state = State.FREE

        # Handle state transitions
        if self.state != self.prev_state:
            cmd_vel = Twist()
            
            if self.state == State.DANGER:
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = 0.0
            elif self.state == State.WARNING:
                cmd_vel.linear.x = 0.2  # Reduced speed
                cmd_vel.angular.z *= 0.5  # Reduced turning
            else:
                cmd_vel.linear.x = 0.5  # Full speed
                cmd_vel.angular.z = 1.0  # Full turning

            self.cmd_vel_pub.publish(cmd_vel)
            self.prev_state = self.state

        # Update marker visibility
        for marker in self.zones.markers:
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.header.frame_id = msg.header.frame_id
        self.zone_pub.publish(self.zones)

def main():
    rclpy.init()
    node = SafetyStop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()