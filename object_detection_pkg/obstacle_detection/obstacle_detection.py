import math

import rclpy  # Import ROS2 client library
from rclpy.node import Node  # Import Node class to create a ROS2 node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy ,QoSReliabilityPolicy, QoSDurabilityPolicy # Import LaserScan message type to handle LiDAR data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool  # Import Bool message type (True/False)


# Define the main LiDAR obstacle filter node class
class LiDARObstacleFilter(Node):
    def __init__(self):
        # Initialize the Node with the name 'lidar_obstacle_filter'
        super().__init__('lidar_obstacle_filter')

        # Set up Quality of Service (QoS) profile for communication
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  
            history=HistoryPolicy.KEEP_LAST,  # Keep the latest message in the history
            depth=5  # Depth of the message queue
        )

        # Create a subscription to the '/scan' topic for LaserScan messages
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.handle_scan, qos)

        # Create a publisher for sending the filtered LaserScan (filtered_scan)
        # to the '/filtered_scan' topic
        self.filtered_pub = self.create_publisher(
            LaserScan, '/filtered_scan', qos)

        # Create a publisher to send obstacle detection status (True/False) to
        # the '/obstacle_status' topic
        self.status_pub = self.create_publisher(
            Bool, '/obstacle_detected', 10)

        # Detection threshold: Obstacles closer than  meters are considered
        # detected
        self.detection_threshold = 0.6  # meters

        # Define the angle range for detection: from -90° (left) to -30°
        # (right)
        self.angle_range = (-(90 * (math.pi / 180)), -
                            (30 * (math.pi / 180)))  # -90° to -30°

        # Log that the node is initialized
        self.get_logger().info("LiDAR obstacle filter node initialized.")

    def handle_scan(self, msg: LaserScan):
        if not msg.ranges:
            self.get_logger().warn("Empty scan received. Skipping.")
            return
        try:
            total_beams = len(msg.ranges)
            masked = [float('inf')] * total_beams
            obstacle_found = False

            angle_min = msg.angle_min
            angle_inc = msg.angle_increment

        # Ensure angle_increment is not zero (avoid division by zero)
            if angle_inc == 0.0:
                self.get_logger().warn("Angle increment is zero. Skipping scan.")
                return

        # Calculate start and end index
            start_idx = max(
                0, int(
                    (self.angle_range[0] - angle_min) / angle_inc))
            end_idx = min(total_beams - 1,
                          int((self.angle_range[1] - angle_min) / angle_inc))

            for i in range(start_idx, end_idx + 1):
                if i < 0 or i >= total_beams:
                    continue
                distance = msg.ranges[i]
                if distance is not None and not math.isnan(
                        distance) and not math.isinf(distance):
                    if 0 < distance < self.detection_threshold:
                        masked[i] = distance
                        obstacle_found = True

            # Create a new LaserScan message for the filtered (masked) scan
            filtered_scan = LaserScan()
            # header (timestamp, frame_id, etc.)
            filtered_scan.header = msg.header
            filtered_scan.angle_min = msg.angle_min  # angle_min from original message
            filtered_scan.angle_max = msg.angle_max  # angle_max from original message
            filtered_scan.angle_increment = msg.angle_increment  # angle_increment
            filtered_scan.time_increment = msg.time_increment  # time_increment
            filtered_scan.scan_time = msg.scan_time  # scan_time
            filtered_scan.range_min = msg.range_min  # range_min
            filtered_scan.range_max = msg.range_max  # range_max
            # Assign the filtered (masked) ranges
            filtered_scan.ranges = masked

            # Publish the masked scan data to the '/filtered_scan' topic
            self.filtered_pub.publish(filtered_scan)

            # Publish the obstacle status (True if an obstacle is found, False
            # otherwise)
            self.status_pub.publish(Bool(data=obstacle_found))

            # Log if an obstacle was found or the path is clear
            log_msg = "Obstacle detected in ROI." if obstacle_found else "Clear path in ROI."
            self.get_logger().info(log_msg)

        except Exception as e:
            # Log any errors that occur while processing the LaserScan message
            self.get_logger().error(f"Failed to process scan: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = LiDARObstacleFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

