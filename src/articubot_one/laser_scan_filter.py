import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class LaserScanFilter(Node):
    def __init__(self):
        super().__init__('laser_scan_filter')

        # Parameters for min and max range
        self.declare_parameter('range_min', 0.1)  # Default 0.1 meters
        self.declare_parameter('range_max', 10.0)  # Default 10 meters

        # Parameter for angle ranges to filter out (flattened list in degrees)
        # Example: [30.0, 60.0, -90.0, -60.0] indicates two ranges: 30° to 60° and -90° to -60° will be filtered out
        self.declare_parameter('angle_filter_ranges_deg', [31.0, 41.5,-36.0, -31.5])

        self.range_min = self.get_parameter('range_min').value
        self.range_max = self.get_parameter('range_max').value
        self.angle_filter_ranges_flat_deg = self.get_parameter('angle_filter_ranges_deg').value

        # Parse flattened angle list into pairs of ranges and convert them to radians
        self.angle_filter_ranges_rad = [
            (math.radians(self.angle_filter_ranges_flat_deg[i]), math.radians(self.angle_filter_ranges_flat_deg[i+1]))
            for i in range(0, len(self.angle_filter_ranges_flat_deg), 2)
        ]

        # Subscriber and Publisher
        self.scan_subscriber = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.scan_publisher = self.create_publisher(LaserScan, '/scan_filtered', 10)

    def scan_callback(self, msg):
        filtered_ranges = []

        # Iterate through each range value
        for i, r in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment

            # Check if the angle falls within any of the filter-out ranges
            angle_filtered_out = any(
                min_angle <= angle <= max_angle for min_angle, max_angle in self.angle_filter_ranges_rad
            )

            # Filter based on range and angle (exclude measurements within filtered angle ranges)
            if self.range_min <= r <= self.range_max and not angle_filtered_out:
                filtered_ranges.append(r)
            else:
                filtered_ranges.append(float('inf'))  # Set to infinity for filtered values

        # Publish the filtered scan
        filtered_msg = msg
        filtered_msg.ranges = filtered_ranges
        self.scan_publisher.publish(filtered_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LaserScanFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

