import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from tf2_geometry_msgs import do_transform_point
from std_msgs.msg import UInt32
import rclpy.time
from std_msgs.msg import Bool
from tf2_ros import TransformListener, Buffer
from nav2_msgs.action import NavigateToPose # type: ignore
from rclpy.action import ActionClient
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
from custom_msgs.msg import TrackedObject
from colored import fore, style
from collections import defaultdict


class PositionFollower(Node):

    def __init__(self):
        super().__init__('position_follower')

        # Publishers
        self.status_publisher_ = self.create_publisher(Bool, '/follower_status', 10)
        self.goal_pose_publisher_ = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel_joy', 10)


        # Subscriptions
        self.laser_subscription = self.create_subscription(LaserScan, '/scan', self.callback_laser_scan, 10)
        self.camera_subscription = self.create_subscription(TrackedObject, '/persondata', self.callback_camera_position, 10)
        self.uwb_subscription = self.create_subscription(Point, '/uwb_position', self.callback_uwb_position, 10)

        # Transform buffer and listener
        self.tf_buffer = Buffer(rclpy.duration.Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Action client for navigation
        self.nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav_action_client.wait_for_server()


        # Parameters and state  
        # self.status_timer = self.create_timer(0.5, self.publish_status(True))
        self.last_goal_position = None
        self.acceptable_distance = 3.0
        self.proximity_threshold = 0.5  # Threshold of proximity to the target
        self.current_uwb_position = None  # Stores the most recent UWB position
        self.camera_positions = defaultdict(list)  # Save positions by ID # Store the most recent camera position by ID
        self.lidar_ang_range = 0.61  # LiDAR positive and negative viewing range, ±35°
        self.target_id = 0  # The initial target ID
        self.navigation_distance_threshold = 0.5


        self.get_logger().info(f"{fore.LIGHT_MAGENTA}PositionFollower node initialized.{style.RESET}\n")
        # self.get_logger().info(f"{fore.LIGHT_YELLOW}Puntos: \n{lidar_points}{style.RESET}\n")



    ### CALLBACKS

    def callback_uwb_position(self, point_msg):

        """Processes the position of the UWB sensor and validates the filtered LIDAR points."""
        self.current_uwb_position = point_msg  # Store the most recent UWB position


        if not hasattr(self, 'lidar_points') or not self.lidar_points:
            self.get_logger().warning("No LIDAR points available. Skipping validation.")
            return
        

        # Validate with UWB sensor
        valid_points = [
            p for p in self.lidar_points
            if abs(p.x - self.current_uwb_position.x) <= self.proximity_threshold
        ]

        if not valid_points:
            self.get_logger().warning("No LIDAR points align with UWB data.")
            return
        
        # Validate with the camera
        best_match = None
        min_distance = float('inf')

        # if self.camera_position:
        #     valid_points = [
        #         p for p in valid_points
        #         if abs(p.x - self.camera_position.x) <= self.proximity_threshold and
        #         abs(p.y - self.camera_position.y) <= self.proximity_threshold
        #     ]
        #     if not valid_points:
        #         self.get_logger().warning("No LIDAR points align with camera data.")
        #         return

        for person_id, points in self.camera_positions.items():

            for camera_point in points:

                for lidar_point in valid_points:
                    distance = self.calculate_distance(lidar_point, camera_point)

                    if distance < min_distance:
                        min_distance = distance
                        best_match = (person_id, lidar_point)

        
        if best_match:
            self.target_id, closest_point = best_match
            self.align_robot_with_camera(self.camera_positions[self.target_id][-1])

             # Seleccionar el punto más cercano al UWB
            # closest_point = min(
            #     better_points,
            #     key=lambda point: self.calculate_distance(point, self.current_uwb_position) 
            # )

            self.send_new_goal(closest_point)
            self.get_logger().info(f"{fore.LIGHT_CYAN}Tracking target ID: {self.target_id}{style.RESET}\n")
        else:
            self.get_logger().warning("No valid match between camera and LIDAR points.")
            return
    


    def callback_laser_scan(self, scan_msg):
        """Processes LIDAR data and filters relevant points to compare with UWB."""

        # Convert angles and ranges to Cartesian coordinates
        angle_min = scan_msg.angle_min
        angle_increment = scan_msg.angle_increment
        ranges = scan_msg.ranges
        self.lidar_points = []

        for i, r in enumerate(ranges):
            angle = angle_min + i * angle_increment

            # Filter by angular range
            if -self.lidar_ang_range <= angle <= self.lidar_ang_range and 0 < r <= self.acceptable_distance:

                # Convert to cartesian coordinates in base_link
                x = r * math.cos(angle) + 0.32 - 0.08122
                y = r * math.sin(angle)
                self.lidar_points.append(Point(x=x, y=y, z=0.0))


    
    def callback_camera_position(self, camera_msg):
        """Processes the position of the target detected by the camera."""
        # Transform camera position to robot frame and extract ID
        camera_point = Point(
            x = camera_msg.point.z + 0.6315 - 0.08122,  # z of the camera → x of the robot
            y = -camera_msg.point.x,  # x of the camera → -y of the robot
            z = camera_msg.point.y  # y of the camera (height) → z of the robot
        )

        person_id = int(camera_msg.id.data)

        # Save the detected position in the list associated with the ID
        self.camera_positions[person_id].append(camera_point)

        # Limit position history by ID (optional)
        if len(self.camera_positions[person_id]) > 10:
            self.camera_positions[person_id].pop(0)

    
    

    ### NAVIGATION

    def send_new_goal(self, point_msg):
        """Sends a new target based on the detected position, but only if it is at a distance greater than the threshold."""
        if self.last_goal_position:
            distance_to_goal = self.calculate_distance(self.last_goal_position, point_msg)
            if distance_to_goal <= self.proximity_threshold:
                return  # Do not send new objectives if we are close to the last one

        # Check if the distance to the target is greater than the threshold
        if distance_to_goal <= self.navigation_distance_threshold:
            self.get_logger().info(f"{fore.LIGHT_RED}Goal is too close (within {self.navigation_distance_threshold} meters). Not sending new goal.{style.RESET}\n")
            return  # Do not send the target if it is within the distance threshold
        
        robot_position = self.transform_to_map_frame(Point(x= 0.0, y= 0.0, z= 0.0))

        # Transform position and send target
        new_goal = self.transform_to_map_frame(point_msg)
        
        if new_goal is None:
            self.get_logger().warning("Failed to transform goal position. Falling back to UWB.")
            return
        
        # Calculate the distance between the robot and the navigation point in the global frame (map)
        distance_to_goal = self.calculate_distance(robot_position.pose.position, new_goal.pose.position)

        # Compare the distance and decide whether to send the target
        if distance_to_goal <= self.navigation_distance_threshold:
            self.get_logger().info(f"{fore.LIGHT_RED}Goal is too close ({distance_to_goal:.2f} m). Skipping navigation.{style.RESET}\n")
            return  # If it's close, don't send the target

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = new_goal
        self.last_goal_position = point_msg
        self.nav_action_client.send_goal_async(goal_msg)
        self.get_logger().info(f"{fore.LIGHT_GREEN}Sent navigation goal: {new_goal.pose.position}{style.RESET}\n")



    ### UTILITIES

    def align_robot_with_camera(self, camera_point):
        """Align the robot to the target using the camera's y position."""
        if camera_point is None:
            self.get_logger().warning("Camera position not available. Skipping alignment.")
            return

        # Keep the target centered within a range of values ​​and
        alignment_threshold = 0.3  # Tolerable range to keep aligned
        y_offset = camera_point.y  # y of camera → lateral alignment

        if abs(y_offset) > alignment_threshold:
            # Calculate angular velocity
            angular_velocity = -0.7 if y_offset > 0 else 0.7  # Negative if y is positive, positive if y is negative
            
            # Create and publish the Twist message
            twist_msg = Twist()
            twist_msg.angular.z = angular_velocity
            twist_msg.linear.x = 0.0  # Do not modify the linear speed
            self.cmd_vel_publisher.publish(twist_msg)

            self.get_logger().info(
                f"{fore.LIGHT_BLUE}Adjusting angular velocity to align: {angular_velocity:.2f} rad/s (y_offset: {y_offset:.2f}){style.RESET}\n"
            )
        else:
            # If the target is already aligned, stop the rotation
            twist_msg = Twist()
            twist_msg.angular.z = 0.0
            twist_msg.linear.x = 0.0
            self.cmd_vel_publisher.publish(twist_msg)
            self.get_logger().info(f"{fore.BLUE}Robot aligned. Stopping angular velocity.{style.RESET}\n")



    def transform_to_map_frame(self, point_msg):
        """Transforms coordinates from the base frame to the map frame."""
        now = rclpy.time.Time(seconds=0)
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_link', now, timeout=rclpy.duration.Duration(seconds=5.0))

            point_msg_stamped = PointStamped()
            point_msg_stamped.header.frame_id = 'map'
            point_msg_stamped.header.stamp = self.get_clock().now().to_msg()
            point_msg_stamped.point.x = point_msg.x
            point_msg_stamped.point.y = point_msg.y
            point_msg_stamped.point.z = 0.0

            point_in_map = do_transform_point(point_msg_stamped, transform)

            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = "map"
            pose.pose.position.x = point_in_map.point.x
            pose.pose.position.y = point_in_map.point.y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0

            # self.get_logger().info("Was able to transform map to base")
            return pose

        except Exception as e:
            self.get_logger().error(f"Error in transform_to_map_frame: {e}")
            return None
        


    def calculate_distance(self, position1, position2):
        dx = position1.x - position2.x
        dy = position1.y - position2.y
        return math.sqrt(dx**2 + dy**2)
    


    def publish_status(self, status: bool):
        """Publish the robot's status to the /follower_status topic."""
        status_msg = Bool()
        status_msg.data = status
        self.status_publisher_.publish(status_msg)



    def destroy_node(self):
        """Close the node cleanly."""
        self.publish_status(False)
        super().destroy_node()



def main(args=None):
    rclpy.init(args=args)
    position_follower = PositionFollower()

    try:
        rclpy.spin(position_follower)
    except KeyboardInterrupt:
        pass
    finally:
        position_follower.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
