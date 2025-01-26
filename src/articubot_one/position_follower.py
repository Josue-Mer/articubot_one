import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from tf2_geometry_msgs import do_transform_point
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
from collections import defaultdict, deque


class PositionFollower(Node):

    def __init__(self):
        super().__init__('position_follower')

        # Publishers
        self.status_publisher_ = self.create_publisher(Bool, '/follower_status', 10)
        self.goal_pose_publisher_ = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel_tracker', 10)


        # Subscriptions
        self.laser_subscription = self.create_subscription(LaserScan, '/scan_filtered', self.callback_laser_scan, 10)
        self.camera_subscription = self.create_subscription(TrackedObject, '/person_data', self.callback_camera_position, 10)
        self.uwb_subscription = self.create_subscription(Point, '/uwb_position', self.callback_uwb_position, 10)
        self.tag_status_subscription = self.create_subscription(Bool, '/tag_status', self.callback_tag_status, 10)

        # Transform buffer and listener
        self.tf_buffer = Buffer(rclpy.duration.Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Action client for navigation
        self.nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav_action_client.wait_for_server()


        # Parameters and state  
        # self.status_timer = self.create_timer(0.5, self.publish_status(True))
        self.last_goal_position = None
        self.acceptable_distance = 4.0 # Max distance for LIDAR points
        self.proximity_threshold = 0.2  # Threshold of proximity to the target
        self.current_uwb_position = None  # Stores the most recent UWB position
        self.lidar_ang_range = 45  # LiDAR positive and negative viewing range, ±45°
        self.target_id = 0  # The initial target ID
        self.navigation_distance_threshold = 0.5
        self.tag_status = None
        
        self.aligned = False  # Tracks alignment status
        self.no_match_start_time = None  # Tracks when "no valid match" state begins
        self.rotation_active = False    # Tracks if rotation for searching is active
        self.rotation_threshold = 2.0  # Time threshold to initiate rotation (in seconds)
        self.camera_rotate = False  # Tracks the camera rotation status
        self.alignment_threshold = 0.3  # Tolerable range to keep aligned

        self.tag_off_time = None  # Tracks the time when the tag is off
        self.tag_clean_threshold = 1.0  # Time threshold to clean tracking variables (in seconds)

        # History for prediction
        self.goal_history = deque(maxlen=5)  # Store recent navigation points
        self.prediction_factor = 0.5  # Factor for weighted prediction
        self.camera_positions = defaultdict(list)  # Save positions by ID # Store the most recent camera position by ID


        self.get_logger().info(f"{fore.LIGHT_MAGENTA}PositionFollower node initialized.{style.RESET}\n")



    ### CALLBACKS

    def callback_uwb_position(self, point_msg):

        """Processes the position of the UWB sensor and validates the filtered LIDAR points."""
        self.current_uwb_position = point_msg  # Store the most recent UWB position

        if not self.tag_status:
            self.get_logger().warning("No tag detected. Skipping validation.")
            return
        

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
            self.handle_no_match()
            return
        
        # Validate with the camera
        if not self.camera_rotate:
            self.no_match_start_time = None
            self.stop_rotation()

        best_match = None
        min_distance = float('inf')


        for person_id, points in self.camera_positions.items():

            for camera_point in points:

                for lidar_point in valid_points:
                    distance = self.calculate_distance(lidar_point, camera_point)

                    if distance < min_distance:
                        min_distance = distance
                        best_match = (person_id, camera_point)

        
        if best_match:
            # Reset rotation tracking and stop rotation
            self.camera_rotate = False
            self.no_match_start_time = None
            self.stop_rotation()

            self.target_id, closest_point = best_match
            if abs(self.camera_positions[self.target_id][-1].y) > self.alignment_threshold:
                self.align_robot_with_camera(self.camera_positions[self.target_id][-1])

            else:
                self.aligned = True

            if self.aligned:
                # Predict position based on history
                predicted_goal = self.predict_goal_position(closest_point)
                self.send_new_goal(predicted_goal)
                self.get_logger().info(f"{fore.LIGHT_CYAN}Tracking target ID: {self.target_id}{style.RESET}\n")
            else:
                self.get_logger().info("Waiting for alignment before sending new goal.")
        else:
            self.handle_no_match()
            self.camera_rotate = True
    


    def callback_laser_scan(self, scan_msg):
        """Processes LIDAR data and filters relevant points to compare with UWB."""

        # Convert angles and ranges to Cartesian coordinates
        angle_min = scan_msg.angle_min
        angle_increment = scan_msg.angle_increment
        ranges = scan_msg.ranges
        self.lidar_points = []
        self.collision_points = []

        for i, r in enumerate(ranges):
            angle = angle_min + i * angle_increment

            if 0 < r <= self.acceptable_distance:
                x_collision = r * math.cos(math.radians(angle))
                y_collision = r * math.sin(math.radians(angle))
                self.collision_points.append(Point(x=x_collision, y=y_collision, z=0.0))

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

        # Set flag to indicate new data
        self.new_camera_data = True

        # Save the detected position in the list associated with the ID
        self.camera_positions[person_id].append(camera_point)

        # Limit position history by ID (optional)
        if len(self.camera_positions[person_id]) > 5:
            self.camera_positions[person_id].pop(0)

        # Clear positions of other IDs if the target ID is known
        if self.target_id and person_id != self.target_id:
            self.camera_positions.pop(person_id, None)


    
    def callback_tag_status(self, status):
        """Processes the status of the tag detection."""
        self.tag_status = status.data
        
        if not self.tag_status:
            # If the tag is off, start or update the timer
            if self.tag_off_time is None:
                self.tag_off_time = self.get_clock().now()
            else:
                elapsed_time = (self.get_clock().now() - self.tag_off_time).nanoseconds / 1e9
                if elapsed_time >= self.tag_clean_threshold:
                    self.clean_tracking_variables()
                    self.get_logger().info(f"{fore.LIGHT_RED}Tag status is false. Cancelling all navigation goals.{style.RESET}")
                    self.cancel_all_goals()
                    self.tag_off_time = None  # Restart to avoid repetitive cleaning
        else:
            # If the tag is active, reset the timer
            self.tag_off_time = None
            

    
    ### NAVIGATION

    def send_new_goal(self, point_msg):
        """Sends a new target based on the detected position, but only if it is at a distance greater than the threshold."""
        if self.last_goal_position:
            distance_to_goal = self.calculate_distance(self.last_goal_position, point_msg)
            if distance_to_goal <= self.proximity_threshold:
                return  # Do not send new objectives if we are close to the last one
        
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

    def predict_goal_position(self, current_point):
        """Predicts the next position based on history and current data."""
        self.goal_history.append(current_point)
        if len(self.goal_history) < 2:
            return current_point
        dx = self.goal_history[-1].x - self.goal_history[-2].x
        dy = self.goal_history[-1].y - self.goal_history[-2].y
        predicted_x = self.goal_history[-1].x + dx * self.prediction_factor
        predicted_y = self.goal_history[-1].y + dy * self.prediction_factor
        return Point(x=predicted_x, y=predicted_y, z=0.0)
    


    def align_robot_with_camera(self, camera_point):
        """Align the robot to the target using the camera's y position."""

        # Try to align with the camera
        if self.target_id in self.camera_positions and self.camera_positions[self.target_id]:

            # Use the last camera point associated with the current ID
            camera_point = self.camera_positions[self.target_id][-1]
            self.get_logger().info(f"{fore.LIGHT_BLUE}Camera_Point :{camera_point}{style.RESET}\n")

            # Keep the target centered within a range of values ​​and
            y_offset = camera_point.y  # y of camera → lateral alignment

            if abs(y_offset) > self.alignment_threshold:
                # Calculate angular velocity
                angular_velocity = 2.5 if y_offset > 0 else -2.5  
                
                # Create and publish the Twist message
                twist_msg = Twist()
                twist_msg.angular.z = angular_velocity
                twist_msg.linear.x = 0.0  # Do not modify the linear speed

                self.cmd_vel_publisher.publish(twist_msg)
                self.aligned = False

                self.get_logger().info(
                    f"{fore.LIGHT_BLUE}Adjusting angular velocity to align: {angular_velocity:.2f} rad/s (y_offset: {y_offset:.2f}){style.RESET}\n"
                )
            else:
                # If the target is already aligned, stop the rotation
                twist_msg = Twist()
                twist_msg.angular.z = 0.0
                twist_msg.linear.x = 0.0
                self.cmd_vel_publisher.publish(twist_msg)
                self.aligned = True
                self.get_logger().info(f"{fore.BLUE}Robot aligned. Stopping angular velocity.{style.RESET}\n")

        else:
            # No valid camera target for alignment
            self.get_logger().warning("No valid camera target for alignment. Waiting for new data.")



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

            return pose

        except Exception as e:
            self.get_logger().error(f"Error in transform_to_map_frame: {e}")
            return None
        


    def handle_no_match(self):
        """Handles the case when no valid match is found between sensors."""
        current_time = self.get_clock().now()

        if self.no_match_start_time is None:
            self.no_match_start_time = current_time
            self.get_logger().warning("No valid match between sensor points. Starting timer.")
            return

        elapsed_time = (current_time - self.no_match_start_time).nanoseconds / 1e9

        if elapsed_time > self.rotation_threshold:
            self.get_logger().info("No valid data for an extended period. Initiating rotation search.")
            self.rotate_to_search()
        else:
            self.get_logger().warning(
                f"No valid match between sensor points for {elapsed_time:.2f} seconds."
            )



    def rotate_to_search(self):
        """Rotates the robot to search for valid camera data."""

        left_clear, right_clear, front_clear, rear_clear = self.check_collision_safety()

        # Create and publish a Twist message to rotate the robot
        twist_msg = Twist()
        twist_msg.angular.z = -2.5  # Set a constant angular velocity
        twist_msg.linear.x = 0.0 # Ensure no forward movement

        self.cmd_vel_publisher.publish(twist_msg)
        self.get_logger().info(f"{fore.LIGHT_BLUE}Rotating to search for valid data.{style.RESET}\n")
    


    def stop_rotation(self):
        """Stops the rotation when valid data is found."""
        if self.rotation_active:
            twist_msg = Twist()
            twist_msg.angular.z = 0.0
            twist_msg.linear.x = 0.0
            self.cmd_vel_publisher.publish(twist_msg)
            self.rotation_active = False
            self.get_logger().info(f"{fore.LIGHT_GREEN}Stopping rotation. Valid data found.{style.RESET}\n")



    def calculate_distance(self, position1, position2):
        dx = position1.x - position2.x
        dy = position1.y - position2.y
        return math.sqrt(dx**2 + dy**2)
    


    def publish_status(self, status):
        """Publish the robot's status to the /follower_status topic."""
        status_msg = Bool()
        status_msg.data = status
        self.status_publisher_.publish(status_msg)


    
    def cancel_all_goals(self):
        """Cancels all active goals in the ActionClient."""
        if not self.nav_action_client._goal_handles:
            self.get_logger().info("No active navigation goals to cancel.")
            return

        for goal_uuid, goal_ref in list(self.nav_action_client._goal_handles.items()):
            goal_handle = goal_ref()
            if goal_handle is not None:
                future = self.nav_action_client._cancel_goal_async(goal_handle)

                # Handle cancellation result
                def cancel_callback(fut):
                    try:
                        cancel_result = fut.result()
                        if cancel_result.goals_canceling:
                            self.get_logger().info(f"Goal {goal_uuid} successfully cancelled.")
                        else:
                            self.get_logger().warning(f"Failed to cancel goal {goal_uuid}.")
                    except Exception as e:
                        self.get_logger().error(f"Error while cancelling goal {goal_uuid}: {e}")

                future.add_done_callback(cancel_callback)
    


    def clean_tracking_variables(self):
        """Cleans all variables related to tracking when the tag is off for too long."""
        self.camera_positions.clear()
        self.lidar_points = []
        self.collision_points = []
        self.goal_history.clear()
        self.last_goal_position = None
        self.target_id = 0 
        self.aligned = False
        self.no_match_start_time_lidar_camera = None
        self.no_match_start_time_uwb_lidar = None

        self.get_logger().info(f"{fore.LIGHT_RED}Tag has been off for too long. Cleared all tracking variables.{style.RESET}")



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
