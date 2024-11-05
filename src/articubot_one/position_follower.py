# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Point, PointStamped, PoseStamped
# from std_msgs.msg import Bool
# from rclpy.time import Time
# from tf2_ros import TransformListener, Buffer

# class PositionFollower(Node):

#     def __init__(self):
#         super().__init__('position_follower')
        
#         # Publishing status to /follower_status
#         self.status_publisher_ = self.create_publisher(Bool, '/follower_status', 10)
        
#         # Publish to topic /goal_update
#         self.publisher_ = self.create_publisher(PoseStamped, '/goal_update', 10)
        
#         # Subscription to topic /uwb_position
#         self.subscription = self.create_subscription(Point, '/uwb_position', self.callback_uwb_position, 10)

#         # Timer to publish active status every 2 seconds
#         self.timer_ = self.create_timer(0.5, lambda: self.activate_status(True))


#     def callback_uwb_position(self, point_msg):
#         # Create a PointStamped message
#         point_stamped_msg = PointStamped()
        
#         # Assign time and reference frame to the header
#         point_stamped_msg.header.stamp = self.get_clock().now().to_msg()
#         point_stamped_msg.header.frame_id = "map"
        
#         # Copy the position received in PointStamped
#         point_stamped_msg.point = point_msg

#         # Convert message to PoseStamped
#         pose = PoseStamped()
#         pose.header = point_stamped_msg.header
#         pose.pose.position = point_stamped_msg.point
        
#         # Fixed orientation
#         pose.pose.orientation.x = 0.0
#         pose.pose.orientation.y = 0.0
#         pose.pose.orientation.z = 0.0
#         pose.pose.orientation.w = 1.0
        
#         # Publish the PoseStamped message to /goal_update
#         self.publisher_.publish(pose)


#     def activate_status(self, is_active):
#         # Publish the activation status in the /follower_status topic
#         status_msg = Bool()
#         status_msg.data = is_active
#         self.status_publisher_.publish(status_msg)

    
#     def destroy_node(self):
#         # Publish idle status before closing
#         self.activate_status(False)
#         super().destroy_node()


# def main(args=None):
#     rclpy.init(args=args)
#     position_follower = PositionFollower()

#     try:
#         rclpy.spin(position_follower)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         position_follower.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from std_msgs.msg import Bool

class PositionFollower(Node):

    def __init__(self):
        super().__init__('position_follower')
        
        # Publishers
        self.status_publisher_ = self.create_publisher(Bool, '/follower_status', 10)
        self.goal_pose_publisher_ = self.create_publisher(PoseStamped, '/goal_pose', 10)  # Publisher for initial goal pose
        self.goal_update_publisher_ = self.create_publisher(PoseStamped, '/goal_update', 10)

        # Subscription to /uwb_position
        self.subscription = self.create_subscription(Point, '/uwb_position', self.callback_uwb_position, 10)

        # Timer to publish active status every 2 seconds
        self.timer_ = self.create_timer(0.5, lambda: self.activate_status(True))

        # Flag to check if the initial goal has been sent
        self.initial_goal_sent = False


    def callback_uwb_position(self, point_msg):
        # If the initial goal hasn't been sent, send it to /goal_pose and set the flag
        if not self.initial_goal_sent:
            self.send_initial_goal(point_msg)
            self.initial_goal_sent = True
        else:
            # Continue with regular goal updates in /goal_update
            self.publish_goal_update(point_msg)


    def send_initial_goal(self, point_msg):
        self.get_logger().info("Sending initial goal to /goal_pose...")
        initial_goal = self.create_pose_stamped(point_msg)
        self.goal_pose_publisher_.publish(initial_goal)


    def publish_goal_update(self, point_msg):
        # Publish the goal to /goal_update for regular updates
        goal_update = self.create_pose_stamped(point_msg)
        self.goal_update_publisher_.publish(goal_update)


    def create_pose_stamped(self, point_msg):
        # Create a PoseStamped message from a Point message
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"
        
        # Copy the position from the point message
        pose.pose.position = point_msg
        
        # Fixed orientation
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0
        
        return pose


    def activate_status(self, is_active):
        # Publish the activation status in the /follower_status topic
        status_msg = Bool()
        status_msg.data = is_active
        self.status_publisher_.publish(status_msg)

    
    def destroy_node(self):
        # Publish idle status before closing
        self.activate_status(False)
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
