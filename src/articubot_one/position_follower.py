import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from rclpy.time import Time
from tf2_ros import TransformListener, Buffer

class PositionFollower(Node):

    def __init__(self):
        super().__init__('position_follower')
        
        # Suscripción al tópico /uwb_position
        self.subscription = self.create_subscription(
            Point,
            '/uwb_position',
            self.callback_uwb_position,
            10)
        
        # Publicación al tópico /goal_update
        self.publisher_ = self.create_publisher(PoseStamped, '/goal_update', 10)

    def callback_uwb_position(self, point_msg):
        # Crear un mensaje PointStamped
        point_stamped_msg = PointStamped()
        
        # Asignar tiempo y marco de referencia al header
        point_stamped_msg.header.stamp = self.get_clock().now().to_msg()
        point_stamped_msg.header.frame_id = "map"  # Cambia el marco si es necesario
        
        # Copiar la posición recibida en PointStamped
        point_stamped_msg.point = point_msg

        # Convertir el mensaje a PoseStamped
        pose = PoseStamped()
        pose.header = point_stamped_msg.header
        pose.pose.position = point_stamped_msg.point
        
        # Orientación fija
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0
        
        # Publicar el mensaje PoseStamped en /goal_update
        self.publisher_.publish(pose)

def main(args=None):
    rclpy.init(args=args)
    position_follower = PositionFollower()
    rclpy.spin(position_follower)
    position_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()






# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Point, PoseStamped
# from nav2_msgs.action import NavigateToPose
# from rclpy.action import ActionClient
# from tf2_ros import TransformListener, Buffer
# import time

# class PositionFollower(Node):
#     def __init__(self):
#         super().__init__('position_follower')

#         # Suscriptor al topic "uwb_position" para recibir x, y, z
#         self.create_subscription(Point, "uwb_position", self.position_callback, 10)

#         # Cliente de acción para enviar la posición a Nav2
#         self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

#         # Buffer y listener para transformaciones de TF
#         self.tf_buffer = Buffer()
#         self.tf_listener = TransformListener(self.tf_buffer, self)

#         # Variable para almacenar la posición objetivo
#         self.goal_pose = None

#         self.get_logger().info("PositionFollower node has been started.")

#     def position_callback(self, msg):
#         # Procesar la posición recibida del ESP32-S3
#         now = rclpy.time.Time()
#         transform = self.tf_buffer.lookup_transform('map', 'base_link', now)
#         self.goal_pose = PoseStamped()
#         self.goal_pose.header.frame_id = "map"  # Asumimos que la posición está en el marco "map"
#         self.goal_pose.pose.position.x = msg.x
#         self.goal_pose.pose.position.y = msg.y
#         self.goal_pose.pose.position.z = transform.transform.translation.z
#         self.goal_pose.pose.orientation.w = 1.0  # Sin rotación específica

#         self.get_logger().info(f"Received new goal position: x={msg.x}, y={msg.y}, z={transform.transform.translation.z}")

#         # Enviar la posición a Nav2
#         self.send_goal(self.goal_pose)

#     def send_goal(self, pose):
#         # Enviar el objetivo de navegación a Nav2
#         if not self.action_client.wait_for_server(timeout_sec=10.0):
#             self.get_logger().error("Nav2 action server not available!")
#             return

#         self.get_logger().info("Sending goal to Nav2...")
#         goal_msg = NavigateToPose.Goal()
#         goal_msg.pose = pose

#         self.action_client.send_goal_async(goal_msg).add_done_callback(self.goal_response_callback)

#     def goal_response_callback(self, future):
#         goal_handle = future.result()
#         if not goal_handle.accepted:
#             self.get_logger().info('Goal was rejected by Nav2.')
#             return

#         self.get_logger().info('Goal accepted by Nav2. Waiting for result...')
#         goal_handle.get_result_async().add_done_callback(self.result_callback)

#     def result_callback(self, future):
#         result = future.result()
#         if result.status == 3:  # STATUS_SUCCEEDED
#             self.get_logger().info('Goal reached successfully!')
#         else:
#             self.get_logger().error(f'Goal failed with status: {result.status}')

# def main(args=None):
#     rclpy.init(args=args)
#     node = PositionFollower()
#     rclpy.spin(node)
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
