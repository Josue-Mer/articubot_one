import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import Bool
from tf2_ros import TransformListener, Buffer
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
import numpy as np
import math


class PositionFollower(Node):

    def __init__(self):
        super().__init__('position_follower')

        # Publishers
        self.status_publisher_ = self.create_publisher(Bool, '/follower_status', 10)
        self.goal_pose_publisher_ = self.create_publisher(PoseStamped, '/goal_pose', 10)

        # Subscriptions
        self.costmap_subscription = self.create_subscription(OccupancyGrid, '/local_costmap/costmap', self.callback_costmap, 10)
        self.laser_subscription = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)

        # Transform buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Action client for navigation
        self.nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav_action_client.wait_for_server()

        # Parameters and state
        self.costmap_data = None
        self.costmap_resolution = None
        self.costmap_origin = None  
        self.last_goal_position = None
        self.last_good_costmap_velocity = None
        self.acceptable_distance = 3.0
        self.monitoring_time = 10.0
        self.monitoring_start_time = None
        self.movement_threshold = 0.5  # Mínimo movimiento detectable
        self.proximity_threshold = 0.5  # Umbral de cercanía al objetivo

        self.object_in_front = False  # Indica si hay un objeto frente al robot
        self.awaiting_movement = False  # Indica si estamos esperando que el objeto se mueva
        self.monitoring_with_lidar = True

        self.current_uwb_position = None  # Almacena la posición más reciente del UWB

        self.get_logger().info("PositionFollower node initialized.")

    ### CALLBACKS

    def callback_uwb_position(self, point_msg):
        """Procesa la posición del sensor UWB y la combina con el costmap."""

        # Almacenar la posición actual del UWB
        self.current_uwb_position = point_msg
        # self.get_logger().info(f"Updated UWB position: ({point_msg.x}, {point_msg.y})")

        # Transformar la posición del UWB al marco map
        uwb_position = self.transform_to_map_frame(point_msg)
        if uwb_position is None:
            return

        self.get_logger().info(f"Received UWB position: ({uwb_position.pose.position.x}, {uwb_position.pose.position.y})")

        # Refinar la posición con el costmap
        refined_position = self.refine_position_with_costmap(uwb_position.pose.position)

        # Usar la posición refinada como nuevo objetivo
        if refined_position:
            self.send_new_goal(refined_position)


    def callback_costmap(self, costmap_msg):
        """Procesa datos del costmap y actualiza posiciones ocupadas."""
        self.costmap_data = np.array(costmap_msg.data).reshape(
            (costmap_msg.info.height, costmap_msg.info.width)
        )
        self.costmap_resolution = costmap_msg.info.resolution
        self.costmap_origin = costmap_msg.info.origin

        front_positions = []  # Lista para almacenar las celdas que están al frente del robot

        for y in range(self.costmap_data.shape[0]):
            for x in range(self.costmap_data.shape[1]):
                if self.costmap_data[y, x] > 0:  # Celda ocupada
                    # Convertir coordenadas del costmap a coordenadas globales (map)
                    cell_x = self.costmap_origin.position.x + x * self.costmap_resolution
                    cell_y = self.costmap_origin.position.y + y * self.costmap_resolution

                    # Crear un punto para representar la celda ocupada
                    point_in_map = Point()
                    point_in_map.x = cell_x
                    point_in_map.y = cell_y
                    point_in_map.z = 0.0

                    # Transformar al marco del robot (base_link)
                    point_in_base = self.transform_to_robot_frame(point_in_map)
                    if point_in_base is None:
                        continue  # Omitir celdas que no se puedan transformar

                    # Filtrar celdas que estén al frente del robot
                    angle = math.atan2(point_in_base.y, point_in_base.x)
                    distance = math.sqrt(point_in_base.x**2 + point_in_base.y**2)

                    if -0.61 <= angle <= 0.61 and distance <= self.acceptable_distance:  # 0.785 ±45 grados            0.61 ±35 grados        0.26 ±15 grados
                        front_positions.append(point_in_map)

        # Seleccionar la celda más cercana al robot dentro del área frontal
        if front_positions:
            origin = Point()
            origin.x = 0.0
            origin.y = 0.0
            origin.z = 0.0
            closest_point = min(front_positions, key=lambda p: self.calculate_distance(origin, p))
            self.send_new_goal(closest_point)


            # Seleccionar la celda más cercana al robot dentro del área frontal
            if front_positions:
                origin = Point()
                origin.x = 0.0
                origin.y = 0.0
                origin.z = 0.0
                closest_point = min(front_positions, key=lambda p: self.calculate_distance(origin, p))
                self.send_new_goal(closest_point)


    def laser_callback(self, scan_msg):
        """Procesa datos del LIDAR para detectar movimiento."""
        if not self.monitoring_with_lidar:
            return
        
        # Verificar que haya una posición UWB válida antes de usarla como fallback
        if not self.current_uwb_position:
            self.get_logger().warning("No UWB position available. Skipping fallback.")
            return

        # Filtrar ángulos del LIDAR en el frente
        angle_min = -0.61  # -45 grados
        angle_max = 0.61   # 45 grados
        ranges = [
            distance for i, distance in enumerate(scan_msg.ranges)
            if angle_min <= scan_msg.angle_min + i * scan_msg.angle_increment <= angle_max
        ]

        # Detectar objetos en frente
        self.object_in_front = any(distance < self.acceptable_distance for distance in ranges if distance > 0)

        # Monitorear movimiento si hay un objeto en frente
        # if self.object_in_front:
        #     self.check_object_movement()

        if not self.object_in_front:
            self.get_logger().warning("No object detected in front. Falling back to UWB.")
            # Usar UWB como respaldo si no hay detección con LIDAR
            if self.current_uwb_position:
                self.callback_uwb_position(self.current_uwb_position)
            return
        
        self.check_object_movement()

    ### NAVIGATION

    def send_new_goal(self, point_msg):
        """Envía un nuevo objetivo basado en la posición detectada."""
        if self.last_goal_position:
            distance_to_goal = self.calculate_distance(self.last_goal_position, point_msg)
            if distance_to_goal <= self.proximity_threshold:
                return  # No enviar nuevos objetivos si estamos cerca del último

        # Transformar la posición y enviar el objetivo
        new_goal = self.transform_to_map_frame(point_msg)
        if new_goal is None:
            self.get_logger().warning("Failed to transform goal position. Falling back to UWB.")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = new_goal
        self.last_goal_position = point_msg
        future = self.nav_action_client.send_goal_async(goal_msg)
        self.get_logger().info(f"Sent navigation goal: {new_goal.pose.position}")

    def follow_object_with_costmap(self):
        """Sigue al objeto calculando nuevos objetivos basados en su velocidad."""
        if not self.last_good_costmap_velocity or not self.last_goal_position:
            return  # No hay datos suficientes para seguir

        adjusted_goal = Point(
            x=self.last_goal_position.x + self.last_good_costmap_velocity[0] * 0.5,
            y=self.last_goal_position.y + self.last_good_costmap_velocity[1] * 0.5,
            z=0.0
        )
        self.send_new_goal(adjusted_goal)

    ### UTILITIES

    def refine_position_with_costmap(self, uwb_position):
        """Refina la posición UWB utilizando las celdas del costmap."""
        if self.costmap_data is None:
            self.get_logger().warning("Costmap data not available. Using UWB position as is.")
            return uwb_position

        closest_cell = None
        min_distance = float('inf')

        # Buscar la celda ocupada más cercana al UWB en el costmap
        for y in range(self.costmap_data.shape[0]):
            for x in range(self.costmap_data.shape[1]):
                if self.costmap_data[y, x] > 0:  # Celda ocupada
                    cell_x = self.costmap_origin.position.x + x * self.costmap_resolution
                    cell_y = self.costmap_origin.position.y + y * self.costmap_resolution
                    distance = self.calculate_x_distance(uwb_position, Point(x=cell_x, y=0.0, z=0.0))
                    if distance < min_distance:
                        min_distance = distance

                        closest_cell = Point(x=cell_x, y=cell_y, z=0.0)

        if closest_cell:
            self.get_logger().info(f"Refined UWB position to closest costmap cell: ({closest_cell.x}, {closest_cell.y})")
            return closest_cell

        self.get_logger().warning("No matching costmap cell found. Using UWB position as is.")
        return uwb_position


    def transform_to_robot_frame(self, point_in_map):
        """Transforma un punto del marco map al marco base_link."""
        try:
            now = rclpy.time.Time()
            transform = self.tf_buffer.lookup_transform('base_link', 'map', now)

            # Transformar coordenadas
            robot_position = transform.transform.translation
            robot_orientation = transform.transform.rotation

            # Convertir orientación del robot a yaw
            _, _, yaw = self.quaternion_to_euler(
                robot_orientation.x, robot_orientation.y,
                robot_orientation.z, robot_orientation.w
            )

            # Calcular coordenadas relativas al robot
            dx = point_in_map.x - robot_position.x
            dy = point_in_map.y - robot_position.y

            point_in_base = Point()
            point_in_base.x = math.cos(yaw) * dx + math.sin(yaw) * dy
            point_in_base.y = -math.sin(yaw) * dx + math.cos(yaw) * dy
            point_in_base.z = 0.0  # Mantener en 2D
            return point_in_base

        except Exception as e:
            self.get_logger().error(f"Error transforming point to robot frame: {e}")
            return None


    def check_object_movement(self):
        """Verifica si el objeto detectado se ha movido significativamente."""
        if not self.last_goal_position or not self.monitoring_start_time:
            self.monitoring_start_time = self.get_clock().now()
            return

        elapsed_time = self.get_clock().now() - self.monitoring_start_time
        elapsed_seconds = elapsed_time.nanoseconds / 1e9

        # if elapsed_seconds > self.monitoring_time:
        #     self.get_logger().warning("Object not moving. Pausing navigation...")
        #     self.awaiting_movement = True
        #     return

    def detect_significant_movement(self, current_position):
        """Verifica si el objetivo se ha movido significativamente."""
        if not self.last_goal_position:
            return True

        distance = self.calculate_distance(self.last_goal_position, current_position)
        return distance >= self.movement_threshold

    def calculate_distance(self, position1, position2):
        dx = position1.x - position2.x
        dy = position1.y - position2.y
        return math.sqrt(dx**2 + dy**2)
    
    def calculate_x_distance(self, position1, position2):
        """Calcula la distancia en el eje x entre dos posiciones."""
        dx = position1.x - position2.x
        return abs(dx)


    def transform_to_map_frame(self, point_msg):
        """Transforma coordenadas del marco base al marco map."""
        try:
            now = rclpy.time.Time()
            transform = self.tf_buffer.lookup_transform('map', 'base_link', now, timeout=rclpy.duration.Duration(seconds=5.0))
            robot_position = transform.transform.translation
            robot_orientation = transform.transform.rotation

            _, _, yaw = self.quaternion_to_euler(
                robot_orientation.x, robot_orientation.y,
                robot_orientation.z, robot_orientation.w
            )

            sensor_x = point_msg.x * math.cos(yaw) - point_msg.y * math.sin(yaw)
            sensor_y = point_msg.x * math.sin(yaw) + point_msg.y * math.cos(yaw)

            pose = PoseStamped()
            pose.header.stamp = now.to_msg()
            pose.header.frame_id = "map"
            pose.pose.position.x = robot_position.x + sensor_x
            pose.pose.position.y = robot_position.y + sensor_y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            return pose

        except Exception as e:
            self.get_logger().error(f"Error in transform_to_map_frame: {e}")
            return None

    def quaternion_to_euler(self, x, y, z, w):
        """Convierte un cuaternión a ángulos de Euler."""
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        yaw = math.atan2(t0, t1)
        return 0.0, 0.0, yaw

    def destroy_node(self):
        """Cierra el nodo limpiamente."""
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

















# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Point, PoseStamped
# from std_msgs.msg import Bool
# from tf2_ros import TransformListener, Buffer
# from nav2_msgs.action import NavigateToPose
# from rclpy.action import ActionClient
# from nav_msgs.msg import OccupancyGrid
# from sensor_msgs.msg import LaserScan
# import numpy as np
# import math


# class PositionFollower(Node):

#     def __init__(self):
#         super().__init__('position_follower')

#         # Publishers
#         self.status_publisher_ = self.create_publisher(Bool, '/follower_status', 10)
#         self.goal_pose_publisher_ = self.create_publisher(PoseStamped, '/goal_pose', 10)

#         # Subscriptions
#         self.subscription = self.create_subscription(Point, '/uwb_position', self.callback_uwb_position, 10)
#         self.costmap_subscription = self.create_subscription(OccupancyGrid, '/local_costmap/costmap', self.callback_costmap, 10)
#         self.laser_subscription = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)

#         # Transform buffer and listener
#         self.tf_buffer = Buffer()
#         self.tf_listener = TransformListener(self.tf_buffer, self)

#         # Action client for navigation
#         self.nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
#         self.nav_action_client.wait_for_server()

#         # Timers
#         self.status_timer = self.create_timer(0.5, self.publish_status)
#         self.parameter_timer = self.create_timer(5.0, self.adjust_parameters)

#         # Parameters and state
#         self.costmap_data = None
#         self.costmap_resolution = None
#         self.costmap_origin = None
#         self.sensor_position_history = []
#         self.costmap_position_histories = {}
#         self.last_good_position = None
#         self.last_good_costmap_velocity = None
#         self.acceptable_distance = 0.8
#         self.monitoring_time = 5.0
#         self.max_history_length = 10

#         self.position_threshold = 0.75

#         self.object_in_front = False

#         # Añadimos el flag para controlar el estado de espera
#         self.awaiting_user_input = False
#         self.current_uwb_position = None  # Almacena temporalmente la posición en espera de validación manual

#         # Último objetivo enviado y monitoreo de movimiento
#         self.last_goal_position = None  # Almacena el último objetivo enviado
#         self.monitoring_start_time = None  # Hora de inicio del monitoreo
#         self.movement_threshold = 0.3  # Distancia mínima para considerar movimiento significativo
#         self.proximity_threshold = 0.5  # Distancia para determinar si el robot está cerca del último objetivo
#         self.movement_check_interval = 5.0  # Tiempo máximo para detectar movimiento
#         self.awaiting_movement = False  # Indica si estamos esperando movimiento

#         # Nueva variable para monitoreo
#         self.monitoring_with_lidar = True  # Indica si estamos monitoreando con el LIDAR
#         self.last_goal_position = None  # Último objetivo al que el robot se dirigió
#         self.awaiting_movement = False  # Indica si esperamos movimiento del objeto

#         self.get_logger().info("PositionFollower node initialized.")

#     ### CALLBACKS

#     def callback_uwb_position(self, point_msg):
#         """Procesa la posición UWB y solicita validación manual del usuario."""
#         # if math.isnan(point_msg.x) or math.isnan(point_msg.y):
#         #     self.get_logger().warning("Received NaN values in UWB position, ignoring...")
#         #     return

#         # Ignorar nuevas posiciones mientras se espera la validación manual
#         # if self.awaiting_user_input:
#         #     # self.get_logger().info("Awaiting user input, ignoring new UWB data...")
#         #     return

#         # self.current_uwb_position = point_msg
#         # self.awaiting_user_input = True

#         # # Pedir input manual al usuario
#         # self.get_logger().info(f"Received UWB position: {point_msg}. Is this position valid? (y/n)")
        
#         # # Capturar input manual en un hilo separado
#         # import threading
#         # threading.Thread(target=self.wait_for_user_input).start()

#     def callback_costmap(self, costmap_msg):
#         """Procesa datos del costmap y actualiza posiciones ocupadas."""
#         self.costmap_data = np.array(costmap_msg.data).reshape(
#             (costmap_msg.info.height, costmap_msg.info.width)
#         )
#         self.costmap_resolution = costmap_msg.info.resolution
#         self.costmap_origin = costmap_msg.info.origin

#         for y in range(self.costmap_data.shape[0]):
#             for x in range(self.costmap_data.shape[1]):
#                 if self.costmap_data[y, x] > 0:  # Celda ocupada
#                     position = Point(
#                         x=self.costmap_origin.position.x + x * self.costmap_resolution,
#                         y=self.costmap_origin.position.y + y * self.costmap_resolution,
#                         z=0.0
#                     )
#                     costmap_id = f"costmap_{x}_{y}"
#                     if costmap_id not in self.costmap_position_histories:
#                         self.costmap_position_histories[costmap_id] = []
#                     self.update_position_history(self.costmap_position_histories[costmap_id], position)

#                     self.send_new_goal(position)

#     def laser_callback(self, scan_msg):
#         """Procesa datos del LIDAR para detectar movimiento."""
#         if not self.monitoring_with_lidar:
#             return  # Solo procesar datos si estamos monitoreando

#         # Filtrar ángulos del LIDAR para detectar objetos frente al robot
#         angle_min = -0.25  # -45 grados
#         angle_max = 0.25   # 45 grados
#         ranges = [
#             distance for i, distance in enumerate(scan_msg.ranges)
#             if angle_min <= scan_msg.angle_min + i * scan_msg.angle_increment <= angle_max
#         ]
        
#         # Verificar si hay objetos dentro de la distancia aceptable
#         self.object_in_front = any(distance < self.acceptable_distance for distance in ranges if distance > 0)

#         # Si no hay objetos en frente, marcar como inactivo
#         if not self.object_in_front:
#             self.get_logger().warning("No object detected in front. Holding navigation...")
#             self.awaiting_movement = True
#             return
        
#         # Iniciar monitoreo si no se ha iniciado
#         if self.awaiting_movement and self.monitoring_start_time is None:
#             self.monitoring_start_time = self.get_clock().now()
#             self.get_logger().info("Started monitoring object movement.")

#         # Si el objeto está en frente, verificar movimiento
#         self.check_object_movement()

#     ### NAVIGATION

#     def send_new_goal(self, point_msg):
#         """Envía un nuevo objetivo si el objetivo se mueve o está lejos del último objetivo."""
#         if self.last_goal_position:
#             # Verificar si estamos cerca del último objetivo
#             distance_to_goal = self.calculate_distance(self.last_goal_position, point_msg)
#             if distance_to_goal <= self.proximity_threshold:
#                 if not self.awaiting_movement:
#                     self.get_logger().info("Close to last goal. Waiting for significant movement...")
#                     self.awaiting_movement = True
#                     self.monitoring_start_time = self.get_clock().now()
#                 else:
#                     # Verificar si se detectó movimiento dentro del intervalo
#                     elapsed_time = self.get_clock().now() - self.monitoring_start_time
#                     elapsed_seconds = elapsed_time.nanoseconds / 1e9
#                     if elapsed_seconds > self.movement_check_interval:
#                         self.get_logger().warning("No significant movement detected. Holding navigation updates...")
#                         return
#                 # Si hay movimiento significativo, continuar
#                 if self.detect_significant_movement(point_msg):
#                     self.get_logger().info("Significant movement detected. Sending new goal.")
#                     self.awaiting_movement = False
#                 else:
#                     return  # No enviar nuevo objetivo si no hay movimiento significativo
                
#         # Enviar nuevo objetivo
#         new_goal = self.transform_to_map_frame(point_msg)
#         if new_goal:
#             goal_msg = NavigateToPose.Goal()
#             goal_msg.pose = new_goal
#             self.last_goal_position = point_msg  # Actualizar el último objetivo enviado
#             future = self.nav_action_client.send_goal_async(goal_msg)
#             # future.add_done_callback(self.nav_goal_response_callback)
#             self.get_logger().info(f"Sent navigation goal: {new_goal.pose.position}")



#     ### COSTMAP REFINEMENT

#     def refine_position_with_costmap(self, uwb_position):
#         """Mejora la posición usando datos del costmap y patrones de movimiento."""
#         if self.costmap_data is None:
#             self.get_logger().warning("Costmap data not available.")
#             return uwb_position

#         self.update_position_history(self.sensor_position_history, uwb_position)
#         refined_position = None
#         min_similarity = float('inf')

#         for costmap_id, costmap_history in self.costmap_position_histories.items():
#             sensor_velocity = self.calculate_velocity(self.sensor_position_history)
#             costmap_velocity = self.calculate_velocity(costmap_history)
#             similarity = self.compare_trajectories(sensor_velocity, costmap_velocity)

#             if similarity < min_similarity:
#                 min_similarity = similarity
#                 refined_position = costmap_history[-1]

#         if refined_position:
#             return refined_position
#         else:
#             return uwb_position

#     def update_position_history(self, history, new_position):
#         """Actualiza el historial de posiciones con una nueva entrada."""
#         history.append(new_position)
#         if len(history) > self.max_history_length:
#             history.pop(0)

#     ### UTILITIES

#     def follow_object_with_costmap(self):
#         """Sigue al objeto calculando objetivos basados en el costmap."""
#         if not self.last_good_costmap_velocity or not self.last_goal_position:
#             return  # Nada que seguir si no hay datos previos

#         adjusted_goal = Point(
#             x=self.last_goal_position.x + self.last_good_costmap_velocity[0] * 0.5,
#             y=self.last_goal_position.y + self.last_good_costmap_velocity[1] * 0.5,
#             z=0.0
#         )
#         self.get_logger().info(f"Following object at adjusted position: {adjusted_goal}")
#         self.send_new_goal(adjusted_goal)


#     def detect_significant_movement(self, current_position):
#         """Verifica si el objetivo se ha movido significativamente."""
#         if not self.last_goal_position:
#             return True  # Si no hay objetivo previo, asumimos movimiento significativo

#         distance = self.calculate_distance(self.last_goal_position, current_position)
#         if distance >= self.movement_threshold:
#             self.get_logger().info(f"Movement detected: {distance}m")
#             return True  # Movimiento significativo detectado

#         self.get_logger().info(f"No significant movement detected: {distance}m")
#         return False


#     def detect_significant_movement(self, current_position):
#         """Verifica si el objetivo se ha movido significativamente."""
#         if not self.last_goal_position:
#             return True  # Si no hay objetivo previo, asumimos movimiento significativo

#         distance = self.calculate_distance(self.last_goal_position, current_position)
#         if distance >= self.movement_threshold:
#             self.get_logger().info(f"Movement detected: {distance}m")
#             return True  # Movimiento significativo detectado

#         self.get_logger().info(f"No significant movement detected: {distance}m")
#         return False


#     def wait_for_user_input(self):
#         """Espera la entrada manual del usuario para validar la posición."""
#         while True:
#             user_input = input("Enter 'y' or 'n': ").strip().lower()
#             if user_input in ['y', 'n']:
#                 self.process_user_input(user_input)
#                 break
#             print("Invalid input. Please enter 'y' or 'n'.")


#     def process_user_input(self, user_input):
#         """Procesa la respuesta del usuario y actúa en consecuencia."""
#         if user_input == 'y':
#             self.get_logger().info("User confirmed position is valid. Refining and sending goal...")
#             refined_position = self.refine_position_with_costmap(self.current_uwb_position)
#             if refined_position:
#                 self.send_new_goal(refined_position)
#                 self.monitoring_with_lidar = True  # Iniciar monitoreo con LIDAR
#         else:
#             self.get_logger().info("User rejected position. Ignoring current UWB position.")

#         # Resetear el estado para recibir nuevas posiciones
#         self.awaiting_user_input = False
#         self.current_uwb_position = None


#     def calculate_distance(self, position1, position2):
#         dx = position1.x - position2.x
#         dy = position1.y - position2.y
#         return math.sqrt(dx**2 + dy**2)
    
#     def create_pose_stamped(self, point):
#         pose = PoseStamped()
#         now = rclpy.time.Time()
#         pose.header.stamp = now.to_msg()
#         pose.header.frame_id = "map"
#         pose.pose.position.x = point.x
#         pose.pose.position.y = point.y
#         pose.pose.position.z = point.z
#         pose.pose.orientation.w = 1.0
#         return pose

#     def transform_to_map_frame(self, point_msg):
#         """Transforma coordenadas del marco base al marco map."""
#         try:
#             now = rclpy.time.Time()
#             transform = self.tf_buffer.lookup_transform('map', 'base_link', now)
#             robot_position = transform.transform.translation
#             robot_orientation = transform.transform.rotation

#             _, _, yaw = self.quaternion_to_euler(
#                 robot_orientation.x, robot_orientation.y,
#                 robot_orientation.z, robot_orientation.w
#             )

#             sensor_x = point_msg.x * math.cos(yaw) - point_msg.y * math.sin(yaw)
#             sensor_y = point_msg.x * math.sin(yaw) + point_msg.y * math.cos(yaw)

#             pose = PoseStamped()
#             pose.header.stamp = now.to_msg()
#             pose.header.frame_id = "map"
#             pose.pose.position.x = robot_position.x + sensor_x
#             pose.pose.position.y = robot_position.y + sensor_y
#             pose.pose.position.z = 0.0
#             pose.pose.orientation.w = 1.0
#             return pose

#         except Exception as e:
#             self.get_logger().error(f"Error in transform_to_map_frame: {e}")
#             return None

#     def quaternion_to_euler(self, x, y, z, w):
#         """Convierte un cuaternión a ángulos de Euler (yaw)."""
#         t0 = +2.0 * (w * x + y * z)
#         t1 = +1.0 - 2.0 * (x * x + y * y)
#         yaw = math.atan2(t0, t1)
#         return 0.0, 0.0, yaw

#     def calculate_velocity(self, history):
#         """Calcula la velocidad promedio usando el historial de posiciones."""
#         if len(history) < 2:
#             return None
#         velocities = [
#             ((history[i].x - history[i - 1].x) / 0.5,
#              (history[i].y - history[i - 1].y) / 0.5)
#             for i in range(1, len(history))
#         ]
#         avg_velocity = (
#             sum(v[0] for v in velocities) / len(velocities),
#             sum(v[1] for v in velocities) / len(velocities)
#         )
#         return avg_velocity

#     def compare_trajectories(self, sensor_velocity, costmap_velocity):
#         """Compara trayectorias del sensor y del costmap."""
#         if not sensor_velocity or not costmap_velocity:
#             return float('inf')
#         dx = sensor_velocity[0] - costmap_velocity[0]
#         dy = sensor_velocity[1] - costmap_velocity[1]
#         return math.sqrt(dx**2 + dy**2)

#     def publish_status(self):
#         """Publica el estado del robot en el tópico /follower_status."""
#         status_msg = Bool()
#         status_msg.data = True
#         self.status_publisher_.publish(status_msg)

#     def adjust_parameters(self):
#         """Ajusta parámetros dinámicamente según el entorno."""
#         if self.costmap_data is None:
#             return
#         occupied_cells = np.sum(self.costmap_data > 0)
#         total_cells = self.costmap_data.size
#         occupancy_ratio = occupied_cells / total_cells
#         self.max_history_length = 5 if occupancy_ratio > 0.5 else 10

#     def check_object_movement(self):
#         """Verifica si el objeto detectado se ha movido."""

#         self.monitoring_start_time = self.get_clock().now()

#         elapsed_time = self.get_clock().now() - self.monitoring_start_time
#         elapsed_seconds = elapsed_time.nanoseconds / 1e9
#         if elapsed_seconds > self.movement_check_interval:
#             self.get_logger().warning("Object not moving. Pausing navigation updates...")
#             self.monitoring_start_time = None  # Resetear tiempo de monitoreo
#             return  # No continuar si no hay movimiento

#         # Detectar movimiento significativo
#         if self.detect_significant_movement(self.last_goal_position):
#             self.get_logger().info("Object movement detected. Switching to costmap following.")
#             self.monitoring_with_lidar = False
#             self.awaiting_movement = False
#             # self.monitoring_start_time = None  # Resetear tiempo de monitoreo
#             # Continuar siguiendo el costmap
#             self.follow_object_with_costmap()

#         elapsed_time = self.get_clock().now() - self.monitoring_start_time
#         elapsed_seconds = elapsed_time.nanoseconds / 1e9
#         if elapsed_seconds > self.monitoring_time:
#             self.get_logger().info("Object validated based on movement.")
#             self.valid_costmap = True
#             self.monitoring_start_time = None

#     def destroy_node(self):
#         """Cierra el nodo limpiamente."""
#         self.publish_status(False)
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