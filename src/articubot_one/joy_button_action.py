import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool

class JoyButtonAction(Node):

    def __init__(self):
        super().__init__('joy_button_action')
        
        # Crear suscriptor al tópico del joystick
        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        # Crear publicador para el tópico booleano
        self.bool_publisher = self.create_publisher(Bool, '/servo_hook', 10)
        
        # Inicializar el estado de los botones y del toggle
        self.button_0_state = False  # Estado inicial del toggle del botón 0
        self.button_0_last_state = 0  # Estado anterior del botón 0

    def joy_callback(self, msg):
        # Revisar el botón 0 para cambiar el estado del booleano
        current_button_0_state = msg.buttons[3]

        if current_button_0_state == 1 and self.button_0_last_state == 0:
            # El botón 0 acaba de ser presionado
            self.button_0_state = not self.button_0_state  # Cambia el estado del toggle

            # Crear el mensaje booleano con el nuevo estado
            bool_msg = Bool()
            bool_msg.data = self.button_0_state

            # Publicar el estado en el tópico
            self.bool_publisher.publish(bool_msg)

        # Guarda el estado actual del botón para la próxima llamada
        self.button_0_last_state = current_button_0_state

def main(args=None):
    rclpy.init(args=args)
    joy_button_action = JoyButtonAction()
    rclpy.spin(joy_button_action)
    joy_button_action.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
