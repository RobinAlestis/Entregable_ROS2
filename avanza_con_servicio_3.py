import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from my_robot_interfaces.srv import MoveRobot  # Importa el servicio MoveRobot
import math 
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor 
from rclpy.callback_groups import ReentrantCallbackGroup


class RobotController(Node):

    def __init__(self):
        super().__init__('Robot_Controler')
        self.reentrant_group = ReentrantCallbackGroup() # Crea el grupo de callbacks recurrente
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10,callback_group=self.reentrant_group)  # Crea un publicador en el topico /cmd_vel, tipo twist
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10, callback_group=self.reentrant_group)  # Crea un suscriptor en el topico /odom, tipo Odometry
        self.loop_rate = self.create_rate(10, self.get_clock()) # crea un objeto Rate que controlará la frecuencia de bucle del nodo
        self.odom_position_x = 0.0 # Establece la posición x de la odometría en 0.0. 
        self.odom_position_y = 0.0 # Establece la posición y de la odometría en 0.0. 
        self.inicio_x = 1.0 # Establece la posición inicial en el eje x en 1.0. 
        self.inicio_y = 0.0 # Establece la posición inicial en el eje y en 0.0. 
        self.k_linear = 0.1 # Establece el coeficiente de velocidad lineal en 0.1.
        self.k_angular = 0.0 #  Establece el coeficiente de velocidad angular en 0.0. 
        self.started = False # Establece el atributo started en False. 
        self.srv = self.create_service(MoveRobot, 'move_robot', self.move_robot_callback, callback_group=self.reentrant_group) # Crea un servicio llamado 'move_robot' utilizando el tipo de mensaje MoveRobot
        

    def odom_callback(self, msg):
        self.odom_position_x = msg.pose.pose.position.x # Establece el valor de odom_position_x en la posición x del mensaje de odometría msg.
        self.odom_position_y = msg.pose.pose.position.y # Establece el valor de odom_position_y en la posición y del mensaje de odometría msg.

    def move_to_target(self):
        dx = self.odom_position_x - self.inicio_x # Calcula las diferencias en la posición entre la posición actual del robot 
        dy = self.odom_position_y - self.inicio_y
        distance_to_target = (dx**2 + dy**2)**0.5 # Calcula la distancia euclidiana entre la posición actual del robot y la posición inicial.
        self.get_logger().info(str(distance_to_target)) #  Registra la distancia al objetivo utilizando el método info del logger del nodo. 

        target_angle = math.atan2(dy, dx) # Calcula el ángulo hacia el objetivo utilizando las diferencias en las coordenadas y. atan2 es una función trigonométrica que calcula el ángulo en radianes entre el eje x positivo y el vector (dy, dx).
        error_angle = target_angle # El error de ángulo se establece inicialmente como el ángulo hacia el objetivo.
        cmd_vel_msg = Twist() # Crea un mensaje de tipo Twist
        cmd_vel_msg.angular.z = self.k_angular * error_angle #  Calcula la velocidad angular del robot multiplicando el error de ángulo por el coeficiente k_angular. Esta velocidad angular se utiliza para ajustar la dirección del movimiento del robot hacia el objetivo.

        if distance_to_target >= 1.0: # Comprueba si la distancia al objetivo (distance_to_target) es mayor o igual a 1.0 metro. Si es así, el robot se considera lo suficientemente cerca del objetivo y se detiene.
            cmd_vel_msg.linear.x = 0.0 # Establece velocidad lineal 0
            self.started = False
        else:
            cmd_vel_msg.linear.x = 0.05 # Establece velocidad lineal 0.05 m/s
        
        self.cmd_vel_pub.publish(cmd_vel_msg) #  Publica el mensaje de velocidad (cmd_vel_msg) en el topic /cmd_vel

    def move_robot_callback(self, request, response): # Maneja las solicitudes de movimiento del robot a través de un servicio
        self.started = True # Indica que el movimiento del robot ha comenzado.
        self.inicio_x = self.odom_position_x # Actualiza la posición inicial en el eje x 
        self.inicio_y = self.odom_position_y # Actualiza la posición inicial en el eje y
        while rclpy.ok(): # Inicia un bucle 
            if self.started: # Comprueba si el movimiento del robot ha comenzado.
                self.move_to_target() #  Llama al método move_to_target(), que calcula y controla el movimiento del robot hacia un objetivo específico.
            else: # Si el movimiento del robot no ha comenzado, pues..
                self.stop() #  Llama al método stop() para detener cualquier movimiento del robot.
                break # Sale del bucle
            self.loop_rate.sleep() # Duerme el hilo de ejecución durante un tiempo calculado para mantener la frecuencia de bucle especificada en self.loop_rate.
        response.success = True # indica que la solicitud de movimiento del robot se ha completado con éxito.
        return response # Devuelve la respuesta al cliente que realizó la solicitud de movimiento del robot.

    def stop(self):
        cmd_vel_msg = Twist() # Este mensaje se utiliza para controlar la velocidad y la dirección del movimiento del robot.
        self.cmd_vel_pub.publish(cmd_vel_msg) # Publica el mensaje de velocidad vacío en el topic /cmd_vel utilizando el publicador cmd_vel_pub. 

def main(args=None):
    rclpy.init(args=args)
    controller = RobotController()
        # Crea el executor multihilo y le añade el nodo 
    executor = MultiThreadedExecutor(num_threads=2) 
    executor.add_node(controller) 
        # Inicia el executor 
    executor.spin() 
        # Destruye el nodo 
    controller.destroy_node() 
       # Finaliza la ejecución 
    rclpy.shutdown()

if __name__ == '__main__':
    main()










