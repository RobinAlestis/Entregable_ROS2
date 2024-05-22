import rclpy 
from geometry_msgs.msg import Twist 
from nav_msgs.msg import Odometry 
from my_robot_interfaces.srv import MoveRobot  # Importa el servicio MoveRobot 
import math 
from rclpy.node import Node 
from rclpy.callback_groups import ReentrantCallbackGroup 
from rclpy.executors import MultiThreadedExecutor  # Importa el executor multi-threaded 
class RobotController(Node): 
    def __init__(self): 
        super().__init__('Robot_Controller') 
        self.reentrant_group = ReentrantCallbackGroup()  # Crea el grupo de callbacks recurrente 
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10, callback_group=self.reentrant_group)  # Crea un publicador en el topico /cmd_vel, tipo twist
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10, callback_group=self.reentrant_group)  # Crea un suscriptor en el topico /odom, tipo Odometry
        self.odom_position_x = 0.0  # Establece la posición x de la odometría en 0.0.
        self.odom_position_y = 0.0  # Establece la posición y de la odometría en 0.0.
        self.odom_orientation_z = 0.0  # Añade la orientación del robot 
        self.inicio_x = 0.0 # Establece la posición inicial en el eje x en 0.0. 
        self.inicio_y = 0.0  # Establece la posición inicial en el eje y en 0.0. 
        self.started = False # Establece el atributo started en False.
        self.srv = self.create_service(MoveRobot, 'move_robot', self.move_robot_callback, callback_group=self.reentrant_group) # Crea un servicio llamado 'move_robot' utilizando el tipo de mensaje MoveRobot
        
    def degrees_to_radians(self, degrees): 
        """Convierte ángulos de grados a radianes.""" 
        return degrees * math.pi / 180.0 
    
    def odom_callback(self, msg): 
        """Callback para la actualización de la odometría.""" 
        self.odom_position_x = msg.pose.pose.position.x # Esto accede a la coordenada x de la posición del robot desde el mensaje de odometría (msg).
        self.odom_position_y = msg.pose.pose.position.y # Esto accede a la coordenada y de la posición del robot desde el mensaje de odometría (msg).
        self.odom_orientation_z = msg.pose.pose.orientation.z  # Actualiza la orientación del robot 
    
    def rotate_to_target(self, target_angle): 
        """Gira el robot hacia un ángulo objetivo.""" 
        error_angle = target_angle - self.odom_orientation_z  # Calcula el error de ángulo directamente desde la orientación actual 
        if error_angle > math.pi: # Comprueba si el error_angle es mayor que π.
            error_angle -= 2 * math.pi # Resta 2π de error_angle para llevarlo al rango de -π a π. Esto garantiza que el error_angle sea negativo y esté en el rango correcto.
        elif error_angle < -math.pi: # Comprueba si el error_angle es menor que -π.
            error_angle += 2 * math.pi # Suma 2π a error_angle para llevarlo al rango de -π a π. Esto garantiza que el error_angle sea positivo y esté en el rango correcto.
        
        print("Ángulo objetivo:", target_angle)
        print("Error de ángulo:", error_angle)        
        
        cmd_vel_msg = Twist() # # Crea un mensaje de tipo Twist
        cmd_vel_msg.angular.z = math.copysign(min(0.5, abs(error_angle)), error_angle) # Calcula y establece la velocidad angular del robot en el mensaje de velocidad (cmd_vel_msg)
        if abs(error_angle) < 0.1:  # Ajuste del umbral de error para la rotación 
            cmd_vel_msg.angular.z = 0.0 # Establece la velocidad angular del mensaje de velocidad en 0.0. Esto detiene la rotación del robot.
            self.started = False # Establece el atributo started en False, lo que indica que la rotación del robot ha finalizado.
            
            print("Alcanzado el objetivo de rotación") 
        
        self.cmd_vel_pub.publish(cmd_vel_msg) #  Publica el mensaje de velocidad (cmd_vel_msg) en el topic /cmd_vel
    
    def move_to_target(self, linear=True, value=1.0, angle_in_degrees=True): 
        """Mueve el robot hacia un objetivo lineal o angular.""" 
        cmd_vel_msg = Twist() 
        distance_to_target = 0.0  # Inicializa la variable 
        if angle_in_degrees: 
            target_angle = self.degrees_to_radians(value) 
        else: 
            target_angle = value 
        if linear: 
            distance_to_target = math.sqrt((self.odom_position_x - self.inicio_x)**2 + (self.odom_position_y - self.inicio_y)**2) 
            if distance_to_target > value: 
                cmd_vel_msg.linear.x = 0.0 
                self.started = False 
                print("Alcanzado el objetivo de lineal") 
            else: 
                cmd_vel_msg.linear.x = 0.05 
        else: 
            if not self.started: 
                cmd_vel_msg.angular.z = 0.0 
            self.rotate_to_target(target_angle) 
        
        self.cmd_vel_pub.publish(cmd_vel_msg) 
       
        # Imprimir información de movimiento 
        print("Distancia recorrida:", distance_to_target) 
        print("Ángulo al objetivo:", target_angle) 
        self.stop()  # Asegura que el robot se detenga al final 
        
    def move_robot_callback(self, request, response): 
        """Maneja las solicitudes de movimiento del robot.""" 
        self.started = True # se utiliza como una bandera para indicar si se ha iniciado un proceso o no.
        self.inicio_x = self.odom_position_x # asignanmos las coordenadas actuales de la posición ("odom_position_x" y "odom_position_y") a las variables "inicio_x" e "inicio_y", respectivamente.
        self.inicio_y = self.odom_position_y 
        linear = request.linear 
        value = request.value 
        if linear: # Se determina si el movimiento es lineal o no. 
            target_distance = value 
        else: 
            target_distance = abs(value) 
        while self.started: # continuará ejecutándose mientras el atributo "started" sea True. Dentro del bucle, se llama a la función 
            self.move_to_target(linear, value) 
            if linear and math.sqrt((self.odom_position_x - self.inicio_x)**2 + (self.odom_position_y - self.inicio_y)**2) > target_distance: # verifica si la distancia recorrida es mayor que la distancia objetivo. Si es así, el bucle se rompe, lo que probablemente detendrá el movimiento hacia el objetivo.
                break 
            elif not linear and abs(self.odom_orientation_z - self.inicio_x) < 0.1:  # Ajuste del umbral de error para la rotación 
                break 
        self.stop()  # Asegura que el robot se detenga al final 
        response.success = True 
        return response 
   
    def stop(self): 
        """Detiene el movimiento del robot.""" 
        cmd_vel_msg = Twist() 
        self.cmd_vel_pub.publish(cmd_vel_msg) 

def main(args=None): 
    rclpy.init(args=args) 
    controller = RobotController() 
    executor = MultiThreadedExecutor()  # Crea un executor multi-threaded y le añade el nodo 
    executor.add_node(controller) 
    executor.spin()  # Usa el executor en lugar de rclpy.spin(controller) 
    controller.destroy_node() 
    rclpy.shutdown() 
if __name__ == '__main__': 
    main()








