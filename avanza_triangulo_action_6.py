import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from my_robot_interfaces.action import LlamadaRobot
import math
from tf_transformations import euler_from_quaternion

class RobotController(Node): 
    def __init__(self): 
        super().__init__('Robot_Controller') 
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10) 
        self.srv = ActionServer(self, LlamadaRobot, 'move_robot', self.move_robot_callback) 
        self.feedback = LlamadaRobot.Feedback() 
        # Inicializa las variables de posición de la odometría 
        self.odom_position_x = 0.0 
        self.odom_position_y = 0.0 
        self.odom_orientation_z = 0.0 
        # Suscribe al tema de la odometría 
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10) 
    def odom_callback(self, msg): 
        # Actualiza las variables de posición de la odometría en el callback 
        self.odom_position_x = msg.pose.pose.position.x 
        self.odom_position_y = msg.pose.pose.position.y 
        _, _, self.odom_orientation_z = euler_from_quaternion([ 
            msg.pose.pose.orientation.x, 
            msg.pose.pose.orientation.y, 
            msg.pose.pose.orientation.z, 
            msg.pose.pose.orientation.w 
        ]) 




    def move_robot_callback(self, goal_handle): 
        L = goal_handle.request.value 
    
    # Realizar los movimientos del triángulo
        for _ in range(3):
            self.move_forward(L)  # Mover hacia adelante
            self.stop_robot()      # Detener el robot para ajustar la posición
            self.rotate(120)       # Rotar 120 grados

    # Después de completar el triángulo, detener el robot 
        self.stop_robot()
    # Establecer el estado del objetivo como exitoso
        goal_handle.succeed()
        
        response = LlamadaRobot.Result() 
        response.success = True 
        return response 

        
    def move_forward(self, distance): 
        cmd_vel_msg = Twist() 
        cmd_vel_msg.linear.x = 0.05 
        self.cmd_vel_pub.publish(cmd_vel_msg) 
        initial_x = self.odom_position_x 
        initial_y = self.odom_position_y 
        while math.sqrt((self.odom_position_x - initial_x) ** 2 + (self.odom_position_y - initial_y) ** 2) < distance: 
            rclpy.spin_once(self)  # Actualiza la posición 
        cmd_vel_msg.linear.x = 0.0 
        self.cmd_vel_pub.publish(cmd_vel_msg) 
        
    def rotate(self, angle): 
        cmd_vel_msg = Twist() 
        cmd_vel_msg.angular.z = 0.1       # math.radians(30)  Velocidad angular para girar 
        self.cmd_vel_pub.publish(cmd_vel_msg) 
        # Calcular el ángulo de destino 
        current_yaw = self.odom_orientation_z 
        target_yaw = current_yaw + math.radians(angle) 
        if target_yaw > math.pi: 
           target_yaw -= 2 * math.pi 
        elif target_yaw < -math.pi: 
           target_yaw += 2 * math.pi 
        # Esperar hasta que el robot alcance el ángulo deseado 
        while abs(self.odom_orientation_z - target_yaw) > 0.1:  # Ajustamos la tolerancia 
            rclpy.spin_once(self)  # Actualiza la orientación 
        cmd_vel_msg.angular.z = 0.0 
        self.cmd_vel_pub.publish(cmd_vel_msg) 

    def stop_robot(self):
        cmd_vel_msg = Twist()
        self.cmd_vel_pub.publish(cmd_vel_msg)

def main(args=None):
    rclpy.init(args=args)
    controller = RobotController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



