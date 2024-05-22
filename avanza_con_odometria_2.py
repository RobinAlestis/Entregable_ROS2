import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

class RobotController(Node): # Genera Nodos.

    def __init__(self): # Llama al constructor de la clase base 
        super().__init__('robot_controller') # Inicia el nodo "robot_controller".
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)  # Publica mensajes de tipo Twist en el topic /cmd_vel
        self.k_linear = 0.05  # Velocidad lineal deseada (0.05 m/s)  Establece el valor de self.k_linear en 0.05, que representa la velocidad lineal deseada del robot en metros por segundo.
        self.target_distance = 1.0  # Distancia objetivo en metros
        self.started = False #  indica que el movimiento del robot no ha comenzado.
        self.initial_position = None # 
        self.odom_sub = None 

    def move_forward(self):
        cmd_vel_msg = Twist() # Crea un mensaje de tipo Twist llamado cmd_vel_msg
        cmd_vel_msg.linear.x = self.k_linear  # Establece la velocidad lineal deseada
        self.cmd_vel_pub.publish(cmd_vel_msg) # Publica el mensaje de velocidad lineal en el topic /cmd_vel utilizando el publicador self.cmd_vel_pub.
        self.started = True # Indica que el movimiento del robot ha comenzado
        self.initial_position = None  # Resetea la posición inicial
        print("Movimiento del Robot ha comenzado.") # Imprime/indica que el movimiento ha comenzado.

    def odom_callback(self, msg): # Invoca cada vez que se recibe un mensaje de odometría (msg) en el nodo
        if self.started: # Asegura que el movimiento del robot esté en curso 
            if self.initial_position is None: # Lo que significa que la posición inicial aún no se ha establecido
                self.initial_position = msg.pose.pose.position.x 
            distance_traveled = abs(msg.pose.pose.position.x - self.initial_position) #  Calcula la distancia recorrida por el robot desde su posición inicial hasta su posición actual.
            if distance_traveled >= self.target_distance: # Establece que el robot ha alcanzado o superado la distancia deseada y debe detenerse
                self.stop_movement() #  Si el robot ha alcanzado o superado la distancia objetivo, se llama al método stop_movement() para detener el movimiento del robot.

    def stop_movement(self):
        cmd_vel_msg = Twist() # Crea un nuevo mensaje de tipo Twist llamado cmd_vel_msg.
        self.cmd_vel_pub.publish(cmd_vel_msg) # Publica este mensaje vacío en el topic /cmd_vel para detener el movimiento del robot.
        print("El Robot ha recorrido 1 metro. Detenido el movimiento.") # Imprime/indica que el movimiento se ha realizado.
        self.started = False # Se utiliza para indicar que el movimiento del robot ha finalizado
        self.initial_position = None 
        if self.odom_sub is not None: # Verifica si el atributo odom_sub no es None, lo que significa que hay un suscriptor de odometría activo.
            self.destroy_subscription(self.odom_sub) # Si hay un suscriptor de odometría activo,  se destruye este suscriptor utilizando el método destroy_subscription()

def main(args=None):
    rclpy.init(args=args)

    try:
        controller = RobotController() # inicia un nodo de ROS 2 que controlará el movimiento del robot.
        controller.move_forward() # inicia el movimiento del robot hacia adelante.
        controller.odom_sub = controller.create_subscription(Odometry, '/odom', controller.odom_callback, 10) #  Crea un suscriptor (Subscriber) para el topic /odom que recibirá mensajes de odometría del robot.
        rclpy.spin(controller)  # Espera a que lleguen mensajes de odometría y publique mensajes de velocidad

    except KeyboardInterrupt: # Cuando el usuario presiona Ctrl+C para detener el programa, ermite que el programa termine de manera controlada.
        pass

    finally:
        rclpy.shutdown() #  Limpia los recursos y libera los nodos y publicadores/subscriptores.

if __name__ == '__main__': # Verifica si el script se está ejecutando directamente como un programa principal
    main()









