#!/usr/bin/env python

import rclpy
from geometry_msgs.msg import Twist
import time
from rclpy.node import Node

class RobotController(Node): # Genera Nodos.

    def __init__(self): # Llama al constructor de la clase base 
        super().__init__('robot_controller') # Inicia el nodo "robot_controller".
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10) # Publica mensajes de tipo Twist en el topic /cmd_vel
        self.k_linear = 0.05  # Velocidad lineal deseada (0.05 m/s) # Establece el valor de self.k_linear en 0.05, que representa la velocidad lineal deseada del robot en metros por segundo.
        self.started = False #  indica que el movimiento del robot no ha comenzado.
        self.start_time = None # se utiliza para registrar el tiempo en que comienza el movimiento del robot.
        self.duration = 20  # Duración del movimiento en segundos

    def move_forward(self):
        cmd_vel_msg = Twist() # Crea un mensaje de tipo Twist llamado cmd_vel_msg
        cmd_vel_msg.linear.x = self.k_linear  # Establece la velocidad lineal deseada
        self.cmd_vel_pub.publish(cmd_vel_msg) # Publica el mensaje de velocidad lineal en el topic /cmd_vel utilizando el publicador self.cmd_vel_pub.

    def start_movement(self):
        self.started = True   # Indica que el movimiento del robot ha comenzado.
        self.start_time = time.time() # Registra el tiempo actual
        self.move_forward() # LLama al método move_forward para iniciar el movimiento del robot.
        print("Movimiento del Robot ha comenzado.") # Imprime/indica que el movimiento ha comenzado

    def stop_movement(self):
        cmd_vel_msg = Twist() # Crea un nuevo mensaje de tipo Twist llamado cmd_vel_msg.
        self.cmd_vel_pub.publish(cmd_vel_msg) # Publica este mensaje vacío en el topic /cmd_vel para detener el movimiento del robot.
        print("Robot se ha detenido.") # Imprime/indica que el movimiento ha sido detenido.
        
def main(args=None):
    rclpy.init(args=args) # Inicia el Nodo

    try:
        controller = RobotController() #  Esto inicializa el nodo del controlador del robot y configura todos los parámetros necesarios para controlar el movimiento del robot.

        # Inicia el movimiento tan pronto como se inicie el nodo
        controller.start_movement()

        # Espera durante el tiempo especificado antes de detener el movimiento
        while time.time() - controller.start_time < controller.duration:
            pass

        # Detiene el movimiento después de 20 segundos
        controller.stop_movement()

    except KeyboardInterrupt: # Cuando el usuario presiona Ctrl+C para detener el programa, ermite que el programa termine de manera controlada.
        pass

    finally:
        rclpy.shutdown() #  Limpia los recursos y libera los nodos y publicadores/subscriptores.

if __name__ == '__main__': # Verifica si el script se está ejecutando directamente como un programa principal
    main()



