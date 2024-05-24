import rclpy
from rclpy.node import Node 
from rclpy.qos import QoSProfile
from rclpy.executors import MultiThreadedExecutor
from my_robot_interfaces.action import LlamadaRobot
from geometry_msgs.msg import Twist
from rclpy.action import ActionClient

import math

class call_move_robot(Node): # Inicializamos el nodo con el nombre movimiento_actionClient.

    def __init__(self, tipo = 0, distancia = 1.0): # Creamos un cliente de acción para el tipo de acción LlamadaRobot 
        super().__init__('movimiento_actionClient')
        self.client = ActionClient(self,LlamadaRobot, 'move_robot')

    def enviar_peticion(self, value, linear): # Asignamos los valores value y linear al mensaje.
        msg = LlamadaRobot.Goal()
        msg.value = value
        msg.linear = linear

    
        self.get_logger().info('Conectando con el servidor...') # Esperamos a que el servidor de acciones esté disponible.
        self.client.wait_for_server()
        
        self.get_logger().info('Servidor conectado. Enviando petición...') # Envíamos la petición de acción y registramos un callback para manejar la respuesta del estado de la petición
        self.action_future = self.client.send_goal_async(msg, feedback_callback=self.feedback_callback)
        self.action_future.add_done_callback(self.estado_peticion_callback)
        return
    def estado_peticion_callback(self, future): # Se ejecuta cuando se recibe la respuesta del servidor sobre si la petición de acción fue aceptada
       
        action_handle = future.result()
        if not action_handle.accepted:
            self.get_logger().info('Petición rechazada') # Si la petición es rechazada, se informa.
            return
        self.get_logger().info('Petición aceptada') # Si la petición es aceptada, se registra un callback para manejar el resultado de la acción.
        self.resultado_future = action_handle.get_result_async()
        self.resultado_future.add_done_callback(self.resultado_callback)

    def feedback_callback(self, feedback_msg): # Ejecutamos para recibir retroalimentación periódica del servidor mientras la acción está en curso
        feedback = feedback_msg.feedback
        distance_to_target = feedback.distance_to_target 
        angle_to_target = feedback.angle_to_target
        self.get_logger().info('La distancia recorrida: {}'.format(distance_to_target))  # Informamos sobre la distancia recorrida y el ángulo restante para alcanzar el objetivo
        self.get_logger().info('Ángulo de rotación faltante para el objetivo: {}'.format(angle_to_target))
        
    def resultado_callback(self, future): #Ejecutamos cuando se recibe el resultado final de la acción:
        result = future.result().result
        self.get_logger().info('Fin de la acción. Resultado: {}'.format(result.success)) # Informamos sobre el éxito de la acción.
        rclpy.shutdown()
        
def main(args=None):

    rclpy.init(args=args) # Inicializamos el sistema ROS 2.
    movimiento_actionClient = call_move_robot() # Creamos una instancia de call_move_robot.
    linear = bool(int(input('Tipo de movimiento: 1 para lineal, 0 para rotación\n'))) # Envíamos la petición de acción al servidor de acuerdo a los valores ingresados.
    print(linear) 
    value = float(input('Magnitud del movimiento: metros en lineal, ángulos sexagesimales en rotación\n'))
    movimiento_actionClient.enviar_peticion(value, linear)
    rclpy.spin(movimiento_actionClient) # Mantenemos el nodo en funcionamiento usando rclpy.spin.
 
if __name__ == '__main__': # Llamamos a la función main() si el script se ejecuta directamente.
    main()
