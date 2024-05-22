# Código del cliente de acción (call_move_robot) 
import rclpy 
from rclpy.node import Node 
from rclpy.action import ActionClient 
from my_robot_interfaces.action import LlamadaRobot 
class CallMoveRobot(Node): 
    def __init__(self): 
        super().__init__('movimiento_actionClient') 
        self.client = ActionClient(self, LlamadaRobot, 'move_robot') 
    def enviar_peticion(self, value): 
        msg = LlamadaRobot.Goal() 
        msg.value = value 
        self.get_logger().info('Conectando con el servidor...') 
        self.client.wait_for_server() 
        self.get_logger().info('Servidor conectado. Enviando petición...') 
        self.action_future = self.client.send_goal_async(msg) 
        self.action_future.add_done_callback(self.resultado_callback) 
    def resultado_callback(self, future): 
        _ = future.result()  # Ignorar el resultado 
        self.get_logger().info('Fin de la acción.') 
        rclpy.shutdown() 
def main(args=None): 
    rclpy.init(args=args) 
    movimiento_action_client = CallMoveRobot() 
    L = float(input('Longitud del lado del triángulo: ')) 
    movimiento_action_client.enviar_peticion(L) 
    rclpy.spin_once(movimiento_action_client)  # Solo ejecutar una vez para enviar la petición 
    rclpy.shutdown() 
if __name__ == '__main__': 
    main() 








