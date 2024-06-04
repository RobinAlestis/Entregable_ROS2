import rclpy 
from my_robot_interfaces.srv import MoveRobot  # Importa el servicio MoveRobot 
# Espera a que el servicio 'move_robot' esté disponible, luego crea un cliente para este servicio y lo llama: 
def move_robot_client(): 
    node = rclpy.create_node('move_robot_client')  # Inicializa el nodo del cliente 
    client = node.create_client(MoveRobot, 'move_robot')  # Crea un cliente del servicio 
    while not client.wait_for_service(timeout_sec=1.0): # Espera hasta que el servicio "move_robot" esté disponible. Si el servicio no está disponible dentro de 1 segundo (timeout_sec=1.0), imprime un mensaje de advertencia. 
        print('Servicio "move_robot" no disponible, esperando ...') 
    request = MoveRobot.Request()  # Crea una solicitud vacía 
    # Nuevos parámetros 
    request.distance_to_travel = 3.0  # Distancia a recorrer 
    request.acceleration_profile = 0.1  # Perfil de aceleración 
    request.max_velocity = 0.2  # Velocidad máxima 
    request.deceleration_constant = 0.05  # Constante proporcional de deceleración 
    try: 
        future = client.call_async(request)  # Llama al servicio de forma asincrónica 
        rclpy.spin_until_future_complete(node, future)  # Espera a que se complete el servicio 
        if future.result() is not None: # Verifica si la llamada al servicio se completó correctamente. Si hay un resultado, imprime un mensaje de que el robot ha terminado el movimiento. 
            print("¡El robot ha terminado el movimiento!")  # Mensaje de información 
        else: 
            print("Error al llamar al servicio 'move_robot'")  # Mensaje de error 
    except Exception as e: 
        print(f"Error al llamar al servicio: {e}") 
    finally: 
        node.destroy_node() 
def main(args=None): 
    rclpy.init() 
    move_robot_client()  # Llama a la función del cliente 
    rclpy.shutdown() 
# Determina si el script se está ejecutando como un programa independiente o si se está importando como un módulo en otro script: 
if __name__ == '__main__': 
    main() 
