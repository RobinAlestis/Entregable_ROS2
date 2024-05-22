import rclpy 
from my_robot_interfaces.srv import MoveRobot 
import time  # Importamos el módulo time para usar la función sleep 

def call_move_robot(linear=True, value=1.0): #  Se define una función llamada call_move_robot que toma dos argumentos opcionales: linear y value
    node = rclpy.create_node('move_robot_client') # Se crea un nodo llamado "move_robot_client" utilizando la función create_node
    client = node.create_client(MoveRobot, 'move_robot') # Se crea un cliente del servicio llamado "move_robot" utilizando el nodo creado anteriormente. 
    # Esperamos hasta que el servicio esté disponible 
    while not client.wait_for_service(timeout_sec=1.0): # espera hasta que el servicio "move_robot" esté disponible.
        print('Servicio "move_robot" no disponible, esperando ...') 
        time.sleep(1)  # Esperamos 1 segundo antes de intentar nuevamente 
    print('Servicio "move_robot" disponible') 
    request = MoveRobot.Request() #  Se crea una solicitud vacía para el servicio "move_robot".
    # Se establecen los valores de la solicitud basados en los argumentos proporcionados a la función.
    request.linear = linear 
    request.value = value 
    
    future = client.call_async(request) # Se llama al servicio de manera asincrónica con la solicitud creada.
    try: 
        rclpy.spin_until_future_complete(node, future) # Espera a que se complete la llamada al servicio. Si se completa con éxito, se imprime un mensaje indicando que el robot ha terminado el movimiento. De lo contrario, se imprime un mensaje de error.
        response = future.result() 
        if response.success: 
            print("¡El robot ha terminado el movimiento!") 
        else: 
            print("Error al llamar al servicio 'move_robot'") 
    except Exception as e: 
        print(f"Error al llamar al servicio: {e}") 
    node.destroy_node() 
    rclpy.shutdown() 

def main(): 
    #while True: 
    if True: 
        rclpy.init() 
        linear = input("¿Movimiento lineal? (si/no): ").lower() == 'si' 
        if linear: 
            value = float(input("¿Cuántos metros? ")) 
        else: 
            value = float(input("¿Cuántos grados? ")) 
        
        call_move_robot(linear=linear, value=value) 
    
if __name__ == '__main__': 
    main()









