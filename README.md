Estos ejercicios implementan nodos ROS 2 para controlar un robot móvil. Utilizan mensajes de velocidad y odometría para mover el robot hacia adelante, girarlo, o seguir trayectorias específicas. Los nodos manejan las solicitudes de movimiento a través de servicios y acciones ROS, proporcionando control preciso y retroalimentación continua durante el proceso. La descripcion general de cada uno:

Avanza 1 metro:  Este código controla un robot en ROS 2 para que se mueva hacia adelante a una velocidad constante durante 20 segundos y luego se detenga. Usa mensajes de Twist para controlar la velocidad del robot y mide el tiempo transcurrido para determinar cuándo detener el movimiento.

Avanza con odometria: Este código controla un robot en ROS 2 para que se mueva hacia adelante 1 metro y luego se detenga, utilizando mensajes de odometría para medir la distancia recorrida y mensajes de velocidad para controlar el movimiento.

Avanza con servicio: este código implementa un nodo ROS 2 que controla un robot móvil, permitiéndole moverse hacia una posición objetivo utilizando mensajes de velocidad y odometría, y gestionando las solicitudes de movimiento a través de un servicio ROS personalizado.

Avanza lineal o rotacion: Este código implementa un nodo ROS 2 que controla un robot móvil. Permite mover el robot hacia una posición específica o girarlo hacia un ángulo objetivo, utilizando mensajes de velocidad y odometría, y gestionando las solicitudes de movimiento a través de un servicio ROS personalizado.

Avanza lineal o rotacion action: Este código implementa un nodo ROS 2 que controla un robot móvil utilizando un ActionServer. Permite mover el robot hacia una posición específica o girarlo hacia un ángulo objetivo, proporcionando retroalimentación continua durante el proceso y manejando las solicitudes de movimiento a través de acciones ROS personalizadas.

Avanza triangulo action: Este código implementa un nodo ROS 2 que controla un robot móvil para realizar movimientos en un patrón de triángulo equilátero, utilizando un ActionServer para recibir y manejar las solicitudes de movimiento. El robot se mueve hacia adelante una distancia específica, se detiene, rota 120 grados, y repite esto tres veces para completar el triángulo.
