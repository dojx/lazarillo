# Robot Móvil Diferencial para Asistencia a Invidentes (Lazarillo)

A continuación se demuestra el prototipo de un robot móvil con configuración diferencial con el fin de ayudar a las personas que han adquirido recientemente alguna discapacidad visual. El robot tiene abordó un Kinect que sirve para la localización, navegación y evasión de obstáculos dentro de un plano.

El proyecto fue creado utilizando Python y C++ en ROS.

## Metodología

Se empleó la técnica de SLAM (Simultaneous Localization and Mapping). La localización y mapeo simultáneo consiste en construir un mapa de algún entorno desconocido mientras que se estima la posición actual del robot en este espacio. Se utiliza algún sensor, típicamente una cámara (RGB, escala gris, etc.) o un sensor Lidar, que, por medio de un algoritmo de extracción de características, retroalimenta estas características del entorno. El objetivo es ubicarse dentro del mapa reconociendo estas características del entorno que se vieron previamente (que, si se ve un objeto en una captura y en la siguiente captura se ha desplazado un poco, que se reconozca como el mismo objeto).

Para poder crear un robot que sea capaz de mapear el entorno, se utilizó un Kinect a bordo del carro diferencial y el paquete _rtabmap_ros_. Este es una envoltura ROS de RTAB-Map (mapeo basado en apariencia en tiempo real), el cual emplea la técnica de RGB-D SLAM basado en un detector de cierre de bucle global con restricciones en tiempo real. Este paquete se utilizó para generar nubes de puntos en 3D del entorno y para crear un mapa de cuadrícula de ocupación en 2D para la navegación. 

## Diseño

<!-- <img src="https://drive.google.com/uc?export=view&id=1yAcRZdWAT-r6jjumIKJg-wjArtCrONZk" /> -->
<p align="center">
  <img src="https://drive.google.com/uc?export=view&id=1yAcRZdWAT-r6jjumIKJg-wjArtCrONZk" width="159" height="455"/>
</p>

El robot cuenta con dos modos: el modo mapeo en donde se crea un mapa mediante la técnica SLAM y el modo navegación en donde se ubica dentro del mapa creado y busca llegar a la posición seleccionada por el usuario.

### Comunicacion con ROS

ROS nos proporciona una manera de conectar una red de procesos (o nodos), en donde estos se comunican entre sí mandando mensajes a través de topics. La mayoría de estos programas fueron escritos en Python, la única excepción siendo el código de Arduino. Se usó el paquete _rosserial_arduino_ para controlar el Arduino Mega con ROS por medio de comunicación serial.

### Modo mapeo

#### Simulación en Gazebo 

<img src="https://drive.google.com/uc?export=view&id=1hXCs5A3Ocyo7oBKXdv7mKZAAoW1rCDAl" width="640" height="360" />

Se utilizo Gazebo, un simulador de entornos 3D que posibilita evaluar el comportamiento de un robot en un mundo virtual. Además, es posible sincronizarlo con ROS de forma que los robots emulados publiquen la información de sus sensores en nodos, así como implementar una lógica y un control que dé ordenes al robot. Gracias a esta herramienta se pudieron probar los algoritmos de SLAM y el seguimiento de trayectoria sin tener que correr el riesgo de dañar el robot verdadero.

#### Cuadrícula de ocupación 

<img src="https://drive.google.com/uc?export=view&id=1PKjJjcd0-zNBVVt_AQ0egya5sH--7h6J" width="640" height="360" />

El mapa 2D está representado por una imagen, en donde cada pixel representa 5cm^2. Los píxeles blancos representan lugares en donde hay espacio libre, los negros lugares en donde sí se encuentran obstáculos y los píxeles grises lugares desconocidos o con incertidumbre.

#### Guardar puntos deseados

Al presionar uno de los botones en el bastón, el usuario puede guardar su posición actual para que después, en el modo localización, pueda regresar a este punto. También tiene la opción de guardar un mensaje de audio para nombrar el punto.

### Modo Localización

Con un mapa establecido el robot se ubica dentro de este con ayuda del paquete _rtabmap_ros_, el usuario selecciona una de las posiciones que guardo en el modo mapeo y se calcula la trayectoria optima de la posición actual a la posición deseada, el robot sigue esta trayectoria con ayuda de un controlador PID hasta llegar a la posición guardada en el botón, guiando al usuario invidente con ayuda de una base con relieve.

#### Obtención de trayectoria optima entre dos puntos utilizando A*
<img src="https://drive.google.com/uc?export=view&id=1rohEKkrAbZTBcTZgwCTLnDiQBos_DEYH" width="360" height="360" />

Para la obtención de la trayectoria optima entre dos puntos del mapa se utilizó el algoritmo de búsqueda A*, un algoritmo de búsqueda inteligente e informada que busca el camino más corto desde un estado inicial al estado meta a través de un espacio de problema, usando una heurística óptima. 

Esta trayectoria, que se publica como una lista de puntos deseados en ROS, lo recibe el nodo de control. Este se encarga de aplicar el controlador PID para llevar el robot a cada uno de estos puntos deseados, hasta llegar al punto final seleccionado.

## Informe Completo

https://drive.google.com/file/d/1QTnWZyZeMlTFSj-66E1XIOWEEc4xgThL/view?usp=sharing
