# Robot Móvil Diferencial para Asistencia a Invidentes (Lazarillo)

A continuación se demuestra el prototipo de un robot móvil con configuración diferencial con el propósito de ayudar a las personas que han adquirido recientemente alguna discapacidad visual. El robot tendrá abordó un Kinect que servirá para la localización, navegación y evasión de obstáculos dentro de un plano.

## Metodología

Se empleó la técnica de SLAM (Simultaneous Localization and Mapping). La localización y mapeo simultáneo consiste en construir un mapa de algún entorno desconocido mientras que se estima la posición actual del robot en este espacio. Se utiliza algún sensor, típicamente una cámara (RGB, escala gris, etc.) o un sensor Lidar, que, por medio de un algoritmo de extracción de características, retroalimenta estas características del entorno. El objetivo es ubicarse dentro del mapa reconociendo estas características del entorno que se vieron previamente (que, si se ve un objeto en una captura y en la siguiente captura se ha desplazado un poco, que se reconozca como el mismo objeto).

Para poder crear un robot que sea capaz de mapear el entorno, se utilizó un Kinect a bordo del carro diferencial y el paquete rtabmap_ros. Este es una envoltura ROS de RTAB-Map (mapeo basado en apariencia en tiempo real), el cual emplea la técnica de RGB-D SLAM basado en un detector de cierre de bucle global con restricciones en tiempo real. Este paquete se utilizó para generar nubes de puntos en 3D del entorno y para crear un mapa de cuadrícula de ocupación en 2D para la navegación. 

Para la obtención de la trayectoria optima entre dos puntos del mapa se utilizó el algoritmo de búsqueda A*, un algoritmo de búsqueda inteligente o informada que busca el camino más corto desde un estado inicial al estado meta a través de un espacio de problema, usando una heurística óptima. 

## Diseño

<img src="https://drive.google.com/uc?export=view&id=1_KlhjTA4BUrHfFv1fwdgCd_1iYuJbekb" width="640" height="360" />

El robot cuenta en un robot móvil diferencial que tiene dos modos, el modo mapeo donde se crea un mapa mediante la técnica SLAM y el modo navegación donde se ubica dentro del mapa creado y busca llegar a la posición seleccionada por el usuario.

### Simulación en Gazebo (modo mapeo)

<img src="https://drive.google.com/uc?export=view&id=1hXCs5A3Ocyo7oBKXdv7mKZAAoW1rCDAl" width="640" height="360" />

### Cuadrícula de ocupación (modo mapeo)

<img src="https://drive.google.com/uc?export=view&id=1PKjJjcd0-zNBVVt_AQ0egya5sH--7h6J" width="640" height="360" />

### Mapa 3D generado
<img src="https://drive.google.com/uc?export=view&id=1n0FqWCFH9X6FIUdiu72r_nKMq-_VlNPu" width="640" height="360" />

### Obtención de la trayectoria optima entre dos puntos utilizando A*
<img src="https://drive.google.com/uc?export=view&id=1rohEKkrAbZTBcTZgwCTLnDiQBos_DEYH" width="360" height="360" />
