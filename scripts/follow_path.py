#!/usr/bin/env python

# ROS librerias
import rospy
import rospkg
import tf
# ROS mensajes
from geometry_msgs.msg import Quaternion, Pose2D
from nav_msgs.msg import Path
# Otros
import numpy as np
from playsound import playsound


# Distancia euclidania con respecto al origen
def norm_2(a):
    return np.sqrt(a[0]**2 + a[1]**2)


# Convertir quaternion a yaw
def quat_2_yaw(q):
    t1 = 2.0 * (q[3] * q[2] + q[0] * q[1])
    t2 = 1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2])
    return np.arctan2(t1, t2)


#Perceptrón para motor Derecho
def perceptronMD(x):
    #Estos valores fueron obtenidos en el entrenamiento del perceptrón
    xprom=7.0518; xmax=9.5319; xmin=0;
    yprom=172.6667; ymax=250; ymin=0;
    
    #pesos del perceptrón
    wO=np.array([[0.5014, -0.5377, 0.7847],[0.1066, 0.5997, -0.2507]])
    wS=np.array([0.5349, -1.3513, 0.4076, 0.7605])[:, np.newaxis] #vector columna
    
    xn=(x-xprom)/(xmax-xmin) #se normaliza el dato de entrada
    #Calculo de la salida de la capa oculta
    vO = wO.T @ np.array([xn, 1])[:, np.newaxis]
    yO=np.tanh(vO) #Función TANGENTE HIPERBÓLICA
    
    #Calculo de la salida de la capa de salida
    vS = wS.T @ np.concatenate((yO, np.array([[1]])))
    yn=float(np.copy(vS)) #Función LINEAL
    
    y=(ymax-ymin)*yn + yprom; #se desnormaliza la salida
    
    return y

#Perceptrón para motor Izquierdo
def perceptronMI(x):
    #Estos valores fueron obtenidos en el entrenamiento del perceptrón
    xprom=6.1449; xmax=8.8553; xmin=0;
    yprom=172.6667; ymax=250; ymin=0;
    
    #pesos del perceptrón
    wO=np.array([[-1.5761, -3.2384, 1.8847],[-0.6233, 0.7661, 1.1458]])
    wS=np.array([1.3594, -0.5455, 1.8497, -0.4440])[:, np.newaxis] #vector columna
    
    xn=(x-xprom)/(xmax-xmin) #se normaliza el dato de entrada
    #Calculo de la salida de la capa oculta
    vO = wO.T @ np.array([xn, 1])[:, np.newaxis]
    yO=np.tanh(vO) #Función TANGENTE HIPERBÓLICA
    
    #Calculo de la salida de la capa de salida
    vS = wS.T @ np.concatenate((yO, np.array([[1]])))
    yn=float(np.copy(vS)) #Función LINEAL
    
    y=(ymax-ymin)*yn + yprom; #se desnormaliza la salida
    
    return y


class PID:
    def __init__(self, path):
        # Parametros del carro
        self.L = 0.19 # Distancia del centro a cualquiera de las llantas (izquierda y derecha)
        self.D = 0.0441 # Diametro de las llantas

        # Posiciones
        self.pos = np.zeros(2) # Posicion [x, y]
        self.th = 0.0 # Orientacion en radianes
        self.pos_g = np.zeros(2)  # Posicion global deseada [x, y]
        self.threshold = 0.08  # Margen de error para posicion deseada (metros)

        # PID
        # self.kp = np.array([1.0, 2.5])  # Proporcional
        self.kp = np.array([1.42456, 7.00658])  # Proporcional

        # Parametros para el algoritmo
        self.k_att = 0.5  # Ganancia de atraccion
        self.h = 0.2  # Escalamiento de fuerzas
        self.d_max = 2  # Distancia maxima que detectan los sensores

        # Numero de nodos que se saltaran en cada iteracion
        self.skip = 4 

        self.it = 0 # Iterador para la trayectoria 
        self.path = [] # Trayectoria actual
        self.update_path(path) # Inicializar trayectoria

        self.listener = tf.TransformListener()
        self.pkg_path = rospkg.RosPack().get_path('lazarillo')

        # ROS
        self.odom_sub = rospy.Subscriber('/lazarillo/map_odom', Pose2D, self.algorithm)
        self.path_sub = rospy.Subscriber('/lazarillo/path', Path, self.update_path)
        self.vel_pub = rospy.Publisher('/lazarillo/llantas_vel', Quaternion, queue_size=10)

    def algorithm(self, data):
        # Actualizar posicion (odometria)
        self.pos = np.array([
            data.x, 
            data.y
        ])
        self.th = data.theta

        # Mensaje que sera publicado a ROS (0 por default)
        vel = Quaternion()

        # Error global
        e_g = self.pos_g - self.pos

        # Si no esta dentro del margen de error
        if norm_2(e_g) > self.threshold:
            # Fuerza de atraccion
            f = self.k_att * e_g

            # Normalizar fuerza neta
            f /= norm_2(f)

            # Calcular siguiente posicion deseada
            pos_d = self.pos + (self.h * f)

            # Error proporcional
            e = pos_d - self.pos
            ep = np.array([norm_2(e), np.arctan2(e[1], e[0]) - self.th])
            ep[1] = np.arctan2(np.sin(ep[1]), np.cos(ep[1]))

            # PID
            u = self.kp * ep

            # Actualizar mensaje para ROS
            wl = (2 * u[0] - self.L * u[1]) / self.D
            wr = (2 * u[0] + self.L * u[1]) / self.D

            # Convertir rad/s a pwm
            vel.x = perceptronMI(abs(wl)) 
            vel.y = perceptronMD(abs(wr)) 
            vel.z = wl >= 0
            vel.w = wr >= 0

            # Limitar pwm
            if vel.x > 255:
                vel.x = 255 
            if vel.y > 255:
                vel.y = 255

        # Ir al siguiente nodo (si existe)
        elif self.it < len(self.path) - 1:
            self.it += 1
            self.update_pos_g()

        # No existen mas nodos; se llego a la meta
        elif self.it == len(self.path) - 1:
            self.vel_pub.publish(vel)
            self.it += 1
            playsound(f'{self.pkg_path}/audio/exito.mp3')
            playsound(f'{self.pkg_path}/audio/exito_2.mp3')

        # Publicar velocidades
        self.vel_pub.publish(vel)

    def update_path(self, data):
        self.path = [] # Inicializar lista de nodos

        # Agregar nodos de la trayectoria recibida
        for i, pose in enumerate(data.poses):
            # Saltarse los primeros 6 nodos (30 cm)
            # despues saltarse cada 'skip' nodos (skip * 5 cm)
            if i >= 6 and i % self.skip == 0:
                point = pose.pose.position
                self.path.append(point)

        # Asegurar que el ultimo nodo se agrega
        self.path.append(data.poses[-1].pose.position)

        self.it = 0 # Resetear el iterador
        self.update_pos_g() # Actualizar meta local

    def update_pos_g(self):
        # Actualizar meta local
        self.pos_g = np.array([
            self.path[self.it].x, 
            self.path[self.it].y
        ])
        print(self.pos_g)


if __name__ == '__main__':
    rospy.init_node('pid')
    path = rospy.wait_for_message('/lazarillo/path', Path, timeout=None)
    pid = PID(path)
    try:
        rospy.spin()
    except Exception as e:
        print(e)
