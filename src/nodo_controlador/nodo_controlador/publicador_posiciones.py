import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math

class PublicadorTrayectoria(Node):

    def __init__(self):
        super().__init__('nodo_publicador_trayectoria')
        
        # 1. Creamos el publicador
        # Publicará mensajes de tipo JointState en el topic '/joint_states'
        # El '10' es el tamaño de la cola (queue size)
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        
        # 2. Definimos la velocidad del bucle
        # 0.05 segundos = 20 Hz (20 veces por segundo)
        timer_period = 0.05 
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Variable para controlar el tiempo/movimiento
        self.t = 0.0
        
        self.get_logger().info('¡Nodo de control iniciado! El robot debería moverse ahora.')

    def timer_callback(self):
        # 3. Preparamos el mensaje
        msg = JointState()
        
        # Es buena práctica poner la marca de tiempo (timestamp)
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # --- NOMBRES DE LAS ARTICULACIONES ---
        # Estos DEBEN ser idénticos a los que pusiste en tu URDF
        msg.name = [
            'motor_1_joint', 
            'motor_2_joint', 
            'motor_3_joint', 
            'motor_4_joint', 
            'muneca_5_joint', 
            'dedo_izquierdo_joint', 
            'dedo_derecho_joint'
        ]
        
        # --- MOVIMIENTO---
        # Usamos funciones Seno y Coseno para crear movimientos suaves y oscilatorios
        self.t += 0.05
        
        # Base girando suavemente
        pos_base = 0.5 * math.sin(self.t * 0.5)
        
        # Brazo saludando
        pos_brazo = 0.3 * math.sin(self.t)
        
        # Pinza abriendo y cerrando (valor siempre positivo entre 0 y 0.03)
        pos_pinza = (math.sin(self.t * 2.0) + 1.0) / 2.0 * 0.03

        # --- ASIGNAR POSICIONES ---
        # El orden debe coincidir exactamente con la lista de nombres de arriba (msg.name)
        msg.position = [
            pos_base,    # motor_1
            pos_brazo,        # motor_2 (fijo en -0.5 radianes)
            pos_brazo,   # motor_3
            pos_brazo,   # motor_4
            pos_brazo,   # muneca_5
            pos_pinza,   # dedo_izq
            pos_pinza    # dedo_der
        ]
        
        # --- Publicador ---
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    nodo = PublicadorTrayectoria()
    
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    
    nodo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()