import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

"""
Este archivo de lanzamiento configura y lanza varios nodos necesarios para visualizar un robot en RViz2 utilizando un archivo URDF.
Incluye nodos para publicar los estados de las articulaciones del robot y para publicar las transformaciones del robot (tf).

En caso de que se desee utilizar una interfaz gráfica para controlar los estados de las articulaciones, se puede descomentar el nodo correspondiente.
(joint_state_publisher_gui_node), y comentar el nodo sin GUI (joint_state_publisher_node).
"""

def generate_launch_description():

    # Obtener la ruta del archivo URDF
    pkg_path = get_package_share_directory('mi_robot_descripcion')
    urdf_file_path = os.path.join(pkg_path, 'urdf', 'mi_robot.urdf')

    # Cargar el contenido del archivo URDF
    robot_description_config = xacro.process_file(urdf_file_path)
    robot_description = {'robot_description': robot_description_config.toxml()}

    """
    # Nodo para publicar los estados de las articulaciones con GUI (interfaz gráfica)
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
    )
    """
    # Nodo para publicar los estados de las articulaciones sin GUI
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
    )  

    
    # Nodo para publicar las transformaciones del robot (tf)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # Nodo para RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
    )

    return LaunchDescription([
        #joint_state_publisher_gui_node, # Nodo con GUI
        joint_state_publisher_node, # Nodo sin GUI
        robot_state_publisher_node, # Nodo para publicar las transformaciones del robot (tf)
        rviz_node, # Nodo para RViz2
    ])