import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # Declarar el argumento para el modelo del robot
    model_arg = DeclareLaunchArgument(
        'model',
        default_value='rosbot_xl.urdf.xacro',
        description='URDF model to be loaded.'
    )

    # Nombre del paquete de descripción del robot
    package_description = 'rosbot_description'

    # Obtener la ruta al archivo XACRO de URDF
    robot_desc_path = PathJoinSubstitution([
        get_package_share_directory(package_description),
        'urdf',
        LaunchConfiguration('model')  # Aquí se usa LaunchConfiguration en lugar de LaunchDescription
    ])

    # Nodo robot_state_publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node',
        emulate_tty=True,
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # Configuración de RViz
    rviz_config_dir = os.path.join(
        get_package_share_directory(package_description),
        'rviz', 'urdf_vis.rviz'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        name='rviz_node',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', rviz_config_dir]
    )

    # Nodo joint_state_publisher
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Nodo joint_state_publisher_gui
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Devolver la descripción del lanzamiento con todos los nodos configurados
    return LaunchDescription(
        [
            model_arg,  # Agregar el argumento de modelo
            robot_state_publisher_node,
            joint_state_publisher_gui_node,
            joint_state_publisher_node,
            rviz_node
        ]
    )
