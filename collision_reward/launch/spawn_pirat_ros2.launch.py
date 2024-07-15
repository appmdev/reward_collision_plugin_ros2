import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription,ExecuteProcess,RegisterEventHandler
from launch.event_handlers import OnProcessExit

from launch_ros.actions import Node
import xacro

# this is the function launch  system will look for


def generate_launch_description():

    ####### DATA INPUT ##########
    urdf_file = 'pirat.urdf'
    xacro_file = "pirat.urdf.xacro"
    #xacro_file = "box_bot.xacro"
    package_description = "collision_reward"
    use_urdf = False
    # Position and orientation
    # [X, Y, Z]
    position = [0.0, 0.0, 0.8]
    # [Roll, Pitch, Yaw]
    orientation = [0.0, 0.0, 0.0]
    # Base Name or robot
    namespace_declared= "pirat2"
    ####### DATA INPUT END ##########

    if use_urdf:
        # print("URDF URDF URDF URDF URDF URDF URDF URDF URDF URDF URDF ==>")
        robot_desc_path = os.path.join(get_package_share_directory(
            package_description), "robot", urdf_file)
    else:
        # print("XACRO XACRO XACRO XACRO XACRO XACRO XACRO XACRO XACRO XACRO XACRO ==>")
        robot_desc_path = os.path.join(get_package_share_directory(
            package_description), "urdf", xacro_file)

    robot_desc = xacro.process_file(robot_desc_path)
    xml = robot_desc.toxml()

    #entity_name = robot_base_name+"-"+str(random.random())
    entity_name = namespace_declared


    ######################

    # Remapping is required for state publisher otherwise /tf and /tf_static 
    # will get be published on root '/' namespace
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # Robot State Publisher
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    namespace = [ '/' + namespace_declared ]
    robot_state_publisher= Node(
        package='robot_state_publisher',
        namespace=namespace,
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': xml}],
        remappings=remappings,
        output="screen"
    )

    # Spawn ROBOT Set Gazebo (Does not spwan robot only communicates with the Gazebo Client)
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=['-entity',
                   entity_name,
                   '-x', str(position[0]), '-y', str(position[1]
                                                     ), '-z', str(position[2]),
                   '-R', str(orientation[0]), '-P', str(orientation[1]
                                                        ), '-Y', str(orientation[2]),
                   '-topic', '/' + namespace_declared + '/robot_description'
                   ]
    )

    # Publish Robot Desciption in String form in the topic /robot_description
    publish_robot_description = Node(
        package='collision_reward',
        executable='robot_description_publisher.py',
        name='robot_description_publisher',
        output='screen',
        arguments=['-xml_string', xml,
                   '-ROBOT_DESCRIPTION_TOPIC', '/' + namespace_declared + '/robot_description'
                   ]
    )

    # create and return launch description object
    return LaunchDescription(
        [   
            publish_robot_description,
            robot_state_publisher,
            spawn_robot
        ]
    )
