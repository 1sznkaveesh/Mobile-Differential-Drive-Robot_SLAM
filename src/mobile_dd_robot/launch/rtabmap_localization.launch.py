#!/usr/bin/env python3
"""
RTABMap LOCALIZATION Launch File with User Parameters
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    robotXacroName = 'differential_drive_robot'
    namePackage = 'mobile_dd_robot'
    modelFileRelativePath = 'model/robot_with_camera.xacro'
    # worldFileRelativePath = 'model/3d_slam_world.world'  # Using TurtleBot3 world instead

    pathModelFile = os.path.join(get_package_share_directory(namePackage), modelFileRelativePath)
    pathWorldFile = f"/opt/ros/{os.environ['ROS_DISTRO']}/share/turtlebot3_gazebo/worlds/turtlebot3_house.world"

    robotDescription = xacro.process_file(pathModelFile).toxml()

    # Gazebo launch
    gazebo_rosPackageLaunch = PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
    )
    gazeboLaunch = IncludeLaunchDescription(
        gazebo_rosPackageLaunch, 
        launch_arguments={'world': pathWorldFile}.items()
    )

    # Spawn robot
    spawnModelNode = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', robotXacroName],
        output='screen'
    )
    
    # Robot state publisher
    nodeRobotStatePublisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robotDescription, 'use_sim_time': True}]
    )
    
    # Launch arguments for RTABMap
    use_sim_time = LaunchConfiguration('use_sim_time')
    qos = LaunchConfiguration('qos')
    database_path = LaunchConfiguration('database_path')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock')
    
    declare_qos_cmd = DeclareLaunchArgument(
        'qos',
        default_value='2',
        description='QoS for sensor topics')
    
    declare_database_path_cmd = DeclareLaunchArgument(
        'database_path',
        default_value='~/.ros/rtabmap.db',
        description='Path to existing RTABMap database (must exist!)')
    
    # RTABMap parameters - LOCALIZATION MODE
    rtabmap_parameters = [{
        'frame_id': 'body_link',
        'use_sim_time': use_sim_time,
        'subscribe_depth': True,
        'subscribe_rgb': True,
        'subscribe_scan': False,
        'subscribe_odom_info': False,
        'use_action_for_goal': True,
        'qos_image': qos,
        'qos_imu': qos,
        'qos_odom': qos,
        'database_path': database_path,
        
        # LOCALIZATION MODE
        'Mem/IncrementalMemory': 'false',  
        'Mem/InitWMWithAllNodes': 'true', 
        
        # ODOMETRY FUSION
        'Odom/Strategy': '0',
        'Odom/GuessMotion': 'true',
        'Odom/ResetCountdown': '5',
        'Odom/FilteringStrategy': '1',
        
        # REGISTRATION
        'Reg/Force3DoF': 'true',
        'Reg/Strategy': '0',
        'Reg/RepeatOnce': 'true',
        
        # MEMORY MANAGEMENT
        'Mem/STMSize': '30',
        'Mem/ImagePreDecimation': '2',
        'Mem/ImagePostDecimation': '4',    # UPDATED: 1 → 4
        'Mem/ReduceGraph': 'false',
        'Mem/NotLinkedNodesKept': 'true',
        'Mem/IntermediateNodeDataKept': 'true',
        
        # RGBD PROCESSING
        'RGBD/CreateOccupancyGrid': 'true',
        'RGBD/NeighborLinkRefining': 'true',
        'RGBD/ProximityBySpace': 'true',
        'RGBD/ProximityByTime': 'false',
        'RGBD/AngularUpdate': '0.05',
        'RGBD/LinearUpdate': '0.05',
        'RGBD/OptimizeFromGraphEnd': 'false',
        'RGBD/OptimizeMaxError': '3.0',
        'RGBD/ProximityPathMaxNeighbors': '10',
        'RGBD/LoopClosureReextractFeatures': 'true',
         
        # VISUAL FEATURES
        'Vis/MaxFeatures': '1000',
        'Vis/MinInliers': '20',
        'Vis/InlierDistance': '0.1',
        'Vis/FeatureType': '6',
        'Vis/MaxDepth': '5.0',
        'Vis/MinDepth': '0.2',
        'Vis/CorGuessWinSize': '40',
        'Vis/EstimationType': '1',
        'Vis/ForwardEstOnly': 'false',
        
        # KEYPOINT DETECTION
        'Kp/MaxFeatures': '500',
        'Kp/DetectorStrategy': '6',
        'Kp/RoiRatios': '0.0 0.0 0.0 0.0',
        'Kp/MaxDepth': '5.0',
        'Kp/MinDepth': '0.2',
        'Kp/IncrementalFlann': 'true',
        'Kp/IncrementalDictionary': 'false',  
        'Kp/BadSignRatio': '0.5',
        
        # LOOP CLOSURE
        'Rtabmap/DetectionRate': '2.0',
        'Rtabmap/TimeThr': '0',
        'Rtabmap/LoopThr': '0.11',
        'Rtabmap/LoopRatio': '0.9',
        
        # RELOCALIZATION SETTINGS
        'Rtabmap/StartNewMapOnLoopClosure': 'false',
        'Rtabmap/StartNewMapOnGoodSignature': 'false',
        'Mem/RehearsalSimilarity': '0.6',
        'Mem/RecentWmRatio': '0.2',
        'RGBD/LocalRadius': '10',
        'RGBD/LocalImmunizationRatio': '0.25',
        
        # Grid/Map Settings - USER PARAMETERS
        'Grid/FromDepth': 'true',
        'Grid/Sensor': '1',
        'Grid/CellSize': '0.05',           # UPDATED: 0.05 → 0.02
        'Grid/RangeMax': '4',              # UPDATED: 5.0 → 4
        'Grid/RangeMin': '0.15',            # UPDATED: 0.15 → 0.3
        'Grid/MaxObstacleHeight': '2.0',
        'Grid/3D': 'true',
        'Grid/ClusterRadius': '0.15',
        'Grid/MinClusterSize': '3',       # UPDATED: 3 → 10
        'Grid/NormalSegmentation': 'false',  # ADDED
        'OctoMap/VoxelSize': '0.05',
        'OctoMap/OccupancyThreshold': '0.3',
        'Grid/GroundIsObstacle': 'false',
        'Grid/NoiseFilteringRadius': '0.12',
        'Grid/NoiseFilteringMinNeighbors': '6',
        
        # Ground Filtering & Refinements
        'Grid/MinGroundHeight': '-0.1',        # Filter points below -10cm (removes underground artifacts)
        'Grid/MaxGroundHeight': '0.05',        # Ground detection threshold (5cm above base)
        'Grid/NormalsSegmentation': 'false',   # Disable for faster processing
        'Grid/FootprintLength': '0.0',         # Robot footprint (0 = auto)
        'Grid/FootprintWidth': '0.0',          # Robot footprint (0 = auto)
        'Grid/FootprintHeight': '0.0',         # Robot footprint height
        
        # Enhanced Noise Filtering
        'Grid/DepthDecimation': '1',           # No decimation for depth
        'Grid/FlatObstacleDetected': 'true',   # Detect flat obstacles
        'Grid/RayTracing': 'true',
        'Grid/MapFrameProjection': 'true',
        
        # Graph Optimization
        'RGBD/OptimizeStrategy': '1',
        'Optimizer/Strategy': '1',
        'Optimizer/Iterations': '30',
        'Optimizer/Epsilon': '0.00001',
        'Optimizer/Robust': 'true',
    }]
    
    # Topic remappings
    rtabmap_remappings = [
        ('rgb/image', '/camera/d455/image_raw'),
        ('rgb/camera_info', '/camera/d455/camera_info'),
        ('depth/image', '/camera/d455/depth/image_raw'),
        ('odom', '/odom'),
    ]
    
    # RTABMap SLAM node in localization mode
    rtabmap_slam = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        output='screen',
        parameters=rtabmap_parameters,
        remappings=rtabmap_remappings,
        arguments=['--delete_db_on_start', 'false'],
    )
    
    # RTABMap visualization
    rtabmap_viz = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        output='screen',
        parameters=rtabmap_parameters,
        remappings=rtabmap_remappings,
    )
    
    # Build launch description
    launchDescriptionObject = LaunchDescription()
    
    # Environment
    launchDescriptionObject.add_action(SetEnvironmentVariable('RTABMAP_SYNC_MULTI_RGBD', '0'))
    
    # Arguments
    launchDescriptionObject.add_action(declare_use_sim_time_cmd)
    launchDescriptionObject.add_action(declare_qos_cmd)
    launchDescriptionObject.add_action(declare_database_path_cmd)
    
    # Gazebo and robot
    launchDescriptionObject.add_action(gazeboLaunch)
    launchDescriptionObject.add_action(nodeRobotStatePublisher)
    launchDescriptionObject.add_action(spawnModelNode)
    
    # RTABMap
    launchDescriptionObject.add_action(rtabmap_slam)
    launchDescriptionObject.add_action(rtabmap_viz)

    return launchDescriptionObject
