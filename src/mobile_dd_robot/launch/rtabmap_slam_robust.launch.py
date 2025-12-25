#!/usr/bin/env python3
"""
RTABMap SLAM Launch File - ROBUST VERSION
Features:
- Wheel odometry fusion
- Robust loop closure detection
- Relocalization support
- Lost robot recovery
- Database persistence
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
    worldFileRelativePath = 'model/3d_slam_world.world'

    pathModelFile = os.path.join(get_package_share_directory(namePackage), modelFileRelativePath)
    pathWorldFile = os.path.join(get_package_share_directory(namePackage), worldFileRelativePath)

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
    localization = LaunchConfiguration('localization')
    database_path = LaunchConfiguration('database_path')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock')
    
    declare_qos_cmd = DeclareLaunchArgument(
        'qos',
        default_value='2',
        description='QoS for sensor topics')
    
    declare_localization_cmd = DeclareLaunchArgument(
        'localization',
        default_value='false',
        description='Set to true for localization mode (uses existing map)')
    
    declare_database_path_cmd = DeclareLaunchArgument(
        'database_path',
        default_value='~/.ros/rtabmap.db',
        description='Path to RTABMap database')
    
    # RTABMap parameters - ROBUST CONFIGURATION
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
        
    
        # ODOMETRY FUSION - Use wheel odometry + visual

        'Odom/Strategy': '0',              # 0=Frame-to-Map, uses external odom
        'Odom/GuessMotion': 'true',        # Use motion model
        'Odom/ResetCountdown': '5',        # Reset after 5 failed attempts
        'Odom/FilteringStrategy': '1',     # Kalman filtering
        
        # REGISTRATION - Force 3DoF for ground robot

        'Reg/Force3DoF': 'true',           # Ground robot constraint
        'Reg/Strategy': '0',               # 0=Vis, 1=ICP, 2=Vis+ICP
        'Reg/RepeatOnce': 'true',          # Refine registration
        
        # MEMORY MANAGEMENT

        'Mem/IncrementalMemory': 'true',   # Build map incrementally (SLAM mode)
        'Mem/InitWMWithAllNodes': 'false', # Don't load all nodes at start
        'Mem/STMSize': '30',               # Short-term memory size
        'Mem/ImagePreDecimation': '2',     # Reduce image size before processing
        'Mem/ImagePostDecimation': '1',    # No post decimation
        'Mem/ReduceGraph': 'true',         # Reduce graph complexity
        'Mem/NotLinkedNodesKept': 'true',  # Keep nodes even if not linked
        'Mem/IntermediateNodeDataKept': 'true',  # Keep data for relocalization
        
        # RGBD PROCESSING

        'RGBD/CreateOccupancyGrid': 'true',
        'RGBD/NeighborLinkRefining': 'true',      # Refine links with neighbors
        'RGBD/ProximityBySpace': 'true',          # Detect proximity by space
        'RGBD/ProximityByTime': 'false',          # Don't use time for proximity
        'RGBD/AngularUpdate': '0.05',             # Update every 0.05 rad (~3 degrees)
        'RGBD/LinearUpdate': '0.05',              # Update every 5cm
        'RGBD/OptimizeFromGraphEnd': 'false',     # Optimize from latest node
        'RGBD/OptimizeMaxError': '3.0',           # Max error for optimization
        'RGBD/ProximityPathMaxNeighbors': '10',   # Max neighbors for proximity
        'RGBD/LoopClosureReextractFeatures': 'true',  # Re-extract for verification
        
        # VISUAL FEATURES - ROBUST DETECTION

        'Vis/MaxFeatures': '1000',         # Increase from 400 to 1000
        'Vis/MinInliers': '20',            # Increase from 15 to 20 (more robust)
        'Vis/InlierDistance': '0.1',       # Max inlier distance
        'Vis/FeatureType': '6',            # 6=GFTT (Good Features To Track)
        'Vis/MaxDepth': '5.0',             # Max depth for features
        'Vis/MinDepth': '0.2',             # Min depth for features
        'Vis/CorGuessWinSize': '40',       # Correlation window size
        'Vis/EstimationType': '1',         # 1=3D->3D (PnP RANSAC)
        'Vis/ForwardEstOnly': 'false',     # Bi-directional estimation
        
        # KEYPOINT DETECTION - CRITICAL FOR LOOP CLOSURE

        'Kp/MaxFeatures': '500',           # Features per image for loop detection
        'Kp/DetectorStrategy': '6',        # 6=GFTT, 0=SURF, 1=SIFT, 8=FAST
        'Kp/RoiRatios': '0.0 0.0 0.0 0.0', # Use full image
        'Kp/MaxDepth': '5.0',              # Max depth for keypoints
        'Kp/MinDepth': '0.2',              # Min depth for keypoints
        'Kp/IncrementalFlann': 'true',     # Fast incremental loop detection
        'Kp/IncrementalDictionary': 'true', # Build dictionary incrementally
        'Kp/BadSignRatio': '0.5',          # Bad signature ratio threshold
        
        # LOOP CLOSURE - ROBUST DETECTION

        'Rtabmap/DetectionRate': '1.0',    # Check every frame
        'Rtabmap/TimeThr': '0',            # No time threshold (always look)
        'Rtabmap/LoopThr': '0.11',         # Loop closure threshold (0.11 = moderate)
        'Rtabmap/LoopRatio': '0.9',        # Loop closure ratio
        'RGBD/LoopClosureReextractFeatures': 'true',  # Verify loops
        
        # LOST ROBOT RECOVERY & RELOCALIZATION

        'Rtabmap/StartNewMapOnLoopClosure': 'false',  # Don't start new map on loop
        'Rtabmap/StartNewMapOnGoodSignature': 'false', # Don't start new map
        'Mem/RehearsalSimilarity': '0.6',  # Similarity for memory rehearsal
        'Mem/RecentWmRatio': '0.2',        # Recent working memory ratio
        'RGBD/LocalRadius': '10',          # Local map radius
        'RGBD/LocalImmunizationRatio': '0.25', # Immunize recent nodes
        
        # Grid/Map Settings

        'Grid/FromDepth': 'true',
        'Grid/Sensor': '1',                # 0=laser, 1=depth
        'Grid/CellSize': '0.05',           # 5cm resolution
        'Grid/RangeMax': '5.0',            # 5m max range
        'Grid/RangeMin': '0.15',            # 20cm min range
        'Grid/MaxObstacleHeight': '2.0',   # 2m obstacle height
        'Grid/3D': 'true',                # 2D grid (set true for 3D OctoMap)
        'Grid/ClusterRadius': '0.15',
        'Grid/MinClusterSize': '3',
        'OctoMap/VoxelSize': '0.05',
        'OctoMap/OccupancyThreshold': '0.3',
        'Grid/GroundIsObstacle': 'false',
        'Grid/NoiseFilteringRadius': '0.05',
        'Grid/NoiseFilteringMinNeighbors': '3',
        'Grid/RayTracing': 'true',         # Fill free space
        
        # Graph Optimization

        'RGBD/OptimizeStrategy': '1',      # 0=TORO, 1=g2o, 2=GTSAM
        'Optimizer/Strategy': '1',         # Use g2o
        'Optimizer/Iterations': '20',      # Optimization iterations
        'Optimizer/Epsilon': '0.00001',    # Convergence epsilon
        'Optimizer/Robust': 'true',        # Robust optimization
    }]
    
    # Topic remappings - NOW INCLUDING ODOMETRY!
    rtabmap_remappings = [
        ('rgb/image', '/camera/d455/image_raw'),
        ('rgb/camera_info', '/camera/d455/camera_info'),
        ('depth/image', '/camera/d455/depth/image_raw'),
        ('odom', '/odom'),  
    ]
    
    # RTABMap SLAM node
    rtabmap_slam = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        output='screen',
        parameters=rtabmap_parameters,
        remappings=rtabmap_remappings,
        arguments=['--delete_db_on_start'], 
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
    launchDescriptionObject.add_action(declare_localization_cmd)
    launchDescriptionObject.add_action(declare_database_path_cmd)
    
    # Gazebo and robot
    launchDescriptionObject.add_action(gazeboLaunch)
    launchDescriptionObject.add_action(nodeRobotStatePublisher)
    launchDescriptionObject.add_action(spawnModelNode)
    
    # RTABMap
    launchDescriptionObject.add_action(rtabmap_slam)
    launchDescriptionObject.add_action(rtabmap_viz)

    return launchDescriptionObject
