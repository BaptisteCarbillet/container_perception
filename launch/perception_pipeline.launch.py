from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Livox Cropbox Node
        Node(
            package='livox_cropbox',
            executable='livox_cropbox_node',
            name='livox_cropbox_node',
            parameters=[
                {'x_length': 1.2},
                {'y_length': 1.2},
            ],
            output='screen',
        ),
        
        # Plane Detector Node
        Node(
            package='container_perception',
            executable='plane_detector_node',
            name='plane_detector_node',
            parameters=[
                {'publish_plane': True},
                {'n_points_threshold': 11000},
                {'n_neighbors': 5},
                {'radius_search': 0.1},
            ],
            output='screen',
        ),
        
        # Container Tracker Node
        Node(
            package='container_perception',
            executable='container_tracker_node',
            name='container_tracker_node',
            output='screen',
        ),
        
        #rs_cloud_saver Node
        Node(
            package='rs_cloud',
            executable='cloud_saver_service_node',
            name='rs_cloud_saver_node',
            parameters=[
                {'crop_dx': 3.1},
                {'crop_dy': 3.1},
                {'crop_dz': 3.0},
            ],
            output='screen',
        ),


        #leg localizer Node
        Node(
            package='container_perception',
            executable='leg_localizer_node',
            name='leg_localizer_node',
            parameters=[
                {'delta_crop': 0.1},
                {'cluster_tolerance': 0.2},
                {'min_cluster_size': 5},
                {'max_cluster_size': 500},
            ],
            output='screen',
        ),

        Node(
            package='container_perception',
            executable='leg_inspector_node',
            name='leg_inspector_node',
            parameters=[
                {'views_per_leg': 12},
                {'r_min': 2.0},
                {'r_max': 4.0},
                {'d_theta_deg': 30.0},
                {'min_angle_sep_deg': 15.0},
            ],
            output='screen',
        ),

        Node(
            package='coverage_roi',
            executable='coverage_node',
            name='coverage_node',
            output='screen',
        ),
    ])
