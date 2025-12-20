import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def _read_pgm_size(pgm_path: str):
    """Read width/height from a PGM file (P2 or P5). Supports comments."""
    with open(pgm_path, 'rb') as f:
        magic = f.readline().strip()
        if magic not in [b'P2', b'P5']:
            raise RuntimeError(f"Unsupported PGM magic: {magic!r} in {pgm_path}")

        # Collect tokens (skip comments/blank lines) until we have: w h maxval
        tokens = []
        while len(tokens) < 3:
            line = f.readline()
            if not line:
                break
            line = line.strip()
            if not line or line.startswith(b'#'):
                continue
            # inline comments
            if b'#' in line:
                line = line.split(b'#', 1)[0].strip()
                if not line:
                    continue
            tokens.extend(line.split())

        if len(tokens) < 3:
            raise RuntimeError(f"Failed to parse PGM header (w h maxval) from {pgm_path}")

        w = int(tokens[0])
        h = int(tokens[1])
        return w, h


def _compute_map_center(map_yaml_path: str):
    """
    Compute map center in 'map' frame using map yaml + pgm dimensions.
    center_x = origin_x + (width * resolution)/2
    center_y = origin_y + (height * resolution)/2
    """
    with open(map_yaml_path, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f)

    image_rel = data.get('image')
    resolution = float(data.get('resolution'))
    origin = data.get('origin')
    if origin is None or len(origin) < 2:
        raise RuntimeError(f"Invalid origin in map yaml: {origin}")

    origin_x = float(origin[0])
    origin_y = float(origin[1])

    yaml_dir = os.path.dirname(map_yaml_path)
    image_path = image_rel
    if not os.path.isabs(image_path):
        image_path = os.path.join(yaml_dir, image_rel)
    image_path = os.path.normpath(image_path)

    w, h = _read_pgm_size(image_path)

    center_x = origin_x + (w * resolution) / 2.0
    center_y = origin_y + (h * resolution) / 2.0
    return center_x, center_y


def _add_dynamic_static_tf(context, *args, **kwargs):
    """
    Publish a static TF map->odom for "no_localization" mode.

    To place the robot near the map center at start (odom at 0,0),
    set: map->odom = (-center_x, -center_y).
    """
    map_yaml_path = LaunchConfiguration('map').perform(context)

    center_x, center_y = 0.0, 0.0
    try:
        center_x, center_y = _compute_map_center(map_yaml_path)
    except Exception as e:
        print(f"[bringup.launch.py] WARN: failed to compute map center from {map_yaml_path}: {e}")
        print("[bringup.launch.py] WARN: fallback center = (0.0,0.0)")

    tx = -float(center_x)
    ty = -float(center_y)

    print(f"[bringup.launch.py] map->odom static TF = ({tx:.3f}, {ty:.3f}, 0) yaw=0")

    static_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom_center',
        arguments=[f'{tx}', f'{ty}', '0', '0', '0', '0', 'map', 'odom'],
        output='screen'
    )
    return [static_map_odom]


def generate_launch_description():
    nav2_bringup_share = FindPackageShare('nav2_bringup')
    cleanrobot_nav_share = FindPackageShare('cleanrobot_nav')

    # ===== Launch arguments =====
    workspace_map_default = '/home/cat/文档/cleanrobot/cleanrobot/src/cleanrobot_nav/maps/cleanrobot_map.yaml'
    nav2_params_default = '/home/cat/文档/cleanrobot/cleanrobot/src/cleanrobot_nav/config/nav2_params_real.yaml'

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=workspace_map_default,
        description='Full path to the map yaml file'
    )

    nav2_params_arg = DeclareLaunchArgument(
        'nav2_params',
        default_value=nav2_params_default,
        description='Nav2 params yaml file (real or sim)'
    )

    use_fake_base_arg = DeclareLaunchArgument(
        'use_fake_base',
        default_value='false',
        description='Whether to start fake_base (Nav2 cmd_vel -> odom)'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Whether to start RViz'
    )

    use_localization_arg = DeclareLaunchArgument(
        'use_localization',
        default_value='true',
        description='Whether to use AMCL localization (map->odom). If false, use static map->odom.'
    )

    # ✅ 新增：是否启动 map_server（离线地图 / no_localization 模式必须要）
    use_map_server_arg = DeclareLaunchArgument(
        'use_map_server',
        default_value='true',
        description='Whether to start nav2_map_server in no_localization mode'
    )

    params_file = LaunchConfiguration('nav2_params')

    # --- 条件：use_localization == false/true ---
    no_localization_cond = IfCondition(
        PythonExpression(["'", LaunchConfiguration('use_localization'), "' == 'false'"])
    )

    localization_cond = IfCondition(
        PythonExpression(["'", LaunchConfiguration('use_localization'), "' == 'true'"])
    )

    # --- 条件：use_map_server == true 且 use_localization == false ---
    map_server_needed_cond = IfCondition(
        PythonExpression([
            "'", LaunchConfiguration('use_localization'), "' == 'false' and '",
            LaunchConfiguration('use_map_server'), "' == 'true'"
        ])
    )

    # --- 条件：use_fake_base == true AND use_localization == false ---
    fakebase_no_loc_cond = IfCondition(
        PythonExpression([
            "'", LaunchConfiguration('use_fake_base'), "' == 'true' and '",
            LaunchConfiguration('use_localization'), "' == 'false'"
        ])
    )

    # 模式1：AMCL（map_server + amcl + navigation）——交给 nav2 bringup
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([nav2_bringup_share, 'launch', 'bringup_launch.py'])
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'params_file': params_file,
            'use_sim_time': 'false',
            'autostart': 'true'
        }.items(),
        condition=localization_cond
    )

    # 模式2：不启用 AMCL（只启动 navigation）
    nav2_navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([nav2_bringup_share, 'launch', 'navigation_launch.py'])
        ),
        launch_arguments={
            'params_file': params_file,
            'use_sim_time': 'false',
            'autostart': 'true'
        }.items(),
        condition=no_localization_cond
    )

    # ✅ no_localization 模式下：可选启动 map_server + lifecycle_manager（发布 /map）
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': LaunchConfiguration('map'),
            'use_sim_time': False
        }],
        condition=map_server_needed_cond
    )

    lifecycle_manager_map = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['map_server']
        }],
        condition=map_server_needed_cond
    )

    # ===== Coverage waypoints node =====
    coverage_node = Node(
        package='cleanrobot_nav',
        executable='coverage_waypoints.py',
        name='coverage_waypoints',
        output='screen',
        parameters=[{
            'step_m': 0.25,
            'keepout_cells': 1
        }]
    )

    # ===== Fake robot (optional) =====
    fake_urdf_path = os.path.join(
        get_package_share_directory('cleanrobot_nav'),
        'config',
        'fake_robot.urdf'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': open(fake_urdf_path, 'r').read()
        }],
        condition=IfCondition(LaunchConfiguration('use_fake_base'))
    )

    fake_base_node = Node(
        package='cleanrobot_nav',
        executable='fake_base.py',
        name='fake_base_sim',
        output='screen',
        parameters=[{
            'initial_x': 0.0,
            'initial_y': 0.0,
            'initial_yaw': 0.0,
            'use_sim_time': False
        }],
        remappings=[
            ('/cmd_vel', '/cmd_vel_nav'),
        ],
        condition=IfCondition(LaunchConfiguration('use_fake_base'))
    )

    # ✅ fake_base + 不用AMCL：发布 map->odom（把机器人放回地图中心）
    static_map_odom_center = OpaqueFunction(
        function=_add_dynamic_static_tf,
        condition=fakebase_no_loc_cond
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=[
            '-d',
            PathJoinSubstitution([cleanrobot_nav_share, 'rviz', 'bringup_fake.rviz'])
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    return LaunchDescription([
        map_arg,
        nav2_params_arg,
        use_fake_base_arg,
        use_rviz_arg,
        use_localization_arg,
        use_map_server_arg,

        nav2_bringup_launch,
        nav2_navigation_launch,

        map_server_node,
        lifecycle_manager_map,

        coverage_node,

        robot_state_publisher,
        fake_base_node,

        static_map_odom_center,
        rviz_node,
    ])
