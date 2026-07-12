import os
import yaml
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

XACRO_PATH = os.path.join(
    get_package_share_directory("openarm_description"),
    "assets", "robot", "openarm_v1.0", "urdf", "openarm_v10.urdf.xacro"
)

LAUNCH_DELAY_SECONDS = 1.0


class NoAliasDumper(yaml.SafeDumper):
    def ignore_aliases(self, data):
        return True

# Order in which generic joint-gains keys map onto the 7 arm DOF.
ARM_JOINT_GAIN_KEYS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]


def load_joint_gains(gains_file_path: str) -> dict:
    """Read the generic per-joint gains YAML into {key: {"kp": ..., "kd": ...}}."""
    with open(gains_file_path, "r") as f:
        data = yaml.safe_load(f)
    return data


def build_gain_arrays(gains: dict) -> tuple[list, list]:
    """
    Extract kp/kd values from the generic gains dict, in ARM_JOINT_GAIN_KEYS order,
    producing the flat arrays the controller's k_joint_gains/d_joint_gains
    parameters expect.
    """
    k_gains, d_gains = [], []
    for key in ARM_JOINT_GAIN_KEYS:
        if key not in gains:
            raise KeyError(
                f"Joint gains file is missing required key '{key}'. "
                f"Expected keys: {ARM_JOINT_GAIN_KEYS}"
            )
        k_gains.append(float(gains[key]["kp"]))
        d_gains.append(float(gains[key]["kd"]))
    return k_gains, d_gains


def write_gains_overlay(context: LaunchContext, gains_file: str, tmp_dir: str) -> str:
    """
    Reads the generic gains YAML once, builds k_joint_gains/d_joint_gains
    (identical for both arms, since both arms share the same joint gain
    profile), and writes a single overlay YAML with a ros__parameters block
    for each impedance controller. Returns the path to that overlay file.
    """
    gains_file_resolved = context.perform_substitution(gains_file)
    gains = load_joint_gains(gains_file_resolved)
    k_gains, d_gains = build_gain_arrays(gains)

    overlay = {
        "left_impedance_controller": {
            "ros__parameters": {
                "k_joint_gains": list(k_gains),
                "d_joint_gains": list(d_gains),
            }
        },
        "right_impedance_controller": {
            "ros__parameters": {
                "k_joint_gains": list(k_gains),
                "d_joint_gains": list(d_gains),
            }
        },
    }

    overlay_path = os.path.join(tmp_dir, "generated_joint_gains_overlay.yaml")
    os.makedirs(tmp_dir, exist_ok=True)
    with open(overlay_path, "w") as f:
        yaml.dump(overlay, f, Dumper=NoAliasDumper)

    return overlay_path


def generate_robot_description(context: LaunchContext,
                                right_can_interface,
                                left_can_interface) -> str:
    return xacro.process_file(
        XACRO_PATH,
        mappings={
            "arm_type": "openarm_v1.0",
            "bimanual": "true",
            "use_fake_hardware": "false",
            "ros2_control": "true",
            "right_can_interface": context.perform_substitution(right_can_interface),
            "left_can_interface":  context.perform_substitution(left_can_interface),
        }
    ).toprettyxml(indent="  ")  # type: ignore[union-attr]


def launch_robot_nodes(context: LaunchContext,
                       right_can_interface,
                       left_can_interface,
                       controllers_file):
    robot_description = generate_robot_description(
        context, right_can_interface, left_can_interface)

    robot_description_param = {"robot_description": robot_description}
    controllers_file_str = context.perform_substitution(controllers_file)

    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description_param],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[robot_description_param, controllers_file_str],
    )

    return [robot_state_pub, control_node]


def spawner(arguments: list, param_file: str = None) -> Node:
    """Helper to create a controller spawner node, optionally with a param-file overlay."""
    full_args = list(arguments)
    if param_file is not None:
        full_args += ["--param-file", param_file]
    full_args += ["--controller-manager", "/controller_manager"]
    return Node(
        package="controller_manager",
        executable="spawner",
        arguments=full_args,
    )


def launch_impedance_spawner(context: LaunchContext, gains_file):
    """
    OpaqueFunction body: builds the gains overlay file at launch time (since
    it depends on reading + reshaping a YAML file, which can't be expressed
    as a pure launch substitution), then returns the impedance-controller
    spawner with that overlay attached.
    """
    tmp_dir = os.path.join(os.path.expanduser("~"), ".ros", "generated_params")
    overlay_path = write_gains_overlay(context, gains_file, tmp_dir)

    return [
        spawner(
            ["left_impedance_controller", "right_impedance_controller"],
            param_file=overlay_path,
        )
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "right_can_interface",
            default_value="can4",
            description="CAN interface for the right arm (e.g. can0).",
        ),
        DeclareLaunchArgument(
            "left_can_interface",
            default_value="can5",
            description="CAN interface for the left arm (e.g. can1).",
        ),
        DeclareLaunchArgument(
            "controllers_file",
            default_value="openarm_impedance_controllers.yaml",
            description="Controller config file name (looked up in openarm_impedance_control/config/).",
        ),
        DeclareLaunchArgument(
            "joint_gains_file",
            default_value="control_gains.yaml",
            description=(
                "Per-joint kp/kd gains file name, in the joint1..joint7/hand "
                "format. Looked up in "
                "openarm_description/assets/robot/openarm_v1.0/config/arm/ "
                "-- NOT this package's own config/ directory. Converted at "
                "launch time into k_joint_gains/d_joint_gains arrays for both "
                "arm controllers."
            ),
        ),
    ]

    right_can_interface = LaunchConfiguration("right_can_interface")
    left_can_interface  = LaunchConfiguration("left_can_interface")
    controllers_file    = PathJoinSubstitution([
        FindPackageShare("openarm_impedance_controller"),
        "config",
        LaunchConfiguration("controllers_file"),
    ])
    joint_gains_file    = PathJoinSubstitution([
        FindPackageShare("openarm_description"),
        "assets", "robot", "openarm_v1.0", "config", "arm",
        LaunchConfiguration("joint_gains_file"),
    ])

    robot_nodes = OpaqueFunction(
        function=launch_robot_nodes,
        args=[right_can_interface, left_can_interface, controllers_file],
    )

    impedance_spawner = OpaqueFunction(
        function=launch_impedance_spawner,
        args=[joint_gains_file],
    )

    rosbag_recorder = Node(
        package="rosbag_recorder",
        executable="recorder_node",
        name="rosbag_recorder",
        output="screen",
        parameters=[{
            "output_dir": os.path.expanduser("~/rosbags"),
            "storage_id": "mcap",
        }],
    )

    start_recording = TimerAction(
        period=LAUNCH_DELAY_SECONDS,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2", "service", "call",
                    "/rosbag_recorder/start",
                    "std_srvs/srv/Trigger", "{}",
                ],
                output="screen",
            ),
        ],
    )

    return LaunchDescription(declared_arguments + [
        robot_nodes,
        rosbag_recorder,
        start_recording,

        # Joint state broadcaster must come up before controllers read joint state
        TimerAction(period=LAUNCH_DELAY_SECONDS, actions=[
            spawner(["joint_state_broadcaster"]),
        ]),

        # Joint-space impedance controllers (one per arm)
        TimerAction(period=LAUNCH_DELAY_SECONDS, actions=[
            impedance_spawner,
        ]),

        # Gripper controllers
        TimerAction(period=LAUNCH_DELAY_SECONDS, actions=[
            spawner(["left_gripper_controller", "right_gripper_controller"]),
        ]),
    ])


