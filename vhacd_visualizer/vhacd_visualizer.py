import rclpy
from rclpy.node import Node

import os
import glob
import subprocess
import yaml

from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import Pose, Quaternion, Vector3
from visualization_msgs.msg import MarkerArray, Marker
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult, ParameterType
from rcl_interfaces.msg import IntegerRange, FloatingPointRange

from std_msgs.msg import Header, ColorRGBA
from std_srvs.srv import Trigger
from rclpy.parameter_event_handler import ParameterEventHandler


def make_mesh_marker(mesh_resource, ns="input_mesh", alpha=1.0):
    return Marker(
        ns=ns,
        header=Header(frame_id="base_link"),
        type=Marker.MESH_RESOURCE,
        pose=Pose(orientation=Quaternion(w=1.0)),
        scale=Vector3(x=1.0, y=1.0, z=1.0),
        color=ColorRGBA(r=1.0, g=1.0, b=1.0, a=alpha),
        mesh_resource=mesh_resource,
        mesh_use_embedded_materials=True,
    )


STARTUP_PARAMETERS = (
    ("vhacd_executable_path", "TestVHACD", ParameterType.PARAMETER_STRING),
    ("mesh_filename", "", ParameterType.PARAMETER_STRING),
)


# Essentially a poor mans https://github.com/PickNikRobotics/generate_parameter_library
def declare_vhacd_parameter(node, param_dict):
    param_type = {
        "bool": ParameterType.PARAMETER_BOOL,
        "int": ParameterType.PARAMETER_INTEGER,
        "double": ParameterType.PARAMETER_DOUBLE,
        "string": ParameterType.PARAMETER_STRING,
    }[param_dict["type"]]

    param_descriptor = ParameterDescriptor(type=param_type, description=param_dict["description"])
    if param_type == ParameterType.PARAMETER_INTEGER and "integer_range" in param_dict:
        small, large, step = param_dict["integer_range"]
        integer_range = IntegerRange(from_value=small, to_value=large, step=step)
        param_descriptor.integer_range.append(integer_range)
    if param_type == ParameterType.PARAMETER_DOUBLE and "floating_point_range" in param_dict:
        small, large, step = param_dict["floating_point_range"]
        floating_point_range = FloatingPointRange(from_value=small, to_value=large, step=step)
        param_descriptor.floating_point_range.append(floating_point_range)

    node.declare_parameter(param_dict["name"], param_dict["default"], param_descriptor)


class InteractiveVHACDPublisherNode(Node):
    def __init__(self):
        super().__init__("vhacd_visualizer")

        for name, default_value, param_type in STARTUP_PARAMETERS:
            self.declare_parameter(name, default_value, ParameterDescriptor(type=param_type))

        pkg_dir = get_package_share_directory("vhacd_visualizer")
        with open(os.path.join(pkg_dir, "config/vhacd_parameter_descriptions.yaml")) as f:
            self.vhacd_flag_map = {}
            for flag, vhacd_param_dict in yaml.safe_load(f).items():
                declare_vhacd_parameter(self, vhacd_param_dict)
                self.vhacd_flag_map[flag] = vhacd_param_dict["name"]

        self.parameter_event_handler = ParameterEventHandler(self)
        self.parameter_event_handler.add_parameter_event_callback(self.parameters_callback)

        _, mesh_filename = self.validate_startup_parameters()

        self.mesh_marker = make_mesh_marker(mesh_resource=f"file://{mesh_filename}")
        self.convex_hull_marker = None
        self.generation_idx = 0

        self.publisher = self.create_publisher(MarkerArray, "vhacd_markers", 10)
        self.timer = self.create_timer(2.0, self.publish_marker_array)
        self.srv = self.create_service(Trigger, "run_vhacd", self.run_vhacd_cb)

        self.run_vhacd_cb(None, Trigger.Response())

    def parameters_callback(self, param_event):
        if param_event.changed_parameters:
            self.run_vhacd_cb(None, Trigger.Response())

        return SetParametersResult(successful=True)

    def publish_marker_array(self):
        array = MarkerArray()
        if self.mesh_marker is not None:
            array.markers.append(self.mesh_marker)
        if self.convex_hull_marker is not None:
            array.markers.append(self.convex_hull_marker)

        if not array.markers:
            array.markers.append(Marker(action=Marker.DELETEALL))

        self.publisher.publish(array)

    def validate_startup_parameters(self):
        vhacd_exec_path = self.get_parameter("vhacd_executable_path").value

        if not os.path.isabs(vhacd_exec_path):
            try:
                commands = ["which", vhacd_exec_path]
                vhacd_exec_path = subprocess.check_output(commands, text=True).strip()
            except subprocess.CalledProcessError as e:
                raise RuntimeError(f"VHACD not found: {e}")

        if not os.path.isfile(vhacd_exec_path):
            raise FileNotFoundError(f"VHACD executable does not exist : {vhacd_exec_path}")

        mesh_filename = self.get_parameter("mesh_filename").value

        if not os.path.isfile(mesh_filename):
            raise FileNotFoundError(f"Input mesh does not exist: {mesh_filename}")

        return vhacd_exec_path, mesh_filename

    def run_vhacd_cb(self, _, response):
        def log_and_return_failure(msg):
            self.get_logger().error(msg)
            response.message = msg
            return response

        try:
            vhacd_exec_path, mesh_filename = self.validate_startup_parameters()
        except (RuntimeError, FileNotFoundError) as e:
            self.mesh_marker = None
            self.convex_hull_marker = None
            return log_and_return_failure(str(e))

        self.mesh_marker = make_mesh_marker(mesh_resource=f"file://{mesh_filename}")

        commands = [vhacd_exec_path, mesh_filename]

        for flag, name in self.vhacd_flag_map.items():
            commands.extend([flag, str(self.get_parameter(name).value).lower()])

        self.get_logger().info(f"Executing {' '.join(commands)}")

        completed_process = subprocess.run(commands, text=True)

        if completed_process.returncode != 0 or str(completed_process.stdout).startswith("Failed"):
            return log_and_return_failure(f"VHACD call failed: {completed_process.stderr}")

        if not os.path.isfile("/tmp/decomp.obj"):
            return log_and_return_failure("VHACD failed. Did you specify the mesh path correctly?")

        try:
            import trimesh

            scene = trimesh.load("decomp.obj")
            if isinstance(scene, trimesh.Trimesh):
                scene = trimesh.Scene([scene])
            num_hulls = len(scene.geometry)
            total_num_vertices = sum(geo.vertices.shape[0] for geo in scene.geometry.values())
            self.get_logger().info(f"{num_hulls} convex hulls ({total_num_vertices} verts total)")
        except ImportError:
            self.get_logger().info("Trimesh not installed, skipping vertex count logging")

        # RViz does not re-load mesh files, so save each new hull to a new name to force re-load
        if self.generation_idx:
            for old_file in glob.glob(f"decomp_{self.generation_idx}.*"):
                os.remove(old_file)
        self.generation_idx += 1
        for new_file in glob.glob("decomp.*"):
            os.rename(new_file, new_file.replace("decomp", f"decomp_{self.generation_idx}"))

        self.convex_hull_marker = make_mesh_marker(
            ns="convex_hull",
            alpha=0.5,
            mesh_resource=f"file://{os.path.abspath(f'decomp_{self.generation_idx}.obj')}",
        )

        self.publish_marker_array()

        response.success = True
        return response


def main():
    rclpy.init()
    node = InteractiveVHACDPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
