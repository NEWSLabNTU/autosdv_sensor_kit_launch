# Copyright 2020 Tier IV, Inc. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Sensing-side point cloud preprocessing, on the CPU or on the GPU.

`pointcloud_backend:=cpu` (the default) is the chain this kit has always run: the
single LiDAR is passed through to `concatenated/pointcloud` in `base_link`, with
no deskew and no outlier filtering.

`pointcloud_backend:=cuda` puts crop-self, distortion correction and ring outlier
filtering on the GPU as one kernel sequence, then hands the result to the same
passthrough. The drivers stay on the CPU either way -- Nebula has no CUDA decoder
for Velodyne -- so there is one host-to-device copy at the preprocessor's input.

Three things worth knowing before changing any of this.

1. ONLY THE VELODYNE CAN BE PREPROCESSED. The CUDA node fuses cropping with
   distortion correction, which needs a per-point time offset. Nebula publishes
   `velodyne_points` in the PointXYZIRCAEDT layout, which carries one. The Seyond
   driver registers PointXYZIRC -- x, y, z, intensity, return_type, ring -- and
   the Blickfeld driver publishes no per-point time either, so neither can be
   deskewed by anything, CPU or GPU. `pointcloud_backend:=cuda` with those models
   is refused rather than silently ignored. See phase 2.3 of
   docs/roadmap/6-golfcart-backport.md, which adds the field to the Seyond driver.

2. THE CONCATENATOR IS NOT USED HERE, and cannot be. Both the CPU and the CUDA
   concatenator refuse a single input topic:

       Component constructor threw an exception:
       Only one topic given. Need at least two topics to continue.

   Listing the same topic twice does load, but measured on a synthetic 10 Hz
   publisher it emits 1.7-2.4 Hz and logs "Reset the oldest collector" on every
   cycle: each message fills one slot, the collector waits out `timeout_sec` for
   a second that never comes, and the collector limit thrashes. Roughly 80% of
   frames are lost. This kit has one LiDAR, so the passthrough does the job the
   concatenator would have done -- including the transform to `base_frame`, which
   the CUDA preprocessor does not do; its output stays in the sensor frame.

3. The passthrough is a CPU node, so `pointcloud_backend:=cuda` still pays a
   device-to-host copy before `concatenated/pointcloud`. What it buys is the
   per-point work, which is the expensive part. A fully GPU-resident sensing
   chain needs a second LiDAR (making the CUDA concatenator usable) or a CUDA
   passthrough, which upstream does not ship.
"""

import os

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterFile

BACKENDS = ("cpu", "cuda")

# Model -> raw topic. The value is what the driver publishes.
LIDAR_TOPICS = {
    "vlp32c": "/sensing/lidar/velodyne_points",
    "cube1": "/sensing/lidar/bf_lidar/points_raw",
    "robin-w": "/sensing/lidar/iv_points",
}

# Models whose layout carries a per-point time offset, and can therefore be
# deskewed. See point 1 in the module docstring.
DESKEWABLE = ("vlp32c",)

TWIST_TOPIC = "/sensing/vehicle_velocity_converter/twist_with_covariance"
IMU_TOPIC = "/sensing/imu/imu_data"


def _param(name, context):
    return ParameterFile(
        param_file=LaunchConfiguration(name).perform(context), allow_substs=True
    )


def get_vehicle_info(context):
    """Derive the ego bounding box from the global vehicle parameters.

    Same derivation as aip_launcher's copy. The keys come from
    `vehicle_info_param_file`, which the sensing launch chain already puts into
    the launch context as global parameters.
    """
    gp = context.launch_configurations.get("ros_params", {})
    if not gp:
        gp = dict(context.launch_configurations.get("global_params", {}))
    return {
        "min_longitudinal_offset": -gp["rear_overhang"],
        "max_longitudinal_offset": gp["front_overhang"] + gp["wheel_base"],
        "min_lateral_offset": -(gp["wheel_tread"] / 2.0 + gp["right_overhang"]),
        "max_lateral_offset": gp["wheel_tread"] / 2.0 + gp["left_overhang"],
        "min_height_offset": 0.0,
        "max_height_offset": gp["vehicle_height"],
    }


def make_cuda_preprocessor_node(context, input_topic, output_topic):
    """Crop-self, deskew and ring outlier filtering, as one GPU node.

    `extra_arguments` is deliberately absent rather than set to false. The node
    cannot run with intra-process comms at all, because cuda_blackboard's
    negotiation topics are transient_local and rclcpp rejects that pairing:

        Component constructor threw an exception:
        intraprocess communication allowed only with volatile durability

    aip_launcher handles it the same way, by commenting the option out.
    """
    vehicle_info = get_vehicle_info(context)
    # The CUDA node takes the crop boxes as lists, one entry per box. This
    # vehicle has one.
    crop = {
        "crop_box.min_x": [vehicle_info["min_longitudinal_offset"]],
        "crop_box.max_x": [vehicle_info["max_longitudinal_offset"]],
        "crop_box.min_y": [vehicle_info["min_lateral_offset"]],
        "crop_box.max_y": [vehicle_info["max_lateral_offset"]],
        "crop_box.min_z": [vehicle_info["min_height_offset"]],
        "crop_box.max_z": [vehicle_info["max_height_offset"]],
        "crop_box.negative": [True],
    }
    return ComposableNode(
        package="autoware_cuda_pointcloud_preprocessor",
        plugin="autoware::cuda_pointcloud_preprocessor::CudaPointcloudPreprocessorNode",
        name="cuda_pointcloud_preprocessor_node",
        parameters=[
            _param("cuda_pointcloud_preprocessor_param_path", context),
            crop,
            {"base_frame": LaunchConfiguration("base_frame")},
        ],
        remappings=[
            ("~/input/pointcloud", input_topic),
            ("~/input/twist", TWIST_TOPIC),
            ("~/input/imu", IMU_TOPIC),
            ("~/output/pointcloud", output_topic),
            ("~/output/pointcloud/cuda", f"{output_topic}/cuda"),
        ],
    )


def make_passthrough_node(input_topic):
    """Transform the cloud into base_frame and publish it as the concatenated one.

    This is what stands in for the concatenator on a single-LiDAR kit; see point
    2 in the module docstring.
    """
    return ComposableNode(
        package="autoware_pointcloud_preprocessor",
        plugin="autoware::pointcloud_preprocessor::PassThroughFilterComponent",
        name="passthrough_as_concatenate",
        remappings=[
            ("input", input_topic),
            ("output", "concatenated/pointcloud"),
        ],
        parameters=[
            {
                "output_frame": LaunchConfiguration("base_frame"),
                "remove_nan": True,  # Remove NaN points
                "float_min": -999.0,  # Don't filter by range
                "float_max": 999.0,  # Don't filter by range
            }
        ],
        extra_arguments=[
            {"use_intra_process_comms": LaunchConfiguration("use_intra_process")}
        ],
    )


def launch_setup(context, *args, **kwargs):
    lidar_model = LaunchConfiguration("lidar_model").perform(context)
    backend = LaunchConfiguration("pointcloud_backend").perform(context)

    if lidar_model not in LIDAR_TOPICS:
        valid = ", ".join(LIDAR_TOPICS)
        raise ValueError(
            f"Invalid lidar_model value: '{lidar_model}'. Valid values are: {valid}"
        )
    if backend not in BACKENDS:
        raise ValueError(
            f"pointcloud_backend must be one of {list(BACKENDS)}, got {backend!r}. "
            "Set it with `just launch pointcloud_backend:=cuda`."
        )

    raw_topic = LIDAR_TOPICS[lidar_model]

    if backend == "cpu":
        nodes = [make_passthrough_node(raw_topic)]
    else:
        if lidar_model not in DESKEWABLE:
            raise ValueError(
                f"pointcloud_backend:=cuda needs a per-point time offset, and the "
                f"{lidar_model} driver does not publish one (it registers "
                f"PointXYZIRC, with no time_stamp field). Nothing can deskew that "
                f"cloud, on the CPU or the GPU. Use pointcloud_backend:=cpu with "
                f"this LiDAR, or lidar_model:={DESKEWABLE[0]}."
            )
        preprocessed = "/sensing/lidar/preprocessed/pointcloud"
        nodes = [
            make_cuda_preprocessor_node(context, raw_topic, preprocessed),
            make_passthrough_node(preprocessed),
        ]

    return [
        LoadComposableNodes(
            composable_node_descriptions=nodes,
            target_container=LaunchConfiguration("pointcloud_container_name"),
            condition=IfCondition(LaunchConfiguration("use_concat_filter")),
        )
    ]


def generate_launch_description():
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None):
        launch_arguments.append(DeclareLaunchArgument(name, default_value=default_value))

    cuda_pre = get_package_share_directory("autoware_cuda_pointcloud_preprocessor")

    add_launch_arg("base_frame", "base_link")
    add_launch_arg("use_multithread", "False")
    add_launch_arg("use_intra_process", "False")
    add_launch_arg("pointcloud_container_name", "pointcloud_container")
    add_launch_arg("use_concat_filter", "True")
    add_launch_arg("lidar_model", "vlp32c")
    add_launch_arg("pointcloud_backend", "cpu")
    # Stock Autoware defaults for deskew and the ring outlier filter. The CUDA
    # node reads both from this one file.
    add_launch_arg(
        "cuda_pointcloud_preprocessor_param_path",
        os.path.join(cuda_pre, "config", "cuda_pointcloud_preprocessor.param.yaml"),
    )

    set_container_executable = SetLaunchConfiguration(
        "container_executable",
        "component_container",
        condition=UnlessCondition(LaunchConfiguration("use_multithread")),
    )

    set_container_mt_executable = SetLaunchConfiguration(
        "container_executable",
        "component_container_mt",
        condition=IfCondition(LaunchConfiguration("use_multithread")),
    )

    return launch.LaunchDescription(
        launch_arguments
        + [set_container_executable, set_container_mt_executable]
        + [OpaqueFunction(function=launch_setup)]
    )
