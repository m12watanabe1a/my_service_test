# Copyright 2022 m12watanabe1a.
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

from launch import LaunchDescription
from launch.actions import Shutdown
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate separated service pair."""
    server_container_node = ComposableNodeContainer(
        name='server',
        package='rclcpp_components',
        namespace='',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                name='my_server',
                package='my_service_test',
                plugin='my_service_test::MyServiceServerNode',), ],)
    client_container_node = ComposableNodeContainer(
        name='client',
        package='rclcpp_components',
        namespace='',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                name='my_client',
                package='my_service_test',
                plugin='my_service_test::MyServiceClientNode',), ],
        on_exit=Shutdown())

    ld = LaunchDescription()

    ld.add_action(server_container_node)
    ld.add_action(client_container_node)

    return ld
