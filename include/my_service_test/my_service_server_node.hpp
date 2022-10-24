// Copyright 2022 m12watanabe1a.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace my_service_test
{
using ServiceT = std_srvs::srv::Trigger;
class MyServiceServerNode : public rclcpp::Node
{
private:
  rclcpp::Service<ServiceT>::SharedPtr server_;

public:
  MyServiceServerNode() = delete;
  explicit MyServiceServerNode(const rclcpp::NodeOptions &);

private:
  void handleService(
    const std::shared_ptr<rmw_request_id_t>,
    const ServiceT::Request::SharedPtr,
    ServiceT::Response::SharedPtr);
};
}  // namespace my_service_test

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(my_service_test::MyServiceServerNode)
