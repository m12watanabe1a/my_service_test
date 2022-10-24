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

#include "my_service_test/my_service_client_node.hpp"

namespace my_service_test
{

MyServiceClientNode::MyServiceClientNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("my_service_client", options)
{
  this->callback_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  this->callback_group_executor_.add_callback_group(
    this->callback_group_, this->get_node_base_interface());

  this->client_ = this->create_client<ServiceT>(
    "my_service", rmw_qos_profile_default, this->callback_group_);

  using namespace std::chrono_literals;    // NOLINT
  this->init_timer_ = this->create_wall_timer(
    1s,
    std::bind(&MyServiceClientNode::invoke, this));
}

void MyServiceClientNode::invoke()
{
  this->init_timer_->cancel();
  RCLCPP_WARN(this->get_logger(), "Invoked");

  if (!this->waitForService(std::chrono::seconds(1))) {
    RCLCPP_ERROR(
      this->get_logger(), "client interrupted while waiting for service to appear.");
    rclcpp::shutdown();
  }


  auto req = std::make_shared<ServiceT::Request>();
  auto future_result = this->client_->async_send_request(req);

  const auto timeout = std::chrono::seconds(3);

  if (this->callback_group_executor_.spin_until_future_complete(
      future_result, timeout) != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "%s service client: async_send_request failed",
      this->client_->get_service_name());
    rclcpp::shutdown();
    return;
  }

  RCLCPP_INFO(
    this->get_logger(),
    "STATUS: [%s] Message: [%s]",
    future_result.get()->success ? "Success" : "Failed",
    future_result.get()->message.c_str());

  rclcpp::shutdown();
}

bool MyServiceClientNode::waitForService(const std::chrono::nanoseconds timeout)
{
  return this->client_->wait_for_service(timeout);
}
}  // namespace my_service_test
