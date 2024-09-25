#include <string>
#include <chrono>
#include <memory>

#include "behaviour_manager/lifecycle_service_client.hpp"

LifecycleServiceClient::LifecycleServiceClient(const std::string &lifecycle_node_name, rclcpp::Node::SharedPtr parent_node)
    : node_(parent_node), lifecycle_node_name_(lifecycle_node_name),
      get_state_(lifecycle_node_name + "/get_state", node_),
      change_state_(lifecycle_node_name + "/change_state", node_)
{
    // Block until server is up
    rclcpp::Rate r(20);

    while (!get_state_.wait_for_service(std::chrono::seconds(2)))
    {
        RCLCPP_INFO(node_->get_logger(), "Waiting for service %s...", get_state_.get_service_name().c_str());
        r.sleep();
    }

    while (!change_state_.wait_for_service(std::chrono::seconds(2)))
    {
        RCLCPP_INFO(node_->get_logger(), "Waiting for service %s...", change_state_.get_service_name().c_str());
        r.sleep();
    }
}

uint8_t LifecycleServiceClient::get_state(
    const std::chrono::seconds timeout)
{
    if (!get_state_.wait_for_service(timeout))
    {
        RCLCPP_ERROR(node_->get_logger(), "Service %s is not available.", get_state_.get_service_name().c_str());
        return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }

    // Request service
    auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
    auto result = get_state_.invoke(request, timeout);
    return result->current_state.id;
}

bool LifecycleServiceClient::change_state(
    const uint8_t transition,
    const std::chrono::seconds timeout)
{
    // Check service availability
    if (!change_state_.wait_for_service(timeout))
    {
        RCLCPP_ERROR(node_->get_logger(), "Service %s is not available.", change_state_.get_service_name().c_str());
        return false;
    }

    // Request service
    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = transition;
    auto response = change_state_.invoke(request, timeout);
    return response.get();
}
