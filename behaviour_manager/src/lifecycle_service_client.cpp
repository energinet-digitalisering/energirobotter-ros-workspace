#include <string>
#include <chrono>
#include <memory>

#include "behaviour_manager/lifecycle_service_client.hpp"

LifecycleServiceClient::LifecycleServiceClient(const std::string &lifecycle_node_name, rclcpp::Node::SharedPtr parent_node)
    : node_(parent_node), lifecycle_node_name_(lifecycle_node_name)
{

    get_state_ = node_->create_client<lifecycle_msgs::srv::GetState>(lifecycle_node_name + "/get_state");
    change_state_ = node_->create_client<lifecycle_msgs::srv::ChangeState>(lifecycle_node_name + "/change_state");

    // Block until server is up
    rclcpp::Rate r(20);

    while (!get_state_->wait_for_service(std::chrono::seconds(2)))
    {
        RCLCPP_INFO(node_->get_logger(), "Waiting for service %s...", get_state_->get_service_name());
        r.sleep();
    }

    while (!change_state_->wait_for_service(std::chrono::seconds(2)))
    {
        RCLCPP_INFO(node_->get_logger(), "Waiting for service %s...", change_state_->get_service_name());
        r.sleep();
    }
}

template <typename FutureT>
std::future_status LifecycleServiceClient::wait_for_result(
    const FutureT &future,
    std::chrono::seconds time_to_wait)
{
    auto end = std::chrono::steady_clock::now() + time_to_wait;
    std::chrono::milliseconds wait_period(100);
    std::future_status status = std::future_status::timeout;
    do
    {
        auto now = std::chrono::steady_clock::now();
        auto time_left = end - now;
        if (time_left <= std::chrono::seconds(0))
        {
            break;
        }
        status = future.wait_for((time_left < wait_period) ? time_left : wait_period);
    } while (rclcpp::ok() && status != std::future_status::ready);
    return status;
}

uint8_t LifecycleServiceClient::get_state(
    const std::chrono::seconds timeout)
{
    if (!get_state_->wait_for_service(timeout))
    {
        RCLCPP_ERROR(node_->get_logger(), "Service %s is not available.", get_state_->get_service_name());
        return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }

    // Request service
    auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();

    FutureGet future_result = get_state_->async_send_request(request);
    std::future_status future_status = wait_for_result(future_result, timeout);

    // Check status
    if (future_status != std::future_status::ready)
    {
        RCLCPP_ERROR_STREAM(
            node_->get_logger(), "Server time out while getting current state for node " << lifecycle_node_name_);
        return false;
    }

    // Get response
    if (future_result.get())
    {
        RCLCPP_INFO(
            node_->get_logger(), "Current state is %s.",
            future_result.get()->current_state.label.c_str());
        return future_result.get()->current_state.id;
    }
    else
    {
        RCLCPP_WARN_STREAM(
            node_->get_logger(), "Failed to get current state for node " << lifecycle_node_name_);
        return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }
}

bool LifecycleServiceClient::change_state(
    const uint8_t transition,
    const std::chrono::seconds timeout)
{
    // Check service availability
    if (!change_state_->wait_for_service(timeout))
    {
        RCLCPP_ERROR(node_->get_logger(), "Service %s is not available.", change_state_->get_service_name());
        return false;
    }

    // Request service
    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = transition;

    FutureChange future_result = change_state_->async_send_request(request);
    std::future_status future_status = wait_for_result(future_result, timeout);

    // Check status
    if (future_status != std::future_status::ready)
    {
        RCLCPP_ERROR_STREAM(
            node_->get_logger(), "Server time out while getting current state for node " << lifecycle_node_name_);
        return false;
    }

    // Get response
    if (future_result.get()->success)
    {
        RCLCPP_INFO(
            node_->get_logger(), "Transition %d successfully triggered.", static_cast<int>(transition));
        return true;
    }
    else
    {
        RCLCPP_WARN(
            node_->get_logger(), "Failed to trigger transition %u", static_cast<unsigned int>(transition));
        return false;
    }
}
