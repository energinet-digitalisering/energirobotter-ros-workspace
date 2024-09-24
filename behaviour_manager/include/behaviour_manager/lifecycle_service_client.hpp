// SOURCE: https://github.com/ros-navigation/navigation2/blob/main/nav2_util/include/nav2_util/lifecycle_service_client.hpp

#ifndef BEHAVIOUR_MANAGER__LIFECYCLE_SERVICE_CLIENT_HPP_
#define BEHAVIOUR_MANAGER__LIFECYCLE_SERVICE_CLIENT_HPP_

#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

// Helper functions to interact with a lifecycle node.
class LifecycleServiceClient
{
public:
    LifecycleServiceClient(const std::string &lifecycle_node_name, rclcpp::Node::SharedPtr parent_node);

    // Get the current state as a lifecycle_msgs::msg::State id value
    uint8_t get_state(const std::chrono::seconds timeout = std::chrono::seconds(2));

    // Trigger a state change
    bool change_state(
        const uint8_t transition, // takes a lifecycle_msgs::msg::Transition id
        const std::chrono::seconds timeout = std::chrono::seconds(5));

private:
    // Variables
    rclcpp::Node::SharedPtr node_;
    std::string lifecycle_node_name_;
    std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>> get_state_;
    std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> change_state_;

    // Typedef
    typedef rclcpp::Client<lifecycle_msgs::srv::GetState>::FutureAndRequestId FutureGet;
    typedef rclcpp::Client<lifecycle_msgs::srv::ChangeState>::FutureAndRequestId FutureChange;

    // Functions
    template <typename FutureT>
    std::future_status wait_for_result(const FutureT &future, std::chrono::seconds time_to_wait);
};

#endif // BEHAVIOUR_MANAGER__LIFECYCLE_SERVICE_CLIENT_HPP_