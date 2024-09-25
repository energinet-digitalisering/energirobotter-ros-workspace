// SOURCE: https://github.com/ros-navigation/navigation2/blob/humble/nav2_util/include/nav2_util/service_client.hpp

#ifndef BEHAVIOUR_MANAGER__SERVICE_CLIENT_HPP_
#define BEHAVIOUR_MANAGER__SERVICE_CLIENT_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"

/**
 * @class ServiceClient
 * @brief A simple wrapper on ROS2 services for invoke() and block-style calling
 */
template <class ServiceT>
class ServiceClient
{
public:
    /**
     * @brief A constructor
     * @param service_name name of the service to call
     * @param provided_node Node to create the service client off of
     */
    explicit ServiceClient(
        const std::string &service_name,
        const rclcpp::Node::SharedPtr &provided_node)
        : service_name_(service_name), node_(provided_node)
    {
        callback_group_ = node_->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive,
            false);
        callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());
        client_ = node_->create_client<ServiceT>(
            service_name,
            rclcpp::ServicesQoS().get_rmw_qos_profile(),
            callback_group_);
    }

    using RequestType = typename ServiceT::Request;
    using ResponseType = typename ServiceT::Response;

    /**
     * @brief Invoke the service and block until completed or timed out
     * @param request The request object to call the service using
     * @param timeout Maximum timeout to wait for, default infinite
     * @return Response A pointer to the service response from the request
     */
    typename ResponseType::SharedPtr invoke(
        typename RequestType::SharedPtr &request,
        const std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1))
    {
        while (!client_->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
                throw std::runtime_error(
                    service_name_ + " service client: interrupted while waiting for service");
            }
            RCLCPP_INFO(
                node_->get_logger(), "%s service client: waiting for service to appear...",
                service_name_.c_str());
        }

        RCLCPP_DEBUG(
            node_->get_logger(), "%s service client: send async request",
            service_name_.c_str());
        auto future_result = client_->async_send_request(request);

        if (callback_group_executor_.spin_until_future_complete(future_result, timeout) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            // Pending request must be manually cleaned up if execution is interrupted or timed out
            client_->remove_pending_request(future_result);
            throw std::runtime_error(service_name_ + " service client: async_send_request failed");
        }

        return future_result.get();
    }

    /**
     * @brief Invoke the service and block until completed
     * @param request The request object to call the service using
     * @param Response A pointer to the service response from the request
     * @return bool Whether it was successfully called
     */
    bool invoke(
        typename RequestType::SharedPtr &request,
        typename ResponseType::SharedPtr &response)
    {
        while (!client_->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
                throw std::runtime_error(
                    service_name_ + " service client: interrupted while waiting for service");
            }
            RCLCPP_INFO(
                node_->get_logger(), "%s service client: waiting for service to appear...",
                service_name_.c_str());
        }

        RCLCPP_DEBUG(
            node_->get_logger(), "%s service client: send async request",
            service_name_.c_str());
        auto future_result = client_->async_send_request(request);

        if (callback_group_executor_.spin_until_future_complete(future_result) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            // Pending request must be manually cleaned up if execution is interrupted or timed out
            client_->remove_pending_request(future_result);
            return false;
        }

        response = future_result.get();
        return response.get();
    }

    /**
     * @brief Block until a service is available or timeout
     * @param timeout Maximum timeout to wait for, default infinite
     * @return bool true if service is available
     */
    bool wait_for_service(const std::chrono::nanoseconds timeout = std::chrono::nanoseconds::max())
    {
        return client_->wait_for_service(timeout);
    }

    /**
     * @brief Gets the service name
     * @return string Service name
     */
    std::string get_service_name()
    {
        return service_name_;
    }

protected:
    std::string service_name_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
    typename rclcpp::Client<ServiceT>::SharedPtr client_;
};

#endif // BEHAVIOUR_MANAGER__SERVICE_CLIENT_HPP_