#include <rclcpp/rclcpp.hpp>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

class BehaviourManager : public rclcpp::Node
{
public:
    BehaviourManager() : Node("behaviour_manager")
    {
        /***** Parameters *****/
        freq_ = this->declare_parameter<double>("freq", 1.0);

        /***** Timers *****/
        timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / freq_), std::bind(&BehaviourManager::callback_timer, this));

        /***** Services *****/
        client_get_state_ = this->create_client<lifecycle_msgs::srv::GetState>(
            node_get_state_topic_);
        client_change_state_ = this->create_client<lifecycle_msgs::srv::ChangeState>(
            node_change_state_topic_);
    }

private:
    /***** Variables *****/
    // Node parameters
    double freq_;

    // Node variables
    rclcpp::TimerBase::SharedPtr timer_;

    // Lifecycle node pointers
    static constexpr char const *lifecycle_node_ = "lifecycle_talker";
    // Lifecycle service pointers
    static constexpr char const *node_get_state_topic_ = "lifecycle_talker/get_state";
    static constexpr char const *node_change_state_topic_ = "lifecycle_talker/change_state";
    std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>> client_get_state_;
    std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> client_change_state_;

    // Variables
    size_t count_ = 0;

    /***** Functions *****/
    template <typename FutureT, typename WaitTimeT>
    std::future_status wait_for_result(
        FutureT &future,
        WaitTimeT time_to_wait)
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

    unsigned int get_state(std::chrono::seconds time_out = std::chrono::seconds(3))
    {
        auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();

        if (!client_get_state_->wait_for_service(time_out))
        {
            RCLCPP_ERROR(
                get_logger(),
                "Service %s is not available.",
                client_get_state_->get_service_name());
            return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
        }

        // We send the service request for asking the current
        // state of the lc_talker node.
        auto future_result = client_get_state_->async_send_request(request);

        // Let's wait until we have the answer from the node.
        // If the request times out, we return an unknown state.
        auto future_status = wait_for_result(future_result, time_out);

        if (future_status != std::future_status::ready)
        {
            RCLCPP_ERROR(
                get_logger(), "Server time out while getting current state for node %s", lifecycle_node_);
            return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
        }

        // We have an succesful answer. So let's print the current state.
        if (future_result.get())
        {
            RCLCPP_INFO(
                get_logger(), "Node %s has current state %s.",
                lifecycle_node_, future_result.get()->current_state.label.c_str());
            return future_result.get()->current_state.id;
        }
        else
        {
            RCLCPP_ERROR(
                get_logger(), "Failed to get current state for node %s", lifecycle_node_);
            return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
        }
    }

    bool change_state(std::uint8_t transition, std::chrono::seconds time_out = std::chrono::seconds(3))
    {
        auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
        request->transition.id = transition;

        if (!client_change_state_->wait_for_service(time_out))
        {
            RCLCPP_ERROR(
                get_logger(),
                "Service %s is not available.",
                client_change_state_->get_service_name());
            return false;
        }

        // We send the request with the transition we want to invoke.
        auto future_result = client_change_state_->async_send_request(request);

        // Let's wait until we have the answer from the node.
        // If the request times out, we return an unknown state.
        auto future_status = wait_for_result(future_result, time_out);

        if (future_status != std::future_status::ready)
        {
            RCLCPP_ERROR(
                get_logger(), "Server time out while getting current state for node %s", lifecycle_node_);
            return false;
        }

        // We have an answer, let's print our success.
        if (future_result.get()->success)
        {
            RCLCPP_INFO(
                get_logger(), "Transition %d successfully triggered.", static_cast<int>(transition));
            return true;
        }
        else
        {
            RCLCPP_WARN(
                get_logger(), "Failed to trigger transition %u", static_cast<unsigned int>(transition));
            return false;
        }
    }

    /***** Callbacks *****/
    void callback_timer()
    {

        RCLCPP_INFO(this->get_logger(), "Count: %ld", count_);

        // configure
        if (count_ == 0)
        {
            if (!this->change_state(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE))
            {
                // return;
            }
            if (!this->get_state())
            {
                // return;
            }
        }

        // activate
        if (count_ == 1)
        {
            if (!rclcpp::ok())
            {
                // return;
            }
            if (!this->change_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE))
            {
                // return;
            }
            if (!this->get_state())
            {
                // return;
            }
        }

        // deactivate
        if (count_ == 2)
        {
            if (!rclcpp::ok())
            {
                // return;
            }
            if (!this->change_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE))
            {
                // return;
            }
            if (!this->get_state())
            {
                // return;
            }
        }

        // activate it again
        if (count_ == 2)
        {
            if (!rclcpp::ok())
            {
                // return;
            }
            if (!this->change_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE))
            {
                // return;
            }
            if (!this->get_state())
            {
                // return;
            }
        }

        // and deactivate it again
        if (count_ == 3)
        {
            if (!rclcpp::ok())
            {
                // return;
            }
            if (!this->change_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE))
            {
                // return;
            }
            if (!this->get_state())
            {
                // return;
            }
        }

        // we cleanup
        if (count_ == 4)
        {
            if (!rclcpp::ok())
            {
                // return;
            }
            if (!this->change_state(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP))
            {
                // return;
            }
            if (!this->get_state())
            {
                // return;
            }
        }

        // and finally shutdown
        // Note: We have to be precise here on which shutdown transition id to call
        // We are currently in the unconfigured state and thus have to call
        // TRANSITION_UNCONFIGURED_SHUTDOWN
        if (count_ == 5)
        {
            if (!rclcpp::ok())
            {
                // return;
            }
            if (!this->change_state(lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN))
            {
                // return;
            }
            if (!this->get_state())
            {
                // return;
            }
        }

        count_++;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BehaviourManager>());
    rclcpp::shutdown();
    return 0;
}