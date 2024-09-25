#include <rclcpp/rclcpp.hpp>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

#include "behaviour_manager/lifecycle_service_client.hpp"

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
        create_lifecycle_service_client();
    }

private:
    /***** Variables *****/
    // Node parameters
    double freq_;

    // Node variables
    rclcpp::TimerBase::SharedPtr init_timer_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Lifecycle node pointers
    std::string lifecycle_node_ = "lifecycle_talker";
    // Lifecycle service pointers
    std::string node_get_state_topic_ = "lifecycle_talker/get_state";
    std::string node_change_state_topic_ = "lifecycle_talker/change_state";

    std::shared_ptr<LifecycleServiceClient> lifecycle_client_;

    // Variables
    size_t count_ = 0;

    /***** Functions *****/
    void
    create_lifecycle_service_client()
    {
        // shared_from_this() has to be called after initialization
        // Create a timer with a very short duration to defer the initialization
        init_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(0),
            [this]() -> void
            {
                lifecycle_client_ =
                    std::make_shared<LifecycleServiceClient>(lifecycle_node_, shared_from_this());
                init_timer_->cancel();
            });
    }

    /***** Callbacks *****/
    void callback_timer()
    {

        RCLCPP_INFO(this->get_logger(), "Count: %ld", count_);

        // configure
        if (count_ == 0)
        {
            lifecycle_client_->change_state(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
        }

        // activate
        if (count_ == 1)
        {
            lifecycle_client_->change_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
        }

        // deactivate
        if (count_ == 2)
        {
            lifecycle_client_->change_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
        }

        // activate it again
        if (count_ == 3)
        {
            lifecycle_client_->change_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
        }

        // and deactivate it again
        if (count_ == 4)
        {
            lifecycle_client_->change_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
        }

        // we cleanup
        if (count_ == 5)
        {
            lifecycle_client_->change_state(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);
        }

        // and finally shutdown
        if (count_ == 6)
        {
            lifecycle_client_->change_state(lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN);
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