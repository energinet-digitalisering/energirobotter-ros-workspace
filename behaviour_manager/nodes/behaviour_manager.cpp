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
        // Create a timer with a very short duration to defer the initialization
        init_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(0),
            [this]() -> void
            {
                create_lifecycle_service_client();
                init_timer_->cancel();
            });
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
        lifecycle_client_ =
            std::make_shared<LifecycleServiceClient>(lifecycle_node_, shared_from_this());
    }

    /***** Callbacks *****/
    void callback_timer()
    {

        RCLCPP_INFO(this->get_logger(), "Count: %ld", count_);

        // configure
        if (count_ == 0)
        {
            if (!lifecycle_client_->change_state(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE))
            {
                return;
            }
            if (!lifecycle_client_->get_state())
            {
                return;
            }
        }

        //         // activate
        //         if (count_ == 1)
        //         {
        //             if (!rclcpp::ok())
        //             {
        //                 // return;
        //             }
        //             if (!this->change_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE))
        //             {
        //                 // return;
        //             }
        //             if (!this->get_state())
        //             {
        //                 // return;
        //             }
        //         }

        //         // deactivate
        //         if (count_ == 2)
        //         {
        //             if (!rclcpp::ok())
        //             {
        //                 // return;
        //             }
        //             if (!this->change_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE))
        //             {
        //                 // return;
        //             }
        //             if (!this->get_state())
        //             {
        //                 // return;
        //             }
        //         }

        //         // activate it again
        //         if (count_ == 2)
        //         {
        //             if (!rclcpp::ok())
        //             {
        //                 // return;
        //             }
        //             if (!this->change_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE))
        //             {
        //                 // return;
        //             }
        //             if (!this->get_state())
        //             {
        //                 // return;
        //             }
        //         }

        //         // and deactivate it again
        //         if (count_ == 3)
        //         {
        //             if (!rclcpp::ok())
        //             {
        //                 // return;
        //             }
        //             if (!this->change_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE))
        //             {
        //                 // return;
        //             }
        //             if (!this->get_state())
        //             {
        //                 // return;
        //             }
        //         }

        //         // we cleanup
        //         if (count_ == 4)
        //         {
        //             if (!rclcpp::ok())
        //             {
        //                 // return;
        //             }
        //             if (!this->change_state(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP))
        //             {
        //                 // return;
        //             }
        //             if (!this->get_state())
        //             {
        //                 // return;
        //             }
        //         }

        //         // and finally shutdown
        //         // Note: We have to be precise here on which shutdown transition id to call
        //         // We are currently in the unconfigured state and thus have to call
        //         // TRANSITION_UNCONFIGURED_SHUTDOWN
        //         if (count_ == 5)
        //         {
        //             if (!rclcpp::ok())
        //             {
        //                 // return;
        //             }
        //             if (!this->change_state(lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN))
        //             {
        //                 // return;
        //             }
        //             if (!this->get_state())
        //             {
        //                 // return;
        //             }
        //         }

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