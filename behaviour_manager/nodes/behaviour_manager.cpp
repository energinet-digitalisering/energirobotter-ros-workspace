#include <map>
#include <string>
#include <vector>

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
        declare_parameter("node_names", rclcpp::PARAMETER_STRING_ARRAY);

        freq_ = this->declare_parameter<double>("freq", 1.0);
        node_names_ = get_parameter("node_names").as_string_array();

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

    std::vector<std::string> node_names_;
    std::map<std::string, std::shared_ptr<LifecycleServiceClient>> node_map_;

    // Variables
    size_t count_ = 0;

    /***** Functions *****/
    void create_lifecycle_service_client()
    {
        // shared_from_this() has to be called after initialization
        // Create a timer with a very short duration to defer the initialization
        init_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(0),
            [this]() -> void
            {
                for (auto &node_name : node_names_)
                {
                    RCLCPP_WARN_STREAM(this->get_logger(), "node name: " << node_name);

                    node_map_[node_name] =
                        std::make_shared<LifecycleServiceClient>(node_name, shared_from_this());
                }

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
            for (auto &node : node_map_)
            {
                node.second->change_state(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
            }
        }

        // activate
        if (count_ == 1)
        {
            for (auto &node : node_map_)
            {
                node.second->change_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
            }
        }

        // deactivate
        if (count_ == 2)
        {
            for (auto &node : node_map_)
            {
                node.second->change_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
            }
        }

        // activate it again
        if (count_ == 3)
        {
            for (auto &node : node_map_)
            {
                node.second->change_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
            }
        }

        // and deactivate it again
        if (count_ == 4)
        {
            for (auto &node : node_map_)
            {
                node.second->change_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
            }
        }

        // we cleanup
        if (count_ == 5)
        {
            for (auto &node : node_map_)
            {
                node.second->change_state(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);
            }
        }

        // and finally shutdown
        if (count_ == 6)
        {
            for (auto &node : node_map_)
            {
                node.second->change_state(lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN);
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