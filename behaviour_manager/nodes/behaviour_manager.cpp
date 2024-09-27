#include <map>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "energirobotter_interfaces/srv/activate_node.hpp"

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

        /***** Services *****/
        service_activate_node_ = this->create_service<energirobotter_interfaces::srv::ActivateNode>("/activate_node", std::bind(&BehaviourManager::callback_activate_node, this,
                                                                                                                                std::placeholders::_1, std::placeholders::_2));

        /***** Lifecycle Nodes Services *****/
        init_allowed_transitions();
        create_lifecycle_service_client();
    }

    ~BehaviourManager()
    {
        shutdown_nodes();
    }

private:
    /***** Variables *****/
    // Node parameters
    double freq_;

    // Node variables
    rclcpp::Service<energirobotter_interfaces::srv::ActivateNode>::SharedPtr service_activate_node_;
    rclcpp::TimerBase::SharedPtr init_timer_;

    // Variables
    std::vector<std::string> node_names_;
    std::map<std::string, uint8_t> node_transition_log_;
    std::map<std::string, std::shared_ptr<LifecycleServiceClient>> node_map_;

    std::map<uint8_t, std::vector<uint8_t>> allowed_transitions_;

    /***** Functions *****/
    void init_allowed_transitions()
    {
        // https://design.ros2.org/articles/node_lifecycle.html
        // Knowing the last transition gives information about the state.
        // allowed_transitions_ given a transition, will return a vector with the allowed transitions, to avoid crash with Python Lifecyle nodes.
        // Will be made redundant when this PR is merged: https://github.com/ros2/rclpy/pull/1319

        allowed_transitions_[lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE] =
            {
                lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE,
                lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP,
                lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN,
            };

        allowed_transitions_[lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE] =
            {
                lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE,
                lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN,
            };

        allowed_transitions_[lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE] = allowed_transitions_[lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE];
    }

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
                    node_map_[node_name] =
                        std::make_shared<LifecycleServiceClient>(node_name, shared_from_this());
                }

                configure_nodes();

                init_timer_->cancel();
            });
    }

    void change_state_node(std::string node_name, uint8_t transition)
    {
        if (node_transition_log_[node_name] == transition)
        {
            return;
        }

        node_map_[node_name]->change_state(transition);
        node_transition_log_[node_name] = transition;
    }

    void change_state_all_nodes(uint8_t transition)
    {
        for (auto &node_name : node_names_)
        {
            std::vector<uint8_t> allowed_transitions = allowed_transitions_[node_transition_log_[node_name]];
            if (not allowed_transitions.empty())
            {
                if (std::count(allowed_transitions.begin(), allowed_transitions.end(), transition) == 0)
                {
                    return;
                }
            }

            change_state_node(node_name, transition);
        }
    }

    void configure_nodes()
    {
        change_state_all_nodes(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    }

    void deactivate_nodes()
    {
        change_state_all_nodes(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
    }

    void shutdown_nodes()
    {
        change_state_all_nodes(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);
        change_state_all_nodes(lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN);
    }

    void activate_node_exclusive(std::string node_name)
    {
        deactivate_nodes();
        change_state_node(node_name, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
    }

    /***** Callbacks *****/
    void callback_activate_node(
        const std::shared_ptr<energirobotter_interfaces::srv::ActivateNode::Request> request,
        std::shared_ptr<energirobotter_interfaces::srv::ActivateNode::Response> response)
    {
        std::string node_name = request->node;

        if (node_map_.count(node_name) == 0)
        {
            std::string message;
            message = node_name + " is not a managed lifecylce node, valid nodes are: \n";
            for (auto &node_name : node_names_)
            {
                message += "    " + node_name + '\n';
            }
            RCLCPP_WARN_STREAM(this->get_logger(), message);

            response->success = false;
            return;
        }

        activate_node_exclusive(node_name);
        response->success = true;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BehaviourManager>());
    rclcpp::shutdown();
    return 0;
}