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
    std::map<std::string, std::shared_ptr<LifecycleServiceClient>> node_map_;

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
                    node_map_[node_name] =
                        std::make_shared<LifecycleServiceClient>(node_name, shared_from_this());
                }

                configure_nodes();

                init_timer_->cancel();
            });
    }

    void configure_nodes()
    {
        for (auto &node : node_map_)
        {
            node.second->change_state(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
        }
    }

    void deactivate_nodes()
    {
        for (auto &node : node_map_)
        {
            node.second->change_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
        }
    }

    void shutdown_nodes()
    {
        for (auto &node : node_map_)
        {
            node.second->change_state(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);
            node.second->change_state(lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN);
        }
    }

    void activate_node_exclusive(std::string node_name)
    {
        deactivate_nodes();
        node_map_[node_name]->change_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
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