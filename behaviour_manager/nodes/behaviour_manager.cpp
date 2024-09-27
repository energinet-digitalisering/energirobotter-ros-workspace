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

        /***** Services *****/
        create_lifecycle_service_client();
    }

private:
    /***** Variables *****/
    // Node parameters
    double freq_;

    // Node variables
    rclcpp::TimerBase::SharedPtr init_timer_;

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

                init_timer_->cancel();
            });
    }

    /***** Callbacks *****/
    {

        {
        }

    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BehaviourManager>());
    rclcpp::shutdown();
    return 0;
}