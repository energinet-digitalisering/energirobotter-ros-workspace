#include <rclcpp/rclcpp.hpp>

class BehaviourManager : public rclcpp::Node
{
public:
    BehaviourManager() : Node("behaviour_manager")
    {
        /***** Parameters *****/
        freq_ = this->declare_parameter<double>("freq", 60.0);

        /***** Timers *****/
        timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / freq_), std::bind(&BehaviourManager::callback_timer, this));
    }

private:
    /***** Variables *****/
    // Node parameters
    double freq_;

    // Node variables
    rclcpp::TimerBase::SharedPtr timer_;

    // Variables

    /***** Functions *****/

    /***** Callbacks *****/
    void callback_timer()
    {
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BehaviourManager>());
    rclcpp::shutdown();
    return 0;
}