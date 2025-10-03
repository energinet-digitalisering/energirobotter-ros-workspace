#include <memory>
#include <vector>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "kinematics_manager/trac_ik_manager.hpp"

class TracIKNode : public rclcpp::Node
{
public:
  TracIKNode() : Node("trac_ik_node") {}

  void init()
  {
    // Parameters
    base_link_ = this->declare_parameter<std::string>("base_link", "link_torso");
    double publish_rate = this->declare_parameter<double>("publish_rate", 30.0);

    // Create solvers and setup for each end-effector
    create_solver("left", "link_left_hand", KDL::Vector{-0.2, 0.5, 0.0});
    create_solver("right", "link_right_hand", KDL::Vector{0.2, 0.5, 0.0});

    // Publishers
    pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

    // Timers
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / publish_rate),
        std::bind(&TracIKNode::publish_ik_solution, this));
  }

private:
  KDL::Frame ros_pose_to_kdl_frame(const geometry_msgs::msg::Pose &pose)
  {
    KDL::Vector pose_position(
        pose.position.x,
        pose.position.y,
        pose.position.z);

    KDL::Rotation pose_rotation = KDL::Rotation::Quaternion(
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w);

    return KDL::Frame(pose_rotation, pose_position);
  }

  void create_solver(
      std::string key,
      std::string tip_link,
      KDL::Vector reachability_space_center,
      bool allow_approximation = true)
  {
    auto ik_manager = std::make_unique<TracIKManager>(
        this->shared_from_this(),
        base_link_,
        tip_link,
        "robot_description",
        reachability_space_center,
        0.001,
        1e-5,
        allow_approximation,
        TRAC_IK::SolveType::Distance);

    if (!ik_manager->initialize())
    {
      RCLCPP_FATAL(this->get_logger(), "Failed to initialize TRAC-IK for %s", tip_link.c_str());
      return;
    }

    ik_managers_[key] = std::move(ik_manager);

    // Create subscription for each end-effector
    auto sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/" + key + "/target_pose", 10,
        [this, key](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
          ik_managers_[key]->set_target_pose(ros_pose_to_kdl_frame(msg->pose));
        });

    subscriptions_[key] = sub;
  }

  void publish_ik_solution()
  {
    sensor_msgs::msg::JointState joint_state_msg;
    joint_state_msg.header.stamp = this->now();

    for (const auto &[key, ik_manager] : ik_managers_)
    {
      KDL::JntArray q_out;
      bool success = ik_manager->compute_ik(q_out);

      if (success)
      {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "IK solution found for %s", key.c_str());
      }
      else
      {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "No IK solution for %s", key.c_str());
      }

      // Append joint names and positions with key prefix to avoid collisions
      const auto &joint_names = ik_manager->get_joint_names();
      for (unsigned int i = 0; i < q_out.rows(); i++)
      {
        joint_state_msg.name.push_back(joint_names[i]);
        joint_state_msg.position.push_back(q_out(i));
      }
    }

    if (!joint_state_msg.name.empty())
    {
      pub_->publish(joint_state_msg);
    }
    else
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "No joint states to publish");
    }
  }

  std::map<std::string, std::unique_ptr<TracIKManager>> ik_managers_;
  std::map<std::string, rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> subscriptions_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string base_link_;
  std::vector<std::string> tip_links_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<TracIKNode>();
  node->init();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
