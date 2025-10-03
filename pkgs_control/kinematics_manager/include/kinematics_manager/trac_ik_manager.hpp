#pragma once

#include <vector>
#include <string>

#include <trac_ik/trac_ik.hpp>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>

class TracIKManager
{
public:
    TracIKManager(
        rclcpp::Node::SharedPtr node,
        const std::string &base_link,
        const std::string &tip_link,
        const std::string &urdf_param,
        KDL::Vector reachability_space_center = KDL::Vector{0.0, 0.0, 0.0},
        double timeout = 0.001,
        double eps = 1e-5,
        bool allow_approximation = false,
        TRAC_IK::SolveType solve_type = TRAC_IK::SolveType::Distance);

    // Initialize the solver and fetch KDL chain/limits
    bool initialize();

    // Compute IK for a given pose
    bool compute_ik(KDL::JntArray &q_out);
    bool compute_ik(const KDL::Frame &pose, KDL::JntArray &q_out);

    // Get joint names
    const std::vector<std::string> &get_joint_names() const { return joint_names_; }

    // Get joint limits
    const KDL::JntArray &get_min_limits() const { return min_limits_; }
    const KDL::JntArray &get_max_limits() const { return max_limits_; }

    // Set target pose
    void set_target_pose(const KDL::Frame &target_pose) { target_pose_ = target_pose; }

private:
    bool compute_ik_internal(const KDL::Frame &pose, KDL::JntArray &q_out);

    // Bisection method for approximating IK solution when out of bounds
    KDL::JntArray find_boundary_bisection(
        const KDL::JntArray &nominal,
        const KDL::Frame &target_out_of_bounds,
        KDL::JntArray &q_out);

    KDL::Frame target_pose_;

    rclcpp::Node::SharedPtr node_;
    std::unique_ptr<TRAC_IK::TRAC_IK> solver_;
    KDL::JntArray min_limits_;
    KDL::JntArray max_limits_;
    KDL::JntArray q_last_valid_;
    std::vector<std::string> joint_names_;
    KDL::Vector reachability_space_center_;
    double eps_;
    bool allow_approximation_;
};
