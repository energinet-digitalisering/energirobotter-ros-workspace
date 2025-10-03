#include "kinematics_manager/trac_ik_manager.hpp"

TracIKManager::TracIKManager(
    rclcpp::Node::SharedPtr node,
    const std::string &base_link,
    const std::string &tip_link,
    const std::string &urdf_param,
    KDL::Vector reachability_space_center,
    double timeout,
    double eps,
    bool allow_approximation,
    TRAC_IK::SolveType solve_type)
    : node_(node), reachability_space_center_(reachability_space_center), eps_(eps), allow_approximation_(allow_approximation), target_pose_(reachability_space_center)
{
    solver_ = std::make_unique<TRAC_IK::TRAC_IK>(
        node_, base_link, tip_link, urdf_param, timeout, eps, solve_type);
}

bool TracIKManager::initialize()
{
    KDL::Chain chain;
    if (!solver_->getKDLChain(chain))
    {
        RCLCPP_FATAL(node_->get_logger(), "Could not fetch KDL chain");
        return false;
    }

    if (!solver_->getKDLLimits(min_limits_, max_limits_))
    {
        RCLCPP_FATAL(node_->get_logger(), "Could not fetch KDL joint limits");
        return false;
    }

    joint_names_.resize(chain.getNrOfJoints());
    q_last_valid_.resize(chain.getNrOfJoints());

    // Initialize joint names and set initial joint positions to 0
    for (size_t i = 0; i < chain.getNrOfJoints(); i++)
    {
        joint_names_[i] = chain.getSegment(i).getJoint().getName();
        q_last_valid_(i) = 0.0;
    }

    RCLCPP_INFO(node_->get_logger(), "TRAC-IK initialized with %d joints", chain.getNrOfJoints());
    return true;
}

bool TracIKManager::compute_ik(KDL::JntArray &q_out)
{
    return compute_ik_internal(target_pose_, q_out);
}

bool TracIKManager::compute_ik(const KDL::Frame &target_pose, KDL::JntArray &q_out)
{
    return compute_ik_internal(target_pose, q_out);
}

bool TracIKManager::compute_ik_internal(const KDL::Frame &target_pose, KDL::JntArray &q_out)
{
    q_out.resize(joint_names_.size());

    // Use last valid solution as nominal
    KDL::JntArray nominal(q_out.rows());
    for (unsigned int i = 0; i < nominal.rows(); i++)
        nominal(i) = q_last_valid_(i);

    int rc = solver_->CartToJnt(nominal, target_pose, q_out);

    if (rc < 0)
    {
        if (allow_approximation_)
        {
            find_boundary_bisection(nominal, target_pose, q_out);
            RCLCPP_INFO_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 1000,
                "IK solution approximated");
            q_last_valid_ = q_out; // store fallback solution
            return true;           // success via approximation
        }
        else
        {
            return false; // no solution and approximation disabled
        }
    }

    q_last_valid_ = q_out; // Update last valid solution
    return true;           // direct IK success
}

KDL::JntArray TracIKManager::find_boundary_bisection(
    const KDL::JntArray &nominal,
    const KDL::Frame &target_out_of_bounds,
    KDL::JntArray &q_out)
{
    KDL::Vector p_low = reachability_space_center_; // Last inside point
    KDL::Vector p_high = target_out_of_bounds.p;    // First outside point
    KDL::Rotation target_rot = target_out_of_bounds.M;

    // Bisection loop
    while ((p_high - p_low).Norm() > eps_)
    {
        KDL::Vector p_mid = 0.5 * (p_low + p_high);
        KDL::Frame target(target_rot, p_mid);

        int rc = solver_->CartToJnt(nominal, target, q_out);

        if (rc >= 0)
        {
            p_low = p_mid; // Move low up
        }
        else
        {
            p_high = p_mid; // Move high down
        }
    }
    return q_out;
}