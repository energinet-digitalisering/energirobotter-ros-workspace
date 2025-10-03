#include <rclcpp/rclcpp.hpp>
#include "kinematics_manager/trac_ik_manager.hpp"

#include <fstream>
#include <iostream>
#include <mutex>
#include <future>

class ReachabilityMapGenerator : public rclcpp::Node
{
public:
    ReachabilityMapGenerator() : Node("reachability_map_generator")
    {
        base_link_ = this->declare_parameter<std::string>("base_link", "link_torso");
        tip_link_ = this->declare_parameter<std::string>("tip_link", "link_left_hand");
        step_size_ = this->declare_parameter<double>("step_size", 0.02); // 2 cm resolution
    }

    void generate_point_cloud()
    {
        // Set desired orientation
        // KDL::Rotation orientation = KDL::Rotation::Quaternion(-0.022, 0.707, -0.022, 0.706); // Left handback towards +z
        KDL::Rotation orientation = KDL::Rotation::Quaternion(0.0, 0.0, 0.0, 1.0); // Left handback towards -x

        // Workspace definition (meters)
        double xmin = -1.0, xmax = 1.0;
        double ymin = 0.0, ymax = 1.0;
        double zmin = -0.5, zmax = 0.5;

        size_t nx = static_cast<size_t>((xmax - xmin) / step_size_) + 1;
        size_t ny = static_cast<size_t>((ymax - ymin) / step_size_) + 1;
        size_t nz = static_cast<size_t>((zmax - zmin) / step_size_) + 1;
        total_points_ = nx * ny * nz;

        RCLCPP_INFO(this->get_logger(), "Sampling a grid of %zu points.", total_points_);

        // Start a progress monitor thread
        processed_points_ = 0; // Reset counter
        std::atomic<bool> running{true};
        std::thread progress_thread([this, &running]()
                                    {
            while (running) {
                double progress = 100.0 * static_cast<double>(processed_points_) / total_points_;
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                     "Progress: %.1f%% (%zu/%zu)", progress, processed_points_.load(), total_points_);
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            } });

        // Split X axis into 10 chunks
        int num_chunks = 10;
        double x_range = xmax - xmin;
        double chunk_size = x_range / num_chunks;

        std::vector<std::future<std::vector<Eigen::Vector3f>>> futures;

        for (int chunk = 0; chunk < num_chunks; ++chunk)
        {
            double x_start = xmin + chunk * chunk_size;
            double x_end = (chunk == num_chunks - 1) ? xmax : (xmin + (chunk + 1) * chunk_size);

            static std::mutex ik_mutex;

            // Each task creates and uses its own TracIKManager instance (thread-local)
            futures.push_back(std::async(std::launch::async, [=]()
                                         {
                std::vector<Eigen::Vector3f> local_points;
                
                // Build a thread-local TRAC-IK manager
                auto local_ik = create_ik_manager_with_mutex();
                
                if (!local_ik->initialize())
                {
                    // initialization failed for this thread — return empty result
                    RCLCPP_WARN(this->get_logger(), "Thread TRAC-IK initialize failed for chunk (%.3f..%.3f).", x_start, x_end);
                    return local_points;
                }

                for (double x = x_start; x <= x_end && rclcpp::ok(); x += step_size_)
                {
                    for (double y = ymin; y <= ymax && rclcpp::ok(); y += step_size_)
                    {
                        for (double z = zmin; z <= zmax && rclcpp::ok(); z += step_size_)
                        {
                            KDL::Frame pose(orientation, KDL::Vector(x, y, z));
                            KDL::JntArray q_out;
                            
                            // Use the thread-local solver
                            if (local_ik->compute_ik(pose, q_out))
                            {
                                local_points.emplace_back(x, y, z);
                            }
                            processed_points_++;
                        }
                    }
                }

                return local_points; }));
        }

        // Collect results (this blocks until all tasks complete)
        for (auto &f : futures)
        {
            auto chunk_points = f.get();
            points_.insert(points_.end(), chunk_points.begin(), chunk_points.end());
        }

        running = false;
        progress_thread.join();

        save_pointcloud();
    }

private:
    // Node parameters
    std::string base_link_;
    std::string tip_link_;
    double step_size_;

    // Member variables
    std::vector<Eigen::Vector3f> points_;
    size_t total_points_;
    std::atomic<size_t> processed_points_;

    std::shared_ptr<TracIKManager> create_ik_manager_with_mutex()
    {
        static std::mutex ik_mutex;
        std::lock_guard<std::mutex> lock(ik_mutex);
        return std::make_shared<TracIKManager>(
            this->shared_from_this(),
            base_link_,
            tip_link_,
            "robot_description",
            KDL::Vector{-0.2, 0.5, 0.0},
            0.001,
            1e-5,
            false,
            TRAC_IK::SolveType::Distance);
    }

    void save_pointcloud()
    {
        // Write PLY header
        std::ofstream outfile("pointcloud.ply");
        outfile << "ply\nformat ascii 1.0\n";
        outfile << "element vertex " << points_.size() << "\n";
        outfile << "property float x\nproperty float y\nproperty float z\n";
        outfile << "end_header\n";

        // Write the points
        for (const auto &p : points_)
        {
            outfile << p.x() << " " << p.y() << " " << p.z() << "\n";
        }

        outfile.close();

        RCLCPP_INFO(this->get_logger(),
                    "Reachability map written with %zu reachable points (out of %zu sampled).",
                    points_.size(), total_points_);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ReachabilityMapGenerator>();

    node->generate_point_cloud();

    rclcpp::shutdown();
    return 0;
}
