#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using std::placeholders::_1;

class ImageRotateNode : public rclcpp::Node
{
public:
    ImageRotateNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : rclcpp::Node("image_rotate_node", options)
    {
        // Parameters
        rotation_deg_ = declare_parameter<double>("rotation", 0.0);
        use_compressed_ = declare_parameter<bool>("use_compressed", false);

        // Pre-map rotation codes
        rotation_map_[90] = cv::ROTATE_90_CLOCKWISE;
        rotation_map_[180] = cv::ROTATE_180;
        rotation_map_[270] = cv::ROTATE_90_COUNTERCLOCKWISE;

        // QoS tuned for image streams
        auto qos = rclcpp::SensorDataQoS().keep_last(1).best_effort();

        if (use_compressed_)
        {
            sub_compressed_ = create_subscription<sensor_msgs::msg::CompressedImage>(
                "/image", qos, std::bind(&ImageRotateNode::compressedCb, this, _1));
            pub_compressed_ = create_publisher<sensor_msgs::msg::CompressedImage>(
                "/image_rotated", 1);
            RCLCPP_INFO(get_logger(), "Using CompressedImage I/O");
        }
        else
        {
            sub_raw_ = create_subscription<sensor_msgs::msg::Image>(
                "/image", qos, std::bind(&ImageRotateNode::rawCb, this, _1));
            pub_raw_ = create_publisher<sensor_msgs::msg::Image>("/image_rotated", 1);
            RCLCPP_INFO(get_logger(), "Using raw Image I/O");
        }
    }

private:
    // RAW image path
    void rawCb(const sensor_msgs::msg::Image::ConstSharedPtr msg)
    {
        try
        {
            cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
            const cv::Mat &frame = cv_ptr->image;

            cv::Mat rotated;
            if (!rotateIfNeeded(frame, rotated))
            {
                // 0° or unsupported angle -> forward as-is (zero-copy when possible)
                pub_raw_->publish(*msg);
                return;
            }

            // Wrap and publish
            cv_bridge::CvImage out;
            out.header = msg->header;
            out.encoding = "bgr8";
            out.image = rotated;
            pub_raw_->publish(*out.toImageMsg());
        }
        catch (const std::exception &e)
        {
            RCLCPP_WARN(get_logger(), "rawCb exception: %s", e.what());
        }
    }

    // Compressed image path
    void compressedCb(const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg)
    {
        try
        {
            // Decode to BGR
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(*msg, "bgr8");

            cv::Mat rotated;
            if (!rotateIfNeeded(cv_ptr->image, rotated))
            {
                // No rotation needed: forward original compressed buffer
                pub_compressed_->publish(*msg);
                return;
            }

            // Re-encode (defaults to JPEG)
            cv_bridge::CvImage out;
            out.header = msg->header;
            out.encoding = "bgr8";
            out.image = rotated;

            sensor_msgs::msg::CompressedImage::SharedPtr out_msg = out.toCompressedImageMsg();
            out_msg->header = msg->header; // preserve header

            pub_compressed_->publish(*out_msg);
        }
        catch (const std::exception &e)
        {
            RCLCPP_WARN(get_logger(), "compressedCb exception: %s", e.what());
        }
    }

    // Return true if rotation performed, false if passthrough
    bool rotateIfNeeded(const cv::Mat &in, cv::Mat &out)
    {
        int rot_key = static_cast<int>(std::round(rotation_deg_)) % 360;
        if (rot_key < 0)
            rot_key += 360;

        auto it = rotation_map_.find(rot_key);
        if (it == rotation_map_.end() || rot_key == 0)
        {
            return false; // passthrough
        }
        cv::rotate(in, out, it->second);
        return true;
    }

private:
    // Params
    double rotation_deg_{0.0};
    bool use_compressed_{false};
    std::unordered_map<int, int> rotation_map_;

    // ROS I/O
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_raw_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_raw_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_compressed_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_compressed_;
};

// Support intra-process for zero-copy when raw images are used within same process
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions opts;
    opts.use_intra_process_comms(true);
    auto node = std::make_shared<ImageRotateNode>(opts);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
