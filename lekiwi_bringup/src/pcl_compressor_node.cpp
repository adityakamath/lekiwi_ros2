#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <point_cloud_interfaces/msg/compressed_point_cloud2.hpp>
#include <cloudini_lib/cloudini.hpp>
#include <cloudini_ros/conversion_utils.hpp>

namespace lekiwi_bringup {

class PCLCompressorNode : public rclcpp::Node {
public:
  PCLCompressorNode(const rclcpp::NodeOptions& options)
      : Node("pcl_compressor", options) {
    
    // Declare parameters
    this->declare_parameter("compressing", true);
    this->declare_parameter("topic_input", "/oak/rgbd/points");
    this->declare_parameter("topic_output", "/oak/rgbd/points/compressed");
    this->declare_parameter("resolution", 0.001);  // 1mm for XYZ
    
    // Get parameters
    compressing_ = this->get_parameter("compressing").as_bool();
    topic_input_ = this->get_parameter("topic_input").as_string();
    topic_output_ = this->get_parameter("topic_output").as_string();
    resolution_ = this->get_parameter("resolution").as_double();
    
    if (!compressing_) {
      RCLCPP_INFO(this->get_logger(), "Compression disabled");
      return;
    }
    
    if (resolution_ <= 0.0) {
      RCLCPP_ERROR(this->get_logger(), "Invalid resolution: %.4f", resolution_);
      return;
    }
    
    RCLCPP_INFO(this->get_logger(), 
                "Compressing %s → %s (resolution: %.4f m, RGB preserved)",
                topic_input_.c_str(), topic_output_.c_str(), resolution_);
    
    // Use SENSOR_DATA QoS for real-time performance
    auto qos = rclcpp::SensorDataQoS();
    
    // Create subscriber and publisher
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        topic_input_, qos,
        std::bind(&PCLCompressorNode::pointCloudCallback, this, std::placeholders::_1));
    
    pub_ = this->create_publisher<point_cloud_interfaces::msg::CompressedPointCloud2>(
        topic_output_, qos);
  }

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    try {
      // Convert to encoding info
      auto info = Cloudini::ConvertToEncodingInfo(*msg, resolution_);
      
      // Fix: Exclude RGB/RGBA fields from lossy quantization
      for (auto& field : info.fields) {
        if (field.name == "rgb" || field.name == "rgba" || 
            field.name == "bgr" || field.name == "bgra") {
          // Preserve color data by not applying resolution
          field.resolution = std::nullopt;
          RCLCPP_DEBUG(this->get_logger(), "Preserving color field: %s", field.name.c_str());
        }
      }
      
      // Encode using cloudini
      Cloudini::PointcloudEncoder encoder(info);
      
      // Prepare output message
      point_cloud_interfaces::msg::CompressedPointCloud2 result;
      result.header = msg->header;
      result.width = msg->width;
      result.height = msg->height;
      result.fields = msg->fields;
      result.is_bigendian = false;
      result.point_step = msg->point_step;
      result.row_step = msg->row_step;
      result.is_dense = msg->is_dense;
      result.format = "cloudini";
      
      // Reserve and encode
      result.compressed_data.resize(msg->data.size());
      Cloudini::ConstBufferView input(msg->data.data(), msg->data.size());
      auto compressed_size = encoder.encode(input, result.compressed_data);
      result.compressed_data.resize(compressed_size);
      
      // Publish
      pub_->publish(result);
      
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Compression failed: %s", e.what());
    }
  }
  
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<point_cloud_interfaces::msg::CompressedPointCloud2>::SharedPtr pub_;
  
  bool compressing_;
  std::string topic_input_;
  std::string topic_output_;
  double resolution_;
};

}  // namespace lekiwi_bringup

RCLCPP_COMPONENTS_REGISTER_NODE(lekiwi_bringup::PCLCompressorNode)
