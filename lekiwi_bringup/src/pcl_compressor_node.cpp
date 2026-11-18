#include <cmath>
#include <cstdint>
#include <cstring>
#include <unordered_set>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <point_cloud_interfaces/msg/compressed_point_cloud2.hpp>
#include <cloudini_lib/cloudini.hpp>
#include <cloudini_ros/conversion_utils.hpp>

namespace lekiwi_bringup {

// Compresses a colored PointCloud2 with Cloudini so it can be streamed to Foxglove.
class PCLCompressorNode : public rclcpp::Node {
public:
  explicit PCLCompressorNode(const rclcpp::NodeOptions& options)
      : Node("pcl_compressor", options) {
    declare_parameter("compressing", true);
    declare_parameter("topic_input", "/gemini2/depth_registered/points");
    declare_parameter("topic_output", "/gemini2/depth_registered/points/compressed");
    declare_parameter("resolution", 0.001);  // 1 mm for XYZ
    declare_parameter("filter_points", true);  // drop invalid points and duplicates within a voxel

    compressing_ = get_parameter("compressing").as_bool();
    topic_input_ = get_parameter("topic_input").as_string();
    topic_output_ = get_parameter("topic_output").as_string();
    resolution_ = get_parameter("resolution").as_double();
    filter_points_ = get_parameter("filter_points").as_bool();

    if (!compressing_) {
      RCLCPP_INFO(get_logger(), "Compression disabled");
      return;
    }
    if (resolution_ <= 0.0) {
      RCLCPP_ERROR(get_logger(), "Invalid resolution: %.4f", resolution_);
      return;
    }

    RCLCPP_INFO(get_logger(), "Compressing %s -> %s (resolution: %.4f m, RGB preserved)",
                topic_input_.c_str(), topic_output_.c_str(), resolution_);

    auto qos = rclcpp::SensorDataQoS();
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        topic_input_, qos,
        std::bind(&PCLCompressorNode::pointCloudCallback, this, std::placeholders::_1));
    pub_ = create_publisher<point_cloud_interfaces::msg::CompressedPointCloud2>(topic_output_, qos);
  }

private:
  // Drops points with non-finite or zero XYZ, then keeps one point per voxel; false if XYZ is not float32
  bool filterPoints(const sensor_msgs::msg::PointCloud2& msg, std::vector<uint8_t>& out) {
    int32_t off[3] = {-1, -1, -1};
    for (const auto& f : msg.fields) {
      if (f.datatype != sensor_msgs::msg::PointField::FLOAT32) continue;
      if (f.name == "x") off[0] = f.offset;
      if (f.name == "y") off[1] = f.offset;
      if (f.name == "z") off[2] = f.offset;
    }
    if (off[0] < 0 || off[1] < 0 || off[2] < 0) return false;

    const size_t step = msg.point_step;
    const size_t count = msg.data.size() / step;
    out.clear();
    out.reserve(msg.data.size());
    seen_.clear();
    seen_.reserve(count);
    const double inv_res = 1.0 / resolution_;
    constexpr int64_t kMask = (1 << 21) - 1;

    for (size_t i = 0; i < count; ++i) {
      const uint8_t* point = msg.data.data() + i * step;
      float xyz[3];
      std::memcpy(&xyz[0], point + off[0], sizeof(float));
      std::memcpy(&xyz[1], point + off[1], sizeof(float));
      std::memcpy(&xyz[2], point + off[2], sizeof(float));
      if (!std::isfinite(xyz[0]) || !std::isfinite(xyz[1]) || !std::isfinite(xyz[2])) continue;
      if (xyz[0] == 0.0f && xyz[1] == 0.0f && xyz[2] == 0.0f) continue;

      const uint64_t key =
          (static_cast<uint64_t>(std::llround(xyz[0] * inv_res) & kMask) << 42) |
          (static_cast<uint64_t>(std::llround(xyz[1] * inv_res) & kMask) << 21) |
          static_cast<uint64_t>(std::llround(xyz[2] * inv_res) & kMask);
      if (!seen_.insert(key).second) continue;

      out.insert(out.end(), point, point + step);
    }
    return true;
  }

  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    try {
      auto info = Cloudini::ConvertToEncodingInfo(*msg, resolution_);

      // Packed color is a float32 field: quantizing it would corrupt the colors
      for (auto& field : info.fields) {
        if (field.name == "rgb" || field.name == "rgba" ||
            field.name == "bgr" || field.name == "bgra") {
          field.resolution = std::nullopt;
        }
      }

      std::vector<uint8_t> filtered;
      const uint8_t* data = msg->data.data();
      size_t data_size = msg->data.size();
      uint32_t width = msg->width;
      uint32_t height = msg->height;
      if (filter_points_ && filterPoints(*msg, filtered)) {
        data = filtered.data();
        data_size = filtered.size();
        width = static_cast<uint32_t>(data_size / msg->point_step);
        height = 1;
        info.width = width;
        info.height = height;
      }

      Cloudini::PointcloudEncoder encoder(info);

      point_cloud_interfaces::msg::CompressedPointCloud2 result;
      result.header = msg->header;
      result.width = width;
      result.height = height;
      result.fields = msg->fields;
      result.is_bigendian = false;
      result.point_step = msg->point_step;
      result.row_step = width * msg->point_step;
      result.is_dense = filter_points_ ? true : msg->is_dense;
      result.format = "cloudini";

      result.compressed_data.resize(data_size);
      Cloudini::ConstBufferView input(data, data_size);
      auto compressed_size = encoder.encode(input, result.compressed_data);
      result.compressed_data.resize(compressed_size);

      pub_->publish(result);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Compression failed: %s", e.what());
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<point_cloud_interfaces::msg::CompressedPointCloud2>::SharedPtr pub_;

  std::unordered_set<uint64_t> seen_;
  bool compressing_;
  bool filter_points_;
  std::string topic_input_;
  std::string topic_output_;
  double resolution_;
};

}  // namespace lekiwi_bringup

RCLCPP_COMPONENTS_REGISTER_NODE(lekiwi_bringup::PCLCompressorNode)
