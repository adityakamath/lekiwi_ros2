/**
 * @file estop_node.cpp
 * @brief Emergency stop relay node
 *
 * Subscribes to estop_button (Bool) and publishes to emergency_stop (Bool).
 * Use topic remapping to override default topic names if needed.
 * Provides configurable relay modes for triggering emergency stops from various sources
 * (joystick buttons, safety switches, user interfaces, etc.).
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

/**
 * @class EstopNode
 * @brief Node for relaying emergency stop signals
 *
 * Subscribes to estop_button and publishes to emergency_stop. Use topic remapping
 * to override defaults if needed. Supports two modes (configured via 'mode' parameter):
 * - "toggle": Push button mode - each button press toggles emergency stop state
 * - "direct": Switch mode - emergency stop directly follows input state
 */
class EstopNode : public rclcpp::Node {
public:
  /**
   * @brief Constructor - initializes parameters and creates pub/sub
   */
  EstopNode() : Node("estop_node"), last_input_state_(false),
                emergency_stop_state_(false) {
    // Declare and get mode parameter
    this->declare_parameter("mode", "toggle");  // "toggle" or "direct"
    mode_ = this->get_parameter("mode").as_string();

    // Validate mode parameter
    if (mode_ != "toggle" && mode_ != "direct") {
      RCLCPP_ERROR(this->get_logger(),
                   "Invalid mode '%s'. Must be 'toggle' or 'direct'. Defaulting to 'toggle'.",
                   mode_.c_str());
      mode_ = "toggle";
    }

    // Create subscription to input topic (default: estop_button, use remapping to override)
    estop_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
        "estop_button", 10,
        std::bind(&EstopNode::estop_callback, this, std::placeholders::_1));

    // Create publisher for emergency stop (default: emergency_stop, use remapping to override)
    emergency_stop_pub_ = this->create_publisher<std_msgs::msg::Bool>(
        "emergency_stop", 10);

    RCLCPP_INFO(this->get_logger(),
                "Emergency stop relay node started (mode: %s)", mode_.c_str());
  }

private:
  std::string mode_;  ///< Operating mode: "toggle" or "direct"
  bool last_input_state_;  ///< Previous input state to detect transitions
  bool emergency_stop_state_;  ///< Current emergency stop state

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_subscription_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr emergency_stop_pub_;

  /**
   * @brief Process emergency stop input and relay to output
   * @param msg Bool message from input topic
   */
  void estop_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    bool current_input = msg->data;
    bool new_emergency_stop_state = emergency_stop_state_;

    if (mode_ == "toggle") {
      // Toggle mode: detect rising edge (0->1 transition) and toggle state
      if (current_input && !last_input_state_) {
        new_emergency_stop_state = !emergency_stop_state_;
      }
    } else {  // mode_ == "direct"
      // Direct mode: emergency stop follows input state directly
      new_emergency_stop_state = current_input;
    }

    // Only publish and log if emergency stop state changed
    if (new_emergency_stop_state != emergency_stop_state_) {
      emergency_stop_state_ = new_emergency_stop_state;

      auto output_msg = std_msgs::msg::Bool();
      output_msg.data = emergency_stop_state_;
      emergency_stop_pub_->publish(output_msg);

      if (emergency_stop_state_) {
        RCLCPP_WARN(this->get_logger(), "EMERGENCY STOP ACTIVATED");
      } else {
        RCLCPP_INFO(this->get_logger(), "Emergency stop released");
      }
    }

    // Update input state for next iteration
    last_input_state_ = current_input;
  }
};

/**
 * @brief Main entry point for estop relay node
 * @param argc Argument count
 * @param argv Argument values
 * @return Exit status
 */
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EstopNode>());
  rclcpp::shutdown();
  return 0;
}
