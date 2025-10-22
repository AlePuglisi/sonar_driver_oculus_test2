#include "sonar_driver_oculus/sonar_driver_oculus.h"

SonarDriverOculusNode::SonarDriverOculusNode()
: Node("sonar_oculus"),
   model("M3000d"), frame_id("sonar"), mode(1), ping_rate(3), range(10.0), gain(20.0), soundspeed(0.0), salinity(0.0)
{
    // Declare and get parameters with defaults
    this->declare_parameter<std::string>("model", model);
    this->declare_parameter<std::string>("frame_id", frame_id);
    this->declare_parameter<int>("Mode", mode);
    this->declare_parameter<int>("PingRate", ping_rate);
    this->declare_parameter<double>("Range", range);
    this->declare_parameter<double>("Gain", gain);
    this->declare_parameter<double>("Salinity", salinity);

    // Define callback for dynamic parameter changes
    param_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&SonarOculusNode::reconfigureSonarCallback, this, std::placeholders::_1));

    // Define Oculus Ping publisher
    ping_pub_ = this->create_publisher<sonar_oculus_interface::msg::OculusPing>(
        "/sonar_oculus_node/" + model + "/ping", 10);

    // Initialize communication: UDP Network Search (If ip unknown) 
    searchSonar();
    // Connect to Sonar, based on previous search
    connectToSonar();

    RCLCPP_INFO(this->get_logger(), "Sonar Oculus node initialized.");
    RCLCPP_INFO(this->get_logger(), "Starting read loop.");

    // Setup timer loop (50 Hz here)
    sonar_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20), std::bind(&SonarOculusNode::publishPing, this));

}

SonarDriverOculusNode::~SonarDriverOculusNode()
{
    // Disconnect from sonar
    sonar.Disconnect();
    // close TCP connection 
    close(sockTCP);
    
    RCLCPP_INFO(this->get_logger(), "Disconnecting from Sonar.");
}

rcl_interfaces::msg::SetParametersResult
SonarOculusNode::reconfigureSonarCallback(const std::vector<rclcpp::Parameter> &params)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    // Cast parameters to the correct data type 
    for (const auto &param : params) {
        if (param.get_name() == "Mode") {
            mode = param.as_int();
        } else if (param.get_name() == "PingRate") {
            ping_rate = param.as_int();
        } else if (param.get_name() == "Gain") {
            gain = param.as_double();
        } else if (param.get_name() == "Range") {
            range = param.as_double();
        } else if (param.get_name() == "Salinity") {
            salinity = param.as_double();
        }
    }

    // Clamb HW configuration based on the Sonar Oculus Model 
    // Specifications here: https://www.blueprintsubsea.com/oculus/oculus-m-series
    if (partNumber == OculusPartNumberType::partNumberM750d) {
        if (mode == 1)
            range = std::max(0.1, std::min(range, 120.0));
        else if (mode == 2)
            range = std::max(0.1, std::min(range, 40.0));
    }
    if (partNumber == OculusPartNumberType::partNumberM1200d) {
        range = std::max(0.1, std::min(range, 30.0));
    }
    if (partNumber == OculusPartNumberType::partNumberM3000d) {
        if (mode == 1) {
            range = std::max(0.1, std::min(range, 30.0));
        }
        else if (mode == 2) {
            range = std::max(0.1, std::min(range, 5.0));
        }
    }

    sonar.Fire(mode, ping_rate, range, gain, soundspeed, salinity);

    RCLCPP_INFO(this->get_logger(), 
        "Reconfigured params -> Mode:%d PingRate:%d Range:%.2f Gain:%.2f Salinity:%.2f",
        mode, ping_rate, range, gain, salinity);

    return result;
}

// Main program for listening to sonar
int main(int argc, char **argv) 
{
  rclcpp::init(argc, argv);
  auto sonar_driver_oculus_node = std::make_shared<SonarDriverOculusNode>();
  rclcpp::spin(sonar_driver_oculus_node);

  rclcpp::shutdown();

  return 0;
} 
