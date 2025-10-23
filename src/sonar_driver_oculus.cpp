#include "sonar_driver_oculus/sonar_driver_oculus.h"

SonarDriverOculusNode::SonarDriverOculusNode()
: Node("sonar_oculus"),
   model("M3000d"), frame_id("sonar"), mode(1), pingRate(3), range(10.0), gain(20.0), 
   speedOfSound(0.0), salinity(0.0), gainAssist(false), gammaCorrection(0x7f), netSpeedLimit(0xff) 
{
    // Declare and get parameters with defaults
    this->declare_parameter<std::string>("model", model);
    this->declare_parameter<std::string>("frame_id", frame_id);
    this->declare_parameter<int>("Mode", mode);
    this->declare_parameter<int>("pingRate", pingRate);
    this->declare_parameter<double>("range", range);
    this->declare_parameter<double>("gain", gain);
    this->declare_parameter<double>("speedOfSound", speedOfSound);
    this->declare_parameter<double>("salinity", salinity);
    this->declare_parameter<bool>("gainAssist", gainAssist);
    this->declare_parameter<uint8_t>("gammaCorrection", gammaCorrection);
    this->declare_parameter<uint8_t>("netSpeedLimit", netSpeedLimit);

    // Define callback for dynamic parameter changes
    param_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&SonarDriverOculusNode::reconfigureSonarCallback, this, std::placeholders::_1));

    // Define Oculus Ping publisher
    // ping_pub_ = this->create_publisher<sonar_oculus_interface::msg::OculusPing>(
    //     "/sonar_oculus_node/" + model + "/ping", 10);
    sonar_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/sonar_oculus_node/" + model + "/image", 10);

    sonar_fire_config.mode = mode; 
    sonar_fire_config.pingRate = pingRate; 
    sonar_fire_config.range = range; 
    sonar_fire_config.gain = gain; 
    sonar_fire_config.speedOfSound = speedOfSound; 
    sonar_fire_config.salinity = salinity; 
    sonar_fire_config.gainAssist = gainAssist; 
    sonar_fire_config.gammaCorrection = gammaCorrection; 
    sonar_fire_config.netSpeedLimit = netSpeedLimit; 

    sonar_driver->readPeriod = int(1/pingRate *1e9); 

    // delayed construction of the object 
    RCLCPP_INFO(this->get_logger(), "Creating Sonar Oculus Driver instance...");
    sonar_driver.emplace(IP_ADDR, sonar_fire_config); 

    // Setup timer loop 
    if(sonar_driver->sonarConnected) {
        RCLCPP_INFO(this->get_logger(), "Sonar Oculus node initialized.");

        RCLCPP_INFO(this->get_logger(), "Start Sonar Oculus data collection and Fire Loop");
        RCLCPP_INFO(this->get_logger(), "Sending first Fire request");
        sonar_driver->fireSonar(); 
        sonar_update_timer_ = this->create_wall_timer(
            std::chrono::nanoseconds(sonar_driver->readPeriod), std::bind(&SonarDriverOculusNode::loopSonarCallback, this));
        sonar_fire_timer_ = this->create_wall_timer(
            std::chrono::nanoseconds(sonar_driver->firePeriod), std::bind(&SonarDriverOculusNode::loopFireCallback, this));
    } else { 
        RCLCPP_INFO(this->get_logger(), "Cannot Start Sonar Oculus data collection, Sonar not connected...");
        rclcpp::shutdown(); 
    }

}

SonarDriverOculusNode::~SonarDriverOculusNode()
{   
    RCLCPP_INFO(this->get_logger(), "Disconnecting from Sonar.");
}

rcl_interfaces::msg::SetParametersResult
SonarDriverOculusNode::reconfigureSonarCallback(const std::vector<rclcpp::Parameter> &params)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    // Cast parameters to the correct data type 
    for (const auto &param : params) {
        if (param.get_name() == "mode") {
            mode = param.as_int();
        } else if (param.get_name() == "pingRate") {
            pingRate = param.as_int();
        } else if (param.get_name() == "range") {
            range = param.as_double();
        } else if (param.get_name() == "gain") {
            gain = param.as_double();
        }  else if (param.get_name() == "speedOfSound") {
            speedOfSound = param.as_double();
        }  else if (param.get_name() == "salinity") {
            salinity = param.as_double();
        }  else if (param.get_name() == "gainAssist") {
            gainAssist = param.as_bool();
        } else if (param.get_name() == "gammaCorrection") {
            gammaCorrection = param.as_int();
        } else if (param.get_name() == "netSpeedLimit") {
            netSpeedLimit = param.as_int();
        }
    }

    // Update Sonar refresh rate at runtime based on the ping rate request 
    if(pingRate != sonar_fire_config.pingRate){
        sonar_driver->readPeriod = int(1/pingRate * 1e9);
        sonar_update_timer_->cancel();
        sonar_update_timer_ = this->create_wall_timer(
            std::chrono::nanoseconds(sonar_driver->readPeriod), 
            std::bind(&SonarDriverOculusNode::loopSonarCallback, this)); 
    }

    sonar_fire_config.mode = mode; 
    sonar_fire_config.pingRate = pingRate; 
    sonar_fire_config.range = range; 
    sonar_fire_config.gain = gain; 
    sonar_fire_config.speedOfSound = speedOfSound; 
    sonar_fire_config.salinity = salinity; 
    sonar_fire_config.gainAssist = gainAssist; 
    sonar_fire_config.gammaCorrection = gammaCorrection; 
    sonar_fire_config.netSpeedLimit = netSpeedLimit; 

    sonar_driver->initializeSonar(sonar_fire_config);

    return result;
}

void SonarDriverOculusNode::loopSonarCallback()
{
    // Update image from the driver API 
    sonar_driver->updateImage(); 
    // Check if No image still received 
    if (sonar_driver->sonarImage == nullptr){
        RCLCPP_INFO(this->get_logger(), "No Image yet received");
        return; 
    } else { // Republish sonar image at the publish rate 
        sonar_image_msg.header.frame_id = frame_id;
        sonar_image_msg.header.stamp = this->get_clock()->now();

        sonar_image_msg.height = sonar_driver->sonarImageHeight;
        sonar_image_msg.width = sonar_driver->sonarImageWidth;
        sonar_image_msg.encoding = "8UC1";
        sonar_image_msg.is_bigendian = 0;
        sonar_image_msg.step = sonar_driver->sonarImageWidth;

        sonar_image_msg.data.resize(sonar_driver->sonarImageSize); 
        memcpy(sonar_image_msg.data.data(), sonar_driver->sonarImage, sonar_driver->sonarImageSize);

        sonar_image_pub_->publish(sonar_image_msg); 

    }

}

void SonarDriverOculusNode::loopFireCallback()
{
    RCLCPP_INFO(this->get_logger(), "New Fire request sent");
    sonar_driver->fireSonar(); 
}   



// Main program for listening to sonar
int main(int argc, char **argv) 
{
  rclcpp::init(argc, argv);
  auto sonar_driver_oculus_node = std::make_shared<SonarDriverOculusNode>();
  if(rclcpp::ok()){
    rclcpp::spin(sonar_driver_oculus_node);

    rclcpp::shutdown();
  }

  return 0;
} 
