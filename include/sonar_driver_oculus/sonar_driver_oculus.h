#ifndef SONAR_DRIVER_OCULUS__SONAR_DRIVER_OCULUS_HPP_
#define SONAR_DRIVER_OCULUS__SONAR_DRIVER_OCULUS_HPP_

#include "OsDriver.h"
#include "OsClientCtrl.h"

//opencv
//#include <opencv2/cv.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include "opencv2/highgui/highgui.hpp"

// ROS includes
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
// #include <sonar_oculus_interface/msg/oculus_fire.hpp>
// #include <sonar_oculus_interface/msg/oculus_ping.hpp>

using namespace cv;

class SonarDriverOculusNode : public rclcpp::Node 
{
   public: 
      SonarDriverOculusNode();
      ~SonarDriverOculusNode();
    
   private:
      // --- Callbacks and Methods ---

      /**
       * @brief This callback function runs when parameter reconfiguration occurs
       *        Tutorial on this at: https://roboticsbackend.com/ros2-rclcpp-parameter-callback/
       * @param &params
       */
      rcl_interfaces::msg::SetParametersResult reconfigureSonarCallback(
         const std::vector<rclcpp::Parameter> &params);
      /**
       * @brief This callback uses the sonar_driver to periodically collect sonar data 
       *        
       */
      void loopSonarCallback();
      /**
       * @brief This callback uses the sonar_driver to periodically Fire sonar 
       *        
       */
      void loopFireCallback();

      // ROS Publishers
      rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr sonar_image_pub_;

      // Parameter callback handle (for dynamic reconfiguration)
      OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

      // --- DRIVER VARIABLES --- //       
      // ROS Timer for the Ping reader loop 
      rclcpp::TimerBase::SharedPtr sonar_update_timer_;
      rclcpp::TimerBase::SharedPtr sonar_fire_timer_;
      // Messages 
      sensor_msgs::msg::Image sonar_image_msg;
      // sonar_oculus_interface::msg::OculusFire fire_msg;
      // sonar_oculus_interface::msg::OculusPing ping_msg;

      // Main interface with Sonar  
      optional<OsDriver> sonar_driver; 
      FireConfig2 sonar_fire_config; 

      // Sonar Configuration (From ROS params)
      std::string model;
      std::string frame_id; 
      int mode                 = 1; 
      int pingRate             = 5;
      double range             = 10.0; 
      double gain              = 20.0; 
      double speedOfSound      = 0.0; 
      double salinity          = 0.0; 
      bool gainAssist          = false;  
      uint8_t gammaCorrection  = 0x7f; 
      uint8_t netSpeedLimit    = 0xff; 

};

#endif //SONAR_OCULUS__SONAR_OCULUS_HPP_