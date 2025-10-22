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
#include <sonar_oculus_interface/msg/oculus_fire.hpp>
#include <sonar_oculus_interface/msg/oculus_ping.hpp>

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


      // ROS Publishers
      rclcpp::Publisher<sonar_oculus_interface::msg::OculusPing>::SharedPtr ping_pub_;

      // Parameter callback handle (for dynamic reconfiguration)
      OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

      // --- DRIVER VARIABLES --- //       
      // ROS Timer for the Ping reader loop 
      rclcpp::TimerBase::SharedPtr sonar_timer_;

      // Messages 
      sensor_msgs::msg::Image sonar_image;
      sonar_oculus_interface::msg::OculusFire fire_msg;
      sonar_oculus_interface::msg::OculusPing ping_msg;

      // Main interface with Sonar  
      OsClientCtrl sonar;               // User facing API to interact with Sonar HW 
      OculusPartNumberType partNumber;  // Data type to represent HW partNumber based on Oculus Model 

      // Sonar Configuration (From ROS params)
      std::string model;
      std::string frame_id; 
      int mode;
      int ping_rate;
      double range;
      double gain;
      double soundspeed;
      double salinity;

};

#endif //SONAR_OCULUS__SONAR_OCULUS_HPP_