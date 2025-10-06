#ifndef SONAR_OCULUS__SONAR_OCULUS_HPP_
#define SONAR_OCULUS__SONAR_OCULUS_HPP_

#include <algorithm>
#include <arpa/inet.h>
#include <iostream>
#include <math.h>
#include <netdb.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

// sonar_oculus dependencies
#include "Oculus.h"
#include "OculusClient.h"

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

#define BUFLEN 200
#define DATALEN 200000
#define PORT_UDP 52102
#define PORT_TCP 52100

using namespace cv;

class SonarOculusNode : public rclcpp::Node 
{
   public: 
      SonarOculusNode();
      ~SonarOculusNode();
    
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
       * @brief This function search Oculus Sonar ip (UDP if ip not found)
       * 
       */
      void searchSonar(); 
      /**
       * @brief This function set up TCP connection
       * 
       */
      void connectToSonar(); 
      /**
       * @brief This function publish ping message
       * 
       */
      void publishPing();      
      /**
       * @brief Simple error handling function
       * 
       * @param *msg 
       */
      // Error handling function
      void error(const char *msg);

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

      //Communication variables 
      std::string ip;
      struct sockaddr_in serverUDP;
      struct sockaddr_in clientUDP;
      struct sockaddr_in serverTCP;
      struct sockaddr_in clientTCP;
      int sockUDP; 
      int sockTCP;
      int sockTCPfd;
      int datagramSize;
      int n;
      int buf_size = DATALEN;
      int keepalive = 1;
      socklen_t lengthServerUDP;
      socklen_t lengthClientUDP;
      socklen_t lengthServerTCP;
      socklen_t lengthClientTCP;
      char datagramMessage[BUFLEN];
      char buffer[BUFLEN];
      char sonardata[DATALEN];


      // track last ping id to avoid republishing
      unsigned int latest_id_ = 0;
};

#endif //SONAR_OCULUS__SONAR_OCULUS_HPP_