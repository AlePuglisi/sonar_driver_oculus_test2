#include "sonar_oculus/sonar_oculus.hpp"

#define DEBUG true

SonarOculusNode::SonarOculusNode()
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

SonarOculusNode::~SonarOculusNode()
{
    // Disconnect from sonar
    sonar.Disconnect();
    // close TCP connection 
    close(sockTCP);
}

void SonarOculusNode::error(const char *msg) {
         perror(msg);
         exit(0);
}

void SonarOculusNode::searchSonar()
{

#if DEBUG
    RCLCPP_INFO(this->get_logger(), "Sonar Oculus Network research Start.");
#endif

    // Clear and intialize values of server and client network info
    lengthServerUDP = sizeof(serverUDP);
    bzero((char *)&serverUDP, lengthServerUDP);
    serverUDP.sin_family = AF_INET;
    serverUDP.sin_addr.s_addr = htonl(INADDR_ANY);
    // serverUDP.sin_addr.s_addr = inet_addr(SONAR_ADDR);
    serverUDP.sin_port = htons(PORT_UDP);

    lengthClientUDP = sizeof(clientUDP);
    lengthServerTCP = sizeof(serverTCP);

    // If the user specified "model" parameter, set ip and partNumber directly
    if (!model.empty()) {
        if (model == "M750d") {
            ip = "192.168.2.3";
            partNumber = OculusPartNumberType::partNumberM750d;
        } else if (model == "M1200d") {
            ip = "192.168.2.4";
            partNumber = OculusPartNumberType::partNumberM1200d;
        } else if (model == "M3000d") {
            ip = "192.168.2.5";
            partNumber = OculusPartNumberType::partNumberM3000d;
        }else {
            RCLCPP_WARN(this->get_logger(), "Model parameter provided but not recognized: '%s'. Will attempt discovery.", model.c_str());
        }
    }
    // If model Not provided, listen on UDP port (PORT_UDP=52102 whaiting for OculusStatusMsg)
    else {
        while (true) {
            // Create the UDP listening socket or exit
            sockUDP = socket(AF_INET, SOCK_DGRAM, 0);
            if (sockUDP < 0)
            error("Error opening UDP listening socket");
            
            int enable = 1;
            if (setsockopt(sockUDP, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int)) < 0)
            error("setsockopt(SO_REUSEADDR) failed");

            // Bind the UDP socket to address and port, or exit with error
            if (bind(sockUDP, (struct sockaddr *)&serverUDP, lengthServerUDP) < 0)
            error("Error binding UDP listening socket");
            listen(sockUDP, 5);

            int64_t bytesAvailable;
            ioctl(sockUDP, FIONREAD, &bytesAvailable);

            OculusStatusMsg osm;
            // When receiving the USP message from Oculus
            if (bytesAvailable > 0) { 
                // unsigned bytesRead (Not used)
                read(sockUDP, (char*)&osm, bytesAvailable);


            // Safety check on temperature status 
            uint32_t ts = (osm.status >> 14) & 0x0003;
            if (ts == 0) {
                RCLCPP_INFO(this->get_logger(), "Temperature OK; the sonar will ping normally");
            } else if (ts == 1) {
                RCLCPP_WARN(this->get_logger(), "Temperature high; the sonar will ping at reduced rate");
            } else if (ts == 3) {
                RCLCPP_ERROR(this->get_logger(), "Temperature shutdown; the sonar will not longer ping");
                rclcpp::shutdown();
                return;
            }

            struct in_addr ip_addr;
            // Set IP address used for TCé connection 
            ip_addr.s_addr = osm.ipAddr;  
            ip = std::string(inet_ntoa(ip_addr));
            partNumber = osm.partNumber;
            // Set partNumber from received OculusStatusMsg
            if (partNumber == OculusPartNumberType::partNumberM750d)
                model = "M750d";
            else if (partNumber == OculusPartNumberType::partNumberM1200d)
                model = "M1200d";
            else if (partNumber == OculusPartNumberType::partNumberM3000d)
                model = "M3000d";
            else
                RCLCPP_ERROR(this->get_logger(), "Part number not recognized");

            close(sockUDP);
            break;
            }

            rclcpp::sleep_for(std::chrono::seconds(1));
        }

    }

    RCLCPP_INFO(this->get_logger(), "The IP address is %s", ip.c_str());
    RCLCPP_INFO(this->get_logger(), "Oculus model is %s", model.c_str());

}

void SonarOculusNode::connectToSonar()
{
#if DEBUG
    RCLCPP_INFO(this->get_logger(), "Sonar Oculus Network TCP Connection Start.");
#endif

    // Setting up TCP socket to the sonar IP after discovery 
    bzero((char *)&serverTCP, lengthServerTCP);
    serverTCP.sin_family = AF_INET;
    serverTCP.sin_addr.s_addr = inet_addr(ip.c_str());
    serverTCP.sin_port = htons(PORT_TCP);

    // Create the TCP socket for main communication or exit
    sockTCP = socket(AF_INET, SOCK_STREAM, 0);
    if (sockTCP < 0)
        error("Error opening TCP main socket");
    // Connect to the sonar Server via TCP socket or exit with error
    if (connect(sockTCP, (struct sockaddr *)&serverTCP, lengthServerTCP) < 0)
        error("Error connecting TCP socket");
    if (setsockopt(sockTCP, SOL_SOCKET, SO_RCVBUF, &buf_size, sizeof(buf_size)) <
        0)
        error("Error increasing RCVBUF for TCP socket");
    if (setsockopt(sockTCP, SOL_SOCKET, SO_KEEPALIVE, &keepalive,
                    sizeof(keepalive)) < 0)
        error("Error keeping alive option set for TCP socket");
    listen(sockTCP, 5);

    // Setup Sonar and messages
    // Pass the socket to the control
    // Raw socket descriptor stored in the client wrapper 
    sonar.m_readData.m_pSocket = &sockTCP;
    // Connect and instance a thread
    // Setting up the OsReadThred internal state 
    sonar.Connect();

    RCLCPP_INFO(this->get_logger(), "Connected!" );

    // Send Ping and initiate data collection
    sonar.Fire(mode, ping_rate, range, gain, soundspeed, salinity);

}
    

void SonarOculusNode::publishPing()
{
    // Run the read-thread work (fills internal buffers)
    sonar.m_readData.run();

    // Defensive: make sure buffers exist
    // if (sonar.m_readData.m_osBuffer->m_rawSize == 0) {
    //     return;
    // }

    unsigned int nbins = sonar.m_readData.m_osBuffer[0].m_rfm.nRanges;
    unsigned int nbeams = sonar.m_readData.m_osBuffer[0].m_rfm.nBeams;
    unsigned int id     = sonar.m_readData.m_osBuffer[0].m_rfm.pingId;

    if (nbeams > 0 && nbins > 0 && id > latest_id_) {
        latest_id_ = id;

        if (sonar.m_readData.m_osBuffer[0].m_rawSize) {
            // --- build sensor_msgs::msg::Image
            sensor_msgs::msg::Image sonar_image_msg;
            sonar_image_msg.header.frame_id = frame_id;
            sonar_image_msg.header.stamp = this->get_clock()->now();
            sonar_image_msg.height = nbins;
            sonar_image_msg.width = nbeams;
            sonar_image_msg.encoding = "8UC1";
            sonar_image_msg.is_bigendian = 0;
            sonar_image_msg.step = nbeams;
            sonar_image_msg.data.resize(static_cast<size_t>(nbins) * static_cast<size_t>(nbeams));
            std::copy(
                sonar.m_readData.m_osBuffer[0].m_pImage,
                sonar.m_readData.m_osBuffer[0].m_pImage + (nbins * nbeams),
                sonar_image_msg.data.begin()
            );

            // --- build fire_msg (fields copied from buffer)
            fire_msg.header.stamp = this->get_clock()->now();
            fire_msg.mode = sonar.m_readData.m_osBuffer[0].m_rfm.fireMessage.masterMode;
            fire_msg.gamma = sonar.m_readData.m_osBuffer[0].m_rfm.fireMessage.gammaCorrection;
            fire_msg.flags = sonar.m_readData.m_osBuffer[0].m_rfm.fireMessage.flags;
            fire_msg.range = sonar.m_readData.m_osBuffer[0].m_rfm.fireMessage.range;
            fire_msg.gain = sonar.m_readData.m_osBuffer[0].m_rfm.fireMessage.gainPercent;
            fire_msg.speed_of_sound = sonar.m_readData.m_osBuffer[0].m_rfm.fireMessage.speedOfSound;
            fire_msg.salinity = sonar.m_readData.m_osBuffer[0].m_rfm.fireMessage.salinity;

            // --- build ping_msg
            ping_msg.header.frame_id = frame_id;
            ping_msg.header.stamp = this->get_clock()->now();

            // compress image to jpeg (cv_bridge required)
            std::vector<int> params;
            params.resize(3, 0);
            params[0] = cv::IMWRITE_JPEG_QUALITY;
            params[1] = 80;

            // Convert sensor_msgs::Image -> cv::Mat via cv_bridge
            cv_bridge::CvImagePtr cv_ptr;

            //define target format for compression	
            std::stringstream targetFormat;        
            targetFormat << "8UC1";

            //const_cast<sensor_msgs::Image &>(sonar_image).step = 512;
            cv_ptr = cv_bridge::toCvCopy(sonar_image,targetFormat.str());

            //compress image here, note this command compresses AND sets the compressed image to the message
            cv::imencode(".jpg", cv_ptr->image, ping_msg.ping.data, params);

            ping_msg.ping.format += std::string("; jpeg compressed");

            ping_msg.fire_msg = fire_msg;
            ping_msg.ping_id = id;
            ping_msg.part_number = partNumber;

            ping_msg.start_time = sonar.m_readData.m_osBuffer[0].m_rfm.pingStartTime;

            // bearings
            ping_msg.bearings.resize(nbeams);
            for (unsigned int i = 0; i < nbeams; ++i)
                ping_msg.bearings[i] = sonar.m_readData.m_osBuffer[0].m_pBrgs[i];

            ping_msg.range_resolution = sonar.m_readData.m_osBuffer[0].m_rfm.rangeResolution;
            ping_msg.num_ranges = nbins;
            ping_msg.num_beams = nbeams;

            // publish
            ping_pub_->publish(ping_msg);
        }
    }
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
  auto node = std::make_shared<SonarOculusNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
} 
