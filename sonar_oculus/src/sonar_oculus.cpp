#include "sonar_oculus/sonar_oculus.hpp"

#define DEBUG true

SonarOculusNode::SonarOculusNode()
: Node("sonar_oculus"),
  mode(1), ping_rate(3), range(10.0), gain(20.0), soundspeed(0.0), salinity(0.0), model("M3000d"), frame_id("sonar")
{
    // Declare and get parameters with defaults
    this->declare_parameter<int>("Mode", mode);
    this->declare_parameter<int>("PingRate", ping_rate);
    this->declare_parameter<double>("Range", range);
    this->declare_parameter<double>("Gain", gain);
    this->declare_parameter<double>("Salinity", salinity);
    this->declare_parameter<std::string>("model", model);
    this->declare_parameter<std::string>("frame_id", frame_id);

    // Register callback for parameter changes
    param_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&SonarOculusNode::reconfigureSonarCallback, this, std::placeholders::_1));

    // Create publisher
    ping_pub_ = this->create_publisher<sonar_oculus_interface::msg::OculusPing>(
        "/sonar_oculus_node/" + model + "/ping", 10);

    // Initialize communication TCP + UDP 
    connectToSonar();

    // Setup timer loop (20 Hz here)
    sonar_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20), std::bind(&SonarOculusNode::publishPing, this));

    RCLCPP_INFO(this->get_logger(), "Sonar Oculus node initialized.");
}


void SonarOculusNode::connectToSonar()
{

    bool have_ip = false;
#if DEBUG
    RCLCPP_INFO(this->get_logger(), "Sonar Oculus Connection Start.");
#endif
    // If the user specified "model" parameter, set ip and partNumber directly
    if (!model.empty()) {
        if (model == "M750d") {
            ip = "192.168.2.3";
            partNumber = OculusPartNumberType::partNumberM750d;
            have_ip = true;
        } else if (model == "M1200d") {
            ip = "192.168.2.4";
            partNumber = OculusPartNumberType::partNumberM1200d;
            have_ip = true;
        } else if (model == "M3000d") {
            ip = "192.168.2.5";
            partNumber = OculusPartNumberType::partNumberM3000d;
            have_ip = true;
        }else {
            RCLCPP_WARN(this->get_logger(), "Model parameter provided but not recognized: '%s'. Will attempt discovery.", model.c_str());
            have_ip = false;
        }
    }

    // If no ip was provided (or model not recognized) -> do UDP discovery loop
    if (!have_ip) {
        RCLCPP_INFO(this->get_logger(), "Starting UDP discovery for Oculus sonar...");

        // Setup UDP server struct
        lengthServerUDP = sizeof(serverUDP);
        bzero((char *)&serverUDP, lengthServerUDP);
        serverUDP.sin_family = AF_INET;
        serverUDP.sin_addr.s_addr = htonl(INADDR_ANY);
        serverUDP.sin_port = htons(PORT_UDP);

        // Create UDP socket
        sockUDP = socket(AF_INET, SOCK_DGRAM, 0);
        if (sockUDP < 0) {
            throw std::runtime_error(std::string("Error opening UDP listening socket: ") + strerror(errno));
        }

        int enable = 1;
        if (setsockopt(sockUDP, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int)) < 0) {
            close(sockUDP);
            throw std::runtime_error("setsockopt(SO_REUSEADDR) failed");
        }

        if (bind(sockUDP, (struct sockaddr *)&serverUDP, lengthServerUDP) < 0) {
            close(sockUDP);
            throw std::runtime_error(std::string("Error binding UDP socket: ") + strerror(errno));
        }

        // Loop until we receive a status message from the sonar
        while (rclcpp::ok()) {
            int64_t bytesAvailable = 0;
            ioctl(sockUDP, FIONREAD, &bytesAvailable);
            if (bytesAvailable > 0) {
                // Read the status message into OculusStatusMsg (same as original)
                OculusStatusMsg osm;
                ssize_t bytesRead = read(sockUDP, (char *)&osm, static_cast<size_t>(std::min<int64_t>(bytesAvailable, (int64_t)sizeof(osm))));
                if (bytesRead <= 0) {
                    RCLCPP_WARN(this->get_logger(), "Read returned <= 0 while discovering: %zd", bytesRead);
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                    continue;
                }

                // temperature state bits (same as original)
                uint32_t ts = (osm.status >> 14) & 0x0003;
                if (ts == 0) {
                    RCLCPP_INFO(this->get_logger(), "Temperature OK; the sonar will ping normally");
                } else if (ts == 1) {
                    RCLCPP_WARN(this->get_logger(), "Temperature high; the sonar will ping at reduced rate");
                } else if (ts == 3) {
                    RCLCPP_ERROR(this->get_logger(), "Temperature shutdown; sonar will not ping");
                    throw std::runtime_error("Sonar reported temperature shutdown");
                }

                struct in_addr ip_addr;
                ip_addr.s_addr = osm.ipAddr;
                ip = std::string(inet_ntoa(ip_addr));
                partNumber = osm.partNumber;

                if (partNumber == OculusPartNumberType::partNumberM750d)
                    model = "M750d";
                else if (partNumber == OculusPartNumberType::partNumberM1200d)
                    model = "M1200d";
                else if (partNumber == OculusPartNumberType::partNumberM3000d)
                    model = "M3000d";
                else
                    RCLCPP_WARN(this->get_logger(), "Part number not recognized: %d", static_cast<int>(partNumber));

                close(sockUDP);
                sockUDP = -1;
                RCLCPP_INFO(this->get_logger(), "Discovered sonar at %s, model %s", ip.c_str(), model.c_str());
                have_ip = true;
                break;
            }
            // sleep then poll again
            std::this_thread::sleep_for(std::chrono::seconds(1));
        } // discovery loop
    } // end discovery

    if (!have_ip) {
        throw std::runtime_error("Could not determine sonar IP (no model param and discovery failed)");
    }


#if DEBUG
    RCLCPP_INFO(this->get_logger(), "Start TCP connection.");
#endif

    // --- Setup TCP connection to sonar
    lengthServerTCP = sizeof(serverTCP);
    bzero((char *)&serverTCP, lengthServerTCP);
    serverTCP.sin_family = AF_INET;
    serverTCP.sin_addr.s_addr = inet_addr(ip.c_str());
    serverTCP.sin_port = htons(PORT_TCP);

    // Create TCP socket
    sockTCP = socket(AF_INET, SOCK_STREAM, 0);
    if (sockTCP < 0) {
        throw std::runtime_error(std::string("Error opening TCP socket: ") + strerror(errno));
    }

    // Connect
    if (connect(sockTCP, (struct sockaddr *)&serverTCP, lengthServerTCP) < 0) {
        close(sockTCP);
        sockTCP = -1;
        throw std::runtime_error(std::string("Error connecting TCP socket: ") + strerror(errno));
    }

    // adjust receive buffer size
    if (setsockopt(sockTCP, SOL_SOCKET, SO_RCVBUF, &buf_size, sizeof(buf_size)) < 0) {
        RCLCPP_WARN(this->get_logger(), "Warning: couldn't increase TCP RCVBUF");
    }
    if (setsockopt(sockTCP, SOL_SOCKET, SO_KEEPALIVE, &keepalive, sizeof(keepalive)) < 0) {
        RCLCPP_WARN(this->get_logger(), "Warning: couldn't set SO_KEEPALIVE");
    }

    // Hook socket to sonar client and connect
    sonar.m_readData.m_pSocket = &sockTCP;
    sonar.Connect();

    // send an initial fire using current params
    sonar.Fire(mode, ping_rate, range, gain, soundspeed, salinity);

    RCLCPP_INFO(this->get_logger(), "Connected to sonar TCP %s:%d", ip.c_str(), PORT_TCP);
}
    

void SonarOculusNode::publishPing()
{
    // Run the read-thread work (fills internal buffers)
    sonar.m_readData.run();

    // Defensive: make sure buffers exist
    if (sonar.m_readData.m_osBuffer->m_rawSize == 0) {
        return;
    }

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
            sonar_oculus_interface::msg::OculusFire fire_msg_local;
            fire_msg_local.header.stamp = this->get_clock()->now();
            fire_msg_local.mode = sonar.m_readData.m_osBuffer[0].m_rfm.fireMessage.masterMode;
            fire_msg_local.gamma = sonar.m_readData.m_osBuffer[0].m_rfm.fireMessage.gammaCorrection;
            fire_msg_local.flags = sonar.m_readData.m_osBuffer[0].m_rfm.fireMessage.flags;
            fire_msg_local.range = sonar.m_readData.m_osBuffer[0].m_rfm.fireMessage.range;
            fire_msg_local.gain = sonar.m_readData.m_osBuffer[0].m_rfm.fireMessage.gainPercent;
            fire_msg_local.speed_of_sound = sonar.m_readData.m_osBuffer[0].m_rfm.fireMessage.speedOfSound;
            fire_msg_local.salinity = sonar.m_readData.m_osBuffer[0].m_rfm.fireMessage.salinity;

            // --- build ping_msg
            sonar_oculus_interface::msg::OculusPing ping_msg_local;
            ping_msg_local.header.frame_id = frame_id;
            ping_msg_local.header.stamp = this->get_clock()->now();

            // compress image to jpeg (cv_bridge required)
            std::vector<int> params;
            params.resize(3, 0);
            params[0] = cv::IMWRITE_JPEG_QUALITY;
            params[1] = 80;

            // Convert sensor_msgs::Image -> cv::Mat via cv_bridge
            cv_bridge::CvImagePtr cv_ptr;
            try {
                cv_ptr = cv_bridge::toCvCopy(sonar_image_msg, "8UC1");
            } catch (const cv_bridge::Exception &e) {
                RCLCPP_ERROR(this->get_logger(), "cv_bridge conversion failed: %s", e.what());
                return;
            }

            // compress into ping_msg_local.ping.data (sensor_msgs::msg::CompressedImage)
            std::vector<unsigned char> encbuf;
            cv::imencode(".jpg", cv_ptr->image, encbuf, params);
            ping_msg_local.ping.format = std::string("8UC1; jpeg compressed");
            ping_msg_local.ping.data = encbuf;

            ping_msg_local.fire_msg = fire_msg_local;
            ping_msg_local.ping_id = id;
            ping_msg_local.part_number = partNumber; // assumes msg uses same enum/field type
            ping_msg_local.start_time = sonar.m_readData.m_osBuffer[0].m_rfm.pingStartTime;

            // bearings
            ping_msg_local.bearings.resize(nbeams);
            for (unsigned int i = 0; i < nbeams; ++i)
                ping_msg_local.bearings[i] = sonar.m_readData.m_osBuffer[0].m_pBrgs[i];

            ping_msg_local.range_resolution = sonar.m_readData.m_osBuffer[0].m_rfm.rangeResolution;
            ping_msg_local.num_ranges = nbins;
            ping_msg_local.num_beams = nbeams;

            // publish
            ping_pub_->publish(ping_msg_local);
        }
    }
}

rcl_interfaces::msg::SetParametersResult 
SonarOculusNode::reconfigureSonarCallback(const std::vector<rclcpp::Parameter> &params)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto &p : params) {
        if (p.get_name() == "Mode") {
            mode = p.as_int();
        } else if (p.get_name() == "PingRate") {
            ping_rate = p.as_int();
        } else if (p.get_name() == "Gain") {
            gain = p.as_double();
        } else if (p.get_name() == "Range") {
            range = p.as_double();
        } else if (p.get_name() == "Salinity") {
            salinity = p.as_double();
        }
    }

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
