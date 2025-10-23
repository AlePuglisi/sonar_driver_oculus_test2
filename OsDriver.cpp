#include "OsDriver.h"

OsDriver::OsDriver()
{
    // Use default configuration 
    string ip_addr = IP_ADDR;
    string file_name = "None";
    OsDriver(ip_addr, file_name); 
}

OsDriver::OsDriver(string ip_addr, string init_file_name) 
{
    sonar.m_hostname = ip_addr;
    cout << "Sonar Driver [OsDriver-initialization]: Strating Oculus Sonar Driver initialization ..." << endl ; 
    
    // Initialize Class Attributes
    latest_id = 0; 
    sonarImage = nullptr; 
    sonarImageHeight = 0; 
    sonarImageWidth = 0; 
    sonarRaw = nullptr; 
    sonarConnected = false; 

    // [TODO] REMOVE THIS -- CURRENTLY JUST FOR DEBUG  
    // if (! initializeSonar(init_file_name))
    //     return;
    // Config should be initialized only AFTER connection

    // Establish TCP connection 
 
    if (connectToSonar()) { // Connection succes
        cout << "Sonar Driver [OsDriver-connection]: Initializing Fire Configuration" << endl ; 
        // After connecting, initiaize sonar fire parameters from csv file or default 
        initializeSonar(init_file_name);
    } else { // Conncetion fail       
        cerr << "Sonar Driver [OsDriver-connection]: Sonar Not Connected, Not Initializing Fire Configuration" << endl ; 
        cerr << "Sonar Driver [OsDriver-connection]: Failed to connect, double check sonar IP ... " << endl ; 
        return; // Stop 
    }

    cout << "Sonar Driver [OsDriver-initialization]: Oculus Sonar Driver Initialized ! Ready to be used" << endl ; 
}

OsDriver::~OsDriver() {
    // Disconnect TCP if still on  
    if (sonar.m_TCPconnected)
        sonar.Disconnect();

}

void OsDriver::setSaving(bool savingOption)
{
    savingOption = savingOption; 
}

bool OsDriver::connectToSonar() 
{
    // Try to connect to the Oculus Sonar at the given IP 5 times 
    int trial_number = 1;
    cout << "Sonar Driver [connectToSonar]: Tentative " << to_string(trial_number) << " to connect at ip: " << sonar.m_hostname << endl ; 


    while ((trial_number <= RECONNECTION_TENTATIVE) and (!sonar.Connect())){
        trial_number += 1;
        if (trial_number <= RECONNECTION_TENTATIVE)
            cout << "Sonar Driver [connectToSonar]: Tentative " << to_string(trial_number) << " to connect at ip: " << sonar.m_hostname << endl ; 
    }

    // Log connection results
    if(sonar.m_TCPconnected){
        cout << "Sonar Driver [connectToSonar]: Connected to Sonar at ip: " << sonar.m_hostname << " ! " << endl ; 
    } else {
        cerr << "Sonar Driver [connectToSonar]: Failed to connect to Sonar at ip: " << sonar.m_hostname << endl ; 
        sonarConnected = false; 
        return sonarConnected; 
    }

    sonarConnected = true; 
    return sonarConnected; 
}

bool OsDriver::readThreadActive(){
    // Information on running thread from the sonar client read thread
    return sonar.IsOpen();
}

bool OsDriver::initializeSonar(string file_name) 
{
    // Initialize local variables to store file data 
    int mode                 = 1; 
    int pingRate             = 5;
    double range             = 10.0; 
    double gain              = 20.0; 
    double speedOfSound      = 0.0; 
    double salinity          = 0.0; 
    bool gainAssist          = false;  
    uint8_t gammaCorrection  = 0x7f; 
    uint8_t netSpeedLimit    = 0xff; 

    // Retrieve data from csv 
    if (file_name != "None"){ // Given configuration file name 
        string file_path = string(CONFIG_DIR_PATH) + file_name; //CONFIG_DIR_PATH defined in the CMakeLists.txt
        cout << "Sonar Driver [initializeSonar]: Retrieving configuration from csv file: " << file_path << endl; 
        ifstream file(file_path);
        string line;
        getline(file, line);
        string param;

        while (getline(file, line)) {
            stringstream ss(line);
            getline(ss, param, ',');

            if(param == "mode")
                ss >> mode;
            else if(param == "pingRate")
                ss >> pingRate;
            else if(param == "range")
                ss >> range;
            else if(param == "gain")
                ss >> gain;
            else if(param == "speedOfSound")
                ss >> speedOfSound;
            else if(param == "salinity")
                ss >> salinity;
            else if(param == "gainAssist")
                ss >> gainAssist;
            else if(param == "gammaCorrection")
                ss >> gammaCorrection;
            else if(param == "netSpeedLimit")
                ss >> netSpeedLimit;
        }

        file.close();
    }
    else{ // "None" file name 
        cout << "Sonar Driver [initializeSonar]: File name not give, using default sonar configuration" << endl; 
    }
    
    // Use given configuration values to update sonar config, only if consistent values 
    bool init_success = setConfig(mode, pingRate, range, gain, speedOfSound, 
                                  salinity, gainAssist, gammaCorrection, netSpeedLimit);

    return init_success; 
}

bool OsDriver::reconfigureSonar() 
{
    // [TODO] Implement a dynamic reconfiguration thread to change sonar fire config 
    return false; 
}  

void OsDriver::updateImage() 
{
    // Update sonar data 
    uint32_t id = sonar.m_readData.m_osBuffer[0].m_rfm.pingId;
    uint16_t nbins = sonar.m_readData.m_osBuffer[0].m_rfm.nRanges;
    uint16_t nbeams = sonar.m_readData.m_osBuffer[0].m_rfm.nBeams;
    uint32_t rawSize = sonar.m_readData.m_osBuffer[0].m_rawSize; 

    // Update image only if new image and data received 
    if(id > latest_id && rawSize > 0) {
        latest_id = id; 
        
        // Initialize current sonar image size 
        sonarImageHeight = nbins; 
        sonarImageWidth  = nbeams; 

        // Realloc memory for sonar image pointer 
        sonarImage = (unsigned char*) realloc (sonarImage, nbins * nbeams); 
        // Update image 
        memcpy(sonarImage, sonar.m_readData.m_osBuffer[0].m_pImage, 
            sonar.m_readData.m_osBuffer[0].m_rfm.imageSize);

        // Update raw sonar data 
        sonarRaw = (uint8_t*) realloc (sonarRaw, rawSize); 
        memcpy(sonarRaw, sonar.m_readData.m_osBuffer[0].m_pRaw, rawSize); 

        cout << "Sonar Driver [updateImage]: Update Sonar Image and Raw Data" << endl; 
    }
}

void OsDriver::showImage() 
{
    // Create a named window
    cv::namedWindow("Oculus Sonar View", cv::WINDOW_NORMAL);

    cv::Mat frame(sonarImageHeight, sonarImageWidth, CV_8UC1, sonarImage);

    cv::VideoWriter writer; 
    if (savingOption){
        int codec = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
        double fps = UPDATE_FREQUENCY;                    // framerate of the created video stream
        string filename = "./sonar_video.avi";            // name of the output video file
        cv::Size videoSize = cv::Size(sonarImageWidth, sonarImageHeight);
        writer.open(filename, codec, fps, videoSize, false);
    }
    

    // Example: simulate continuous updates
    while (true)
    {
        //cv::Mat frame(sonarImageHeight, sonarImageWidth, CV_8UC1, sonarImage);
    
        // Show the image in the window
        cv::imshow("Oculus Sonar View", frame);

        if(savingOption){
            writer.write(frame); 
        }

        // refresh view 
        int key = cv::waitKey(1);
        // Stop visualization if ESC is pressed 
        if (key == 27) 
            break;
    }

    cv::destroyAllWindows();
    writer.~VideoWriter(); 
}       

void OsDriver::startSonarView()
{
    // Wait until first image retrieved to create CV Window 
    while(sonarImage == nullptr) { 
        cout << "Sonar Driver [startSonarView]: No image yet received from Oculus Sonar" << endl; 
        this_thread::sleep_for(chrono::milliseconds(100)); 
    }

    // Start OpenCV Image show as thread 
    thread viewThread(&OsDriver::showImage, this);
    viewThread.detach(); 

    // Compute period from given frequency 
    int read_period = int(1 / UPDATE_FREQUENCY * 1e9); 
    // Start Sensor Update loop 
    while(readThreadActive()){
        updateImage(); 
        this_thread::sleep_for(chrono::nanoseconds(read_period)); 
    }
}

void OsDriver::startSonarDataProcessing() 
{
  cout << "Sonar Driver [startSonarDataProcessing]: Sending First Fire request" << endl; 
  // Send a first fire message request 
  fireSonar();

  // Start a view thread on the data 
  thread sonarViewThread(&OsDriver::startSonarView, this);
  sonarViewThread.detach(); 
  cout << "Sonar Driver [startSonarDataProcessing]: Detach View Thread" << endl; 

  // Compute Fire period from given frequency 
  int fire_period = int(1 / FIRE_FREQUENCY * 1e9); 

  // Send Periodic Fire Message request 
  while(sonarConnected and readThreadActive()){
    cout << "Sonar Driver [startSonarDataProcessing]: Sending New Fire request" << endl; 
    fireSonar();

    // retrieve data saved on sonar 
    std::this_thread::sleep_for(chrono::nanoseconds(fire_period));
  }

  cout << "Sonar Driver [startSonarDataProcessing]: Stop sonar data retrieval" << endl; 

}

bool OsDriver::setConfig(int modeIn, int pingRateIn, double rangeIn, double gainIn, double speedOfSoundIn, 
    double salinityIn, bool gainAssistIn, uint8_t gammaCorrectionIn, uint8_t netSpeedLimitIn)
{
    // Set ping rate
    if (pingRateIn == 0) // Standby Ping
        sonarFireConfig.pingRate = pingRateStandby; //  0Hz
    else if (pingRateIn == 1)
        sonarFireConfig.pingRate = pingRateLowest;  //  2Hz
    else if (pingRateIn == 2)
        sonarFireConfig.pingRate = pingRateLow;     //  5Hz
    else if (pingRateIn == 3)
        sonarFireConfig.pingRate = pingRateNormal;  // 10Hz
    else if (pingRateIn == 4)
        sonarFireConfig.pingRate = pingRateHigh;    // 15Hz
    else if (pingRateIn == 5)
        sonarFireConfig.pingRate = pingRateHighest; // 40Hz   
    else {
        cerr << "Sonar Driver [setConfig]: ping rate must be given as an integer from 0 to 5" << endl;
        return false;
    }

    // Ensure Range is consistent with master Mode 1= Low Freq / 2 = High Freq
    sonarFireConfig.mode = modeIn;
    if (modeIn == 1) {
        if (rangeIn > 0 and rangeIn <= RANGE_MAX_LF){
            sonarFireConfig.range = rangeIn; 
        }
        else if(rangeIn <= 0) {
            cerr << "Sonar Driver [setConfig]: Negative or 0 range are not accepted" << endl;
            return false;
        }
        else if(rangeIn > RANGE_MAX_LF) {
            cerr << "Sonar Driver [setConfig]: The given range: " << to_string(rangeIn) << 
                     "m is above the " <<to_string(RANGE_MAX_LF) << "m limits of Low Freq Mode" << endl;
            return false;
        }
    } else if (modeIn == 2) {
        if (rangeIn > 0 and rangeIn <= RANGE_MAX_HF){
            sonarFireConfig.range = rangeIn; 
        }
        else if(rangeIn <= 0) {
            cerr << "Sonar Driver [setConfig]: Negative or 0 range are not accepted" << endl;
            return false;
        }
        else if(rangeIn > RANGE_MAX_HF) {
            cerr << "Sonar Driver [setConfig]: The given range: " << to_string(rangeIn) << 
                     "m is above the " <<to_string(RANGE_MAX_HF) << "m limits of High Freq Mode" << endl;
            return false;
        }
    }

    // check gain value (0-100 percentage)
    if(gainIn > 0 and gainIn <= 100){
        sonarFireConfig.gain = gainIn;
    }
    else {
        cerr << "Sonar Driver [setConfig]: gain must be given in percentage as a 0-100 double" << endl;
        return false; 
    }

    // Assign other values 
    // [TO DO] Further check required on this paramters 
    sonarFireConfig.speedOfSound = speedOfSoundIn;
    sonarFireConfig.salinity = salinityIn;
    sonarFireConfig.gainAssist = gainAssistIn;
    sonarFireConfig.gammaCorrection = gammaCorrectionIn;
    sonarFireConfig.netSpeedLimit = netSpeedLimitIn;

    // Log results of configuration 
    cout << "Sonar Driver [setConfig]: Sonar Fire Configuration initialized !" << endl;
    cout << "Current configuration: \n" << "Mode: " << to_string(modeIn) << "\nPing Rate: " << to_string(pingRateIn)
         << "\nRange: " << to_string(rangeIn) << "\nGain: " << gainIn << "\nSpeed of Sound:" << to_string(speedOfSoundIn)
         << "\nsalinity: " << to_string(salinityIn) << "\nGain Assist: " << boolalpha << to_string(gainAssistIn)
         << "\nGamma Correction:" << to_string(gammaCorrectionIn) << "\nNetwork Speed Limit: " << to_string(netSpeedLimitIn) << endl;  

    return true; 
}

void OsDriver::fireSonar() 
{
    cout << "Sonar Driver [fireSonar]: Fire request !" << endl;
    // Fire sonar using the assigned configuration 
    sonar.Fire(sonarFireConfig.mode, 
               sonarFireConfig.pingRate,
               sonarFireConfig.range,
               sonarFireConfig.gain, 
               sonarFireConfig.speedOfSound,
               sonarFireConfig.salinity, 
               sonarFireConfig.gainAssist, 
               sonarFireConfig.gammaCorrection, 
               sonarFireConfig.netSpeedLimit);

}
    
