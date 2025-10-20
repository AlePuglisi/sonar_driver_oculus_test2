#include "OsDriver.h"

string(DATA_DIR_PATH);

OsDriver::OsDriver()
{
    string ip_addr = IP_ADDR;
    string file_name = "None";
    OsDriver(ip_addr, file_name);
}

OsDriver::OsDriver(string ip_addr, string init_file_name) 
{
    sonar.m_hostname = ip_addr;
    cout << "Sonar Driver [initialization]: Strating Oculus Sonar Driver initialization ..." << endl ; 
    initializeSonar(init_file_name);

    if (connectToSonar()) {
        cout << "Sonar Driver [connection]: Initializing Fire Configuration" << endl ; 
        initializeSonar(init_file_name);
    } else { 
        cerr << "Sonar Driver [connection]: Sonar Not Connected, Not Initializing Fire Configuration" << endl ; 
        cerr << "Sonar Driver [connection]: Try to connect again, double check IP ... " << endl ; 
        return;
    }

    cout << "Sonar Driver [initialization]: Oculus Sonar Driver Initialized ! Ready to be used" << endl ; 
}

OsDriver::~OsDriver() {
    sonar.Disconnect();
}

bool OsDriver::connectToSonar() 
{
    // Try to connect to the Oculus Sonar at the given IP 5 times 
    int trial_number = 1;
    cout << "Sonar Driver [connection]: Tentative " << to_string(trial_number) << " to connect at ip: " << sonar.m_hostname << endl ; 


    while ((trial_number < 5) and (!sonar.Connect())){
        trial_number += 1;
        cout << "Sonar Driver [connection]: Tentative " << to_string(trial_number) << " to connect at ip: " << sonar.m_hostname << endl ; 
    }

    if(sonar.m_TCPconnected){
        cout << "Sonar Driver [connection]: Connected to Sonar at ip: " << sonar.m_hostname << " ! " << endl ; 
    } else {
        cerr << "Sonar Driver [connection]: Failed to connect to Sonar at ip: " << sonar.m_hostname << endl ; 
        return false; 
    }

    return true;
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
    if (file_name != "None"){
        cout << "Sonar Driver [pre-configuration]: Retrieving configuration from csv" << endl; 
        string file_path = string(CONFIG_DIR_PATH) + file_name;
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
    else{
        cout << "Sonar Driver [pre-configuration]: File name not give, usingdefault sonar configuration" << endl; 
    }
    
    
    bool init_success = setConfig(mode, pingRate, range, gain, speedOfSound, 
                                  salinity, gainAssist, gammaCorrection, netSpeedLimit);

    return init_success; 
}

bool OsDriver::reconfigureSonar() 
{
    return false; 
}  

void OsDriver::processImage() 
{

}

void OsDriver::showImage() 
{

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
        cerr << "Sonar Driver [configuration]: ping rate must be given as an integer from 0 to 5" << endl;
        return false;
    }

    // check master Mode 1= Low Freq / 2 = High Freq
    sonarFireConfig.mode = modeIn;
    if (modeIn == 1) {
        if (rangeIn > 0 and rangeIn <= 30){
            sonarFireConfig.range = rangeIn; 
        }
        else if(rangeIn <= 0) {
            cerr << "Sonar Driver [configuration]: Negative or 0 range are not accepted" << endl;
            return false;
        }
        else if(rangeIn > 30) {
            cerr << "Sonar Driver [configuration]: The given range: " << to_string(rangeIn) << "m is above the 30m limits of Low Freq Mode" << endl;
            return false;
        }
    } else if (modeIn == 2) {
        if (rangeIn > 0 and rangeIn <= 5){
            sonarFireConfig.range = rangeIn; 
        }
        else if(rangeIn <= 0) {
            cerr << "Sonar Driver [configuration]: Negative or 0 range are not accepted" << endl;
            return false;
        }
        else if(rangeIn > 5) {
            cerr << "Sonar Driver [configuration]: The given range: " << to_string(rangeIn) << "m is above the 5m limits of High Freq Mode" << endl;
            return false;
        }
    }

    if(gainIn > 0 and gainIn <= 100){
        sonarFireConfig.gain = gainIn;
    }
    else {
        cerr << "Sonar Driver [configuration]: gain must be given in percentage as a 0-100 double" << endl;
        return false; 
    }

    sonarFireConfig.speedOfSound = speedOfSoundIn;
    sonarFireConfig.salinity = salinityIn;
    sonarFireConfig.gainAssist = gainAssistIn;
    sonarFireConfig.gammaCorrection = gammaCorrectionIn;
    sonarFireConfig.netSpeedLimit = netSpeedLimitIn;

    cout << "Sonar Driver [configuration]: Sonar Fire Configuration initialized !" << endl;
    cout << "Current configuration: \n" << "Mode: " << to_string(modeIn) << "\nPing Rate: " << to_string(pingRateIn)
         << "\nRange: " << to_string(rangeIn) << "\nGain: " << gainIn << "\nSpeed of Sound:" << to_string(speedOfSoundIn)
         << "\nsalinity: " << to_string(salinityIn) << "\nGain Assist: " << boolalpha << to_string(gainAssistIn)
         << "\nGamma Correction:" << to_string(gammaCorrectionIn) << "\nNetwork Speed Limit: " << to_string(netSpeedLimitIn) << endl;  

    return true; 
}

void OsDriver::fireSonar() 
{
    cout << "Sonar Driver [fire]: Fire request !" << endl;
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
    
