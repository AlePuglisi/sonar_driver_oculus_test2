#include "OsClientCtrl.h"

#include <fstream>
#include <sstream>

#define IP_ADDR "169.254.22.70"

using namespace std;

struct FireConfig
{
    int mode; 
    PingRateType pingRate;
    double range; 
    double gain; 
    double speedOfSound; 
    double salinity; 
    bool gainAssist; 
    uint8_t gammaCorrection;
    uint8_t netSpeedLimit;
};


class OsDriver
{
public:
    OsDriver();
    OsDriver(string ip_addr, string init_file_name);
    ~OsDriver();

    // attributes
    string sonarIp;             // Sonar IP for connection
    uint8_t sonarImage[10];     // Current sonar image (FIX THIS RESIZABLE !)
    OsClientCtrl sonar;         // Sonar API 
    FireConfig sonarFireConfig; // Sonar Fire configuration
    bool sonarConnected = false; 

    // methods 
    bool connectToSonar();     // Establish TCP connection
    bool readThreadActive();   // Check if read thread is still running
    bool initializeSonar(string file_name);    // Initialize fire configuration after connecting 
    bool reconfigureSonar();   // Change fire configuration (mode, range, etc)
    void processImage();       // Processing snar image (Maybe NOT Required) [!]
    void showImage();          // Visualize the sonar image in real time 

    bool setConfig(int modeIn, int pingRateIn, double rangeIn, double gainIn, 
        double speedOfSoundIn, double salinityIn, bool gainAssistIn, 
        uint8_t gammaCorrectionIn, uint8_t netSpeedLimitIn); // Check consistency of configuration request and set it

    void fireSonar();                                        // Send fire request to sonar 


};

