#include "OsClientCtrl.h"

#include <fstream>
#include <sstream>
#include <opencv2/opencv.hpp>

#define IP_ADDR "169.254.22.70"  // Oculus Sonar IP
#define RANGE_MAX_HF 5           // Maximum High Frequency range for M3000d Oculus
#define RANGE_MAX_LF 30          // Maximum Low  Frequency range for M3000d Oculus

using namespace std;

// Driver Struct collecting fire parameters  
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

    // Driver Attributes

    string sonarIp;              // Sonar IP for connection
    unsigned char* sonarImage;   // Sonar Image
    int sonarImageWidth;         // Sonar Image Width
    int sonarImageHeight;        // Sonar Image Height
    uint8_t* sonarRaw;           // Sonar Raw data
    OsClientCtrl sonar;          // Sonar API 
    FireConfig sonarFireConfig;  // Sonar Fire configuration
    bool sonarConnected = false; // Flag for TCP connection state 

    // Driver Methods 

    /**
     * @brief Establish TCP connection with the oculus sonar 
     *
     * @return true if TCP connection is succesful 
     */
    bool connectToSonar();                     
    /**
     * @brief Assest if Sonar Client's Read Thread is still running 
     *
     * @return true if reading thread active 
     */
    bool readThreadActive();      
    /**
     * @brief Initialize fire configuration after connecting 
     *
     * @param file_name
     * 
     * @return true if succesful initialization of fire parameters 
     */            
    bool initializeSonar(string file_name);   
    /**
     * @brief Dynamic reconfigure sonar fire parameters (at runtime)
     * 
     * @return true if succesfully reconfigured fire state 
     */
    bool reconfigureSonar();                   // Change fire configuration (mode, range, etc)
    /**
     * @brief Update SonarImage and image size, accessing read thread buffer 
     *
     */
    void updateImage();                        // Updating snar image 
    /**
     * @brief Show SonarImage in an OpenCV window 
     *
     */
    void showImage();                          // Visualize the sonar image in real time 
    /**
     * @brief Check consistency of configuration parameters request and set it
     *
     * @param modeIn
     * @param pingRateIn
     * @param rangeIn
     * @param gainIn
     * @param speedOfSoundIn
     * @param salinityIn
     * @param gainAssistIn
     * @param gammaCorrectionIn
     * @param netSpeedLimitIn
     * 
     * @return true if Consistent configuartion and initialization success
     */
    bool setConfig(int modeIn, int pingRateIn, double rangeIn, double gainIn, 
        double speedOfSoundIn, double salinityIn, bool gainAssistIn, 
        uint8_t gammaCorrectionIn, uint8_t netSpeedLimitIn); 
    /**
     * @brief Send a Fire request to sonar, using driver configuration
     *
     */
    void fireSonar();                                       

};

