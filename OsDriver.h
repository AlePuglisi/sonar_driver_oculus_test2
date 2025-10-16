#include "OsClientCtrl.h"


using namespace std;

struct FireConfig
{
    int mode; 
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
    OsDriver(string ip_addr);
    ~OsDriver();

    // attributes
    string sonarIp;         // sonar IP for connection
    uint8_t sonarImage[10];   // current sonar image (FIX THIS RESIZABLE !)
    OsClientCtrl sonar;     // sonar API 

    // methods 
    bool connectToSonar(); // establish TCP connection
    void processImage();                 // processing snar image (Maybe NOT Required) [!]
    void showImage();                    // visualize the sonar image in real time 

    void reconfiggureSonar();            // change fire configuration (mode, range, etc)
    void fireSonar();                    // send fire request to sonar 


};

