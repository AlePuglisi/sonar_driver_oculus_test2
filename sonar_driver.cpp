#include "OsDriver.h"
#include "OsClientCtrl.h"

int main(int argc, char *argv[])
{

  // "None" file name uses default fire config 
  string sonar_config_file = "None";
  bool save_option = false; 
  // Check for user input as file name  
  if (argc == 2){
    sonar_config_file = string(argv[1]);
  }
  if (argc == 3){
    save_option = bool(argv[2]);
  }

  // Instantiate sonar class 
  OsDriver sonar_driver = OsDriver(IP_ADDR, sonar_config_file);
  sonar_driver.setSaving(save_option); 

  // Check if the initialization succesfully connected the sonar 
  if(!sonar_driver.sonarConnected){
    cerr << "Sonar Not connected after five tentatives ... Driver OFF, check configuration, IP or Sonar state and retry " << endl; 
    return 0; 
  }

  sonar_driver.startSonarDataProcessing(); 

  return 0;

}
