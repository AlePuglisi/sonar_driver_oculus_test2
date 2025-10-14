/******************************************************************************
 * (c) Copyright 2017 Blueprint Subsea.
 * This file is part of Oculus Viewer
 *
 * Oculus Viewer is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Oculus Viewer is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *****************************************************************************/
#include "OsClientCtrl.h"

int main(int argc, char *argv[])
{
  OsClientCtrl sonar = OsClientCtrl();

  sonar.m_hostname = "169.254.22.70";

  int trial_number = 1;
  std::cout << "Sonar Driver: Tentative " << std::to_string(trial_number) << " to connect at ip: " << sonar.m_hostname << std::endl ; 


  while ((trial_number < 5) and (!sonar.Connect())){
    trial_number += 1;
    std::cout << "Sonar Driver: Tentative " << std::to_string(trial_number) << " to connect at ip: " << sonar.m_hostname << std::endl ; 
  }

  if(sonar.Connect()){
    std::cout << "Sonar Driver: Connected to Sonar at ip: " << sonar.m_hostname << std::endl ; 
  } else {
    std::cout << "Sonar Driver: Failed to connect to Sonar at ip: " << sonar.m_hostname << std::endl ; 
    return 0; 
  }

  // Set up Fire Message request 
  int mode                 = 1; 
  double range             = 10.0; 
  double gain              = 20.0; 
  double speedOfSound      = 0.0; 
  double salinity          = 0.0; 
  bool gainAssist          = false;  
  uint8_t gammaCorrection  = 0x7f; 
  uint8_t netSpeedLimit    = 0xff; 

  // Send Periodic Fire Message request 
  while(sonar.IsOpen()){
    sonar.Fire(mode, range, gain, speedOfSound, salinity, gainAssist, gammaCorrection, netSpeedLimit);

    // retrieve data saved on sonar 

    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  return 0;

}
