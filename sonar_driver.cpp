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
#include "OsDriver.h"
#include "OsClientCtrl.h"

int main(int argc, char *argv[])
{

  string sonar_config_file = "None";

  if (argc >= 2){
    sonar_config_file = string(argv[1]);
  }

  OsDriver sonar_driver = OsDriver(IP_ADDR, sonar_config_file);

  // Send Periodic Fire Message request 
  while(sonar_driver.sonar.IsOpen()){
    sonar_driver.fireSonar();

    // retrieve data saved on sonar 
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  return 0;

}
