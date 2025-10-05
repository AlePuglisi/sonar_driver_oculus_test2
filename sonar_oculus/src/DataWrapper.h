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

#pragma once                    // Avoid multiple include in the header 

// ----------------------------------------------------------------------------
// User data configuration structure

// this structure is probably used for sending or receiving network configuration 
// data between the sonar and the host computer (ROS driver).
#pragma pack(push, 1)           // Managment of padding bytes in the struct 
struct UserConfig
{
    uint32_t m_ipAddr;          // Sonar IP Adress
    uint32_t m_ipMask;          // Sonar SubNet Mask
    uint32_t m_bDhcpEnable;     // DHCP enable flag (protocol to assign IP addresses)
};
#pragma pack(pop)               // Managment of padding bytes in the struct 

