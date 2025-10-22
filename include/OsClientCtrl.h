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

#ifndef CLIENTCTRL_H
#define CLIENTCTRL_H

#include "Oculus.h"
#include "DataWrapper.h"

#include <mutex>
#include <thread>
#include <string>
#include <iostream>
#include <stdio.h>

#include <bitset>
#include <climits>
#include <cstring>

#include <sys/types.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <sys/ioctl.h>

#include <algorithm>
#include <math.h>

#define PORT_UDP 52102
#define PORT_TCP 52100

#define DATALEN 200000

// ----------------------------------------------------------------------------
// OsBufferEntry - contains a return message and an embedded image
class OsBufferEntry
{
public:
  OsBufferEntry();
  ~OsBufferEntry();

  // Methods
  void AddRawToEntry(char* pData, uint64_t nData);
  void ProcessRaw(char* pData);

  // Data
  OculusSimplePingResult  m_rfm;         // The fixed length return fire message
  OculusReturnFireMessage m_rff;
  OculusSimplePingResult2 m_rfm2;	

  unsigned char*          m_pImage;      // The image data
  short*                  m_pBrgs;       // The bearing table
  std::mutex              m_mutex;       // Lock for buffer accesss

  uint16_t				         m_version;

  uint8_t*                 m_pRaw;        // The raw data
  uint32_t                 m_rawSize;     // Size of the raw data record

  bool					           m_simple;
};


class OsClientCtrl;
#define OS_BUFFER_SIZE 10

// ----------------------------------------------------------------------------
// OsReadThread - a worker thread used to read data from the network for the client
class OsReadThread : public std::thread
{

public:
  OsReadThread();
  ~OsReadThread();

  void run();

  void Startup();
  void Shutdown();
  bool IsActive();
  void SetActive(bool active);
  void ProcessRxBuffer();
  void ProcessPayload(char* pData, uint64_t nData);

  // Data
  OsClientCtrl* m_pClient;   // back pointer to the parent client
  bool          m_active;    // Is the run exec active
  std::mutex    m_mutex;     // Mutex protection for m_active
  std::mutex    m_sending;   // Mutex protection for m_active

  std::string   m_hostname;  // The hostname/address of the sonar
  uint16_t      m_port;      // The port for sonar comms (currently fixed)
  uint32_t      m_nFlushes;  // Number of times the rx buffer has had to be flushed

  // The raw receive buffer
  char*         m_pRxBuffer; // The rx buffer for incomming data
  int32_t       m_nRxMax;    // The maximum size of the rx Buffer
  int32_t       m_nRxIn;     // The current amount of unprocessed data in the buffer

  // The recieve buffer for messages
  OsBufferEntry m_osBuffer[OS_BUFFER_SIZE];
  unsigned      m_osInject;   // The position for the next inject

  // The raw send buffer
  int*          m_pSocket;  // (TCP Socket ?)
  char*         m_pToSend;
  int64_t       m_nToSend;

  //Communication
  struct sockaddr_in serverTCP;
  struct sockaddr_in clientTCP;

  int sockTCP;
  int sockTCPfd;
  int datagramSize;
  int n;
  int buf_size = DATALEN;
  int keepalive = 1;

  socklen_t lengthServerTCP;
  socklen_t lengthClientTCP;

};


// ----------------------------------------------------------------------------
// ClientCtrl - used to communicate with the oculus sonar
class OsClientCtrl 
{

public:
  OsClientCtrl();
  ~OsClientCtrl();

  bool Connect();
  bool Disconnect();
  bool IsOpen();
  void WriteToDataSocket(char* pData, uint16_t length);
  void Fire(int mode, PingRateType pingRate, double range, double gain, double speedOfSound, 
    double salinity, bool gainAssist, uint8_t gammaCorrection, uint8_t netSpeedLimit);
  void DummyMessage();


  // void WriteUserConfig(uint32_t ipAddress, uint32_t ipMask, bool dhcpEnable);
  // bool RequestUserConfig();

  bool WaitForReadOrTimeout(uint32_t ms);

  bool         m_received;

  std::string  m_hostname;    
  std::string  m_mask;

  bool         m_TCPconnected;

  UserConfig   m_config;   // Oculus user configuration
  OsReadThread m_readData; // The worker thread for reading data
};

#endif // CLIENTCTRL_H
