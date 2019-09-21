// Copyright (C) 2019 Yohei Matsumoto
// This program is a modified version of the program code with following lines.
// Thus the version is under GPLv2.

/******************************************************************************
 *
 * Project:  OpenCPN
 * Purpose:  Radar Plugin
 * Author:   David Register
 *           Dave Cowell
 *           Kees Verruijt
 *           Hakan Svensson
 *           Douwe Fokkema
 *           Sean D'Epagnier
 ***************************************************************************
 *   Copyright (C) 2010 by David S. Register              bdbcat@yahoo.com *
 *   Copyright (C) 2012-2013 by Dave Cowell                                *
 *   Copyright (C) 2012-2016 by Kees Verruijt         canboat@verruijt.net *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************
 */

#include <cstdio>
#include <cstring>
#include <cmath>

#include <iostream>
#include <vector>
#include <queue>
#include <string>
#include <map>
#include <mutex>
using namespace std;
#include <opencv2/opencv.hpp>
using namespace cv;


#include "../../util/aws_stdlib.h"
#include "../../util/aws_thread.h"
#include "../../util/c_clock.h"

#include "../f_radar.h"

/*
 * This file not only contains the radar receive threads, it is also
 * the only unit that understands what the radar returned data looks like.
 * The rest of the plugin uses a (slightly) abstract definition of the radar.
 */

#define MILLIS_PER_SELECT 250
#define SECONDS_SELECT(x) ((x)*1000 / MILLIS_PER_SELECT)

//
//
#define SCALE_RAW_TO_DEGREES(raw) ((raw) * (double)DEGREES_PER_ROTATION / GARMIN_XHD_SPOKES)
#define SCALE_DEGREES_TO_RAW(angle) ((int)((angle) * (double)GARMIN_XHD_SPOKES / DEGREES_PER_ROTATION))

#pragma pack(push, 1)

struct radar_line {
  uint32_t packet_type;
  uint32_t len1;
  uint16_t fill_1;
  uint16_t scan_length;
  uint16_t angle;
  uint16_t fill_2;
  uint32_t range_meters;
  uint32_t display_meters;
  uint16_t fill_3;
  uint16_t scan_length_bytes_s;  // Number of video bytes in the packet, Short
  uint16_t fills_4;
  uint32_t scan_length_bytes_i;  // Number of video bytes in the packet, Integer
  uint16_t fills_5;
  uint8_t line_data[GARMIN_XHD_MAX_SPOKE_LEN];
};

#pragma pack(pop)

// ProcessLine
// ------------
// Process one radar line, which contains exactly one line or spoke of data extending outwards
// from the radar up to the range indicated in the packet.
//
void GarminxHDReceive::ProcessFrame(const uint8_t *data, size_t len) {
  // log_line.time_rec = wxGetUTCTimeMillis();
  long long time_rec = pfilter->get_time();
  
  time_t now = (time_t)(time_rec / SEC);

  radar_line *packet = (radar_line *)data;

  pfilter->m_radar_timeout = now + WATCHDOG_TIMEOUT;
  pfilter->m_data_timeout = now + DATA_TIMEOUT;
  radar_state->set_state(RADAR_TRANSMIT);

  const size_t packet_header_length = sizeof(radar_line) - GARMIN_XHD_MAX_SPOKE_LEN;
  pfilter->m_statistics.packets++;
  if (len < packet_header_length || len < packet_header_length + packet->scan_length_bytes_s) {
    // The packet is incomplete!
    pfilter->m_statistics.broken_packets++;
    return;
  }
  len -= packet_header_length;

  if (m_first_receive) {
    m_first_receive = false;
    long long  startup_elapsed = (pfilter->get_time() - pfilter->GetBootTime()) / MSEC;
    printf(("radar_pi: %s first radar spoke received after %lld ms\n"), pfilter->get_name(), startup_elapsed);
  }

  int angle_raw = packet->angle / 8;
  int spoke = angle_raw;  // Garmin does not have radar heading, so there is no difference between spoke and angle
  pfilter->m_statistics.spokes++;
  if (m_next_spoke >= 0 && spoke != m_next_spoke) {
    if (spoke > m_next_spoke) {
      pfilter->m_statistics.missing_spokes += spoke - m_next_spoke;
    } else {
      pfilter->m_statistics.missing_spokes += GARMIN_XHD_SPOKES + spoke - m_next_spoke;
    }
  }

  m_next_spoke = (spoke + 1) % GARMIN_XHD_SPOKES;

  short int heading_raw = 0;
  int bearing_raw;

  {
    long long tatt;
    float r,p,y;
    state->get_attitude(tatt, r, p, y);
    heading_raw = SCALE_DEGREES_TO_RAW(y);  // include variation
  }
  bearing_raw = angle_raw + heading_raw;

  int a = (angle_raw + GARMIN_XHD_SPOKES * 2) % GARMIN_XHD_SPOKES;
  int b = (bearing_raw + GARMIN_XHD_SPOKES * 2) % GARMIN_XHD_SPOKES;;

  long long tpos;
  float lat, lon;
  state->get_position(tpos, lat, lon);
  radar_state->set_range(packet->range_meters);
  radar_image->set_spoke(tpos, lat, lon, b, packet->line_data, len, packet->range_meters);
  
}

SOCKET GarminxHDReceive::PickNextEthernetCard() {
  SOCKET socket = INVALID_SOCKET;
  memset(&m_interface_addr, 0, sizeof(m_interface_addr));

  // Pick the next ethernet card
  // If set, we used this one last time. Go to the next card.
  if (m_interface) {
    m_interface = m_interface->ifa_next;
  }
  // Loop until card with a valid IPv4 address
  while (m_interface && !VALID_IPV4_ADDRESS(m_interface)) {
    m_interface = m_interface->ifa_next;
  }
  if (!m_interface) {
    if (m_interface_array) {
      freeifaddrs(m_interface_array);
      m_interface_array = 0;
    }
    if (!getifaddrs(&m_interface_array)) {
      m_interface = m_interface_array;
    }
    // Loop until card with a valid IPv4 address
    while (m_interface && !VALID_IPV4_ADDRESS(m_interface)) {
      m_interface = m_interface->ifa_next;
    }
  }
  if (m_interface && VALID_IPV4_ADDRESS(m_interface)) {
    m_interface_addr.addr = ((struct sockaddr_in *)m_interface->ifa_addr)->sin_addr;
    m_interface_addr.port = 0;
  }

  socket = GetNewReportSocket();

  return socket;
}

SOCKET GarminxHDReceive::GetNewReportSocket() {
  SOCKET socket;
  std::string error;

  if (m_interface_addr.addr.s_addr == 0) {
    return INVALID_SOCKET;
  }

  error = ("");
  socket = startUDPMulticastReceiveSocket(m_interface_addr, m_report_addr, error);
  if (socket != INVALID_SOCKET) {
    std::string addr = m_interface_addr.FormatNetworkAddress();
    std::string rep_addr = m_report_addr.FormatNetworkAddressPort();

    printf(("radar_pi: %s scanning interface %s for data from %s"), pfilter->get_name(), addr.c_str(), rep_addr.c_str());

    printf("Scanning interface %s", addr.c_str());
  } else {
    printf(("radar_pi: Unable to listen to socket: %s"), error.c_str());
  }
  return socket;
}

SOCKET GarminxHDReceive::GetNewDataSocket() {
  SOCKET socket;
  std::string error;

  if (m_interface_addr.addr.s_addr == 0) {
    return INVALID_SOCKET;
  }

  printf(("%s data: "), pfilter->get_name());
  socket = startUDPMulticastReceiveSocket(m_interface_addr, m_data_addr, error);
  if (socket != INVALID_SOCKET) {
    std::string addr = m_interface_addr.FormatNetworkAddress();
    std::string rep_addr = m_data_addr.FormatNetworkAddressPort();

    printf(("radar_pi: %s listening for data on %s from %s"), pfilter->get_name(), addr.c_str(), rep_addr.c_str());
  } else {
    printf(("radar_pi: Unable to listen to socket: %s"), error.c_str());
  }
  return socket;
}


bool GarminxHDReceive::Init(ch_state * _state, ch_radar_state * _radar_state, ch_radar_image * _radar_image, NetworkAddress interfaceAddr)
{
  state = _state;
  radar_state = _radar_state;
  radar_image = _radar_image;
  
  no_data_timeout = 0;
  no_spoke_timeout = 0;
  
  m_interface_array = 0;
  m_interface = 0;
  radar_addr = 0;  
  dataSocket = INVALID_SOCKET;
  reportSocket = INVALID_SOCKET;

  /*
  printf(("radar_pi: GarminxHDReceive thread starting"));
  if(m_receive_socket == INVALID_SOCKET)
    return false;
  */
  if (m_interface_addr.addr.s_addr == 0) {
    reportSocket = GetNewReportSocket();
  }  
  return true;
}

bool GarminxHDReceive::Loop()
{
  int r = 0;
  union {
    sockaddr_storage addr;
    sockaddr_in ipv4;
  } rx_addr;
  socklen_t rx_len;
  uint8_t data[sizeof(radar_line)];

  /*
  if(m_receive_socket != INVALID_SOCKET)
    return false;
  */
  if (reportSocket == INVALID_SOCKET) {
    reportSocket = PickNextEthernetCard();
    if (reportSocket != INVALID_SOCKET) {
      no_data_timeout = 0;
      no_spoke_timeout = 0;
    }
  }
  
  if (radar_addr) {
    // If we have detected a radar antenna at this address start opening more sockets.
    // We do this later for 2 reasons:
    // - Resource consumption
    // - Timing. If we start processing radar data before the rest of the system
    //           is initialized then we get ordering/race condition issues.
    if (dataSocket == INVALID_SOCKET) {
      dataSocket = GetNewDataSocket();
    }
  } else {
    if (dataSocket != INVALID_SOCKET) {
      closesocket(dataSocket);
      dataSocket = INVALID_SOCKET;
    }
  }
  
  struct timeval tv = {(long)0, (long)(MILLIS_PER_SELECT * 1000)};
  
  fd_set fdin;
  FD_ZERO(&fdin);
  
  int maxFd = INVALID_SOCKET;
  /*
  if (m_receive_socket != INVALID_SOCKET) {
    FD_SET(m_receive_socket, &fdin);
    maxFd = max(m_receive_socket, maxFd);
  }
  */
  if (reportSocket != INVALID_SOCKET) {
    FD_SET(reportSocket, &fdin);
    maxFd = max(reportSocket, maxFd);
  }
  if (dataSocket != INVALID_SOCKET) {
    FD_SET(dataSocket, &fdin);
    maxFd = max(dataSocket, maxFd);
  }
  
  r = select(maxFd + 1, &fdin, 0, 0, &tv);


  if (r > 0) {
      /*
    if (m_receive_socket != INVALID_SOCKET
	&& FD_ISSET(m_receive_socket, &fdin)) {
      rx_len = sizeof(rx_addr);
      r = recvfrom(m_receive_socket, (char *)data,
		   sizeof(data), 0, (struct sockaddr *)&rx_addr, &rx_len);
      if (r > 0) {
	printf(("radar_pi: received stop instruction"));
	return false;
      }
    }
      */
    if (dataSocket != INVALID_SOCKET && FD_ISSET(dataSocket, &fdin)) {
      rx_len = sizeof(rx_addr);
      r = recvfrom(dataSocket, (char *)data, sizeof(data),
		   0, (struct sockaddr *)&rx_addr, &rx_len);
      if (r > 0) {
	ProcessFrame(data, (size_t)r);
	no_data_timeout = -15;
	no_spoke_timeout = -5;
      } else {
	closesocket(dataSocket);
	dataSocket = INVALID_SOCKET;
	printf(("radar_pi: %s illegal frame"), pfilter->get_name());
      }
    }
    
    if (reportSocket != INVALID_SOCKET && FD_ISSET(reportSocket, &fdin)) {
      rx_len = sizeof(rx_addr);
      r = recvfrom(reportSocket, (char *)data, sizeof(data),
		   0, (struct sockaddr *)&rx_addr, &rx_len);
      if (r > 0) {
	NetworkAddress radar_address;
	radar_address.addr = rx_addr.ipv4.sin_addr;
	radar_address.port = rx_addr.ipv4.sin_port;
	
	if (ProcessReport(data, (size_t)r)) {
	  if (!radar_addr) {

	    pfilter->DetectedRadar(radar_address);  // enables transmit data
	    
	    // the dataSocket is opened in the next loop
	    
	    radarFoundAddr = rx_addr.ipv4;
	    radar_addr = &radarFoundAddr;
	    m_addr = radar_address.FormatNetworkAddress();
	    
	    if (radar_state->get_state() == RADAR_OFF) {
	      printf(("radar_pi: %s detected at %s"), pfilter->get_name(), m_addr.c_str());
	      radar_state->set_state(RADAR_STANDBY);
	    }
	  }
	  no_data_timeout = SECONDS_SELECT(-15);
	}
      } else {
	printf(("radar_pi: %s illegal report"), pfilter->get_name());
	closesocket(reportSocket);
	reportSocket = INVALID_SOCKET;
      }
    }    
  } else {  // no data received -> select timeout    
    if (no_data_timeout >= SECONDS_SELECT(2)) {
      no_data_timeout = 0;
      if (reportSocket != INVALID_SOCKET) {
	closesocket(reportSocket);
	reportSocket = INVALID_SOCKET;
	radar_state->set_state(RADAR_OFF);
	memset(&m_interface_addr, 0, sizeof(m_interface_addr));
	radar_addr = 0;
      }
    } else {
      no_data_timeout++;
    }
    
    if (no_spoke_timeout >= SECONDS_SELECT(2)) {
      no_spoke_timeout = 0;
      radar_image->reset_image();
    } else {
      no_spoke_timeout++;
    }
  }

  if (reportSocket == INVALID_SOCKET) {
    // If we closed the reportSocket then close the command and data socket
    if (dataSocket != INVALID_SOCKET) {
      closesocket(dataSocket);
      dataSocket = INVALID_SOCKET;
    }
  }
  
  return true;
}

void GarminxHDReceive::Destroy()
{
  state = NULL;
  radar_state = NULL;
  radar_image = NULL;
  
  if (dataSocket != INVALID_SOCKET) {
    closesocket(dataSocket);
  }
  
  if (reportSocket != INVALID_SOCKET) {
    closesocket(reportSocket);
  }

  /*
  if (m_send_socket != INVALID_SOCKET) {
    closesocket(m_send_socket);
    m_send_socket = INVALID_SOCKET;
  }
  
  if (m_receive_socket != INVALID_SOCKET) {
    closesocket(m_receive_socket);
  }
  */

  if (m_interface_array) {
    freeifaddrs(m_interface_array);
  }
}

/*
 RADAR REPORTS

 The radars send various reports.

 */

#pragma pack(push, 1)

typedef struct {
  uint32_t packet_type;
  uint32_t len1;
  uint8_t parm1;
} rad_ctl_pkt_9;

typedef struct {
  uint32_t packet_type;
  uint32_t len1;
  uint16_t parm1;
} rad_ctl_pkt_10;

typedef struct {
  uint32_t packet_type;
  uint32_t len1;
  uint32_t parm1;
} rad_ctl_pkt_12;

typedef struct {
  uint32_t packet_type;
  uint32_t len1;
  uint32_t parm1;
  uint32_t parm2;
  uint16_t parm3;
  uint16_t parm4;
  uint8_t parm5;
  uint8_t parm6;
  uint16_t parm7;
} rad_respond_pkt_16;

typedef struct {
  uint32_t packet_type;
  uint32_t len1;
  uint16_t parm1;
  uint16_t parm2;
  uint32_t parm3;
  uint32_t parm4;
  uint32_t parm5;
  char info[64];
} rad_pkt_0x099b;

#pragma pack(pop)

bool GarminxHDReceive::UpdateScannerStatus(int status) {
  bool ret = true;

  if (status != m_radar_status) {
    m_radar_status = status;

    std::string stat;
    time_t now = time(0);

    switch (m_radar_status) {
      case 2:
	radar_state->set_state(RADAR_WARMING_UP);
        printf(("radar_pi: %s reports status WARMUP"), pfilter->get_name());
        stat = ("Warmup");
        break;
      case 3:
	radar_state->set_state(RADAR_STANDBY);
        printf(("radar_pi: %s reports status STANDBY"), pfilter->get_name());
        stat = ("Standby");
        break;
      case 4:
	radar_state->set_state(RADAR_SPINNING_UP);
        pfilter->m_data_timeout = now + DATA_TIMEOUT;
        printf(("radar_pi: %s reports status SPINNING UP"), pfilter->get_name());
        stat = ("Spinning up");
        break;
      case 5:
	radar_state->set_state(RADAR_TRANSMIT);
        printf(("radar_pi: %s reports status TRANSMIT"), pfilter->get_name());
        stat = ("Transmit");
        break;
      case 6:
	radar_state->set_state(RADAR_STOPPING);
        pfilter->m_data_timeout = now + DATA_TIMEOUT;
        printf(("radar_pi: %s reports status STOPPING"), pfilter->get_name());
        stat = ("Stopping");
        break;
      case 7:
	radar_state->set_state(RADAR_SPINNING_DOWN);
        printf(("radar_pi: %s reports status SPINNING DOWN"), pfilter->get_name());
        stat = ("Spinning down");
        break;
      case 10:
	radar_state->set_state(RADAR_STARTING);
        printf(("radar_pi: %s reports status STARTING"), pfilter->get_name());
        stat = ("Starting");
        break;
      default:
        printf(("radar_pi: %s reports status %d"), pfilter->get_name(), m_radar_status);
	{
	  stat = ("Unknown status");
	}	
        ret = false;
        break;
    }
    char buf[32];
    snprintf(buf, 32, ("IP %s %s"), m_addr.c_str(), stat.c_str());
    printf("%s", buf);
  }
  return ret;
}

bool GarminxHDReceive::ProcessReport(const uint8_t *report, size_t len) {
  
  time_t now = pfilter->get_time() / SEC;

  pfilter->resetTimeout(now);

  if (len >= sizeof(rad_ctl_pkt_9)) {  //  sizeof(rad_respond_pkt_9)) {
    rad_ctl_pkt_9 *packet9 = (rad_ctl_pkt_9 *)report;
    rad_ctl_pkt_10 *packet10 = (rad_ctl_pkt_10 *)report;
    rad_ctl_pkt_12 *packet12 = (rad_ctl_pkt_12 *)report;
    uint16_t packet_type = packet9->packet_type;

    switch (packet_type) {
      case 0x0916:  // Dome Speed
        printf(("radar_pi: Garmin xHD 0x0916: scan speed %d\n"), packet9->parm1);
        radar_state->set_scan_speed(packet9->parm1 >> 1);
        return true;

      case 0x0919:  // Standby/Transmit
                    // parm1 = 0 : Standby request
                    // parm1 = 1 : TX request
                    // Ignored, gxradar did nothing with this
        printf(("radar_pi: Garmin xHD 0x0919: standby/transmit %d\n"), packet9->parm1);
        return true;

      case 0x091e:  // Range
        printf(("radar_pi: Garmin xHD 0x091e: range %d\n"), packet12->parm1);
        radar_state->set_range(packet12->parm1);  // Range in meters
        return true;

        //
        // Garmin sends range in three separate packets, in the order 0x924, 0x925, 0x91d every
        // two seconds.
        // Auto High: 0x924 = 2, 0x925 = gain, 0x91d = 1
        // Auto Low:  0x924 = 2, 0x925 = gain, 0x91d = 0
        // Manual:    0x924 = 0, 0x925 = gain, 0x91d = 0 (could be last one used?)

      case 0x0924:  // AutoGain on/off
        printf(("radar_pi: Garmin xHD 0x0924: autogain %d\n"), packet9->parm1);
        m_auto_gain = packet9->parm1 > 0;
        return true;

      case 0x0925:  // Gain
        printf(("radar_pi: Garmin xHD 0x0925: gain %d\n"), packet10->parm1);
        m_gain = packet10->parm1 / 100;
        return true;

      case 0x091d: {  // Auto Gain Mode
        printf(("radar_pi: Garmin xHD 0x091d: auto-gain mode %d\n"), packet9->parm1);
        RadarControlState state = RCS_MANUAL;
        if (m_auto_gain) {
          switch (packet9->parm1) {
            case 0:
              state = RCS_AUTO_1;
              break;

            case 1:
              state = RCS_AUTO_2;  // AUTO HIGH
              break;

            default:
              break;
          }
        }
        printf(("radar_pi: %s m_gain.Update(%d, %d)\n"), pfilter->get_name(), m_gain, (int)state);
	radar_state->set_gain(m_gain, state);
        return true;
      }

      case 0x0930:  // Dome offset, called bearing alignment here
        printf(("radar_pi: Garmin xHD 0x0930: bearing alignment %d\n"), (int32_t)packet12->parm1 / 32);
	radar_state->set_bearing_alignment((int32_t)packet12->parm1 / 32);
        return true;

      case 0x0932:  // Crosstalk reject, I guess this is the same as interference rejection?
        printf(("radar_pi: Garmin xHD 0x0932: crosstalk/interference rejection %d\n"), packet9->parm1);
	radar_state->set_interference_rejection(packet9->parm1);
        return true;

      case 0x0933:  // Rain clutter mode
        printf(("radar_pi: Garmin xHD 0x0933: rain mode %d\n"), packet9->parm1);
        switch (packet9->parm1) {
          case 0: {
            m_rain_mode = RCS_OFF;
            return true;
          }
          case 1: {
            m_rain_mode = RCS_MANUAL;
            return true;
          }
        }
        break;

      case 0x0934: {
        // Rain clutter level
        printf(("radar_pi: Garmin xHD 0x0934: rain clutter %d\n"), packet10->parm1);
        m_rain_clutter = packet10->parm1 / 100;
	radar_state->set_rain(m_rain_clutter, m_rain_mode);
        return true;
      }

      case 0x0939: {
        // Sea Clutter On/Off
        printf(("radar_pi: Garmin xHD 0x0939: sea mode %d\n"), packet9->parm1);
        switch (packet9->parm1) {
          case 0: {
            m_sea_mode = RCS_OFF;
            return true;
          }
          case 1: {
            // Manual sea clutter, value set via report 0x093a
            m_sea_mode = RCS_MANUAL;
            return true;
          }
          case 2: {
            // Auto sea clutter, but don't set it if we already have a better state
            // via 0x093b
            if (m_sea_mode < RCS_AUTO_1) {
              m_sea_mode = RCS_AUTO_1;
            }
            return true;
          }
        }
        break;
      }

      case 0x093a: {
        // Sea Clutter level
        printf(("radar_pi: Garmin xHD 0x093a: sea clutter %d\n"), packet10->parm1);
        m_sea_clutter = packet10->parm1 / 100;
	radar_state->set_sea(m_sea_clutter, m_sea_mode);
        return true;
      }

      case 0x093b: {
        // Sea Clutter auto level
        printf(("radar_pi: Garmin xHD 0x093a: sea clutter auto %d\n"), packet9->parm1);
        if (m_sea_mode >= RCS_AUTO_1) {
          m_sea_mode = (RadarControlState)(RCS_AUTO_1 + packet9->parm1);
	  radar_state->set_sea(m_sea_clutter, m_sea_mode);
        }
        return true;
      }

      case 0x093f: {
        printf(("radar_pi: Garmin xHD 0x093a: no transmit zone mode %d\n"), packet9->parm1);
        m_no_transmit_zone_mode = packet9->parm1 > 0;
        // parm1 = 0 = Zone off, in that case we want AUTO_RANGE - 1 = 'Off'.
        // parm1 = 1 = Zone on, in that case we will receive 0x0940+0x0941.
        if (!m_no_transmit_zone_mode) {
	  radar_state->set_no_transmit_start(0, RCS_OFF);
	  radar_state->set_no_transmit_end(0, RCS_OFF);	  
        }
        return true;
      }
      case 0x0940: {
        printf(("radar_pi: Garmin xHD 0x0940: no transmit zone start %d\n"), packet12->parm1 / 32);
        if (m_no_transmit_zone_mode) {
	  radar_state->set_no_transmit_start(packet12->parm1 / 32, RCS_MANUAL);	  
        }
        return true;
      }
      case 0x0941: {
        printf(("radar_pi: Garmin xHD 0x0941: no transmit zone end %d\n"), (int32_t)packet12->parm1 / 32);
        if (m_no_transmit_zone_mode) {
	  radar_state->set_no_transmit_end(packet12->parm1 / 32, RCS_MANUAL);	  
        }
        return true;
      }
      case 0x02bb: {
        printf(("radar_pi: Garmin xHD 0x02bb: something %d\n"), (int32_t)packet12->parm1);
        return true;
      }
      case 0x02ec: {
        printf(("radar_pi: Garmin xHD 0x02ec: something %d\n"), (int32_t)packet12->parm1);
        return true;
      }
      case 0x0942: {
        printf(("radar_pi: Garmin xHD 0x0942: timed idle mode %d\n"), (int32_t)packet9->parm1);
        if (packet9->parm1 == 0) {
          m_timed_idle_mode = RCS_OFF;
        } else {
          m_timed_idle_mode = RCS_MANUAL;
        }
        return true;
      }

      case 0x0943: {
        printf(("radar_pi: Garmin xHD 0x0943: timed idle time %d s\n"), (int32_t)packet10->parm1);
	radar_state->set_timed_idle(packet10->parm1 / 60, m_timed_idle_mode);
        return true;
      }

      case 0x0944: {
        printf(("radar_pi: Garmin xHD 0x0944: timed run time %d s\n"), (int32_t)packet10->parm1);
	radar_state->set_timed_run((packet10->parm1 / 60));
        return true;
      }

      case 0x0992: {
        // Scanner state
        if (UpdateScannerStatus(packet9->parm1)) {
	  
          return true;
        }
      }

      case 0x0993: {
        // State change announce
        printf(("radar_pi: Garmin xHD 0x0993: state-change in %d ms\n"), packet12->parm1);
        radar_state->set_next_state_change(packet12->parm1 / 1000);
        return true;
      }

      case 0x099b: {
        rad_pkt_0x099b *packet = (rad_pkt_0x099b *)report;

        // Not sure that this always contains an error message
        // Observed with Timed Transmit (hardware control via plotter) it reports
        // 'State machine event fault - unhandled state transition request'

        printf(("radar_pi: Garmin xHD 0x099b: error '%s'\n"), packet->info);
        return true;
      }
    }
  }

  //  printf("radar_pi: Garmin xHD received unknown message\n");
  return false;
}
