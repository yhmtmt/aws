/******************************************************************************
 *
 * Project:  OpenCPN
 * Purpose:  Radar Plugin
 * Author:   David Register
 *           Dave Cowell
 *           Kees Verruijt
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

#ifndef _GARMIN_HD_RECEIVE_H_
#define _GARMIN_HD_RECEIVE_H_

#include "../RadarReceive.h"
#include "../socketutil.h"


struct radar_line;

//
// An intermediary class that implements the common parts of any Navico radar.
//

class GarminHDReceive : public RadarReceive {
 public:
 GarminHDReceive(NetworkAddress interfaceAddr, NetworkAddress reportAddr, NetworkAddress dataAddr) : RadarReceive() {
    m_report_addr = reportAddr;
    m_next_spoke = -1;
    m_radar_status = 0;
    m_shutdown_time_requested = 0;
    m_is_shutdown = false;
    m_first_receive = true;
    m_interface_addr = interfaceAddr;
    m_receive_socket = GetLocalhostServerTCPSocket();
    m_send_socket = GetLocalhostSendTCPSocket(m_receive_socket);
    //    m_ri->m_showManualValueInAuto = true;

    printf(("radar_pi: %s receive thread created"), m_ri->m_name.c_str());
  };

  ~GarminHDReceive() {}

  void *Entry(void);
  void Shutdown(void);

  NetworkAddress m_interface_addr;
  NetworkAddress m_report_addr;

  wxLongLong m_shutdown_time_requested;  // Main thread asks this thread to stop
  volatile bool m_is_shutdown;

 private:
  void ProcessFrame(radar_line *packet);
  bool ProcessReport(const uint8_t *data, size_t len);

  SOCKET PickNextEthernetCard();
  SOCKET GetNewReportSocket();

  std::string m_ip;

  SOCKET m_receive_socket;  // Where we listen for message from m_send_socket
  SOCKET m_send_socket;     // A message to this socket will interrupt select() and allow immediate shutdown

  struct ifaddrs *m_interface_array;
  struct ifaddrs *m_interface;

  int m_next_spoke;
  int m_radar_status;
  bool m_first_receive;

  std::string m_addr;  // Radar's IP address

  bool m_auto_gain;               // True if auto gain mode is on
  int m_gain;                     // 0..100
  RadarControlState m_sea_mode;   // RCS_OFF, RCS_MANUAL, RCS_AUTO_1
  int m_sea_clutter;              // 0..100
  RadarControlState m_rain_mode;  // RCS_OFF, RCS_MANUAL, RCS_AUTO_1
  int m_rain_clutter;             // 0..100
  int m_no_spoke_timeout;

  bool UpdateScannerStatus(int status);

};

#endif /* _GARMIN_XH_RECEIVE_H_ */
