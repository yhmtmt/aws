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

#ifndef _NAVICORECEIVE_H_
#define _NAVICORECEIVE_H_

#include "NavicoCommon.h"
#include "NavicoLocate.h"
#include "RadarReceive.h"
#include "socketutil.h"

//
// An intermediary class that implements the common parts of any Navico radar.
//

class NavicoReceive : public RadarReceive {
 public:
  NavicoReceive(radar_pi *pi, RadarInfo *ri, NetworkAddress reportAddr, NetworkAddress dataAddr, NetworkAddress sendAddr)
      : RadarReceive(pi, ri) {
    m_data_addr = dataAddr;
    m_report_addr = reportAddr;
    m_send_addr = sendAddr;
    m_next_spoke = -1;
    m_radar_status = 0;
    m_shutdown_time_requested = 0;
    m_is_shutdown = false;
    m_first_receive = true;
    m_interface_addr = m_pi->GetRadarInterfaceAddress(ri->m_radar);
    m_receive_socket = GetLocalhostServerTCPSocket();
    m_send_socket = GetLocalhostSendTCPSocket(m_receive_socket);
    char buf[32];
    snprintf(buf, 32, "%s: %s", m_ri->m_name.c_str(), _("Initializing"));
    SetInfoStatus(std::string(buf));
    SetPriority(wxPRIORITY_MAX);
    printf(("radar_pi: %s receive thread created, prio= %i"), m_ri->m_name.c_str(), GetPriority());
    InitializeLookupData();

    NavicoRadarInfo info = m_pi->GetNavicoRadarInfo(m_ri->m_radar);

    if (info.report_addr.IsNull() && !m_data_addr.IsNull()) {
      // BR24, 3G, 4G initial setup, when ini file doesn't contain multicast addresses yet
      // In this case m_data_addr etc. are correct, these don't really change in the wild according to our data,
      // so write them into the NavicoRadarInfo object.
      info.spoke_data_addr = m_data_addr;
      info.report_addr = m_report_addr;
      info.send_command_addr = m_send_addr;

      m_pi->SetNavicoRadarInfo(m_ri->m_radar, info);
    } else if (!info.report_addr.IsNull() && m_data_addr.IsNull()) {
      // HALO restart, when ini file contains multicast addresses, that are hopefully still correct.
      // If not we will time-out and then NavicoLocate will find the radar.
      info.spoke_data_addr = m_data_addr;
      info.report_addr = m_report_addr;
      info.send_command_addr = m_send_addr;

      m_pi->SetNavicoRadarInfo(m_ri->m_radar, info);
    }
  };

  ~NavicoReceive(){};

  void InitializeLookupData();

  void *Entry(void);
  void Shutdown(void);
  std::string GetInfoStatus();

  NetworkAddress m_interface_addr;
  NetworkAddress m_data_addr;
  NetworkAddress m_report_addr;
  NetworkAddress m_send_addr;

  wxLongLong m_shutdown_time_requested;  // Main thread asks this thread to stop
  volatile bool m_is_shutdown;

 private:
  void ProcessFrame(const uint8_t *data, size_t len);
  bool ProcessReport(const uint8_t *data, size_t len);

  SOCKET PickNextEthernetCard();
  SOCKET GetNewReportSocket();
  SOCKET GetNewDataSocket();

  void UpdateSendCommand();
  void SetRadarType(RadarType t);

  SOCKET m_receive_socket;  // Where we listen for message from m_send_socket
  SOCKET m_send_socket;     // A message to this socket will interrupt select() and allow immediate shutdown

  struct ifaddrs *m_interface_array;
  struct ifaddrs *m_interface;

  int m_next_spoke;
  char m_radar_status;
  bool m_first_receive;

  wxCriticalSection m_lock;  // Protects m_status
  std::string m_status;         // Userfriendly string
  std::string m_firmware;       // Userfriendly string #2

  void SetInfoStatus(std::string status) {
    wxCriticalSectionLocker lock(m_lock);
    m_status = status;
  }
  void SetFirmware(std::string s) {
    wxCriticalSectionLocker lock(m_lock);
    m_firmware = s;
  }
};

PLUGIN_END_NAMESPACE

#endif /* _NAVICORECEIVE_H_ */
