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

#include "GarminHDReceive.h"


/*
 * This file not only contains the radar receive threads, it is also
 * the only unit that understands what the radar returned data looks like.
 * The rest of the plugin uses a (slightly) abstract definition of the radar.
 */

#define MILLIS_PER_SELECT 250
#define SECONDS_SELECT(x) ((x)*MILLISECONDS_PER_SECOND / MILLIS_PER_SELECT)

//
//
#define SCALE_RAW_TO_DEGREES(raw) ((raw) * (double)DEGREES_PER_ROTATION / GARMIN_HD_SPOKES)
#define SCALE_DEGREES_TO_RAW(angle) ((int)((angle) * (double)GARMIN_HD_SPOKES / DEGREES_PER_ROTATION))

#pragma pack(push, 1)

struct radar_line {
  uint32_t packet_type;
  uint32_t len1;
  uint16_t angle;
  uint16_t scan_length;
  uint32_t display_meters;
  uint32_t range_meters;
  uint8_t gain_level[4];
  uint8_t sea_clutter[4];
  uint8_t rain_clutter[4];
  uint16_t dome_offset;
  uint8_t FTC_mode;
  uint8_t crosstalk_onoff;
  uint16_t fill_2;
  uint16_t fill_3;
  uint8_t timed_transmit[4];
  uint8_t dome_speed;
  uint8_t fill_4[7];
  uint8_t line_data[GARMIN_HD_MAX_SPOKE_LEN];
};

#pragma pack(pop)

// ProcessLine
// ------------
// Process one radar line, which contains exactly one line or spoke of data extending outwards
// from the radar up to the range indicated in the packet.
//
// Note that Garmin HD only has 1 bit per point, not 8 bits like most other radars.
//
void GarminHDReceive::ProcessFrame(radar_line *packet) {
  // log_line.time_rec = wxGetUTCTimeMillis();
  wxLongLong time_rec = wxGetUTCTimeMillis();
  time_t now = (time_t)(time_rec.GetValue() / MILLISECONDS_PER_SECOND);
  uint8_t line[GARMIN_HD_MAX_SPOKE_LEN];
  int i;
  uint8_t *p, *s;

  if (packet->scan_length * 8 > GARMIN_HD_MAX_SPOKE_LEN) {
    printf(("radar_pi: %s truncating data, %d longer than expected max length %d"), packet->scan_length * 8,
             GARMIN_HD_MAX_SPOKE_LEN);
    packet->scan_length = GARMIN_HD_MAX_SPOKE_LEN / 8;
  }
  for (p = line, s = packet->line_data, i = 0; i < packet->scan_length; i++, s++) {
    *p++ = (*s & 0x01) > 0 ? 255 : 0;
    *p++ = (*s & 0x02) > 0 ? 255 : 0;
    *p++ = (*s & 0x04) > 0 ? 255 : 0;
    *p++ = (*s & 0x08) > 0 ? 255 : 0;
    *p++ = (*s & 0x10) > 0 ? 255 : 0;
    *p++ = (*s & 0x20) > 0 ? 255 : 0;
    *p++ = (*s & 0x40) > 0 ? 255 : 0;
    *p++ = (*s & 0x80) > 0 ? 255 : 0;
  }

  m_ri->m_state.Update(RADAR_TRANSMIT);
  m_ri->m_range.Update(packet->range_meters);
  m_ri->m_gain.Update(packet->gain_level[0], packet->gain_level[1] ? RCS_AUTO_1 : RCS_MANUAL);
  m_ri->m_rain.Update(packet->sea_clutter[0], packet->sea_clutter[1] ? RCS_AUTO_1 : RCS_MANUAL);
  m_ri->m_rain.Update(packet->rain_clutter[0]);
  m_ri->m_bearing_alignment.Update(packet->dome_offset);
  m_ri->m_ftc.Update(packet->FTC_mode);
  m_ri->m_interference_rejection.Update(packet->crosstalk_onoff);
  m_ri->m_scan_speed.Update(packet->dome_speed);

  wxCriticalSectionLocker lock(m_ri->m_exclusive);

  m_ri->m_radar_timeout = now + WATCHDOG_TIMEOUT;
  m_ri->m_data_timeout = now + DATA_TIMEOUT;

  if (m_first_receive) {
    m_first_receive = false;
    wxLongLong startup_elapsed = wxGetUTCTimeMillis() - m_pi->GetBootMillis();
    printf(("radar_pi: %s first radar spoke received after %llu ms\n"), m_ri->m_name.c_str(), startup_elapsed);
  }

  int angle_raw = packet->angle;
  int spoke = angle_raw;  // Garmin does not have radar heading, so there is no difference between spoke and angle
  m_ri->m_statistics.spokes++;
  if (m_next_spoke >= 0 && spoke != m_next_spoke) {
    if (spoke > m_next_spoke) {
      m_ri->m_statistics.missing_spokes += spoke - m_next_spoke;
    } else {
      m_ri->m_statistics.missing_spokes += GARMIN_HD_SPOKES + spoke - m_next_spoke;
    }
  }

  m_next_spoke = (spoke + 1) % GARMIN_HD_SPOKES;

  short int heading_raw = 0;
  int bearing_raw;

  heading_raw = SCALE_DEGREES_TO_RAW(m_pi->GetHeadingTrue());  // include variation
  bearing_raw = angle_raw + heading_raw;

  SpokeBearing a = MOD_SPOKES(angle_raw);
  SpokeBearing b = MOD_SPOKES(bearing_raw);

  m_ri->ProcessRadarSpoke(a, b, line, p - line, packet->display_meters, time_rec);
}

SOCKET GarminHDReceive::PickNextEthernetCard() {
  SOCKET socket = INVALID_SOCKET;
  CLEAR_STRUCT(m_interface_addr);

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

SOCKET GarminHDReceive::GetNewReportSocket() {
  SOCKET socket;
  wxString error;

  if (m_interface_addr.addr.s_addr == 0) {
    return INVALID_SOCKET;
  }

  error = ("");
  socket = startUDPMulticastReceiveSocket(m_interface_addr, m_report_addr, error);
  if (socket != INVALID_SOCKET) {
    wxString addr = m_interface_addr.FormatNetworkAddress();
    wxString rep_addr = m_report_addr.FormatNetworkAddressPort();

    printf(("radar_pi: %s scanning interface %s for data from %s"), m_ri->m_name.c_str(), addr.c_str(), rep_addr.c_str());

    wxString s;
    s << ("Scanning interface") << (" ") << addr;
    SetInfoStatus(s);
  } else {
    SetInfoStatus(error);
    printf(("radar_pi: Unable to listen to socket: %s"), error.c_str());
  }
  return socket;
}

/*
 * Entry
 *
 * Called by wxThread when the new thread is running.
 * It should remain running until Shutdown is called.
 */
void *GarminHDReceive::Entry(void) {
  int r = 0;
  int no_data_timeout = 0;
  union {
    sockaddr_storage addr;
    sockaddr_in ipv4;
  } rx_addr;
  socklen_t rx_len;

  uint8_t data[sizeof(radar_line)];
  m_interface_array = 0;
  m_interface = 0;
  m_no_spoke_timeout = 0;
  struct sockaddr_in radarFoundAddr;
  sockaddr_in *radar_addr = 0;

  SOCKET reportSocket = INVALID_SOCKET;

  printf(("radar_pi: GarminHDReceive thread %s starting"), m_ri->m_name.c_str());

  if (m_interface_addr.addr.s_addr == 0) {
    reportSocket = GetNewReportSocket();
  }

  while (m_receive_socket != INVALID_SOCKET) {
    if (reportSocket == INVALID_SOCKET) {
      reportSocket = PickNextEthernetCard();
      if (reportSocket != INVALID_SOCKET) {
        no_data_timeout = 0;
        m_no_spoke_timeout = 0;
      }
    }

    struct timeval tv = {(long)0, (long)(MILLIS_PER_SELECT * 1000)};

    fd_set fdin;
    FD_ZERO(&fdin);

    int maxFd = INVALID_SOCKET;
    if (m_receive_socket != INVALID_SOCKET) {
      FD_SET(m_receive_socket, &fdin);
      maxFd = MAX(m_receive_socket, maxFd);
    }
    if (reportSocket != INVALID_SOCKET) {
      FD_SET(reportSocket, &fdin);
      maxFd = MAX(reportSocket, maxFd);
    }

    r = select(maxFd + 1, &fdin, 0, 0, &tv);

    if (r > 0) {
      if (m_receive_socket != INVALID_SOCKET && FD_ISSET(m_receive_socket, &fdin)) {
        rx_len = sizeof(rx_addr);
        r = recvfrom(m_receive_socket, (char *)data, sizeof(data), 0, (struct sockaddr *)&rx_addr, &rx_len);
        if (r > 0) {
          printf(("radar_pi: %s received stop instruction"), m_ri->m_name.c_str());
          break;
        }
      }

      if (reportSocket != INVALID_SOCKET && FD_ISSET(reportSocket, &fdin)) {
        rx_len = sizeof(rx_addr);
        r = recvfrom(reportSocket, (char *)data, sizeof(data), 0, (struct sockaddr *)&rx_addr, &rx_len);
        if (r > 0) {
          NetworkAddress radar_address;
          radar_address.addr = rx_addr.ipv4.sin_addr;
          radar_address.port = rx_addr.ipv4.sin_port;

          if (ProcessReport(data, (size_t)r)) {
            if (!radar_addr) {
              wxCriticalSectionLocker lock(m_lock);
              m_ri->DetectedRadar(m_interface_addr, radar_address);  // enables transmit data

              // the dataSocket is opened in the next loop

              radarFoundAddr = rx_addr.ipv4;
              radar_addr = &radarFoundAddr;
              m_addr = radar_address.FormatNetworkAddress();

              if (m_ri->m_state.GetValue() == RADAR_OFF) {
                printf(("radar_pi: %s detected at %s"), m_ri->m_name.c_str(), m_addr.c_str());
                m_ri->m_state.Update(RADAR_STANDBY);
              }
            }
            no_data_timeout = SECONDS_SELECT(-15);
          }
        } else {
          printf(("radar_pi: %s illegal report"), m_ri->m_name.c_str());
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
          m_ri->m_state.Update(RADAR_OFF);
          CLEAR_STRUCT(m_interface_addr);
          radar_addr = 0;
        }
      } else {
        no_data_timeout++;
      }

      if (m_no_spoke_timeout >= SECONDS_SELECT(2)) {
        m_no_spoke_timeout = 0;
        m_ri->ResetRadarImage();
      } else {
        m_no_spoke_timeout++;
      }
    }

  }  // endless loop until thread destroy

  if (reportSocket != INVALID_SOCKET) {
    closesocket(reportSocket);
  }
  if (m_send_socket != INVALID_SOCKET) {
    closesocket(m_send_socket);
    m_send_socket = INVALID_SOCKET;
  }
  if (m_receive_socket != INVALID_SOCKET) {
    closesocket(m_receive_socket);
  }

  if (m_interface_array) {
    freeifaddrs(m_interface_array);
  }

#ifdef TEST_THREAD_RACES
  printf(("radar_pi: %s receive thread sleeping"), m_ri->m_name.c_str());
  wxMilliSleep(1000);
#endif
  printf(("radar_pi: %s receive thread stopping"), m_ri->m_name.c_str());
  m_is_shutdown = true;
  return 0;
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
  uint16_t scanner_state;
  uint16_t warmup;
  uint32_t range_meters;
  uint8_t gain_level;
  uint8_t gain_mode;
  uint16_t fill_1;
  uint8_t sea_clutter_level;
  uint8_t sea_clutter_mode;
  uint16_t fill_2;
  uint8_t rain_clutter_level;
  uint8_t fill_3[3];
  int16_t dome_offset;
  uint8_t FTC_mode;
  uint8_t crosstalk_onoff;
  uint16_t fill_4[2];
  uint8_t timed_transmit_mode;
  uint8_t timed_transmit_transmit;
  uint8_t timed_transmit_standby;
  uint8_t fill_5;
  uint8_t dome_speed;
  uint8_t fill_6[7];
} rad_response_pkt;

#pragma pack(pop)

bool GarminHDReceive::UpdateScannerStatus(int status) {
  bool ret = true;

  if (status != m_radar_status) {
    m_radar_status = status;

    std::string stat;
    time_t now = time(0);

    switch (m_radar_status) {
      case 1:
        m_ri->m_sta () te.Update(RADAR_WARMING_UP);
        printf(("radar_pi: %s reports status WARMUP"), m_ri->m_name.c_str());
        stat = ("Warmup");
        break;
      case 3:
        m_ri->m_state.Update(RADAR_STANDBY);
        printf(("radar_pi: %s reports status STANDBY"), m_ri->m_name.c_str());
        stat = ("Standby");
        break;
      case 5:
        m_ri->m_state.Update(RADAR_SPINNING_UP);
        m_ri->m_data_timeout = now + DATA_TIMEOUT;
        printf(("radar_pi: %s reports status SPINNING UP"), m_ri->m_name.c_str());
        stat = ("Spinning up");
        break;
      case 4:
        m_ri->m_state.Update(RADAR_TRANSMIT);
        printf(("radar_pi: %s reports status TRANSMIT"), m_ri->m_name.c_str());
        stat = ("Transmit");
        break;
      default:
	{
	  char buf [32];
	  snprintf(buf, 32, " %d", m_radar_status);
	  std::string(buf);
	  stat << ("Unknown status") << buf;
	}
        ret = false;
        break;
    }
    char buf[32];
    snprintf(buf, 32, ("IP %s %s"), m_addr.c_str(), stat.c_str());
    SetInfoStatus(std::string(buf));
  }
  return ret;
}

bool GarminHDReceive::ProcessReport(const uint8_t *report, size_t len) {
  logBinaryData(("ProcessReport"), report, len);

  time_t now = time(0);

  m_ri->resetTimeout(now);

  if (len >= sizeof(rad_response_pkt)) {  //  sizeof(rad_response_pkt)) {
    rad_response_pkt *packet = (rad_response_pkt *)report;
    uint16_t packet_type = packet->packet_type;

    switch (packet_type) {
      case 0x2a3: {
        radar_line *line = (radar_line *)report;

        ProcessFrame(line);
        m_no_spoke_timeout = -5;
        return true;
      }

      case 0x2a5: {
        // Scanner state
        if (!UpdateScannerStatus(packet->scanner_state)) {
          return false;
        }
        printf(("0x02a5: next-state-change %d"), packet->warmup);
        m_ri->m_next_state_change.Update(packet->warmup);
        return true;
      }

      case 0x2a7: {
        printf(("0x02a7: range %d"), packet->range_meters);  // Range in meters
        m_ri->m_range.Update(packet->range_meters);

        printf(("0x02a7: gain %d"), packet->gain_level);  // Gain
        m_gain = packet->gain_level;

        printf(("0x02a7: auto-gain mode %d"), packet->gain_mode);  // Auto Gain Mode
        RadarControlState state = RCS_MANUAL;
        if (m_auto_gain) {
          switch (packet->gain_mode) {
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
        printf(("radar_pi: %s m_gain.Update(%d, %d)"), m_ri->m_name.c_str(), m_gain, (int)state);
        m_ri->m_gain.Update(m_gain, state);

        // Sea Clutter level
        printf(("0x02a7: sea clutter %d"), packet->sea_clutter_level);
        m_sea_clutter = packet->sea_clutter_level;
        m_ri->m_sea.Update(m_sea_clutter, m_sea_mode);

        // Sea Clutter On/Off
        printf(("0x02a7: sea mode %d"), packet->sea_clutter_mode);
        switch (packet->sea_clutter_mode) {
          case 0: {
            m_sea_mode = RCS_OFF;
          }
          case 1: {
            // Manual sea clutter, value set via report 0x093a
            m_sea_mode = RCS_MANUAL;
          }
          case 2: {
            // Auto sea clutter, but don't set it if we already have a better state
            if (m_sea_mode < RCS_AUTO_1) {
              m_sea_mode = RCS_AUTO_1;
            }
          }
          default:
            break;
        }

        // Rain clutter level
        printf(("0x02a7: rain clutter %d"), packet->rain_clutter_level);
        m_rain_clutter = packet->rain_clutter_level;
        m_ri->m_rain.Update(m_rain_clutter, m_rain_mode);

        // Dome offset, called bearing alignment here
        printf(("0x02a7: bearing alignment %d"), (int32_t)packet->dome_offset);
        m_ri->m_bearing_alignment.Update((int32_t)packet->dome_offset);

        // FTC mode
        printf(("0x02a7: FTC %d"), packet->FTC_mode);
        m_ri->m_ftc.Update(packet->FTC_mode);

        // Crosstalk reject, I guess this is the same as interference rejection?
        printf(("0x02a7: crosstalk/interference rejection %d"), packet->crosstalk_onoff);
        m_ri->m_interference_rejection.Update(packet->crosstalk_onoff);

        // Timed transmit status should go here

        // Dome Speed
        printf(("0x02a7: scan speed %d"), packet->dome_speed);
        m_ri->m_scan_speed.Update(packet->dome_speed);

        return true;
      }
    }
  }

  logBinarydata(("received unknown message"), report, len);
  return false;
}

// Called from the main thread to stop this thread.
// We send a simple one byte message to the thread so that it awakens from the select() call with
// this message ready for it to be read on 'm_receive_socket'. See the constructor in GarminHDReceive.h
// for the setup of these two sockets.

void GarminHDReceive::Shutdown() {
  if (m_send_socket != INVALID_SOCKET) {
    m_shutdown_time_requested = wxGetUTCTimeMillis();
    if (send(m_send_socket, "!", 1, MSG_DONTROUTE) > 0) {
      printf(("radar_pi: %s requested receive thread to stop"), m_ri->m_name.c_str());
      return;
    }
  }
  printf(("radar_pi: %s receive thread will take long time to stop"), m_ri->m_name.c_str());
}

