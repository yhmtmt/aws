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

#ifndef _GARMIN_HD_CONTROL_H_
#define _GARMIN_HD_CONTROL_H_

#include "socketutil.h"

class GarminHDControl : public RadarControl {
 public:
  GarminHDControl(NetworkAddress sendMultiCastAddress);
  ~GarminHDControl();

  bool Init(std::string & name, NetworkAddress &interfaceAddress, NetworkAddress &radarAddress);
  void RadarTxOff();
  void RadarTxOn();
  bool RadarStayAlive();
  bool SetRange(int meters);

  bool SetControlValue(ControlType controlType, RadarControlItem &item);

 private:
  void logBinaryData(std::string & what, const void *data, int size);
  bool TransmitCmd(const void *msg, int size);

  struct sockaddr_in m_addr;
  SOCKET m_radar_socket;
  std::string m_name;
};

#endif /* _GARMIN_HD_CONTROL_H_ */
