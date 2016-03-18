// Copyright(c) 2012 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// ch_navdat.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// ch_navdat.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with ch_navdat.h.  If not, see <http://www.gnu.org/licenses/>. 

#ifndef _CH_NAVDAT_H_
#define _CH_NAVDAT_H_

#include "../util/aws_coord.h"

struct s_ship_ctrl{
	double	CPP_com		;	//- CPP翼角 指令値
	double	Rud_com		;	//- 舵角    指令値
	double	Resv1;
	double	Gov;
	double	B_th_com   	;	//- B/T翼角 指令値
	double	Ss_th_com	;	//- S/T回転数指令値(±1200rpm)
	double	Sp_th_com;
	double	Resv2		;	//- ガバナ用にキープ
	s_ship_ctrl(): CPP_com(-1.5), Rud_com(0.0),
		B_th_com(0.0), Ss_th_com(0.0), Sp_th_com(0.0){
	}

	~s_ship_ctrl(){
	}
};

class ch_ship_ctrl: public ch_base
{
public:
	ch_ship_ctrl(const char * name): ch_base(name)
	{
	}

	~ch_ship_ctrl()
	{
	}

	s_ship_ctrl m_ctrl;

	void get(s_ship_ctrl & ctrl){
		lock();
		ctrl = m_ctrl;
		unlock();
	}

	void set(s_ship_ctrl & ctrl){
		lock();
		m_ctrl = ctrl;
		unlock();
	}
};

class ch_navdat: public ch_base
{
public:
	char m_day, m_hour, m_min;
	float m_sec;
	bool m_valid;
	double m_cpp; // cpp 
	double m_rud; // rudder
	double m_bth; // bow thrust
	double m_sth; // starn thrust
	double m_emspd; // speed by emlog
	double m_windir; // wind direction
	double m_winspd; // wind speed
	double m_lat; // latitude
	double m_lon; // longitude
	double m_alt; // altitude
	double m_gpdir; // gps direction
	double m_gpspd; // gps speed
	double m_hdg; // heading (autopilot)
	double m_roll; // roll
	double m_pitch; // pitch
	double m_yaw; // yaw
	double m_arroll; //angular rate of roll
	double m_arpitch;// angular rate of pitch
	double m_aryaw;  // angular rate of yaw
	double m_aheave; // acceleration of heave
	double m_asway; // acceleration of sway
	double m_asurge; // acceleration of surge

	ch_navdat(const char * name):ch_base(name)
	{
	};
	virtual ~ch_navdat()
	{
	}

	bool is_valid()
	{return m_valid;}

	void set_done()
	{
		m_valid = true;
	}

	void get_done()
	{
		m_valid = false;
	}

	void dump(ostream & out)
	{
		out << "time:" << (int) m_day << "-" << (int) m_hour << ":" << (int) m_min << ":" << m_sec << endl;
		out << "cpp:" << m_cpp * (180 / PI) 
			<< "(deg),rad:" << m_rud * (180 / PI) << "(deg),bth:" 
			<< m_bth * (180 / PI) << "(deg),sth:" << m_sth << "(rpm)" << endl;
		out << "wind:" << m_windir * (180/PI) << "(deg)," << m_winspd << "m/s" << endl;
		/*
		out << "pos:" << m_lat * (180/PI) << "(deg)," 
			<< m_lon * (180/PI) << "(deg)," << m_alt << "(m)" << endl;
		out << "spd:" << m_emspd << "(knot eml)," << m_gpspd << "(knot gps)," << endl;
		out << "dir:" << m_hdg * (180/PI)  << "(deg gyro)," << m_gpdir * (180/PI)  << "(deg gps)" << endl;
		out << "rpy:" << m_roll * (180/PI) << "," << m_pitch * (180/PI) << "," << m_yaw * (180/PI) << "(deg)" << endl;
		out << "arrpy:"<< m_arroll * (180/PI) << "," << m_arpitch * (180/PI) << "," << m_aryaw * (180/PI) << "(deg/s)" << endl;
		out << "ahvswsg:" << m_aheave << "," << m_asway << "," << m_asurge << "(m/ss)" << endl;
		*/
	}
};

#endif