#ifndef _AWS_NMEA_GPS_H_
#define _AWS_NMEA_GPS_H_
// Copyright(c) 2016 Yohei Matsumoto, All right reserved. 

// aws_nmea_gps.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// aws_nmea_gps.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with aws_nmea_gps.h.  If not, see <http://www.gnu.org/licenses/>. 


enum e_gp_dir{
	EGP_N, EGP_S, EGP_E, EGP_W
};

enum e_gp_fix_stat{
	EGPF_LOST, EGPF_GPSF, EGPF_DGPSF, EGPF_PPS, EGPF_RTK, EGPF_FRTK, EGPF_ESTM, EGPF_MAN, EGPF_SIM
};

// class for $gga
class c_gga: public c_nmea_dat
{
public:
	short m_h, m_m;
	float m_s;
	e_gp_fix_stat m_fix;
	short m_num_sats;
	double m_lon_deg/* (deg) */, m_lat_deg/* (deg) */;
	e_gp_dir m_lon_dir, m_lat_dir;
	float m_hdop, m_alt/* (m) */, m_geos/* (m) */, m_dgps_age /* (sec) */;
	short m_dgps_station;

	c_gga(): m_h(0), m_m(0), m_s(0.), m_fix(EGPF_LOST), 
		m_num_sats(0), m_lon_deg(0.), m_lat_deg(0.),
		m_lon_dir(EGP_N), m_lat_dir(EGP_E), m_hdop(0), m_alt(0.),
		m_geos(0.), m_dgps_age(0.), m_dgps_station(-1)
	{
	}

	static c_nmea_dat * dec_gga(const char * str);
	virtual ostream & show(ostream & out){
		out << "GPGGA >";
		out << " UTC:" << m_h << ":" << m_m << ":" << m_s << endl;
		out << " SAT:" << m_num_sats;
		out << " POS(lt, ln, h):" 
			<< m_lat_deg << (m_lat_dir == EGP_N ? "N":"S") << ","
			<< m_lon_deg << (m_lon_dir == EGP_E ? "E":"W") << "," 
			<< m_alt << "(" << m_geos << ")";
		out << " HDOP:" << m_hdop;
		out << " DGPS ID:" << m_dgps_station 
			<< " DGPS Age:" << m_dgps_age;
		out << " CS:" << (m_cs ? "yes" : "no") << endl;
		return out;
	}
	virtual e_nd_type get_type()
	{return ENDT_GGA;};
};

class c_gsa: public c_nmea_dat
{
public:
	unsigned char s3d2d; // Selection of the measurement mode {1: Auto, 2: Manual}
	unsigned char mm; // Measurement mode {1: No Measurement, 2: 2D, 3: 3D}
	unsigned short sused[12]; // Satellited used for the calculation (upto 12 sats)
	float pdop, hdop, vdop;

	c_gsa():s3d2d(0), mm(0), pdop(0.), hdop(0.), vdop(0.)
	{
		for(int i = 0; i< 12; i++)
			sused[i] = 0;
	}

	static c_nmea_dat * dec_gsa(const char * str);
	virtual ostream & show(ostream & out){
		return out;
	};
	virtual e_nd_type get_type(){
		return ENDT_GSA;
	};
};

class c_gsv: public c_nmea_dat
{
public:
	unsigned char ns /* number of sentence */, si /* sentence index */;
	unsigned short nsats_usable; // number of usable satellite.
	unsigned short sat[4], el[4], az[4], sn[4];
	c_gsv():ns(0), si(0), nsats_usable(0)
	{
		for(int i = 0; i < 4; i++){
			sat[i] = el[i] = sn[i] = 0;
		}
	}

	static c_nmea_dat * dec_gsv(const char * str);
	virtual ostream & show(ostream & out){
		return out;
	};
	virtual e_nd_type get_type(){
		return ENDT_GSV;
	};
};

// class for $rmc
class c_rmc: public c_nmea_dat
{
public:
	short m_h, m_m;
	float m_s;
	bool m_v;
	double m_lon_deg/* (deg) */, m_lat_deg/* (deg) */;
	e_gp_dir m_lon_dir, m_lat_dir;
	double m_vel, m_crs, m_crs_var;
	short m_yr, m_mn, m_dy;
	e_gp_dir m_crs_var_dir;

	c_rmc(): m_h(0), m_m(0), m_s(0.0), m_v(false),
		m_lon_deg(0.0), m_lat_deg(0.0),
		m_lon_dir(EGP_E), m_lat_dir(EGP_N),
		m_vel(0.0), m_crs(0.0), m_crs_var(0.0),
		m_yr(0), m_mn(0), m_dy(0), m_crs_var_dir(EGP_E)
	{
	}

	static c_nmea_dat * dec_rmc(const char * str);
	virtual ostream & show(ostream & out){
		out << "GPRMC>";
		out << " UTC:" << m_dy << "," << m_mn << "," << m_yr << "," << m_h << ":" << m_m << ":" << m_s;
		out << " Pos(lt, ln, h):" 
			<< m_lat_deg << (m_lat_dir == EGP_N ? "N":"S") << ","
			<< m_lon_deg << (m_lon_dir == EGP_E ? "E":"W");
		out << " CRS(var):" << m_crs << "(" << m_crs_var << (m_crs_var_dir == EGP_E ? "E":"W") << ")";
		out << " SPD:" << m_vel;
		out << " CS:" << (m_cs ? "yes" : "no") << endl;
		return out;
	}
	virtual e_nd_type get_type()
	{return ENDT_RMC;};
};

class c_vtg: public c_nmea_dat
{
public:
	float crs_t, crs_m, v_n, v_k;
	e_gp_fix_stat fm;

	c_vtg():crs_t(0.), crs_m(0.), v_n(0.), v_k(0.)
	{
	}

	static c_nmea_dat * dec_vtg(const char * str);
	virtual ostream & show(ostream & out){
		return out;
	};
	virtual e_nd_type get_type(){
		return ENDT_VTG;
	};
};

class c_zda: public c_nmea_dat
{
public:
	short m_h, m_m;
	float m_s;
	short m_dy, m_mn, m_yr;
	short m_lzh, m_lzm;

	c_zda(): m_h(0), m_m(0), m_s(0), m_dy(0), m_mn(0),
		m_yr(0), m_lzh(0), m_lzm(0)
	{
	}

	static c_nmea_dat * dec_zda(const char * str);

	virtual ostream & show(ostream & out){
		out << "ZDA>";
		out << " UTC:" << m_dy << "," << m_mn << "," << m_yr << "," << m_h << ":" << m_m << ":" << m_s;
		out << " CS:" << (m_cs ? "yes" : "no") << endl;
		return out;
	}
	virtual e_nd_type get_type()
	{return ENDT_ZDA;};
};

#endif
