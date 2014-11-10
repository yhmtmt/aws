// Copyright(c) 2012 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// c_nmeadec.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// c_nmeadec.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with c_nmeadec.h.  If not, see <http://www.gnu.org/licenses/>. 

enum e_gp_dir{
	EGP_N, EGP_S, EGP_E, EGP_W
};

enum e_gp_fix_stat{
	EGPF_LOST, EGPF_GPSF, EGPF_DGPSF
};

enum e_nd_type{
	ENDT_GGA, ENDT_RMC, ENDT_ZDA,
	ENDT_TTM,
	ENDT_VDM, ENDT_VDM1, ENDT_VDM4, ENDT_VDM5,
	ENDT_VDM8, ENDT_VDM18, ENDT_VDM19, ENDT_VDM24,
	ENDT_ABK
};


struct s_binary_message{
	int len; // bit length
	int sq; // sequence id
	int id; // message 8 or 14
	int ch; // channel 0/*auto*/, 1 /*A*/, 2 /*B*/, 3 /*A and B*/
	unsigned int mmsi; // for message 6 or 12
	unsigned char msg[120]; // message up to 120 bytes
	s_binary_message():len(0), sq(0), mmsi(0), id(0), ch(0){
	}

	~s_binary_message(){
	}

	bool set_msg_hex(char * buf);
	bool set_msg_c6(char * buf);
	bool set_msg_c8(char * buf);
	bool set_msg_bin(char * buf);
	bool set_msg_pvc(
					unsigned char id, /* 8bit */
					unsigned short sog /* 10bit 0.1 knot / unit */,
					unsigned short cog /* 12bit 0.1 deg / unit */,
					int lon /* 28bit 10000 minutes  = 1/600000 deg /unit*/ , 
					int lat /* 27bit 10000 minutes  = 1/600000 deg /unit*/);
	bool get_msg_pvc(
					unsigned char & id, /* 8bit */
					unsigned short & sog,
					unsigned short & cog,
					int & lon, 
					int & lat);
	bool set_msg_pvc2(
					unsigned char id, /* 8bit */
					unsigned short sog /* 10bit 0.1 knot / unit */,
					unsigned short cog /* 12bit 0.1 deg / unit */,
					int lon /* 28bit 10000 minutes  = 1/600000 deg /unit*/ , 
					int lat /* 27bit 10000 minutes  = 1/600000 deg /unit*/);
	bool get_msg_pvc2(
					unsigned char & id, /* 8bit */
					unsigned short & sog,
					unsigned short & cog,
					int & lon, 
					int & lat);
	bool set_msg_pvc3(
					unsigned char id, /* 8bit */
					unsigned short sog /* 10bit 0.1 knot / unit */,
					unsigned short cog /* 12bit 0.1 deg / unit */,
					unsigned short dist /* 10bit 0.1 mile / unit */,
					unsigned short bear /* 12bit 0.1 deg / unit */);
	bool get_msg_pvc3(
					unsigned char & id, /* 8bit */
					unsigned short & sog /* 10bit 0.1 knot / unit */,
					unsigned short & cog /* 12bit 0.1 deg / unit */,
					unsigned short & dist /* 10bit 0.1 mile / unit */,
					unsigned short & bear /* 12bit 0.1 deg / unit */);

	bool set_msg_pvc4(
					unsigned char id, /* 8bit */
					unsigned short sog /* 10bit 0.1 knot / unit */,
					unsigned short cog /* 12bit 0.1 deg / unit */,
					int lon /* 28bit 10000 minutes  = 1/600000 deg /unit*/ , 
					int lat /* 27bit 10000 minutes  = 1/600000 deg /unit*/,
					unsigned char sec /* 6bit 1sec / unit */);

	bool get_msg_pvc4(
					unsigned char & id, /* 8bit */
					unsigned short & sog,
					unsigned short & cog,
					int & lon, 
					int & lat,
					unsigned char & sec /* 6bit 1sec / unit */);

	bool set_msg_pvc5(
					unsigned char id, /* 8bit */
					unsigned short sog /* 10bit 0.1 knot / unit */,
					unsigned short cog /* 12bit 0.1 deg / unit */,
					unsigned short dist /* 10bit 0.1 mile / unit */,
					unsigned short bear /* 12bit 0.1 deg / unit */,
					unsigned char sec /* 6bit 1sec / unit */);
	bool get_msg_pvc5(
					unsigned char & id, /* 8bit */
					unsigned short & sog /* 10bit 0.1 knot / unit */,
					unsigned short & cog /* 12bit 0.1 deg / unit */,
					unsigned short & dist /* 10bit 0.1 mile / unit */,
					unsigned short & bear /* 12bit 0.1 deg / unit */,
					unsigned char & sec /* 6bit 1sec / unit */);

};

const char * get_ship_type_name(unsigned char uc);

int enc_cbr(int cbr);

/////////////////////////// c_nmea_dat and its inheritant
class c_nmea_dat
{
public:
	char m_toker[2];
	bool m_cs;
	static c_nmea_dat * dec_nmea_dat(const char * str);
	virtual ostream & show(ostream & out) = 0;
	virtual e_nd_type get_type() = 0;
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
/////////////////////// c_ttm (from ARPA)
class c_ttm: public c_nmea_dat
{
public:
	short m_id;				// field1 the target number 0 to 99 (but we found 100 in JRC's ARPA)
	float m_dist;			// field 2 the target distance (mile or kilometer, specified later)
	float m_bear;			// field 3 the target bearing from own ship (in degree)
	bool m_is_bear_true;	// field 4 true if target bearing is true direction otherwise relative direction
	float m_spd;			// field 5 the target speed in knot or km/h.
	float m_crs;			// field 6 the target course
	float m_is_crs_true;	// field 7 true if the target cousre is true direction otherwise relative direction
	float m_dcpa;			// field 8 Distance of closest point of approach.
	float m_tcpa;			// field 9 Time of closest point of approach
	char m_dist_unit;		// field 10 Distance unit. K=Kilometer, N=Nautical-mile, S=Statute-mile
	char m_data[20];		// field 11 User data (typicaly a name)
	char m_state;			// field 12 Tracking status. L=Lost, Q=Under Acquisition, T=Under Tracking
	bool m_is_ref;			// field 13 Reference target flag
	char m_utc_h, m_utc_m, m_utc_s, m_utc_ms;			 
							// field 14 UTC hour, minute, second, milisecond
	bool m_is_auto;			// field 15 true if the target is automaticaly acquisited

	static c_nmea_dat * dec_ttm(const char * str);

	virtual ostream & show(ostream & out){
		out << "TTM>";
		out << " ID:" << m_id 
			<< " DIST:" << m_dist << "(" << m_dist_unit << "M)"
			<< " BEAR:" << m_bear << (m_is_bear_true ? "(deg, T)" : "(deg, R)")
			<< " SPD:" << m_spd << (m_dist_unit == 'K' ? "(K/h)" : (m_dist_unit == 'N' ? "(KT)" : (m_dist_unit == 'S' ? "(MPH)" : "(???)")))
			<< " CRS:" << m_crs << (m_is_crs_true ? "(deg, T)" : "(deg, R)")
			<< " DCPA:" << m_dcpa << "(" << m_dist_unit << "M)"
			<< " TCPA:" << m_tcpa << "(MIN)"
			<< " DATA:" << m_data
			<< " STATE:" << (m_state == 'L' ? "LOST": (m_state == 'Q' ? "ACQ" : (m_state == 'T' ? "TRC" : "???")))
			<< " REF:" << (m_is_ref ? "TRUE":"FALSE")
			<< " UTC:" << (int) m_utc_h << ":" << (int) m_utc_m << ":" << (int) m_utc_s << "." << (int) m_utc_ms
			<< " AUTO:" << (m_is_auto ? "TRUE":"FALSE") << endl;
		return out;
	}

	virtual e_nd_type get_type()
	{return ENDT_TTM;};
};


//////////////////////// c_vdm (from AIS)
class c_vdm;

struct s_vdm_pl
{
	static vector<s_vdm_pl*> m_tmp; // temporaly store multi fragments message
	static s_vdm_pl * m_pool;
	s_vdm_pl * m_pnext;

	short m_fcounts; // number of frangments
	short m_fnumber; // fragment number
	short m_seqmsgid; // sequential message id for multi sentence messaage
	int m_pl_size;
	char m_payload[256];
	bool m_is_chan_A; 
	short m_num_padded_zeros;
	bool m_cs; // check sum

	char m_type; // message type
	s_vdm_pl():m_pnext(NULL), m_fcounts(0), m_fnumber(0),
		m_seqmsgid(-1), m_pl_size(0),  m_is_chan_A(true), 
		m_cs(false)
	{}

	static void clear();

	c_vdm * dec_payload();

	bool is_complete()
	{
		return  m_fcounts == m_fnumber;
	}

	static s_vdm_pl * alloc(){
		if(m_pool == NULL)
			return new s_vdm_pl;
		s_vdm_pl * tmp = m_pool;
		m_pool = m_pool->m_pnext;
		tmp->m_pl_size = 0;
		return tmp;
	}

	static void free(s_vdm_pl * ptr){
		if(m_pool == NULL)
			m_pool = ptr;
		else{
			ptr->m_pnext = m_pool;
			m_pool = ptr;
		}
	}

	void dearmor(const char * str);
};


class c_vdm: public c_nmea_dat
{
public:
	bool m_vdo;
	char m_repeate; // repeate indicator
	unsigned int m_mmsi; // mmsi number
	bool m_is_chan_A;

	c_vdm(): m_vdo(false), m_repeate(0), m_mmsi(0), m_is_chan_A(true)
	{
	}

	~c_vdm()
	{
	}

	virtual void dec_payload(s_vdm_pl * ppl)
	{
		m_is_chan_A = ppl->m_is_chan_A;
		char * dat = ppl->m_payload;
		m_repeate = (dat[1] & 0x30) >> 4;

		m_mmsi = ((dat[1] & 0x0F) << 26) |
			((dat[2] & 0x3F) << 20) | 
			((dat[3] & 0x3F) << 14) |
			((dat[4] & 0x3F) << 8) |
			((dat[5] & 0x3F) << 2) |
			((dat[6] & 0x30) >> 4);
	};

	static c_vdm * dec_vdm(const char * str);

	virtual e_nd_type get_type()
	{return ENDT_VDM;};
};


class c_vdm_msg1:public c_vdm
{// class A position report
public:
	char m_status; // navigation status
	float m_turn; // Rate of Turn degrees/min
	float m_speed; // knot
	char m_accuracy; // DGPS = 1, GPS = 0
	float m_lon; // degree
	float m_lat; // degree
	float m_course; // xxx.x degree
	unsigned short m_heading; // degree
	unsigned char m_second; // UTC second
	char m_maneuver; // 0:na 1: no special 2:special
	char m_raim; // RAIM flag
	unsigned int m_radio; // radio status
	virtual void dec_payload(s_vdm_pl * ppl);
	virtual ostream & show(ostream & out);
	static c_vdm_msg1 * dec_msg1(const char * str);
	virtual e_nd_type get_type()
	{return ENDT_VDM1;};
};

class c_vdm_msg4: public c_vdm
{// class A base station report
public:
	unsigned short m_year;
	char m_month, m_day, m_hour, m_minute;
	char m_accuracy; // DGPS = 1, GPS = 0
	float m_lon; // degree
	float m_lat; // degree
	unsigned char m_second; // UTC second
	char m_epfd; // positioning device
	char m_raim; // RAIM flag
	unsigned int m_radio; // radio status

	virtual void dec_payload(s_vdm_pl * ppl);
	virtual ostream & show(ostream & out);
	static c_vdm_msg4 * dec_msg4(const char * str);
	virtual e_nd_type get_type()
	{return ENDT_VDM4;};
};

class c_vdm_msg5: public c_vdm
{// class A static information
public:
	char m_ais_version;
	unsigned int m_imo;
	unsigned char m_callsign[8];
	unsigned char m_shipname[21];
	unsigned char m_shiptype;
	short m_to_bow, m_to_stern;
	unsigned char m_to_port, m_to_starboard;
	char m_epfd;
	unsigned short m_year;
	char m_month, m_day, m_hour, m_minute;
	float m_draught;
	bool m_dte;
	unsigned char m_destination[21];

	virtual void dec_payload(s_vdm_pl * ppl);
	virtual ostream & show(ostream & out);
	static c_vdm_msg5 * dec_msg5(const char * str);
	virtual e_nd_type get_type()
	{return ENDT_VDM5;};
};

class c_vdm_msg8: public c_vdm
{
public:
	unsigned short m_dac;
	unsigned short m_fid;
	s_binary_message m_msg;
//	char m_msg[128];
	unsigned short m_msg_size;
	virtual void dec_payload(s_vdm_pl * ppl);
	virtual ostream & show(ostream & out);
	static c_vdm_msg8 * dec_msg8(const char * str);
	virtual e_nd_type get_type()
	{return ENDT_VDM8;};
};

class c_vdm_msg18: public c_vdm
{// class B position report
public:
	float m_speed; // knot
	char m_accuracy; // DGPS = 1, GPS = 0
	float m_lon; // degree
	float m_lat; // degree
	float m_course; // xxx.x degree
	unsigned short m_heading; // degree
	unsigned char m_second; // UTC second
	bool m_cs;
	bool m_disp;
	bool m_dsc;
	bool m_band;
	bool m_msg22;
	bool m_assigned;
	bool m_raim; 
	unsigned int m_radio; // radio status

	virtual ostream & show(ostream & out);
	virtual void dec_payload(s_vdm_pl * ppl);
	virtual e_nd_type get_type()
	{return ENDT_VDM18;};
};

class c_vdm_msg19: public c_vdm
{// class B position report ex
public:
	float m_speed; // knot
	char m_accuracy; // DGPS = 1, GPS = 0
	float m_lon; // degree
	float m_lat; // degree
	float m_course; // xxx.x degree
	unsigned short m_heading; // degree
	unsigned char m_second; // UTC second
	bool m_assigned;
	bool m_raim; 
	bool m_dte;
	unsigned int m_radio; // radio status
	unsigned char m_shipname[21];
	unsigned char m_shiptype;
	short m_to_bow, m_to_stern;
	char m_epfd;

	unsigned char m_to_port, m_to_starboard;

	virtual ostream & show(ostream & out);
	virtual void dec_payload(s_vdm_pl * ppl);
	virtual e_nd_type get_type()
	{return ENDT_VDM19;};
};

class c_vdm_msg24: public c_vdm
{// class B static information
public:
	unsigned char m_part_no;
	unsigned char m_shipname[21];

	unsigned char m_shiptype;
	unsigned char m_vendorid[8];
	unsigned char m_callsign[8];

	short m_to_bow, m_to_stern;
	unsigned char m_to_port, m_to_starboard;
	unsigned int m_ms_mmsi;

	char m_epfd;

	virtual ostream & show(ostream & out);
	virtual void dec_payload(s_vdm_pl * ppl);
	virtual e_nd_type get_type()
	{return ENDT_VDM24;};
};

//////////////////////// c_abk
class c_abk: public c_nmea_dat
{
public:
	unsigned char m_cs; // check sum
	unsigned int m_mmsi;
	enum e_recv_chan{ NA, CHA, CHB} m_rch;
	unsigned short m_msg_id;
	unsigned short m_seq;
	unsigned short m_stat;

	c_abk(): m_mmsi(0), m_rch(NA), m_msg_id(0), m_seq(0), m_stat(0){};
	~c_abk(){};

	static c_nmea_dat * dec_abk(const char * str);
	virtual ostream & show(ostream & out);
	virtual e_nd_type get_type()
	{return ENDT_ABK;};
};


//////////////////////// c_nmeadec
// c_nmeadec decode nmea data from files, COM ports, and UDP.
#ifndef _WIN32
#include <termios.h>
#endif

class c_nmeadec{
private:
	enum e_nmea_src{NONE, FILE, COM, UDP} m_nmea_src;

	ofstream m_file_log;
	int m_buf_tail, m_buf_head;
	char m_tmp_buf[1024];
	int m_dat_head;
	int m_num_dats;
	int m_num_max_dats;
	vector<c_nmea_dat *> m_dats;

	// nmea in file
	ifstream m_file;

#ifdef _WIN32
	HANDLE m_hcom;
	DCB m_dcb;
	COMMTIMEOUTS m_timeout;
#else /* for linux */
	int m_hcom;
	termios m_copt;
#endif

	SOCKET m_sock;
	sockaddr_in m_sock_addr;

	bool m_exit;
	pthread_t m_loop;
	pthread_cond_t m_cond;
	pthread_mutex_t m_mtx;
	static void * loop(void * p);
	long long m_sys_time;
	char m_m, m_h;
	float m_s;

	// binary message related members
	int m_len_msg;
	char m_msg[120]; // binariy message buffer
	char m_toker[2];
	int m_seq_id;
	bool send_binary_message(unsigned int mmsi_dst, 
		int id /* 8 6 12 14*/, 
		int chan /*0, 1, 2, 3*/,
		const unsigned char * buf, int bits);
public:
	c_nmeadec();
	~c_nmeadec();

	bool is_exit();

	bool open(const char * fname);
	bool open(int icom, int cbr);
	bool open(unsigned short port);
	void close();

	bool enable_log(const char * fname);
	void disable_log();

	void set_toker(char toker[2])
	{m_toker[0] = toker[0]; m_toker[1] = toker[1];};

	int send(const char * str_nmea);
	bool send_binary_message(s_binary_message & msg);

	bool getline(char * buf, size_t size);
	void push(c_nmea_dat * pnd);
	c_nmea_dat * get_dat(long long cur_time = -1);
	bool seek(long long seek_time);
};

char armor(char c);
unsigned char calc_nmea_chksum(const char * str);
