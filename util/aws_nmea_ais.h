
#ifndef _AWS_NMEA_AIS_H_
#define _AWS_NMEA_AIS_H_
// Copyright(c) 2016 Yohei Matsumoto, All right reserved. 

// aws_nmea_ais.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// aws_nmea_ais.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with aws_nmea_ais.h.  If not, see <http://www.gnu.org/licenses/>. 

//s_binary_message helps generating AIS binary messages.
struct s_binary_message{
	int len; // bit length
	int sq; // sequence id
	int type; // message 8 or 14
	int ch; // channel 0/*auto*/, 1 /*A*/, 2 /*B*/, 3 /*A and B*/
	int txtseq;
	bool ackreq;
	unsigned int mmsi; // for message 6 or 12
	unsigned char msg[120]; // message up to 120 bytes
	char nmea[86]; // nmea buffer
	s_binary_message():len(0), sq(0), mmsi(0), type(8), ch(0), txtseq(0), ackreq(false){
	}

	~s_binary_message(){
	}

	// text DAC 1 FI 0
	bool set_msg_text(char * buf);

	// interpreting data in buf as hex data. The characters in buf should be 0 to 9 or a to f.
	bool set_msg_hex(char * buf);

	// interpreting data in buf as 6bit character string. The characters should be in the armoring table.
	bool set_msg_c6(char * buf);

	// interpreting data in buf as 8bit character string. Any data byte can be OK. 
	bool set_msg_c8(char * buf);

	// interpreting data in buf as binary string. The characters given in the buffer should be 0 or 1. 
	bool set_msg_bin(char * buf);

	// binary message pvc sends an absolue position and velosity as message 8
	// This message does not care about DAC/FI/AckReq/TextSeq 
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

	// pvc2 sends an absolute positon and velocity as message 8
	// DAC/FI/AckReq/TextSeq are set as zero.
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

	// pvc3 sends a relative position to the own ship and the velocity
	// DAC/FI/AckReq/TextSeq are set as zero
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

	// pvc4 sends an absolute position, the velocity and UTC sec as message 8
	// DAC/FI/AckReq/TextSeq are set as zero
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

	// pvc5 sends an relative position, the velocity and UTC sec as message 8
	// DAC/FI/AckReq/TextSeq are set as zero.
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

	// generating BBM or ABM nmea sentence. The results are stored in nmeas.
	// Note that multiple sentences can be produced.
	bool gen_nmea(const char * toker, vector<string> & nmeas);
};

char armor(char c);
unsigned char decchar(unsigned char c6);
unsigned char encchar(unsigned char c8);
const char * get_ship_type_name(unsigned char uc);


//////////////////////// c_vdm (from AIS)
class c_vdm;

struct s_pl{
	s_pl * pnext;
	char payload[256];
	int pl_size;
	short fcounts; // number of frangments
	short fnumber; // fragment number
	short seqmsgid; // sequential message id for multi sentence messaage
	bool is_chan_A; 
	short num_padded_zeros;
	bool cs; // check sum
	s_pl():pnext(NULL), pl_size(0), fcounts(0), fnumber(0),
		seqmsgid(-1), is_chan_A(true), 
		cs(false)
	{}

	bool is_complete()
	{
		return  fcounts == fnumber;
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

	virtual void dec_payload(s_pl * ppl)
	{
		m_is_chan_A = ppl->is_chan_A;
		char * dat = ppl->payload;
		m_repeate = (dat[1] & 0x30) >> 4;

		m_mmsi = ((dat[1] & 0x0F) << 26) |
			((dat[2] & 0x3F) << 20) | 
			((dat[3] & 0x3F) << 14) |
			((dat[4] & 0x3F) << 8) |
			((dat[5] & 0x3F) << 2) |
			((dat[6] & 0x30) >> 4);
	};

	virtual e_nd_type get_type() const
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
	virtual void dec_payload(s_pl * ppl);
	virtual ostream & show(ostream & out) const;
	static c_vdm_msg1 * dec_msg1(const char * str);
	virtual e_nd_type get_type() const
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

	virtual void dec_payload(s_pl * ppl);
	virtual ostream & show(ostream & out) const;
	static c_vdm_msg4 * dec_msg4(const char * str);
	virtual e_nd_type get_type() const
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

	virtual void dec_payload(s_pl * ppl);
	virtual ostream & show(ostream & out) const;
	static c_vdm_msg5 * dec_msg5(const char * str);
	virtual e_nd_type get_type() const
	{return ENDT_VDM5;};
};

class c_vdm_msg6: public c_vdm
{
public:
	unsigned int m_mmsi_dst;
	unsigned short m_dac;
	unsigned short m_fid;
	s_binary_message m_msg;
	unsigned short m_msg_size;
	virtual void dec_payload(s_pl * ppl);
	virtual ostream & show(ostream & out) const;
	static c_vdm_msg6 * dec_msg6(const char * str);
	virtual e_nd_type get_type() const
	{return ENDT_VDM6;}
};

class c_vdm_msg8: public c_vdm
{
public:
	unsigned short m_dac;
	unsigned short m_fid;
	s_binary_message m_msg;
//	char m_msg[128];
	unsigned short m_msg_size;
	virtual void dec_payload(s_pl * ppl);
	virtual ostream & show(ostream & out) const;
	static c_vdm_msg8 * dec_msg8(const char * str);
	virtual e_nd_type get_type() const
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

	virtual ostream & show(ostream & out) const;
	virtual void dec_payload(s_pl * ppl);
	virtual e_nd_type get_type() const 
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

	virtual ostream & show(ostream & out) const;
	virtual void dec_payload(s_pl * ppl);
	virtual e_nd_type get_type() const
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

	virtual ostream & show(ostream & out) const;
	virtual void dec_payload(s_pl * ppl);
	virtual e_nd_type get_type() const
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
	virtual ostream & show(ostream & out) const;
	virtual e_nd_type get_type() const
	{return ENDT_ABK;};
};

/////////////////////////////////////////////////////////// vdm decoder
class c_vdm_dec
{
protected:
	bool m_vdo;
	c_vdm_msg1 vdm_msg1;
	c_vdm_msg4 vdm_msg4;
	c_vdm_msg5 vdm_msg5;
	c_vdm_msg6 vdm_msg6;
	c_vdm_msg8 vdm_msg8;
	c_vdm_msg18 vdm_msg18;
	c_vdm_msg19 vdm_msg19;
	c_vdm_msg24 vdm_msg24;

	s_pl * m_pool;
	list<s_pl*> m_tmp; // temporaly store multi fragments message

	c_vdm_dec * m_pnext;

	char m_type; // message type

	
	c_vdm * dec_payload(s_pl * ppl);

	s_pl * alloc(){
		if(m_pool == NULL)
			return new s_pl;
		s_pl * tmp = m_pool;
		m_pool = m_pool->pnext;
		tmp->pl_size = 0;
		return tmp;
	}

	void free(s_pl * ptr){
		ptr->pnext = NULL;
		m_pool = ptr;
	}

	void clear();
public:
	c_vdm_dec():m_pool(NULL), m_vdo(false)
	{
	}

	~c_vdm_dec()
	{
		while(m_pool){
			s_pl * ppl = m_pool->pnext;
			delete m_pool;
		}
	}

	void set_vdo(){
		m_vdo = true;
	}

	c_vdm * dec(const char * str);
};

#endif
