// Copyright(c) 2013 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// f_com.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_com.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_com.h.  If not, see <http://www.gnu.org/licenses/>. 

#ifndef _F_COM_H_
#define _F_COM_H_

#include "../channel/ch_image.h"
#include "../channel/ch_vector.h"

#include "f_base.h"

// data recorder interfacing ring buffer ch_ring<char>
class f_rec_data: public f_base{
private:
	ch_ring<char> * m_pin;
	char m_fname[1024];
	int m_buf_size;
	char * m_buf;
	ofstream m_flog;
	bool m_bdump;

public:
	f_rec_data(const char * name): f_base(name), m_pin(NULL), m_buf_size(1024), m_buf(NULL), m_bdump(true)
	{
		m_fname[0] = '\0';
		register_fpar("fname", m_fname, 1024, "File name to save the recieved data.");
		register_fpar("buf_size", &m_buf_size, "Buffer size.");
		register_fpar("dump", &m_bdump, "Dump recieved data to STDOUT.");
	}

	virtual ~f_rec_data()
	{
	}

	virtual bool init_run()
	{
		if(m_fname[0] != '\0'){
			m_flog.open(m_fname);
			if(!m_flog.is_open())
				return false;
		}

		if(m_chin.size() != 1)
			return false;

		m_pin = dynamic_cast<ch_ring<char>*>(m_chin[0]);
		if(m_pin == NULL)
			return false;

		m_buf = new char[m_buf_size];
		if(m_buf == NULL){
			return false;
		}
		return true;
	}

	virtual void destroy_run()
	{
		if(m_flog.is_open())
			m_flog.close();
		delete[] m_buf;
		m_buf = NULL;
	}

	virtual bool proc()
	{
		int len;
		while(len = m_pin->read(m_buf, m_buf_size-1)){
			if(m_bdump && len != 0){
				m_buf[len] = '\0';

				cout << m_time_str;
				cout.write(m_buf, len);
				cout << endl;
			}
			if(m_flog.is_open()){
				// DATA RECORD 
				//   8byte  4byte len byte
				// [ TIME ][ LEN ][ DATA]
				m_flog.write((const char*)&m_cur_time, sizeof(m_cur_time));
				m_flog.write((const char*)&len, sizeof(len));
				m_flog.write((const char*)m_buf, len);
			}
		}
		return true;

	}
};

// Dummy data generator interfacing ring buffer ch_ring<char>
class f_dummy_data: public f_base{
private:
	ch_ring<char> * m_pout;
	unsigned int m_len_pkt;
	char m_fname_data[1024];
	ifstream m_fdata;
	enum e_data_type {
		EDT_RAND /* only random number */, EDT_TIME/* only time */,
		EDT_BYTE_SEQ, /* byte sequence */
		EDT_ASCII_SEQ, /* ASCII sequence */
		EDT_TIME_RAND /* Both time and random number */,
		EDT_FILE /* Data from file */, 
		EDT_TIME_FILE /* data from file with time stamp */
	} m_dtype;

	static const char * m_str_edt[EDT_TIME_FILE+1];
	long long m_total;

	unsigned char * m_buf;
	int m_head_buf, m_tail_buf;
public:
	f_dummy_data(const char * name):f_base(name), m_pout(NULL), m_len_pkt(1024), m_dtype(EDT_RAND),
		m_buf(NULL), m_head_buf(0), m_tail_buf(0), m_total(0)
	{
		register_fpar("lpkt", &m_len_pkt, "Length of the packet.");
		register_fpar("type", (int*)&m_dtype, EDT_TIME_FILE+1, m_str_edt, "Data type.");
		register_fpar("fdata", m_fname_data, 1024, "File path of the data.");
	}

	virtual ~f_dummy_data()
	{
	}

	virtual bool init_run();

	virtual void destroy_run();

	virtual bool proc();
};

// Serial communication filter interfacing ring buffer.
class f_serial: public f_base
{
private:
	ch_ring<char> * m_pin, * m_pout;

	AWS_SERIAL m_hserial;
	char m_dname[1024];
	unsigned short m_port;
	unsigned int m_br;
	unsigned int m_frm_len;

	// buffers
	char * m_rbuf, * m_wbuf;
	int m_head_rbuf, m_head_wbuf;
	int m_tail_rbuf, m_tail_wbuf;
	int m_len_buf;
public:
	f_serial(const char *fname):f_base(fname), m_hserial(NULL_SERIAL), m_pin(NULL), m_pout(NULL), m_port(1),
		m_br(9600), m_frm_len(256), m_rbuf(NULL), m_wbuf(NULL), m_head_rbuf(0), m_head_wbuf(0), m_tail_rbuf(0),
		m_tail_wbuf(0), m_len_buf(0)
	{
		m_dname[0] = '\0';
		register_fpar("dev", m_dname, 1024, "Device file path of the serial port to be opened.");
		register_fpar("port", &m_port, "Port number of the serial port to be opened. (for Windows)");
		register_fpar("br", &m_br, "Baud rate.");
	}

	virtual ~f_serial()
	{
	}

	virtual bool init_run();

	virtual void destroy_run();

	virtual bool proc();
};

class f_write_ch_log: public f_base
{
private:
	char m_path[1024];
	bool m_verb;
	bool m_benable, m_bopened;
	vector<FILE *> m_logs;
	vector<size_t> m_szs;
	size_t m_max_size;
	bool open_log(const int ich);
	bool open_logs();
	bool close_log(const int ich);
	void close_logs();
public:
	f_write_ch_log(const char * fname) : f_base(fname), m_verb(false), m_max_size(1024 * 1024 * 1024), m_benable(true)
	{
		m_path[0] = '.';m_path[1] = '\0';
		register_fpar("path", m_path, 1024, "Storage path for logging");
		register_fpar("sz_max", (int*)&m_max_size, "Maximum size of a log file.");
		register_fpar("enable", &m_benable, "Enable logging.");
	}

	virtual ~f_write_ch_log()
	{
	}

	virtual bool init_run();
	virtual void destroy_run();
	virtual bool proc();
};

class f_read_ch_log: public f_base
{
private:
	char m_path[1024];
	bool m_verb;
	vector<FILE *> m_logs;
	vector<long long> m_ts, m_te;
	bool open_log(const int och);

public:
	f_read_ch_log(const char * fname): f_base(fname), m_verb(false)
	{
		m_path[0] = '.';m_path[1] = '\0';
		register_fpar("path", m_path, 1024, "Storage path for logging");
	}

	virtual ~f_read_ch_log()
	{
	}

	virtual bool init_run();
	virtual void destroy_run();
	virtual bool proc();
};


class f_ch_share: public f_base
{
 private:
  char m_host_dst[1024];
  unsigned short m_port, m_port_dst;
  int m_len_pkt_snd, m_len_pkt_rcv;
  SOCKET m_sock;
  sockaddr_in m_sock_addr_snd, m_sock_addr_rcv;
  socklen_t m_sz_sock_addr_snd;
  bool m_svr;
  bool m_client_fixed;
  bool m_verb;

  // buffers
  char * m_rbuf, * m_wbuf;
  int m_rbuf_head, m_wbuf_head;
  int m_rbuf_tail, m_wbuf_tail;
  int m_len_buf;
  
  // log file
  char m_fname_out[1024];
  ofstream m_fout;
  char m_fname_in[1024];
  ofstream m_fin;

  long long m_tshare;
 public:
	 f_ch_share(const char * fname) : f_base(fname), m_verb(false),
		 m_port(20100), m_port_dst(20101),
		 m_len_pkt_snd(1024), m_len_pkt_rcv(1024), m_sock(-1),
		 m_svr(false),
		 m_rbuf(NULL), m_wbuf(NULL), m_rbuf_head(0), m_wbuf_head(0),
		 m_rbuf_tail(0), m_wbuf_tail(0), m_len_buf(0), m_tshare(0)
	 {
		 m_fname_out[0] = '\0';
		 m_fname_in[0] = '\0';
		 m_host_dst[0] = '\0';
		 register_fpar("verb", &m_verb, "For debug.");
		 register_fpar("port", &m_port, "UDP port.");
		 register_fpar("port_dst", &m_port_dst, "Destination UDP port.");
		 register_fpar("host_dst", m_host_dst, 1024, "Host address.");
		 //register_fpar("lpkt", &m_len_pkt, "Packet length");
		 register_fpar("fout", m_fname_out, 1024, "Output log file.");
		 register_fpar("fin", m_fname_in, 1024, "Input log file.");
		 register_fpar("tshare", &m_tshare, "Time data sharing");
	 }
  
  virtual ~f_ch_share()
    {
    }
  
  virtual bool init_run();
  virtual void destroy_run();
  virtual bool proc();
};

// udp communication filter interfacing ch_ring<char> input/output channels
class f_udp: public f_base
{
private:
	ch_ring<char> * m_pin, * m_pout;
	char m_host_dst[1024];
	unsigned short m_port, m_port_dst;
	int m_len_pkt;
	SOCKET m_sock;
	sockaddr_in m_sock_addr_snd, m_sock_addr_rcv;
	
	// buffers
	char * m_rbuf, * m_wbuf;
	int m_head_rbuf, m_head_wbuf;
	int m_tail_rbuf, m_tail_wbuf;
	int m_len_buf;

	// log file
	char m_fname_out[1024];
	ofstream m_fout;
	char m_fname_in[1024];
	ofstream m_fin;
public:
	f_udp(const char * fname): f_base(fname), m_pin(NULL), m_pout(NULL),
		m_port(20100), m_port_dst(20101), m_len_pkt(1024), m_sock(-1),
		m_rbuf(NULL), m_wbuf(NULL), m_head_rbuf(0), m_head_wbuf(0), 
		m_tail_rbuf(0), m_tail_wbuf(0), m_len_buf(0)
	{
		m_fname_out[0] = '\0';
		m_fname_in[0] = '\0';
		register_fpar("port", &m_port, "UDP port.");
		register_fpar("port_dst", &m_port_dst, "Destination UDP port.");
		register_fpar("host_dst", m_host_dst, 1024, "Host address.");
		register_fpar("lpkt", &m_len_pkt, "Packet length");
		register_fpar("fout", m_fname_out, 1024, "Output log file.");
		register_fpar("fin", m_fname_in, 1024, "Input log file.");
	}

	virtual ~f_udp()
	{
	}

	virtual bool init_run();
	virtual void destroy_run();
	virtual bool proc();
};


// filter not used
class f_trn: public f_base{
protected:
	SOCKET m_sock;
	sockaddr_in m_sock_addr;
public:
	f_trn(const char * name): f_base(name){
	}
	virtual ~f_trn(){
	}

	virtual bool init_run()
	{
		m_sock = socket(AF_INET, SOCK_DGRAM, 0);	
		m_sock_addr.sin_family =AF_INET;
		m_sock_addr.sin_port = htons(20001);
		set_sockaddr_addr(m_sock_addr, "127.0.0.1");
		/*
		if(bind(m_sock, (sockaddr*) &m_sock_addr, sizeof(m_sock_addr)) ==
			SOCKET_ERROR)
			return false;
			*/
		return true;
	}

	virtual void int_destroy(){
		closesocket(m_sock);
	}

	virtual bool proc()
	{
		const char * buf = "Hello World";
		int len = (int) strlen(buf) + 1;
		int len_send;
	//	len_send = send(m_sock, buf, len, 0);
		len_send = sendto(m_sock, buf, len, 0, (sockaddr*) &m_sock_addr, sizeof(m_sock_addr));
		if(len_send != len){
			cerr << "Failed to send text length " << len_send  << "/" << len << endl;
		}
		return true;
	}
};

// filter not used
class f_rcv: public f_base{
protected:
	SOCKET m_sock;
	sockaddr_in m_sock_addr;
public:
	f_rcv(const char * name): f_base(name){
	}

	virtual ~f_rcv(){
	}

	virtual bool init_run(){
		m_sock = socket(AF_INET, SOCK_DGRAM, 0);	
		m_sock_addr.sin_family =AF_INET;
		m_sock_addr.sin_port = htons(20001);
		set_sockaddr_addr(m_sock_addr);
		if(::bind(m_sock, (sockaddr*)&m_sock_addr, sizeof(m_sock_addr)) == SOCKET_ERROR){
			cerr << "Socket error" << endl;
			return false;
		}

		return true;
	}

	virtual void int_destroy(){
		closesocket(m_sock);
	}

	virtual bool proc(){
		char buf[1024];
		int sz = (int) sizeof(m_sock_addr);
		int len = recv(m_sock, buf, sizeof(buf), 0);
		if(len <= 0){
			cerr << "Failed to receive text" << endl;
		}else{
			cout << buf << endl;
		}

		return true;
	}
};

////////////////////////////////////////////// transfer / receive image data (cv::Mat)
struct s_img_pkt{
	char code[4];
	unsigned int id;

	s_img_pkt():id(1){
		code[0] = 'f';
		code[1] = 'i';
		code[2] = 'm';
		code[3] = 'g';
	}

	bool check(){
		return code[0] == 'f' && code[1] == 'i' && code[2] == 'm' && code[3] == 'g';
	}
};

struct s_img_pkt0{
	char code[4];
	unsigned int id;
	unsigned int len;
	uchar fmt;
	uchar cfmt;
	int type;
	unsigned int w, h;
	s_img_pkt0()
	{
	};

	s_img_pkt0(unsigned int alen,int atype, uchar afmt, uchar acfmt, 
		unsigned int aw, unsigned int ah){
		code[0] = 'f';
		code[1] = 'i';
		code[2] = 'm';
		code[3] = 'g';
		id = 0;
		len = alen;
		fmt = afmt;
		cfmt = acfmt;
		type = atype;
		w = aw;
		h = ah;
	}

	~s_img_pkt0(){
	}

	bool check(){
		return code[0] == 'f' && code[1] == 'i' && code[2] == 'm' && code[3] == 'g' && id == 0;
	}
};

// f_trn_img transmits Mat typed image data by TCP. 
// TCP server is up after invoking filter thread. Data is transmitted after connection
// established. The data can be compressed as JPEG or PNG. The compression algorithm
// is automatically detected at connecting f_rcv_img object.
class f_trn_img: public f_base
{
private:
	ch_image * m_pimgin;

	// configurable parameters
	int m_svr_port;		   // destination port

	int m_fmt;		// image format 0: raw, 1: jpg 
	int m_depth;	// in byte
	int m_channel;	// number of color channels
	int m_cfmt;		// 0: Mono 1: Bayer 2: RGB
	int m_qjpeg;	// jpeg quality 0 to 100
	int m_qpng;		// png quality 0 to 9
	float m_scale;

	// socket
	SOCKET m_sock_svr;
	sockaddr_in m_sock_svr_addr;
	SOCKET m_sock_client;		
	sockaddr_in m_sock_client_addr;
#ifdef _WIN32
	int m_sz_sock_client_addr;
#else
	socklen_t m_sz_sock_client_addr;
#endif

	bool wait_connection();
	void disconnect()
	{
		closesocket(m_sock_client);
		m_sock_client = 0;
		m_bconnected = false;
	}
	bool m_bconnected;
public:
	f_trn_img(const char * fname): f_base(fname), m_pimgin(NULL),
		m_svr_port(22222),
		m_fmt(1), m_depth(1), m_channel(1),
		m_cfmt(0), m_qjpeg(100), m_qpng(0), m_scale(1.0)
		, m_sock_client(0), m_bconnected(false)
	{

		// parameters for connection
		register_fpar("port", &m_svr_port, "Destination port number");

		// parameters for image data (these are sent to the destination)
		register_fpar("fmt", &m_fmt, "Image data format {0: raw 1: jpg 2: png}");
		register_fpar("depth", &m_depth, "Color depth in byte}");
		register_fpar("channel", &m_channel, "Number of color channels");
		register_fpar("cfmt", &m_cfmt, "Color format {0: Mono, 1: Bayer, 2:RGB}");
		register_fpar("qjpg", &m_qjpeg, "Jpeg quality [0-100]");
		register_fpar("qpng", &m_qpng, "PNG quality [0-10]");

		// parameters for data modification
		register_fpar("scale", &m_scale, "Scale for resizing.");
	}

	virtual const char * get_err_msg(int code);

	virtual ~f_trn_img(){
	}

	virtual bool init_run();
	virtual void destroy_run();

	virtual bool proc();
};

// f_rcv_img recieves Mat image transmitted by f_trn_img object. 
class f_rcv_img: public f_base
{
private:
	ch_image * m_pimgout;

	char m_svr_addr[1024];
	int m_svr_port;

	// these parameters are same as f_trn_img
	int m_fmt;
	int m_depth;
	int m_channel;
	int m_cfmt;

	// socket
	SOCKET m_sock;		
	sockaddr_in m_sock_addr;

	bool try_connection();
	void disconnect()
	{		
		closesocket(m_sock);
		m_sock = 0;
		m_bconnected = false;
	}
	bool m_bconnected;
public:
	f_rcv_img(const char * fname): f_base(fname),m_pimgout(NULL), 
		m_svr_port(22222), m_fmt(1), m_bconnected(false)
	{
		strncpy(m_svr_addr, "127.0.0.1", sizeof(m_svr_addr) - 1);

		// parameters for connection
		register_fpar("addr", m_svr_addr, 1024, "Server address (in IPv4)");
		register_fpar("port", &m_svr_port, "Destination port number");

		// parameters for image data (these are sent to the destination)
		register_fpar("fmt", &m_fmt, "Image data format {0: raw 1: jpg 2: png}");
		register_fpar("depth", &m_depth, "Color depth in byte}");
		register_fpar("channel", &m_channel, "Number of color channels");
		register_fpar("cfmt", &m_cfmt, "Color format {0: Mono, 1: Bayer, 2:RGB}");
	}

	virtual ~f_rcv_img()
	{
	}

	virtual bool init_run();

	virtual void destroy_run();

	virtual bool proc();
};

#endif
