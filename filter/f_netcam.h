// Copyright(c) 2011 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, Ltd. All right reserved. 

// f_netcam.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_netcam.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_netcam.h.  If not, see <http://www.gnu.org/licenses/>. 
#ifdef SANYO_HD5400
#define XMD_H
#include <jpeglib.h>
#include <curl/curl.h>
#include "../util/aws_sock.h"
#include "../util/aws_thread.h"
#include "../channel/ch_scalar.h"
#include "f_cam.h"
#include "../util/my_jsrc_mgr.h"

#define STR_SIZE 128

enum e_jsrc_dec_stat{
	EJD_INIT, EJD_HEADER, EJD_START, EJD_SCAN, EJD_FINISH, EJD_FOOTER
};

enum e_ctrl_focus {
	ECF_NEAR, ECF_FAR, ECF_ONE_PUSH, ECF_AUTO
};

enum e_cfg_cam_params{
	ECCP_FOCUS_SW, ECCP_SET_NORTH_SW
};

// f_netcam is class for controling network camera SANYO VCC-HD5400
// via http protocol. Http requests are processed by libcurl,
// therefore, the libs are to be installed.
class f_netcam :public f_cam
{
protected:
	mutex m_cm_mt;
	Mat m_frm_back;
	void lock_cm()
	{
		m_cm_mt.lock();
	}

	void unlock_cm()
	{
		m_cm_mt.unlock();
	}

	unsigned char m_y, m_mt, m_d, m_h, m_mn, m_s;
	unsigned char m_fcnt;	// 15 byte
	unsigned short m_p;		// 17, 18byte
	short m_t;	// 19, 20 byte
	unsigned short m_z;	// 21, 22 byte

	////////////////////////access control under m_cm_mt lock
	CURL * m_curl;
	char * m_cmd;
	char * m_cmd_foot;

	char * m_base_url;
	char * m_usrpswd;
	const char * m_cookie_file;
	const char * m_header_file;
	FILE * m_header;
	const char * m_body_file;
	FILE * m_body;

	////////////////////////access control under m_cv_mt lock

	//access control under m_cv_mt lock //////////////////////
	Mat m_tvec_img;  // translation vector from the camera center to the image center.

	unsigned short m_pan;
	unsigned short m_tilt_dir;
	unsigned short m_tilt;

	///////////////////////////////// for jpeg decompression
	jpeg_decompress_struct m_cinfo;
	jpeg_error_mgr m_jerr;
	JSAMPARRAY m_img_line;
	
	e_jsrc_dec_stat m_ejd_stat;
	unsigned char * m_pfill_data;

	// flags
	bool m_session_alive;
	bool check_and_restart_session()
	{
		if(m_session_alive)
			return true;

		curl_easy_setopt(m_curl, CURLOPT_URL, m_base_url);
		curl_easy_setopt(m_curl, CURLOPT_USERPWD, m_usrpswd);
		CURLcode res = curl_easy_perform(m_curl);
		if(res != CURLE_OK){
			cout << curl_easy_strerror(res) << endl;
			return false;
		}

		m_session_alive = true;

		return true;
	}

	ch_ptzctrl * m_pptzin;
	ch_ptz * m_pptzout;

	bool m_bgrabbing;
	thread * m_grab_th;
public:
	f_netcam(const char * name);
	virtual ~f_netcam();

	// dec jpeg image
	// these functions return false for io-suspension.
	bool dec_jpeg_init();
	bool dec_jpeg_fill_buffer(void * ptr, size_t nmemb);
	bool dec_jpeg_header();
	bool dec_jpeg_start();
	bool dec_jpeg_scanline();
	bool dec_jpeg_finish();
	bool dec_jpeg_footer();

	bool is_content_image()
	{ 
		char * ct;
		curl_easy_getinfo(m_curl, CURLINFO_CONTENT_TYPE, &ct);
		return ct[0] == 'i';
	}

	void dead_session()
	{
		m_session_alive = false;
	}

	e_jsrc_dec_stat get_jpeg_dec_stat()
	{return m_ejd_stat;}

	virtual bool open(const char * base_url, const char * usrpswd);
	virtual void close();

	bool rot(short h, short v);
	bool rot_step(short h, short v);

	bool pt(unsigned short p, short t);

	bool pan_direct(unsigned short arg);
	bool tilt_direct(short arg);

	bool zoom_step(bool tele = true);
	bool zoom_direct(short t);
	bool ctrl_focus(enum e_ctrl_focus val);

	bool cfg_params(e_cfg_cam_params par, short val);

	bool cmd_direct(const char * cmd);

	unsigned short get_pan_pos()
	{return m_p;};
	unsigned short get_tilt_pos()
	{return m_t;};
	unsigned short get_zoom_pos()
	{return m_z;};

	virtual bool grab(Mat & img);
	static void async_grab(f_netcam * pncam);

	virtual bool cmd_proc(s_cmd & cmd);
	bool cmdp_pan(char ** args, int num_args);
	bool cmdp_tilt(char ** args, int num_args);
	bool cmdp_zoom(char ** args, int num_args);
	bool cmdp_focus(char ** args, int num_args);
	bool cmdp_cfg(char ** args, int num_args);
	bool cmdp_dcmd(char ** args, int num_args);

	virtual bool proc()
	{
//		m_time = m_cur_time;
	
		ch_ptzctrl * pptzin = dynamic_cast<ch_ptzctrl*>(m_chin[0]);

		if(pptzin != NULL){
			unsigned short val;
			if(pptzin->is_pan(val)){
				pan_direct(val);
			}else if(pptzin->is_tilt(val)){
				tilt_direct(val);
			}else if(pptzin->is_zoom(val)){
				zoom_direct(val);
			}
		}
		
		if (!m_bgrabbing){
			if (m_grab_th)
				delete m_grab_th;
			m_grab_th = NULL;
			m_grab_th = new thread(async_grab, this);
		}
		return true;
	}
};

#endif