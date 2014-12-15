// Copyright(c) 2011 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// f_netcam.cpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_netcam.cpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_netcam.  If not, see <http://www.gnu.org/licenses/>. 

// f_netcam.cpp is classs for controling network camera SANYO VCC-HD5400
// via http protocol. Http requests are processed with curllib,
// therefore, the libs are to be installed.

#include "stdafx.h"

#include <cstdio>

#include <iostream>
#include <fstream>
#include <vector>
#define _USE_MATH_DEFINES
#include <cmath>

using namespace std;

#include <opencv2/opencv.hpp>
using namespace cv;

#include "../util/aws_sock.h"
#include "../util/thread_util.h"
#include "../util/c_clock.h"
#include "../util/util.h"
#include "../channel.h"
#include "f_base.h"
#include "f_cam.h"
#include "f_netcam.h"

///////////////////// write_data callback function for libcurl to write file 
static size_t write_data(void * ptr, size_t size,
	size_t nmemb, void * stream)
{
	int written = (int) fwrite(ptr, size, nmemb, (FILE*) stream);
	return written;
}

/////////////////////  write_data callback function for libcurl to decode jpeg
static size_t img_data(void * ptr, size_t size, 
	size_t nmemb, void * pcam)
{
	f_netcam * pccam = (f_netcam*) pcam;

	if(ptr == NULL)
		return 0;

	if(!pccam->is_content_image()){
		pccam->dead_session();
		cout.write((const char *) ptr, size * nmemb);
		cout << "Session dead. Failed to load image." << endl;
		return nmemb;
	}

	pccam->dec_jpeg_fill_buffer(ptr, nmemb);

	switch(pccam->get_jpeg_dec_stat()){
	case EJD_HEADER:
		if(!pccam->dec_jpeg_header())
			break;
	case EJD_START:
		if(!pccam->dec_jpeg_start())
			break;
	case EJD_SCAN:
		if(!pccam->dec_jpeg_scanline())
			break;
	case EJD_FINISH:
		if(!pccam->dec_jpeg_finish())
			break;
	case EJD_FOOTER:
		if(!pccam->dec_jpeg_footer())
			break;
	}

	return nmemb;
}

//////////////////////////////////////////////////////////////// f_netcam members
f_netcam::f_netcam(const char * name):f_cam(name), m_cmd(NULL), m_cmd_foot(NULL),
	m_base_url(NULL), 
	m_usrpswd(NULL), m_cookie_file("cookie.txt"), m_header_file("header.out"),
	m_body_file("body.out"), m_header(NULL), m_body(NULL), m_curl(NULL), 
	m_img_line(NULL), m_pfill_data(NULL), m_ejd_stat(EJD_INIT), m_bgrabbing(false)
{
	// initialize mutex
	pthread_mutex_init(&m_cm_mt, NULL);

	m_cam_mat = Mat::eye(3, 3, CV_64F);
	m_dist_coeff = Mat::zeros(8, 1, CV_64F);
}

f_netcam::~f_netcam()
{
	close();

	// free mutex
	pthread_mutex_destroy(&m_cm_mt);
}

bool f_netcam::open(const char * base_url, const char * usrpswd)
{

	m_base_url = new char[strlen(base_url) + 1];
	strcpy(m_base_url, base_url);

	m_usrpswd = new char[strlen(usrpswd) + 1];
	strcpy(m_usrpswd, usrpswd);

	m_cmd = new char [strlen(m_base_url) + 128];
	sprintf(m_cmd, "http://%s/", m_base_url);
	m_cmd_foot = m_cmd + strlen(m_cmd);

	// getting cookie
	m_curl = curl_easy_init();
	if(m_curl == NULL)
		return false;

	m_header = fopen(m_header_file, "w");
	if(m_header == NULL)
		return false;

	m_body = fopen(m_body_file, "w");
	if(m_body == NULL)
		return false;

//	curl_easy_setopt(m_curl, CURLOPT_VERBOSE, 1);
	curl_easy_setopt(m_curl, CURLOPT_URL, m_base_url);
	curl_easy_setopt(m_curl, CURLOPT_USERPWD, m_usrpswd);
	curl_easy_setopt(m_curl, CURLOPT_COOKIEJAR, m_cookie_file);
	curl_easy_setopt(m_curl, CURLOPT_HEADERFUNCTION, write_data);
	curl_easy_setopt(m_curl, CURLOPT_WRITEHEADER, m_header);
	curl_easy_setopt(m_curl, CURLOPT_WRITEFUNCTION, write_data);
	curl_easy_setopt(m_curl, CURLOPT_WRITEDATA, m_body);

	CURLcode res = curl_easy_perform(m_curl);
	if(res != CURLE_OK){
		cout << curl_easy_strerror(res) << endl;
		return false;
	}

	m_session_alive = true;

	m_cinfo.err = jpeg_std_error(&m_jerr);
	jpeg_create_decompress(&m_cinfo);

	if(m_chout[1]){
		ch_campar * pcamparout = dynamic_cast<ch_campar*>(m_chout[1]);
		if(pcamparout == NULL){
			throw "1st channel is bad ";
		}

		pcamparout->set_Pix(1920, 1080);
	}

	pt(0, 0);

	return true;
}

void f_netcam::close()
{
	if(m_session_alive)
		jpeg_destroy_decompress(&m_cinfo);

	// free objects related to libcurl
	curl_easy_cleanup(m_curl);
	fclose(m_header);
	fclose(m_body);

	delete [] m_base_url;
	delete [] m_usrpswd;
	delete [] m_cmd;
	m_base_url = m_usrpswd = m_cmd = NULL;

	m_curl = NULL;

	// free objects related to ijg lib 
	if(m_img_line != NULL){
		free(m_img_line[0]);
		free(m_img_line);
	}

	m_session_alive = false;
}

bool f_netcam::grab(Mat & img)
{
	lock_cm();
	try{
		if(!check_and_restart_session())
			throw "Failed to restart session.";

		{// swap back buffer
			Mat tmp = m_frm;
			m_frm = m_frm_back;
			m_frm_back = tmp;
			sprintf(m_cmd_foot, "liveimg.cgi", m_base_url);
		}

		curl_easy_setopt(m_curl, CURLOPT_URL, m_cmd);

		curl_easy_setopt(m_curl, CURLOPT_WRITEFUNCTION, img_data);
		curl_easy_setopt(m_curl, CURLOPT_WRITEDATA, this);

		dec_jpeg_init();

		CURLcode res = curl_easy_perform(m_curl);

		if(res != CURLE_OK)
			throw curl_easy_strerror(res);

		img = m_frm;

		curl_easy_setopt(m_curl, CURLOPT_WRITEFUNCTION, write_data);
		curl_easy_setopt(m_curl, CURLOPT_WRITEDATA, m_body);

		unlock_cm();
	}catch(char * msg){
		cerr << msg << endl;
		curl_easy_setopt(m_curl, CURLOPT_WRITEFUNCTION, write_data);
		curl_easy_setopt(m_curl, CURLOPT_WRITEDATA, m_body);
		unlock_cm();
		return false;
	}

	return true;
}


void * f_netcam::async_grab(void * pncam)
{
	f_netcam * pcam = (f_netcam*) pncam;
	pcam->m_bgrabbing = true;

	ch_image * pout = dynamic_cast<ch_image*>(pcam->m_chout[0]);
	Mat img;
	bool result = pcam->grab(img);
	pout->set_img(img, m_cur_time);

	pcam->m_bgrabbing = false;
	return NULL;
}


bool f_netcam::pt(unsigned short p, short t)
{
	if(p < 0 || p > 35999)
		return false;

	if(t < 0)
		m_tilt_dir = 1;
	else
		m_tilt_dir = 0;

	t = abs(t);
	if(t > 18000)
		return false;

	m_tilt = (t == 18000 ? 17999 : t);

	lock_cm();

	if(!check_and_restart_session()){
		unlock_cm();
		cout << "Failed to restart session." << endl;
		return false;
	}

	m_pan = p;
	sprintf(m_cmd_foot, "cgi-bin/opecmd.cgi?ope=54&Pan_deg=%d&Tilt_dir=%d&Tilt_deg=%d", m_pan, m_tilt_dir, m_tilt);
	curl_easy_setopt(m_curl, CURLOPT_URL, m_cmd);

	CURLcode res = curl_easy_perform(m_curl);
	unlock_cm();

	if(res != 0){
		cout << curl_easy_strerror(res) << endl;
		return false;
	}

	return true;
}

bool f_netcam::pan_direct(unsigned short arg)
{
	if(arg < 0 || arg > 35999)
		return false;

	lock_cm();

	if(!check_and_restart_session()){
		unlock_cm();
		cout << "Failed to restart session." << endl;
		return false;
	}

	m_pan = arg;
	sprintf(m_cmd_foot, "cgi-bin/opecmd.cgi?ope=54&Pan_deg=%d&Tilt_dir=%d&Tilt_deg=%d", m_pan, m_tilt_dir, m_tilt);
	curl_easy_setopt(m_curl, CURLOPT_URL, m_cmd);

	CURLcode res = curl_easy_perform(m_curl);
	unlock_cm();

	if(res != 0){
		cout << curl_easy_strerror(res) << endl;
		return false;
	}

	return true;
}

bool f_netcam::tilt_direct(short arg)
{
	if(arg < 0)
		m_tilt_dir = 1;
	else
		m_tilt_dir = 0;

	arg = abs(arg);
	if(arg > 18000)
		return false;

	m_tilt = (arg == 18000 ? 17999 : arg);

	lock_cm();
	if(!check_and_restart_session()){
		unlock_cm();
		cout << "Failed to restart session." << endl;
		return false;
	}

	sprintf(m_cmd_foot, "cgi-bin/opecmd.cgi?ope=54&Tilt_dir=%d&Tilt_deg=%d", m_tilt_dir, m_tilt);
	curl_easy_setopt(m_curl, CURLOPT_URL, m_cmd);
	CURLcode res = curl_easy_perform(m_curl);
	unlock_cm();

	if(res != 0){
		cout << curl_easy_strerror(res) << endl;
		return false;
	}

	return true;
}

bool f_netcam::cmd_direct(const char * cmd)
{
	lock_cm();
	if(!check_and_restart_session()){
		unlock_cm();
		cout << "Failed to restart session." << endl;
		
		return false;
	}

	strcpy(m_cmd_foot, cmd);
	curl_easy_setopt(m_curl, CURLOPT_URL, m_cmd);
	CURLcode res = curl_easy_perform(m_curl);
	unlock_cm();

	if(res != 0){
		cout << curl_easy_strerror(res) << endl;
		return false;
	}

	return true;
}

bool f_netcam::rot(short h, short v)
{

	if(h < -8 || h > 8 || v < -8 || v > 8)
		return false;

	lock_cm();
	if(!check_and_restart_session()){
		unlock_cm();
		cout << "Failed to restart session." << endl;
		return false;
	}

	if(h < 0){
		if(v < 0){ // down left
			sprintf(m_cmd_foot, "cgi-bin/opecmd.cgi?ope=51&cmd=6&p_speed=%d&t_speed=%d", -(h+1), -(v+1));
		}else if(v > 0){ // up left
			sprintf(m_cmd_foot, "cgi-bin/opecmd.cgi?ope=51&cmd=1&p_speed=%d&t_speed=%d", -(h+1), v-1);
		}else{ // left
			sprintf(m_cmd_foot, "cgi-bin/opecmd.cgi?ope=51&cmd=4&p_speed=%d", -(h+1));
		}
	}else if(h > 0){
		if(v < 0){ // down right
			sprintf(m_cmd_foot, "cgi-bin/opecmd.cgi?ope=51&cmd=8&p_speed=%d&t_speed=%d", h-1, -(v+1));
		}else if(v > 0){ // up right
			sprintf(m_cmd_foot, "cgi-bin/opecmd.cgi?ope=51&cmd=3&p_speed=%d&t_speed=%d", h-1, v-1);
		}else{ // right
			sprintf(m_cmd_foot, "cgi-bin/opecmd.cgi?ope=51&cmd=5&p_speed=%d", h-1);
		}
	}else{
		if(v < 0){ // down
			sprintf(m_cmd_foot, "cgi-bin/opecmd.cgi?ope=51&cmd=7&p_speed=%d", -(v+1));
		}else if(v > 0){ //up
			sprintf(m_cmd_foot, "cgi-bin/opecmd.cgi?ope=51&cmd=2&p_speed=%d", v-1);
		}else{ //stop
			sprintf(m_cmd_foot, "cgi-bin/opecmd.cgi?ope=51&cmd=12");
		}
	}

	curl_easy_setopt(m_curl, CURLOPT_URL, m_cmd);
	CURLcode res = curl_easy_perform(m_curl);

	unlock_cm();

	if(res != 0){
		cout << curl_easy_strerror(res) << endl;
		return false;
	}

	return true;
}

bool f_netcam::rot_step(short h, short v)
{
	if(h < -100 || h > 100 || v < -100 || v > 100)
		return false;

	lock_cm();
	if(!check_and_restart_session()){
		unlock_cm();
		cout << "Failed to restart session." << endl;
		return false;
	}

	sprintf(m_cmd_foot, "cgi-bin/opecmd.cgi?ope=20&pan_dir=%d&pan_percent=%d&tilt_dir=%d&tilt_percent=%d", (h < 0 ? 2 : 1), abs((int) h), (v < 0 ? 2 : 1), abs((int) v));

	curl_easy_setopt(m_curl, CURLOPT_URL, m_cmd);
	CURLcode res = curl_easy_perform(m_curl);

	unlock_cm();

	if(res != 0){
		cout << curl_easy_strerror(res) << endl;
		return false;
	}

	return true;
}

bool f_netcam::zoom_step(bool tele)
{
	lock_cm();
	if(!check_and_restart_session()){
		unlock_cm();
		cout << "Failed to restart session." << endl;
		return false;
	}

	if(tele){
		sprintf(m_cmd_foot, "cgi-bin/opecmd.cgi?ope=18");
	}else{
		sprintf(m_cmd_foot, "cgi-bin/opecmd.cgi?ope=19");
	}

	curl_easy_setopt(m_curl, CURLOPT_URL, m_cmd_foot);
	CURLcode res = curl_easy_perform(m_curl);
	unlock_cm();

	if(res != 0){
		cout << curl_easy_strerror(res) << endl;
		return false;
	}

	return true;
}

bool f_netcam::zoom_direct(short t)
{
	if(t < 1 || t > 10)
		return false;

	lock_cm();
	if(!check_and_restart_session()){
		unlock_cm();
		cout << "Failed to restart session." << endl;
		return false;
	}

	sprintf(m_cmd_foot, "cgi-bin/opecmd.cgi?ope=52&num=%d", t);

	curl_easy_setopt(m_curl, CURLOPT_URL, m_cmd);
	CURLcode res = curl_easy_perform(m_curl);

	unlock_cm();

	if(res != 0){
		cout << curl_easy_strerror(res) << endl;
		return false;
	}

	return true;
}

bool f_netcam::ctrl_focus(e_ctrl_focus val)
{
	lock_cm();
	if(!check_and_restart_session()){
		unlock_cm();
		cout << "Failed to restart session." << endl;
		return false;
	}

	short code;

	switch(val){
	case ECF_NEAR:
		code = 0;
		break;
	case ECF_FAR:
		code = 1;
		break;
	case ECF_ONE_PUSH:
		code = 2;
		break;
	case ECF_AUTO:
		code = 3;
		break;

	}
	
	sprintf(m_cmd_foot, "cgi-bin/opecmd.cgi?ope=49&action=%d", code);
	curl_easy_setopt(m_curl, CURLOPT_URL, m_cmd);
	CURLcode res = curl_easy_perform(m_curl);

	unlock_cm();

	if(res != 0){
		cout << curl_easy_strerror(res) << endl;
		return false;
	}

	return true;
}

bool f_netcam::cfg_params(e_cfg_cam_params par, short val)
{
	lock_cm();

	if(!check_and_restart_session()){
		unlock_cm();
		cout << "Failed to restart session." << endl;
		return false;
	}

	bool success = true;

	switch(par){
	case ECCP_FOCUS_SW:
		if(val != 0 && val != 1){
			success = false;
			break;
		}
		
		sprintf(m_cmd_foot, "cgi-bin/camera_quality.cgi?focus_sw=%d", val);
		break;
	case ECCP_SET_NORTH_SW:
		if(val != 0 && val != 1){
			success = false;
			break;
		}

		sprintf(m_cmd_foot, "cgi-bin/pan_tilt.cgi?set_north_sw=%d", val);
		break;
	default:
		success = false;
		break;
	}

	if(!success){
		unlock_cm();
		cout << "Configuration failed." << endl;
		return false;
	}

	curl_easy_setopt(m_curl, CURLOPT_URL, m_cmd);
	CURLcode res = curl_easy_perform(m_curl);
	unlock_cm();

	if(res != 0){
		cout << curl_easy_strerror(res) << endl;
		return false;
	}

	return true;
}



////////////////////////////////////////////// helper fucntions for jpeg decode 
bool f_netcam::dec_jpeg_init()
{
	jpeg_my_src(&m_cinfo);
	m_ejd_stat = EJD_HEADER;

	return true;
}

bool f_netcam::dec_jpeg_fill_buffer(void * ptr, size_t nmemb)
{
	s_jsrc_mgr * psrc = (s_jsrc_mgr*) m_cinfo.src;
	// remained bytes in the buffer are copied first to the head.
	if(psrc->pub.bytes_in_buffer != 0)
		memcpy(psrc->buffer, psrc->pub.next_input_byte, psrc->pub.bytes_in_buffer);

	// skip_bytes is remained only if the bytes_in_buffer is zero. 
	// (previous call of skip_input_data triggered io-suspension)
	// thus subtracted from newly loaded data.
	memcpy(psrc->buffer + psrc->pub.bytes_in_buffer, ptr, nmemb - psrc->skip_bytes);
	psrc->pub.bytes_in_buffer = nmemb + psrc->pub.bytes_in_buffer;
	psrc->pub.next_input_byte = psrc->buffer;

	return true;
}

bool f_netcam::dec_jpeg_header()
{
	if(jpeg_read_header(&m_cinfo, TRUE) == JPEG_SUSPENDED)
		return false;

	if(m_img_line == NULL){
		m_img_line = (JSAMPARRAY) malloc(sizeof(JSAMPROW));
		m_img_line[0] = (JSAMPROW) malloc(sizeof(JSAMPLE) * m_cinfo.image_width * 3);
	}

	if(m_frm.rows != m_cinfo.image_height && m_frm.cols != m_cinfo.image_width){
		m_frm = Mat(m_cinfo.image_height, m_cinfo.image_width, CV_8UC3);
		m_frm_back = Mat(m_cinfo.image_height, m_cinfo.image_width, CV_8UC3);
	}

	m_ejd_stat = EJD_START;

	return true;
}

bool f_netcam::dec_jpeg_start()
{
	if(jpeg_start_decompress(&m_cinfo) == FALSE)
		return false;

	m_ejd_stat = EJD_SCAN;

	return true;
}

bool f_netcam::dec_jpeg_scanline()
{

	if(m_pfill_data == NULL){
		m_pfill_data = (unsigned char *) m_frm.data;
	}

	while(m_cinfo.output_scanline < m_cinfo.image_height){
		if(jpeg_read_scanlines(&m_cinfo, m_img_line, 1) == 0)
			return false;

		JSAMPROW line = m_img_line[0];
		for(unsigned int ipix = 0; ipix < m_cinfo.image_width; ipix++){
			m_pfill_data[0] = line[2]; // B
			m_pfill_data[1] = line[1]; // G
			m_pfill_data[2] = line[0]; // R
			m_pfill_data += 3;
			line += 3;
		}
	}

	m_pfill_data = NULL;

	m_ejd_stat = EJD_FINISH;

	return true;
}

bool f_netcam::dec_jpeg_finish()
{
	if(jpeg_finish_decompress(&m_cinfo) == FALSE)
		return false;

	m_ejd_stat = EJD_FOOTER;

	return true;
}

bool f_netcam::dec_jpeg_footer()
{
	if(m_cinfo.src->bytes_in_buffer != 48) // decode footer info
		return false;

	const JOCTET * buf = ((s_jsrc_mgr * ) m_cinfo.src)->pub.next_input_byte;

	m_y = (unsigned char) buf[8];
	m_mt = (unsigned char) buf[9];
	m_d = (unsigned char) buf[10];
	m_h = (unsigned char) buf[11];
	m_mn = (unsigned char) buf[12];
	m_s = (unsigned char) buf[13];

	m_fcnt = (unsigned char) buf[14];
	m_p = (unsigned short) (buf[17] | buf[16] << 8);
	m_t = (short) (buf[19] | buf[18] << 8);
	m_z = (unsigned short) (buf[21] | buf[20] << 8);
	
	ch_ptz * pptzout = dynamic_cast<ch_ptz*>(m_chout[2]);

	if(pptzout != NULL)
		pptzout->set(m_p, m_t, m_z);

	m_ejd_stat = EJD_INIT;

	return true;
}

////////////////////////////////////////// helper functions for Computer Vision

bool f_netcam::cmd_proc(s_cmd & cmd)
{
	int num_args = cmd.num_args;
	char ** args = cmd.args;

	if(num_args < 2)
		return false;
	int itok = 2;

	if(strcmp(args[itok], "p") == 0){
		return cmdp_pan(args, num_args);
	}else if(strcmp(args[itok], "t") == 0){
		return cmdp_tilt(args, num_args);
	}else if(strcmp(args[itok], "z") == 0){
		return cmdp_zoom(args, num_args);
	}else if(strcmp(args[itok], "f") == 0){
		return cmdp_focus(args, num_args);
	}else if(strcmp(args[itok], "c") == 0){
		return cmdp_cfg(args, num_args);
	}else if(strcmp(args[itok], "d") == 0){
		return cmdp_dcmd(args, num_args);
	}else if(strcmp(args[itok], "open") == 0){
		if(num_args != 5)
			return false;
		return open(args[itok+1], args[itok+2]);
	}

	return f_cam::cmd_proc(cmd);
}

bool f_netcam::cmdp_pan(char ** args, int num_args)
{
	if(num_args < 3)
		return false;

	int itok = 3;
	int arg = (int) (atoi(args[itok])* 100) % 36000;
	if(arg < 0)
		arg = (arg + 36000) % 36000;

	return pan_direct((unsigned short) arg);
}

bool f_netcam::cmdp_tilt(char ** args, int num_args)
{
	if(num_args < 3)
		return false;

	int itok = 3;
	int arg = (int) (atof(args[itok]) * 100.) % 36000;
	if(arg > 18000)
		arg -= 36000;
	else if(arg < -18000)
		arg += 36000;

	return tilt_direct((short) arg);
}

bool f_netcam::cmdp_zoom(char ** args, int num_args)
{
	if(num_args < 3)
		return false;

	int itok = 3;
	short value = atoi(args[itok]);
	return zoom_direct(value);
}

bool f_netcam::cmdp_focus(char ** args, int num_args)
{
	if(num_args < 3)
		return false;

	int itok = 3;
	e_ctrl_focus cf;
	if(strcmp("near", args[itok]) == 0){
		cf = ECF_NEAR;
	}else if(strcmp("far", args[itok]) == 0){
		cf = ECF_FAR;
	}else if(strcmp("push", args[itok]) == 0){
		cf = ECF_ONE_PUSH;
	}else if(strcmp("auto", args[itok]) == 0){
		cf = ECF_AUTO;
	}else{
		cout << args[itok] << " is unknown focus command" << endl;
		return false;
	}

	return ctrl_focus(cf);	
}

bool f_netcam::cmdp_cfg(char ** args, int num_args)
{
	if(num_args < 4)
		return false;

	e_cfg_cam_params par;
	int itok = 3;

	if(strcmp(args[itok], "focus_sw") == 0){
		par = ECCP_FOCUS_SW;
	}else{
		cout << args[itok] << " is unknown cam parameter." << endl;
		return false;
	}

	itok++;
	short val = atoi(args[itok]);

	return cfg_params(par, val);
}

bool f_netcam::cmdp_dcmd(char ** args, int num_args)
{
	if(num_args < 3)
		return false;

	return cmd_direct(args[3]);
}
