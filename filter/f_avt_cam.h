#ifndef _F_AVT_CAM_H_
#define _F_AVT_CAM_H_
// Copyright(c) 2013 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// f_avt_cam.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_avt_cam.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_avt_cam.  If not, see <http://www.gnu.org/licenses/>. 

#include "PvApi.h"
#include "../channel/ch_image.h"

class f_avt_cam: public f_base
{
	enum eBandwidthCtrlMode {
	StreamBytesPerSecond=0, SCPD, Both
};

	enum eExposureMode{
		emManual, emAuto, emAutoOnce, emExternal
	};

	enum eExposureAutoAlg{
		eaaMean, eaaFitRange
	};

	enum eGainMode{
		egmManual, egmAuto, egmAutoOnce, egmExternal
	};

	enum eWhitebalMode{
		ewmManual, ewmAuto, ewmAutoOnce
	};

protected:
	static const char * strPvFmt[ePvFmtBayer12Packed+1];
	static const char * strBandwidthCtrlMode[Both+1];
	static const char * strExposureMode[emExternal+1];
	static const char * strExposureAutoAlg[eaaFitRange+1];
	static const char * strGainMode[egmExternal+1];
	static const char * strWhitebalMode[ewmAutoOnce+1];

	vector<bool> m_frm_done;
	ch_image_ref * pout;
	static bool m_bready_api;
	char m_host[1024];

	int m_num_buf;
	tPvAccessFlags m_access;
	int m_size_buf;
	tPvFrame * m_frame;
	unsigned char ** m_img_buf;
	tPvHandle m_hcam;

	int m_cur_frm;
	
	///////////////////// parameters
	// static parameters. these parameters should not be modified during running state
	tPvImageFormat m_PixelFormat;
	bool config_param();

	// dynamic parameters. These parameters can be modified during running state
	bool m_update;
	eBandwidthCtrlMode m_BandwidthCtrlMode;
	unsigned int m_StreamBytesPerSecond;
	eExposureMode m_ExposureMode;
	unsigned int m_ExposureAutoAdjustTol;
	eExposureAutoAlg m_ExposureAutoAlg;
	unsigned int m_ExposureAutoMax;
	unsigned int m_ExposureAutoMin;
	unsigned int m_ExposureAutoOutliers;
	unsigned int m_ExposureAutoRate;
	unsigned int m_ExposureAutoTarget;
	unsigned int m_ExposureValue;
	unsigned int m_GainMode;
	unsigned int m_GainAutoAdjustTol;
	unsigned int m_GainAutoMax;
	unsigned int m_GainAutoMin;
	unsigned int m_GainAutoOutliers;
	unsigned int m_GainAutoRate;
	unsigned int m_GainAutoTarget;
	unsigned int m_GainValue;
	eWhitebalMode m_WhitebalMode;
	unsigned int m_WhitebalAutoAdjustTol;
	unsigned int m_WhitebalAutoRate;
	unsigned int m_WhitebalValueRed;
	unsigned int m_WhitebalValueBlue;

	bool config_param_dynamic();

	virtual bool init_run();
	virtual void destroy_run();
public:
	static bool init_interface();
	static void destroy_interface();
	f_avt_cam(const char * name);
	virtual ~f_avt_cam();

	virtual const char * get_err_msg(int code);
	virtual bool proc();

	void set_new_frm(tPvFrame * pfrm);
};
#endif