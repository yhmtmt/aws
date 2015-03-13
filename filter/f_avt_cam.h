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
		bcmStreamBytesPerSecond=0, bcmSCPD, bcmBoth, bcmUndef
	};

	enum eExposureMode{
		emManual, emAuto, emAutoOnce, emExternal, emUndef
	};

	enum eExposureAutoAlg{
		eaaMean, eaaFitRange, eaaUndef
	};

	enum eGainMode{
		egmManual, egmAuto, egmAutoOnce, egmExternal, egmUndef
	};

	enum eWhitebalMode{
		ewmManual, ewmAuto, ewmAutoOnce, ewmUndef
	};

protected:
	static const char * strPvFmt[ePvFmtBayer12Packed+1];
	tPvImageFormat getPvImageFmt(const char * str){
		for(int i = 0; i < ePvFmtBayer12Packed + 1; i++){
			if(strcmp(strPvFmt[i], str) == 0)
				return (tPvImageFormat) i;
		}
		return __ePvFmt_force_32;
	};

	static const char * strBandwidthCtrlMode[bcmUndef];
	eBandwidthCtrlMode getBandwidthCtrlMode(const char * str){
		for(int i = 0; i < bcmUndef; i++){
			if(strcmp(strBandwidthCtrlMode[i], str) == 0)
				return (eBandwidthCtrlMode) i;
		}
		return bcmUndef;
	}

	static const char * strExposureMode[emUndef];
	eExposureMode getExposureMode(const char * str){
		for(int i = 0; i < emUndef; i++){
			if(strcmp(strExposureMode[i], str) == 0)
				return (eExposureMode) i;
		}
		return emUndef;
	}
	static const char * strExposureAutoAlg[eaaUndef];
	eExposureAutoAlg getExposureAutoAlg(const char * str){
		for(int i = 0; i < eaaUndef; i++){
			if(strcmp(strExposureAutoAlg[eaaUndef], str) == 0){
				return (eExposureAutoAlg) i;
			}
		}
		return eaaUndef;
	}

	static const char * strGainMode[egmUndef];
	eGainMode getGainMode(const char * str){
		for(int i = 0; i < egmUndef; i++){
			if(strcmp(strGainMode[i], str) == 0){
				return (eGainMode) i;
			}
		}
		return egmUndef;
	}

	static const char * strWhitebalMode[ewmUndef];
	eWhitebalMode getWhitebalMode(const char * str){
		for(int i = 0; i < ewmUndef; i++){
			if(strcmp(strWhitebalMode[i], str) == 0){
				return (eWhitebalMode) i;
			}
		}
		return ewmUndef;
	}

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
	unsigned int m_Height;
	unsigned int m_RegionX;
	unsigned int m_RegionY;
	unsigned int m_Width;
	tPvImageFormat m_PixelFormat;

	unsigned int m_BinningX;
	unsigned int m_BinningY;
	int m_DecimationHorizontal;
	int m_DecimationVertical;

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