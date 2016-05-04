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

#include "f_base.h"
#include "PvApi.h"
#include "../util/aws_vlib.h"
#include "../util/c_clock.h"

#include "../channel/ch_image.h"

#ifdef _WIN32
#define _STDCALL __stdcall
#else
#define _STDCALL
#endif

class f_avt_cam: public f_base
{
public:
	enum eFrameStartTriggerMode {
		efstmFreerun, efstmSyncIn1, efstmSyncIn2, efstmSyncIn3, efstmSyncIn4,
		efstmFixedRate, efstmSoftware, efstmUndef
	};

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

	enum eStrobeMode{
		esmAcquisitionTriggerReady, esmFrameTriggerReady, esmFrameTrigger, esmExposing, esmFrameReadabout,
		esmImaging, esmAcquiring, esmSynqIn1, esmSynqIn2, esmUndef
	};

	enum eStrobeControlledDuration{
		escdOn, escdOff, escdUndef
	};

	enum eSyncOutMode{
		esomGPO, esomAcquisitionTriggerReady, esomFrameTriggerReady, esomFrameTrigger, esomExposingFrameReadout,
		esomAcquiring, esomSyncIn1, esomSyncIn2, esomStrobe1, esomUndef
	};

	enum eSyncOutInvert{
		esoiOn, esoiOff, esoiUndef
	};

protected:
	static const char * strFrameStartTriggerMode[efstmUndef];
	static eFrameStartTriggerMode getFrameStartTriggerMode(const char * str){
		for(int i = 0; i < efstmUndef; i++){
			if(strcmp(strFrameStartTriggerMode[i], str) == 0)
				return (eFrameStartTriggerMode) i;
		}
		return efstmUndef;
	}

	static const char * strPvFmt[ePvFmtBayer12Packed+1];
	static tPvImageFormat getPvImageFmt(const char * str){
		for(int i = 0; i < ePvFmtBayer12Packed + 1; i++){
			if(strcmp(strPvFmt[i], str) == 0)
				return (tPvImageFormat) i;
		}
		return __ePvFmt_force_32;
	};

	static const char * strBandwidthCtrlMode[bcmUndef];
	static eBandwidthCtrlMode getBandwidthCtrlMode(const char * str){
		for(int i = 0; i < bcmUndef; i++){
			if(strcmp(strBandwidthCtrlMode[i], str) == 0)
				return (eBandwidthCtrlMode) i;
		}
		return bcmUndef;
	}

	static const char * strExposureMode[emUndef];
	static eExposureMode getExposureMode(const char * str){
		for(int i = 0; i < emUndef; i++){
			if(strcmp(strExposureMode[i], str) == 0)
				return (eExposureMode) i;
		}
		return emUndef;
	}
	static const char * strExposureAutoAlg[eaaUndef];
	static eExposureAutoAlg getExposureAutoAlg(const char * str){
		for(int i = 0; i < eaaUndef; i++){
			if(strcmp(strExposureAutoAlg[eaaUndef], str) == 0){
				return (eExposureAutoAlg) i;
			}
		}
		return eaaUndef;
	}

	static const char * strGainMode[egmUndef];
	static eGainMode getGainMode(const char * str){
		for(int i = 0; i < egmUndef; i++){
			if(strcmp(strGainMode[i], str) == 0){
				return (eGainMode) i;
			}
		}
		return egmUndef;
	}

	static const char * strWhitebalMode[ewmUndef];
	static eWhitebalMode getWhitebalMode(const char * str){
		for(int i = 0; i < ewmUndef; i++){
			if(strcmp(strWhitebalMode[i], str) == 0){
				return (eWhitebalMode) i;
			}
		}
		return ewmUndef;
	}

	static const char * strStrobeControlledDuration[escdUndef];
	static eStrobeControlledDuration getStrobeControlledDuration(const char * str){
		for(int i = 0; i < escdUndef; i++){
			if(strcmp(strStrobeControlledDuration[i], str) == 0){
				return (eStrobeControlledDuration) i;
			}
		}
		return escdUndef;
	}

	static const char * strStrobeMode[esmUndef];
	static eStrobeMode getStrobeMode(const char * str){
		for(int i = 0; i < esmUndef; i++){
			if(strcmp(strStrobeMode[i], str) == 0){
				return (eStrobeMode) i;
			}
		}
		return esmUndef;
	}

	static const char * strSyncOutMode[esomUndef];
	static eSyncOutMode getSyncOutMode(const char * str){
		for(int i = 0; i < esomUndef; i++){
			if(strcmp(strSyncOutMode[i], str) == 0){
				return (eSyncOutMode) i;
			}
		}
		return esomUndef;
	}

	static const char * strSyncOutInvert[esoiUndef];
	static eSyncOutInvert getSyncOutInvert(const char * str){
		for(int i = 0; i < esoiUndef; i++){
			if(strcmp(strSyncOutInvert[i], str) == 0){
				return (eSyncOutInvert) i;
			}
		}
		return esoiUndef;
	}

	static bool m_bready_api;
	long long m_ttrig_int;
	long long m_ttrig_prev;

	#define NUM_PV_PARAMS 55
	static const char * m_strParams[NUM_PV_PARAMS];
	struct s_cam_params{
		bool m_bactive;
		const char ** strParams;
		vector<bool> m_frm_done;
		ch_image_ref * pout;
		char m_host[1024];

		int m_num_buf;
		tPvAccessFlags m_access;
		int m_size_buf;
		tPvFrame * m_frame;
		unsigned char ** m_img_buf;
		tPvHandle m_hcam;

		int m_cur_frm;

		///////////////////// parameters
		unsigned int m_PacketSize;

		// static parameters. these parameters should not be modified during running state
		eFrameStartTriggerMode m_FrameStartTriggerMode;
		
		unsigned int m_Height;
		unsigned int m_RegionX;
		unsigned int m_RegionY;
		unsigned int m_Width;
		tPvImageFormat m_PixelFormat;

		unsigned int m_BinningX;
		unsigned int m_BinningY;
		int m_DecimationHorizontal;
		int m_DecimationVertical;
		bool m_ReverseSoftware;
		bool m_ReverseX;
		bool m_ReverseY;

		bool init(f_avt_cam * pcam, ch_base * pch);
		void stop(){
			m_bactive = false;
		}
		void destroy(f_avt_cam * pcam);

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

		eStrobeMode m_Strobe1Mode;
		eStrobeControlledDuration m_Strobe1ControlledDuration;
		unsigned int m_Strobe1Duration;
		unsigned int m_Strobe1Delay;
		eSyncOutMode m_SyncOut1Mode, m_SyncOut2Mode, m_SyncOut3Mode, m_SyncOut4Mode;
		eSyncOutInvert m_SyncOut1Invert, m_SyncOut2Invert, m_SyncOut3Invert, m_SyncOut4Invert;

		bool config_param_dynamic();
		s_cam_params(int icam = -1);
		~s_cam_params();
		void set_new_frm(tPvFrame * pfrm);
		static void _STDCALL proc_frame(tPvFrame * pfrm);

		// Camera parameter and the related parameters and methods
		AWSCamPar cp;   // Camera parameter
		char fcp[1024]; // Path to the parameter file.
		bool bundist;   // Flag enabling undistort
		Mat Pud;		// camera parameter without distortion
		Mat R;			// Rotation matrix for remap.
		Mat udmap1, udmap2; // undistort map1, undistort map2
		Size szud;		// Size of undistorted image 
		bool bemsg;			// Error message enabling flag
		bool verb;			// verbose for debug
	};

	void register_params(s_cam_params & cam);

public:
	static bool init_interface();
	static void destroy_interface();
	f_avt_cam(const char * name);
	virtual ~f_avt_cam();

	virtual const char * get_err_msg(int code);
};



#endif
