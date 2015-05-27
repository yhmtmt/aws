// Copyright(c) 2013 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// f_avt_cam.cpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_avt_cam.cpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_avt_cam.  If not, see <http://www.gnu.org/licenses/>. 

#include "stdafx.h"

#include <cstdio>

#include <iostream>
#include <fstream>
#include <vector>
#define _USE_MATH_DEFINES
#include <cmath>
#include <list>

using namespace std;

#include <opencv2/opencv.hpp>
using namespace cv;

#include "../util/aws_sock.h"
#include "../util/aws_thread.h"
#include "../util/c_clock.h"
#include "../util/util.h"
#include "f_base.h"
#include "f_avt_cam.h"

bool f_avt_cam::m_bready_api = false;
#ifdef _WIN32
#define _STDCALL __stdcall
#else
#define _STDCALL
#endif

// enum is defined in PvApi.h
const char * f_avt_cam::strPvFmt[ePvFmtBayer12Packed+1] = {
	"Mono8", "Mono16", "Bayer8", "Bayer16", "Rgb24", "Rgb48",
	"Yuv411", "Yuv422", "Yuv444", "Bgr24", "Rgba32", "Bgra32"
	"Mono12Packed",  "Bayer12Packed"
};

const char * f_avt_cam::strBandwidthCtrlMode[bcmUndef] = {
	"StreamBytesPerSecond", "SCPD", "Both"
};

const char * f_avt_cam::strExposureMode[emExternal+1] = {
	"Manual", "Auto", "AutoOnce", "External"
};

const char * f_avt_cam::strExposureAutoAlg[eaaFitRange+1] = {
	"Mean", "FitRange"
};

const char * f_avt_cam::strGainMode[egmExternal+1] = {
	"Manual", "Auto", "AutoOnce", "External"
};

const char * f_avt_cam::strWhitebalMode[ewmAutoOnce+1] = {
	"Manual", "Auto", "AutoOnce"
};

void _STDCALL proc_frame(tPvFrame * pfrm)
{
	f_avt_cam * pcam = (f_avt_cam *) pfrm->Context[0];
	pcam->set_new_frm(pfrm);
}

f_avt_cam::f_avt_cam(const char * name): f_base(name), m_num_buf(5), 
	m_access(ePvAccessMaster), m_frame(NULL), 
	m_PixelFormat(__ePvFmt_force_32),
	m_update(false),
	m_BandwidthCtrlMode(bcmUndef),
	m_StreamBytesPerSecond(0/*115000000*/), m_ExposureMode(emUndef), m_ExposureAutoAdjustTol(UINT_MAX /*5*/),
	m_ExposureAutoAlg(eaaUndef/*eaaMean*/), m_ExposureAutoMax(UINT_MAX/*500000*/), m_ExposureAutoMin(UINT_MAX/*1000*/),
	m_ExposureAutoOutliers(UINT_MAX/*0*/), m_ExposureAutoRate(UINT_MAX/*100*/), m_ExposureAutoTarget(UINT_MAX/*50*/),
	m_ExposureValue(UINT_MAX/*100*/), m_GainMode(egmUndef), m_GainAutoAdjustTol(UINT_MAX/*5*/), m_GainAutoMax(UINT_MAX/*30*/),
	m_GainAutoMin(UINT_MAX/*10*/), m_GainAutoOutliers(UINT_MAX/*0*/), m_GainAutoRate(UINT_MAX /*100*/), m_GainAutoTarget(UINT_MAX /*50*/),
	m_GainValue(UINT_MAX/*10*/),
	m_WhitebalMode(ewmAuto), m_WhitebalAutoAdjustTol(5), m_WhitebalAutoRate(100),
	m_WhitebalValueRed(0), m_WhitebalValueBlue(0),
	m_Height(UINT_MAX), m_RegionX(UINT_MAX), m_RegionY(UINT_MAX), m_Width(UINT_MAX),
	m_BinningX(UINT_MAX), m_BinningY(UINT_MAX), m_DecimationHorizontal(0), m_DecimationVertical(0)
{
	register_fpar("host", m_host, 1024, "Network address of the camera to be opened.");
	register_fpar("nbuf", &m_num_buf, "Number of image buffers.");
	register_fpar("update", &m_update, "Update flag.");
	register_fpar("BandwidthCtrlMode", (int*)&m_BandwidthCtrlMode, bcmUndef, strBandwidthCtrlMode, "Bandwidth control mode (default StreamBytesPerSecond)");
	register_fpar("StreamBytesPerSecond", &m_StreamBytesPerSecond, "StreamBytesPerSecond (default 115000000)");

	// about exposure
	register_fpar("ExposureMode", (int*) &m_ExposureMode, (int)(emExternal) + 1, strExposureMode, "ExposureMode (default Auto)");
	register_fpar("ExposureAutoAdjustTol", &m_ExposureAutoAdjustTol, "ExposureAutoAdjusttol (default 5)");
	register_fpar("ExposureAutoAlg", (int*)&m_ExposureAutoAlg, (int)eaaFitRange, strExposureAutoAlg, "ExposureAutoAlg (default Mean)");
	register_fpar("ExposureAutomax", &m_ExposureAutoMax, "ExposureAutoMax (default 500000us)");
	register_fpar("ExposureAutomin", &m_ExposureAutoMin, "ExposureAutoMin (default 1000us)");
	register_fpar("ExposureAutoOutliers", &m_ExposureAutoOutliers, "ExposureAutoOutliers (default 0)");
	register_fpar("ExposureAutoRate", &m_ExposureAutoRate, "ExposureAutoRate (default 100)");
	register_fpar("ExposureAutoTarget", &m_ExposureAutoTarget, "ExposureAutoTarget (default 50)");
	register_fpar("ExposureValue", &m_ExposureValue, "ExposureValue (default 100us)");

	// about gain
	register_fpar("GainMode", (int*)&m_GainMode, (int)(egmExternal) + 1, strGainMode, "GainMode (default Auto)");
	register_fpar("GainAutoAdjustTol", &m_GainAutoAdjustTol, "GainAutoAdjusttol (default 5)");
	register_fpar("GainAutomax", &m_GainAutoMax, "GainAutoMax (default 30db)");
	register_fpar("GainAutomin", &m_GainAutoMin, "GainAutoMin (default 5db)");
	register_fpar("GainAutoOutliers", &m_GainAutoOutliers, "GainAutoOutliers (default 0)");
	register_fpar("GainAutoRate", &m_GainAutoRate, "GainAutoRate (default 100)");
	register_fpar("GainAutoTarget", &m_GainAutoTarget, "GainAutoTarget (default 50)");
	register_fpar("GainValue", &m_GainValue, "GainValue (default 10db)");

	// about white balance
	register_fpar("WhitebalMode", (int*)&m_WhitebalMode, (int)ewmAutoOnce+1, strWhitebalMode, "WhitebalMode (default Auto)");
	register_fpar("WhitebalAutoAdjustTol", &m_WhitebalAutoAdjustTol, "WhitebaAutoAdjustTol (percent, default 5)");
	register_fpar("WhitebalAutoRate", &m_WhitebalAutoRate, "WhitebalAutoRate (percent, default 100)");
	register_fpar("WhitebalValueRed", &m_WhitebalValueRed, "WhitebalValueRed (percent, default 0)");
	register_fpar("WhitebalValueBlue", &m_WhitebalValueBlue, "WhitebalValueBlue (percent, default 0)");

	// Image format
	register_fpar("Height", &m_Height, "Height of the ROI (1 to Maximum Height)");
	register_fpar("Width", &m_Width, "Width of the ROI(1 to Maximum Width)");
	register_fpar("RegionX", &m_RegionX, "Top left x position of the ROI (0 to Maximum Camera Width - 1)");
	register_fpar("RegionY", &m_RegionY, "Top left y position of the ROI (0 to Maximum Camera Height -1)");
	register_fpar("PixelFormat", (int*)&m_PixelFormat, (int)(ePvFmtBayer12Packed+1), strPvFmt, "Image format.");

}

f_avt_cam::~f_avt_cam()
{
}

const char * f_avt_cam::get_err_msg(int code)
{
	const char * msg = f_base::get_err_msg(code);
	if(msg)
		return msg;

	switch(code){
	case FERR_AVT_CAM_INIT:
		return "Failed to initialize PvAPI.";
	case FERR_AVT_CAM_OPEN:
		return "Failed to open camera.";
	case FERR_AVT_CAM_ALLOC:
		return "Failed to allocate frame buffers.";
	case FERR_AVT_CAM_CLOSE:
		return "Failed to close camera.";
	case FERR_AVT_CAM_CFETH:
		return "Failed to reconfigure ethernet frame size.";
	case FERR_AVT_CAM_START:
		return "Failed to start capture.";
	case FERR_AVT_CAM_STOP:
		return "Failed to stop capture.";
	case FERR_AVT_CAM_CH:
		return "Channel is not set correctly.";
	case FERR_AVT_CAM_DATA:
		return "Incomplete image data.";
	}

	return NULL;
}


bool f_avt_cam::init_interface(){
	tPvErr err = PvInitialize();

	if(err != ePvErrSuccess){
		f_base::send_err(NULL, __FILE__, __LINE__, FERR_AVT_CAM_INIT);
		return false;
	}

	m_bready_api = true;
	return true;
}

void f_avt_cam::destroy_interface(){
	PvUnInitialize();
	m_bready_api = false;
}


bool f_avt_cam::config_param()
{
	tPvErr err;

	if(m_PixelFormat == __ePvFmt_force_32){
		char buf[64];
		err = PvAttrEnumGet(m_hcam, "PixelFormat", buf, 64, NULL);
		if(err != ePvErrSuccess){
			cerr << "Failed to get PiexelFomrat" << endl;
			return false;
		}
		m_PixelFormat = getPvImageFmt(buf);
	}else{
		err = PvAttrEnumSet(m_hcam, "PixelFormat", strPvFmt[m_PixelFormat]);
		if(err != ePvErrSuccess){
			cerr << "Failed to set PiexelFomrat" << endl;
			return false;
		}
	}

	if(m_Height == UINT_MAX){
		err = PvAttrUint32Get(m_hcam, "Height", (tPvUint32*)&m_Height);
		if(err != ePvErrSuccess){
			cerr << "Failed to get Height" << endl;
			return false;
		}
	}else{
		err = PvAttrUint32Set(m_hcam, "Height", (tPvUint32)m_Height);
		if(err != ePvErrSuccess){
			cerr << "failed to set Height" << endl;
			return false;
		}
	}

	if(m_Width == UINT_MAX){
		err = PvAttrUint32Get(m_hcam, "Width", (tPvUint32*)&m_Width);
		if(err != ePvErrSuccess){
			cerr << "Failed to get Width" << endl;
			return false;
		}
	}else{
		err = PvAttrUint32Set(m_hcam, "Width", (tPvUint32)m_Width);
		if(err != ePvErrSuccess){
			cerr << "failed to set Width" << endl;
			return false;
		}
	}

	if(m_RegionX == UINT_MAX){
		err = PvAttrUint32Get(m_hcam, "RegionX", (tPvUint32*)&m_RegionX);
		if(err != ePvErrSuccess){
			cerr << "Failed to get RegionX" << endl;
			return false;
		}
	}else{
		err = PvAttrUint32Set(m_hcam, "RegionX", (tPvUint32)m_RegionX);
		if(err != ePvErrSuccess){
			cerr << "failed to set RegionX" << endl;
			return false;
		}
	}

	if(m_RegionY == UINT_MAX){
		err = PvAttrUint32Get(m_hcam, "RegionY", (tPvUint32*)&m_RegionY);
		if(err != ePvErrSuccess){
			cerr << "Failed to get RegionY" << endl;
			return false;
		}
	}else{
		err = PvAttrUint32Set(m_hcam, "RegionY", (tPvUint32)m_RegionY);
		if(err != ePvErrSuccess){
			cerr << "failed to set RegionY" << endl;
			return false;
		}
	}

	if(m_BinningX == UINT_MAX){
		err = PvAttrUint32Get(m_hcam, "BinningX", (tPvUint32*)&m_BinningX);
		if(err != ePvErrSuccess){
			cerr << "Failed to get BinningX" << endl;
			return false;
		}
	}else{
		err = PvAttrUint32Set(m_hcam, "BinningX", (tPvUint32)m_BinningX);
		if(err != ePvErrSuccess){
			cerr << "failed to set BinningX" << endl;
			return false;
		}
	}
	
	if(m_BinningY == UINT_MAX){
		err = PvAttrUint32Get(m_hcam, "BinningY", (tPvUint32*)&m_BinningY);
		if(err != ePvErrSuccess){
			cerr << "Failed to get BinningY" << endl;
			return false;
		}
	}else{
		err = PvAttrUint32Set(m_hcam, "BinningY", (tPvUint32)m_BinningY);
		if(err != ePvErrSuccess){
			cerr << "failed to set BinningY" << endl;
			return false;
		}
	}
	/*
	if(m_DecimationHorizontal == 0){
		long long val;
		err = PvAttrInt64Get(m_hcam, "DecimationHorizontal", (tPvInt64*)&val);
		if(err != ePvErrSuccess){
			cerr << "Failed to get DecimationHorizontal" << endl;
			return false;
		}
		m_DecimationHorizontal = (int) val;
	}else{
		long long val = m_DecimationHorizontal;
		err = PvAttrInt64Set(m_hcam, "DecimationHorizontal", (tPvInt64)val);
		if(err != ePvErrSuccess){
			cerr << "failed to set DecimationHorizontal" << endl;
			return false;
		}
	}

	if(m_DecimationVertical == 0){
		long long val;
		err = PvAttrInt64Get(m_hcam, "DecimationVertical", (tPvInt64*)&val);
		if(err != ePvErrSuccess){
			cerr << "Failed to get DecimationVertical" << endl;
			return false;
		}
		m_DecimationVertical = (int) val;
	}else{
		long long val = m_DecimationVertical;
		err = PvAttrInt64Set(m_hcam, "DecimationVertical", (tPvInt64)val);
		if(err != ePvErrSuccess){
			cerr << "Failed to set DecimationVertical" << endl;
			return false;
		}
	}
	*/
	return config_param_dynamic();
}

bool f_avt_cam::config_param_dynamic()
{
	tPvErr err;
	if(m_BandwidthCtrlMode == bcmUndef){
		char buf[64];
		err = PvAttrEnumGet(m_hcam, "BandwidthCtrlMode", buf, 64, NULL);
		if(err != ePvErrSuccess){
			cerr << "Failed to get BandwidthCtrlMode" << endl;
			return false;
		}
		m_BandwidthCtrlMode = getBandwidthCtrlMode(buf);
	}else{
		err = PvAttrEnumSet(m_hcam, "BandwidthCtrlMode", strBandwidthCtrlMode[m_BandwidthCtrlMode]);
		if(err != ePvErrSuccess){
			cerr << "Failed to set BandwidthCtrlMode" << endl;
			return false;
		}
	}

	if(m_StreamBytesPerSecond == 0){
		err = PvAttrUint32Get(m_hcam, "StreamBytesPerSecond", (tPvUint32*) &m_StreamBytesPerSecond);
		if(err != ePvErrSuccess){
			cerr << "Failed to get Stream BytesPerSecond" << endl;
			return false;
		}
	}else{
		err = PvAttrUint32Set(m_hcam, "StreamBytesPerSecond", (tPvUint32) m_StreamBytesPerSecond);
		if(err != ePvErrSuccess){
			cerr << "Failed to set StreamBytesPerSecond" << endl;
			return false;
		}
	}

	if(m_ExposureMode == emUndef){
		char buf[64];
		err = PvAttrEnumGet(m_hcam, "ExposureMode", buf, 64, NULL);
		if(err != ePvErrSuccess){
			cerr << "Failed to get ExposureMode" << endl;
			return false;
		}
		m_ExposureMode = getExposureMode(buf);
	}else{
		err = PvAttrEnumSet(m_hcam, "ExposureMode", strExposureMode[m_ExposureMode]);
		if(err != ePvErrSuccess){
			cerr << "Failed to set ExposureMode" << endl;
			return false;
		}
	}

	if(m_ExposureAutoAdjustTol == UINT_MAX){
		err = PvAttrUint32Get(m_hcam, "ExposureAutoAdjustTol", (tPvUint32*)&m_ExposureAutoAdjustTol);
		if(err != ePvErrSuccess){
			cerr << "Failed to get ExposureAutoAdjustTol" << endl;
			return false;
		}
	}else{
		err = PvAttrUint32Set(m_hcam, "ExposureAutoAdjustTol", m_ExposureAutoAdjustTol);
		if(err != ePvErrSuccess){
			cerr << "Failed to set ExposureAutoAdjustTol" << endl;
			return false;
		}
	}

	if(m_ExposureAutoAlg == eaaUndef){
		char buf[64];
		err = PvAttrEnumGet(m_hcam, "ExposureAutoAlg", buf, 64, NULL);
		if(err != ePvErrSuccess){
			cerr << "Failed to get ExposureAutoAlg" << endl;
			return false;
		}
		m_ExposureAutoAlg = getExposureAutoAlg(buf);
	}else{
		err = PvAttrEnumSet(m_hcam, "ExposureAutoAlg", strExposureAutoAlg[m_ExposureAutoAlg]);
		if(err != ePvErrSuccess){
			cerr << "Failed to set ExposureAutoAlg" << endl;
			return false;
		}
	}

	if(m_ExposureAutoMax == UINT_MAX){
		err = PvAttrUint32Get(m_hcam, "ExposureAutoMax", (tPvUint32*)&m_ExposureAutoMax);
		if(err != ePvErrSuccess){
			cerr << "Failed to get ExposureAutomax" << endl;
			return false;
		}
	}else{
		err = PvAttrUint32Set(m_hcam, "ExposureAutoMax", m_ExposureAutoMax);
		if(err != ePvErrSuccess){
			cerr << "Failed to set ExposureAutoMax" << endl;
			return false;
		}
	}

	if(m_ExposureAutoMin == UINT_MAX){
		err = PvAttrUint32Get(m_hcam, "ExposureAutoMin", (tPvUint32*)&m_ExposureAutoMin);
		if(err != ePvErrSuccess){
			cerr << "Failed to get ExposureAutoMin" << endl;
			return false;
		}
	}else{
		err = PvAttrUint32Set(m_hcam, "ExposureAutoMin", m_ExposureAutoMin);
		if(err != ePvErrSuccess){
			cerr << "Failed to set ExposureAutoMin" << endl;
			return false;
		}
	}

	if(m_ExposureAutoOutliers == UINT_MAX){
		err = PvAttrUint32Get(m_hcam, "ExposureAutoOutliers", (tPvUint32*)&m_ExposureAutoOutliers);
		if(err != ePvErrSuccess){
			cerr << "Failed to get ExposureAutoOutliers" << endl;
			return false;
		}
	}else{
		err = PvAttrUint32Set(m_hcam, "ExposureAutoOutliers", m_ExposureAutoOutliers);
		if(err != ePvErrSuccess){
			cerr << "Failed to set ExposureAutoOutliers" << endl;
			return false;
		}
	}

	if(m_ExposureAutoRate == UINT_MAX){
		err = PvAttrUint32Get(m_hcam, "ExposureAutoRate", (tPvUint32*)&m_ExposureAutoRate);
		if(err != ePvErrSuccess){
			cerr << "Failed to get ExposureAutoRate" << endl;
			return false;
		}
	}else{
		err = PvAttrUint32Set(m_hcam, "ExposureAutoRate", m_ExposureAutoRate);
		if(err != ePvErrSuccess){
			cerr << "Failed to set ExposureAutoRate" << endl;
			return false;
		}
	}

	if(m_ExposureAutoTarget == UINT_MAX){
		err = PvAttrUint32Get(m_hcam, "ExposureAutoTarget", (tPvUint32*)&m_ExposureAutoTarget);
		if(err != ePvErrSuccess){
			cerr << "Failed to set ExposureAutoTarget" << endl;
			return false;
		}
	}else{
		err = PvAttrUint32Set(m_hcam, "ExposureAutoTarget", m_ExposureAutoTarget);
		if(err != ePvErrSuccess){
			cerr << "Failed to set ExposureAutoTarget" << endl;
			return false;
		}
	}

	if(m_ExposureValue == UINT_MAX){
		err = PvAttrUint32Get(m_hcam, "ExposureValue", (tPvUint32*)&m_ExposureValue);
		if(err != ePvErrSuccess){
			cerr << "Failed to get ExposureValue" << endl;
			return false;
		}
	}else{
		err = PvAttrUint32Set(m_hcam, "ExposureValue", m_ExposureValue);
		if(err != ePvErrSuccess){
			cerr << "Failed to set ExposureValue" << endl;
			return false;
		}
	}

	if(m_GainMode == egmUndef){
		char buf[64];
		err = PvAttrEnumGet(m_hcam, "GainMode", buf, 64, NULL);
		if(err != ePvErrSuccess){
			cerr << "Failed to get GainMode" << endl;
			return false;
		}
		m_GainMode = getGainMode(buf);
	}else{
		err = PvAttrEnumSet(m_hcam, "GainMode", strGainMode[m_GainMode]);
		if(err != ePvErrSuccess){
			cerr << "Failed to set GainMode" << endl;
			return false;
		}
	}


	if(m_GainAutoAdjustTol == UINT_MAX){
		err = PvAttrUint32Get(m_hcam, "GainAutoAdjustTol", (tPvUint32*)&m_GainAutoAdjustTol);
		if(err != ePvErrSuccess){
			cerr << "Failed to get GainAutoAdjustTol" << endl;
			return false;
		}
	}else{
		err = PvAttrUint32Set(m_hcam, "GainAutoAdjustTol", m_GainAutoAdjustTol);
		if(err != ePvErrSuccess){
			cerr << "Failed to set GainAutoAdjustTol" << endl;
			return false;
		}
	}

	if(m_GainAutoMax == UINT_MAX){
		err = PvAttrUint32Get(m_hcam, "GainAutoMax", (tPvUint32*)&m_GainAutoMax);
		if(err != ePvErrSuccess){
			cerr << "Failed to get GainAutoMax" << endl;
			return false;
		}
	}else{
		err = PvAttrUint32Set(m_hcam, "GainAutoMax", m_GainAutoMax);
		if(err != ePvErrSuccess){
			cerr << "Failed to set GainAutoMax" << endl;
			return false;
		}
	}

	if(m_GainAutoMin == UINT_MAX){
		err = PvAttrUint32Get(m_hcam, "GainAutoMin", (tPvUint32*)&m_GainAutoMin);
		if(err != ePvErrSuccess){
			cerr << "Failed to get GainAutoMin" << endl;
			return false;
		}
	}else{
		err = PvAttrUint32Set(m_hcam, "GainAutoMin", m_GainAutoMin);
		if(err != ePvErrSuccess){
			cerr << "Failed to set GainAutoMin" << endl;
			return false;
		}
	}

	if(m_GainAutoOutliers == UINT_MAX){
		err = PvAttrUint32Get(m_hcam, "GainAutoOutliers", (tPvUint32*)&m_GainAutoOutliers);
		if(err != ePvErrSuccess){
			cerr << "Failed to get GainAutoOutliers" << endl;
			return false;
		}
	}else{
		err = PvAttrUint32Set(m_hcam, "GainAutoOutliers", m_GainAutoOutliers);
		if(err != ePvErrSuccess){
			cerr << "Failed to set GainAutoOutliers" << endl;
			return false;
		}
	}

	if(m_GainAutoRate == UINT_MAX){
		err = PvAttrUint32Get(m_hcam, "GainAutoRate", (tPvUint32*)&m_GainAutoRate);
		if(err != ePvErrSuccess){
			cerr << "Failed to get GainAutoRate" << endl;
			return false;
		}
	}else{
		err = PvAttrUint32Set(m_hcam, "GainAutoRate", m_GainAutoRate);
		if(err != ePvErrSuccess){
			cerr << "Failed to set GainAutoRate" << endl;
			return false;
		}
	}

	if(m_GainAutoTarget == UINT_MAX){
		err = PvAttrUint32Get(m_hcam, "GainAutoTarget", (tPvUint32*)&m_GainAutoTarget);
		if(err != ePvErrSuccess){
			cerr << "Failed to get GainAutoTarget" << endl;
			return false;
		}
	}else{
		err = PvAttrUint32Set(m_hcam, "GainAutoTarget", m_GainAutoTarget);
		if(err != ePvErrSuccess){
			cerr << "Failed to set GainAutoTarget" << endl;
			return false;
		}
	}

	if(m_GainValue == UINT_MAX){
		err = PvAttrUint32Get(m_hcam, "GainValue", (tPvUint32*)&m_GainValue);
		if(err != ePvErrSuccess){
			cerr << "Failed to get GainValue" << endl;
			return false;
		}
	}else{
		err = PvAttrUint32Set(m_hcam, "GainValue", m_GainValue);
		if(err != ePvErrSuccess){
			cerr << "Failed to set GainValue" << endl;
			return false;
		}
	}

	return true;
}


bool f_avt_cam::init_run()
{
	int m_size_buf = 0;
	if(!m_chout.size()){
		f_base::send_err(this, __FILE__, __LINE__, FERR_AVT_CAM_CH);
		return false;
	}

	pout = dynamic_cast<ch_image_ref*>(m_chout[0]);

	if(!pout){
		f_base::send_err(this, __FILE__, __LINE__, FERR_AVT_CAM_CH);
		return false;
	}

	tPvErr err;

	// init_interface shoudl be called before running this filter
	if(!m_bready_api){
		f_base::send_err(this, __FILE__, __LINE__, FERR_AVT_CAM_INIT);
		return false;
	}

	// opening camera by IP address
	unsigned long IpAddr = inet_addr(m_host);
	err = PvCameraOpenByAddr(IpAddr, m_access, &m_hcam);

	if(err != ePvErrSuccess){
		f_base::send_err(this, __FILE__, __LINE__, FERR_AVT_CAM_OPEN);
		return false;
	}

	if(!config_param()){
		goto cam_close;
	}

	// getting frame size sent from camera
	err = PvAttrUint32Get(m_hcam, "TotalBytesPerFrame", (tPvUint32*)&m_size_buf);

	if(err != ePvErrSuccess){
		f_base::send_err(this, __FILE__, __LINE__, FERR_AVT_CAM_ALLOC);
		goto cam_close;
	}
	// allocating image buffer
	m_frame = new tPvFrame[m_num_buf];
	if(m_frame == NULL){
		f_base::send_err(this, __FILE__, __LINE__, FERR_AVT_CAM_ALLOC);
		goto cam_close;
	}

	memset(m_frame, 0, sizeof(tPvFrame) * m_num_buf);
	m_frm_done.resize(m_num_buf);

	unsigned int ibuf;
	for(ibuf = 0; ibuf < (unsigned) m_num_buf; ibuf++){
		m_frm_done[ibuf] = false;
		m_frame[ibuf].Context[0] = (void*) this;
		m_frame[ibuf].Context[1] = (void*) ibuf;
		m_frame[ibuf].ImageBufferSize = m_size_buf;
		m_frame[ibuf].ImageBuffer = (void*) new unsigned char[m_size_buf];
		if(!m_frame[ibuf].ImageBuffer){
			f_base::send_err(this, __FILE__, __LINE__, FERR_AVT_CAM_ALLOC);
			goto free_buf;
		}
	}

	err = PvCaptureAdjustPacketSize(m_hcam, 8228);

	if(err != ePvErrSuccess){
		f_base::send_err(this, __FILE__, __LINE__, FERR_AVT_CAM_CFETH);
		goto free_buf;
	}

	err = PvCaptureStart(m_hcam);
	if(err != ePvErrSuccess){
		f_base::send_err(this, __FILE__, __LINE__, FERR_AVT_CAM_START);
		goto free_buf;
	}

	m_cur_frm = 0;
	for(ibuf = 0; ibuf < (unsigned) m_num_buf; ibuf++){
		PvCaptureQueueFrame(m_hcam, &m_frame[ibuf], proc_frame);
	}

	err = PvAttrEnumSet(m_hcam, "FrameStartTriggerMode", "Freerun");
	if(err != ePvErrSuccess){
		f_base::send_err(this, __FILE__, __LINE__, FERR_AVT_CAM_START);
		goto free_buf;
	}

	err = PvAttrEnumSet(m_hcam, "AcquisitionMode", "Continuous");
	if(err != ePvErrSuccess){
		f_base::send_err(this, __FILE__, __LINE__, FERR_AVT_CAM_START);
		goto free_buf;
	}

	err = PvCommandRun(m_hcam, "AcquisitionStart");
	if(err != ePvErrSuccess){
		f_base::send_err(this, __FILE__, __LINE__, FERR_AVT_CAM_START);
		goto free_buf;
	}

	return true;

free_buf:
	for(ibuf = 0; ibuf < (unsigned) m_num_buf; ibuf++){
		delete[] (unsigned char *) m_frame[ibuf].ImageBuffer;
	}
	delete m_frame;
	m_frame = NULL;

cam_close:
	err = PvCameraClose(m_hcam);
	if(err != ePvErrSuccess){
		f_base::send_err(this, __FILE__, __LINE__, FERR_AVT_CAM_CLOSE);
	}
	return false;
}

void f_avt_cam::destroy_run()
{
	tPvErr err;
	err = PvCommandRun(m_hcam, "AcquisitionStop");
	if(err != ePvErrSuccess){
		f_base::send_err(this, __FILE__, __LINE__, FERR_AVT_CAM_STOP);
	}

#ifdef _WIN32
	Sleep(200);
#else
	sleep(1);
#endif

	err = PvCaptureQueueClear(m_hcam);
	if(err != ePvErrSuccess){
		f_base::send_err(this, __FILE__, __LINE__, FERR_AVT_CAM_STOP);
	}

	err = PvCaptureEnd(m_hcam);
	if(err != ePvErrSuccess){
		f_base::send_err(this, __FILE__, __LINE__, FERR_AVT_CAM_STOP);
	}

	if(m_frame != NULL){
		int ibuf;
		for(ibuf = 0; ibuf < m_num_buf; ibuf++){
			if(m_frame[ibuf].ImageBuffer){
				delete[] (unsigned char *) m_frame[ibuf].ImageBuffer;
			}
		}
		delete[] m_frame;
		m_frame = NULL;
	}

	err = PvCameraClose(m_hcam);

	if(err != ePvErrSuccess){
		f_base::send_err(this, __FILE__, __LINE__, FERR_AVT_CAM_CLOSE);
	}
}

void f_avt_cam::set_new_frm(tPvFrame * pfrm)
{
  if(!m_bactive)
    return;

	unsigned int ibuf;
	if(pfrm->Status == ePvErrSuccess){
		Mat img;
		switch(pfrm->Format){
		case ePvFmtMono8:
			img = Mat(pfrm->Height, pfrm->Width, CV_8UC1, pfrm->ImageBuffer);
			pout->set_img(img, m_cur_time);
			break;
		case ePvFmtMono16:
			img = Mat(pfrm->Height, pfrm->Width, CV_16UC1, pfrm->ImageBuffer);
			pout->set_img(img, m_cur_time);
			break;
		case ePvFmtBayer8:
			img = Mat(pfrm->Height, pfrm->Width, CV_8UC1, pfrm->ImageBuffer);
			pout->set_img(img, m_cur_time);
			break;
		case ePvFmtBayer16:
			img = Mat(pfrm->Height, pfrm->Width, CV_16UC1, pfrm->ImageBuffer);
			pout->set_img(img, m_cur_time);
			break;
		case ePvFmtRgb24:
			img = Mat(pfrm->Height, pfrm->Width, CV_8UC3, pfrm->ImageBuffer);
			pout->set_img(img, m_cur_time);
			break;
		case ePvFmtRgb48:
			img = Mat(pfrm->Height, pfrm->Width, CV_16UC3, pfrm->ImageBuffer);
			pout->set_img(img, m_cur_time);
			break;
		case ePvFmtYuv411:
		case ePvFmtYuv422:
		case ePvFmtYuv444:
		case ePvFmtBgr24:
		case ePvFmtRgba32:
		case ePvFmtBgra32:
		case ePvFmtMono12Packed:
		case ePvFmtBayer12Packed:
		default:
			break;
		}
	}

	ibuf = *((unsigned int*) (&pfrm->Context[1]));
	m_cur_frm = pfrm->FrameCount;
	m_frm_done[ibuf] = true;

	for(ibuf = 0; ibuf < (unsigned) m_num_buf; ibuf++){
		if(m_frm_done[ibuf]){
			if(!pout->is_buf_in_use((const unsigned char*) m_frame[ibuf].ImageBuffer))
				PvCaptureQueueFrame(m_hcam, &m_frame[ibuf], proc_frame);
		}
	}
}

bool f_avt_cam::proc()
{
	// if any, reconfigure camera
	if(m_update){
		config_param_dynamic();
		m_update = false;
	}

	return true;
}

