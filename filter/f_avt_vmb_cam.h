#ifndef AVT_VMB_CAM_H
#define AVT_VMB_CAM_H
// Copyright(c) 2013 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// f_avt_vmb_cam.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_avt_vmb_cam.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_avt_vmb_cam.  If not, see <http://www.gnu.org/licenses/>. 

#include "f_base.h"
#include <VimbaC/Include/VmbCommonTypes.h>
#include <VimbaCPP/Include/VimbaCPPCommon.h>

#include <VimbaCPP/Include/Camera.h>
#include <VimbaCPP/Include/Interface.h>
#include <VimbaCPP/Include/VimbaSystem.h>
#include <VimbaCPP/Include/FeatureContainer.h>
#include <VimbaCPP/Include/ICameraFactory.h>
#include <VimbaCPP/Include/ICameraListObserver.h>
#include <VimbaCPP/Include/IInterfaceListObserver.h>
#include <VimbaCPP/Include/IFeatureObserver.h>
#include <VimbaCPP/Include/IFrameObserver.h>
#include <VimbaCPP/Include/Frame.h>

#include "../channel/ch_image.h"

namespace avt_vmb_cam{
  using namespace AVT::VmbAPI;
  
  class FrameObserver : public IFrameObserver
  {
  private:
    CameraPtr m_pCamera;
    ch_image_ref * pch;
    e_imfmt fmt_out;
    long long received_frames, dropped_frames;
  public:
  FrameObserver(CameraPtr pCamera, ch_image_ref * _pch, const e_imfmt _fmt_out = IMF_Undef) :IFrameObserver(pCamera), m_pCamera(pCamera), pch(_pch),
      fmt_out(_fmt_out), received_frames(0), dropped_frames(0)
    {}

    const long long getNumRecievedFrames(){
      return received_frames;
    }

    const long long getNumDroppedFrames()
    {
      return dropped_frames;
    }

    void demosaic8(Mat & src, Mat & dst, e_imfmt & fmt, int bayer)
    {
      ColorConversionCodes code;
      switch(bayer){
      case 0://BG
	code = (fmt_out == IMF_RGB8 ? COLOR_BayerBG2RGB : COLOR_BayerBG2BGR);
	break;
      case 1://GB
	code = (fmt_out == IMF_RGB8 ? COLOR_BayerBG2RGB :COLOR_BayerGB2BGR);
	break;
      case 2://RG
	code = (fmt_out == IMF_RGB8 ? COLOR_BayerBG2RGB :COLOR_BayerRG2BGR);
	break;
      case 3://GR
	code = (fmt_out == IMF_RGB8 ? COLOR_BayerBG2RGB :COLOR_BayerGR2BGR);	
      }
		   
      switch(fmt_out){
      case IMF_GRAY8:
	{
	  Mat tmp;
	  cvtColor(src, tmp, code);
	  cvtColor(tmp, dst, COLOR_BGR2GRAY);
	  fmt = fmt_out;
	}
	break;
      case IMF_RGB8:
	{
	  cvtColor(src, dst, code);
	  fmt = fmt_out;
	}
	break;
      case IMF_BGR8:
	{
	  cvtColor(src, dst, code);
	  fmt = fmt_out;
	}
	break;	  
      case IMF_I420:
	{
	  Mat tmp;
	  cvtColor(src, tmp, code);
	  cvtColor(tmp, dst, COLOR_BGR2YUV_I420);
	  fmt = fmt_out;
	}
      default:
	dst = src.clone();	
      }
    }
    
    void setImg(const VmbPixelFormatType & pixelFormat,
		VmbUint8_t * pBuffer, const VmbUint32_t Width, const VmbUint32_t Height,  const unsigned long long fid)
    {
      e_imfmt fmt;
      Mat img, img_set;

      switch (pixelFormat){
      case VmbPixelFormatMono8:	      
      case VmbPixelFormatBayerBG8:
      case VmbPixelFormatBayerGB8:
      case VmbPixelFormatBayerGR8:
      case VmbPixelFormatBayerRG8:
	img = Mat(Height, Width, CV_8UC1, pBuffer);
	break;
      case VmbPixelFormatMono10:
      case VmbPixelFormatMono12:
      case VmbPixelFormatMono14:
      case VmbPixelFormatMono16:
      case VmbPixelFormatBayerBG10:
      case VmbPixelFormatBayerBG12:
      case VmbPixelFormatBayerGB10:
      case VmbPixelFormatBayerGB12:
      case VmbPixelFormatBayerGR10:
      case VmbPixelFormatBayerGR12:
      case VmbPixelFormatBayerRG10:
      case VmbPixelFormatBayerRG12:
	img = Mat(Height, Width, CV_16UC1, pBuffer);
	break;
      case VmbPixelFormatBgr8:
      case VmbPixelFormatRgb8:
	img = Mat(Height, Width, CV_8UC3, pBuffer);
	break;
      case VmbPixelFormatBgr10:
      case VmbPixelFormatBgr12:
      case VmbPixelFormatBgr14:
      case VmbPixelFormatBgr16:
      case VmbPixelFormatRgb10:
      case VmbPixelFormatRgb12:
      case VmbPixelFormatRgb14:
      case VmbPixelFormatRgb16:
	img = Mat(Height, Width, CV_16UC3, pBuffer);
	break;
      }
      
      switch (pixelFormat){
      case VmbPixelFormatMono8:
	fmt = (IMF_GRAY8);
	img_set = img.clone();
	break;
      case VmbPixelFormatBayerBG8:
	fmt = (IMF_BayerBG8);
	demosaic8(img, img_set, fmt, 0);
	break;
      case VmbPixelFormatBayerGB8:
	fmt = (IMF_BayerGB8);
	demosaic8(img, img_set, fmt, 1);	
	break;
      case VmbPixelFormatBayerGR8:
	fmt = (IMF_BayerGR8);
	demosaic8(img, img_set, fmt, 3);
	break;
      case VmbPixelFormatBayerRG8:
	fmt = (IMF_BayerRG8);
	demosaic8(img, img_set, fmt, 2);
	break;
      case VmbPixelFormatMono10:
	fmt = (IMF_GRAY10);
	img_set = img.clone();
	break;
      case VmbPixelFormatMono12:
	fmt = (IMF_GRAY12);
	img_set = img.clone();
	break;
      case VmbPixelFormatMono14:
	fmt = (IMF_GRAY14);
	img_set = img.clone();
	break;
      case VmbPixelFormatMono16:
	fmt = (IMF_GRAY16);
	img_set = img.clone();
	break;
      case VmbPixelFormatBayerBG10:
	fmt = (IMF_BayerBG10);
	img_set = img.clone();
	break;
      case VmbPixelFormatBayerBG12:
	fmt = (IMF_BayerBG12);
	img_set = img.clone();
	break;
      case VmbPixelFormatBayerGB10:
	fmt = (IMF_BayerGB10);
	img_set = img.clone();
	break;
      case VmbPixelFormatBayerGB12:
	fmt = (IMF_BayerGB12);
	img_set = img.clone();
	break;
      case VmbPixelFormatBayerGR10:
	fmt = (IMF_BayerGR10);
	img_set = img.clone();
	break;
      case VmbPixelFormatBayerGR12:
	fmt = (IMF_BayerGR12);
	img_set = img.clone();
	break;
      case VmbPixelFormatBayerRG10:
	fmt = (IMF_BayerRG10);
	img_set = img.clone();
	break;
      case VmbPixelFormatBayerRG12:
	fmt = (IMF_BayerRG10);
	img_set = img.clone();
	break;
      case VmbPixelFormatBgr8:
	fmt = (IMF_BGR8);
	img_set = img.clone();
	break;
      case VmbPixelFormatRgb8:
	fmt = (IMF_RGB8);
	img_set = img.clone();
	break;
      case VmbPixelFormatBgr10:
	fmt = (IMF_BGR10);
	img_set = img.clone();
	break;
      case VmbPixelFormatBgr12:
	fmt = (IMF_BGR12);
	img_set = img.clone();
	break;
      case VmbPixelFormatBgr14:
	fmt = (IMF_BGR14);
	img_set = img.clone();
	break;
      case VmbPixelFormatBgr16:
	fmt = (IMF_BGR16);
	img_set = img.clone();
	break;
      case VmbPixelFormatRgb10:
	fmt = (IMF_RGB10);
	img_set = img.clone();
	break;
      case VmbPixelFormatRgb12:
	fmt = (IMF_RGB12);
	img_set = img.clone();
	break;
      case VmbPixelFormatRgb14:
	fmt = (IMF_RGB14);
	img_set = img.clone();
	break;
      case VmbPixelFormatRgb16:
	fmt = (IMF_RGB16);
	img_set = img.clone();
	break;	  
      }

      pch->set_img(img_set, f_base::get_time(), (const long long)fid);
      pch->set_fmt(fmt);
      
    }
    
    void FrameReceived(const FramePtr pFrame)
    {
      VmbFrameStatusType eReceiveStatus;
      
      if (VmbErrorSuccess == pFrame->GetReceiveStatus(eReceiveStatus))
	{
	  if (VmbFrameStatusComplete == eReceiveStatus){
	    
	    VmbUint32_t Width, Height;
	    VmbUint8_t * pBuffer;
	    pFrame->GetHeight(Height);
	    pFrame->GetWidth(Width);
	    pFrame->GetBuffer(pBuffer);
	    VmbPixelFormatType pixelFormat;
	    pFrame->GetPixelFormat(pixelFormat);
	    	    
	    unsigned long long fid;
	    pFrame->GetFrameID(fid);

	    setImg(pixelFormat, pBuffer, Width, Height, fid);
	    received_frames++;
	  }
	  else{
	    unsigned long long fid;
	    pFrame->GetFrameID(fid);
	    cerr << "Frame[" << fid << "]'s data is not valid." << endl;
	    dropped_frames++;
	  }
	}else{
	cerr << "Failed to acquire Frame." << endl;
	dropped_frames++;
      }
      m_pCamera->QueueFrame(pFrame);
    }
  };
  
  class f_avt_vmb_cam : public f_base
  {
  public:
  protected:
    static VimbaSystem * psys;
    enum eFeature{
      GVCPCmdRetries,
      GVCPCmdTimeout,      
      AcquisitionMode, // enum
      AcquisitionFrameRateAbs, // float
      SensorShutterMode, //enum
      TriggerActivation, //enum
      TriggerDelayAbs, //float
      TriggerMode, //enum
      TriggerSelector, //enum
      TriggerSource,  //enum
      DSPSubregionBottom, //int
      DSPSubregionLeft, //int
      DSPSubregionRight, //int
      DSPSubregionTop,//int
      ExposureAuto, //enum
      ExposureAutoAdjustTol, //int
      ExposureAutoAlg, //enum
      ExposureAutoMax, // int
      ExposureAutoMin, //int 
      ExposureAutoOutliers, //int
      ExposureAutoRate, //int
      ExposureAutoTarget, //int
      ExposureMode, //enum
      ExposureTimeAbs,//float
      ExposureTimePWL1,//float 
      ExposureTimePWL2,//float
      Gain, // float
      GainAuto, // enum
      GainAutoAdjustTol, // int
      GainAutoMax, // float
      GainAutoMin, //float 
      GainAutoOutliers, // int
      GainAutoRate, //int
      GainAutoTarget,//int
      BlackLevel,//float
      BalanceRatioAbs,//float
      BalanceRatioSelector,//enum
      BalanceWhiteAuto, //enum
      BalanceWhiteAutoAdjustTol,//int
      BalanceWhiteAutoRate,//int
      BandwidthControlMode, //enum
      GevSCPSPacketSize, // int
      StreamBytesPerSecond,//int
      StreamFrameRateConstrain,//boolean
      Height, //int
      OffsetX, // int
      OffsetY, // int
      PixelFormat, //enum
      Width, // int
      ReverseX, // bool
      ReverseY, // bool
      PayloadSize,
      nfrmbuf,
      addr, // char[1024]
      update, // bool
      verb,
      channel,
      fmt_out,
      FeatureUndef
    };
    
    static const char * strFeature[FeatureUndef];
    static const char * expFeature[FeatureUndef];
    
    enum class eAcquisitionMode {
      Continuous, SingleFrame, MultiFrame, Recorder, Undef
	};
    static const char * strAcquisitionMode[(unsigned long long) eAcquisitionMode::Undef];
    eAcquisitionMode findAcquisitionMode(const string & str)
    {
      for (int i = (int)eAcquisitionMode::Continuous; i != (int)eAcquisitionMode::Undef; i++)
	{
	  if (strAcquisitionMode[i] == str)
	    return (eAcquisitionMode)i;
	}
      return eAcquisitionMode::Undef;
    }
    
    enum class eSensorShutterMode{
      Global, Rolling, GlobalReset, Undef
	};
    static const char * strSensorShutterMode[(unsigned long long)eSensorShutterMode::Undef];
    eSensorShutterMode findSensorShutterMode(const string & str)
    {
      for (int i = (int)eSensorShutterMode::Global; i != (int)eSensorShutterMode::Undef; i++){
	if (strSensorShutterMode[i] == str)
	  return (eSensorShutterMode)i;
      }
      return eSensorShutterMode::Undef;
    }
    
    enum class eTriggerActivation{
      RisingEdge, FallingEdge, AnyEdge, LevelHigh, LevelLow, Undef
	};
    static const char * strTriggerActivation[(unsigned long long)eTriggerActivation::Undef];
    eTriggerActivation findTriggerActivation(const string & str)
    {
      for (int i = 0; i != (int)eTriggerActivation::Undef; i++){
	if (strTriggerActivation[i] == str)
	  return (eTriggerActivation)i;
      }
      return eTriggerActivation::Undef;
    }
    
    enum class eTriggerMode{
      Off, On, Undef
	};
    static const char * strTriggerMode[(unsigned long long)eTriggerMode::Undef];
    eTriggerMode findTriggerMode(const string & str)
    {
      for (int i = 0; i != (int)eTriggerMode::Undef; i++){
	if (strTriggerMode[i] == str)
	  return (eTriggerMode)i;
      }
      return eTriggerMode::Undef;
    }
    
    enum class eTriggerSelector{
      FrameStart, AcquisitionStart, AcquisitionEnd, AcquisitionRecord, Undef
		};
    static const char * strTriggerSelector[(unsigned long long)eTriggerSelector::Undef];
    eTriggerSelector findTriggerSelector(const string & str)
    {
      for (int i = 0; i != (int)eTriggerSelector::Undef; i++){
	if (strTriggerSelector[i] == str)
	  return (eTriggerSelector)i;
      }
      return eTriggerSelector::Undef;
    }
    
    enum class eTriggerSource{
      Freerun = 0, Line1, Line2, Line3, Line4, FixedRate, Software, Action0, Action1, Undef
	};
    static const char * strTriggerSource[(unsigned long long)eTriggerSource::Undef];
    eTriggerSource findTriggerSource(const string & str)
    {
      for (int i = 0; i != (int)eTriggerSource::Undef; i++){
	if (strTriggerSource[i] == str)
	  return (eTriggerSource)i;
      }
      return eTriggerSource::Undef;
    }
    
    enum class eExposureAuto{
      Off, Once, Continuous, Undef
	};
    static const char * strExposureAuto[(unsigned long long)eExposureAuto::Undef];
    eExposureAuto findExposureAuto(const string & str)
    {
      for (int i = 0; i != (int)eExposureAuto::Undef; i++){
	if (strExposureAuto[i] == str)
	  return (eExposureAuto)i;
      }
      return eExposureAuto::Undef;
    }
    
    enum class eExposureAutoAlg{
      Mean, FitRange, Undef
	};
    static const char * strExposureAutoAlg[(unsigned long long)eExposureAutoAlg::Undef];
    eExposureAutoAlg findExposureAutoAlg(const string & str)
    {
      for (int i = 0; i != (int)eExposureAutoAlg::Undef; i++){
	if (strExposureAutoAlg[i] == str)
	  return (eExposureAutoAlg)i;
      }
      return eExposureAutoAlg::Undef;
    }
    
    enum class eExposureMode{
      Timed, TriggerWidth, PieceWiseLinearHDR, Undef
	};
    static const char * strExposureMode[(unsigned long long)eExposureMode::Undef];
    eExposureMode findExposureMode(const string & str)
    {
      for (int i = 0; i != (int)eExposureMode::Undef; i++){
	if (strExposureMode[i] == str)
	  return (eExposureMode)i;
      }
      return eExposureMode::Undef;
    }
    
    enum class eGainAuto{
      Off, Once, Continuous, Undef
	};
    static const char * strGainAuto[(unsigned long long)eGainAuto::Undef];
    eGainAuto findGainAuto(const string & str)
    {
      for (int i = 0; i != (int)eGainAuto::Undef; i++){
	if (strGainAuto[i] == str)
	  return (eGainAuto)i;
      }
      return eGainAuto::Undef;
    }
    
    enum class eBalanceRatioSelector{
      Red, Blue, Undef
	};
    static const char * strBalanceRatioSelector[(unsigned long long)eBalanceRatioSelector::Undef];
    eBalanceRatioSelector findBalanceRatioSelector(const string & str)
    {
      for (int i = 0; i != (int)eBalanceRatioSelector::Undef; i++){
	if (strBalanceRatioSelector[i] == str)
	  return (eBalanceRatioSelector)i;
      }
      return eBalanceRatioSelector::Undef;
		}
    
    enum class eBalanceWhiteAuto{
      Off, Once, Continuous, Undef
	};
    static const char * strBalanceWhiteAuto[(unsigned long long)eBalanceWhiteAuto::Undef];
    eBalanceWhiteAuto findBalanceWhiteAuto(const string & str)
    {
      for (int i = 0; i != (int)eBalanceWhiteAuto::Undef; i++){
	if (strBalanceWhiteAuto[i] == str)
	  return (eBalanceWhiteAuto)i;
      }
      return eBalanceWhiteAuto::Undef;
    }
    
    enum class eBandwidthControlMode{
      StreamBytesPerSecond, SCPD, Both, Undef
	};
    static const char * strBandwidthControlMode[(unsigned long long)eBandwidthControlMode::Undef];
    eBandwidthControlMode findBandwidthControlMode(const string & str)
    {
      for (int i = 0; i != (int)eBandwidthControlMode::Undef; i++){
	if (strBandwidthControlMode[i] == str)
	  return (eBandwidthControlMode)i;
      }
      return eBandwidthControlMode::Undef;
    }
    
    enum class ePixelFormat {
      Mono8, Mono10, Mono12, Mono12Packed, Mono14,
	BayerGB8, BayerRG8, BayerGR8, BayerBG8,
	BayerBG10, BayerGB12Packed, BayerGR12Packed,
	BayerGB12, BayerRG12, BayerGR12, Bayer8Packed,
	BGR8Packed, RGBA8Packed, BGRA8Packed,
	RGB12Packed, YUV411Packed, YUV422Packed,
	YUV444Packed, Undef
	};
    static const char * strPixelFormat[(unsigned long long)ePixelFormat::Undef];
    ePixelFormat findPixelFormat(const string & str)
    {
      for (int i = 0; i != (int)ePixelFormat::Undef; i++){
	if (strPixelFormat[i] == str)
	  return (ePixelFormat)i;
      }
      return ePixelFormat::Undef;
    }
    
    struct s_cam_params{
      int icam;
      char * strFeature[FeatureUndef];
      int GVCPCmdRetries;
      int GVCPCmdTimeout;
      eAcquisitionMode AcquisitionMode;
      eTriggerSource TriggerSource;
      float AcquisitionFrameRateAbs;
      eSensorShutterMode SensorShutterMode;
      eTriggerActivation TriggerActivation;
      float TriggerDelayAbs;
      eTriggerMode TriggerMode;
      eTriggerSelector TriggerSelector;
      int DSPSubregionBottom, DSPSubregionLeft, DSPSubregionRight, DSPSubregionTop;
      eExposureAuto ExposureAuto;
      int ExposureAutoAdjustTol;
      eExposureAutoAlg ExposureAutoAlg;
      int ExposureAutoMax, ExposureAutoMin;
      int ExposureAutoOutliers;
      int ExposureAutoRate;
      int ExposureAutoTarget;
      eExposureMode ExposureMode;
      float ExposureTimeAbs, ExposureTimePWL1, ExposureTimePWL2;
      float Gain;
      int GainAutoAdjustTol;
      float GainAutoMax, GainAutoMin;
      eGainAuto GainAuto;
      int GainAutoOutliers, GainAutoRate, GainAutoTarget;
      float BlackLevel;
      float BalanceRatioAbs;
      eBalanceRatioSelector BalanceRatioSelector;
      eBalanceWhiteAuto BalanceWhiteAuto;
      int BalanceWhiteAutoAdjustTol;
      int BalanceWhiteAutoRate;
      eBandwidthControlMode BandwidthControlMode;
      int GevSCPSPacketSize;
      int StreamBytesPerSecond;
      bool StreamFrameRateConstrain;
      int Height;
      int OffsetX, OffsetY;
      ePixelFormat PixelFormat;
      int Width;
      bool ReverseX, ReverseY;
      int PayloadSize;
      
      CameraPtr pcam;
      char addr[1024];
      int nfrmbuf;
      bool update;
      bool verb;
      ch_image_ref * pch;
      e_imfmt fmt_out;
      
      FramePtrVector frmbuf;
      IFrameObserverPtr pObserver;
      bool bopened;
      s_cam_params(int _icam = -1);
      ~s_cam_params();
    };

    template <class T> bool config_param(CameraPtr pcam, FeaturePtr pf, const char * fstr, T & val, const char ** strvals)
      {
	if (VmbErrorSuccess == pcam->GetFeatureByName(fstr, pf)){
	  if (val == T::Undef){
	    string strval;
	    if (VmbErrorSuccess == pf->GetValue(strval)){
	      for (int i = 0; i < (int)T::Undef; i++){
		if (strval == strvals[i])
		  val = (T)i;
	      }
	    }
	    else{
	      cerr << "Failed to get " << fstr << endl;
	      return false;
	    }
	  }
	  else{
	    if (VmbErrorSuccess != pf->SetValue(strvals[(int)val])){
	      cerr << "Failed to set " << fstr << endl;
	      return false;
	    }
	  }
	}
	else{
	  cerr << "Failed to get " << fstr << " feature object" << endl;
	  return false;
	}
	return true;
      }
    
    bool config_param(CameraPtr pcam, FeaturePtr pf, const char * fstr, int & val, const int Undef, const char ** strvals)
    {
      if (VmbErrorSuccess == pcam->GetFeatureByName(fstr, pf)){
	if (val == Undef){
	  string strval;
	  if (VmbErrorSuccess == pf->GetValue(strval)){
	    for (int i = 0; i < Undef; i++){
	      if (strval == strvals[i])
		val = i;
	    }
	  }
	  else{
	    cerr << "Failed to get " << fstr << endl;
	    return false;
	  }
	}
	else{
	  if (VmbErrorSuccess != pf->SetValue(strvals[val])){
	    cerr << "Failed to set " << fstr << endl;
	    return false;
	  }
	}
      }
      else{
	cerr << "Failed to get " << fstr << " feature object" << endl;
	return false;
      }
      return true;
    }
    
    bool config_param(CameraPtr pcam, FeaturePtr pf, const char * fstr, float & val)
    {
      if (VmbErrorSuccess == pcam->GetFeatureByName(fstr, pf)){
	if (val == -FLT_MAX){
	  double _val;
	  if (VmbErrorSuccess == pf->GetValue(_val)){
	    val = (float)_val;
	  }
	  else{
	    cerr << "Failed to get " << fstr << endl;
	    return false;
	  }
	}
	else{
	  double _val = val;
	  if (VmbErrorSuccess != pf->SetValue(_val)){
	    cerr << "Failed to set " << fstr << endl;
	    return false;
	  }
	}
      }
      else{
	cerr << "Failed to get " << fstr << " feature object" << endl;
	return false;
      }
      return true;
    }
    bool config_param(CameraPtr pcam, FeaturePtr pf, const char * fstr, int & val)
    {
      
      if (VmbErrorSuccess == pcam->GetFeatureByName(fstr, pf)){
	if (val == INT_MIN){
	  VmbInt64_t _val;
	  if (VmbErrorSuccess == pf->GetValue(_val)){
	    val = (int)_val;
	  }
	  else{
	    cerr << "Failed to get " << fstr << endl;
	    return false;
	  }
	}
	else{
	  VmbInt64_t _val = val;
	  if (VmbErrorSuccess != pf->SetValue(_val)){
	    cerr << "Failed to set " << fstr << endl;
	    return false;
	  }
	}
      }
      else{
	cerr << "Failed to get " << fstr << " feature object" << endl;
	return false;
      }
      return true;
    }
    bool config_param(CameraPtr pcam, FeaturePtr pf, const char * fstr, bool & val)
    {

      if (VmbErrorSuccess == pcam->GetFeatureByName(fstr, pf)){
	VmbBool_t _val;
	if (VmbErrorSuccess == pf->GetValue(_val)){
	  val = (int)_val;
	}
	else{
	  cerr << "Failed to get " << fstr << endl;
	  return false;
	}
	
	if (_val != val){
	  _val = val;
	  if (VmbErrorSuccess != pf->SetValue(_val)){
	    cerr << "Failed to set " << fstr << endl;
						return false;
	  }
	}
      }
      else{
	cerr << "Failed to get " << fstr << " feature object" << endl;
	return false;
      }
      return true;
    }
    
    bool config_static_params(s_cam_params & par);
    bool config_dynamic_params(s_cam_params & par);
    static int m_num_avt_vmb_cams;
    void register_params(s_cam_params & par);
  public:
    static bool init_interface();
    static void destroy_interface();
    f_avt_vmb_cam(const char * name);
    virtual ~f_avt_vmb_cam();
  };
  
  template<int ncam>
    class f_avt_vmb_cam_impl : public f_avt_vmb_cam
    {
    protected:
      long long ttrig_prev;
      long long ttrig_int;
     
      vector<s_cam_params*> pcam_pars;
    public:
    f_avt_vmb_cam_impl(const char * name) :f_avt_vmb_cam(name), ttrig_prev(0),
	ttrig_int(100 * MSEC)
	  {
	    register_fpar("Ttrig", &ttrig_int, "Interval of the software trigger in 100ns");
	    pcam_pars.resize(ncam, NULL);
	    if (ncam == 1){
	      pcam_pars[0] = (new s_cam_params());
	      register_params(*pcam_pars[0]);
	    }
	    else{
	      for (int icam = 0; icam < ncam; icam++){
		pcam_pars[icam] =  new s_cam_params(icam);
		register_params(*pcam_pars[icam]);
	      }
	    }
	  }
      
      virtual ~f_avt_vmb_cam_impl()
	{
	  for (int icam = 0; icam < ncam; icam++){
	    delete pcam_pars[icam];
	  }
	}
      
      virtual bool init_run()
      {
	// opening and configuring cameras
	for (int icam = 0; icam < ncam; icam++){
	  if (VmbErrorSuccess == psys->OpenCameraByID(pcam_pars[icam]->addr, VmbAccessModeFull, pcam_pars[icam]->pcam)){
	    cout << icam << "th camera at " << pcam_pars[icam]->addr << " is opened." << endl;
	    pcam_pars[icam]->bopened = true;					
	  }
	  else{
	    cout << icam << "th camera at " << pcam_pars[icam]->addr << " cannot be opened." << endl;
	    
	    return false;
	  }
	  
	  FeaturePtr pFeature;
	  if (pcam_pars[icam]->GevSCPSPacketSize == INT_MIN){ // if packet size is not specified
	    pcam_pars[icam]->pcam->GetFeatureByName("GVSPAdjustPacketSize", pFeature);
	    pFeature->RunCommand();
	  }
	  if (config_static_params(*pcam_pars[icam]) == false){
	    return false;
	  }
	  pcam_pars[icam]->update = false;
	}
	
	// allocating frame buffer
	for (int icam = 0; icam < ncam; icam++){
	  pcam_pars[icam]->pObserver.reset(new FrameObserver(pcam_pars[icam]->pcam, pcam_pars[icam]->pch, pcam_pars[icam]->fmt_out));
	  pcam_pars[icam]->frmbuf.resize(pcam_pars[icam]->nfrmbuf);
	  for (int ifrm = 0; ifrm < pcam_pars[icam]->nfrmbuf; ifrm++){
	    pcam_pars[icam]->frmbuf[ifrm].reset(new Frame(pcam_pars[icam]->PayloadSize));
	    pcam_pars[icam]->frmbuf[ifrm]->RegisterObserver(pcam_pars[icam]->pObserver);
	    pcam_pars[icam]->pcam->QueueFrame(pcam_pars[icam]->frmbuf[ifrm]);
	  }
	}
	
	// starting acquisition
	for (int icam = 0; icam < ncam; icam++){
	  FeaturePtr pFeature;
	  pcam_pars[icam]->pcam->GetFeatureByName("AcquisitionStart", pFeature);
	  pcam_pars[icam]->pcam->StartCapture();
	  pFeature->RunCommand();
	}
	return true;
      }
      
      virtual void destroy_run()
      {
	for (int icam = 0; icam < ncam; icam++){
	  if(!pcam_pars[icam]->bopened)
	    continue;
	  FeaturePtr pFeature;
	  pcam_pars[icam]->pcam->GetFeatureByName("AcquisitionStop", pFeature);
	  pFeature->RunCommand();
	  pcam_pars[icam]->pcam->EndCapture();
	  pcam_pars[icam]->pcam->FlushQueue();
	  pcam_pars[icam]->pcam->RevokeAllFrames();
	  if (VmbErrorSuccess == pcam_pars[icam]->pcam->Close())
	    {
	      cout << icam << "th camera at " << pcam_pars[icam]->addr << " closed." << endl;	      
	    }
	  long long td = f_base::m_clk.get_time_from_start();
	  FrameObserver * pObserver = dynamic_cast<FrameObserver*>(pcam_pars[icam]->pObserver.get());
	  double fps_drop =  (double)SEC * (double)pObserver->getNumDroppedFrames() / (double) td;

	  double fps_rcv = (double) SEC * (double)pObserver->getNumRecievedFrames() / (double) td;

	  cout << icam << "th camera fps:" << fps_rcv << " dps:" << fps_drop << endl;
	}
      }
      
      virtual bool proc()
      {
	
	if (ttrig_prev + ttrig_int <= m_cur_time){
	  FeaturePtrVector pFeatures;
	  pFeatures.reserve(ncam);
	  for (int icam = 0; icam < ncam; icam++){
	    if (pcam_pars[icam]->TriggerSource == eTriggerSource::Software){
	      FeaturePtr pFeature;
	      pcam_pars[icam]->pcam->GetFeatureByName("TriggerSoftware", pFeature);
	      if(pcam_pars[icam]->verb){
		cout << m_time_str << "cam[" << icam << "] triggered" << endl;
	      }
	      pFeatures.push_back(pFeature);
	    }
	  }
	  
	  for (FeaturePtrVector::iterator itr = pFeatures.begin();
	       itr != pFeatures.end(); itr++)
	    (**itr).RunCommand();
	  
	  ttrig_prev = m_cur_time;
	}
	
	for(int icam = 0; icam < ncam; icam++){
	  if(pcam_pars[icam]->update){
	    if(config_dynamic_params(*pcam_pars[icam]) == false){
	      cerr << "Failed to update parameters of cam["
		   << icam << "]." << endl;		  
	    }
	    pcam_pars[icam]->update = false;
	  }
	}
	return true;
      }
    };
};
#endif
