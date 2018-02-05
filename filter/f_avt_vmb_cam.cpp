
#include "stdafx.h"
#include <cstdio>
#include <cstring>
#define _USE_MATH_DEFINES
#include <cmath>

#include <iostream>
#include <fstream>
#include <vector>

#include <list>

#include "../util/aws_stdlib.h"
#include "../util/aws_thread.h"
#include "../util/c_clock.h"

using namespace std;

#include <opencv2/opencv.hpp>
using namespace cv;

#include "f_avt_vmb_cam.h"

namespace avt_vmb_cam{
  VimbaSystem * f_avt_vmb_cam::psys = NULL;
  int f_avt_vmb_cam::m_num_avt_vmb_cams = 0;
  
  const char * f_avt_vmb_cam::strFeature[FeatureUndef] =
    {
      "GVCPCmdRetries",
      "GVCPCmdTimeout",
      "AcquisitionMode", 
      "AcquisitionFrameRateAbs", 
      "SensorShutterMode",
      "TriggerActivation", 
      "TriggerDelayAbs",
      "TriggerMode", // one for cameras 
      "TriggerSelector", 
      "TriggerSource",// one for cameras
      "DSPSubregionBottom", 
      "DSPSubregionLeft", 
      "DSPSubregionRight", 
      "DSPSubregionTop",
      "ExposureAuto", 
      "ExposureAutoAdjustTol", 
      "ExposureAutoAlg", 
      "ExposureAutoMax",
      "ExposureAutoMin", 
      "ExposureAutoOutliers", 
      "ExposureAutoRate",
      "ExposureAutoTarget", 
      "ExposureMode", 
      "ExposureTimeAbs",
      "ExposureTimePWL1", 
      "ExposureTimePWL2",
      "Gain", 
      "GainAuto", 
      "GainAutoAdjustTol", 
      "GainAutoMax", 
      "GainAutoMin",
      "GainAutoOutliers", 
      "GainAutoRate", 
      "GainAutoTarget",
      "BlackLevel",
      "BalanceRatioAbs", 
      "BalanceRatioSelector", 
      "BalanceWhiteAuto",
      "BalanceWhiteAutoAdjustTol", 
      "BalanceWhiteAutoRate",
      "BandwidthControlMode", 
      "GevSCPSPacketSize", 
      "StreamBytesPerSecond",
      "StreamFrameRateConstrain",
      "Height", 
      "OffsetX", 
      "OffsetY", 
      "PixelFormat", 
      "Width",
      "ReverseX", 
      "ReverseY",
      "PayloadSize",
      "nfrmbuf",
      "addr",
      "update",
      "verb",
      "channel",
      "fmt_out",
      "fcampar"
    };
  
  const char * f_avt_vmb_cam::expFeature[FeatureUndef] = {
    "Controls Maximum number of resend requests that the host will attempt when trying to recover a lost",
    "The timeout waiting for an answer from the camera",
    "Determines the behaviour of the camera if the acquisition start is triggered.",
    "Specifies frame rate if TriggerSelector=FrameStart TriggerMode=off TriggerSource=FixedRate",
    "Type of the shutter",
    "Type of Activation",
    "Start of image can be delayed to the time after a trigger event is received by the camera.",
    "Controls the trigger set in TriggerSelector",
    "Select a trigger to setup and read the trigger features",
    "Determines how an image frame is initiated within an acquisition stream.",
    "Defines the bottom edge of the DSP subregion",
    "Defines the left edge of the DSP subregion",
    "Defines the right edge of the DSP subregion",
    "Defines the top edge of the DSP subregion",
    "Apply the Auto algorithm setting to the next image",
    "Tolerance in variation from ExposureAutoTarget in whidh the auto exposure algorithm will not respond.",
    "Algorithm used to calculate auto exposure.",
    "The upper bound to the exposure setting in auto exposure mode.",
    "The lower bound to the exposure setting in auto exposure mode.",
    "The total pixels from top of the distribution that are ignored by the auto exposure algorithm",
    "The rate at which the auto exposure function changes the exposure setting",
    "The general lightness or darkness of the auto exposure feature.",
    "The control for exposure duration", 
    "The sensor integration time", 
    "Exposure time after ThresholdPWL1 is reached", 
    "Exposure time after ThresholdPWL2 is reached", 
    "The gain setting applied to the sensor.", 
    "Auto Gain algorithm setting.", 
    "Tolerance in variation from GainAutoTarget in which the auto exposure algorithm will not respond.", 
    "The upper bound to the gain setting in auto gain mode", 
    "The lower bound to the gain setting in auto gain mode",
    "The total pixels from top of the distribution that are ignored by the auto gain algorithm.", 
    "The rate at which the auto gain function changes. A percentage of the maximum rate.", 
    "The general lightness or darkness of the auto gain feature. A percentage of maximum brightness.",
    "The black level value.",
    "Adjusts the gain of the channel selected in the BalanceRatioSelector.", 
    "Select the red or blue channel to adjust with BalanceRatioAbs", 
    "Auto White Balance algorithm ",
    "Tolerance allowed from the ideal white balance values, whithin which the auto white balance does not run.", 
    "The rate of white balance adjustment.",
    "Selects the desired mode of bandwidth control.", 
    "Determines the ethernet packet size.", 
    "Moderates the data rate of the camera",
    "If true, the camerra automatically limits frame rate to bandwidth.",
    "The height of the image", 
    "The starting column of the readout region.", 
    "The starting row of the readout region", 
    "PixelFormat", 
    "The width of the image",
    "Flip the image sent by camera horizontally", 
    "Flip the image sent by camera vertically",
    "Payload size",
    "Number of frames in queue.",
    "IP address of the camera",
    "Updates dynamic parameters",
    "Verbose for debug",    
    "Image channel",
    "Output format",
    "File path to the camera parameter"
  };
  
  const char * f_avt_vmb_cam::strAcquisitionMode[(unsigned long long)eAcquisitionMode::Undef] = {
    "Continuous", "SingleFrame", "MultiFrame", "Recorder"
  };
  
  const char * f_avt_vmb_cam::strTriggerSource[(unsigned long long)eTriggerSource::Undef] = {
    "Freerun", "Line1", "Line2", "Line3", "Line4",
    "FixedRate", "Software", "Action0", "Action1"
  };
  
  const char * f_avt_vmb_cam::strSensorShutterMode[(unsigned long long)eSensorShutterMode::Undef] = {
    "Global", "Rolling", "GlobalReset"
  };
  
  const char * f_avt_vmb_cam::strTriggerActivation[(unsigned long long)eTriggerActivation::Undef] ={
    "RisingEdge", "FallingEdge", "AnyEdge", "LevelHigh", "LevelLow"
  };
  
  const char * f_avt_vmb_cam::strTriggerMode[(unsigned long long)eTriggerMode::Undef] = {
    "Off", "On"
  };
  
  const char * f_avt_vmb_cam::strTriggerSelector[(unsigned long long)eTriggerSelector::Undef] = {
    "FrameStart", "AcquisitionStart", "AcquisitionEnd", "AcquisitionRecord"
  };
  
  const char * f_avt_vmb_cam::strExposureAuto[(unsigned long long)eExposureAuto::Undef] = {
    "Off", "Once", "Continuous"
  };
  
  const char * f_avt_vmb_cam::strExposureAutoAlg[(unsigned long long)eExposureAutoAlg::Undef] = {
    "Mean", "FitRange"
  };
  
  const char * f_avt_vmb_cam::strExposureMode[(unsigned long long)eExposureMode::Undef] =
    {
      "Timed", "TriggerWidth", "PieceWiseLinearHDR"
    };
  
  const char * f_avt_vmb_cam::strGainAuto[(unsigned long long)eGainAuto::Undef] = {
    "Off", "Once", "Continuous"
  };
  
  const char * f_avt_vmb_cam::strBalanceRatioSelector[(unsigned long long)eBalanceRatioSelector::Undef] = {
    "Red", "Blue"
  };
  
  const char * f_avt_vmb_cam::strBalanceWhiteAuto[(unsigned long long)eBalanceWhiteAuto::Undef] = {
    "Off", "Once", "Continuous"
  };
  
  const char * f_avt_vmb_cam::strBandwidthControlMode[(unsigned long long)eBandwidthControlMode::Undef] = {
    "StreamBytesPerSecond", "SCPD", "Both"
  };
  
  const char * f_avt_vmb_cam::strPixelFormat[(unsigned long long)ePixelFormat::Undef]
  {
    "Mono8", "Mono10", "Mono12", "Mono12Packed", "Mono14",
      "BayerGB8", "BayerRG8", "BayerGR8", "BayerBG8",
      "BayerBG10", "BayerGB12Packed", "BayerGR12Packed",
      "BayerGB12", "BayerRG12", "BayerGR12", "Bayer8Packed",
      "BGR8Packed", "RGBA8Packed", "BGRA8Packed",
      "RGB12Packed", "YUV411Packed", "YUV422Packed",
      "YUV444Packed"
      };
  
  bool f_avt_vmb_cam::init_interface()
  {
    cout << "Initializing Vimba interface ";
    if (psys == NULL){
      psys = &VimbaSystem::GetInstance();
    }
    psys->Startup();
    cout << " ... done" << endl;
    return true;
  }
  
  void f_avt_vmb_cam::destroy_interface()
  {
    cout << "Shutting down Vimba interface ";
    psys->Shutdown();
    psys = NULL;
    cout << " ... done" << endl;
  }
  
  f_avt_vmb_cam::f_avt_vmb_cam(const char * name) : f_base(name)
  {
    cout << "Creating " << name;
    if (m_num_avt_vmb_cams == 0){
      init_interface();
    }
    m_num_avt_vmb_cams++;
    cout << " ... done." << endl;
  }
  
  f_avt_vmb_cam::~f_avt_vmb_cam()
  {
    cout << "Destructing " << m_name;
    m_num_avt_vmb_cams--;
    if (m_num_avt_vmb_cams == 0){
      
      destroy_interface();
    }
    cout << " ... done." << endl;
  }
  
  void f_avt_vmb_cam::register_params(s_cam_params & par)
  {
    char ** pstr = par.strFeature;
    register_fpar(pstr[GVCPCmdRetries], &par.GVCPCmdRetries, expFeature[GVCPCmdRetries]);
    register_fpar(pstr[GVCPCmdTimeout], &par.GVCPCmdRetries, expFeature[GVCPCmdTimeout]);
    register_fpar(pstr[AcquisitionMode], (int*)&par.AcquisitionMode, (int)eAcquisitionMode::Undef, strAcquisitionMode, expFeature[AcquisitionMode]);
    register_fpar(pstr[AcquisitionFrameRateAbs], &par.AcquisitionFrameRateAbs, expFeature[AcquisitionFrameRateAbs]);
    register_fpar(pstr[SensorShutterMode], (int*)&par.SensorShutterMode, (int)eSensorShutterMode::Undef, strSensorShutterMode, expFeature[SensorShutterMode]);
    register_fpar(pstr[TriggerActivation], (int*)&par.TriggerActivation, (int)eTriggerActivation::Undef, strTriggerActivation, expFeature[TriggerActivation]);
    register_fpar(pstr[TriggerDelayAbs], &par.TriggerDelayAbs, expFeature[TriggerDelayAbs]);
    register_fpar(pstr[TriggerMode], (int*)&par.TriggerMode, (int)eTriggerMode::Undef, strTriggerMode, expFeature[TriggerMode]);
    register_fpar(pstr[TriggerSelector], (int*)&par.TriggerSelector, (int)eTriggerSelector::Undef, strTriggerSelector, expFeature[TriggerSelector]);
    register_fpar(pstr[TriggerSource], (int*)&par.TriggerSource, (int)eTriggerSource::Undef, strTriggerSource, expFeature[TriggerSource]);		
    register_fpar(pstr[DSPSubregionBottom], &par.DSPSubregionBottom, expFeature[DSPSubregionBottom]);
    register_fpar(pstr[DSPSubregionLeft], &par.DSPSubregionLeft, expFeature[DSPSubregionLeft]);
    register_fpar(pstr[DSPSubregionRight], &par.DSPSubregionRight, expFeature[DSPSubregionRight]);
    register_fpar(pstr[DSPSubregionTop], &par.DSPSubregionTop, expFeature[DSPSubregionTop]);
    register_fpar(pstr[ExposureAuto], (int*)&par.ExposureAuto, (int)eExposureAuto::Undef, strExposureAuto, expFeature[ExposureAuto]);
    register_fpar(pstr[ExposureAutoAdjustTol], &par.ExposureAutoAdjustTol, expFeature[ExposureAutoAdjustTol]);
    register_fpar(pstr[ExposureAutoAlg], (int*)&par.ExposureAutoAlg, (int)eExposureAutoAlg::Undef, strExposureAutoAlg, expFeature[ExposureAutoAlg]);
    register_fpar(pstr[ExposureAutoMax], &par.ExposureAutoMax, expFeature[ExposureAutoMax]);
    register_fpar(pstr[ExposureAutoMin], &par.ExposureAutoMin, expFeature[ExposureAutoMin]);
    register_fpar(pstr[ExposureAutoOutliers], &par.ExposureAutoOutliers, expFeature[ExposureAutoOutliers]);
    register_fpar(pstr[ExposureAutoRate], &par.ExposureAutoRate, expFeature[ExposureAutoRate]);
    register_fpar(pstr[ExposureAutoTarget], &par.ExposureAutoTarget, expFeature[ExposureAutoTarget]);
    register_fpar(pstr[ExposureMode], (int*)&par.ExposureMode, (int)eExposureMode::Undef, strExposureMode, expFeature[ExposureMode]);
    register_fpar(pstr[ExposureTimeAbs], &par.ExposureTimeAbs, expFeature[ExposureTimeAbs]);
    register_fpar(pstr[ExposureTimePWL1], &par.ExposureTimePWL1, expFeature[ExposureTimePWL1]);
    register_fpar(pstr[ExposureTimePWL2], &par.ExposureTimePWL2, expFeature[ExposureTimePWL2]);
    register_fpar(pstr[Gain], &par.Gain, expFeature[Gain]);
    register_fpar(pstr[GainAuto], (int*)&par.GainAuto, (int)eGainAuto::Undef, strGainAuto, expFeature[GainAuto]);
    register_fpar(pstr[GainAutoAdjustTol], (int*)&par.GainAutoAdjustTol, expFeature[GainAutoAdjustTol]);
    register_fpar(pstr[GainAutoOutliers], &par.GainAutoOutliers, expFeature[GainAutoOutliers]);
    register_fpar(pstr[GainAutoRate], &par.GainAutoRate, expFeature[GainAutoRate]);
    register_fpar(pstr[GainAutoTarget], &par.GainAutoTarget, expFeature[GainAutoTarget]);
    register_fpar(pstr[BlackLevel], &par.BlackLevel, expFeature[BlackLevel]);
    register_fpar(pstr[BalanceRatioAbs], &par.BalanceRatioAbs, expFeature[BalanceRatioAbs]);
    register_fpar(pstr[BalanceRatioSelector], (int*)&par.BalanceRatioSelector, (int)eBalanceRatioSelector::Undef, strBalanceRatioSelector, expFeature[BalanceRatioSelector]);
    register_fpar(pstr[BalanceWhiteAuto], (int*)&par.BalanceWhiteAuto, (int)eBalanceWhiteAuto::Undef, strBalanceWhiteAuto, expFeature[BalanceWhiteAuto]);
    register_fpar(pstr[BalanceWhiteAutoAdjustTol], &par.BalanceWhiteAutoAdjustTol, expFeature[BalanceWhiteAutoAdjustTol]);
    register_fpar(pstr[BalanceWhiteAutoRate], &par.BalanceWhiteAutoRate, expFeature[BalanceWhiteAutoRate]);
    register_fpar(pstr[BandwidthControlMode], (int*)&par.BandwidthControlMode, (int)eBandwidthControlMode::Undef, strBandwidthControlMode, expFeature[BandwidthControlMode]);
    register_fpar(pstr[GevSCPSPacketSize], &par.GevSCPSPacketSize, expFeature[GevSCPSPacketSize]);
    register_fpar(pstr[StreamBytesPerSecond], &par.StreamBytesPerSecond, expFeature[StreamBytesPerSecond]);
    register_fpar(pstr[StreamFrameRateConstrain], &par.StreamFrameRateConstrain, expFeature[StreamFrameRateConstrain]);
    register_fpar(pstr[Height], &par.Height, expFeature[Height]);
    register_fpar(pstr[OffsetX], &par.OffsetX, expFeature[OffsetX]);
    register_fpar(pstr[OffsetY], &par.OffsetY, expFeature[OffsetY]);
    register_fpar(pstr[PixelFormat], (int*)&par.PixelFormat, (int)ePixelFormat::Undef, strPixelFormat, expFeature[PixelFormat]);
    register_fpar(pstr[Width], (int*)&par.Width, expFeature[Width]);
    register_fpar(pstr[ReverseX], &par.ReverseX, expFeature[ReverseX]);
    register_fpar(pstr[ReverseY], &par.ReverseY, expFeature[ReverseY]);
    register_fpar(pstr[nfrmbuf], &par.nfrmbuf, expFeature[nfrmbuf]);
    register_fpar(pstr[addr], par.addr, 1024, expFeature[addr]);
    register_fpar(pstr[update], &par.update, expFeature[update]);
    register_fpar(pstr[verb], &par.verb, expFeature[verb]);    
    register_fpar(pstr[channel], (ch_base**)&par.pch, typeid(ch_image_ref).name(), "Image channel");

    register_fpar(pstr[fmt_out], (int*)&par.fmt_out, (int)IMF_Undef, str_imfmt, expFeature[fmt_out]);
    register_fpar(pstr[fcampar], par.fcampar, 1024, expFeature[fcampar]);
    par.fcampar[0] = '\0';
  }
  
  f_avt_vmb_cam::s_cam_params
  ::s_cam_params(int _icam) :
    icam(_icam),
    GVCPCmdRetries(INT_MIN),
    GVCPCmdTimeout(INT_MIN),
    AcquisitionMode(eAcquisitionMode::Undef),
    AcquisitionFrameRateAbs(-FLT_MAX),
    SensorShutterMode(eSensorShutterMode::Undef),
    TriggerActivation(eTriggerActivation::Undef),
    TriggerDelayAbs(-FLT_MAX),
    TriggerMode(eTriggerMode::Undef),
    TriggerSelector(eTriggerSelector::Undef),
    TriggerSource(eTriggerSource::Undef),
    DSPSubregionBottom(INT_MIN),
    DSPSubregionLeft(INT_MIN),
    DSPSubregionRight(INT_MIN),
    DSPSubregionTop(INT_MIN),
    ExposureAuto(eExposureAuto::Undef),
    ExposureAutoAdjustTol(INT_MIN),
    ExposureAutoAlg(eExposureAutoAlg::Undef),
    ExposureAutoMax(INT_MIN),
    ExposureAutoMin(INT_MIN),
    ExposureAutoOutliers(INT_MIN),
    ExposureAutoRate(INT_MIN),
    ExposureAutoTarget(INT_MIN),
    ExposureMode(eExposureMode::Undef),
    ExposureTimeAbs(-FLT_MAX),
    ExposureTimePWL1(-FLT_MAX),
    ExposureTimePWL2(-FLT_MAX),
    Gain(-FLT_MAX),
    GainAuto(eGainAuto::Undef),
    GainAutoAdjustTol(INT_MIN),
    GainAutoMax(-FLT_MAX),
    GainAutoMin(-FLT_MAX),
    GainAutoOutliers(INT_MIN),
    GainAutoRate(INT_MIN),
    GainAutoTarget(INT_MIN),
    BlackLevel(-FLT_MAX),
    BalanceRatioAbs(-FLT_MAX),
    BalanceRatioSelector(eBalanceRatioSelector::Undef),
    BalanceWhiteAuto(eBalanceWhiteAuto::Undef),
    BalanceWhiteAutoAdjustTol(INT_MIN),
    BalanceWhiteAutoRate(INT_MIN),
    BandwidthControlMode(eBandwidthControlMode::Undef),
    GevSCPSPacketSize(INT_MIN),
    StreamBytesPerSecond(INT_MIN),
    StreamFrameRateConstrain(true),
    Height(INT_MIN),
    OffsetX(INT_MIN),
    OffsetY(INT_MIN),
    PixelFormat(ePixelFormat::Undef),
    Width(INT_MIN),
    ReverseX(false),
    ReverseY(false),
    PayloadSize(INT_MIN),
    nfrmbuf(3),
    verb(false),
    pch(NULL),
    fmt_out(IMF_Undef),
    bopened(false)
  {
    // creating parameter string
    if (icam > 1000){
      cerr << "Error in f_avt_vmb_cam::register_params. Number of camera exceeded the limit." << endl;
      return;
    }
    
    int len_dgt = (icam < 0 ? 0 : icam < 10 ? 1 : icam < 100 ? 2 : 3);
    
    for (int ipar = 0; ipar < FeatureUndef; ipar++){
      int len = strlen(f_avt_vmb_cam::strFeature[ipar]);
      strFeature[ipar] = new char[len + 1 + len_dgt];
      if (icam < 0)
	snprintf(strFeature[ipar], len + 1, "%s", f_avt_vmb_cam::strFeature[ipar]);
      else
	snprintf(strFeature[ipar], len + 1 + len_dgt, "%s%d", f_avt_vmb_cam::strFeature[ipar], icam);
    }
  }
  
  f_avt_vmb_cam::s_cam_params::~s_cam_params()
  {
    for (int ipar = 0; ipar < FeatureUndef; ipar++){
      delete[] strFeature[ipar];
      strFeature[ipar] = NULL;
    }
  }
  
  bool f_avt_vmb_cam::config_static_params(s_cam_params & par)
  {
    CameraPtr pcam = par.pcam;
    if (pcam == NULL){
      cerr << "Error in f_avt_vmb_cam::s_cam_params::config_static_params. Cannot find camera instance." << endl;
      return false;
    }
    
    FeaturePtr pf;
    config_param<eAcquisitionMode>(pcam, pf, strFeature[AcquisitionMode], par.AcquisitionMode, strAcquisitionMode);
    config_param(pcam, pf, strFeature[Height], par.Height);
    config_param(pcam, pf, strFeature[OffsetX], par.OffsetX);
    config_param(pcam, pf, strFeature[OffsetY], par.OffsetY);
    config_param(pcam, pf, strFeature[Width], par.Width);
    config_param<ePixelFormat>(pcam, pf, strFeature[PixelFormat], par.PixelFormat, strPixelFormat);
    if (config_param(pcam, pf, strFeature[PayloadSize], par.PayloadSize) == false){
      return false;
    }
    return config_dynamic_params(par);
  }
  
  bool f_avt_vmb_cam::config_dynamic_params(s_cam_params & par)
  {
    CameraPtr pcam = par.pcam;
    FeaturePtr pf;
    config_param(pcam, pf, strFeature[GVCPCmdRetries], par.GVCPCmdRetries);
    config_param(pcam, pf, strFeature[GVCPCmdTimeout], par.GVCPCmdTimeout);
    config_param(pcam, pf, strFeature[AcquisitionFrameRateAbs], par.AcquisitionFrameRateAbs);
    config_param<eSensorShutterMode>(pcam, pf, strFeature[SensorShutterMode], par.SensorShutterMode, strSensorShutterMode);
    config_param<eTriggerActivation>(pcam, pf, strFeature[TriggerActivation], par.TriggerActivation, strTriggerActivation);
    config_param(pcam, pf, strFeature[TriggerDelayAbs], par.TriggerDelayAbs);
    config_param<eTriggerMode>(pcam, pf, strFeature[TriggerMode], par.TriggerMode, strTriggerMode);
    config_param<eTriggerSelector>(pcam, pf, strFeature[TriggerSelector], par.TriggerSelector, strTriggerSelector);
    config_param<eTriggerSource>(pcam, pf, strFeature[TriggerSource], par.TriggerSource, strTriggerSource);
    config_param(pcam, pf, strFeature[DSPSubregionBottom], par.DSPSubregionBottom);
    config_param(pcam, pf, strFeature[DSPSubregionLeft], par.DSPSubregionLeft);
    config_param(pcam, pf, strFeature[DSPSubregionRight], par.DSPSubregionRight);
    config_param(pcam, pf, strFeature[DSPSubregionTop], par.DSPSubregionTop);
    config_param<eExposureAuto>(pcam, pf, strFeature[ExposureAuto], par.ExposureAuto, strExposureAuto);
    config_param(pcam, pf, strFeature[ExposureAutoAdjustTol], par.ExposureAutoAdjustTol);
    config_param<eExposureAutoAlg>(pcam, pf, strFeature[ExposureAutoAlg], par.ExposureAutoAlg, strExposureAutoAlg);
    config_param(pcam, pf, strFeature[ExposureAutoMax], par.ExposureAutoMax);
    config_param(pcam, pf, strFeature[ExposureAutoMin], par.ExposureAutoMin);
    config_param(pcam, pf, strFeature[ExposureAutoOutliers], par.ExposureAutoOutliers);
    config_param(pcam, pf, strFeature[ExposureAutoRate], par.ExposureAutoRate);
    config_param(pcam, pf, strFeature[ExposureAutoTarget], par.ExposureAutoTarget);
    config_param<eExposureMode>(pcam, pf, strFeature[ExposureMode], par.ExposureMode, strExposureMode);
    config_param(pcam, pf, strFeature[ExposureTimeAbs], par.ExposureTimeAbs);
    config_param(pcam, pf, strFeature[ExposureTimePWL1], par.ExposureTimePWL1);
    config_param(pcam, pf, strFeature[ExposureTimePWL2], par.ExposureTimePWL2);
    config_param(pcam, pf, strFeature[Gain], par.Gain);
    config_param<eGainAuto>(pcam, pf, strFeature[GainAuto], par.GainAuto, strGainAuto);
    config_param(pcam, pf, strFeature[GainAutoAdjustTol], par.GainAutoAdjustTol);
    config_param(pcam, pf, strFeature[GainAutoMax], par.GainAutoMax);
    config_param(pcam, pf, strFeature[GainAutoMin], par.GainAutoMin);
    config_param(pcam, pf, strFeature[GainAutoOutliers], par.GainAutoOutliers);
    config_param(pcam, pf, strFeature[GainAutoRate], par.GainAutoRate);
    config_param(pcam, pf, strFeature[GainAutoTarget], par.GainAutoTarget);
    config_param(pcam, pf, strFeature[BlackLevel], par.BlackLevel);
    config_param(pcam, pf, strFeature[BalanceRatioAbs], par.BalanceRatioAbs);
    config_param<eBalanceRatioSelector>(pcam, pf, strFeature[BalanceRatioSelector], par.BalanceRatioSelector, strBalanceRatioSelector);
    config_param<eBalanceWhiteAuto>(pcam, pf, strFeature[BalanceWhiteAuto], par.BalanceWhiteAuto, strBalanceWhiteAuto);
    config_param(pcam, pf, strFeature[BalanceWhiteAutoAdjustTol], par.BalanceWhiteAutoAdjustTol);
    config_param(pcam, pf, strFeature[BalanceWhiteAutoRate], par.BalanceWhiteAutoRate);
    config_param<eBandwidthControlMode>(pcam, pf, strFeature[BandwidthControlMode], par.BandwidthControlMode, strBandwidthControlMode);
    config_param(pcam, pf, strFeature[GevSCPSPacketSize], par.GevSCPSPacketSize);
    config_param(pcam, pf, strFeature[StreamBytesPerSecond], par.StreamBytesPerSecond);
    config_param(pcam, pf, strFeature[StreamFrameRateConstrain], par.StreamFrameRateConstrain);
    config_param(pcam, pf, strFeature[ReverseX], par.ReverseX);
    config_param(pcam, pf, strFeature[ReverseY], par.ReverseY);
    
    return true;
  }
}
