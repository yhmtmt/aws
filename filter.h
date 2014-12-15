#include "filter/f_base.h"
#include "filter/f_misc.h"
#include "filter/f_stabilizer.h"
#include "filter/f_cam.h"
#ifdef AVT_CAM
#include "filter/f_avt_cam.h"
#endif
#ifdef SANYO_HD5400
#include "filter/f_netcam.h"
#endif
#include "filter/f_imgshk.h"
#ifdef _WIN32
#include "filter/f_imgs.h"
#include "filter/f_ds_video.h"
#else
#include "filter/f_uvc_cam.h"
#endif
#ifdef FWINDOW
#include "filter/f_window.h"
#include "filter/f_inspector.h"
#endif
#include "filter/f_nmea.h"
#include "filter/f_shioji.h"
#include "filter/f_ship_detector.h"
#include "filter/f_camcalib.h"
#include "filter/f_com.h"
