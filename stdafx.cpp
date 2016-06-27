// stdafx.cpp : 標準インクルード aws.pch のみを
// 含むソース ファイルは、プリコンパイル済みヘッダーになります。
// stdafx.obj にはプリコンパイル済み型情報が含まれます。

#include "stdafx.h"

// TODO: このファイルではなく、STDAFX.H で必要な
// 追加ヘッダーを参照してください。


#ifdef _DEBUG

/*
// OpenCV 2.4.11
#pragma comment(lib, "opencv_core2411d.lib")
#pragma comment(lib, "opencv_contrib2411d.lib")
#pragma comment(lib, "opencv_features2d2411d.lib")
#pragma comment(lib, "opencv_legacy2411d.lib")
#pragma comment(lib, "opencv_objdetect2411d.lib")
#pragma comment(lib, "opencv_highgui2411d.lib")
#pragma comment(lib, "opencv_imgproc2411d.lib")
#pragma comment(lib, "opencv_calib3d2411d.lib")
*/

// OpenCV3.1.0
#pragma comment(lib, "opencv_aruco310d.lib")
#pragma comment(lib, "opencv_bgsegm310d.lib")
#pragma comment(lib, "opencv_bioinspired310d.lib")
#pragma comment(lib, "opencv_calib3d310d.lib")
#pragma comment(lib, "opencv_ccalib310d.lib")
#pragma comment(lib, "opencv_core310d.lib")
#pragma comment(lib, "opencv_cudaarithm310d.lib")
#pragma comment(lib, "opencv_cudabgsegm310d.lib")
#pragma comment(lib, "opencv_cudacodec310d.lib")
#pragma comment(lib, "opencv_cudafeatures2d310d.lib")
#pragma comment(lib, "opencv_cudafilters310d.lib")
#pragma comment(lib, "opencv_cudaimgproc310d.lib")
#pragma comment(lib, "opencv_cudalegacy310d.lib")
#pragma comment(lib, "opencv_cudaobjdetect310d.lib")
#pragma comment(lib, "opencv_cudaoptflow310d.lib")
#pragma comment(lib, "opencv_cudastereo310d.lib")
#pragma comment(lib, "opencv_cudawarping310d.lib")
#pragma comment(lib, "opencv_cudev310d.lib")
#pragma comment(lib, "opencv_datasets310d.lib")
#pragma comment(lib, "opencv_dnn310d.lib")
#pragma comment(lib, "opencv_dpm310d.lib")
#pragma comment(lib, "opencv_face310d.lib")
#pragma comment(lib, "opencv_features2d310d.lib")
#pragma comment(lib, "opencv_flann310d.lib")
#pragma comment(lib, "opencv_fuzzy310d.lib")
#pragma comment(lib, "opencv_highgui310d.lib")
#pragma comment(lib, "opencv_imgcodecs310d.lib")
#pragma comment(lib, "opencv_imgproc310d.lib")
#pragma comment(lib, "opencv_line_descriptor310d.lib")
#pragma comment(lib, "opencv_ml310d.lib")
#pragma comment(lib, "opencv_objdetect310d.lib")
#pragma comment(lib, "opencv_optflow310d.lib")
#pragma comment(lib, "opencv_photo310d.lib")
#pragma comment(lib, "opencv_plot310d.lib")
#pragma comment(lib, "opencv_reg310d.lib")
#pragma comment(lib, "opencv_rgbd310d.lib")
#pragma comment(lib, "opencv_saliency310d.lib")
#pragma comment(lib, "opencv_shape310d.lib")
#pragma comment(lib, "opencv_stereo310d.lib")
#pragma comment(lib, "opencv_stitching310d.lib")
#pragma comment(lib, "opencv_structured_light310d.lib")
#pragma comment(lib, "opencv_superres310d.lib")
#pragma comment(lib, "opencv_surface_matching310d.lib")
#pragma comment(lib, "opencv_text310d.lib")
#pragma comment(lib, "opencv_tracking310d.lib")
#pragma comment(lib, "opencv_ts310d.lib")
#pragma comment(lib, "opencv_video310d.lib")
#pragma comment(lib, "opencv_videoio310d.lib")
#pragma comment(lib, "opencv_videostab310d.lib")
#pragma comment(lib, "opencv_xfeatures2d310d.lib")
#pragma comment(lib, "opencv_ximgproc310d.lib")
#pragma comment(lib, "opencv_xobjdetect310d.lib")
#pragma comment(lib, "opencv_xphoto310d.lib")


#ifdef SANYO_HD5400
#pragma comment(lib, "libjpegd.lib")
#pragma comment(lib, "libcurld_imp.lib")
#endif
#pragma comment(lib, "pthreadVC2.lib")
#pragma comment(lib, "ws2_32.lib")
#pragma comment(lib, "Strmiids.lib")
#pragma comment(lib, "Quartz.lib")
#pragma comment(lib, "d3d9.lib")

#pragma comment(lib, "d3dx9d.lib")
#pragma comment(lib, "dwrite.lib")
#pragma comment(lib, "d2d1.lib")
#else
/*
// OpenCV 2.4.11
#pragma comment(lib, "opencv_core2411.lib")
#pragma comment(lib, "opencv_contrib2411.lib")
#pragma comment(lib, "opencv_features2d2411.lib")
#pragma comment(lib, "opencv_legacy2411.lib")
#pragma comment(lib, "opencv_objdetect2411.lib")
#pragma comment(lib, "opencv_highgui2411.lib")
#pragma comment(lib, "opencv_imgproc2411.lib")
#pragma comment(lib, "opencv_calib3d2411.lib")
*/

// OpenCV 3.1.0
#pragma comment(lib, "opencv_aruco310.lib")
#pragma comment(lib, "opencv_bgsegm310.lib")
#pragma comment(lib, "opencv_bioinspired310.lib")
#pragma comment(lib, "opencv_calib3d310.lib")
#pragma comment(lib, "opencv_ccalib310.lib")
#pragma comment(lib, "opencv_core310.lib")
#pragma comment(lib, "opencv_cudaarithm310.lib")
#pragma comment(lib, "opencv_cudabgsegm310.lib")
#pragma comment(lib, "opencv_cudacodec310.lib")
#pragma comment(lib, "opencv_cudafeatures2d310.lib")
#pragma comment(lib, "opencv_cudafilters310.lib")
#pragma comment(lib, "opencv_cudaimgproc310.lib")
#pragma comment(lib, "opencv_cudalegacy310.lib")
#pragma comment(lib, "opencv_cudaobjdetect310.lib")
#pragma comment(lib, "opencv_cudaoptflow310.lib")
#pragma comment(lib, "opencv_cudastereo310.lib")
#pragma comment(lib, "opencv_cudawarping310.lib")
#pragma comment(lib, "opencv_cudev310.lib")
#pragma comment(lib, "opencv_datasets310.lib")
#pragma comment(lib, "opencv_dnn310.lib")
#pragma comment(lib, "opencv_dpm310.lib")
#pragma comment(lib, "opencv_face310.lib")
#pragma comment(lib, "opencv_features2d310.lib")
#pragma comment(lib, "opencv_flann310.lib")
#pragma comment(lib, "opencv_fuzzy310.lib")
#pragma comment(lib, "opencv_highgui310.lib")
#pragma comment(lib, "opencv_imgcodecs310.lib")
#pragma comment(lib, "opencv_imgproc310.lib")
#pragma comment(lib, "opencv_line_descriptor310.lib")
#pragma comment(lib, "opencv_ml310.lib")
#pragma comment(lib, "opencv_objdetect310.lib")
#pragma comment(lib, "opencv_optflow310.lib")
#pragma comment(lib, "opencv_photo310.lib")
#pragma comment(lib, "opencv_plot310.lib")
#pragma comment(lib, "opencv_reg310.lib")
#pragma comment(lib, "opencv_rgbd310.lib")
#pragma comment(lib, "opencv_saliency310.lib")
#pragma comment(lib, "opencv_shape310.lib")
#pragma comment(lib, "opencv_stereo310.lib")
#pragma comment(lib, "opencv_stitching310.lib")
#pragma comment(lib, "opencv_structured_light310.lib")
#pragma comment(lib, "opencv_superres310.lib")
#pragma comment(lib, "opencv_surface_matching310.lib")
#pragma comment(lib, "opencv_text310.lib")
#pragma comment(lib, "opencv_tracking310.lib")
#pragma comment(lib, "opencv_ts310.lib")
#pragma comment(lib, "opencv_video310.lib")
#pragma comment(lib, "opencv_videoio310.lib")
#pragma comment(lib, "opencv_videostab310.lib")
#pragma comment(lib, "opencv_xfeatures2d310.lib")
#pragma comment(lib, "opencv_ximgproc310.lib")
#pragma comment(lib, "opencv_xobjdetect310.lib")
#pragma comment(lib, "opencv_xphoto310.lib")

#ifdef SANYO_HD5400
#pragma comment(lib, "libcurl_imp.lib")
#pragma comment(lib, "libjpeg.lib")
#endif
#pragma comment(lib, "pthreadVC2.lib")
#pragma comment(lib, "ws2_32.lib")
#pragma comment(lib, "Strmiids.lib")
#pragma comment(lib, "Quartz.lib")
#pragma comment(lib, "d3d9.lib")
#pragma comment(lib, "d3dx9.lib")
#pragma comment(lib, "dwrite.lib")
#pragma comment(lib, "d2d1.lib")
#endif

#ifdef AVT_CAM
#pragma comment(lib, "PvAPI.lib")
#endif

#ifdef GLFW_WINDOW
#pragma comment(lib, "OpenGL32.lib")
#pragma comment(lib, "glfw3.lib")
#pragma comment(lib, "glu32.lib")
#pragma comment(lib, "glew32.lib")
#ifdef _DEBUG
#pragma comment(lib, "freeglutd.lib")
#else
#pragma comment(lib, "freeglut.lib")
#endif
#endif