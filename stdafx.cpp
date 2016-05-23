// stdafx.cpp : 標準インクルード aws.pch のみを
// 含むソース ファイルは、プリコンパイル済みヘッダーになります。
// stdafx.obj にはプリコンパイル済み型情報が含まれます。

#include "stdafx.h"

// TODO: このファイルではなく、STDAFX.H で必要な
// 追加ヘッダーを参照してください。

//#pragma comment(lib, "opencv_ffmpeg310_64.lib")

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
#pragma comment(lib, "opencv_core310d.lib")
//#pragma comment(lib, "opencv_contrib310d.lib")
#pragma comment(lib, "opencv_features2d310d.lib")
//#pragma comment(lib, "opencv_legacy310d.lib")
#pragma comment(lib, "opencv_objdetect310d.lib")
#pragma comment(lib, "opencv_highgui310d.lib")
#pragma comment(lib, "opencv_imgproc310d.lib")
#pragma comment(lib, "opencv_calib3d310d.lib")
#pragma comment(lib, "opencv_imgcodecs310d.lib")
#pragma comment(lib, "opencv_video310d.lib")
#pragma comment(lib, "opencv_videoio310d.lib")


#ifdef SANYO_HD5400
#pragma comment(lib, "libjpegd.lib")
#pragma comment(lib, "libcurld_imp.lib")
#endif
#pragma comment(lib, "pthreadVC2.lib")
#pragma comment(lib, "ws2_32.lib")
#pragma comment(lib, "Strmiids.lib")
#pragma comment(lib, "Quartz.lib")
#pragma comment(lib, "d3d9.lib")cd

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
#pragma comment(lib, "opencv_core310.lib")
//#pragma comment(lib, "opencv_contrib310.lib")
#pragma comment(lib, "opencv_features2d310.lib")
//#pragma comment(lib, "opencv_legacy310.lib")
#pragma comment(lib, "opencv_objdetect310.lib")
#pragma comment(lib, "opencv_highgui310.lib")
#pragma comment(lib, "opencv_imgproc310.lib")
#pragma comment(lib, "opencv_calib3d310.lib")
#pragma comment(lib, "opencv_imgcodecs310.lib")
#pragma comment(lib, "opencv_video310.lib")
#pragma comment(lib, "opencv_videoio310.lib")

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
#pragma comment(lib, "glfw3dll.lib")
#pragma comment(lib, "glu32.lib")
#pragma comment(lib, "glew32.lib")
#ifdef _DEBUG
#pragma comment(lib, "freeglutd.lib")
#else
#pragma comment(lib, "freeglut.lib")
#endif
#endif