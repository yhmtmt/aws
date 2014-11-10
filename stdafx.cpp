// stdafx.cpp : 標準インクルード aws.pch のみを
// 含むソース ファイルは、プリコンパイル済みヘッダーになります。
// stdafx.obj にはプリコンパイル済み型情報が含まれます。

#include "stdafx.h"

// TODO: このファイルではなく、STDAFX.H で必要な
// 追加ヘッダーを参照してください。

#ifdef _DEBUG

// OpenCV 2.4.2
#pragma comment(lib, "opencv_core249d.lib")
#pragma comment(lib, "opencv_contrib249d.lib")
#pragma comment(lib, "opencv_features2d249d.lib")
#pragma comment(lib, "opencv_legacy249d.lib")
#pragma comment(lib, "opencv_objdetect249d.lib")
#pragma comment(lib, "opencv_highgui249d.lib")
#pragma comment(lib, "opencv_imgproc249d.lib")
#pragma comment(lib, "opencv_calib3d249d.lib")

#pragma comment(lib, "libcurld_imp.lib")
#pragma comment(lib, "pthreadVC2.lib")
#pragma comment(lib, "libjpegd.lib")
#pragma comment(lib, "ws2_32.lib")
#pragma comment(lib, "Strmiids.lib")
#pragma comment(lib, "Quartz.lib")
#pragma comment(lib, "d3d9.lib")
#pragma comment(lib, "d3dx9d.lib")

#else
// OpenCV 2.4.2s
#pragma comment(lib, "opencv_core249.lib")
#pragma comment(lib, "opencv_contrib249.lib")
#pragma comment(lib, "opencv_features2d249.lib")
#pragma comment(lib, "opencv_legacy249.lib")
#pragma comment(lib, "opencv_objdetect249.lib")
#pragma comment(lib, "opencv_highgui249.lib")
#pragma comment(lib, "opencv_imgproc249.lib")
#pragma comment(lib, "opencv_calib3d249.lib")

#pragma comment(lib, "libcurl_imp.lib")
#pragma comment(lib, "pthreadVC2.lib")
#pragma comment(lib, "libjpeg.lib")
#pragma comment(lib, "ws2_32.lib")
#pragma comment(lib, "Strmiids.lib")
#pragma comment(lib, "Quartz.lib")
#pragma comment(lib, "d3d9.lib")
#pragma comment(lib, "d3dx9.lib")
#endif

#pragma comment(lib, "PvAPI.lib");