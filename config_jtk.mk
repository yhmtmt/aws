CC	= g++
CPU 	= arm

# n: disable all the window filter later
WINDOW = y	
FWINDOW = n
GLFW_WINDOW = y

# n: disable all the imaging and image processing filter later
IMGPROC = y
AVT_CAM = y
VMB_CAM = n
UVC_CAM = y
GST_CAM = y
SANYO_HD5400 = n
ORB_SLAM = n
STEREO = y
CAMCALIB = y
STABILIZER = y
MISC = y

############################################################ Path configuration
CUR_DIR = $(shell pwd)
FDIR = $(CUR_DIR)/filter
CDIR = $(CUR_DIR)/channel
UDIR = $(CUR_DIR)/util
RCMD_DIR = $(CUR_DIR)/rcmd
INC_CV_DIR = /usr/local/inlude
LIB_CV_DIR = /usr/local/lib
INC_PVAPI_DIR = $(CUR_DIR)/PvAPI/include
LIB_PVAPI_DIR = $(CUR_DIR)/PvAPI/lib
INC_VMB_DIR = /mnt/ssd1/Vimba_2_1
LIB_VMB_DIR = /mnt/ssd1/Vimba_2_1/VimbaCPP/DynamicLib/arm_64bit
INC_GLFW_DIR = $(CUR_DIR)/GLFW/include
LIB_GLFW_DIR = $(CURDIR)/GLFW/lib
INC_EIGEN_DIR = /usr/include/eigen3
INC_MAVLINK = $(CUR_DIR)/mavlink/include_1.0/ardupilotmega
INC_GST = /usr/include/gstreamer-1.0
LIB_GST = /usr/lib/arm-linux-gnueabihf/gstreamer-1.0
INC_GLIB = /usr/include/glib-2.0
INC_GLIB_CONFIG = /usr/lib/arm-linux-gnueabihf/glib-2.0/include
INC_GLM = $(CUR_DIR)/glm

########################################################### OpenCV Linker option
LIB_CV = -L$(LIB_CV_DIR) -lopencv_calib3d -lopencv_core -lopencv_features2d -lopencv_flann -lopencv_highgui -lopencv_imgproc -lopencv_ml -lopencv_objdetect -lopencv_photo -lopencv_stitching -lopencv_superres -lopencv_video -lopencv_videostab  -lopencv_imgcodecs -lopencv_videoio -lopencv_video

############################################################ GLFW_WINDOW Linker option

LIB_GLFW_WINDOW = -Wl,--unresolved-symbols=ignore-in-shared-libs -L$(LIB_GLFW_DIR) -dy -lGL -lGLU -lglut -dn -lglfw3 -lGLEW -dy -lXxf86vm  -lX11 -ldl -lrt -lXi -lXrandr -lXinerama -lXcursor
