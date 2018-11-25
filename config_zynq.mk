CC	= arm-xilinx-linux-gnueabi-g++
CPU 	= arm

# n: disable all the window filter later
WINDOW = n	
FWINDOW = n
GLFW_WINDOW = n

# n: disable all the imaging and image processing filter later
AWS1_AP = n
IMGPROC = n
AVT_CAM = n
VMB_CAM = n
UVC_CAM = n
GST_CAM = n
SANYO_HD5400 = n	
ORB_SLAM = n
STEREO = n
CAMCALIB = n
STABILIZER = n
MISC = n

############################################################ Path configuration
CUR_DIR = $(shell pwd)
FDIR = $(CUR_DIR)/filter
CDIR = $(CUR_DIR)/channel
UDIR = $(CUR_DIR)/util
PROTO_DIR = $(CUR_DIR)/proto
LIB_PROTO =
RCMD_DIR = $(CUR_DIR)/rcmd
INC_CV_DIR = $(CUR_DIR)/../opencv/include
LIB_CV_DIR = $(CUR_DIR)/../opencv/lib
#INC_CV_DIR = /usr/local/include
#LIB_CV_DIR = /usr/local/lib
INC_PVAPI_DIR = $(CUR_DIR)/PvAPI/include
LIB_PVAPI_DIR = $(CUR_DIR)/PvAPI/lib
INC_VMB_DIR = /mnt/ssd1/Vimba_2_1
LIB_VMB_DIR = /mnt/ssd1/Vimba_2_1/VimbaCPP/DynamicLib/arm_64bit
#INC_GLFW_DIR = /usr/local/include/GLFW
#LIB_GLFW_DIR = /usr/lib/aarch64-linux-gnu
INC_GLFW_DIR = $(CUR_DIR)/GLFW/include
LIB_GLFW_DIR = $(CUR_DIR)/GLFW/lib
INC_EIGEN_DIR = /usr/include/eigen3
INC_MAVLINK = $(CUR_DIR)/mavlink/include_1.0/ardupilotmega
INC_GST = /usr/include/gstreamer-1.0
LIB_GST = /usr/lib/arm-linux-gnueabihf/gstreamer-1.0
INC_GLIB = /usr/include/glib-2.0
INC_GLIB_CONFIG = /usr/lib/arm-linux-gnueabihf/glib-2.0/include
INC_GLM = /usr/local/include/glm

########################################################### OpenCV Linker option
LIB_CV = -L$(LIB_CV_DIR) -lopencv_world
