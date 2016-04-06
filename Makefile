#compiler
CC	= g++

#linker
LD	= ld

#debug option (y: build as debug binary)
#DEBUG = y

#install Directory
INST_DIR = ./bin
#optimization option
OFLAGS = -O3

# Platform specification (y: zynq environment is enabled)
ZYNQ	= n

# cpu architecture (currently arm, x64, x86, WIN64)
CPU	= arm
#CPU	= x64
#CPU	= x86

#operating system (currently only for LINUX)
OS	= LINUX

# preprocessor constant definition
DEFS = -D_$(CPU) -D_$(OS) 

# module selection switch
SANYO_HD5400 = n
AVT_CAM = y
UVC_CAM = y
FWINDOW = n
GLFW_WINDOW = y

#directory settings 
CUR_DIR = $(shell pwd)
FDIR = $(CUR_DIR)/filter
CDIR = $(CUR_DIR)/channel
UDIR = $(CUR_DIR)/util
RCMD_DIR = $(CUR_DIR)/rcmd

INC_CV_DIR = $(CUR_DIR)/opencv/include
LIB_CV_DIR = $(CUR_DIR)/opencv/lib
INC_PVAPI_DIR = $(CUR_DIR)/PvAPI/include
LIB_PVAPI_DIR = $(CUR_DIR)/PvAPI/lib
INC_GLFW_DIR = $(CUR_DIR)/GLFW/include
LIB_GLFW_DIR = $(CUR_DIR)/GLFW/lib

# module listing 
# listing filter module
FILTER = f_base f_nmea f_cam f_camcalib f_imgshk f_misc \
	f_shioji f_ship_detector f_stabilizer f_com f_uvc_cam f_event f_fep01 f_time \
	f_aws1_nmea_sw f_aws1_ctrl f_ahrs f_aws1_ap f_map

# listing channel module
CHANNEL = ch_base ch_image ch_aws1_ctrl

# listing utility module
UTIL =  c_clock c_imgalign aws_nmea aws_nmea_gps aws_nmea_ais c_ship aws_coord aws_serial aws_sock aws_vobj aws_vlib aws_stdlib

# for x86 CPU architecture
ifeq ($(CPU), x86)
	CC := $(CC) -m32
endif

# for debug mode
ifeq ($(DEBUG), y)
	DFLAGS = -g 	
endif

# for ZYNQ system
ifeq ($(ZYNQ), y)
	CC	= arm-xilinx-linux-gnueabi-g++
	CPU 	= arm
	SANYO_HD5400 = n
	FWINDOW = n
	GLFW_WINDOW = n
	#OFLAGS += -mfloat-abi=hard
endif

ifeq ($(FWINDOW), y)
	DEFS += -DFWINDOW
	FILTER += f_window
endif

ifeq ($(GLFW_WINDOW), y)
	INC += -I$(INC_GLFW_DIR)

ifeq ($(CPU), arm)
	LIB += -Wl,--unresolved-symbols=ignore-in-shared-libs -L$(LIB_GLFW_DIR) -dy -lGL -lGLU -lglut -dn -lglfw3 -lGLEW -dy -lXxf86vm  -lX11 -lrt -lXi -lXrandr 
else 
	LIB += -lGLEW -lglfw3 -lGL -ldl  -lX11 -lXi -lXrandr -lXxf86vm -lXinerama -lXcursor -lrt -lm -pthread -lglut 

endif 
	DEFS += -DGLFW_WINDOW 
	FILTER += f_glfw_window
	FILTER += f_aws1_ui
endif

ifeq ($(SANYO_HD5400),y)
	INC += -I$(CUR_DIR)/curl/include  -I$(CUR_DIR)/3rdparty/include 
	LIB += -lcurl -ljpeg
	FILTER += f_netcam
	UTIL +=  my_jsrc_mgr
	DEFS += -DSANYO_HD5400
endif

ifeq ($(AVT_CAM),y)
	INC += -I$(INC_PVAPI_DIR) 
	LIB += -L$(LIB_PVAPI_DIR) -lPvAPI
	FILTER += f_avt_cam f_avt_mono f_avt_stereo
	DEFS += -DAVT_CAM
endif

ifeq ($(UVC_CAM),y)
	DEFS += -DUVC_CAM
endif

INC += -I$(INC_CV_DIR) 
LIB += -L$(LIB_CV_DIR) -lrt -lpthread -lopencv_core -lopencv_nonfree -lopencv_contrib -lopencv_features2d -lopencv_imgproc -lopencv_imgproc -lopencv_calib3d -lopencv_ml  -lopencv_flann -lopencv_video -lopencv_legacy -lopencv_objdetect -lopencv_highgui -lopencv_photo -lopencv_gpu

FOBJS = $(addsuffix .o,$(FILTER))
COBJS = $(addsuffix .o,$(CHANNEL))
UOBJS = $(addsuffix .o,$(UTIL))
OBJS = command.o c_aws.o aws.o factory.o

FSRCS = $(addsuffix .cpp,$(FILTER))
CSRCS = $(addsuffix .cpp,$(CHANNEL))
USRCS = $(addsuffix .cpp,$(UTIL))
SRCS = command.cpp c_aws.cpp aws.cpp factory.cpp
FDEPS = $(addsuffix .d,$(FILTER))
CDEPS = $(addsuffix .d,$(CHANNEL))
UDEPS = $(addsuffix .d,$(UTIL))
DEPS = command.d c_aws.d aws.d factroy.d

EXE = aws
FLAGS = -std=gnu++0x $(DEFS) $(INC) $(OFLAGS) $(DFLAGS)

.PHONY: rcmd
all:
	make rcmd
	touch c_aws.cpp
	make aws
rcmd: 
	cd $(RCMD_DIR); make CC="$(CC)"; 

aws: $(OBJS) filter channel util 
	$(CC) $(FLAGS) $(OBJS) $(addprefix $(FDIR)/,$(FOBJS)) $(addprefix $(CDIR)/,$(COBJS)) $(addprefix $(UDIR)/,$(UOBJS)) -o $(EXE) $(LIB)

.PHONY: filter
.PHONY: channel
.PHONY: util

filter: 
	cd $(FDIR); make CC="$(CC)" FLAGS="$(FLAGS)" OBJS="$(FOBJS)" DEPS="$(FDEPS)"

channel:
	cd $(CDIR); make CC="$(CC)" FLAGS="$(FLAGS)" OBJS="$(COBJS)" DEPS="$(CDEPS)"

util:
	cd $(UDIR); make CC="$(CC)" FLAGS="$(FLAGS)" OBJS="$(UOBJS)" DEPS="$(UDEPS)"

-include *.d

.cpp.o:
	$(CC) $(FLAGS) -c -MMD -MP $< -o $@

.PHONY: clean
clean:
	rm -f *.o $(FDIR)/*.o $(CDIR)/*.o $(UDIR)/*.o
	rm -f *.d $(FDIR)/*.d $(CDIR)/*.d $(UDIR)/*.d	
	rm -f aws
	cd $(RCMD_DIR); make clean

.PHONY: clean
install:
	cp aws $(INST_DIR)/
	cd $(RCMD_DIR); make install INST_DIR="$(INST_DIR)"
