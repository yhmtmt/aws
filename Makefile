ZYNQ	= n

#compiler
CC	= g++

#linker
LD	= ld

#debug option
DFLAG = -g 

#optimization option
OFLAG = -O3

# cpu architecture (currently arm, x64, x86, WIN64)
#CPU	= arm
#CPU	= x64
CPU	= x86

#operating system (currently only for LINUX)
OS	= LINUX

# preprocessor constant definition
DEFS = -D_$(CPU) -D_$(OS) 

# module selection switch
SANYO_HD5400 = y
AVT_CAM = y
FWINDOW = n

#directory settings 
CUR_DIR = $(shell pwd)
FDIR = $(CUR_DIR)/filter
CDIR = $(CUR_DIR)/channel
UDIR = $(CUR_DIR)/util

# module listing 
# listing filter module
FILTER = f_base f_nmea f_cam f_camcalib f_imgshk f_misc \
	f_shioji f_ship_detector f_stabilizer f_com f_uvc_cam
# listing channel module
CHANNEL = ch_base

# listing utility module
UTIL =  c_clock c_imgalign c_nmeadec c_ship coord util 

# listing include path 
INC = -I/usr/local/include  -I$(CUR_DIR)/opencv/include 

LIB = -L$(CUR_DIR)/opencv/$(CPU)/lib -lpthread -lopencv_core -lopencv_contrib -lopencv_features2d -lopencv_imgproc -lopencv_imgproc -lopencv_calib3d -lopencv_ml  -lopencv_flann -lopencv_video -lopencv_legacy -lopencv_nonfree -lopencv_objdetect -lopencv_highgui -lopencv_photo -lopencv_gpu

LIB += -L$(CUR_DIR)/cminpack/$(OS)/$(CPU) -lcminpack

ifeq ($(ZYNQ), y)
	include	$(PETALINUX)/software/petalinux-dist/tools/user-commons.mk
	CC	= $(CXX)
	CPU 	= arm
	SANYO_HD5400 = n
	FWINDOW = n
endif

ifeq ($(FWINDOW), y)
	DEFS += -DFWINDOW
	FILTER += f_window
endif

ifeq ($(SANYO_HD5400),y)
	INC += -I$(CUR_DIR)/curl/include  -I$(CUR_DIR)/3rdparty/include 
	LIB += -lcurl -ljpeg
	FILTER += f_netcam
	UTIL +=  my_jsrc_mgr
	DEFS += -DSANYO_HD5400
endif

ifeq ($(AVT_CAM),y)
	INC += -I$(CUR_DIR)/PvAPI/include 
	LIB += -L$(CUR_DIR)/PvAPI/bin/$(CPU) -lPvAPI
	FILTER += f_avt_cam
	DEFS += -DAVT_CAM
endif

FOBJS = $(addsuffix .o,$(FILTER))
COBJS = $(addsuffix .o,$(CHANNEL))
UOBJS = $(addsuffix .o,$(UTIL))
OBJS = command.o c_aws.o aws.o 

EXE = aws
FLAGS = -std=gnu++0x $(DEFS) $(INC) $(OFLAGS) $(DFLAGS)

aws: $(OBJS)	
	cd $(FDIR); make CC="$(CC)" FLAGS="$(FLAGS)" OBJS="$(FOBJS)"
	cd $(CDIR); make CC="$(CC)" FLAGS="$(FLAGS)" OBJS="$(COBJS)"
	cd $(UDIR); make CC="$(CC)" FLAGS="$(FLAGS)" OBJS="$(UOBJS)"
	$(CC) $(FLAGS) $(OBJS) $(addprefix $(FDIR)/,$(FOBJS)) $(addprefix $(CDIR)/,$(COBJS)) $(addprefix $(UDIR)/,$(UOBJS)) -o $(EXE) $(LIB)

.cpp.o:
	$(CC) $(FLAGS) -c $< -o $@

clean:
	rm *.o $(FDIR)/*.o $(CDIR)/*.o $(UDIR)/*.o
	rm aws

install:
	cp aws bin/
