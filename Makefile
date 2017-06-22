# Target Name
EXE = aws

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

# Platform specification
#BOARD = jtx
#BOARD = jtk
#BOARD = zed
BOARD = pc

# cpu architecture (currently arm, x64, x86)
#CPU	= arm
CPU	= x64
#CPU	= x86

#operating system (currently only for LINUX)
OS	= LINUX

# preprocessor constant definition
DEFS = -D_$(CPU) -D_$(OS) 

# module switch setting
SANYO_HD5400 = n
AVT_CAM = n
VMB_CAM = y
UVC_CAM = y
GST_CAM = y
FWINDOW = n
GLFW_WINDOW = y
F_ORB_SLAM = y

############################################################ Path configuration
CUR_DIR = $(shell pwd)
FDIR = $(CUR_DIR)/filter
CDIR = $(CUR_DIR)/channel
UDIR = $(CUR_DIR)/util
RCMD_DIR = $(CUR_DIR)/rcmd
INC_CV_DIR = /usr/local/opencv/include
LIB_CV_DIR = /usr/local/opencv/lib
INC_PVAPI_DIR = $(CUR_DIR)/PvAPI/include
LIB_PVAPI_DIR = $(CUR_DIR)/PvAPI/lib
INC_VMB_DIR = $(CUR_DIR)/Vimba/include
LIB_VMB_DIR = $(CUR_DIR)/Vimba/lib
INC_GLFW_DIR = $(CUR_DIR)/GLFW/include
LIB_GLFW_DIR = $(CUR_DIR)/GLFW/lib
INC_EIGEN_DIR = /usr/include/eigen3
INC_MAVLINK = $(CUR_DIR)/mavlink/include_1.0/ardupilotmega
INC_GST = /usr/include/gstreamer-1.0
LIB_GST = /usr/lib/arm-linux-gnueabihf/gstreamer-1.0
INC_GLIB = /usr/include/glib-2.0
INC_GLIB_CONFIG = /usr/lib/arm-linux-gnueabihf/glib-2.0/include
#INC_GLM = $(CUR_DIR)/glm
INC_GLM = /usr/include/glm

# modules
MODS = filter channel util

INC = -I$(INC_CV_DIR) -I$(INC_EIGEN_DIR) -I$(INC_GLM) -I$(INC_MAVLINK)

# base linker option
LIB = -lrt -lpthread

# listing filters
FILTER = f_base f_nmea f_cam f_camcalib f_imgshk f_misc \
	f_shioji f_ship_detector f_stabilizer f_com f_uvc_cam f_event f_fep01 f_time \
	f_aws1_nmea_sw f_aws1_ctrl f_ahrs f_aws1_ap f_map f_obj_manager \
	f_wp_manager f_stereo f_aws3_com

# listing channels
CHANNEL = ch_base ch_image ch_aws1_ctrl ch_obj ch_aws3 ch_state

# listing utilities
UTIL =  c_clock c_imgalign aws_nmea aws_nmea_gps aws_nmea_ais c_ship aws_coord aws_serial aws_sock aws_vobj aws_vlib aws_stdlib aws_map

# for debug mode
ifeq ($(DEBUG), y)
	DFLAGS = -g 	
endif

########################################### Board specific build configuration
# for Zedboard
ifeq ($(BOARD), zed)
	CC	= arm-xilinx-linux-gnueabi-g++
	CPU 	= arm
	SANYO_HD5400 = n
	FWINDOW = n
	GLFW_WINDOW = n
	F_ORB_SLAM = n
	GST_CAM = n
	#OFLAGS += -mfloat-abi=hard
endif

ifeq ($(BOARD), jtk)
	F_ORB_SLAM = n
	CPU 	= arm
	INC_CV_DIR = /usr/local/include
	LIB_CV_DIR = /usr/local/lib
	INC_EIGEN_DIR = /usr/include/eigen3
endif

ifeq ($(BOARD), jtx)
	CPU 	= arm
	INC_CV_DIR = /usr/local/include
	LIB_CV_DIR = /usr/local/lib
	INC_EIGEN_DIR = /usr/local/include/eigen3
endif

ifeq ($(BOARD), pc)
	GST_CAM = n
endif

################################################# cpu specific compiler option
ifeq ($(CPU), x86)
	CC = $(CC) -m32
endif

################################################## f_glfw_window configuration
ifeq ($(GLFW_WINDOW), y)
	INC += -I$(INC_GLFW_DIR)
	UTIL += aws_glib
ifeq ($(CPU), arm)
ifeq ($(BOARD), jtx)
	LIB += -Wl,--unresolved-symbols=ignore-in-shared-libs -L$(LIB_GLFW_DIR) -dy -lGL -lGLU -lglut -dn -lglfw3 -lGLEW -dy -lXxf86vm -lX11 -ldl -lrt -lXi -lXrandr -lXinerama -lXcursor 
	FILTER += f_glfw_window  f_glfw_stereo_view 	
endif # jtx
ifeq ($(BOARD), jtk)
	LIB += -Wl,--unresolved-symbols=ignore-in-shared-libs -L$(LIB_GLFW_DIR) -dy -lGL -lGLU -lglut -dn -lglfw3 -lGLEW -dy -lXxf86vm  -lX11 -lrt -lXi -lXrandr
	FILTER += f_glfw_window  f_glfw_stereo_view 
endif # jtk
else
	LIB += -lGLEW -lglfw3 -lGL -ldl  -lX11 -lXi -lXrandr -lXxf86vm -lXinerama -lXcursor -lrt -lm -pthread -lGLU -lglut 
endif # cpu is not arm
	DEFS += -DGLFW_WINDOW 
	FILTER += f_glfw_window  f_glfw_stereo_view 
	FILTER += f_aws1_ui c_aws1_ui_normal c_aws1_ui_map c_aws1_ui_dev f_aws3_ui f_state_estimator
	OFLAGS += -fopenmp
endif

###################################################### f_orb_slam configuration
ifeq ($(F_ORB_SLAM), y)
	FILTER += f_orb_slam
	MODS += orb_slam g2o DBoW2
	ORB_SLAM_OBJS = $(addprefix $(ORB_SLAM_DIR)/, $(addsuffix .o,$(ORB_SLAM)))
	G2O_OBJS = $(addsuffix .o,$(G2O))
	DBOW2_OBJS = $(addsuffix .o,$(DBOW2))

	ORB_SLAM_DEPS = $(addsuffix .d,$(ORB_SLAM))
	G2O_DEPS = $(addsuffix .d,$(G2O))
	DBOW2_DEPS = $(addsuffix .d,$(DBOW2))
	DEFS += -DORB_SLAM

	ORB_SLAM_DIR = $(CUR_DIR)/orb_slam
	G2O_DIR = $(CUR_DIR)/g2o
	G2O_CORE_DIR = $(G2O_DIR)/core
	G2O_SOLVERS_DIR = $(G2O_DIR)/solvers
	G2O_STUFF_DIR = $(G2O_DIR)/stuff
	G2O_TYPES_DIR = $(G2O_DIR)/types

	ORB_SLAM = Converter Frame Initializer KeyFrame KeyFrameDatabase Map MapPoint Optimizer ORBextractor ORBmatcher PnPsolver Sim3Solver

	G2O_CORE = batch_stats optimization_algorithm_factory cache optimization_algorithm_gauss_newton estimate_propagator  optimization_algorithm_levenberg g2o_factory optimization_algorithm_with_hessian hyper_dijkstra parameter hyper_graph parameter_container hyper_graph_action robust_kernel jacobian_workspace robust_kernel_factory marginal_covariance_cholesky robust_kernel_impl matrix_structure solver optimizable_graph  optimization_algorithm sparse_optimizer optimization_algorithm_dogleg
	G2O_SOLVERS =
	G2O_STUFF = property string_tools timeutil
	G2O_TYPES = types_sba types_seven_dof_expmap types_six_dof_expmap
	G2O = $(addprefix $(G2O_CORE_DIR)/, $(G2O_CORE)) $(addprefix $(G2O_SOLVERS)/, $(G2O_SOLVERS)) $(addprefix $(G2O_STUFF_DIR)/, $(G2O_STUFF)) $(addprefix $(G2O_TYPES_DIR)/, $(G2O_TYPES))
	DBOW2 = DBoW2/BowVector DBoW2/FeatureVector DBoW2/FORB DBoW2/ScoringObject DUtils/Random DUtils/Timestamp

endif

###################################################### f_net_cam configuration
ifeq ($(SANYO_HD5400),y)
	INC += -I$(CUR_DIR)/curl/include  -I$(CUR_DIR)/3rdparty/include 
	LIB += -lcurl -ljpeg
	FILTER += f_netcam
	UTIL +=  my_jsrc_mgr
	DEFS += -DSANYO_HD5400
endif

###################################################### f_avt_cam configuration
ifeq ($(AVT_CAM),y)
	INC += -I$(INC_PVAPI_DIR) 
	LIB += -L$(LIB_PVAPI_DIR) -lPvAPI
	FILTER += f_avt_cam f_avt_mono f_avt_stereo
	DEFS += -DAVT_CAM
endif

################################################## f_avt_vmb_cam configuration
ifeq ($(VMB_CAM),y)
	INC += -I$(INC_VMB_DIR)
	LIB += -L$(LIB_VMB_DIR) -lVimbaC -lVimbaCPP
	FILTER += f_avt_vmb_cam
	DEFS += -DAVT_VMB_CAM
endif

###################################################### f_uvc_cam configuration
ifeq ($(UVC_CAM),y)
	DEFS += -DUVC_CAM
endif

ifeq ($(GST_CAM),y)
	INC += -I$(INC_GST) -I$(INC_GLIB) -I$(INC_GLIB_CONFIG)
	LIB += -L$(LIB_GST) -lgstreamer-1.0 -lgobject-2.0 -lglib-2.0 -lgstapp-1.0
	DEFS += -DGST_CAM
	FILTER += f_gst_cam
endif

######################################################## f_window configutaion
ifeq ($(FWINDOW), y)
	DEFS += -DFWINDOW
	FILTER += f_window
endif

################################################ Board specific linker options
ifeq ($(BOARD), jtx)
	LIB += -L$(LIB_CV_DIR) -lopencv_calib3d -lopencv_core -lopencv_features2d -lopencv_flann -lopencv_highgui -lopencv_imgproc -lopencv_ml -lopencv_objdetect -lopencv_photo -lopencv_stitching -lopencv_superres -lopencv_video -lopencv_videostab  -lopencv_imgcodecs -lopencv_videoio -lopencv_video
endif

ifeq ($(BOARD), jtk)
	LIB +=  -L$(LIB_CV_DIR) -lopencv_calib3d -lopencv_core -lopencv_features2d -lopencv_flann -lopencv_highgui -lopencv_imgproc -lopencv_ml -lopencv_objdetect -lopencv_photo -lopencv_stitching -lopencv_superres -lopencv_video -lopencv_videostab  -lopencv_imgcodecs -lopencv_videoio -lopencv_video
endif

ifeq ($(BOARD), zed)
	LIB += -L$(LIB_CV_DIR) -lopencv_world
endif

ifeq ($(BOARD), pc)
	LIB += -L$(LIB_CV_DIR) -lopencv_calib3d -lopencv_core -lopencv_features2d -lopencv_flann -lopencv_highgui -lopencv_imgproc -lopencv_ml -lopencv_objdetect -lopencv_photo -lopencv_stitching -lopencv_superres -lopencv_video -lopencv_videostab  -lopencv_imgcodecs -lopencv_videoio -lopencv_video
endif

################################################################# Object lists
FOBJS = $(addsuffix .o,$(FILTER))
COBJS = $(addsuffix .o,$(CHANNEL))
UOBJS = $(addsuffix .o,$(UTIL))
OBJS = command.o c_aws.o aws.o factory.o

################################################################# Source lists
FSRCS = $(addsuffix .cpp,$(FILTER))
CSRCS = $(addsuffix .cpp,$(CHANNEL))
USRCS = $(addsuffix .cpp,$(UTIL))
SRCS = command.cpp c_aws.cpp aws.cpp factory.cpp

######################################################## Dependency file lists
FDEPS = $(addsuffix .d,$(FILTER))
CDEPS = $(addsuffix .d,$(CHANNEL))
UDEPS = $(addsuffix .d,$(UTIL))
DEPS = command.d c_aws.d aws.d factroy.d

############################################################## Compiler option
FLAGS = -std=gnu++0x $(DEFS) $(INC) $(OFLAGS) $(DFLAGS) 

########################################################### Target description
.PHONY: rcmd
all:
	make rcmd
	touch c_aws.cpp
	make aws
	make log2txt
	make t2str

rcmd: 
	cd $(RCMD_DIR); make CC="$(CC)"; 

aws: $(OBJS) $(MODS)
	$(CC) $(FLAGS) $(OBJS) $(addprefix $(FDIR)/,$(FOBJS)) $(addprefix $(CDIR)/,$(COBJS)) $(addprefix $(UDIR)/,$(UOBJS)) $(ORB_SLAM_OBJS) $(G2O_OBJS) $(DBOW2_OBJS) -o $(EXE) $(LIB)

log2txt: util/log2txt.o factory.o command.o c_aws.o filter channel util orb_slam g2o DBoW2
	$(CC) $(FLAGS) $(addprefix $(FDIR)/,$(FOBJS)) $(addprefix $(CDIR)/,$(COBJS)) $(addprefix $(UDIR)/,$(UOBJS)) $(ORB_SLAM_OBJS) $(G2O_OBJS) $(DBOW2_OBJS)  command.o c_aws.o factory.o util/log2txt.o -o log2txt $(LIB)

t2str: util/t2str.o util/c_clock.o
	$(CC) util/t2str.o util/c_clock.o -o t2str

.PHONY: filter
.PHONY: channel
.PHONY: util
.PHONY: orb_slam
.PHONY: g2o
.PHONY: DBoW2
.PHONY: clean

filter: 
	cd $(FDIR); make CC="$(CC)" FLAGS="$(FLAGS)" OBJS="$(FOBJS)" DEPS="$(FDEPS)"

channel:
	cd $(CDIR); make CC="$(CC)" FLAGS="$(FLAGS)" OBJS="$(COBJS)" DEPS="$(CDEPS)"

util:
	cd $(UDIR); make CC="$(CC)" FLAGS="$(FLAGS)" OBJS="$(UOBJS)" DEPS="$(UDEPS)"

orb_slam: $(ORB_SLAM_OBJS)

g2o: $(G2O_OBJS)

DBoW2: $(DBOW2_OBJS)

-include *.d

.cpp.o:
	$(CC) $(FLAGS) -c -MMD -MP $< -o $@
.cc.o:
	$(CC) $(FLAGS) -c -MMD -MP $< -o $@

clean:
	rm -f *.o $(FDIR)/*.o $(CDIR)/*.o $(UDIR)/*.o
	rm -f *.d $(FDIR)/*.d $(CDIR)/*.d $(UDIR)/*.d	
	cd $(UDIR); make clean
	cd $(RCMD_DIR); make clean
	rm -f aws
	rm -f t2str
	rm -f log2txt

install:
	cp aws $(INST_DIR)/
	cp t2str $(INST_DIR)/
	cp log2txt $(INST_DIR)/
	cd $(RCMD_DIR); make install INST_DIR="$(INST_DIR)"
	cp logawstime $(INST_DIR)/
	cp logtime $(INST_DIR)/
