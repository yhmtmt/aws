# Target Name
EXE = aws

#linker
LD	= ld

#debug option (y: build as debug binary)
DEBUG = y

#install Directory
INST_DIR = ./bin

#optimization option
OFLAGS = -O3

# Platform specification
#ARCH = jtx
ARCH = jtk
#ARCH = zynq
#ARCH = x64
#ARCH = x86

-include config_${ARCH}.mk

#operating system (currently only for LINUX)
OS	= LINUX

# preprocessor constant definition
DEFS = -D_$(CPU) -D_$(OS) 

ifeq ($(WINDOW), n)
	FWINDOW = n
	GLFW_WINDOW = n
endif


# for debug mode
ifeq ($(DEBUG), y)
	DFLAGS = -g
	OFLAGS = -O0
endif


# modules
MODS = filter channel util

# base include paths
INC = -I$(INC_CV_DIR) -I$(INC_GLIB) -I$(INC_EIGEN_DIR) -I$(INC_GLM) -I$(INC_MAVLINK)

# base libs
LIB = -lrt -lpthread $(LIB_CV)

# base filters
FILTER = f_base f_nmea \
	f_shioji f_com f_event f_fep01 f_time \
	f_aws1_nmea_sw f_aws1_ctrl f_aws1_sim c_model f_ahrs f_aws1_ap f_map \
	f_obj_manager f_wp_manager f_aws3_com f_env_sensor f_test_vsrc \
	f_ngt1 ngt1/common ngt1/pgn f_router

# base channels
CHANNEL = ch_base ch_image ch_aws1_ctrl ch_obj ch_aws3 ch_state ch_wp

# base utilities
UTIL =  c_clock  aws_nmea aws_nmea_gps aws_nmea_ais c_ship aws_coord aws_serial aws_sock aws_stdlib aws_map

################################################# Image processing configuration
ifeq ($(IMGPROC),y)
	UTIL += aws_vlib aws_vobj c_imgalign 
	FILTER += f_cam
else	
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
	DEFS += -D_C_IMG_ALIGHN_H_
#	DEFS += -D_AWS_VLIB_H_
	DEFS += -D_AWS_VOBJ_H_
	DEFS += -D_F_CAM_H
endif

################################################## f_misc configuration
ifeq ($(MISC),y)
	FILTER += f_misc f_imgshk
	DEFS += -DMISC -DIMGSHK
else
	DEFS += -D_F_MISC_H_
	DEFS += -D_F_IMGSHK_H_
endif

################################################## f_stabilizer
ifeq ($(STABILIZER),y)
	FILTER += f_stabilizer
	DEFS += -DSTABILIZER
else
	DEFS += -D_F_STABILIZER_H_
endif

################################################## f_stabilizer
ifeq ($(SHIP_DETECTOR),y)
	FILTERS += f_ship_detector
	DEFS += -DSHIP_DETECTOR
else
	DEFS += -D_F_SHIP_DETECTOR_H_
endif


################################################## f_camcalib configuration
ifeq ($(CAMCALIB),y)
	FILTER += f_camcalib
	DEFS += -DCAMCALIB
else
	DEFS += -D_F_CAMCALIB_H_
endif

################################################## f_stereo configuration
ifeq ($(STEREO),y)
	FILTER += f_stereo
	DEFS += -DSTEREO
else
	DEFS += -D_F_STEREO_H_
endif

################################################## f_glfw_window configuration
ifeq ($(GLFW_WINDOW),y)
	INC += -I$(INC_GLFW_DIR)
	UTIL += aws_glib
	LIB += $(LIB_GLFW_WINDOW)
	DEFS += -DGLFW_WINDOW 
	FILTER += f_glfw_window  f_glfw_stereo_view f_aws1_ui f_test_aws1_ui f_aws1_ui_util/c_map_obj f_aws1_ui_util/c_ui_box f_aws3_ui f_state_estimator 
	OFLAGS += -fopenmp
endif

###################################################### f_orb_slam configuration
ifeq ($(ORB_SLAM), y)
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
	FILTER += f_uvc_cam
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

################################################################# Object lists
FOBJS = $(addsuffix .o,$(FILTER))
COBJS = $(addsuffix .o,$(CHANNEL))
UOBJS = $(addsuffix .o,$(UTIL))
OBJS = command.o c_aws.o c_aws_temp.o aws.o channel_factory.o filter_factory.o

################################################################# Source lists
FSRCS = $(addsuffix .cpp,$(FILTER))
CSRCS = $(addsuffix .cpp,$(CHANNEL))
USRCS = $(addsuffix .cpp,$(UTIL))
SRCS = command.cpp c_aws.cpp c_aws_temp.cpp aws.cpp channel_factory.cpp filter_factory.cpp

######################################################## Dependency file lists
FDEPS = $(addsuffix .d,$(FILTER))
CDEPS = $(addsuffix .d,$(CHANNEL))
UDEPS = $(addsuffix .d,$(UTIL))
DEPS = command.d c_aws.d c_aws_temp.d aws.d channel_factroy.d filter_factory.d

############################################################## Compiler option
FLAGS = -std=gnu++0x $(DEFS) $(INC) $(OFLAGS) $(DFLAGS) 

########################################################### Target description
.PHONY: rcmd
all:
	make rcmd
	touch c_aws_temp.cpp
	make aws
	make log2txt
	make t2str

rcmd: 
	cd $(RCMD_DIR); make CC="$(CC)"; 

aws: $(OBJS) $(MODS)
	$(CC) $(FLAGS) $(OBJS) $(addprefix $(FDIR)/,$(FOBJS)) $(addprefix $(CDIR)/,$(COBJS)) $(addprefix $(UDIR)/,$(UOBJS)) $(ORB_SLAM_OBJS) $(G2O_OBJS) $(DBOW2_OBJS) -o $(EXE) $(LIB)

log2txt: util/log2txt.o channel_factory.o filter_factory.o command.o c_aws.o c_aws_temp.o filter channel util orb_slam g2o DBoW2
	$(CC) $(FLAGS) $(addprefix $(FDIR)/,$(FOBJS)) $(addprefix $(CDIR)/,$(COBJS)) $(addprefix $(UDIR)/,$(UOBJS)) $(ORB_SLAM_OBJS) $(G2O_OBJS) $(DBOW2_OBJS)  command.o c_aws.o c_aws_temp.o filter_factory.o channel_factory.o util/log2txt.o -o log2txt $(LIB)

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
	find . -type f -name '*.o' -delete
	find . -type f -name '*.d' -delete
	rm -f aws
	rm -f t2str
	rm -f log2txt

install:
	cp aws $(INST_DIR)/
	cp t2str $(INST_DIR)/
	cp log2txt $(INST_DIR)/
	cd $(RCMD_DIR); make install INST_DIR="$(INST_DIR)"
	cp logtools/* $(INST_DIR)/
