ZYNQ = n

CC = g++

INST_DIR = ../bin


ifeq ($(ZYNQ), y)
	include	$(PETALINUX)/software/petalinux-dist/tools/user-commons.mk
	CC	= $(CXX)
endif

OBJ = aws_cmd.o filter.o channel.o fcmd.o fset.o fget.o cyc.o online.o pause.o go.o quit.o stop.o step.o trat.o clear.o finf.o chinf.o fpar.o
TGT = filter channel fcmd fset fget cyc online pause go quit stop step trat clear finf chinf fpar
OBJv2 = c_aws_cmd.o fls.o chls.o fpls.o
TGTv2 = fls chls fpls

all: $(TGT) $(TGTv2)
$(TGT): $(OBJ)
	$(CC) aws_cmd.o $@.o -o $@
$(TGTv2): $(OBJv2)
	$(CC) c_aws_cmd.o $@.o -o $@
%.o: %.cpp
	$(CC) -c $< -o $@

clean:
	rm $(OBJ) $(TGT) $(OBJv2) $(TGTv2)
install:
	for cmd in $(TGT); do \
	 cp $$cmd $(INST_DIR)/$$cmd; \
	done
	for cmd in $(TGTv2); do\
	 cp $$cmd $(INST_DIR)/$$cmd; \
	done
	
