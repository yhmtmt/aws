ZYNQ = n

CC = g++

INST_DIR = ../bin


ifeq ($(ZYNQ), y)
	include	$(PETALINUX)/software/petalinux-dist/tools/user-commons.mk
	CC	= $(CXX)
endif

OBJ = aws_cmd.o filter.o channel.o fcmd.o fset.o fget.o cyc.o online.o pause.o go.o quit.o stop.o step.o trat.o clear.o  
TGT = filter channel fcmd fset fget cyc online pause go quit stop step trat clear

all: $(TGT)
$(TGT): $(OBJ)
	$(CC) aws_cmd.o $@.o -o $@
%.o: %.cpp
	$(CC) -c $< -o $@

clean:
	rm $(OBJ) $(TGT)

install:
	for cmd in $(TGT); do \
	 cp $$cmd $(INST_DIR)/$$cmd; \
	done
