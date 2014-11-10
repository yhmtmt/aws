ZYNQ = n

CC = g++

ifeq ($(ZYNQ), y)
	include	$(PETALINUX)/software/petalinux-dist/tools/user-commons.mk
	CC	= $(CXX)
endif

all: aws_cmd.o filter.o channel.o fcmd.o fset.o fget.o cyc.o online.o pause.o syn.o go.o quit.o stop.o step.o trat.o clear.o 
	$(CC) aws_cmd.o filter.o -o filter
	$(CC) aws_cmd.o channel.o -o channel
	$(CC) aws_cmd.o fcmd.o -o fcmd
	$(CC) aws_cmd.o fset.o -o fset
	$(CC) aws_cmd.o fget.o -o fget
	$(CC) aws_cmd.o cyc.o -o cyc
	$(CC) aws_cmd.o online.o -o online
	$(CC) aws_cmd.o pause.o -o pause
	$(CC) aws_cmd.o syn.o -o syn
	$(CC) aws_cmd.o go.o -o go
	$(CC) aws_cmd.o quit.o -o quit
	$(CC) aws_cmd.o stop.o -o stop
	$(CC) aws_cmd.o step.o -o step
	$(CC) aws_cmd.o trat.o -o trat
	$(CC) aws_cmd.o clear.o -o clear
%.o: %.cpp
	$(CC) -c $< -o $@

clean:
	rm *.o
	rm filter
	rm channel
	rm fcmd
	rm fset
	rm fget
	rm cyc
	rm online
	rm pause
	rm syn
	rm go 
	rm quit
	rm stop
	rm step
	rm trat

install:
	cp filter ../bin/
	cp channel ../bin/
	cp fset ../bin/
	cp fget ../bin/
	cp fcmd ../bin/
	cp cyc ../bin/
	cp online ../bin/
	cp pause ../bin/
	cp syn ../bin/
	cp go ../bin/
	cp quit ../bin/
	cp stop ../bin/
	cp step ../bin/
	cp trat ../bin/
