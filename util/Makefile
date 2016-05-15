all: $(OBJS)

%.o: %.cpp
	$(CC) $(FLAGS) $(RFLAGS) -c -MMD -MP $< -o $@

t2str: t2str.o c_clock.o
	$(CC) $(FLAGS) $(RFLAGS) -c t2str.o c_clock.o -o t2str

clean: 
	rm -f t2str
	rm -f *.d *.o

install:
	cp t2str $(INST_DIR)/
-include *.d
