all: $(OBJS)

%.o: %.cpp
	$(CC) $(FLAGS) $(RFLAGS) -c -MMD -MP $< -o $@

-include *.d
