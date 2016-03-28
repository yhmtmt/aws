all: $(OBJS)

-include $(DEPS)

%.o: %.cpp
	$(CC) $(FLAGS) $(RFLAGS) -c -MMD -MP $< -o $@
