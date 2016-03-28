
all: $(OBJS)

-include $(DEPS)

%.o: %.cpp
	$(CC) $(FLAGS) -c -MMD -MP $< -o $@
