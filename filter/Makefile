
all: $(OBJS)

%.o: %.cpp
	$(CC) $(FLAGS) -c -MMD -MP $< -o $@

-include *.d
