
INC_PATH = -I /usr/local/include/
LIB_FLAGS = -L /usr/local/lib/ -l imu -L /usr/lib/ -l LpSensor -l dl -l pthread

CC = g++
CFLAGS = -Wall -std=c++11 $(INC_PATH)

SOURCES = main.cpp
OBJECTS = $(SOURCES:.cpp=.o)
NAME = tracking

all: $(NAME)

$(NAME): $(OBJECTS)
	$(CC) $(CFLAGS) $(OBJECTS) $(LIB_FLAGS) -o $(NAME)

.cpp.o:
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(NAME) *.o
