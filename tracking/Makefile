
INC_PATH = -I /usr/local/include/ -I /usr/local/include/gm
LIB_FLAGS = -L /usr/local/lib/ -l imu -l gm -l usb-1.0 -l hidapi-libusb -L /usr/lib/ -l LpSensor -l dl -l pthread

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
