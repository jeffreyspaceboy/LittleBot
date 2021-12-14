# Make sure and install pigpio apt-get install pigpio

CC = gcc
CFLAGS = -I. \
		-I../../include \
		-lpthread \
		-lpigpio \
		-lncurses

SRCS = $(wildcard ../../src/*.c)
SRCS += keyboard_control.c
OBJS = $(SRCS:.c=.o)

TARGET := keyboard_control

all: $(TARGET)
	echo $(SRCS)
	echo $(OBJS)

%.o: %.c
	$(CC) -c -o $@ $< $(CFLAGS)
	$(CC) -MM $(CFLAGS) $< > $*.d

$(TARGET): $(OBJS)
	$(CC) -o $@ $^ $(CFLAGS)

clean:
	rm ../../*.o *.o ../../*.d *.d ../../src/*.o ../../src/*.d


-include $(OBJS:.o=.d)
