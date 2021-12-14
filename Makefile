# Make sure and install pigpio apt-get install pigpio

CC = gcc
CFLAGS = -I. \
		-Iinclude \
		-lpthread \
		-lpigpio

SRCS = $(wildcard src/*.c)
SRCS += main.c
OBJS = $(SRCS:.c=.o)

TARGET := little_bot_program

all: $(TARGET)
	echo $(SRCS)
	echo $(OBJS)

%.o: %.c
	$(CC) -c -o $@ $< $(CFLAGS)
	$(CC) -MM $(CFLAGS) $< > $*.d

$(TARGET): $(OBJS)
	$(CC) -o $@ $^ $(CFLAGS)

clean:
	rm *.o src/*.o src/*.d


-include $(OBJS:.o=.d)
