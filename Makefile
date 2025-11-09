CC = gcc

CFLAGS = -Wall #-g

LIBS = -lrt -lm -lpthread -lftdi1 -lliquid

INCLUDES = -I/usr/local/include

TARGET = rfspace_usb2hpsdr

SRCS = rfspace_usb2hpsdr.c

OBJS = $(SRCS:.c=.o)

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(OBJS) $(INCLUDES) $(LIBS) -o $(TARGET)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(OBJS) $(TARGET)

.PHONY: all clean
