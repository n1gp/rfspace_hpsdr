CC = gcc

CFLAGS = -Wall #-g

LIBS = -lrt -lm -lpthread -lftdi1 -lliquid

INCLUDES = -I/usr/local/include

TARGET = rfspace_hpsdr

SRCS = rfspace_hpsdr.c #protocol2.c

OBJS = $(SRCS:.c=.o)

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(OBJS) $(INCLUDES) $(LIBS) -o $(TARGET)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(OBJS) $(TARGET)

.PHONY: all clean
