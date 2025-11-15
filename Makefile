CC = gcc

CFLAGS = -Wall -O3 #-ffast-math -funroll-loops -ftree-vectorize -march=armv8-a+fp+simd #-g

LIBS = -lrt -lm -lpthread -lftdi1 -lliquid

INCLUDES =

TARGET = rfspace_hpsdr

SRCS = rfspace_hpsdr.c protocol2.c

OBJS = $(SRCS:.c=.o)

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(OBJS) $(INCLUDES) $(LIBS) -o $(TARGET)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(OBJS) $(TARGET)

.PHONY: all clean
