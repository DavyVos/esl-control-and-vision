CC = gcc
CFLAGS = -Wall -Wextra -std=c11 -Icommon
LDFLAGS = -lm

TARGET = main

SRCS = main.c \
       jiwy_controller.c \
       panmodel.c \
       pansubmod.c \
       tiltmodel.c \
       tiltsubmod.c \
       common/EulerAngles.c \
       common/motionprofiles.c \
       common/xxfuncs.c \
       common/xxinverse.c \
       common/xxmatrix.c

OBJS = $(SRCS:.c=.o)

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(OBJS) -o $(TARGET) $(LDFLAGS)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(OBJS) $(TARGET)

.PHONY: all clean