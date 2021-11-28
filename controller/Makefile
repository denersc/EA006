#--Paths--
SDIR = .

#--Compiler config
CC = gcc
CCFLAGS = -MMD -Wall -Werror=format-security -Werror=implicit-function-declaration `pkg-config --cflags glib-2.0 libcjson liblo`
LIBS = -lpthread -lserialport -llo -lcjson `pkg-config --libs glib-2.0 libcjson liblo` -lm
PROG = controller

#--Vars
SRCS=$(wildcard $(SDIR)/*.c)
OBJS=$(SRCS:.c=.o)

TSRCS=$(wildcard $(TDIR)/*.c)
TOBJS=$(TSRCS:.c=.o)
TESTS=$(TSRCS:.c=)

#--Set Flags for release
all: CCFLAGS  += -O3
all: $(PROG)

$(PROG): $(OBJS)
	$(CC) -o $@ $^ $(CCFLAGS) $(LIBS)

.c.o:
	$(CC) -c $< -o $@  $(CCFLAGS)

.PHONY: clean

clean:
	rm -f $(SDIR)/*.o && rm -f $(TDIR)/*.o && rm -f $(PROG)
