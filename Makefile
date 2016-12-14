#
# libmcp23017.so Makefile (Linux)
#
CC=gcc
LINK=g++
OPTIONS=-g -fPIC -O3

LIBS=-lwiringPi

COMPILE=$(CC) $(INCLUDES)

PROGRAM=libmcp23017.so

SOURCES=mcp23017.c

HEADERS=mcp23017.h

OBJS=mcp23017.o

.PHONY: all demo clean

all: $(PROGRAM) $(HEADERS) $(SOURCES)

$(PROGRAM): $(OBJS)
	$(LINK) -shared -z noexecstack -o $(PROGRAM) $(OBJS) $(LIBS)

demo: $(PROGRAM)
	test -s /usr/local/lib/$(PROGRAM) || { echo "sudo make install first, Exiting..."; exit 1; }
	$(LINK) demo.c -lwiringPi -lmcp23017 -lpthread -o $@

.cpp.o:
	$(COMPILE) $(OPTIONS) -c -o $@ $<

install: $(PROGRAM)
	cp $(PROGRAM) /usr/local/lib
	cp $(HEADERS) /usr/local/include
	ldconfig

clean:
	-rm -f *.o $(PROGRAM) demo 

