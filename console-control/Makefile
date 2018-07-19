CC=gcc
CPP=g++
PROG=../bin/console-control
COMMON_SRCS=
COMMON_DIR=../mikes-common
SRCS=$(COMMON_DIR)/modules/passive/mikes_logs.c \
     $(COMMON_DIR)/modules/live/ncurses_control.c \
     $(COMMON_DIR)/modules/live/base_module.c \
     $(COMMON_DIR)/modules/passive/pose.c \
     $(COMMON_DIR)/config/config.c \
     $(COMMON_DIR)/bites/util.c \
     $(COMMON_DIR)/bites/mikes.c \
     console-control.c \
     core/config_mikes.c \
     ui.c

OPTIMIZE=-O0
DEBUG_FLAGS=-g
CPPSRCS=
OBJS=${SRCS:.c=.o}
CPPOBJS=${CPPSRCS:.cpp=.o}
CFLAGS=${OPTIMIZE} ${DEBUG_FLAGS} -std=c11 -D_BSD_SOURCE -D_XOPEN_SOURCE=600 -I. -I/usr/include/cairo -I/usr/local/rplidar/sdk/sdk/include -I/usr/include/librsvg-2.0/librsvg -I/usr/include/glib-2.0 -I/usr/lib/arm-linux-gnueabihf/glib-2.0/include -I/usr/include/libxml2 -I/usr/include/gdk-pixbuf-2.0 -Wall
CPPFLAGS=${OPTIMIZE} ${DEBUG_FLAGS} -D_BSD_SOURCE -D_XOPEN_SOURCE=600 -I. -I/usr/include/cairo -I/usr/local/rplidar/sdk/sdk/include -Wall -Wno-write-strings -I/usr/include/librsvg-2.0/librsvg -I/usr/include/glib-2.0 -I/usr/lib/arm-linux-gnueabihf/glib-2.0/include -I/usr/include/gdk-pixbuf-2.0 
LDFLAGS=${DEBUG_FLAGS} -lpthread -lrt -lcairo -lX11 -lm -lncurses -L/usr/local/rplidar/sdk/output/Linux/Release -lrplidar_sdk -lrsvg-2 -lxml2
PREFIX=/usr/local

all: ${OBJS} ${CPPOBJS}
	${CC} ${OBJS} ${CPPOBJS} -o $(PROG) ${CFLAGS} ${LDFLAGS}

install:

test:	

uninstall:

clean:
	echo "cleaning"
	rm -f *.o */*.o ${PROG}
	$(MAKE) -C ../mikes-common clean



