CC=gcc
CPP=g++
PROG=../bin/localize_rect
COMMON_SRCS=
COMMON_DIR=../mikes-common
SRCS=$(COMMON_DIR)/modules/passive/mikes_logs.c \
     $(COMMON_DIR)/modules/live/ncurses_control.c \
     $(COMMON_DIR)/modules/live/base_module.c \
     $(COMMON_DIR)/modules/passive/pose.c \
     $(COMMON_DIR)/config/config.c \
     $(COMMON_DIR)/bites/util.c \
     $(COMMON_DIR)/bites/mikes.c \
     $(COMMON_DIR)/bites/pngwriter.c \
     $(COMMON_DIR)/modules/live/gui.c \
     $(COMMON_DIR)/modules/live/tim571.c \
     $(COMMON_DIR)/modules/passive/x_base.c \
     $(COMMON_DIR)/modules/passive/x_tim571.c \
     $(COMMON_DIR)/modules/live/tim_hough_transform.c \
     $(COMMON_DIR)/modules/live/line_filter.c \
     $(COMMON_DIR)/modules/live/tim_segment.c \
     $(COMMON_DIR)/modules/live/tim_corner.c \
     $(COMMON_DIR)/bites/hough.c \
     $(COMMON_DIR)/bites/filter.c \
     $(COMMON_DIR)/bites/math_2d.c \
     $(COMMON_DIR)/bites/corner.c \
     $(COMMON_DIR)/bites/segment.c \
     $(COMMON_DIR)/modules/passive/x_line_map.c \
     localize_rect.c \
     core/config_mikes.c \
     modules/live/rect_localization.c \
     modules/passive/rect_map_localization.c \
     ui.c

OPTIMIZE=-O0
DEBUG_FLAGS=-g
CPPSRCS=
OBJS=${SRCS:.c=.o}
CPPOBJS=${CPPSRCS:.cpp=.o}
CFLAGS=${OPTIMIZE} ${DEBUG_FLAGS} -std=c11 -D_BSD_SOURCE -D_XOPEN_SOURCE=600 -I. -I/usr/include/cairo -I/usr/local/rplidar/sdk/sdk/include -I/usr/include/librsvg-2.0/librsvg `pkg-config --cflags gio-2.0` -I/usr/include/libxml2 -I/usr/include/gdk-pixbuf-2.0 -Wall
CPPFLAGS=${OPTIMIZE} ${DEBUG_FLAGS} -D_BSD_SOURCE -D_XOPEN_SOURCE=600 -I. -I/usr/include/cairo -I/usr/local/rplidar/sdk/sdk/include -Wall -Wno-write-strings -I/usr/include/librsvg-2.0/librsvg `pkg-config --cflags gio-2.0` -I/usr/include/gdk-pixbuf-2.0
LDFLAGS=${DEBUG_FLAGS} -pthread -lrt -lcairo -lX11 -lm -lncurses -L/usr/local/rplidar/sdk/output/Linux/Release -lrplidar_sdk -lrsvg-2 -lxml2 -lpng -lstdc++ `pkg-config --libs gio-2.0` -lOpenNI

PREFIX=/usr/local
MIKES_CORE=$(shell pwd)
export MIKES_CORE

all: ${OBJS} ${CPPOBJS}
	${CC} ${OBJS} ${CPPOBJS} -o $(PROG) ${CFLAGS} ${LDFLAGS}

install:

test:

uninstall:

clean:
	echo "cleaning"
	rm -f *.o */*.o ${PROG}
	$(MAKE) -C ../mikes-common clean
