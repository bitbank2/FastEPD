CFLAGS=-D__LINUX__ -Wall -O2 -I/usr/include/drm
LIBS=-lFastEPD -lAnimatedGIF -lgpiod -ldrm -L/usr/lib/aarch64-linux-gnu/

all: eink_monitor

eink_monitor: main.o
	$(CXX) main.o $(LIBS) -o eink_monitor

main.o: main.cpp
	$(CXX) $(CFLAGS) -c main.cpp

clean:
	rm -rf *.o eink_monitor
