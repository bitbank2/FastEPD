CFLAGS=-D__LINUX__ -Wall -O2 -I../../
LIBS=-lFastEPD -lAnimatedGIF -lgpiod 

all: video_demo

video_demo: main.o
	$(CXX) main.o $(LIBS) -o video_demo 

main.o: main.cpp
	$(CXX) $(CFLAGS) -c main.cpp

clean:
	rm -rf *.o video_demo
