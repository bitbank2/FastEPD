CFLAGS=-D__LINUX__ -Wall -O2 -I../../
LIBS=-lFastEPD -lJPEGDEC -lgpiod 

all: show_jpeg

show_jpeg: main.o
	$(CXX) main.o $(LIBS) -o show_jpeg 

main.o: main.cpp
	$(CXX) $(CFLAGS) -c main.cpp

clean:
	rm -rf *.o show_jpeg 
