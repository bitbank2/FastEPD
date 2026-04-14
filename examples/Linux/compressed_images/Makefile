CFLAGS=-D__LINUX__ -Wall -O2 -I../../
LIBS=-lFastEPD -lgpiod 

all: compressed_images

compressed_images: main.o
	$(CXX) main.o $(LIBS) -o compressed_images 

main.o: main.cpp
	$(CXX) $(CFLAGS) -c main.cpp

clean:
	rm -rf *.o compressed_images 
