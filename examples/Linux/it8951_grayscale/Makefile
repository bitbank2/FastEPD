CFLAGS=-g -D__LINUX__ -Wall -O2 -I../../
LIBS=-g -lFastEPD -lgpiod 

all: grayscale_demo

grayscale_demo: main.o
	$(CXX) main.o $(LIBS) -o grayscale_demo

main.o: main.cpp
	$(CXX) $(CFLAGS) -c main.cpp

clean:
	rm -rf *.o grayscale_demo
