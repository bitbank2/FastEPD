CFLAGS=-g -D__LINUX__ -Wall -O2 -I../../
LIBS=-g -lFastEPD -lgpiod 

all: hat_demo

hat_demo: main.o
	$(CXX) main.o $(LIBS) -o hat_demo

main.o: main.cpp
	$(CXX) $(CFLAGS) -c main.cpp

clean:
	rm -rf *.o hat_demo
