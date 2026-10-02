CFLAGS=-Wall -O2
LIBS=

all: gfx_tests

gfx_tests: main.o
	$(CXX) main.o $(LIBS) -o gfx_tests 

main.o: main.cpp ../../../src/FastEPD.h
	$(CXX) $(CFLAGS) -c main.cpp

clean:
	rm -rf *.o gfx_tests
