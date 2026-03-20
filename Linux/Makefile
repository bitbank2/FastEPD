PROCESSOR:=$(shell uname -m)
ifeq ($(PROCESSOR), armv7l)
CFLAGS=-c -Wall -O2 -ggdb -D__HAS_NEON__ -D__LINUX__ -I../src
else
CFLAGS=-c -Wall -O2 -ggdb -D__LINUX__ -I../src
endif
LIBS = -lm -lpthread

all: libFastEPD.a

libFastEPD.a: FastEPD.o
	ar -rc libFastEPD.a FastEPD.o ;\
	sudo cp libFastEPD.a /usr/local/lib ;\
	sudo cp ../src/FastEPD.h /usr/local/include

FastEPD.o: ../src/FastEPD.cpp ../src/FastEPD.h ../src/FastEPD.inl ../src/bb_ep_gfx.inl ../src/linux_io.inl
	$(CXX) $(CFLAGS) ../src/FastEPD.cpp

clean:
	rm *.o libFastEPD.a
