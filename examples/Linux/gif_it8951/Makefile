CFLAGS=-g -D__LINUX__ -Wall -O2 -I../../
LIBS=-g -lFastEPD -lAnimatedGIF -lgpiod 

all: gif_player

gif_player: main.o
	$(CXX) main.o $(LIBS) -o gif_player 

main.o: main.cpp
	$(CXX) $(CFLAGS) -c main.cpp

clean:
	rm -rf *.o gif_player 
