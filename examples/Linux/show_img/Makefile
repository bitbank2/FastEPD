CFLAGS=-ggdb -D__LINUX__ -Wall -O2 -I../../
LIBS=-g -lFastEPD -lpngdec -lJPEGDEC -lgpiod 

all: show_img

show_img: main.o cJSON.o cJSON_Utils.o
	$(CXX) main.o cJSON.o cJSON_Utils.o $(LIBS) -o show_img ;\
	sudo cp show_img /usr/bin

cJSON.o: cJSON.h cJSON.c
	$(CC) $(CFLAGS) -c cJSON.c

cJSON_Utils.o: cJSON_Utils.h cJSON_Utils.c
	$(CC) $(CFLAGS) -c cJSON_Utils.c

main.o: main.cpp
	$(CXX) $(CFLAGS) -c main.cpp

clean:
	rm -rf *.o show_img
