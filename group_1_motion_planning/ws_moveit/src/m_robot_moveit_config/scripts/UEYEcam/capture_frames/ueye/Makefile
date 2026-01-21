CC=g++
FLAGS=-g -pedantic -Wall -Wextra -Wno-error
COMPILE=$(CC) $(FLAGS)
UEYE_INCLUDE=/opt/ids/ueye/include
OBJECTS_REC=obj/main_rec.o obj/ueye_camera.o obj/console.o
OBJECTS_WB=obj/main_wb.o obj/ueye_camera.o obj/console.o

rec:$(OBJECTS_REC)
	$(COMPILE) $(OBJECTS_REC) -I$(UEYE_INCLUDE) -L/usr/lib -lueye_api -lpthread -o rec

obj/main_rec.o:main_rec.cpp ueye_camera.h console.cpp
	$(COMPILE) -c main_rec.cpp -I$(UEYE_INCLUDE) -L/usr/lib -lueye_api -lpthread -o obj/main_rec.o

obj/ueye_camera.o:ueye_camera.h ueye_camera.cpp
	$(COMPILE) -c ueye_camera.cpp -I$(UEYE_INCLUDE) -L/usr/lib -lueye_api -o obj/ueye_camera.o

obj/console.o:console.h console.cpp
	$(COMPILE) -c console.cpp -o obj/console.o

clean:
	-rm -r $(OBJECTS_REC) $(OBJECTS_WB) rec wb images/*
