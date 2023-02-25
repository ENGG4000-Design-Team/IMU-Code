CC = g++

BASEFLAGS = -Wall -std=c++20
DEBUG_FLAGS = -DDEBUG -g
LIBS = -lwiringPi -lcmps14

EXE = imu

release: CFLAGS = $(BASEFLAGS)
release: $(EXE)

debug: CFLAGS = $(BASEFLAGS) $(DEBUG_FLAGS)
debug: $(EXE)

imu: imu.cpp sun_pos.cpp sun_pos.h
	$(CC) $(CFLAGS) -o $@ imu.cpp sun_pos.cpp $(LIBS)

clean:
	rm -f $(EXE)
