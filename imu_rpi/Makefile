CC = g++

BASEFLAGS = -Wall -std=c++20
DEBUG_FLAGS = -DDEBUG -g
LIBS = -lcmps14 -lwiringPi

OBJS = imu.o sun_pos.o

EXE = imu

release: CFLAGS = $(BASEFLAGS)
release: $(EXE)

debug: CFLAGS = $(BASEFLAGS) $(DEBUG_FLAGS)
debug: $(EXE)

$(EXE): $(OBJS)
	$(CC) $(CFLAGS) $(OBJS) -o $(EXE)

imu.o: imu.cpp sun_pos.h
	$(CC) $(CFLAGS) -c imu.cpp $(LIBS)

sun_pos.o: sun_pos.cpp sun_pos.h
	$(CC) $(CFLAGS) -c sun_pos.cpp

clean:
	rm -f $(OBJS)
	rm -f $(EXE)

run:
	./$(EXE)