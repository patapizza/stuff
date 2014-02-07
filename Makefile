CC=g++
CFLAGS=-g -O0
SRC=testTSP.cpp
OBJ=testTSP.o

all: testTSP

testTSP: $(OBJ)
	$(CC) $(CFLAGS) -o testTSP $(OBJ)

testTSP.o: testTSP.cpp LSBase.h
	$(CC) $(CFLAGS) -c testTSP.cpp

clean:
	rm testTSP *.o
