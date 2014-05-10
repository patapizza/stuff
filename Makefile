CC=g++
CFLAGS=-std=c++11 -g -O0
SRC=testTSP.cpp
OBJ=testTSP.o

all: NQueens

testTSP: $(OBJ)
	$(CC) $(CFLAGS) -o NQueens $(OBJ)

testTSP.o: NQueens.cpp LSBase.h
	$(CC) $(CFLAGS) -c NQueens.cpp

clean:
	rm NQueens *.o
