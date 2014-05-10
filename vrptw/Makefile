CC		= g++
CFLAGS	= -g -O0 -std=c++0x
LIBPATH = ../lib
SRC		= testVRPTW.cpp $(LIBPATH)/VRPTW.cpp $(LIBPATH)/VRP.cpp
INC	= $(LIBPATH)/LSBase.h $(LIBPATH)/LSProgram.h $(LIBPATH)/NM.h $(LIBPATH)/VRPlib.h 
PROGS	= testVRPTW


OBJ		= $(SRC:.cpp=.o)

all: $(PROGS) $(SRC) $(INC)

$(PROGS): $(OBJ)
	$(CC) $(CFLAGS) $(OBJ) -o $@

.cpp.o: 
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm $(PROGS) *.o $(LIBPATH)/*.o


.PHONY: clean
