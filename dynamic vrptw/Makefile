CC		= g++
CFLAGS	= -g -O3 -std=c++0x
LIBPATH = ../lib
SRC		= testDynamicVRPTW.cpp $(LIBPATH)/VRPTW.cpp $(LIBPATH)/VRP.cpp $(LIBPATH)/readInstanceFilesVRPTW.cpp $(LIBPATH)/Scenario.cpp
INC		= $(LIBPATH)/tools.h $(LIBPATH)/LSBase.h $(LIBPATH)/LSProgram.h $(LIBPATH)/NM.h $(LIBPATH)/VRPlib.h $(LIBPATH)/Scenario.h
PROGS	= testVRPTW_Static_Only


OBJ		= $(SRC:.cpp=.o)

all: $(PROGS) $(SRC) $(INC)

$(PROGS): $(OBJ)
	$(CC) $(CFLAGS) $(OBJ) -o $@

.cpp.o: 
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm $(PROGS) *.o $(LIBPATH)/*.o


.PHONY: clean
