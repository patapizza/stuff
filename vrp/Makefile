CC		= g++
CFLAGS	= -g -O0 
LIBPATH = ../lib
SRC		= testVRP.cpp $(LIBPATH)/VRPlib.cpp
INC	= $(LIBPATH)/LSBase.h $(LIBPATH)/VRPlib.h
PROGS	= testVRP


OBJ		= $(SRC:.cpp=.o)

all: $(PROGS) $(SRC) $(INC)

$(PROGS): $(OBJ)
	$(CC) $(CFLAGS) $(OBJ) -o $@

.cpp.o: 
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm $(PROGS) *.o 


.PHONY: clean
