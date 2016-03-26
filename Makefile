#!/bin/bash
CPPFLAGS += -std=c++11 
MAKEFILE="Makefile"
COMPILER=arm-none-eabi-g++


# Build executable 
bin/wkquad: main.cpp $(DATA_STRUCTURE_OBJS)
	$(COMPILER) $(CPPFLAGS) main.cpp  $(DATA_STRUCTURE_OBJS) -o bin/wkquad

# Specify data structure source files
DATA_STRUCTURE_SRCS = \
    $(wildcard src/*.cpp)

# Specify data structure objects    
DATA_STRUCTURE_OBJS=$(patsubst %.cpp, %.o, $(DATA_STRUCTURE_SRCS))


# Build DataStructure Objects
$(DATA_STRUCTURE_OBJS): src/%.o : src/%.cpp
	$(COMPILER) $(CPPFLAGS) -c $< -o $@


clean:
	rm src/*.o
	rm bin/*

