#
# Plus One Makefile
#
# Usage:
#    make        	(to build program
#    make clean         (to remove old files)
#

#
# Variables to Use
#

# Compiler to use
CC = g++
# Flags for stricter compile options
CFLAGS = -Wall -O3
# Flags for debugging output
DFLAGS =
# OpenCV includes
OPENCVI = `pkg-config --cflags opencv`
# OpenCV libraries
OPENCVL = `pkg-config --libs opencv`
# Boost Filesystem libraries
BOOSTFSL = -lboost_filesystem
# Name of program executable
POUT = plus_one

#
# Makefile
#

# build program
all: plus_one.o calc_flows.o
	$(CC) $(CFLAGS) $(OPENCVL) $(BOOSTFSL) plus_one.o calc_flows.o -o $(POUT)

# compile program
plus_one.o: plus_one.cxx plus_one.h
	$(CC) -c $(DFLAGS) $(OPENCVI) plus_one.cxx

# compile flow component of program
calc_flows.o: calc_flows.cxx calc_flows.h
	$(CC) -c $(DFLAGS) $(OPENCVI) calc_flows.cxx

# clean products of compile and build
clean:
	rm -f *.o
	rm -f *~*
	rm -f $(POUT)
	
