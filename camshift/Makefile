#
# camshift Example Makefile
#
# Usage:
#    make        	(to build program
#    make clean         (to remove old files)
#

#
# Variables to Use
#

# Compiler to use
CC = gcc
# Flags for stricter compile options
CFLAGS = -Wall -O3
# Flags for debugging output
DFLAGS =
# OpenCV includes
OPENCVI = `pkg-config --cflags opencv`
# OpenCV libraries
OPENCVL = `pkg-config --libs opencv`
# Name of program executable
POUT = camshift

#
# Makefile
#

# build program
all: camshift.o
	$(CC) $(CFLAGS) $(OPENCVL) camshift.o -o $(POUT)

# compile program
camshift.o: camshift.c
	$(CC) -c $(DFLAGS) $(OPENCVI) camshift.c

# clean products of compile and build
clean:
	rm -f *.o
	rm -f *~*
	rm -f $(POUT)
	
