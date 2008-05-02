#
# LKDemo Example Makefile
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
POUT = lkdemo

#
# Makefile
#

# build program
all: lkdemo.o
	$(CC) $(CFLAGS) $(OPENCVL) lkdemo.o -o $(POUT)

# compile program
lkdemo.o: lkdemo.c
	$(CC) -c $(DFLAGS) $(OPENCVI) lkdemo.c

# clean products of compile and build
clean:
	rm -f *.o
	rm -f *~*
	rm -f $(POUT)
	
