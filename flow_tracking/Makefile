#
# Flow Track Makefile
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
POUT = flow_track

#
# Makefile
#

# build program
all: flow_track.o calc_flows.o
	$(CC) $(CFLAGS) $(OPENCVL) $(BOOSTFSL) flow_track.o calc_flows.o -o $(POUT)

# compile program
flow_track.o: flow_track.cxx flow_track.h
	$(CC) -c $(DFLAGS) $(OPENCVI) flow_track.cxx

# compile flow component of program
calc_flows.o: calc_flows.cxx calc_flows.h img_template.tpl
	$(CC) -c $(DFLAGS) $(OPENCVI) calc_flows.cxx

# clean products of compile and build
clean:
	rm -f *.o
	rm -f *~*
	rm -f $(POUT)
	
