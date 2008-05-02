/*
 * calc_flows.h - Calculate Optical Flows and Differences
 * (c) 2008 Michael Sullivan
 *
 * Version 1.0.0
 * Last Revised: 04/29/08
 *
 * Version History:
 *  1.0.0 - Primary implementation (04/29/08)
 *
 *  This program utilizes the Open Computer Vision Library (OpenCV)
 *
 */

#ifdef _CH_
#pragma package <opencv>
#endif

#ifndef _CALC_FLOWS_H_
#define _CALC_FLOWS_H_

// includes
#include "cv.h"
#include "highgui.h"
#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <math.h>

// namespace preparation
using namespace std;

// constants
const bool DEFAULT_VERBOSITY = true;	// assume verbose
const int MAX_POINTS_TO_TRACK = 500;	// maximum number of points to track
const int SIZE_OF_CORNER_NEIGHBORHOOD = 10;	// size of neighborhood about a pixel to determine corners

// error codes
#define IMAGE_CONSISTENCY_FAILED -1

// types
enum direction { FORWARD, BACKWARD, UNDETERMINED };

class Calc_Flows{
public:
  // Constructors
  Calc_Flows();	 // constructor 
  ~Calc_Flows(); // destructor

  // Access Functions
  int get_size();		// returns number of processed images
    
  // Action Functions
  bool add(string filename);		// add an image
  int run();				// assumes DEFAULT_VERBOSITY
  int run(bool verbose);		// find optical flow
  void animate();			// output a movie of the results
    
private:
  // Data representation objects
  vector<IplImage*> orig_images;
  vector<direction> directions;

  // State of machine
  bool ran;				// whether differences have been calculated
  char* machine_status;
  int num_tracked_points;
  int flags;

  // Points to track
  CvPoint2D32f* points[2], *swap_points;
};


#endif
