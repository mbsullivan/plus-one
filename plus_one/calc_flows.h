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
//# define M_PI 3.14159265358979323846 	// pi (from math.h)

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
};


#endif
