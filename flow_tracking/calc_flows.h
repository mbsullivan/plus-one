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
const int WINDOW_SIZE = 10;	// size of neighborhood about a pixel to determine corners

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
  bool add(string);			// add an image
  int run();				// assumes DEFAULT_VERBOSITY
  int run(bool);			// find optical flow
  void animate(string);			// assumes DEFAULT_VERBOSITY
  void animate(string, bool);		// output a movie of the results
  int run_webcam(bool verbose);
    
private:
  // Data representation objects
  vector<IplImage*> orig_images;
  vector<IplImage*> gray_images;
  vector<IplImage*> annotated_images;

  // State of machine
  bool ran;				// whether differences have been calculated
  char* where_flow_found;
  int num_tracked_points;
  int lk_flags;

  // Points to track
  CvPoint2D32f *prev_points, *curr_points, *swap_points;

  // Pyramids
  IplImage *prev_pyramid, *curr_pyramid, *swap_pyramid;

  // Action Functions
  void init(IplImage*);
  void pair_flow(IplImage*,IplImage*);	// calculate the flow between two images
  IplImage* annotate_img(IplImage*);
};


#endif
