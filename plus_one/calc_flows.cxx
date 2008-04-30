/*
 * calc_flows.cxx - Implementation of Calc_Flows functions
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

#include "calc_flows.h" 	// calc_flows header file

// Constructors

Calc_Flows::Calc_Flows(){
// Default Constructor
  ran = false;
}

Calc_Flows::~Calc_Flows(){
// Destructor
  // release all images
  for(int i = 0; i < orig_images.size(); i++){
    cvReleaseImage(&orig_images[i]);
  }
  for(int j = 0; j < orig_images.size(); j++){
    cvReleaseImage(&velocity_x[j]);
  }
  for(int k = 0; k < orig_images.size(); k++){
    cvReleaseImage(&velocity_y[k]);
  }
}

// Access Functions

int Calc_Flows::get_size(){
  return orig_images.size();
}

// Action Functions

bool Calc_Flows::add(string filename){
  
  // load the image
  IplImage *img = NULL;
  img = cvLoadImage(filename.c_str(),BOOL_COLOR_IMG);
  if(!img) return false;

  // store the image
  orig_images.push_back(img);

  return true;
}

int Calc_Flows::run(){
// Default run function (with default verbosity)
  return run(DEFAULT_VERBOSITY);
}

int Calc_Flows::run(bool verbose){

  if(verbose)
    cout << endl << "  * " << "Calculating optical flow for " << orig_images.size() - 1 << " pairs of images..." << endl;
  
  for(int i = 0; i < orig_images.size() - 1; i++){
    
    // load images
    IplImage *img1 = NULL, *img2 = NULL;
    img1 = orig_images[i];
    img2 = orig_images[i+1];
    
    // check that images are consistent
    int height1,width1,channels1;			
    int height2,width2,channels2;

    // parse first image data
    height1    = img1->height;
    width1     = img1->width;
    channels1  = img1->nChannels;
  
    // parse second image data
    height2    = img2->height;
    width2     = img2->width;
    channels2  = img2->nChannels;
 
    // check for inequality in image dimensions, channels
    if(height1 != height2 || width1 != width2 || channels1 != channels2){
      printf("[ERROR] Images are not same dimensions, number of channels!");
      return IMAGE_CONSISTENCY_FAILED;
    }

    // calculate flow
    IplImage *gray1 = NULL, *gray2 = NULL;	// used to store grayscale image information
    IplImage *lkvelX = NULL, *lkvelY = NULL; 	// used to store flow velocities (Lucas & Kanade)

    // convert images to 8-bit, single channel
    gray1 = cvCreateImage(cvGetSize(img1), IPL_DEPTH_8U, BOOL_COLOR_IMG);
    gray2 = cvCreateImage(cvGetSize(img2), IPL_DEPTH_8U, BOOL_COLOR_IMG);
    cvCvtColor(img1, gray1, CV_BGR2GRAY);	 // convert img1 to color space of gray1
    cvCvtColor(img2, gray2, CV_BGR2GRAY);  // convert img2 to color space of gray2

    // create images to store velocities in X and Y directions
    lkvelX = cvCreateImage(cvGetSize(img1), IPL_DEPTH_32F, BOOL_COLOR_IMG);
    lkvelY = cvCreateImage(cvGetSize(img2), IPL_DEPTH_32F, BOOL_COLOR_IMG);

    if(verbose)
      cout << "    * " << "Processing optical flow of image pair #" << i << "...";

    // calculate flow (Lucas & Kanade algorithm)
    cvCalcOpticalFlowLK(gray1, gray2, cvSize(3, 3), lkvelX, lkvelY); 

    // store flow
    velocity_x.push_back(lkvelX);
    velocity_y.push_back(lkvelY);

    if(verbose)
      cout << "\t\t\t\t[OK]" << endl;
  }

  return 0;
}

