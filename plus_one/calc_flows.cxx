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
#include "img_template.tpl"	// provides efficient access to pixels

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
}

// Access Functions

int Calc_Flows::get_size(){
  return orig_images.size();
}

// Action Functions

bool Calc_Flows::add(string filename){
  
  // load the image
  IplImage *img = NULL;
  img = cvLoadImage(filename.c_str(),0);  // scan in grayscale
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
    //img1 = orig_images[i];
    //img2 = orig_images[i+1];
    img1 = cvCloneImage(orig_images[i]);
    img2 = cvCloneImage(orig_images[i+1]);
    
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
    if(!img1 || !img2 || height1 != height2 || width1 != width2 || channels1 != channels2){
      printf("[ERROR] Images are not same dimensions, number of channels!");
      return IMAGE_CONSISTENCY_FAILED;
    }

    // calculate flow
    IplImage *lkvelX = NULL, *lkvelY = NULL; 	// used to store flow velocities (Lucas & Kanade)

    // create images to store velocities in X and Y directions
    lkvelX = cvCreateImage(cvGetSize(img1), IPL_DEPTH_32F, 1);
    lkvelY = cvCreateImage(cvGetSize(img2), IPL_DEPTH_32F, 1);

    if(verbose)
      cout << "    * " << "Processing optical flow of image pair #" << i << "...";

    // calculate flow (Lucas & Kanade algorithm)
    cvCalcOpticalFlowLK(img1, img2, cvSize(3, 3), lkvelX, lkvelY); 

    // create magnitude/angle image maps for velocities 
    IplImage *mag = NULL, *ang = NULL;	// holds magnitude, angle of each flow vector
    mag = cvCreateImage(cvGetSize(img1), IPL_DEPTH_32F, 1);
    ang = cvCreateImage(cvGetSize(img1), IPL_DEPTH_32F, 1);
    cvCartToPolar(lkvelX, lkvelY, mag, ang);

    // find edges of original image
    //IplImage *mag_edges = NULL;
    //mag_edges = cvCreateImage(cvGetSize(img1), IPL_DEPTH_8U, 1);
    //cvCanny(img1, mag_edges, 3, 6, 3);  

    // count angles
    int left = 0, right = 0;
    double sum_left_col = 0, sum_right_col = 0, ave_left_col = 0, ave_right_col = 0;
    BwImageFloat angles(ang);
    for(int r = 0; r < ang->height; r++){
      for(int c = 0; c < ang->width; c++){
        if(angles[r][c] > M_PI){	
          left++;			// count leftward angles
          sum_left_col += c;		// record leftward positions
          //sum_sq_left_col += c*c;	// used for sample standard deviation
        }
        else{
          right++;			// count rightward angles
          sum_right_col += c;		// record rightward positions
          //sum_sq_right_col += c*c;	// used for sample standard deviation
        }
      }
    }
    ave_left_col = sum_left_col / (ang->height*ang->width);		// average leftward column
    ave_right_col = sum_right_col / (ang->height*ang->width);		// average rightward column

    if(left < right && ave_left_col < ave_right_col){
      directions.push_back(FORWARD);
      //cout << "FORWARD" << endl;
    }
    else if(left > right && ave_left_col > ave_right_col){
      directions.push_back(BACKWARD);
      //cout << "BACKWARD" << endl;
    }
    else{
      directions.push_back(UNDETERMINED);
    }

    // display edge magnitude data
    //cvNamedWindow("edges", CV_WINDOW_AUTOSIZE); 
    //cvShowImage("edges", mag_edges);
    //cvNamedWindow("magnitudes", CV_WINDOW_AUTOSIZE); 
    //cvShowImage("magnitudes", mag);
    //cvNamedWindow("img", CV_WINDOW_AUTOSIZE); 
    //cvShowImage("img", img2);
    //cvWaitKey(0);
    //cvDestroyAllWindows();

    // release the images
    cvReleaseImage(&lkvelX);
    cvReleaseImage(&lkvelY);
    cvReleaseImage(&mag);
    cvReleaseImage(&ang);
    cvReleaseImage(&img1);
    cvReleaseImage(&img2);
    //cvReleaseImage(&mag_edges);

    if(verbose)
      cout << "\t\t\t\t[OK]" << endl;
  }

  return 0;
}

void Calc_Flows::animate(){

  direction prev_direction = UNDETERMINED;

  for(int i = 0; i < orig_images.size() - 1; i++){
    
    // load image
    IplImage *img = NULL;
    //img = orig_images[i+1];
    img = cvCloneImage(orig_images[i+1]);

    // add text
    CvFont font;
    double hScale=1.0;
    double vScale=1.0;
    int    lineWidth=1;
    cvInitFont(&font,CV_FONT_HERSHEY_TRIPLEX, hScale,vScale,0,lineWidth);
    
    if(directions[i] == FORWARD)
      cvPutText (img,"FORWARD",cvPoint(200,420), &font, cvScalar(255,255,0));
    else if(directions[i] == BACKWARD)
      cvPutText (img,"BACKWARD",cvPoint(200,420), &font, cvScalar(255,255,0));
    else if(directions[i] == UNDETERMINED){
      if(prev_direction == FORWARD)
        cvPutText (img,"FORWARD",cvPoint(200,420), &font, cvScalar(255,255,0));
      else if(prev_direction == BACKWARD)
        cvPutText (img,"BACKWARD",cvPoint(200,420), &font, cvScalar(255,255,0));
      else
        cvPutText (img,"UNDETERMINED",cvPoint(200,420), &font, cvScalar(255,255,0));
    }

    prev_direction = directions[i];

    // output images
    //char* outfile;
    //if(i < 10) sprintf(outfile, "out/00%d.png", i);
    //else if(i < 100)  sprintf(outfile, "out/0%d.png", i);
    //else sprintf(outfile, "out/%d.png", i);
    stringstream ostream;
    string outfile;
    ostream << i << ".png";
    outfile = ostream.str();
    if(i < 10) outfile = "0" + outfile;
    if(i < 100) outfile = "0" + outfile;
    outfile = "out/" + outfile;

    cvSaveImage(outfile.c_str(),img);      // add the frame to a file   
  }
}
