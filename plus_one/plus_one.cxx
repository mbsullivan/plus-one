/*
 * plus_one.cxx - Executable Image Flow Analysis Application
 * (c) 2008 Michael Sullivan
 *
 * Version 1.0.1
 * Last Revised: 04/29/08
 *
 * Version History:
 *   1.0.1 - Made object oriented and incorporated getopt.h for option parsing (04/29/08)
 *   1.0.0 - Initial "simple image flow example" (04/22/08)
 *
 * Code snippets for original "simple image flow example" taken from:
 * (1) lkdemo.c (available at: http://www.csie.ntu.edu.tw/~r94082/samples/c/lkdemo.c)
 * (2) Gady Agam's Introduction to programming with OpenCV (available at: http://www.cs.iit.edu/~agam/cs512/lect-notes/opencv-intro/opencv-intro.html#SECTION00025000000000000000)
 * (3) ymazari@yahoo.fr (available at: http://groups.google.ws/group/OpenCV/msg/ec125224a1123fc2)
 * (4) cegparamesh@gmail.com (available at: http://opencvlibrary.sourceforge.net/DisplayManyImages)
 *
 * Code for Boost filesystem code taken from:
 * (1) simple_ls.cpp (available at http://www.boost.org/doc/libs/1_35_0/libs/filesystem/example/simple_ls.cpp)
 *
 * All code snippets have been heavily modified for the most recent version of the program.
 *
 */

#include "plus_one.h"	// program header
#include "calc_flows.h" // performs optical flow operations

int main(int argc, char *argv[]){

  int optchar;							// for option input

  // handle input flags
  while((optchar = getopt(argc, argv, "i:f:s?")) != -1){	// read in arguments
    switch(optchar){
      case 'i':			// input directory
        input_directory = new string(optarg);
        break;
      case 'f':                 // file format
        file_format = new string(optarg);
        if(file_format->at(0) != '.') *file_format = "." + *file_format;
        break;
      case 's':                 // verbosity
        verbose = false;
        break;
      default:                  // display syntax help
      case '?':
        return display_program_syntax();
      }
    }
  
  // output title block, if applicable
  if(verbose) display_program_header();

  // resolve path name and find directory
  fs::path full_path(fs::initial_path<fs::path>());
  full_path = fs::system_complete(fs::path(input_directory->c_str(), fs::native));
  if(!fs::exists(full_path) || !fs::is_directory(full_path)){
    cout << "[ERROR] Invalid input directory (" << full_path.native_directory_string() << ")!" << endl;
    return INVALID_INPUT_DIRECTORY;
  }

  if(verbose){
    cout << "  * " << "Finding *" << *file_format << " in " << full_path.native_directory_string() << endl;
  }

  // to store calculated flow information and intermediary data
  Calc_Flows *flows = new Calc_Flows();

  // recurse through directory and handle all valid files
  fs::directory_iterator end_iter;
  for(fs::directory_iterator dir_itr(full_path); dir_itr != end_iter; ++dir_itr){

    // make sure is regular (i.e. non-directory) file
    if(fs::is_regular(dir_itr->status())){

      // make sure file extension is correct
      string ext = fs::extension(dir_itr->leaf());
      if(file_format->compare(ext) == 0){
        if(verbose){
          cout << "    * " << "Processing " << dir_itr->leaf() << "...";
        }

        // parse file (for later processing)
	fs::path target_file(full_path);
        target_file /= dir_itr->leaf();
        //bool added = flows->add(fs::system_complete(fs::path(dir_itr->leaf(), fs::native)).native_directory_string());
        bool added = flows->add(target_file.native_directory_string());
	
        if(verbose){
          cout << "\t\t\t\t";
          if(added)
            cout << "[OK]" << endl;
          else
            cout << "[FAIL]" << endl;
        }
      }
    }
  }

  // find optical flow for each pair of images
  flows->run();
  flows->animate();

  return 0;
}

void display_program_header(){
  cout << endl;
  cout << " " << PROGRAM_NAME << " v." << PROGRAM_VERSION << endl;
  cout << " " << "Written by " << AUTHOR_INFO << endl;
  cout << " " << VERSION_DATE << endl;
  cout << " --------------------------------------------" << endl;
  cout << endl;
}

int display_program_syntax(){
  cout << endl;
  cout << PROGRAM_NAME << ": Approximate a third dimension from images using optical flow." << endl;
  cout << endl;

  cout << "Syntax: " << PROGRAM_NAME << " -i (directory) [-f (file format) -s -v]" << endl;
  cout << "  " << "-i (directory)" << ": Process all image files from the given directory (default \"" << DEFAULT_INPUT_DIRECTORY << "/\")" << endl;
  cout << "  " << "-f (file format)" << ": Read any images with the given file extension (default *." << DEFAULT_FILE_FORMAT << ")" << endl;
  cout << "  " << "-s" << ": Disable program output" << endl;
  cout << "  " << "-?" << ": Display this screen" << endl;
  cout << endl;

  return 0;
}


