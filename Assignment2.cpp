/**
 * @file Assignment2.cpp
 * @brief 3D Reconstruction of Kinect Data
 */

#include "Utils.h"
#include <string>
#include <fstream>
#include <vector>


/**
 * @function main
 * @brief Main routine
 */
int main( int argc, char* argv[] ) {

  std::string rgbWindow( "RGB Window" );
  std::string depthWindow( "Depth Window" );

  // Create reconstruction3d object
  Reconstruction3D r3d( argv[1], argv[2] );


  // Get 3D Local data

  // Get the 3D (local) Points with color
  // getPCDData( rgbImgs, depthImgs );
  
  // Show it
  r3d.showRgbImage( rgbWindow, 0 );
  r3d.showDepthImage( depthWindow, 0 );
  
  int key;
  while(true) {
    key = cv::waitKey(30);

    if( key != -1 ) {

      // Stop the program
      if( key == 27 ) {
	break;
      }
      
      // Show different set
      for( int i = 0; i < r3d.getNumImages(); ++i ) {
	if( key ==  i + 48 ) {
	  printf("Showing image set %d \n", i );
	  r3d.showRgbImage( rgbWindow, i );
	  r3d.showDepthImage( depthWindow, i );
	  break;
	}
      }
    } // Pressed something (!= -1)
  }

  return 0;
}

