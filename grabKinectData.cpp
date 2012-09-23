/**
 * @file grabKinectData
 */

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <stdio.h>
#include <string>

#include <iostream>

/// Global variables
char rgbWindow[50] = "RGB Image";
char depthMapWindow[50] = "Depth Map";

/// Function headers
void printDeviceInfo( cv::VideoCapture &_capture );
void parseCommandLine( int _argc, char* _argv[], 
		       std::string &_filename,
		       bool &_isFileReading );
void printCommandLineParams();

/**
 * @function main 
 * @brief Main routine
 */
int main( int argc, char* argv[] ) {

  // Variables to indicate video file reading (no capturing)
  std::string filename;
  bool isVideoReading;
  
  // Parse input directives
  parseCommandLine( argc, argv, filename, isVideoReading );

  // Open video source
  printf("[i] Device is opening! \n");
  cv::VideoCapture capture;

  // If reading a video file
  if( isVideoReading ) {
    capture.open( filename );
  }
  // Otherwise, get video stream from Kinect
  else {
    capture.open( CV_CAP_OPENNI );
  }
  printf( "[i] Device opened \n" );

  // If video source could not be opened
  if( !capture.isOpened() ) {
    printf(" Cannot open a capture object \n");
    return -1;
  }

  // Capture 
  if( !isVideoReading ) {
    capture.set( CV_CAP_OPENNI_IMAGE_GENERATOR_OUTPUT_MODE, CV_CAP_OPENNI_VGA_30HZ );   
  } 

  //-- Print device info
  printDeviceInfo( capture );

  // Loop
  for( ;; ) {
    cv::Mat depthMap;
    cv::Mat bgrImage;
    cv::Mat depthMapShow;

    // Grab a frame
    if( !capture.grab() ) {
      printf( "[!] Cannot grab images, bang! \n" );
      return -1;
    }
    else {
      if( capture.retrieve( depthMap, CV_CAP_OPENNI_DEPTH_MAP ) ) {
	const float scaleFactor = 0.05f;
	depthMap.convertTo( depthMapShow, CV_8UC1, scaleFactor );
	cv::imshow( depthMapWindow, depthMapShow );	
      }

      if( capture.retrieve( bgrImage, CV_CAP_OPENNI_BGR_IMAGE) ) {
	cv::imshow( rgbWindow, bgrImage );
      }

    }

    if( cv::waitKey( 30 ) >= 0 ) {
      break;
    }
  }

  return 0;

}
  

///////////////////// FUNCTION IMPLEMENTATIONS ///////////////////////////


/**
 * @function printDeviceInfo
 */
void printDeviceInfo( cv::VideoCapture &_capture ) {

  // Print depth info
  printf( " ** Depth generator output mode: \n" );
  printf( " * Frame width: %d \n pixels ", (int)( _capture.get(CV_CAP_PROP_FRAME_WIDTH) ) );
  printf( " * Frame height: %d \n pixels ", (int)( _capture.get(CV_CAP_PROP_FRAME_HEIGHT) ) );
  printf( " * Frame max depth: %d mm \n", (int)( _capture.get(CV_CAP_PROP_OPENNI_FRAME_MAX_DEPTH) ) );
  printf( " * Frames per second: %d \n", (int)( _capture.get(CV_CAP_PROP_FPS) ) );
  printf( " * Registration: %d \n", (int)( _capture.get(CV_CAP_PROP_OPENNI_REGISTRATION) ) );

  // Print image info if available
  if( _capture.get( CV_CAP_OPENNI_IMAGE_GENERATOR_PRESENT ) ) {
    printf( "** Image generator output mode: \n" );
    printf( "* Frame width: %d pixels \n", (int)( _capture.get( CV_CAP_OPENNI_IMAGE_GENERATOR+CV_CAP_PROP_FRAME_WIDTH ) ) );
    printf( "* Frame height: %d pixels \n", (int)( _capture.get( CV_CAP_OPENNI_IMAGE_GENERATOR+CV_CAP_PROP_FRAME_HEIGHT ) ) );
    printf( "* Frames per second: %d \n", (int)( _capture.get( CV_CAP_OPENNI_IMAGE_GENERATOR+CV_CAP_PROP_FPS ) ) );
  }

}

/**
 * @function parseCommandLine
 */
void parseCommandLine( int _argc, char* _argv[], 
		       std::string &_filename,
		       bool &_isFileReading ) {

  // Set default values
  
  _filename.clear();
  _isFileReading = false;

  if( _argc == 1 ) {
    // Default
  } else {
    for( int i = 1; i < _argc; ++i ) {
      if( !strcmp( _argv[i], "--help" ) || !strcmp( _argv[i], "-h") ) {
	printCommandLineParams();
	exit(0);
      }
      else if( !strcmp( _argv[i], "-r" ) ) {
	_filename = _argv[++i];
	_isFileReading = true;
      }
      else {
	printf( "Unsupported command line argument: %s \n", _argv[i] );
	exit(-1);
      }

    } // end for 
  } // end else
}


/**
 * @function printCommandLineParams
 */
void printCommandLineParams() {
  printf( "--help: Usage help \n");
  printf( " -r: Filename of video file \n" );
}
