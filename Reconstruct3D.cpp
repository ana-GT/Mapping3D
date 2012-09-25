/**
 * @file Reconstruct3D.cpp
 */

#include "R3D.h"
#include "PCL_Tools/PCL_Tools.h"

char rgbWindow[50] = "RGB Window";
char depthWindow[50] = "Depth Window";
char matchWindow[50] = "Match Window";
char keypointWindow[50] = "Keypoint Window";

/**
 * @function main
 */
int main( int argc, char* argv[] )  {
  
  R3D r3d( argv[1] );
  r3d.matchAllFrames_3( );
  r3d.findMaxPutativeMatches();
  r3d.getAllTransforms();
  r3d.getTransformedPointClouds();
  /*
  // Create viewer
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = createViewer();

  // View the path
  reset_PCL_Tools_counters();
  int num = r3d.getNumFrames();
  for( int i = 0; i < num; ++i ) {
    viewPCDRGBA( r3d.getTransformedPointCloud(i), viewer );
  }

  // Loop
    while( !viewer->wasStopped() ) {
    viewer->spinOnce(100);
    boost::this_thread::sleep( boost::posix_time::microseconds(100000));
    } 
  */

  cv::imshow( rgbWindow, r3d.getRgb( 0 ) ); 
    
  int key;
  int ind = 0;
  while( true ) {
    key = cv::waitKey(30);

    // If a key is pressed
    if( key != -1 ) {
      
      if( key == 27 ) {
	break;
      }
      
      if( key == 'a' ) {
	ind++;
	ind = ind % r3d.getNumFrames();
	printf("Showing image %d out of %d \n", ind, r3d.getNumFrames() );
	//cv::imshow( rgbWindow, r3d.getRgb( ind ) );
	//cv::imshow( depthWindow, r3d.getDepth( ind ) );
	cv::imshow( matchWindow, r3d.getMaxMatchingDraw( ind ) );
	//cv::imshow( keypointWindow, r3d.getKeypointsDraw( ind ) );
      }
   
    } // end if( key != -1 )
  }
  
  return 0;
}
