/**
 * @file Reconstruct3D.cpp
 */

#include "R3D.h"
#include "PCL_Tools/PCL_Tools.h"

char rgbWindow[50] = "RGB Window";

/**
 * @function main
 */
int main( int argc, char* argv[] )  {
  
  R3D r3d( argv[1] );
  r3d.matchAllFrames_3( );
  r3d.findMaxPutativeMatches();
  r3d.Ransac_Rigid3D( 0, 1 );

  // Create viewer
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = createViewer();

  // View the path
  reset_PCL_Tools_counters();
  /*
  for( int i = 0; i < r3d.getNumFrames(); ++i ) {
    viewPCDRGBA( r3d.getPointCloud(i), viewer );
  }

  // Loop
  while( !viewer->wasStopped() ) {
    viewer->spinOnce(100);
    boost::this_thread::sleep( boost::posix_time::microseconds(100000));
    } 

  //cv::imshow( rgbWindow, r3d.copyRgb( 0 ) ); 
  */

  
  int key;
  int ind = 0;
  while( true ) {
    key = cv::waitKey(30);

    // If a key is pressed
    if( key != -1 ) {

      if( key == 27 ) {
	break;
      }
      for( int i = 0; i < r3d.getNumFrames(); ++i ) {
	if( key == i + 48 ) {
	  ind = i;
	  cv::imshow( rgbWindow, r3d.copyRgb( ind ) );
	  break;
	}
  if( key == 'a' ) {
	  ind++;
    ind = ind % r3d.getNumFrames();
    printf("Showing image %d out of %d \n", ind, r3d.getNumFrames() );
    //	  cv::imshow( rgbWindow, r3d.copyRgb( ind ) );
    cv::imshow( rgbWindow, r3d.getMaxMatchingDraw( ind ) );
    break;
  }
      }
    } // end if( key != -1 )
  }
  
  return 0;
}
