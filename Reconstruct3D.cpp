/**
 * @file Reconstruct3D.cpp
 */

#include "R3D.h"
#include "PCL_Tools/PCL_Tools.h"

char rgbWindow[50] = "RGB Window";
char depthWindow[50] = "Depth Window";
char matchWindow[50] = "Match Window";

/**
 * @function main
 */
int main( int argc, char* argv[] )  {
  
  R3D r3d( argv[1] );
  r3d.matchAllFrames_3( );
  r3d.findMaxPutativeMatches();
  cv::Mat t01;
  int img1 = 8;
  int img2 = 9;
  t01 = r3d.Ransac_Rigid3D( img1, img2 );

  // Create viewer
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = createViewer();

  // View the path
  reset_PCL_Tools_counters();
  /*
  for( int i = 0; i < r3d.getNumFrames(); ++i ) {
    viewPCDRGBA( r3d.applyRigid3DToPCD( t01, r3d.getPointCloud(i) ), viewer );
    }*/
  viewPCDRGBA( r3d.getPointCloud(img2), viewer );
  viewPCDRGBA( r3d.applyRigid3DToPCD( t01,r3d.getPointCloud(img1) ), viewer );

  // Loop
    while( !viewer->wasStopped() ) {
    viewer->spinOnce(100);
    boost::this_thread::sleep( boost::posix_time::microseconds(100000));
    } 
  
    //  cv::imshow( rgbWindow, t01 );
 
  //cv::imshow( rgbWindow, r3d.copyRgb( 0 ) ); 
    /* 
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
	cv::imshow( rgbWindow, r3d.getRgb( ind ) );
	cv::imshow( depthWindow, r3d.getDepth( ind ) );
	cv::imshow( matchWindow, r3d.getMaxMatchingDraw( ind ) );
      }
   
    } // end if( key != -1 )
  }
    */
  return 0;
}
