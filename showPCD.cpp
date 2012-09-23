/**
 * @file showPCD.cpp
 */

#include <iostream>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>

//-- Global variables
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr gCloudPtr( new pcl::PointCloud<pcl::PointXYZRGBA> );
boost::shared_ptr<pcl::visualization::CloudViewer> gViewer;
std::vector< pcl::PointCloud<pcl::PointXYZRGBA>::Ptr > gPointclouds;
int gNumPointclouds;
int gCurrentShown;

//-- Functions declaration
boost::shared_ptr<pcl::visualization::CloudViewer> createViewer();
bool readPCDData( char* _filenames, 
		  std::vector< pcl::PointCloud<pcl::PointXYZRGBA>::Ptr > &_pcdData );

/**
 * @function main
 * @brief Main function what else?
 */
int main( int argc, char* argv[] ) {

  std::string filename;
  
  if( argc != 3 ) {
    printf( "Enter text file with PCD names on it and number to show \n" );
    return 0;
  }
  filename = argv[1];
  
  // Read  
  if( !readPCDData( argv[1], gPointclouds ) ) {
    printf( " [X] Error reading PCD text file exiting \n" );
  }

  gNumPointclouds = gPointclouds.size();
  gCurrentShown = atoi( argv[2] ) % gNumPointclouds;

  // Create viewer
  gViewer = createViewer();
  
  // Visualize 
  printf("Showing pointcloud %d  out of %d \n", gCurrentShown, gNumPointclouds );
  gViewer->showCloud( gPointclouds[gCurrentShown] );

  while( !gViewer->wasStopped() ) {
    boost::this_thread::sleep( boost::posix_time::seconds(1) );
  }

  
}

///////////////// FUNCTION DEFINITIONS ////////////////////

/**
 * @function createViewer
 */
boost::shared_ptr<pcl::visualization::CloudViewer> createViewer() {
  boost::shared_ptr<pcl::visualization::CloudViewer> v( new pcl::visualization::CloudViewer("3D Viewer") );

  return (v);
}


/**
 * @function readPCDData
 */
bool readPCDData( char* _filenames, 
		  std::vector< pcl::PointCloud<pcl::PointXYZRGBA>::Ptr > &_pcdData ) {

  std::vector<std::string> pcdFiles;
  
  // Open Stream
  _pcdData.resize(0);
  std::ifstream ifs( _filenames );
  
  // Read
  std::string temp;
  while( getline( ifs, temp ) ) {
    pcdFiles.push_back( temp );
  }

  // Load them 
  for( int i = 0; i < pcdFiles.size(); ++i ) {
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZRGBA> );
    if( pcl::io::loadPCDFile<pcl::PointXYZRGBA>( pcdFiles[i], *cloud ) == -1 ) {
      PCL_ERROR("Could not read file \n");
      _pcdData.resize(0);
      return false;
    }

    _pcdData.push_back( cloud );
  }

  return true;
}
