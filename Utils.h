/**
 * @file Utils.h
 */

//-- OpenCV headers
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
//-- PCL headers
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/console/parse.h>
#include <pcl/surface/gp3.h>

#include <boost/thread/thread.hpp>
#include <iostream>

//-- General headers
#include <vector>
#include <string>

#ifndef _RECONSTRUCTION_3D_
#define _RECONSTRUCTION_3D_

/**
 * @class Reconstruction3D
 */
class Reconstruction3D {

 public:

  //-- Must be
  Reconstruction3D( char* _rgbFilenames, char* _depthFilenames );
  ~Reconstruction3D();

  //-- Getters
  inline int getNumImages();

  //-- Utilities
  bool readDepthImages( std::string _depthFilename, std::vector<cv::Mat> &_depthImages );
  bool readRgbImages( std::string _rgbFilename, std::vector<cv::Mat> &_rgbImages );
  bool showRgbImage( std::string _windowName, int _ind );
  bool showDepthImage( std::string _windowName, int _ind );

  //-- Point cloud Utilities
  bool getPCDData( std::vector<cv::Mat> _rgbImgs,
		   std::vector<cv::Mat> _depthImgs,
		   std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &_pcdData );
  void depthToColorPixel( int _x_d, int _y_d, int _depth,
			  float &_P3Dx, float &_P3Dy, float &_P3Dz );
  float rawDepthToMeters( int _rawDepth );

 private:

  std::vector<cv::Mat> mRgbImages;
  std::vector<cv::Mat> mDepthImages;
  int mNumImages;


  int mCx_d;
  int mCy_d;
  float mFx_d;
  float mFy_d;

  
};

//////////////////// INLINE FUNCTIONS /////////////////////////

/**
 * @function getNumImages
 */
inline int Reconstruction3D::getNumImages() {
  return mNumImages;
}

#endif /** _RECONSTRUCTION_3D_ */
