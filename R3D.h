/**
 * @file R3D.h
 */

#ifndef __R_3D_H__
#define __R_3D_H__

//-- OpenCV headers
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
// For SURF
#include "opencv2/nonfree/features2d.hpp"
//-- PCL headers
#include <iostream>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>

#include <vector>

/**
 * @class R3D
 */
class R3D {

 public:
  R3D( char *_filenames );
  ~R3D();
  bool readPCDData( char* _filenames, 
		    std::vector< pcl::PointCloud<pcl::PointXYZRGBA>::Ptr > &_pcdData );

  bool getRgbImages( std::vector< pcl::PointCloud<pcl::PointXYZRGBA>::Ptr > _pcdData,
		     std::vector<cv::Mat> &_rgbImages ); 
  
  bool getGrayImages( const std::vector<cv::Mat> &_rgbImages,
		      std::vector<cv::Mat> &_grayImages );

  void matchAllFrames_1();
  void matchAllFrames_3();

  void findMaxPutativeMatches();
  cv::Mat getMaxMatchingDraw( int _ind  );

  inline cv::Mat copyRgb( int _ind );
  inline int getNumFrames();
  int getNumMaxMatchesSize( int _ind );
  int getMaxMatchesIndices( int _ind );
  bool matchHasDepth( int _ind1, int _ind2,
		      const cv::DMatch &_match );

  // RANSAC stuff
  bool Ransac_Rigid3D( int _ind1, int _ind2 );
  cv::Mat get_Model_Rigid3D( int _ind1,
			     int _ind2, 
			     std::vector<int> _indices );
  void get3DPointsFromMatch( int _ind1, int _ind2, 
			     const cv::DMatch &_match,
			     float &_X1, float &_Y1, float &_Z1, 
			     float &_X2, float &_Y2, float &_Z2 );
  std::vector<int> getRandomIndices( int _numSamples, int _totalSamples );
  bool isInSet( int _index, 
		std::vector<int> _currentIndices );
  int getRandom( int _max );

  // Getters
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr getPointCloud( int _ind );

 private:

  // Image / PCD Data variables
  std::vector<cv::Mat> mRgbImages;
  std::vector<cv::Mat> mGrayImages;
  std::vector< pcl::PointCloud<pcl::PointXYZRGBA>::Ptr > mPointClouds;
  int mNumFrames;
  int mHeight;
  int mWidth;
  
  // SURF Keypoint detection and descriptor
  int mMinHessian;
  float mMaxDistance;
  float mRadiusFactor;

  std::vector< std::vector<cv::KeyPoint> > mKeypoints; 
  std::vector< cv::Mat > mDescriptors;
  std::vector< std::vector< std::vector< cv::DMatch > > > mMatches;
  
  std::vector<int> mMaxMatchesIndices;
  std::vector<int> mMaxMatchesSize;

  // RANSAC variables
  int mN; /**< Num Trials */
  int mS; /**< Model instantiation size */

  cv::SVD svd;
  cv::Mat mA;
  cv::Mat mB;
  cv::Mat mParams;
};
  
////////////////// INLINE FUNCTIONS /////////////////////////////////////// 

/**
 * @function copyRgb
 */
inline cv::Mat R3D::copyRgb( int _ind ) {
  
  if( _ind < mNumFrames ) {
      return mRgbImages[_ind];
  }
  }

/**
 * @function getNumFrames
 */
inline int R3D::getNumFrames() {
  return mNumFrames;
}


#endif /** __R_3D_H__ */
  
