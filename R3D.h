/**
 * @file R3D.h
 * @author A. Huaman :D
 * @date Damn 2012/09/23
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
#include <pcl/common/transforms.h>

#include <vector>

/**
 * @class R3D
 */
class R3D {

 public:

  // Constructor and destructor
  R3D( char *_filenames );
  ~R3D();

  //-- RANSAC 
  cv::Mat Ransac_Rigid3D( int _ind1, int _ind2 );
  cv::Mat get_Model_Rigid3D( int _ind1,
			     int _ind2, 
			     std::vector<int> _indices );  
  float getErrorModel( int _ind1, int _ind2, int _ind,
		       const cv::Mat &_params );
  void get3DPointsFromMatch( int _ind1, int _ind2, 
			     const cv::DMatch &_match,
			     float &_X1, float &_Y1, float &_Z1, 
			     float &_X2, float &_Y2, float &_Z2 );
  std::vector<int> getRandomIndices( int _numSamples, 
				     int _totalSamples );
  bool isInSet( int _index, 
		std::vector<int> _currentIndices );
  int getRandom( int _max );
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr applyRigid3DToPCD( cv::Mat _m, 
							     pcl::PointCloud<pcl::PointXYZRGBA>::Ptr _inputCloud );

  //-- Matching
  void matchAllFrames_1();
  void matchAllFrames_3();
  void findMaxPutativeMatches();
  cv::Mat getMaxMatchingDraw( int _ind  );
  bool matchHasDepth( int _ind1, int _ind2,
		      const cv::DMatch &_match );
  cv::Mat getSomeMatchesDraw( int _ind1, int _ind2,
			      const std::vector<int> &matchesIndices );

  //-- Data acquisition
  bool readPCDData( char* _filenames, 
		    std::vector< pcl::PointCloud<pcl::PointXYZRGBA>::Ptr > &_pcdData );

  bool getRgbImages( std::vector< pcl::PointCloud<pcl::PointXYZRGBA>::Ptr > _pcdData,
		     std::vector<cv::Mat> &_rgbImages ); 
  
  bool getGrayImages( const std::vector<cv::Mat> &_rgbImages,
		      std::vector<cv::Mat> &_grayImages );
  bool getDepthImages( const std::vector< pcl::PointCloud<pcl::PointXYZRGBA>::Ptr > &_pcdData,
		       std::vector<cv::Mat> &_depthImages, std::vector<cv::Mat> &_depthData );

  //-- Get functions
  inline cv::Mat getRgb( int _ind );
  inline cv::Mat getDepth( int _ind );
  inline int getNumFrames();
  int getNumMaxMatchesSize( int _ind );
  int getMaxMatchesIndices( int _ind );
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr getPointCloud( int _ind );

 private:
  // Image / PCD data variables
  std::vector<cv::Mat> mRgbImages;
  std::vector<cv::Mat> mGrayImages;
  std::vector<cv::Mat> mDepthImages;
  std::vector<cv::Mat> mDepthData;
  std::vector< pcl::PointCloud<pcl::PointXYZRGBA>::Ptr > mPointClouds;
  int mNumFrames;
  int mHeight;
  int mWidth;
  
  // SURF Keypoint detection and descriptor
  int mMinHessian;
  float mRadiusFactor;

  std::vector< std::vector<cv::KeyPoint> > mKeypoints; 
  std::vector< cv::Mat > mDescriptors;
  std::vector< std::vector< std::vector< cv::DMatch > > > mMatches;
  
  std::vector<int> mMaxMatchesIndices;
  std::vector<int> mMaxMatchesSize;

  // RANSAC variables
  int mN; /**< Num Trials */
  int mS; /**< Model instantiation size */
  float mThresh;

  // SVD 
  cv::SVD svd;
  cv::Mat mA;
  cv::Mat mB;
  std::vector<cv::Mat> mParams;
};
  
////////////////// INLINE FUNCTIONS /////////////////////////////////////// 

/**
 * @function getRgb
 */
inline cv::Mat R3D::getRgb( int _ind ) {
  
  if( _ind < mNumFrames ) {
      return mRgbImages[_ind];
  }
}

/**
 * @function getDepth
 */
inline cv::Mat R3D::getDepth( int _ind ) {
  
  if( _ind < mNumFrames ) {
      return mDepthImages[_ind];
  }
}

/**
 * @function getNumFrames
 */
inline int R3D::getNumFrames() {
  return mNumFrames;
}


#endif /** __R_3D_H__ */
  
