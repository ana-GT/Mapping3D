/**
 * @file R3D.cpp
 */

// Cool stuff about calibration and getting X,Y from Z
// http://nicolas.burrus.name/index.php/Research/KinectCalibration
// https://groups.google.com/forum/?fromgroups=#!topic/openkinect/AxNRhG_TPHg

#include "R3D.h"
#include <time.h>
//#include <Eigen/Core>

/**
 * @function R3D
 */
R3D::R3D( char *_filenames ) {
  
  // Initialize a few things
  mMinHessian = 400;
  mMaxDistance = 0.2;
  mRadiusFactor = 3.0; // 3.0

  srand( time(NULL) );

  // Read PCDs
  mPointClouds.resize(0);
  if( !readPCDData( _filenames, mPointClouds ) ) {
    printf( "[X] No PCD Data read! \n" );
  }

  mNumFrames = mPointClouds.size();
  printf( "Loaded %d PCDs \n", mNumFrames );

  // Get RGBs
  getRgbImages( mPointClouds, mRgbImages );
  printf( "Loaded RGB images \n");

  // Get gray
  getGrayImages( mRgbImages, mGrayImages );
  printf( "Loaded %d Gray images \n", mGrayImages.size() );  
}

/**
 * @function ~R3D
 */
R3D::~R3D() {

}

/**
 * @function readPCDData
 */
bool R3D::readPCDData( char* _filenames, 
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

  // Initialize height and width
  mHeight = _pcdData[0]->height;
  mWidth = _pcdData[0]->width;
  return true;
 
}


/**
 * @function getRgbImages
 * @brief Explained in this thread: http://www.pcl-users.org/RGB-image-only-td3473623.html
 */
bool R3D::getRgbImages( std::vector< pcl::PointCloud<pcl::PointXYZRGBA>::Ptr > _pcdData,
			std::vector<cv::Mat> &_rgbImages ) {

  _rgbImages.resize(0);

  for( int i = 0; i < _pcdData.size(); ++i ) {

    cv::Mat rgbImage = cv::Mat( _pcdData[i]->height,
				_pcdData[i]->width,
				CV_8UC3 );
    
    int height = _pcdData[i]->height;
    int width = _pcdData[i]->width;
    for( int h = 0; h < height; ++h ) {
      for( int w = 0; w < width; ++w ) {
	rgbImage.at<cv::Vec3b>(h,w)[0] = _pcdData[i]->at(h*width + w ).b;
	rgbImage.at<cv::Vec3b>(h,w)[1] = _pcdData[i]->at(h*width + w ).g;
	rgbImage.at<cv::Vec3b>(h,w)[2] = _pcdData[i]->at(h*width + w ).r;
      }
    }
 
    _rgbImages.push_back( rgbImage );
  }

  return true;
}


/**
 * @function getGrayImages
 * @brief 
 */
bool R3D::getGrayImages( const std::vector<cv::Mat> &_rgbImages,
			 std::vector<cv::Mat> &_grayImages ) {

  _grayImages.resize(0);

  for( int i = 0; i < _rgbImages.size(); ++i ) {

    cv::Mat grayImage;
    _rgbImages[i].convertTo( grayImage, CV_8UC1 );
    _grayImages.push_back( grayImage );
  }

  return true;
}

/**
 * @function matchAllFrames_1
 */
void R3D::matchAllFrames_1() {
 
  printf("Match all frames_1 \n");
  cv::SURF surf( mMinHessian );
  

  // Detect keypoints and generate descriptors
  for( int i = 0; i < mNumFrames; ++i ) {

    std::vector<cv::KeyPoint> keypoint;
    cv::Mat descriptor;
    surf( mGrayImages[i], cv::Mat(), keypoint, descriptor, false );
    mKeypoints.push_back( keypoint );
    mDescriptors.push_back( descriptor );
    printf("[%d] Num keypoints: %d \n", i, keypoint.size() );
  }

  // Create a Brute Force Matcher object
  cv::BFMatcher matcher( cv::NORM_L2, false );

  // Match each descriptor successively
  printf( " Start matching process 1 \n" );
  for( int i = 0; i < mNumFrames; ++i ) {

    std::vector< std::vector< cv::DMatch > > matches_iToAll;
    for( int j = 0; j < mNumFrames; ++j ) {
      std::vector< cv::DMatch> matches_iToj;
      matcher.match( mDescriptors[i], mDescriptors[j], matches_iToj );
      matches_iToAll.push_back( matches_iToj );
    }

    mMatches.push_back( matches_iToAll );
  }
  
  printf("End matching process 1 with these results: \n");
  for( int i = 0; i < mNumFrames; ++ i ) {
    for( int j = 0; j < mNumFrames; ++j ) {
      printf(":[%d] %d  ", j, mMatches[i][j].size() );
    }
    printf("\n");
  }

}


/**
 * @function matchAllFrames_3
 */
void R3D::matchAllFrames_3() {
 
  printf("Match all frames_3 \n");
  cv::SURF surf( mMinHessian );
  

  // Detect keypoints and generate descriptors
  for( int i = 0; i < mNumFrames; ++i ) {

    std::vector<cv::KeyPoint> keypoint;
    cv::Mat descriptor;
    surf( mGrayImages[i], cv::Mat(), keypoint, descriptor, false );
    mKeypoints.push_back( keypoint );
    mDescriptors.push_back( descriptor );
    printf("[%d] Num keypoints: %d \n", i, keypoint.size() );
  }

  // Create a FlannBasedMatcher
  cv::FlannBasedMatcher matcher;
  std::vector< std::vector< std::vector< cv::DMatch > > > tempMatches;

  // Match each descriptor successively
  printf( " Start matching process_3 \n" );
  for( int i = 0; i < mNumFrames; ++i ) {

    std::vector< std::vector< cv::DMatch > > matches_iToAll;
    for( int j = 0; j < mNumFrames; ++j ) {
      std::vector< cv::DMatch> matches_iToj;
      matcher.match( mDescriptors[i], mDescriptors[j], matches_iToj );
      matches_iToAll.push_back( matches_iToj );
    }
    tempMatches.push_back( matches_iToAll );
  }

  // Resize
  mMatches.resize( mNumFrames );
  for( int i = 0; i < mNumFrames; ++i ) {
    mMatches[i].resize( mNumFrames );
  }

  // Quick calculation of max and min distances
  double max_dist; 
  double min_dist;
  double dist;

  for( int i = 0; i < mNumFrames; ++i ) {
    for( int j = 0; j < mNumFrames; ++j ) {

      max_dist = 0; min_dist = 100;
      for( int k = 0; k < tempMatches[i][j].size(); ++k ) {
	dist = tempMatches[i][j][k].distance;
	if( dist < min_dist ) { min_dist = dist; }
	if( dist > max_dist ) { max_dist = dist; }
      }

      // Get only good matches
      for( int k = 0; k < tempMatches[i][j].size(); ++k ) {
	if( tempMatches[i][j][k].distance <= mRadiusFactor*min_dist
	    && matchHasDepth( i, j, tempMatches[i][j][k] ) == true ) {
	  mMatches[i][j].push_back( tempMatches[i][j][k] );
	}
      }

    }
  }

  printf("End matching process 3 with these results: \n");
  for( int i = 0; i < mNumFrames; ++ i ) {
    for( int j = 0; j < mNumFrames; ++j ) {
      printf(":[%d] %d  ", j, mMatches[i][j].size() );
    }
    printf("\n");
  }

}

/**
 * @function matchHasDepth
 */
bool R3D::matchHasDepth( int _ind1, int _ind2,
			 const cv::DMatch &_match ) {

  int idx1; int idx2;
  cv::Point2f p1, p2;
  float X1; float Y1; float Z1;
  float X2; float Y2; float Z2;
 
  idx1 = _match.queryIdx;
  idx2 = _match.trainIdx;
  p1 = mKeypoints[_ind1][idx1].pt;
  p2 = mKeypoints[_ind2][idx2].pt;   

  // Get 3D Points
  X1 = mPointClouds[_ind1]->at( p1.y*mWidth + p1.x ).x;
  Y1 = mPointClouds[_ind1]->at( p1.y*mWidth + p1.x ).y;
  Z1 = mPointClouds[_ind1]->at( p1.y*mWidth + p1.x ).z;
  
  X2 = mPointClouds[_ind2]->at( p2.y*mWidth + p2.x ).x;
  Y2 = mPointClouds[_ind2]->at( p2.y*mWidth + p2.x ).y;
  Z2 = mPointClouds[_ind2]->at( p2.y*mWidth + p2.x ).z;
  
  if( X1 != X1 || Y1 != Y1 || Z1 != Z1 ||
      X2 != X2 || Y2 != Y2 || Z2 != Z2 ) {
    return false;
  }

  return true;
}


/**
 * @function findMaxPutativeMatches
 */
void R3D::findMaxPutativeMatches() {

  int maxPutativeMatches;
  int maxInd;

  mMaxMatchesIndices.resize( mNumFrames );
  mMaxMatchesSize.resize( mNumFrames );

  for( int i = 0; i < mNumFrames; ++i ) {
    maxPutativeMatches = 0;
    maxInd = -1;
    for( int j = 0; j < mNumFrames; ++j ) {
      if( i == j ) { continue; }
      if( mMatches[i][j].size() > maxPutativeMatches ) {
	maxInd = j; 
	maxPutativeMatches =  mMatches[i][j].size();
      }
    }     
    mMaxMatchesIndices[i] = maxInd;
    mMaxMatchesSize[i] = maxPutativeMatches;
  }
  
  // Use logic what the heck!
  for( int i = 0; i < mNumFrames; ++i ) {

    mMaxMatchesIndices[i] = i+1;   
    if( i == mNumFrames - 1 ) {
      mMaxMatchesIndices[i] = i - 1;
    }
    mMaxMatchesSize[i] = mMatches[i][ mMaxMatchesIndices[i] ].size();
    
  }
  

  for( int i = 0; i < mNumFrames; ++i ) {
    printf("[%d] Putative match: %d - Num matches: %d \n", i, mMaxMatchesIndices[i], mMaxMatchesSize[i] );
  }
}


/**
 * @function getMaxMatchingDraw
 */
cv::Mat R3D::getMaxMatchingDraw( int _ind  ) {

  //-- Draw matches
  cv::Mat matchesImage;
  int ind2 = mMaxMatchesIndices[_ind];

  cv::drawMatches( mRgbImages[_ind], 
		   mKeypoints[_ind], 
		   mRgbImages[ind2], 
		   mKeypoints[ind2], 
		   mMatches[_ind][ind2], 
		   matchesImage );  
  
  return matchesImage;
}

/**
 * @function getNumMaxMatchesSize
 */
int R3D::getNumMaxMatchesSize( int _ind ) {
  return mMaxMatchesSize[_ind];
}

/**
 * @function getMaxMatchesIndices
 */
int R3D::getMaxMatchesIndices( int _ind ) {
  return mMaxMatchesIndices[_ind];
}

/**
 * @function Ransac_Rigid3D
 */
bool R3D::Ransac_Rigid3D( int _ind1, int _ind2 ) {
  printf("Ransac Rigid 3D \n");
  mN = 3;
  mS = 4;
  int totalSamples = mMatches[_ind1][_ind2].size();
  std::vector<int> randomSampleIndices;
  printf("Model: %d total samples: %d \n", mS, totalSamples );

  for( int i = 0; i < mN; ++i ) {
    randomSampleIndices.resize(0);
    randomSampleIndices = getRandomIndices( mS, totalSamples );

    mParams = get_Model_Rigid3D( _ind1, _ind2, randomSampleIndices  );

  }

}

/**
 * @function get_Model_Rigid3D 
 */
cv::Mat R3D::get_Model_Rigid3D( int _ind1,
				int _ind2, 
				std::vector<int> _indices ) {
 
  cv::Mat param(12, 12, CV_32FC1 );
  float X1, Y1, Z1; float X2, Y2, Z2;

  // Check that there are four params
  if( _indices.size() != 4 ) {
    printf("[X] [geModelRigid3D] What the heck is this? Give me 4 freaking points! Exiting \n");
  }
  else {
  
    for( int i = 0; i < _indices.size(); ++i ) {
      cv::Mat block = cv::Mat::zeros(3, 12, CV_32FC1 );
      get3DPointsFromMatch( _ind1, _ind2, mMatches[_ind1][_ind2][ _indices[i] ],
			    X1, Y1, Z1, X2, Y2, Z2 );
      

    }

  } // end else

  return param;
}

/**
 * @function get3DPointsFromMatch
 */
void R3D::get3DPointsFromMatch( int _ind1, int _ind2, 
				const cv::DMatch &_match,
				float &_X1, float &_Y1, float &_Z1, 
				float &_X2, float &_Y2, float &_Z2 ) {

  int idx1; int idx2;
  cv::Point2f p1; cv::Point2f p2;
  
  // Get both 2D Points from keypoint info
  idx1 = _match.queryIdx;
  idx2 = _match.trainIdx;
  
  p1 = mKeypoints[_ind1][idx1].pt;
  p2 = mKeypoints[_ind2][idx2].pt;
  
  // Get 3D Points
  _X1 = mPointClouds[_ind1]->at( p1.y*mWidth + p1.x ).x;
  _Y1 = mPointClouds[_ind1]->at( p1.y*mWidth + p1.x ).y;
  _Z1 = mPointClouds[_ind1]->at( p1.y*mWidth + p1.x ).z;
  
  _X2 = mPointClouds[_ind2]->at( p2.y*mWidth + p2.x ).x;
  _Y2 = mPointClouds[_ind2]->at( p2.y*mWidth + p2.x ).y;
  _Z2 = mPointClouds[_ind2]->at( p2.y*mWidth + p2.x ).z;
  
}


/**
 * @function getRandomIndices
 */
std::vector<int> R3D::getRandomIndices( int _numSamples, int _totalSamples ) {

  std::vector<int> randomIndices;
  int currentIndice;

  if( _numSamples > _totalSamples ) {
    printf( "[X] [getRandomIndices] Model needs more than available keypoints. Exiting! \n");
  }

  else {
    for( int i = 0; i < _numSamples; ++i ) {  
      do {
	currentIndice = getRandom( _totalSamples );
      } while( isInSet( currentIndice, randomIndices ) == true );
      randomIndices.push_back( currentIndice );
    }
  }

  return randomIndices;
}

/**
 * @function isInSet
 */
bool R3D::isInSet( int _index, 
		      std::vector<int> _currentIndices ) {

  for( int i = 0; i < _currentIndices.size(); ++i ) {
    if( _index == _currentIndices[i] ) {
      return true;
    }
  }
  return false;
}

/**
 * @function getRandom
 */
int R3D::getRandom( int _max ) {
  return( rand() % _max );
}

/**
 * @function getPointCloud
 */
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr R3D::getPointCloud( int _ind ) {
  if( _ind < mNumFrames ) {
    return mPointClouds[_ind];
  }
}

/**
 * @function applyRigid3DToPCD
 */ /*
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr R3D::applyRigid3DToPCD( cv::Mat _m, 
								pcl::PointCloud<pcl::PointXYZRGBA>::Ptr _inputCloud ) {

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr outputCloud;
  //Eigen::Transform<Scalar, 3, Eigen::Affine> a3d;
  
  // Put transformation to Eigen::Affine3f format
  //pcl::transformPointCloud( _inputCloud,
  //			    outputCloud,
  //			    a3d );

  return outputCloud;
}
    */
