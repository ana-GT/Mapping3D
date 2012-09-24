/**
 * @file R3D.cpp
 * @author A. Huaman
 */

// Cool stuff about calibration and getting X,Y from Z
// http://nicolas.burrus.name/index.php/Research/KinectCalibration
// https://groups.google.com/forum/?fromgroups=#!topic/openkinect/AxNRhG_TPHg

#include "R3D.h"
#include <time.h>
#include <Eigen/Core>

/**
 * @function R3D
 * @brief Constructor
 */
R3D::R3D( char *_filenames ) {
  
  // Initialize a few things
  mMinHessian = 400;
  mRadiusFactor = 3.0; // 3.0
  mThresh = 0.1; // m.

  // For random generation
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

  // Get depth images + data
  getDepthImages( mPointClouds,
		  mDepthImages,
		  mDepthData );
}

/**
 * @function ~R3D
 * @brief Destructor
 */
R3D::~R3D() {

}

////////////////////////////// RANSAC ////////////////////////////////////////////

/**
 * @function Ransac_Rigid3D
 */
cv::Mat R3D::Ransac_Rigid3D( int _ind1, int _ind2 ) {

  mN = 20;
  mS = 4;

  int totalSamples;
  std::vector<int> randomSampleIndices;
  cv::Mat params; cv::Mat bestParams;
  std::vector<int> bestInitSet;
  int count; int bestCount; float error;

  totalSamples = mMatches[_ind1][_ind2].size();
  bestCount = 0;
  printf("[Ransac] Model: %d total samples: %d \n", mS, totalSamples );

  for( int i = 0; i < mN; ++i ) {

    randomSampleIndices = getRandomIndices( mS, totalSamples );
    params = get_Model_Rigid3D( _ind1, _ind2, randomSampleIndices  );

    // Calculate inliers / outliers
    count = 0;
    for( int j = 0; j < totalSamples; ++j ) {
      error = getErrorModel( _ind1, _ind2, j, params );
      if( error < mThresh ) {
	count++;
      }
    }
    printf("[%d] Count : %d  Best count: %d \n", i, count, bestCount);
    // Compare with best
    if( count > bestCount ) {
      bestCount = count;
      bestParams = params;
      bestInitSet = randomSampleIndices;
    }
  }
  printf("Here bestParams rows: %d cols: %d \n", bestParams.rows, bestParams.cols );
  std::cout << "Best params: \n" << bestParams << std::endl;

  //return getSomeMatchesDraw( _ind1, _ind2, bestInitSet );
 
  return bestParams;
}

/**
 * @function getErrorModel
 */
float R3D::getErrorModel( int _ind1, int _ind2, int _ind,
			  const cv::Mat &_params ) {

  float X1, Y1, Z1;
  float X2, Y2, Z2;
  float pX2, pY2, pZ2;

  get3DPointsFromMatch( _ind1, _ind2, 
			mMatches[_ind1][_ind2][_ind],
			X1, Y1, Z1, 
			X2, Y2, Z2 );

  // Apply model
  pX2 = _params.at<float>(0,0)*X1 +  _params.at<float>(0,1)*Y1 +
    _params.at<float>(0,2)*Z1 +  _params.at<float>(0,3)*1;
 
  pY2 = _params.at<float>(1,0)*X1 +  _params.at<float>(1,1)*Y1 +
    _params.at<float>(1,2)*Z1 +  _params.at<float>(1,3)*1;

  pZ2 = _params.at<float>(2,0)*X1 +  _params.at<float>(2,1)*Y1 +
    _params.at<float>(2,2)*Z1 +  _params.at<float>(2,3)*1;


  return( sqrt( (X2 - pX2)*(X2-pX2) +  (Y2 - pY2)*(Y2-pY2) +  (Z2 - pZ2)*(Z2-pZ2) ) );
}

/**
 * @function get_Model_Rigid3D 
 */
cv::Mat R3D::get_Model_Rigid3D( int _ind1,
				int _ind2, 
				std::vector<int> _indices ) {
 
  cv::Mat param(12, 1, CV_32FC1 );
  cv::Mat A; cv::Mat B;
  float X1, Y1, Z1; 
  float X2, Y2, Z2;
  
  // Check that there are four params
  if( _indices.size() != 4 ) {
    printf("[X] [geModelRigid3D] What the heck is this? Give me 4 freaking points! Exiting \n");
    return param;
  }

  int ind;
  A = cv::Mat::zeros(12, 12, CV_32FC1 );
  B = cv::Mat::zeros(12, 1, CV_32FC1 );
  
  for( int i = 0; i < _indices.size(); ++i ) {
    get3DPointsFromMatch( _ind1, _ind2, mMatches[_ind1][_ind2][ _indices[i] ],
			  X1, Y1, Z1, X2, Y2, Z2 );
    // Fill A and B
    ind = i*3;
    A.at<float>(ind,0) = X1; A.at<float>(ind,1) = Y1; A.at<float>(ind,2) = Z1; A.at<float>(ind,3) = 1;
    B.at<float>(ind,0) = X2;
    ind = i*3 + 1;
    A.at<float>(ind,4) = X1; A.at<float>(ind,5) = Y1; A.at<float>(ind,6) = Z1; A.at<float>(ind,7) = 1;
    B.at<float>(ind,0) = Y2;
    ind = i*3 + 2;
    A.at<float>(ind,8) = X1; A.at<float>(ind,9) = Y1; A.at<float>(ind,10) = Z1; A.at<float>(ind,11) = 1; 
    B.at<float>(ind,0) = Z2;
  }
  
  svd(A);
  svd.backSubst( B, param );
  
  cv::Mat transf = cv::Mat::zeros( 4, 4, CV_32FC1 );
  ind = 0;
  for( int j = 0; j < 3; ++j ) {
    for( int i = 0; i < 4; ++i ) {
      transf.at<float>(j,i) = param.at<float>(ind, 0);
      ind++;
    }
  }
  // Last row: 0 0 0 1
  transf.at<float>(3,3) = 1.0;
  
  return transf;
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
  int h1, w1; int h2, w2;
  
  // Get both 2D Points from keypoint info
  idx1 = _match.queryIdx;
  idx2 = _match.trainIdx;
  
  p1 = mKeypoints[_ind1][idx1].pt; h1 = (int)p1.y; w1 = (int)p1.x;
  p2 = mKeypoints[_ind2][idx2].pt; h2 = (int)p2.y; w2 = (int)p2.x;
  
  // Get 3D Points
  _X1 = mDepthData[_ind1].at<cv::Vec3f>(h1,w1)[0];
  _Y1 = mDepthData[_ind1].at<cv::Vec3f>(h1,w1)[1];
  _Z1 = mDepthData[_ind1].at<cv::Vec3f>(h1,w1)[2];

  _X2 = mDepthData[_ind2].at<cv::Vec3f>(h2,w2)[0];
  _Y2 = mDepthData[_ind2].at<cv::Vec3f>(h2,w2)[1];
  _Z2 = mDepthData[_ind2].at<cv::Vec3f>(h2,w2)[2];  
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
 * @function applyRigid3DToPCD
 */ 
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr R3D::applyRigid3DToPCD( cv::Mat _m, 
								pcl::PointCloud<pcl::PointXYZRGBA>::Ptr _inputCloud ) {

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr outputCloud( new pcl::PointCloud<pcl::PointXYZRGBA> );
  Eigen::Matrix4f a3d;

  // Pass cv::Mat to Matrix4f
  if( _m.cols != 4 || _m.rows < 3 ) {
    printf("[X] [applyRigid3DToPCD] Error, exiting and output garbage! \n");
  }
  for( int i = 0; i < _m.cols; ++i ) {
    for( int j = 0; j < _m.rows; ++j ) {
      a3d(j,i) = _m.at<float>(j,i);
    }
  }

  if( _m.rows == 3 ) {
    a3d(3,0) = 0; a3d(3,1) = 0; a3d(3,2) = 0; a3d(3,3) = 1;
  } 
  
  // Apply transformation
  pcl::transformPointCloud( *_inputCloud,
  			    *outputCloud,
  			    a3d );

  return outputCloud;
}

/**
 * @function getAllTransforms
 */
void R3D::getAllTransforms() {

  printf( "get All Transforms \n" );

  mLocalTransforms.resize( mNumFrames );
  mGlobalTransforms.resize( mNumFrames );

  cv::Mat tf_local( 4, 4, CV_32FC1 );
  cv::Mat tf_global( 4, 4, CV_32FC1 );
  
  // Take global frame as 0
  tf_local = cv::Mat::eye( 4, 4, CV_32FC1 );
  mLocalTransforms[0] = tf_local.clone();

  // Get following local transforms
  for( int i = 1; i < mNumFrames; ++i ) {
    tf_local = Ransac_Rigid3D( i, i-1 );
    mLocalTransforms[i] = tf_local.clone();
  }

  // Get global transforms

  // Global frame 0 is still identity  
  mGlobalTransforms[0] = mLocalTransforms[0].clone();

  for( int i = 1; i < mNumFrames; ++i ) {
    tf_global = mGlobalTransforms[i-1]*mLocalTransforms[i];
    mGlobalTransforms[i] = tf_global.clone();
  }

}

/**
 * @function getTransformedPointClouds
 */
void R3D::getTransformedPointClouds() {

  mTransformedPointClouds.resize(0);

  for( int i = 0; i < mNumFrames; ++i ) {
     pcl::PointCloud<pcl::PointXYZRGBA>::Ptr transformedPointCloud = applyRigid3DToPCD( mGlobalTransforms[i],mPointClouds[i] );
     mTransformedPointClouds.push_back( transformedPointCloud );
  }

}

///////////////////////////// MATCHING //////////////////////////////////////////////////
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
    printf("<%d> ", i );
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

  float X1; float Y1; float Z1;
  float X2; float Y2; float Z2;
 
  // Get 3D Points
  get3DPointsFromMatch( _ind1, _ind2, _match, 
			X1, Y1, Z1, X2, Y2, Z2 );
  
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
 * @function getSomeMatchesDraw
 */
cv::Mat R3D::getSomeMatchesDraw( int _ind1, int _ind2,
				 const std::vector<int> &matchesIndices ) {

  cv::Mat matchesImage;
  std::vector<cv::DMatch> matches;
  float X1, Y1, Z1, X2, Y2, Z2;
  for( int i = 0; i < matchesIndices.size(); ++i ) {
    matches.push_back( mMatches[_ind1][_ind2][ matchesIndices[i] ] );

    get3DPointsFromMatch( _ind1, _ind2, matches[i],
			  X1, Y1, Z1, X2, Y2, Z2 ); 
    printf("Drawing match between: (%.3f, %.3f, %.3f) and  (%.3f, %.3f, %.3f) \n", X1, Y1, Z1, X2, Y2, Z2);
  }

  cv::drawMatches( mRgbImages[_ind1],
		   mKeypoints[_ind1],
		   mRgbImages[_ind2],
		   mKeypoints[_ind2],
		   matches,
		   matchesImage );
  return matchesImage;
}

/**
 * @function getKeypointsDraw
 */
cv::Mat R3D::getKeypointsDraw( int _ind ) {

  cv::Mat keypointsDraw;
  cv::drawKeypoints( mRgbImages[_ind],  mKeypoints[_ind], keypointsDraw, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT );
  return keypointsDraw;
}
    

//////////////////////////// DATA ACQUISITION ///////////////////////////////////////////

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
  printf("* Pointclouds: Height %d Width: %d \n", mHeight, mWidth );
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
    
    cv::Mat rgbImage = cv::Mat( mHeight, mWidth, CV_8UC3 );
    
    for( int h = 0; h < mHeight; ++h ) {
      for( int w = 0; w < mWidth; ++w ) {
	rgbImage.at<cv::Vec3b>(h,w)[0] = _pcdData[i]->at(h*mWidth + w ).b;
	rgbImage.at<cv::Vec3b>(h,w)[1] = _pcdData[i]->at(h*mWidth + w ).g;
	rgbImage.at<cv::Vec3b>(h,w)[2] = _pcdData[i]->at(h*mWidth + w ).r;
      }
    } 
    _rgbImages.push_back( rgbImage );
  }

  return true;
}

/**
 * @function getDepthImages
 */
bool R3D::getDepthImages( const std::vector< pcl::PointCloud<pcl::PointXYZRGBA>::Ptr > &_pcdData,
			  std::vector<cv::Mat> &_depthImages,
			  std::vector<cv::Mat> &_depthData ) {
  
  _depthImages.resize(0);
  _depthData.resize(0);

  for( int i = 0; i < _pcdData.size(); ++i ) {
    
    cv::Mat depthImage = cv::Mat( mHeight, mWidth, CV_8UC3 );
    cv::Mat depthData = cv::Mat( mHeight, mWidth, CV_32FC3 );

    for( int h = 0; h < mHeight; ++h ) {
      for( int w = 0; w < mWidth; ++w ) {
	depthData.at<cv::Vec3f>(h,w)[0] = _pcdData[i]->at(h*mWidth + w ).x;
	depthData.at<cv::Vec3f>(h,w)[1] = _pcdData[i]->at(h*mWidth + w ).y;
	depthData.at<cv::Vec3f>(h,w)[2] = _pcdData[i]->at(h*mWidth + w ).z;
      }
    }
 
    // Get that into an image
    //-- Get max and min depth
    double minZ, maxZ;
    double minY, maxY, minX, maxX;
    double alpha; float beta;
    std::vector<cv::Mat> depthXYZ;
    cv::split( depthData, depthXYZ );
    
    cv::minMaxLoc( depthXYZ[2], &minZ, &maxZ );
    cv::minMaxLoc( depthXYZ[1], &minY, &maxY );
    cv::minMaxLoc( depthXYZ[0], &minX, &maxX );
    printf("[%d] Min max Z: %.3f  %.3f \n", i, minZ, maxZ );
    printf("[%d] Min max Y: %.3f  %.3f \n", i, minY, maxY );
    printf("[%d] Min max X: %.3f  %.3f \n", i, minX, maxX );

    alpha = 255 / (maxZ - minZ);
    beta = - (255*minZ) / (maxZ - minZ);

    // Get the image
    uchar val; float valF;
    for( int h = 0; h < mHeight; ++h ) {
      for( int w = 0; w < mWidth; ++w ) {
	valF = depthXYZ[2].at<float>(h,w);
	val = (uchar) ( valF* alpha + beta );
	depthImage.at<cv::Vec3b>(h,w)[0] = val;
	depthImage.at<cv::Vec3b>(h,w)[1] = val;
	depthImage.at<cv::Vec3b>(h,w)[2] = val;
	if( valF != valF ) { // NaN
	  depthImage.at<cv::Vec3b>(h,w)[0] = 255;
	  depthImage.at<cv::Vec3b>(h,w)[1] = 0;
	  depthImage.at<cv::Vec3b>(h,w)[2] = 255;
	}

      }
    }

    _depthData.push_back( depthData );
    _depthImages.push_back( depthImage );
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

////////////////////////// GETTERS /////////////////////////////////////

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
 * @function getPointCloud
 */
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr R3D::getPointCloud( int _ind ) {
  if( _ind < mNumFrames ) {
    return mPointClouds[_ind];
  }
}

/**
 * @function getTransformedPointCloud
 */
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr R3D::getTransformedPointCloud( int _ind ) {

  if( _ind < mNumFrames ) {
    return mTransformedPointClouds[_ind];
  }
}
