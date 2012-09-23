/**
 * @file Utils.cpp
 */
#include "Utils.h"
#include <fstream>
#include <iostream>
#include <stdio.h>

/**
 * @function Reconstruction3D
 */
Reconstruction3D::Reconstruction3D( char* _rgbFilenames, 
				    char* _depthFilenames ) {

  // Read RGB + Depth Images
  if( !readRgbImages( _rgbFilenames, mRgbImages ) || 
      !readDepthImages( _depthFilenames, mDepthImages ) ) {

    printf( "[X] RGB or Depth images not read correctly - ERROR!  \n" );
  }
  else {
    mNumImages = mRgbImages.size();
    printf(" [i] Depth and RGB read \n" );
  }


}

/**
 * @function ~Reconstruction3D
 */
Reconstruction3D::~Reconstruction3D() {
}

/**
 * @function readDepthImages
 */
bool Reconstruction3D::readDepthImages( std::string _depthFilename, 
					std::vector<cv::Mat> &_depthImages ) {
  
  std::vector<std::string> depthFiles;
  
  // Open Stream
  _depthImages.resize(0);
  std::ifstream ifs( _depthFilename.c_str() );
  
  // Read
  std::string temp;
  while( getline( ifs, temp ) ) {
    depthFiles.push_back( temp );
  }
  
  // Load them 
  for( int i = 0; i < depthFiles.size(); ++i ) {
    cv::Mat depthImg = cv::imread( depthFiles[i], 1 );
    if( !depthImg.data ) {
      std::cout<< "[readDepthImages] ["<< i << "] Could not read image " << depthFiles[i] <<" . This cannot be good \n";
      _depthImages.resize(0);
      return false;
    } 
    _depthImages.push_back( depthImg );
  }

  return true;
}

/**
 * @function readRgbImages
 */
bool Reconstruction3D::readRgbImages( std::string _rgbFilename, 
				      std::vector<cv::Mat> &_rgbImages ) {
  
  std::vector<std::string> rgbFiles;
  
  // Open Stream
  _rgbImages.resize(0);
  std::ifstream ifs( _rgbFilename.c_str() );

  // Read
  std::string temp;
  while( getline( ifs, temp ) ) {
    rgbFiles.push_back( temp );
  }

  // Load them 
  for( int i = 0; i < rgbFiles.size(); ++i ) {
    cv::Mat rgbImg = cv::imread( rgbFiles[i], 1 );
    if( !rgbImg.data ) {
      std::cout<< "[readRgbImages] ["<< i << "] Could not read image " << rgbFiles[i] <<" . This cannot be good \n";
      _rgbImages.resize(0);
      return false;
    } 
    _rgbImages.push_back( rgbImg );
  }

  return true;

}

/**
 * @function showRgbImage
 */
bool Reconstruction3D::showRgbImage( std::string _windowName,
				     int _ind ) {
  
  if( _ind < mNumImages ) {
    cv::imshow( _windowName.c_str(), mRgbImages[_ind] ); 
    return true;
  }
  else {
    printf("[!] Index exceed size of matrix vector \n");
    return false;
  }
}

/**
 * @function showDepthImage
 */
bool Reconstruction3D::showDepthImage( std::string _windowName,
				       int _ind ) {
  
  if( _ind < mNumImages ) {
    cv::imshow( _windowName.c_str(), mDepthImages[_ind] ); 
    return true;
  }
  else {
    printf("[!] Index exceed size of matrix vector \n");
    return false;
  }
}


/**
 * @function PCDData
 */
bool Reconstruction3D::getPCDData( std::vector<cv::Mat> _rgbImgs,
				   std::vector<cv::Mat> _depthImgs,
				   std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &_pcdData ) {
  
  // Mapping Depth pixels to color pixels
  
  return true;
}

/**
 * @function depthToColorPixel
 */
void Reconstruction3D::depthToColorPixel( int _x_d, int _y_d, int _depth,
					  float &_P3Dx, float &_P3Dy, float &_P3Dz ) {
  
  // Get depth
  _P3Dz = rawDepthToMeters( _depth );

  _P3Dx = ( _x_d - mCx_d )*_P3Dz / mFx_d;
  _P3Dy = ( _y_d - mCy_d )*_P3Dz / mFy_d;
}


/**
 * @function rawDepthToMeters
 * @brief Taken from ROS Data
 */
float Reconstruction3D::rawDepthToMeters( int _rawDepth ) {

  if( _rawDepth < 2047 ) {
    return 1.0 / (_rawDepth*-0.0030711016 + 3.3309495161);
  }

  return 0;
}
