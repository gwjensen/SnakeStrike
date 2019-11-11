#ifndef io_File_h
#define io_File_h

#include <set>

#include <opencv2/core.hpp>

#include "common_types.h" 
#include "image/ImageSet.h"
#include "processing/CamMats.h"

//Fixes a problem where we get this weird error with type not being a member in some opencv code
#define OPENCV_TRAITS_ENABLE_DEPRECATED

void GetCameraMatricesFromFile( const std::string& iPathToInputFile, CamMats& iCamMatrices
                                     /*vector<Mat>& oIntrinsicMat,
                                     vector<Mat>& oExtrinsicMat,
                                     vector<Mat>& oRotationMat,
                                     vector<Point3d>& oTranslationMat,
                                     vector<Mat>& oDistortionMat,
                                     vector<Mat>& oInvProjectiveMat*/
                                     );

double CalcRotatedImageBufferSize( const cv::Mat& iImage );

uint64_t ReadImages( const TrackerConfigFile& iConfig,
                     ImageSet& oTimestepImages,
                      bool (*CancelFunc)());

void ReadImages( const std::string& iFilename,
                 const TrackerConfigFile& iConfig,
                 const uint64_t iTotalFrames,
                 const int iImageWidth,
                 const int iImageHeight,
                 const int iCamNum,
                 std::vector<SmtImage>& oTimestepImages,
                 bool (*CancelFunc)() );

void ReadXMLImages( const std::string& iLocation,
                        const std::string& iBaseDir,
                        std::vector< std::string >& oImageLocations,
                        uint32_t& oNumCams,
                        uint64_t& oNumImagesFound );

void WriteTriangulationInfoToFile( cv::FileStorage& iFs,
                                   const std::string& iName,
                                   std::vector< std::vector< cv::Point3d > > & iList,
                                   const int iMaxNumPoints = 0);

std::vector< std::vector< cv::Point3d > > ReadTriangulationInfoToFile( const cv::FileNode& iFn );

void ProcessConfigFile( const std::string& iFileName, TrackerConfigFile& oConfigStruct );

void WriteConfig( const std::string& iFileName, TrackerConfigFile& iConfigStruct );


void WriteMarkedPointsToFile( const std::string& iFilename/*should be the image file name*/,
                              const std::vector< std::vector< cv::Point2d > >& iMarkedPoints,
                              const std::vector< std::vector< std::vector< int32_t > > >& iBestFit);

void ReadMarkedPointsFromFile( const std::string& iFilename,
                               std::vector< std::vector< SmtPixel > >& oMarkedPoints,
                               std::vector< std::vector< std::vector< int32_t > > >& oBestFit);

std::string GetFileExt(const std::string& iPath);

void PopulateImagesFromVideos( std::vector< std::string > iVideoFileNames,
                               const TrackerConfigFile& iConfig,
                              ImageSet& oTimestepImages,
                              bool (*CancelFunc)() );

std::string CreateVideoFilename(const unsigned int& iCamNum,
                                const unsigned int& iImageWidth,
                                const unsigned int& iImageHeight,
                                const uint64_t& iTotalFrames,
                                const float& iOrigCamHz,
                                const float& iVideoPlaybackHz);

bool ParseVideoFilename( const std::string& iFilename,
                         unsigned int& oCamNum,
                         unsigned int& oImageWidth,
                         unsigned int& oImageHeight,
                         uint64_t& oTotalFrames,
                         float& oOrigCamHz,
                         float& oVideoPlaybackHz,
                         std::string& oFileType);
	
#endif //io_File_h
