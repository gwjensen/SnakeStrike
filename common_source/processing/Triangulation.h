#ifndef processing_Triangulation_h
#define processing_Triangulation_h

#include <set>
#include <opencv2/core.hpp>

#include "MultiPointTracker.h"
#include "MultiPointSmoother3d.h"
#include "common_types.h" 
#include "image/PixelSet.h"
#include "CamMats.h"

cv::Point2d operator*( cv::Mat_< double > iM, const cv::Point2d& iP);
void CalculateTransformedPosition( const int& iCamIdx,
                                   const std::vector< SmtPixel >& iCandidateEndPoints,
                                   const CamMats& iCamMatrix,
                                   cv::Point3d& oStart,
                                   std::vector< cv::Point3d >& oEnds,
                                   float iExtendLength = 5000 );

/* Gives you an output that has the triangulated pixels sorted into timesteps */
int TriangulatePixelsWithHistory( const PixelSet& iPixelSet,
                                  const CamMats& iCamMats,
                                  std::vector< std::vector< cv::Point3d > >& oCalcdPoints );

/* Gives you an output that has the triangulated pixels sorted into timesteps */
int TriangulatePixels( const PixelSet& iPixelSet, std::vector< std::vector< cv::Point3d> >& oCalcdPoints );

void TriangulatePixelsWithKalman( const PixelSet& iPixelSet,
                                 const CamMats& iCamMats,
                                 const std::pair<int, std::set<int> >& iCamIndexesToExclude,
                                 MultiPointSmoother3d& ioTracker,
                                 const int iNumPoints,
                                 std::vector< std::vector< cv::Point3d > >& oCalcdPoints );

double CorrectTimestepPoints( const std::vector< cv::Mat >& iCamProj,
                              const std::vector< cv::Mat >& iPoints,
                              const std::pair<int, std::set<int> >& iCamIndexesToExclude,
                              std::vector< cv::Mat >& oPoints );
/* Gives you an output that has all the pixels for all timesteps in one vector. */

double CorrectTimestepPoints( const std::vector< cv::Mat >& iCamProj,
                              const std::vector< cv::Mat >& iPoints,
                              std::vector< cv::Mat >& oPoints );

/*int triangulatePixels( const PixelSet& iPixelSet, const std::vector< cv::Mat >& iExtrinsicMats, std::vector< cv::Point3d >& oCalcdPoints );*/

#endif //processing_Triangulation_h
