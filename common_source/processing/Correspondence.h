#ifndef processing_Correspondence_h
#define processing_Correspondence_h

#include <opencv2/core.hpp>
#include <map>
#include <set>
#include <utility> //std::pair<,>() 

#include "common_types.h"
#include "CamMats.h"
#include "image/PixelSet.h"
#include "image/ImageSet.h"
#include "image/SmtPixel.h"

//divide and conquer -esque approach to get global minimal matching
//The double value in the std::pair accumulates the total distance
void OrderCorrespondingPointsUsingAffine( const std::vector< SmtPixel >& iCamFixedGroup,
                                            const int iFixedIdx,
                                            const int iFixedCamIdx,
                                            const std::vector< SmtPixel >& iCamUnorderedGroup,
                                            const std::vector< int >& iIndexesLeft,
                                            const CamMats& iCamMatrix,
                                            const int iCamIdx,
                                            const std::pair< double, std::vector< SmtPixel > >&  iCamWorkingGroup,
                                            const std::map< std::string, cv::Mat >& iAffineMats,
                                            std::vector< std::pair< double, std::vector< SmtPixel > > >& oCamOrderedGroups);

// only covers the timesteps
void OrderCorrespondingPointsUsingAffine( const PixelSet& iPixelSet,
                                    const CamMats& iCamMatrix,
                                    const std::vector< std::map< std::string, cv::Mat > >& iAffineMats,
                                    PixelSet& oPixelSet,
                                    ImageSet iImageSet );

//divide and conquer -esque approach to get global minimal matching
//The double value in the std::pair accumulates the total distance
void OrderCorrespondingPoints( const std::vector< SmtPixel >& iCamFixedGroup,
                                            const int iFixedIdx,
                                            const int iFixedCamIdx,
                                            const std::vector< SmtPixel >& iCamUnorderedGroup,
                                            const std::vector< int >& iIndexesLeft,
                                            const CamMats& iCamMatrix,
                                            const int iCamIdx,
                                            const std::pair< double, std::vector< SmtPixel > >&  iCamWorkingGroup,
                                            std::vector< std::pair< double, std::vector< SmtPixel > > >& oCamOrderedGroups);

// only covers the timesteps
void OrderCorrespondingPoints( const PixelSet& iPixelSet,
                               const CamMats& iCamMatrix,
                               PixelSet& oPixelSet,
                               ImageSet iImageSet );


//Greedy ordering, does not order by minimum total distance -- Code could be buggy as it is currently written...
void OrderCorrespondingPointsGreedy( const PixelSet& iPixelSet,
                                     const CamMats& iCamMatrix,
                                     PixelSet& oPixelSet );

void EnumerateIndexCombinations( unsigned long iNumPoints,
                               unsigned long iNumCams,
                               std::vector< std::vector< std::vector< int32_t > > >& oIndexesPerPoint );

void FindCorrespondingPoints( const PixelSet& iPixelSet,
                              std::vector< std::vector< std::vector< int32_t > > >& oBestFit,
                              std::vector< double >& oBestFitErrors );

void GetUserHelpWithCorrespondence( const ImageSet& iRawImages,
                                    const PixelSet& iPixelSet,
                                    bool (*CancelFunc)(),
                                    const std::pair<unsigned long, std::set<unsigned long> >& iCamsToExclude,
                                    const unsigned long iNumPoints,
                                    const std::vector< unsigned long >& iTimesteps,
                                    std::vector< std::vector< std::vector< int32_t > > >& oUserFit,
                                    std::vector< std::vector< cv::Point2d > >& oMarkedPoints,
                                    const bool iAllowHiddenPointMarking);

#endif //processing_Correspondence_h
