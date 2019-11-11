#include <ctime>
#include "MultiPointTracker.h"

#include "MatchingAlgorithm.h"

MatchingAlgorithm::MatchingAlgorithm()
{

}


void MatchingAlgorithm::MatchPoints( uint64_t& iFirstMatchTimestep,
                 PixelSet&   ioPixelsToTrack,
                 const TrackerConfigFile&       iConfig,
                 //const ImageSet& iTimestepImages,
                 const std::pair<unsigned long, std::set<unsigned long> >& iCamIndexesToExclude,
                 bool (*CancelFunc)(),
                 std::vector< std::vector< SmtPixel > > iMarkedPoints,
                 bool& oRunSuccessfully

                 //std::vector< std::vector< std::vector< int32_t > > >& oBestFit,
                 //std::vector< double >& oBestFitErrors,
                 //PixelSet& iFirstTimestepWithAllPoints
                                     )
{
//    if (oFirstTimestepWithAllPoints.size() == 0)
//    {
//        std::fprintf(stderr, "\nThere were no timesteps in which all cameras could see all points.\n\nExiting..\n");
//        return;
//    }

    if (!iConfig.allowHiddenPointMarking)
    {
        std::fprintf(stderr, "\nThe first timestep where all the cameras could see all the points together was %lu\n", iFirstMatchTimestep);
    }

    //Use the best fit indexes to order the points according to point0, point1, etc... then create a
    //MultiPointTracker that tracks the path of that point through time.
    std::vector< MultiPointTracker* > trackers;
    for (uint32_t i =0; i < iConfig.numCameras; ++i)
    {
        if (iCamIndexesToExclude.second.count( i ) == 1)
        {
            trackers.push_back( NULL );
            continue;
        }
        MultiPointTracker* mpt = new MultiPointTracker( iConfig.maxNumPoints );
        std::vector< SmtPixel > orderedPoints;
        for (uint32_t idx = 0; idx < iConfig.maxNumPoints; ++idx)
        {
            orderedPoints.push_back( SmtPixel( iMarkedPoints[i][idx], i ) );
        }
        mpt->Initialize( orderedPoints );
        trackers.push_back( mpt );
    }

    for (uint32_t j = 0; j < ioPixelsToTrack.size(); ++j)
    {
        if (j < iFirstMatchTimestep)
        {
            std::vector< std::vector< SmtPixel > > emptyHolderForTimestep;
            ioPixelsToTrack[j] = emptyHolderForTimestep;
            continue;
        }

        std::vector< std::vector< SmtPixel > > timestepOrderedPoints;
        for (uint32_t t=0; t < trackers.size(); ++t)
        {
            if (CancelFunc())
            {
                return;
            }
            if (trackers[t] == NULL)
            {
                continue;
            }
            std::vector< SmtPixel > singleCamOrderedPoints;
            //A timestep can be empty if it doesn't have enough cameras with matching points. "Enough" is
            //determined by the user passed in value.
            if (ioPixelsToTrack[j].size() > 0)
            {
                if (ioPixelsToTrack[j][t].size() != iConfig.maxNumPoints
                        && iConfig.allowHiddenPointMarking)
                {
                    std::vector< SmtPixel > filledInList;
                    if (trackers[t]->FillInMissingPointsFromPredictions( ioPixelsToTrack[j][t],
                                                                         j,
                                                                         filledInList ) )
                    {
                        ioPixelsToTrack[j][t] = filledInList;
                    }
                    else
                    {
                        //Couldn't fill in the missing points, so we remove that tracker/camera from possibility of triangulation later.
                        ioPixelsToTrack[j][t].clear();
                    }
                }

                if (ioPixelsToTrack[j][t].size() == iConfig.maxNumPoints)
                {
                    if (trackers[t]->Next( ioPixelsToTrack[j][t], j, singleCamOrderedPoints ))
                    {
                        timestepOrderedPoints.push_back( singleCamOrderedPoints );
                    }
                }
                else
                {
                    //std::fprintf(stderr, "Should never have gotten here because cameras with not enough points per frame should have already been removed.\n");
                    //assert( false );
                }
            }

        }
        //if( !timestepOrderedPoints.empty() ){
            ioPixelsToTrack[j] = timestepOrderedPoints;
        //}

    }

    for (uint32_t t=0; t < trackers.size(); ++t)
    {
        delete trackers[t];
    }
    oRunSuccessfully = true;
}
