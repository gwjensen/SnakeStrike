#ifndef TriangulationPipeline_h
#define TriangulationPipeline_h

#include <iostream>
#include <ctime>
#include <string>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/optflow/motempl.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <rapidjson/document.h>
#include <rapidjson/writer.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/error/en.h>
#include <boost/algorithm/string.hpp>

#include "common.h" //common_source
#include "visualization/opencv_viz.h"
#include "distance_3d_seg.h" //opensource
#include "stacktrace.h"  //opensource
#include "image/PixelClusterSet.h"
#include "image/PixelCluster.h"
#include "processing/MultiPointTracker.h"
#include "processing/Correspondence.h"

class TriangulationPipeline
{
    public:
        static TriangulationPipeline* Instance();
        static void CloseInstance();
        void Cleanup();
        void Cancel(); //have to call Initialize after this to put object back in non-cancelled state.
        int Run( const std::string& iConfigFile );
        int Run( const TrackerConfigFile& iConfig );
        static bool Cancelled();
        void Initialize( const TrackerConfigFile& iConfigFile );
        void Undistort();
        void Threshold();
        void ClusterPixels();
        bool FirstMatchingTimestep();
        bool RunUserMatchingHelper();
        bool PointCorrespondence( );
        void Triangulate();
        void GetNewMarkedPointsInfo();

        //void CreateVisualHull(const std::string& iConfigFile);

        std::vector< SmtImage > Images();
        int TimestepUsed();
        std::vector< std::vector< SmtPixel > > MarkersFound();
        std::pair<unsigned long, std::set<unsigned long> > CamsToExclude();
        int NumMarkersToFind();

        void DeleteImages(){             //::TRICKY:: We have to do this in order to make sure the cv::Mat objects get deleted.
            ImageSet delete_vector;
            mTimestepImages.swap(delete_vector);}
        void DeleteThresholdedImages(){             //::TRICKY:: We have to do this in order to make sure the cv::Mat objects get deleted.
            ImageSet delete_vector;
            delete_vector.swap(mBinaryImages);}

    private:
        static TriangulationPipeline* mpsInstance;
        int Factorial(int n);
        static bool CompareContourAreas ( std::vector<cv::Point> contour1, std::vector<cv::Point> contour2 );


        TrackerConfigFile    mConfig;
        ImageSet             mTimestepImages;
        ImageSet            mBinaryImages;
        PixelSet            mPixelsToTrack;
        std::pair<unsigned long, std::set<unsigned long> > mCamIndexesToExcludePair;
        bool                mIsCancelled;
        bool                mIsInit;
        uint64_t            mImagesRead;
        uint64_t            mFirstMatchTimestep;
        double              mImageWidth;
        double              mImageHeight;
        double              mImageBufferLength;
        bool                mSkipUserInput;
        std::vector< std::vector< SmtPixel> > mMarkedPoints;

        TriangulationPipeline();
        ~TriangulationPipeline(); //Only allow instantiation on heap, no stack

        TriangulationPipeline(TriangulationPipeline const&);        // Don't Implement
        void operator=(TriangulationPipeline const&); // Don't implement

};


#endif // TriangulationPipeline_h
