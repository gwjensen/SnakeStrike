#include <opencv2/opencv.hpp>
#include <opencv2/video/video.hpp>

#include "CamMats.h"
#include "PixelSet.h"
#include "ThreadPool.h"

#include "ImageSet.h"

//MOG2 Background subtractor vector for each camera //static so it remains a file variable and not global.
static 	std::vector< cv::Ptr< cv::BackgroundSubtractor > > gMOG2List;
const uint32_t NUM_TIMESTEPS_PER_THREAD = 100;
const uint32_t NUM_THREADS_IN_POOL = 15;

//void BackgroundSubtraction( const int iCamIdx )
//{
//    BackgroundSubtraction(iCamIdx, *this);
//}


void BackgroundSubtraction( const cv::Mat& iImage, const uint32_t iCamIdx, cv::Mat& oMask  )
{
    //This function needs to use a global background subtractor because it uses
    //sequential image information to subtract the background.

    if (gMOG2List.size() < iCamIdx + 1)
    {
        for (uint32_t i = ::gMOG2List.size(); i < iCamIdx + 1; ++i)
        {
            //adding all subtractors between this cam number and the current cam number
            //because we will need these subtractors at some point.
            gMOG2List.push_back( cv::createBackgroundSubtractorMOG2() ); //MOG2 create Background Subtractor objects
        }
    }

    gMOG2List[iCamIdx]->apply( iImage, oMask );

    //std::fprintf( stderr,
    //              "The size of the mask is rows:%d, cols:%d, chan:%d\n", oMask.rows, oMask.cols, oMask.channels() );
    //std::fprintf( stderr,
    //              "The size of input image rows:%d, cols:%d, chan:%d\n",
    //              iImage.rows, iImage.cols, iImage.channels() );
    if (oMask.type() != iImage.type())
    {
        //cv::Mat newMask( iImage.rows, iImage.cols, iImage.type() );

        assert( iImage.channels() == 3 );
        std::vector<cv::Mat> newMask = { oMask, oMask, oMask };
        cv::Mat tmp3Chan = cv::Mat::zeros( cv::Size( iImage.cols, iImage.rows ), iImage.type() );
        cv::merge( newMask, tmp3Chan ); //possible bug in opencv code, so I have to use this weird call as workaround

        oMask = tmp3Chan.clone();
    }
}

void ThresholdImageTimestepGroup(const uint32_t iStartTimestep,
                                 const uint32_t iEndTimestep,
                                 const ImageSet& iImagesSet,
                                 ImageSet& oImagesSet,
                                 bool (*CancelFunc)(),
                                 const cv::Scalar& iLowerb,
                                 const cv::Scalar& iUpperb,
                                 const cv::Scalar iNoiseThresholdInfo)
{
    for (uint32_t j = iStartTimestep; j < iEndTimestep; ++j)
    {
        std::vector< SmtImage > timestep_images;
        for (uint32_t i = 0; i < iImagesSet[j].size(); ++i)
        { //camera #
            SmtImage new_image( iImagesSet[j][i] );
            new_image.Threshold( iLowerb, iUpperb, iNoiseThresholdInfo );
            timestep_images.push_back( new_image );
        }
        oImagesSet[j] = timestep_images;
        if (CancelFunc())
        {
            std::fprintf(stdout, "Cancelling thresholding...\n");
            //oImagesSet.clear();

            //::TRICKY:: We have to do this in order to make sure the cv::Mat objects get deleted.
            ImageSet delete_vector;
            delete_vector.swap(oImagesSet);

            return;
        }
    }
}

void ThresholdImages( const ImageSet& iImagesSet,
                      ImageSet& oImagesSet,
                      bool (*CancelFunc)(),
                      const cv::Scalar& iLowerb,
                      const cv::Scalar& iUpperb,
                      const int iNoiseFilterSize,
                      const int iNoiseIterations,
                      const int iNoiseThreshold)
{
    //oImagesSet.clear();
    //::TRICKY:: We have to do this in order to make sure the cv::Mat objects get deleted.
    ImageSet delete_vector;
    delete_vector.swap(oImagesSet);

    oImagesSet.resize( iImagesSet.size() );
    ThreadPool pool( NUM_THREADS_IN_POOL );
    uint32_t num_timesteps_per_thread = NUM_TIMESTEPS_PER_THREAD;
    for( uint32_t j = 0; j < iImagesSet.size() ; ){ //for each timestep
        if( iImagesSet.size() - j < num_timesteps_per_thread ){
            num_timesteps_per_thread = iImagesSet.size() - j;
        }
        pool.enqueue( boost::bind( ThresholdImageTimestepGroup,
                                   j,
                                   j + num_timesteps_per_thread,
                                   boost::ref( iImagesSet ),
                                   boost::ref( oImagesSet ),
                                   CancelFunc,
                                   boost::ref( iLowerb ),
                                   boost::ref( iUpperb ),
                                   cv::Scalar( iNoiseFilterSize, iNoiseIterations, iNoiseThreshold) ) );
        j += num_timesteps_per_thread;
    }
}

//void thresholdImages( ImageSet& ioImagesSet, const cv::Scalar& iLowerb, const cv::Scalar& iUpperb ){
//	for( int j = 0; j < ioImagesSet.size(); ++j ){ //timestep
//		for( int i = 0; i < ioImagesSet[j].size(); ++i ){ //camera #
//			cv::Mat new_image;
//			thresholdImage( ioImagesSet[j][i], new_image, iLowerb, iUpperb );
//			ioImagesSet[j][i].replaceImage( new_image );
//		}
//	}
//}
void MergeThresholdHalvesTimestepGroup(uint32_t iStartTimestep,
                                        uint32_t iEndTimestep,
                                        const ImageSet& iImageSet1,
                                        const ImageSet& iImageSet2,
                                        ImageSet& oImageSet)
{
    for (uint32_t j = iStartTimestep; j < iEndTimestep; ++j)
    {
        std::vector< SmtImage > current_timestep;
        for (uint32_t i =0; i < iImageSet1[j].size(); ++i)
        {
            cv::Mat tmp_mat;
            assert( iImageSet1[j][i].RGB() == iImageSet2[j][i].RGB());
            cv::addWeighted( iImageSet1[j][i], 1.0, iImageSet2[j][i], 1.0, 0.0, tmp_mat );
            current_timestep.push_back( SmtImage( iImageSet1[j][i], tmp_mat, iImageSet1[j][i].RGB() ) );
        }
        oImageSet[j] = current_timestep;
    }
}

void MergeThresholdHalves( const ImageSet& iImageSet1, const ImageSet& iImageSet2, ImageSet& oImageSet )
{
    //oImageSet.clear();
    //::TRICKY:: We have to do this in order to make sure the cv::Mat objects get deleted.
    ImageSet delete_vector;
    delete_vector.swap(oImageSet);

    oImageSet.resize( iImageSet1.size() );
    assert( iImageSet1.size() == iImageSet2.size() );

    ThreadPool pool( NUM_THREADS_IN_POOL );
    uint32_t num_timesteps_per_thread = NUM_TIMESTEPS_PER_THREAD;
    for (uint32_t j = 0; j < iImageSet1.size() ; )
    { //for each timestep
        if (iImageSet1.size() - j < num_timesteps_per_thread)
        {
            num_timesteps_per_thread = iImageSet1.size() - j;
        }
        pool.enqueue( boost::bind( MergeThresholdHalvesTimestepGroup,
                                   j,
                                   j + num_timesteps_per_thread,
                                   boost::ref( iImageSet1 ),
                                   boost::ref( iImageSet2 ),
                                   boost::ref( oImageSet ) ));
        j += num_timesteps_per_thread;
    }
}

void UndistortImageTimestepGroup( uint32_t iStartTimestep,
                                  uint32_t iEndTimestep,
                                  ImageSet& ioImagesSet,
                                  bool (*CancelFunc)(),
                                  const std::pair<unsigned long, std::set<unsigned long> >& iCamIndexesToExclude,
                                  const std::string& iWriteLocation )
{

    for (uint32_t j= iStartTimestep; j < iEndTimestep; ++j)
    {
        for (uint32_t i = 0; i < ioImagesSet[j].size(); ++i)
        { //for each camera that had an image at the timestep
            if (CancelFunc())
            {
                std::fprintf(stdout, "Cancelling during undistortion, some images could have already been undistorted...\n");
                return;
            }
            if (1 == iCamIndexesToExclude.second.count( i ))
            {
                continue;
            }
//            cv::Mat image_out;
//            int camIdx = ioImagesSet[j][i].Cam();
//            undistort( ioImagesSet[j][i], image_out, cam_matrices->K()[camIdx], cam_matrices->Distortion()[camIdx] );

//            //Note: only image is changed, none of the attributes.
//            ioImagesSet[j][i] = SmtImage(ioImagesSet[j][i], image_out, ioImagesSet[j][i].RBG());
            ioImagesSet[j][i].Undistort();

            if (!iWriteLocation.empty())
            {
                std::stringstream stringStream;
                stringStream << iWriteLocation << "/" << ioImagesSet[j][i].Cam() << "-" << j << "." << ioImagesSet[j][i].FileSuffix();
                bool written = ioImagesSet[j][i].Save( stringStream.str() );

                if (!written)
                {
                    std::fprintf( stderr,
                                  "Could not write the undistorted image to '%s'\n",
                                  ( stringStream.str() ).c_str() );
                    continue;
                }
            }
        }
    }
}

void UndistortImages( ImageSet& ioImagesSet,
                      bool (*CancelFunc)(),
                      const std::pair<unsigned long, std::set<unsigned long> >& iCamIndexesToExclude,
                      const std::string& iWriteLocation  )
{
    //Need to create a list of images for the platform and undistort them according to camera cv::Matrices.
    // images[camera][image locations]

    ThreadPool pool( NUM_THREADS_IN_POOL );
    uint32_t num_timesteps_per_thread = NUM_TIMESTEPS_PER_THREAD;
    for (uint32_t j = 0; j < ioImagesSet.size() ; )
    { //for each timestep
        if (ioImagesSet.size() - j < num_timesteps_per_thread)
        {
            num_timesteps_per_thread = ioImagesSet.size() - j;
        }
        pool.enqueue( boost::bind( UndistortImageTimestepGroup,
                                   j,
                                   j + num_timesteps_per_thread,
                                   boost::ref(ioImagesSet),
                                   CancelFunc,
                                   boost::ref(iCamIndexesToExclude),
                                   boost::ref(iWriteLocation) ));
        j += num_timesteps_per_thread;
    }
}

void GetInterestingPixels( const ImageSet& iImageSet, PixelSet& oPixelSet )
{
    //This function goes through the input ImageSet and finds the pixels that are not 0

    ///@TODO Not sure if the threshold function will be good enough for this to work as I expect.

    for (uint32_t j = 0; j < iImageSet.size(); ++j)
    {
        std::vector<std::vector< SmtPixel > > interest_timestep_pixels;
        for (uint32_t i = 0; i < iImageSet[j].size(); ++i)
        {
            std::vector< SmtPixel > interest_pixels;
            iImageSet[j][i].GetInterestingPixels( interest_pixels );
            interest_timestep_pixels.push_back( interest_pixels );
        }
        oPixelSet.push_back( interest_timestep_pixels );
    }

}

void GetPixelClustersGroupTimestep( int iStartTimestep,
                                    int iEndTimestep,
                                    const ImageSet& iImageSet,
                                    bool (*CancelFunc)(),
                                    const int iMaxNumClusters,
                                    const std::pair<unsigned long, std::set<unsigned long> >& iCamIndexesToExclude,
                                    PixelClusterSet& oClusterSet )
{
    for (int j = iStartTimestep; j < iEndTimestep; ++j)
    {
        std::vector< std::vector<PixelCluster> > timestep_clusters( iCamIndexesToExclude.first );
        for (uint32_t i = 0; i < iImageSet[j].size(); ++i)
        {            
            std::vector<PixelCluster> clusters;
            unsigned long cam_idx = iImageSet[j][i].Cam();

            if (1 == iCamIndexesToExclude.second.count( cam_idx ))
            {
                timestep_clusters[ cam_idx ] = clusters;
                continue;
            }
            iImageSet[j][i].GetPixelClusters( iMaxNumClusters, clusters );
            if (clusters.size() > 0)
            {
                timestep_clusters[cam_idx] =  clusters;
            }
            else
            {
                std::fprintf(stderr, "No pixel clusters in timestep %d, image %d.\n", j, cam_idx);
            }
        }
        oClusterSet[j] = timestep_clusters;
        if (CancelFunc())
        {
            std::fprintf(stdout, "Cancelling pixel cluster collection...\n");
            {
                PixelClusterSet delete_vector;
                delete_vector.swap(oClusterSet);
            }
            oClusterSet.clear();
            return;
        }
    }
}

void GetPixelClusters( const ImageSet& iImagesSet,
                       bool (*CancelFunc)(),
                       const std::pair<unsigned long, std::set<unsigned long> >& iCamIndexesToExclude,
                       const int iMaxNumClusters,
                       PixelClusterSet& oClusterSet )
{

    oClusterSet.clear();
    oClusterSet.resize( iImagesSet.size() );
    ThreadPool pool( 1/*NUM_THREADS_IN_POOL*/ );
    uint32_t num_timesteps_per_thread = NUM_TIMESTEPS_PER_THREAD;
//    std::cout << "Showing image" << std::endl;
//    cv::namedWindow( "GetPixelClusters", CV_WINDOW_AUTOSIZE | CV_GUI_NORMAL | CV_WINDOW_KEEPRATIO);
//    cv::imshow("GetPixelClusters", iImagesSet[0][0] );
//    cv::setWindowProperty("GetFocus", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
//    cv::waitKeyEx(1);
//    cv::setWindowProperty("GetFocus", CV_WND_PROP_FULLSCREEN, CV_WINDOW_NORMAL);
//    cv::waitKeyEx(0);
    for( uint32_t j = 0; j < iImagesSet.size() ; )
    { //for each timestep
        if( iImagesSet.size() - j < num_timesteps_per_thread )
        {
            num_timesteps_per_thread = iImagesSet.size() - j;
        }
        pool.enqueue( boost::bind( GetPixelClustersGroupTimestep,
                                   j,
                                   j + num_timesteps_per_thread,
                                   boost::ref( iImagesSet ),
                                   CancelFunc,
                                   iMaxNumClusters,
                                   boost::ref( iCamIndexesToExclude ),
                                   boost::ref( oClusterSet )  ));
        j += num_timesteps_per_thread;
    }


    //------

}
