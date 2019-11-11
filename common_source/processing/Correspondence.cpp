#include <math.h>
#include <ctime>
#include <map>
#include <list>
#include <chrono>
#include <thread>
#include <utility> //std::pair<,>()
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

#include "distance_3d_seg.h"
#include "common_types.h"
#include "Triangulation.h"
#include "CamMats.h"
#include "SmtPixel.h"
#include "PixelSet.h"
#include "ImageSet.h"

#include "Correspondence.h"


//divide and conquer -esque approach to get global minimal matching
//The double value in the std::pair accumulates the total distance
void OrderCorrespondingPointsUsingAffine( const std::vector< SmtPixel >& iCamFixedGroup,
                                            const unsigned long iFixedIdx,
                                            const unsigned long iFixedCamIdx,
                                            const std::vector< SmtPixel >& iCamUnorderedGroup,
                                            const std::vector< unsigned long >& iIndexesLeft,
                                            const CamMats& iCamMatrix,
                                            const unsigned long iCamIdx,
                                            const std::pair< double, std::vector< SmtPixel > >&  iCamWorkingGroup,
                                            const std::map< std::string, cv::Mat >& iAffineMats,
                                            std::vector<std::pair< double, std::vector< SmtPixel > > >& oCamOrderedGroups )
{

    //if (iIndexesLeft.size() == 1)
    //{
    //	oCamOrderedGroup.push_back( iCamUnorderedGroup[ iIndexesLeft[0] );
    //	return true;
    //}
    //std::fprintf( stderr,
    //              "\n iIndexesLeft.size():%ld, iFixedIdx:%d,  iCamWorkingGroup.size():%ld, iCamUnorderedGroup.size():%ld\n",
    //              iIndexesLeft.size(), iFixedIdx, iCamWorkingGroup.second.size(),  iCamUnorderedGroup.size() );
    if (0 == iIndexesLeft.size())
    {
        if (iCamWorkingGroup.second.size() == iCamUnorderedGroup.size())
        {
            oCamOrderedGroups.push_back( iCamWorkingGroup );
        }
        else
        {
            std::fprintf( stderr, "ERROR: orderCorrespondingPoints:: no indexes left to check.\n" );
            std::fprintf( stderr,
                          "ERROR: orderCorrespondingPoints:: group size: %lu, expected: %lu \n",
                          iCamWorkingGroup.second.size(),
                          iCamUnorderedGroup.size() );
            assert( false );
        }
        return;
    }
    else if (iFixedIdx >= iCamUnorderedGroup.size())
    {
        std::fprintf( stderr, "Major Problem in set size during recursion!!!" );
        assert( false );
        return;
    }
    else
    {
        //std::vector< SmtPixel > orderedCamPixels;

        //Get point info for fixed anchor point and camera
        //std::vector< SmtPixel > endPoints = iCamFixedGroup;
        //cv::Point3d startFixedPoint;


        //Get point infor the the other camera and the rays from that camera.
        std::vector< SmtPixel > end_points_left;
        for (unsigned long i = 0; i < iIndexesLeft.size(); ++ i)
        {
            end_points_left.push_back( iCamUnorderedGroup[ iIndexesLeft[ i ] ] );
        }

        //std::vector<SmtPixel>::const_iterator candPointIter = endPointsLeft.begin();
        std::vector< double > distances;

        for (unsigned long idx =0; idx < end_points_left.size(); ++idx)
        {
            unsigned long camIdx1 =  iCamFixedGroup[iFixedIdx].Cam();
            unsigned long camIdx2 = end_points_left[idx].Cam();
            std::stringstream str;
            str << camIdx1 << "-" << camIdx2;
            //cv::Mat affine_mat;

            cv::Mat_< double > transPointMat = cv::Mat( end_points_left[idx] );

            cv::Point2d transPoint = cv::Point2d( transPointMat.at< double >( 0, 0 ),
                                                  transPointMat.at< double >( 1, 0 ) );//at(row,col)

            double dist = opensource::getShortestDistance( iCamFixedGroup[iFixedIdx], transPoint );
            //std::fprintf( stderr,
            //              "The distance between (%lf, %lf) <-->(%lf, %lf) is %lf. idx:%d, fixedidx:%d\n",
            //              iCamFixedGroup[iFixedIdx].x,
            //              iCamFixedGroup[iFixedIdx].y,
            //              transPoint.x,
            //              transPoint.y,
            //              dist, idx, iFixedIdx );
            distances.push_back( dist );
        }
        //std::fprintf( stderr,"\n" );
        //std::fprintf( stderr,
        //              "\n iIndexesLeft.size():%ld, endPointsLeft.size():%ld, distances.size():%ld, iFixedIdx:%d, iCamUnorderedGroup.size():%ld\n",
        //              iIndexesLeft.size(),
        //              endPointsLeft.size(),
        //              distances.size(),
        //              iFixedIdx,
        //              iCamUnorderedGroup.size() );
        for (unsigned long idx = 0; idx < distances.size(); ++idx )
        {
            std::vector< unsigned long > tmp_indexes_left = iIndexesLeft;
            std::pair< double, std::vector<SmtPixel> >  tmp_cam_working_group = iCamWorkingGroup;

            if (0 == tmp_cam_working_group.second.size())
            {
                tmp_cam_working_group.first = 0;
            }

            tmp_cam_working_group.first += distances[idx];

            tmp_cam_working_group.second.push_back( iCamUnorderedGroup[ tmp_indexes_left[idx] ] );

            tmp_indexes_left.erase( tmp_indexes_left.begin() + idx );

            //std::fprintf( stderr,
            //              "Calling orderCorresPoints with indexesLeft:%ld,
            //              iFixedIdx:%d \n",
            //              tmpIndexesLeft.size(),
            //              iFixedIdx + 1 );
            OrderCorrespondingPointsUsingAffine( iCamFixedGroup,
                                                 iFixedIdx + 1,
                                                 iFixedCamIdx,
                                                 iCamUnorderedGroup,
                                                 tmp_indexes_left,
                                                 iCamMatrix,
                                                 iCamIdx,
                                                 tmp_cam_working_group,
                                                 iAffineMats,
                                                 oCamOrderedGroups );

        }
    }
}


// only covers the timesteps
void OrderCorrespondingPointsUsingAffine( const PixelSet& iPixelSet,
                                    const CamMats& iCamMatrix,
                                    const std::vector< std::map< std::string, cv::Mat> >& iAffineMats,
                                    PixelSet& oPixelSet,
                                    ImageSet iImageSet )
{

    if (iImageSet.empty())
    {
        //put this here to avoid function signature change
    }
    for (unsigned long j = 0; j < iPixelSet.size(); ++j)
    {
        std::vector< std::vector< SmtPixel > > timestep_sets;

        if (0 == iPixelSet[j].size())
        {
            oPixelSet.push_back( timestep_sets );
            continue;
        }
        std::vector< SmtPixel > fixed_group = iPixelSet[j][0];

        if (0 == fixed_group.size())
        {
            continue;
        }

        timestep_sets.push_back( fixed_group );

        unsigned long i, idx;
        for (i = 1, idx = 0; i < iPixelSet[j].size();++idx, ++i)
        {
            std::vector< unsigned long > indexes_not_sorted( iPixelSet[j][i].size() );
            generate( indexes_not_sorted.begin(), indexes_not_sorted.end(), IndexFiller() );

            std::vector< std::pair< double, std::vector< SmtPixel > > > cam_ordered_groups;
            std::pair< double, std::vector< SmtPixel > >  cam_working_group;
            if (iPixelSet[j][i].size() > 0)
            {
                OrderCorrespondingPointsUsingAffine( fixed_group,
                                                     0, /* should always be zero since we're starting a new set*/
                                                     fixed_group[0].Cam(),
                                                     iPixelSet[j][i],
                                                     indexes_not_sorted,
                                                     iCamMatrix,
                                                     iPixelSet[j][i][0].Cam(),
                                                     cam_working_group,
                                                     iAffineMats[j],
                                                     cam_ordered_groups );

                double min_value = std::numeric_limits<double>::infinity();
                std::vector< SmtPixel > ordered_set;

                //std::vector<std::pair<double,std::vector<SmtPixel> > >::iterator outputIter = camOrderedGroups.begin();
                //std::fprintf( stderr, "Calculating points with min distances...\n" );
                //std::fprintf( stderr, "          fixed set:[ " );
                //for (unsigned long i = 0; i < fixedGroup.size(); ++i)
                //{
                //	std::fprintf( stderr, "(%lf,%lf) ", fixedGroup[i].x, fixedGroup[i].y );
                //}
                //std::fprintf( stderr, " ]\n" );
                //std::fprintf( stderr, "    %ld possibilities, expected 24\n", camOrderedGroups.size() );
                for (unsigned long tmpIdx = 0; tmpIdx < cam_ordered_groups.size(); ++tmpIdx)
                {
                    if (cam_ordered_groups[tmpIdx].first < min_value)
                    {
                        //std::fprintf( stderr, " ****New Min Distance (%lf) set:[ ", camOrderedGroups[tmpIdx].first);
                        min_value = cam_ordered_groups[tmpIdx].first;
                        ordered_set = cam_ordered_groups[tmpIdx].second;
                        //for (unsigned long i = 0; i < orderedSet.size(); ++i)
                        //{
                        //	std::fprintf( stderr, "(%lf,%lf) ", orderedSet[i].x, orderedSet[i].y );
                        //}
                        //std::fprintf( stderr, " ]\n" );

                        //ViewCorrespondence( fixedGroup,
                        //                    camOrderedGroups[tmpIdx].second,
                        //                    iImageSet[j][fixedGroup[0].cam],
                        //                    iImageSet[j][camOrderedGroups[0].second[0].cam],
                        //                    "**New Min Distance**" );
                    }
                    else
                    {
                        //std::fprintf( stderr, "          distance (%lf) set:[ ", camOrderedGroups[tmpIdx].first );
                        //for (unsigned long i = 0; i < camOrderedGroups[tmpIdx].second.size(); ++i)
                        //{
                        //	std::fprintf( stderr,
                        //                "(%lf,%lf) ",
                        //                camOrderedGroups[tmpIdx].second[i].x,
                        //                camOrderedGroups[tmpIdx].second[i].y );
                        //}
                        //std::fprintf( stderr, " ]\n" );
                        //ViewCorrespondence( fixedGroup,
                        //                    camOrderedGroups[tmpIdx].second,
                        //                    iImageSet[j][fixedGroup[0].cam],
                        //                    iImageSet[j][camOrderedGroups[0].second[0].cam],
                        //                    "Candidate, but not min" );
                    }
                }
                timestep_sets.push_back( ordered_set );
            }
            else
            {
                //camera didn't have any points so we aren't including it.
            }

            //viewCorrespondence(fixedGroup,
            //                   orderedSet,
            //                   iImageSet[j][fixedGroup[0].cam],
            //                   iImageSet[j][camOrderedGroups[0].second[0].cam],
            //                   "Winner of this set." );

            //if( orderedSet.size() == 0 ){
            //	std::fprintf( stderr, "Should NEVER get here\n" );
            //    assert( false );
            //}
            //else{
                //timestep_sets.push_back( orderedSet );
            //}
        }

        oPixelSet.push_back( timestep_sets );
    }
}

void FindCorrespondingPoints( const PixelSet& iPixelSet,
                              std::vector< std::vector< std::vector< int32_t > > >& oBestFit,
                              std::vector< double >& oBestFitErrors )
{
    CamMats* cam_matrices = CamMats::Instance();
    if (!cam_matrices->isInit())
    {
        assert("Must initialize CamMats object before use.");
    }

    std::vector< std::vector< std::vector< std::vector< int32_t > > > > possible_combinations;
    time_t start = std::time( nullptr );
    for (unsigned long i = 0; i < 6; ++i)
    {
        std::vector< std::vector< std::vector< int32_t > > > indexes_per_point;
        EnumerateIndexCombinations( i/*numPoints*/, iPixelSet[0].size()/* 4numCams*/,  indexes_per_point );
        possible_combinations.push_back( indexes_per_point );
    }
    time_t stop = std::time( nullptr );
    std::fprintf( stderr, "%lu seconds to run all combos.\n", stop -start );

    oBestFit.resize( iPixelSet.size() );
    oBestFitErrors.resize( iPixelSet.size() );
    for (unsigned long j=0; j < iPixelSet.size(); ++j)
    {
        oBestFitErrors[j] = std::numeric_limits<double>::max();
        unsigned long participating_cams = 0;
        unsigned long num_points = 0;
        for (unsigned long i = 0; i < iPixelSet[j].size(); ++i)
        {
            if (iPixelSet[j][i].size() > 0)
            {
                ++participating_cams;

                 //only cameras with matching number of points will show up as non-zero
                if (0== num_points)
                {
                    num_points = iPixelSet[j][i].size();
                }
            }
        }
        if (iPixelSet[j].size() > 1 && participating_cams > 1)
        {
            for (unsigned long p=0; p < possible_combinations[num_points].size(); ++p)
            {
                std::vector< cv::Mat > cameras_in_timestep;
                //int totalPointsInAllImages = 0;
                std::vector< cv::Mat > keypoints;
                unsigned long index = 0;
                //unsigned long prevIndex = -1;
                for (unsigned long i = 0; i < iPixelSet[j].size(); ++i)
                {
                    if (iPixelSet[j][i].size() > 0)
                    {
                        cv::Mat tmp( 2, num_points, CV_64FC1 ); //64 expected by sfm, convert here to save time later.

                        for ( int x = 0 ; x < tmp.cols; ++x)
                        {
                            tmp.at<double>(0, x) = iPixelSet[j][i][possible_combinations[num_points][p][x][i]].x ;
                            tmp.at<double>(1, x) = iPixelSet[j][i][possible_combinations[num_points][p][x][i]].y ;
                        }

                        keypoints.push_back( tmp );
                        index = iPixelSet[j][i][0].Cam();
                        //std::fprintf( stderr, "-------adding camera %d matrix to list.\n", index );
                        //totalPointsInAllImages += iPixelSet[j][i].size();

                        cameras_in_timestep.push_back( cam_matrices->M()[index] );
                        //prevIndex = index;
                    }
                    else{
                        std::fprintf( stderr, "Camera didn't have enough points to participate in this timestep\n" );
                    }
                }
                //Optimize point locations to solve the M-linear contraints
                std::vector< cv::Mat > corrected_keypoints;
                double total_error = CorrectTimestepPoints( cameras_in_timestep, keypoints, corrected_keypoints );
                if (total_error <= oBestFitErrors[j])
                {
                    oBestFitErrors[j] = total_error;
                    oBestFit[j] = possible_combinations[num_points][p];
                }
            }
        }
    }

}

/* Enumerates all possible combinations of a 2 tier nested list where the length of
 * the first list is variable, and each list contained is of the same length.
 */
void EnumerateCombinations( std::vector< std::list< uint32_t > > iIndexesLeft,
                            uint32_t iIdx,
                            std::vector< std::pair<uint32_t,uint32_t> > iAccum,
                            std::vector< std::vector< int32_t > > iCombinations,
                            std::vector< std::vector< std::vector< int32_t > > >& oFullCombinations )
{

    if (iIdx == iIndexesLeft.size() || 0 == iIndexesLeft[0].size()) // done, no more rows
    {
        if (0 == iIndexesLeft[0].size())
        {
            oFullCombinations.push_back( iCombinations );
        }
        else
        {
            std::vector<int32_t> accum_values;
            for (uint32_t i = 0; i < iAccum.size(); ++i)
            {
                std::list<uint32_t>::iterator it = iIndexesLeft[i].begin();
                advance( it, iAccum[i].first );
                iIndexesLeft[i].erase( it );
                accum_values.push_back( iAccum[i].second );
            }

            iCombinations.push_back( accum_values );

            std::vector< std::pair< uint32_t, uint32_t > > tmp;
            EnumerateCombinations( iIndexesLeft, 0, tmp, iCombinations, oFullCombinations );        }
    }
    else
    {
        std::list<uint32_t> row = iIndexesLeft[iIdx];
        uint32_t row_length = row.size();

        if( iIdx == 0 )
        {
            //We are fixing the camera so that we don't get multiples of the same combos but assigned to diff points.
            //i.e. ( 0,0,0),(1,1,1) and (1,1,1),(0,0,0) are the same
            row_length = 1;
        }

        for( uint32_t j = 0; j < row_length; ++j )
        {
            std::vector< std::pair< uint32_t, uint32_t> > tmp( iAccum );
            std::list<uint32_t>::iterator iter = row.begin();
            std::advance( iter, j );
            std::pair< uint32_t, uint32_t > tmp_pair(j, *iter);
            tmp.push_back( tmp_pair );
            EnumerateCombinations( iIndexesLeft, iIdx+1, tmp, iCombinations, oFullCombinations );
        }

    }
}


void EnumerateIndexCombinations( unsigned long iNumPoints,
                                 unsigned long iNumCams,
                                 std::vector< std::vector< std::vector< int32_t > > >& oIndexesPerPoint )
{
    std::vector< std::list< uint32_t >  > all_indexes;
    for (uint32_t cam = 0; cam < iNumCams; ++cam)
    {
        std::list< uint32_t > cam_point_index_list;
        for (uint8_t idx = 0; idx < iNumPoints; ++idx)
        {
            cam_point_index_list.push_back( idx );
        }
        all_indexes.push_back( cam_point_index_list );
    }

    //for( uint32_t p = 0; p < iNumPoints; ++p ){
        std::vector< std::pair< uint32_t, uint32_t > > tmp;
        std::vector< std::vector< int32_t > > combinations;
        EnumerateCombinations( all_indexes, 0, tmp, combinations, oIndexesPerPoint );
    //}
}





//divide and conquer -esque approach to get global minimal matching
//The double value in the std::pair accumulates the total distance
void OrderCorrespondingPoints( const std::vector< SmtPixel >& iCamFixedGroup,
                               const unsigned long iFixedIdx,
                               const unsigned long iFixedCamIdx,
                               const std::vector< SmtPixel >& iCamUnorderedGroup,
                               const std::vector< unsigned long >& iIndexesLeft,
                               const CamMats& iCamMatrix,
                               const unsigned long iCamIdx,
                               const std::pair< double, std::vector< SmtPixel> >&  iCamWorkingGroup,
                               std::vector< std::pair< double, std::vector< SmtPixel > > >& oCamOrderedGroups )
{

    //base case
    if (0 == iIndexesLeft.size())
    {
        if (iCamWorkingGroup.second.size() == iCamUnorderedGroup.size())
        {
            oCamOrderedGroups.push_back( iCamWorkingGroup );
        }
        else
        {
            std::fprintf( stderr, "ERROR: orderCorrespondingPoints:: no indexes left to check.\n" );
            std::fprintf( stderr,
                          "ERROR: orderCorrespondingPoints:: group size: %lu, expected: %lu \n",
                          iCamWorkingGroup.second.size(),
                          iCamUnorderedGroup.size() );
            assert( false ); //should never get here and we want it to fail horribly if it does.
        }
        return;
    }
    else if (iFixedIdx >= iCamUnorderedGroup.size())
    {
        std::fprintf( stderr, "Major Problem in set size during recursion!!!" );
        assert( false ); //should never get here and should fail horribly it it does.
        return;
    }
    else
    {
        //std::vector< SmtPixel > orderedCamPixels;

        //Get point info for fixed anchor point and camera
        std::vector< SmtPixel > end_points = iCamFixedGroup;
        cv::Point3d start_fixed_point;
        std::vector< cv::Point3d > end_fixed_points;
        CalculateTransformedPosition( iFixedCamIdx, end_points, iCamMatrix, start_fixed_point, end_fixed_points );

        //Get point infor the the other camera and the rays from that camera.
        std::vector< SmtPixel > end_points_left;
        for (unsigned long i = 0; i < iIndexesLeft.size(); ++ i)
        {
            end_points_left.push_back( iCamUnorderedGroup[ iIndexesLeft[ i ] ] );
        }

        cv::Point3d start_cand_point;
        std::vector< cv::Point3d > end_cand_points;
        CalculateTransformedPosition( iCamIdx, end_points_left, iCamMatrix, start_cand_point, end_cand_points );

        std::vector< cv::Point3d >::const_iterator cand_point_iter = end_cand_points.begin();
        std::vector< double > distances;

        for ( ; cand_point_iter != end_cand_points.end(); ++cand_point_iter)
        {
            double dist = opensource::getShortestDistance( start_fixed_point,
                                                           end_fixed_points[iFixedIdx],
                                                           start_cand_point,
                                                           *cand_point_iter );
            distances.push_back( dist );
        }

        for (unsigned long idx = 0; idx < distances.size(); ++idx)
        {
            std::vector< unsigned long > tmp_indexes_left = iIndexesLeft;
            std::pair< double, std::vector< SmtPixel > >  tmp_cam_working_group = iCamWorkingGroup;

            if (0 == tmp_cam_working_group.second.size())
            {
                tmp_cam_working_group.first = 0;
            }
            else
            {
                tmp_cam_working_group.first += distances[idx];
            }
            tmp_cam_working_group.second.push_back( iCamUnorderedGroup[ tmp_indexes_left[idx] ] );

            tmp_indexes_left.erase( tmp_indexes_left.begin() + idx );


            OrderCorrespondingPoints( iCamFixedGroup,
                                      idx,
                                      iFixedCamIdx,
                                      iCamUnorderedGroup,
                                      tmp_indexes_left,
                                      iCamMatrix,
                                      iCamIdx,
                                      tmp_cam_working_group,
                                      oCamOrderedGroups );

        }
    }
}

// only covers the timesteps
void OrderCorrespondingPoints( const PixelSet& iPixelSet,
                               const CamMats& iCamMatrix,
                               PixelSet& oPixelSet,
                               ImageSet iImageSet )
{
    if ( iImageSet.empty() )
    {
        //put this here so that the function wouldn't have to change signature
    }
    for (unsigned long j = 0; j < iPixelSet.size(); ++j)
    {
        if (0 == iPixelSet[j].size())
        {
            std::vector< std::vector< SmtPixel > > empty;
            oPixelSet.push_back( empty );
            continue;
        }
        std::vector<SmtPixel> fixedGroup = iPixelSet[j][0];

        if (0 == fixedGroup.size())
        {
            continue;
        }
        std::vector< std::vector< SmtPixel > > timestep_sets;
        timestep_sets.push_back( fixedGroup );

        unsigned long i, idx;
        for (i = 1, idx = 0; i < iPixelSet[j].size();++idx, ++i)
        {
            std::vector< unsigned long > indexes_not_sorted( iPixelSet[j][i].size() );
            generate( indexes_not_sorted.begin(), indexes_not_sorted.end(), IndexFiller() );

            std::vector< std::pair< double, std::vector< SmtPixel > > > cam_ordered_groups;
            std::pair< double, std::vector< SmtPixel > >  cam_working_group;
            OrderCorrespondingPoints( fixedGroup,
                                      idx,
                                      fixedGroup[0].Cam(),
                                      iPixelSet[j][i],
                                      indexes_not_sorted,
                                      iCamMatrix,
                                      iPixelSet[j][i][0].Cam(),
                                      cam_working_group,
                                      cam_ordered_groups );

            double min_value = std::numeric_limits< double >::infinity();
            std::vector< SmtPixel > ordered_set;

            //std::vector<std::pair<double,std::vector<SmtPixel> > >::iterator outputIter = camOrderedGroups.begin();
            //std::fprintf( stderr, "Calculating points with min distances...\n" );
            for (unsigned long tmp_idx = 0; tmp_idx < cam_ordered_groups.size(); ++tmp_idx)
            {
                if (cam_ordered_groups[tmp_idx].first < min_value)
                {
                    //std::fprintf( stderr, "    ****New Min Distance (%lf) set:[ ", camOrderedGroups[tmpIdx].first );
                    min_value = cam_ordered_groups[tmp_idx].first;
                    ordered_set = cam_ordered_groups[tmp_idx].second;
                    //for (unsigned long i = 1; i < orderedSet.size(); ++i)
                    //{
                    //    std::fprintf( stderr, "(%lf,%lf) ", orderedSet[i].x, orderedSet[i].y );
                    //}
                    //std::fprintf( stderr, " ]\n" );

                    //viewCorrespondence(fixedGroup,
                    //                   camOrderedGroups[tmpIdx].second,
                    //                   iImageSet[j][fixedGroup[0].cam],
                    //                   iImageSet[j][camOrderedGroups[0].second[0].cam],
                    //                   "**New Min Distance**" );
                }
                else{
                    //std::fprintf( stderr, "          distance (%lf) set:[ ", camOrderedGroups[tmpIdx].first );
                    //for (unsigned long i = 0; i < camOrderedGroups[tmpIdx].second.size(); ++i)
                    //{
                     //   std::fprintf( stderr,
                    //                  "(%lf,%lf) ",
                    //                  camOrderedGroups[tmpIdx].second[i].x,
                    //                  camOrderedGroups[tmpIdx].second[i].y );
                    //}
                    //std::fprintf( stderr, " ]\n" );
                    //viewCorrespondence(fixedGroup,
                    //                   camOrderedGroups[tmpIdx].second,
                    //                   iImageSet[j][fixedGroup[0].cam],
                    //                   iImageSet[j][camOrderedGroups[0].second[0].cam],
                    //                   "Candidate, but not min" );
                }
            }

            //viewCorrespondence(fixedGroup,
            //                   orderedSet,
            //                   iImageSet[j][fixedGroup[0].cam],
            //                   iImageSet[j][camOrderedGroups[0].second[0].cam],
            //                   "Winner of this set." );

            if  (ordered_set.size() == 0)
            {
                std::fprintf( stderr, "Should NEVER get here\n" );
                assert( false );
            }
            else
            {
                timestep_sets.push_back( ordered_set );
            }
        }
        oPixelSet.push_back( timestep_sets );
    }
}

//Greedy ordering, does not order by minimum total distance -- Code could be buggy as it is currently written...
void OrderCorrespondingPointsGreedy( const PixelSet& iPixelSet, const CamMats& iCamMatrix, PixelSet& oPixelSet )
{
    std::vector< double > focal_lengths;
    std::vector< cv::Point2d > optical_centers;
    iCamMatrix.GetPrincipalPointInfo( focal_lengths, optical_centers );

    for (unsigned long j = 0; j < iPixelSet.size(); ++j)
    {
        std::vector< std::vector< SmtPixel > > timestep_pixels;
        std::vector< SmtPixel > base_set = iPixelSet[j][0];
        timestep_pixels.push_back( base_set );

        //We key off of cam0 to match up the corresponding pixels, can't do anything about
        //the order through timesteps, need more sophisticated algs for that.
        for (unsigned long i = 1; i < iPixelSet[j].size(); ++i)
        {
            std::vector< SmtPixel > ordered_cam_pixels;
            std::vector< SmtPixel > unordered_cam_pixels = iPixelSet[j][i];
            std::vector< unsigned long > indexes_not_sorted( unordered_cam_pixels.size() );
            std::generate( indexes_not_sorted.begin(), indexes_not_sorted.end(), IndexFiller() );

            cv::Point3d start( 0, 0, 0 );

            cv::Point3d tmp_start( 0, 0, 0 );
            //std::fprintf( stderr,
            //              " cv::Mat( tmpStart ): rows: %d, cols: %d, channels: %d \n",
            //               cv::Mat( tmpStart).rows, cv::Mat(tmpStart).cols, cv::Mat(tmpStart).channels());
            //std::fprintf( stderr,"
            //              iCamMatrix.R[i]: rows: %d, cols: %d, channels: %d \n",
            //              iCamMatrix.R[i].rows, iCamMatrix.R[i].cols,iCamMatrix.R[i].channels());
            //std::fprintf( stderr,
            //              " cv::Mat( iCamMatrix.T[i] ): rows: %d, cols: %d, channels: %d \n",
            //              cv::Mat( iCamMatrix.T[i] ).rows,
            //              cv::Mat( iCamMatrix.T[i] ).cols, cv::Mat( iCamMatrix.T[i] ).channels());

            //cv::Mat_< double > tmpPoseMat2 = iCamMatrix.Pose()[i]; //use to be computedPose , this matrix was gotten rid of and this matrix could be different....
            cv::Mat_< double > tmp_rotated_mat = iCamMatrix.R()[i] * cv::Mat( start );
            const double *p_rotate = tmp_rotated_mat.ptr< double >( 0 );

            //Point3d tmpPosePoint = Point3d(rotatePtr[0], rotatePtr[1], rotatePtr[2]) + iCamMatrix.T[i];
            tmp_rotated_mat = tmp_rotated_mat * cv::Mat( iCamMatrix.T()[i] );

            p_rotate = tmp_rotated_mat.ptr< double >(0);
            cv::Point3d tmp_pose_point = cv::Point3d( p_rotate[0], p_rotate[1], p_rotate[2] );

            tmp_start = tmp_pose_point;
            //std::fprintf( stderr,
            //              " \n***Camera point calculated to be at (%lf,%lf,%lf), old calculation (%lf,%lf,%lf)\n",
            //             tmpStart.x,
            //             tmpStart.y,
            //             tmpStart.z,
            //             tmpPoseMat2.at< double >(0,0), tmpPoseMat2.at< double >(1,0), tmpPoseMat2.at<double>(2,0) );

            if (unordered_cam_pixels.size() == base_set.size())
            {
                for (unsigned long basePoint = 0; basePoint < base_set.size(); ++basePoint)
                {
                    std::vector< double > distances;
                    double dist_z = sqrt( pow( (base_set[basePoint].x - optical_centers[0].x), 2 ) +
                                         pow( (base_set[basePoint].y - optical_centers[0].y), 2 ) );
                    cv::Point3d end( base_set[basePoint].x,
                                     base_set[basePoint].y,
                                     sqrt( pow( focal_lengths[0] ,2) + pow(dist_z,2) ) );
                    //std::fprintf( stderr, "BasePointInfo: start:(0,0,0)  end:(%lf,%lf,%lf)\n", end.x, end.y, end.z);
                    std::vector< unsigned long >::iterator cand_point_iter = indexes_not_sorted.begin();
                    for ( ; cand_point_iter != indexes_not_sorted.end(); ++cand_point_iter)
                    {
                        dist_z = sqrt( pow( (unordered_cam_pixels[*cand_point_iter].x - optical_centers[i].x), 2 ) +
                                      pow( (unordered_cam_pixels[*cand_point_iter].y - optical_centers[i].y), 2 ) );
                        cv::Point3d tmp_end( unordered_cam_pixels[*cand_point_iter].x,
                                            unordered_cam_pixels[*cand_point_iter].y ,
                                            sqrt( pow( focal_lengths[i], 2 ) + pow( dist_z, 2 ) ) );
                        tmp_rotated_mat = iCamMatrix.R()[i] * cv::Mat( tmp_end );
                        p_rotate = tmp_rotated_mat.ptr< double >(0);
                        tmp_end = cv::Point3d( p_rotate[0], p_rotate[1], p_rotate[2] ) + iCamMatrix.C()[i];
                        double dist = opensource::getShortestDistance( start, end, tmp_start, tmp_end );
                        distances.push_back( dist );
                    }

                    unsigned long high_idx = ULONG_MAX;
                    double val = std::numeric_limits< double >::infinity();
                    //std::fprintf( stderr, "Distances:\n    " );
                    for (unsigned long idx = 0; idx < distances.size(); ++idx)
                    {
                        //std::fprintf( stderr, "%lf,",distances[idx] );
                        if (distances[idx] < val)
                        {
                            high_idx = indexes_not_sorted[idx];
                            val = distances[idx];
                        }
                    }
                    if (0 == distances.size())
                    {
                        std::fprintf( stderr, "   No Points listed....shouldn't have gotten this far..." );
                        assert( false );
                    }
                    else
                    {
                        //std::fprintf( stderr,
                        //              "      highIdx: %d, point:(%lf, %lf)",
                        //              highIdx, unorderedCamPixels[highIdx].x, unorderedCamPixels[highIdx].y );
                    }
                    //std::fprintf( stderr, "\n\n");

                    if (ULONG_MAX != high_idx)
                    {
                        if (1 == indexes_not_sorted.size())
                        {
                                ordered_cam_pixels.push_back( unordered_cam_pixels[ high_idx ] );
                                indexes_not_sorted.erase( indexes_not_sorted.begin() );
                        }
                        else
                        {
                            std::vector< unsigned long >::iterator index_iter = find(indexes_not_sorted.begin(),
                                                                                    indexes_not_sorted.end(),
                                                                                    high_idx );

                            if (indexes_not_sorted.end() == index_iter && indexes_not_sorted.size() > 0)
                            {
                                std::fprintf( stderr,
                                              "Should have found something! This shouldn't happen! highIdx: %ld point:(%f,%f)\n",
                                              high_idx,
                                              unordered_cam_pixels[ high_idx ].x,
                                              unordered_cam_pixels[ high_idx ].y );
                                for (unsigned long z = 0; z < indexes_not_sorted.size(); ++z)
                                {
                                    std::fprintf( stderr, "----Index %lu: %lu \n",z, indexes_not_sorted[z] );
                                }
                                assert( false );
                            }
                            else
                            {
                                ordered_cam_pixels.push_back( unordered_cam_pixels[ high_idx ] );
                                indexes_not_sorted.erase( index_iter );
                            }
                        }
                    }
                }
            }
            else
            {
                std::fprintf( stderr, "Found unequal number of pixels when trying to match points!!!!! NOOOOOOOOOO!!!!! This shouldn't happen!!!\n" );
                assert( false );
            }
            if (ordered_cam_pixels.size() > 0)
            {
                //std::fprintf( stderr, "unordered Cam pixels:  [" );
                //for (unsigned long idx = 0; idx < unorderedCamPixels.size(); ++idx)
                //{
                //	std::fprintf( stderr, "(%lf,%lf),", unorderedCamPixels[idx].x, unorderedCamPixels[idx].y );
                //}
                //std::fprintf( stderr, "]\n" );
                //std::fprintf( stderr, "Ordered Cam pixels:  [" );
                //for (unsigned long idx = 0; idx < orderedCamPixels.size(); ++idx)
                //{
                //	std::fprintf( stderr, "(%lf,%lf),", orderedCamPixels[idx].x, orderedCamPixels[idx].y );
                //}
                //std::fprintf( stderr, "]\n" );
                timestep_pixels.push_back( ordered_cam_pixels );
            }
        }
        if (timestep_pixels.size() > 0)
        {
            oPixelSet.push_back( timestep_pixels );
        }
    }
}

void MouseClickCallBackFunc( int iEvent, int iX, int iY, int flags, void* ipUserData )
{
    if  (cv::EVENT_LBUTTONDBLCLK == iEvent)
    {
        std::cout << "Left button of the mouse is clicked - position (" << iX << ", " << iY << ")" << std::endl;
        double* ptr = static_cast<double*>( ipUserData );
        ptr[0] = iX;
        ptr[1] = iY;

        //The shift key flag is EVENT_FLAG_SHIFTKEY = 16
        if (flags & (1<<4))
        {
            ptr[2] = 1;
        }
    }
    if (cv::EVENT_RBUTTONDBLCLK == iEvent)
    { //undo last left click
        double* ptr = static_cast<double*>( ipUserData );
        ptr[0] = -2;
        ptr[1] = -2;
    }


}

cv::Scalar GetRGBInfo( int iIdx )
{
    switch( iIdx % 10 )
    {
        case 0:
            return cv::Scalar( unsigned(255), unsigned(0), unsigned(0) );
        case 1:
            return cv::Scalar( unsigned(0), unsigned(255), unsigned(0) );
        case 2:
            return cv::Scalar( unsigned(0), unsigned(0), unsigned(255) );
        case 3:
            return cv::Scalar( unsigned(255), unsigned(0), unsigned(255) );
        case 4:
            return cv::Scalar( unsigned(255), unsigned(255), unsigned(0) );
        case 5:
            return cv::Scalar( unsigned(0), unsigned(255), unsigned(255) );
        case 6:
            return cv::Scalar( unsigned(128), unsigned(0), unsigned(0) );
        case 7:
            return cv::Scalar( unsigned(128), unsigned(128), unsigned(0) );
        case 8:
            return cv::Scalar( unsigned(0), unsigned(128), unsigned(0) );
        case 9:
            return cv::Scalar( unsigned(0), unsigned(0), unsigned(128) );
        default:
            std::fprintf( stderr, "Should never get here.\n");
            assert( false );
    }
    return cv::Scalar( 0, 0, 0 );
}

void GetUserHelpWithCorrespondence( const ImageSet& iRawImages,
                                    const PixelSet& iPixelSet,
                                    bool (*CancelFunc)(),
                                    const std::pair<unsigned long, std::set<unsigned long> >& iCamsToExclude,
                                    const unsigned long iNumPoints,
                                    const std::vector< unsigned long >& iTimesteps, //allows us to check multiple timesteps if needed in future
                                    std::vector< std::vector< std::vector< int32_t > > >& oUserFit,
                                    std::vector< std::vector< cv::Point2d > >& oMarkedPoints,
                                    const bool iAllowHiddenPointMarking)
{
    cv::destroyAllWindows();
    oUserFit.clear();
    unsigned long max_num_cams = iCamsToExclude.first;
    //unsigned long numUsableCams = max_num_cams - iCamsToExclude.second.size();

    for (uint32_t j = 0; j < iTimesteps.size(); ++j)
    {
        if (CancelFunc())
        {
            std::fprintf(stdout, "Cancelling correspondences...\n");
            oUserFit.clear();
            oMarkedPoints.clear();
            cv::destroyAllWindows();
            return;
        }
        //timestepList holds the corresponding point indexes at the end of the loop
        std::vector< std::vector< int32_t > > timestep_list;// first vector is points second is cameras,
        for (uint32_t t = 0; t < iNumPoints; ++t)
        {
            std::vector< int32_t > point_idx_list;
            point_idx_list.resize( max_num_cams );
            timestep_list.push_back( point_idx_list );
        }
        std::vector< std::list<unsigned long> > point_idx_left( max_num_cams );// first vector is cams, then point index
        for (unsigned long c = 0; c < max_num_cams; ++c)
        {
            if (1 == iCamsToExclude.second.count( c ))
            {
                continue;
            }
            std::list<unsigned long> tmp_list;
            for (unsigned long t = 0; t < iNumPoints; ++t)
            {
                tmp_list.push_back( t );
            }
            point_idx_left[ c ] = tmp_list;
        }

        uint32_t cam_start_idx = 0; //Need to keep track for undoing points
        //std::vector< std::vector< cv::Point2d > > alreadyClickedPoints;
        oMarkedPoints.clear();
        oMarkedPoints.resize( max_num_cams );
        for (uint32_t i = 0; i< max_num_cams; ++i)
        {
            if (1 == iCamsToExclude.second.count( i ))
            {
                continue;
            }
            oMarkedPoints[i].resize( iNumPoints );
        }
        for (uint32_t p = 0; p < iNumPoints; ++p)
        {
            std::string name = "Point " + std::to_string(p+1) + " ";
            cv::namedWindow( name, CV_WINDOW_AUTOSIZE | CV_GUI_NORMAL | CV_WINDOW_KEEPRATIO);
            bool undo_called_prev_point = false;

            for (uint32_t i = cam_start_idx; i < max_num_cams; ++i)
            {
                if (CancelFunc())
                {
                    std::fprintf(stdout, "Cancelling correspondences...\n");
                    oUserFit.clear();
                    oMarkedPoints.clear();
                    cv::destroyAllWindows();
                    return;
                }
                if (1 == iCamsToExclude.second.count( i ))
                {
                    continue;
                }
                cam_start_idx = 0;
                double point[3] = { -1, -1, -1};//{x,y,isHiddenPoint}
                cv::setMouseCallback( name, MouseClickCallBackFunc, &point );
                //show cam image for timestep and wait for user click
                cv::Mat cur_image = iRawImages[iTimesteps[j]][i].clone();
                for (uint32_t pix = 0; pix < iPixelSet[iTimesteps[j]][i].size(); ++pix)
                {
                    cv::drawMarker( cur_image,
                                    iPixelSet[iTimesteps[j]][i][pix],
                                    cv::Scalar( 255, 105, 180 ),
                                    cv::MARKER_TRIANGLE_DOWN,
                                    8/*size*/,
                                    2 /*thickness*/ );
                }
                for (uint32_t mark = 0; mark < p; ++mark)
                {
                    cv::drawMarker( cur_image,
                                    oMarkedPoints[i][mark],
                                    GetRGBInfo( mark ),
                                    cv::MARKER_STAR,
                                    6/*size*/,
                                    3/*thickness*/ );
                }
                cv::imshow( name, cur_image );
                uint32_t count = 0;
                while (point[0] == -1 || point[1] == -1)
                {
                    std::cerr << "x: " << point[0] << "    y:" << point[1] << std::endl;
                    cv::waitKey(500); //doesn't play nice with QT, so we use C++11 lib
                    //std::this_thread::sleep_for(std::chrono::milliseconds(500));
                    ++count;
                    if ( count > 20)
                    {
                        count = 0;
                        if (CancelFunc())
                        {
                            std::fprintf(stdout, "Cancelling correspondences...\n");
                            oUserFit.clear();
                            oMarkedPoints.clear();
                            cv::destroyAllWindows();
                            return;
                        }
                    }

                }
                std::cerr << "Done waiting for click." << std::endl;

                cv::setMouseCallback(name, NULL, NULL);
                if (-2 == point[0] || -2 == point[1])
                {
                        //undo previous click
                    if (0 == i)
                    {
                        //Need to check if we have any cameras excluded by the user and skip over them.
                        int count_skipped_cams = 0;
                        for (int prevCam = max_num_cams -1; prevCam >= 0; --prevCam)
                        {
                            if (0 == iCamsToExclude.second.count( prevCam ))
                            {
                                break;
                            }
                            else
                            {
                                ++count_skipped_cams;
                            }
                        }

                        //undo the last click from a previous point. I.e. We are now on point 4, cam0, but we want to undo point 3 cam3 (with 4 cams total)
                        undo_called_prev_point = true;
                        int32_t previous_point_index = timestep_list[p-1][ max_num_cams - 1 - count_skipped_cams];
                        if (previous_point_index >= 0)
                        {
                            point_idx_left[ max_num_cams - 1 - count_skipped_cams].push_back( previous_point_index ); //Order doesn't matter, just need the index back in the list
                        }
                        break;
                    }
                    else
                    {
                        //Need to check if we have any cameras excluded by the user and skip over them.
                        int count_skipped_cams = 0;
                        for (int prevCam = i -1; prevCam >= 0; --prevCam)
                        {
                            if (0 == iCamsToExclude.second.count( prevCam ))
                            {
                                break;
                            }
                            else
                            {
                                ++count_skipped_cams;
                            }
                        }
                        int32_t previous_point_index = timestep_list[p][i-1 -count_skipped_cams];
                        if (previous_point_index >= 0)
                        {
                            point_idx_left[i -1 - count_skipped_cams].push_back( previous_point_index ); //Order doesn't matter, just need the index back in the list
                        }
                        i -= (2 + count_skipped_cams); //one for the ++ and one to move backwards
                        continue;
                    }
                }
                //get click coords and find point closest to click and save point's index in oUserFit

                ///@todo :TODO:
                //This method assumes the user can somewhat accurately click on the points to be tracked.
                //Furthermore, the user can't currently click on places where no point was found, or use one
                //point more than once in the case of merged points.
                if (!iAllowHiddenPointMarking)
                {
                    double best_dist = std::numeric_limits<double>::max();
                    int8_t index = std::numeric_limits<int8_t>::max();
                    for (uint32_t px = 0; px < point_idx_left[i].size(); ++px)
                    {
                        std::list< unsigned long >::iterator tmp_iter = point_idx_left[i].begin();
                        std::advance( tmp_iter, px );
                        int8_t idx = *tmp_iter;
                        double dist = sqrt( pow( iPixelSet[iTimesteps[j]][i][ idx ].x - point[0], 2) +
                                            pow( iPixelSet[iTimesteps[j]][i][ idx ].y - point[1], 2 ) );
                        if (dist < best_dist)
                        {
                            best_dist = dist;
                            index =  *tmp_iter;
                        }
                    }
                    //std::vector< cv::Point2d >::iterator iter = alreadyClickedPoints[i].begin();
                    //std::advance( iter, p );
                    oMarkedPoints[i][p].x = iPixelSet[iTimesteps[j]][i][index].x;
                    oMarkedPoints[i][p].y = iPixelSet[iTimesteps[j]][i][index].y;
                    timestep_list[p][i] = index ;
                    point_idx_left[i].remove( index );
                }
                else
                {
                    oMarkedPoints[i][p].x = point[0];
                    oMarkedPoints[i][p].y = point[1];
                    timestep_list[p][i] = -5;
                }
            }
            if (undo_called_prev_point)
            {
                //this only gets called if the mistake was made on point x, but we were already on x+1 before we got the button click

                //Need to check if we have any cameras excluded by the user and skip over them.
                int count_skipped_cams = 0;
                for (int prevCam = max_num_cams -1; prevCam >= 0; --prevCam)
                {
                    if (0 == iCamsToExclude.second.count( prevCam ))
                    {
                        break;
                    }
                    else
                    {
                        ++count_skipped_cams;
                    }
                }
                cam_start_idx = max_num_cams - 1 - count_skipped_cams;
                p -= 2; //subract one for the ++ at the end of the loop and one to go back
            }
            //remove window
            cv::destroyWindow( name );
        }
        oUserFit.push_back( timestep_list );
    }

    //If there are multiple timesteps, we should be able to use user coords to double check how well
    //the filters are working, but that requires having the filters as input since the indexes of the
    //points are only useful for a single timestep since after that we care about the index of the filter
    //whose predicted value is closest to the point.
}

