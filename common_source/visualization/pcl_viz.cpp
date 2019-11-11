#include <string>
#include <opencv2/core.hpp>
#include <pcl/visualization/pcl_visualizer.h>

#include "common_types.h"
#include "processing/Triangulation.h"
#include "utils.h"

#include "pcl_viz.h"

void ViewEpipolarLines( const std::vector< std::vector< std::pair< cv::Point3d, cv::Point3d > > >& iRays )
{
    pcl::visualization::PCLVisualizer viewer ("PCL visualizer");
    int r = 0;
    int g = 0;
    int b = 0;
    //int index = 0;
    //std::vector<std::vector<cv::Point3d> >::iterator timestepIter = calculatedPoints.begin();
    //int count = 0;
    //float checkVal = 0.123456;
    //pcl::PointXYZ oldPoint(checkVal, checkVal, checkVal);
    //for ( ; timestepIter != calculatedPoints.end(); ++timestepIter, ++index){

    pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud( new pcl::PointCloud< pcl::PointXYZRGB > );

    int sum_rays = 0;
    for (unsigned int i = 0; i < iRays.size(); ++i)
    {
        sum_rays += iRays[i].size();
    }
    int numPoints = sum_rays * 2;
    std::fprintf( stderr, "Total number of Rays being plotted %d.\n", numPoints );

    // Fill in the cloud data
    cloud->width = numPoints; //pixelsTriangulated;
    cloud->height = 1;
    cloud->is_dense = false;
    cloud->points.resize( numPoints );

    /*for (int i = 0; i < iRays.size(); ++i)
      {
        cloud->points[2 * i].x = iFixedRays[i].first.x;
        cloud->points[2 * i].y = iFixedRays[i].first.y;
        cloud->points[2 * i].z = iFixedRays[i].first.z;

        cloud->points[2 * i + 1].x = iFixedRays[i].second.x;
        cloud->points[2 * i + 1].y = iFixedRays[i].second.y;
        cloud->points[2 * i + 1].z = iFixedRays[i].second.z;

        std::fprintf( stderr, "   ---fixed (%lf,%lf,%lf) --> (%lf,%lf,%lf)\n", iFixedRays[i].first.x,
                                                            iFixedRays[i].first.y,
                                                            iFixedRays[i].first.z,
                                                            iFixedRays[i].second.x,
                                                            iFixedRays[i].second.y,
                                                            iFixedRays[i].second.z );
        getColor( i, r, g, b );
        std::string lineName = "fixed_line" + std::to_string( i );
        viewer.addLine( cloud->points[2 * i], cloud->points[2 * i + 1], r, g, b, lineName );

    }

    viewer.addSphere( cloud->points[0], 10, .5, .5, 0, "fixed_optical_center" );

    int count = 2 * iFixedRays.size();
    for (int cam = 0; cam < iCandidateRays.size(); ++cam)
    {
        const int intermediateCount = count;
        for (int i = 0; i < iCandidateRays[cam].size() && count < numPoints; ++i)
        {
            cloud->points[ count ].x = iCandidateRays[cam][i].first.x;
            cloud->points[ count ].y = iCandidateRays[cam][i].first.y;
            cloud->points[ count ].z = iCandidateRays[cam][i].first.z;
            count++;

            cloud->points[ count ].x = iCandidateRays[cam][i].second.x;
            cloud->points[ count ].y = iCandidateRays[cam][i].second.y;
            cloud->points[ count ].z = iCandidateRays[cam][i].second.z;
            count++;

            std::fprintf( stderr, "   ---cand cam %d (%lf,%lf,%lf) --> (%lf,%lf,%lf)      points left:%ld\n",
                                  cam, iCandidateRays[cam][i].first.x,
                                iCandidateRays[cam][i].first.y,
                                iCandidateRays[cam][i].first.z,
                                iCandidateRays[cam][i].second.x,
                                iCandidateRays[cam][i].second.y,
                                iCandidateRays[cam][i].second.z,
                                iCandidateRays[cam].size() - (i + 1) );


            //viewer.addPointCloud (cloud, "cloud"); // Method #1


            std::string lineName2 = "candidate_line_" + std::to_string(cam) + "_" + std::to_string( i );
            viewer.addLine( cloud->points[count - 2], cloud->points[count - 1], r, g, b, lineName2 );
        }
        std::string lineName = "candidate_optical_center" + std::to_string(cam);
        viewer.addSphere( cloud->points[intermediateCount], 10, lineName );
    }*/
    unsigned int cloud_point_idx = 0;
    for (unsigned int cam = 0; cam < iRays.size(); ++cam)
    {
        GetColor(cam, r, g, b );
        int start_count = cloud_point_idx;

        for (unsigned int i = 0; i < iRays[cam].size(); ++i)
        {
            cloud->points[ cloud_point_idx ].x = iRays[cam][i].first.x;
            cloud->points[ cloud_point_idx ].y = iRays[cam][i].first.y;
            cloud->points[ cloud_point_idx ].z = iRays[cam][i].first.z;
            cloud_point_idx++;

            cloud->points[ cloud_point_idx ].x = iRays[cam][i].second.x;
            cloud->points[ cloud_point_idx ].y = iRays[cam][i].second.y;
            cloud->points[ cloud_point_idx ].z = iRays[cam][i].second.z;
            cloud_point_idx++;

//				std::fprintf( stderr, "   ---cand cam %d (%lf,%lf,%lf) --> (%lf,%lf,%lf)      points left:%ld\n", cam, iRays[cam][i].first.x,
//																	iRays[cam][i].first.y,
//																	iRays[cam][i].first.z,
//																	iRays[cam][i].second.x,
//																	iRays[cam][i].second.y,
//																	iRays[cam][i].second.z,
//																	iRays[cam].size() - (i + 1) );

            std::string lineName2 = "line_" +
                                    std::to_string( cloud_point_idx ) +
                                    "_" + std::to_string(cam) + "_" + std::to_string( i );
            viewer.addLine( cloud->points[cloud_point_idx - 2],
                            cloud->points[cloud_point_idx - 1],
                            r, g, b, lineName2 );
        }

        //Add spheres for the different camera positions.
        std::string name = "optical_center" + std::to_string( cam );
        if (!viewer.contains( name ))
        {
            viewer.addSphere( cloud->points[start_count], 10, name);
        }
    }
    std::fprintf( stderr, "Added points to cloud, now adding lines to viewer\n" );
    viewer.addPointCloud( cloud, "cloud" ); // Method #1
    viewer.addCoordinateSystem( 100.0, "axis", 0 );
    viewer.setBackgroundColor( 255, 255, 255, 0 );	// Setting background to a white
    viewer.setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud" );

    //viewer.resetCamera();
    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }
}

void AddEpipolarLines( const std::vector< std::vector< std::pair< cv::Point3d, cv::Point3d > > >& iRays,
                       const std::vector< int >& iCams,
                       pcl::visualization::PCLVisualizer& ioViewer,
                       pcl::PointCloud< pcl::PointXYZRGB >::Ptr& ioCloud,
                       unsigned int& ioCloudPointIdx )
{
    int r = 0;
    int g = 0;
    int b = 0;

    for (unsigned int cam = 0; cam < iRays.size(); ++cam)
    {
        GetColor(iCams[cam], r, g, b );
        int start_count = ioCloudPointIdx;

        for (unsigned int i = 0; i < iRays[cam].size(); ++i)
        {
            ioCloud->points[ ioCloudPointIdx ].x = iRays[cam][i].first.x;
            ioCloud->points[ ioCloudPointIdx ].y = iRays[cam][i].first.y;
            ioCloud->points[ ioCloudPointIdx ].z = iRays[cam][i].first.z;
            ioCloudPointIdx++;

            ioCloud->points[ ioCloudPointIdx ].x = iRays[cam][i].second.x;
            ioCloud->points[ ioCloudPointIdx ].y = iRays[cam][i].second.y;
            ioCloud->points[ ioCloudPointIdx ].z = iRays[cam][i].second.z;
            ioCloudPointIdx++;

            std::fprintf( stderr, "   ---cand cam %d (%lf,%lf,%lf) --> (%lf,%lf,%lf)      points left:%ld\n",
                          cam,
                          iRays[cam][i].first.x,
                          iRays[cam][i].first.y,
                          iRays[cam][i].first.z,
                          iRays[cam][i].second.x,
                          iRays[cam][i].second.y,
                          iRays[cam][i].second.z,
                          iRays[cam].size() - (i + 1) );

            std::string lineName2 = "line_" +
                                    std::to_string( ioCloudPointIdx ) +
                                    "_" + std::to_string(iCams[cam]) + "_" + std::to_string( i );
            ioViewer.addLine( ioCloud->points[ioCloudPointIdx - 2],
                              ioCloud->points[ioCloudPointIdx - 1],
                              r, g, b, lineName2 );
        }

        //Add spheres for the different camera positions.
        std::string name = "optical_center" + std::to_string( iCams[cam] );
        if (!ioViewer.contains( name ))
        {
            ioViewer.addSphere( ioCloud->points[start_count], 10, name );
        }
    }
}

void ConvertPointsToLinesForViewing( const PixelSet& iPixelSet,
                                     const CamMats* const iCamMatrix,
                                     const uint64_t iNumTimesteps )
{

    pcl::visualization::PCLVisualizer viewer("PCL visualizer");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud( new pcl::PointCloud< pcl::PointXYZRGB > );
    //bool spheresAdded = false;

    unsigned int sum_points = 0;
    for (unsigned int j = 0; j < iPixelSet.size(); ++j)
    {
        for (unsigned int i = 0; i < iPixelSet[j].size(); ++i)
        {
            sum_points += iPixelSet[j][i].size();
        }
    }
    unsigned int num_points = sum_points * 2; //Inefficient because I continuosly plot cameras centers as new points...
    std::fprintf( stderr, "Total number of Rays being plotted %d .\n", num_points );

    // Fill in the cloud data
    cloud->width = num_points; //pixelsTriangulated;
    cloud->height = 1;
    cloud->is_dense = false;
    cloud->points.resize( num_points );

    unsigned int cloud_point_idx = 0;

    for (uint64_t j = 0; j < iNumTimesteps; ++j)
    { //timestep
        std::vector< std::vector< std::pair< cv::Point3d, cv::Point3d > > > rays;
        std::vector<int> cams;
        for (unsigned int i = 0; i < iPixelSet[j].size(); ++i)
        {
            //Get point infor the the other camera and the rays from that camera.
            const std::vector< SmtPixel > end_points = iPixelSet[j][i];
            cv::Point3d start_point;
            std::vector< cv::Point3d > end_point_list;
            float extend = 1000;

            if (end_points.size() < 1)
            {
                continue;
            }

            CalculateTransformedPosition( end_points[0].Cam(),
                                          end_points,
                                          *iCamMatrix,
                                          start_point,
                                          end_point_list,
                                          extend );

            cams.push_back( end_points[0].Cam() );

            std::vector< std::pair< cv::Point3d, cv::Point3d > > tmp_ray_group;
            for (unsigned int rayIdx = 0; rayIdx < end_points.size(); ++rayIdx)
            {
                tmp_ray_group.push_back( std::pair<cv::Point3d, cv::Point3d>( start_point, end_point_list[ rayIdx ] ) );
            }
            rays.push_back( tmp_ray_group );
        }
        AddEpipolarLines( rays, cams, viewer, cloud, cloud_point_idx );
    }
    viewer.addPointCloud( cloud, "cloud" );
    viewer.addCoordinateSystem( 100.0, "axis", 0 );
    viewer.setBackgroundColor( 255, 255, 255, 0 );	// Setting background to a white
    viewer.setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud" );

    //viewer.resetCamera();
    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }
    viewer.close();
}

//	void convertPointsToLinesForViewing( const PixelSet& iPixelSet, const CamMats& iCamMatrix, bool iCorrectPoints ){
//		std::fprintf( stderr, "Plotting each of %ld timesteps.\n", iPixelSet.size());
//		for( int j = 0; j < iPixelSet.size(); ++j ){ //timestep
//			std::fprintf( stderr, "Attempting to plot %ld pixels in this timestep.\n", iPixelSet[j].size());
//			std::vector< std::vector< std::pair< cv::Point3d, cv::Point3d > > > rays;

//			//Get point info for fixed anchor point and camera
//			//const std::vector< SmtPixel > endPoints = iPixelSet[j][0];
//			//cv::Point3d startFixedPoint;
//			//std::vector< cv::Point3d > endFixedPoints;
//			//calculateTransformedPosition( endPoints[0].cam, endPoints, iCamMatrix, startFixedPoint, endFixedPoints );
			
//			//for( int rayIdx = 0; rayIdx < endFixedPoints.size(); ++rayIdx ) fixedRays.push_back( std::pair< cv::Point3d, cv::Point3d >( startFixedPoint, endFixedPoints[ rayIdx ] ) );
					
//			for( int i = 0; i < iPixelSet[j].size(); ++i ){
//				//Get point infor the the other camera and the rays from that camera.
//				const std::vector< SmtPixel > endPointsCand = iPixelSet[j][i];
//				cv::Point3d startCandPoint;
//				std::vector< cv::Point3d > endCandPoints;
//				calculateTransformedPosition( endPointsCand[0].cam, endPointsCand, iCamMatrix, startCandPoint, endCandPoints );
//				std::vector< std::pair< cv::Point3d, cv::Point3d > > tmpRayGroup;
				
//				for( int rayIdx = 0; rayIdx < endCandPoints.size(); ++rayIdx ) tmpRayGroup.push_back( std::pair< cv::Point3d, cv::Point3d >( startCandPoint, endCandPoints[ rayIdx ] ) );
				
//				rays.push_back( tmpRayGroup );
//			}
//			viewEpipolarLines( rays );

//		}

//	}


