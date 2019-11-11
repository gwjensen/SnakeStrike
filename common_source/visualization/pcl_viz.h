#ifndef COMMON_VIZ_PCL_VIZ_H
#define COMMON_VIZ_PCL_VIZ_H

#include <opencv2/core.hpp>
#include <pcl/visualization/pcl_visualizer.h>

#include "image/PixelSet.h"
#include "processing/CamMats.h"
#include "common_types.h" 


void ViewEpipolarLines( const std::vector< std::vector< std::pair< cv::Point3d, cv::Point3d > > >& iRays  );

void AddEpipolarLines( const std::vector< std::vector< std::pair< cv::Point3d, cv::Point3d> > >& iRays,
                        const std::vector< int >& iCams, pcl::visualization::PCLVisualizer& ioViewer,
                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ioCloud, int& ioCloudPointIdx );

void ConvertPointsToLinesForViewing( const PixelSet& iPixelSet,
                                     const CamMats* const iCamMatrix,
                                     const uint64_t iNumTimesteps);
//void convertPointsToLinesForViewing( const PixelSet& iPixelSet, const CamMats& iCamMatrix, bool iCorrectPoints = false );


#endif //COMMON_VIZ_PCL_VIZ_H

