#ifndef PixelClusterSet_h
#define PixelClusterSet_h

#include <vector>

#include "PixelCluster.h"
#include "PixelSet.h"

typedef std::vector< std::vector< std::vector<PixelCluster> > > PixelClusterSet;


#include "PixelClusterSet.h"

//This function goes through the clusters and tries to determine the number of clusters that are visible in the most number of images. If this number
//is less than the total amount of expected clusters, I have no way of guaranteeing that the subset of clusters is the same in the images
///@TODO Think of a way to force the clusters to line up as well...or possibly modify sfm::triangulatePoints to accept them but toss them out if needed?
void ShrinkClustersToMaxSubset( PixelClusterSet& ioClusterSet, int iNumMinCams );

//My naive attempt at clustering before I discovered findContours() in opencv
void MergeCloseClusters( const PixelClusterSet& iClusterSet, PixelClusterSet& oClusterSet );

void ConvertClustersToPixels( const PixelClusterSet& iClusterSet, PixelSet& oPixelSet );






#endif // PixelClusterSet_h
