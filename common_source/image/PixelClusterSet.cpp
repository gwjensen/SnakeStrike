#include <map>

#include "PixelClusterSet.h"

void GetMostRecurringNumber( const std::vector<uint32_t>& iClusters, uint32_t& oOccurrences, uint32_t& oValue )
{
    oOccurrences = 0;
    oValue = 0;
    ///TODO use the map instead of sorting...perhaps sorting would be faster as I shouldn't have a whole ton of points...
    std::map<uint32_t,uint32_t> tmp_occur = { {0,0} }; //populate with zero because we shouldn't ever get zero as a value
    std::map<uint32_t,uint32_t>::iterator occur_iter;
    uint32_t tmp_value = 0;

    for (uint32_t idx = 0; idx < iClusters.size(); ++idx)
    {
        if (tmp_value == iClusters[idx])
        {
            tmp_occur[tmp_value] += 1;
        }
        else
        {
            tmp_value = iClusters[idx];
            occur_iter = tmp_occur.find( tmp_value );
            if (occur_iter == tmp_occur.end())
            {
                tmp_occur[tmp_value] = 1;
            }
            else
            {
                tmp_occur[tmp_value] += 1;
            }
        }

        if (tmp_occur[ tmp_value ] >  oOccurrences && tmp_value != 0)
        {
            oValue = tmp_value;
            oOccurrences = tmp_occur[ tmp_value ];
        }
        else if (tmp_occur[ tmp_value ] == oOccurrences && tmp_value > oValue)
        {
            //Handles the case where two numbers have the same number of occurrences, but we want to
            //take those indexes with the highest number of points.
            oValue = tmp_value;
            oOccurrences = tmp_occur[ tmp_value ];
        }
    }
}

//This function goes through the clusters and tries to determine the number of clusters that are
//visible in the most number of images. If this number is less than the total amount of expected
//clusters, I have no way of guaranteeing that the subset of clusters is the same in the images
///@TODO Think of a way to force the clusters to line up as well...or possibly modify sfm::triangulatePoints to accept them but toss them out if needed?
void ShrinkClustersToMaxSubset( PixelClusterSet& ioClusterSet, int iNumMinCams )
{
    assert( iNumMinCams >=2 );
    PixelClusterSet::iterator timestep_iter = ioClusterSet.begin();
    //std::fprintf( stderr, "shrinkClustersToMaxSubset: start\n" );
    for( ; timestep_iter != ioClusterSet.end(); /*no increment because erase increments iterator*/)
    {
        std::vector< uint32_t > num_clusters_per_image;
        uint32_t occurrences = 0;
        uint32_t value = 0;
        //std::vector< std::vector<PixelCluster> > empty;

        std::vector< std::vector<PixelCluster> >::iterator image_cluster_iter = timestep_iter->begin();
        for ( ; image_cluster_iter!= timestep_iter->end(); ++image_cluster_iter)
        {
            num_clusters_per_image.push_back( image_cluster_iter->size() );
        }
        GetMostRecurringNumber( num_clusters_per_image, occurrences, value );
        int count = 0;
        image_cluster_iter = timestep_iter->begin();
        for ( ; image_cluster_iter != timestep_iter->end(); /*no increment because erase increments iterator*/)
        {
            if (image_cluster_iter->size() != value)
            {
                //std::fprintf(stderr, "--shrinkClustersToMaxSubset:: deleting cluster\n");
                image_cluster_iter->clear();//wanna keep the empty list around
            }
            else
            {
                ++count;
            }
            ++image_cluster_iter;
        }
        if (count /*timestepIter->size()*/ < iNumMinCams)
        {
            //std::fprintf( stderr,"--shrinkClustersToMaxSubset:: deleting timestep\n" );
            //ioClusterSet.erase( timestepIter );
            ///  This creates an empty timestep value...this means we need to be careful later about
            /// blindly accessing camera indexes on a timestep.
            timestep_iter->clear( );
            ++timestep_iter;
        }
        else
        {
            ++timestep_iter;
        }
    }
    //std::fprintf( stderr, "shrinkClustersToMaxSubset:: end\n" );
}

//My naive attempt at clustering before I discovered findContours() in opencv
void MergeCloseClusters( const PixelClusterSet& iClusterSet, PixelClusterSet& oClusterSet )
{
    //Use brute force to figure out in which groups the pixels belong...Must be group number invariant.
    //I'll be using agglomerative hierarchical clustering to get things grouped.
    for (uint32_t j = 0; j < iClusterSet.size(); ++j)
    {
        std::vector< std::vector<PixelCluster> > timestep_clusters;
        for (uint32_t i = 0; i < iClusterSet[j].size(); ++i)
        {
            std::vector<PixelCluster> clusters = iClusterSet[j][i];
            if( clusters.size() == 0 ){
                timestep_clusters.push_back( clusters );
                continue;
            }
            std::vector<PixelCluster>::iterator cluster_iter;

            //Start merging clusters based on their distance and the allowable
            //distance( dist in relation to # of std devs )
            bool keep_merging = true;
            bool empty_pass = false;
            while (keep_merging)
            {
                for (uint32_t m = 0; m < clusters.size(); ++m)
                {
                    //Chose this arbitrarily, will need to be able to change this by a selector of some sort
                    float min_distance = clusters[m].StdDev() * 2;
                    //std::fprintf(stderr,
                    //              "Minimum distance clusters need to be apart for this cluster %f, num points: %d, mean:(%f,%f)\n",
                    //              minDistance, clusters[m].size, clusters[m].mean.x, clusters[m].mean.y);
                    uint32_t minIndex = m;
                    for (uint32_t n = 0; n < clusters.size(); ++n)
                    {
                        if (m != n)
                        {
                            float dist = clusters[m].CalcDistance( clusters[n] );
                            if (dist < min_distance)
                            {
                                min_distance = dist;
                                minIndex = n;
                            }
                        }
                    }
                    if (minIndex != m)
                    {
                        clusters[m].Merge(clusters[minIndex]);
                        cluster_iter = clusters.begin();
                        advance( cluster_iter, minIndex );
                        clusters.erase( cluster_iter );
                        empty_pass = false;
                        continue;
                    }
                    if (empty_pass)
                    {
                        //Already made one empty pass, we are done merging
                        keep_merging = false;
                    }
                    if (m == clusters.size() - 1)
                    {
                        //We went through the whole thing and didn't make any merges
                        //We will go back through one time to make sure we didn't miss any potential merges
                        empty_pass = true;
                    }
                }
            }
            timestep_clusters.push_back( clusters );
        }
        oClusterSet.push_back( timestep_clusters );
    }
}

/*void sortPixelsIntoGroups( const PixelSet& iPixelSet, ClusterSet& oClusterSet)
{
    for (int j = 0; j < iPixelSet.size(); ++j)
    {
        std::vector<std::vector<Cluster> > timestep_clusters;
        for (int i = 0; i < iPixelSet[j].size(); ++i )
        {
            std::vector<Cluster> grouped_points;
            ///@TODO might need to only return the mean of gaussian as the Pixel Point
            sortPixelsIntoGroups( iPixelSet[j][i], grouped_points );
            timestep_clusters.push_back( grouped_points );
        }
        oClusterSet.push_back( timestep_clusters );
    }
}*/

void ConvertClustersToPixels( const PixelClusterSet& iClusterSet, PixelSet& oPixelSet )
{
    for (uint32_t j = 0; j < iClusterSet.size(); ++j)
    {
        std::vector< std::vector< SmtPixel > > timestep_pixels;
        std::vector< std::vector<PixelCluster> >::const_iterator timestep_iter = iClusterSet[j].begin();

        ///@TODO double check that I don't need to be updating the i value, seems fishy...
        for (int i = 0; timestep_iter != iClusterSet[j].end(); ++i, ++timestep_iter)
        {
            std::vector< SmtPixel > pixels;

            //Handle timesteps for which we don't have a sufficient number of clusters.
            if (timestep_iter->size() == 0)
            {
                timestep_pixels.push_back( pixels );
                continue;
            }
            for (uint32_t h = 0; h < iClusterSet[j][i].size(); ++h)
            {
                //std::fprintf( stderr,
                //              "convertClustersToPixels:: timestep:%d Cam:%d  Location:(%f,%f)\n",
                //              j,
                //              iClusterSet[j][i][h].points[0].cam,
                //              iClusterSet[j][i][h].mean.x,
                //              iClusterSet[j][i][h].mean.y );
                SmtPixel new_pixel( iClusterSet[j][i][h].Mean(), iClusterSet[j][i][h].Points()[0].Cam() );
                if (iClusterSet[j][i][h].Size() == 0)
                {
                    new_pixel.rIsValid() = false;
                }
                pixels.push_back( new_pixel );
            }
            timestep_pixels.push_back( pixels );
        }
        oPixelSet.push_back( timestep_pixels );
    }
}
