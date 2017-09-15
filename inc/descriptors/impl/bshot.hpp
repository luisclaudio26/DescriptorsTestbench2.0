#ifndef _BSHOT_HPP_
#define _BSHOT_HPP_

#include <pcl/features/shot_omp.h>
#include <pcl/features/impl/shot_omp.hpp>
#include "../../parameters.h"

template<typename PointInT, typename PointNT, typename PointOutT>
void BSHOTEstimation<PointInT, PointNT, PointOutT>::computeFeature(PointCloudOut& out)
{
	//B-SHOT needs SHOT to be calculated! Compute it.
	pcl::PointCloud<pcl::SHOT352> shotDesc;
	pcl::SHOTEstimationOMP<PointInT, PointNT, pcl::SHOT352> shot;
	typename pcl::search::KdTree<PointInT>::Ptr kdtree(new pcl::search::KdTree<PointInT>());

    shot.setNumberOfThreads( Parameters::getNThreads() );
    shot.setRadiusSearch( this->search_radius_);
	shot.setInputCloud(this->input_);
    shot.setSearchSurface(this->surface_);
	shot.setInputNormals(this->normals_);
	shot.setSearchMethod(kdtree);

	shot.compute(shotDesc);

	for(int i = 0; i < this->indices_->size(); ++i)
		compute_bshot_from_SHOT(shotDesc.at(i), out.at(i));
}

#define SETBIT(i,n) (n = n | (1 << i))
#define TESTBIT(i,n) (n & (1 << i))

template<typename PointInT, typename PointNT, typename PointOutT>
void BSHOTEstimation<PointInT, PointNT, PointOutT>::compute_bshot_from_SHOT(const pcl::SHOT352& shot_in, BSHOTDescriptor& out)
{
	//TODO: MAKE THIS WORK
    for (int j = 0 ; j < 88 ; j++)
    {
        float vec[4] = { 0 };
        for (int k = 0 ; k < 4 ; k++)
            vec[k] = shot_in.descriptor[ j*4 + k ];

        int bit = 0;
        float sum = vec[0]+vec[1]+vec[2]+vec[3];

        if (vec[0] == 0 and vec [1] == 0 and vec[2] == 0 and vec[3] == 0)
        {
            //bin[0] = bin[1] = bin[2] = bin[3] = 0;
            // by default , they are all ZEROS
        }
        else if ( vec[0] > (0.9 * (sum) ) )
        {
            SETBIT(0, bit);
        }
        else if ( vec[1] > (0.9 * (sum) ) )
        {
            SETBIT(1, bit);
        }
        else if ( vec[2] > (0.9 * (sum) ) )
        {
            SETBIT(2, bit);
        }
        else if ( vec[3] > (0.9 * (sum) ) )
        {

            SETBIT(3, bit);
        }
        else if ( (vec[0]+vec[1]) > (0.9 * (sum))  )
        {

            SETBIT(0, bit);
            SETBIT(1, bit);
        }
        else if ( (vec[1]+vec[2]) > (0.9 * (sum)) )
        {

            SETBIT(1, bit);
            SETBIT(2, bit);
        }

        else if ( (vec[2]+vec[3]) > (0.9 * (sum)) )
        {
            SETBIT(2, bit);
            SETBIT(3, bit);
        }
        else if ( (vec[0]+vec[3]) > (0.9 * (sum)) )
        {

            SETBIT(0, bit);
            SETBIT(3, bit);
        }
        else if ( (vec[1]+vec[3]) > (0.9 * (sum)) )
        {

            SETBIT(1, bit);
            SETBIT(3, bit);
        }
        else if ( (vec[0]+vec[2]) > (0.9 * (sum)) )
        {

            SETBIT(0, bit);
            SETBIT(2, bit);
        }
        else if ( (vec[0]+ vec[1] +vec[2]) > (0.9 * (sum)) )
        {

            SETBIT(0, bit);
            SETBIT(1, bit);
            SETBIT(2, bit);
        }
        else if ( (vec[1]+ vec[2] +vec[3]) > (0.9 * (sum)) )
        {

            SETBIT(1, bit);
            SETBIT(2, bit);
            SETBIT(3, bit);
        }
        else if ( (vec[0]+ vec[2] +vec[3]) > (0.9 * (sum)) )
        {

            SETBIT(0, bit);
            SETBIT(2, bit);
            SETBIT(3, bit);
        }
        else if ( (vec[0]+ vec[1] +vec[3]) > (0.9 * (sum)) )
        {

            SETBIT(0, bit);
            SETBIT(1, bit);
            SETBIT(3, bit);
        }
        else
        {

            SETBIT(0, bit);
            SETBIT(1, bit);
            SETBIT(2, bit);
            SETBIT(3, bit);
        }

        if (TESTBIT(0, bit))
            out.bitset[j*4] = -1;

        if (TESTBIT(1, bit))
            out.bitset[j*4 + 1] = -1;

        if (TESTBIT(2, bit))
            out.bitset[j*4 + 2] = -1;

        if (TESTBIT(3, bit))
            out.bitset[j*4 + 3] = -1;
    }
}

#endif