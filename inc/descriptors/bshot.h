#ifndef _BSHOT_H_
#define _BSHOT_H_

#include <pcl/point_types.h>
#include <pcl/features/feature.h>
#include <pcl/features/impl/feature.hpp>
using pcl::FeatureFromNormals;

//---------------------------------
//---------- DISCLAIMER -----------
//---------------------------------
// This code was adapted from B-SHOT's original
// code, available in https://sites.google.com/site/bshotdescriptor/
// provided by the authors of the article.
// Code was adapted so to fit our benchmarking platform.

struct BSHOTDescriptor
{
	//For descriptiveness measures,
	//using a std::bitset or an array of ints
	//yields the same results.
	int bitset[352];


	static void NNHamming(BSHOTDescriptor point, 
							const pcl::PointCloud<BSHOTDescriptor>::Ptr& cloud, 
							int& index, int& min_dist)
	{
		min_dist = std::numeric_limits<int>::max();
		for(int d = 0; d < cloud->size(); ++d)
		{
			int dist = 0; 

			for(int i = 0; i < 352; i++)
				dist += point.bitset[i] ^ cloud->at(d).bitset[i];

			if(dist < min_dist)
			{
				min_dist = dist;
				index = d;
			}
		}
	}
};

POINT_CLOUD_REGISTER_POINT_STRUCT(BSHOTDescriptor, 
									(int[352], bitset, bitset)
									)

template<typename PointInT, typename PointNT, typename PointOutT = BSHOTDescriptor>
class BSHOTEstimation : public FeatureFromNormals<PointInT, PointNT, PointOutT>
{
public:
	typedef typename pcl::Feature<PointInT, PointOutT>::PointCloudOut PointCloudOut;

	/** \brief Empty constructor. */
	BSHOTEstimation()
	{
		this->feature_name_ = "B-SHOT";
	}

private:
	void compute_bshot_from_SHOT(const pcl::SHOT352& shot_in, BSHOTDescriptor& out);
	void computeFeature(PointCloudOut& out) override;
};

#include "impl/bshot.hpp"

#endif