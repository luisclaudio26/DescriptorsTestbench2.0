#ifndef _RANDOM_DESCRIPTOR_H_
#define _RANDOM_DESCRIPTOR_H_

#include <pcl/point_types.h>
#include <pcl/features/feature.h>
#include <pcl/features/impl/feature.hpp>
using pcl::FeatureFromNormals;

struct DummyDesc
{
	int dummy;
};

POINT_CLOUD_REGISTER_POINT_STRUCT(DummyDesc, 
									(int, dummy, dummy)
									) 

template<typename PointInT, typename PointOutT>
class RandomGuessDescriptor : public pcl::Feature<PointInT, PointOutT>
{
public:
	typedef typename pcl::Feature<PointInT, PointOutT>::PointCloudOut PointCloudOut;

	/** \brief Empty constructor. */
	RandomGuessDescriptor()
	{
		this->feature_name_ = "RANDOMGUESS";
	}

private:
	void computeFeature(PointCloudOut& out) override
	{
		return;
	}
};

#endif