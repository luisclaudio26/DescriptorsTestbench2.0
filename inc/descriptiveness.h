#ifndef _DESCRIPTIVENESS_H_
#define _DESCRIPTIVENESS_H_

#include "cloud.h"
#include <vector>
#include <string>
#include <pcl/registration/correspondence_estimation.h>

namespace Descriptiveness
{
	//Structure for Precision-Recall curve
	typedef struct PREntry_ { 
		float p, r;
		PREntry_ operator+(PREntry_ rhs) const;
		PREntry_ operator*(float s) const;
	} PREntry;

	typedef struct {
		std::vector<PREntry> curve;
		std::string label;
	} PRC;
	PRC operator+(const PRC& lhs, const PRC& rhs);
	PRC operator*(const PRC& p, float s);

	template<typename DescType>
	using DistanceMetric = float(*)(const DescType& lhs, const DescType& rhs);


	void groundtruthCorrespondences(const Cloud& target, Cloud& source);

	void filterByGroundtruth(const pcl::Correspondences& groundtruth, 
								const pcl::Correspondences& matches,
								pcl::Correspondences& out);

	template<typename DescType>
	void filterByNNDR(const pcl::Correspondences& matches, float NNDR, 
						pcl::Correspondences& out);

	// TODO: some descriptors will use distance metrics other than
	// L2, so we can't always use KdTrees. Because of this, we'll do
	// brute-force search for all descriptors types.
	// In a near future, we can make this a template<typename DescType, bool UseTree>
	// and create two specifications, one using kdtrees and the other using bruteforce
	// search.
	// In a far future, we should implement kdtrees able to handle any
	// distance metric.
	template<typename DescType>
	void correspondenceEstimationNNDR(const typename pcl::PointCloud<DescType>::Ptr& target, 
										const typename pcl::PointCloud<DescType>::Ptr& source,
										DistanceMetric<DescType> distance, pcl::Correspondences& matches);

	template<typename DescType>
	void evaluateDescriptiveness(const Cloud& scene, const Cloud& model, 
									const pcl::Correspondences& groundtruth,
									DistanceMetric<DescType> dist, 
									FeatureInitializer<DescType> initFeature,
									pcl::Feature<pcl::PointXYZRGB, DescType>& featureEstimation,
									PRC& out);
}

#include "impl/descriptiveness.hpp"

#endif