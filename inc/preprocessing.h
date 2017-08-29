#ifndef _PREPROCESSING_H_
#define _PREPROCESSING_H_

#include "cloud.h"
#include "parameters.h"

namespace Preprocessing
{
	float computeArea(const Cloud& in);
	float computeResolution(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& in);
	float computeSupportRadius(float area);
	void cleanOutliers(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& in, float search_radius);
}

#endif