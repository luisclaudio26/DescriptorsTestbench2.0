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

	void cleanNaNNormals(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& p_cloud, 
							const pcl::PointCloud<pcl::Normal>::Ptr& n_cloud);

	void computeNormals(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& in, float search_radius, 
						const pcl::PointCloud<pcl::Normal>::Ptr& out);

	void extractKeypoints(const PointNormal& point, float resolution, float support_radius,
							const PointNormal& keypoints);
}

#endif