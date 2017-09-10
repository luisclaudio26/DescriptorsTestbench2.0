#ifndef _PREPROCESSING_H_
#define _PREPROCESSING_H_

#include "cloud.h"
#include "parameters.h"

namespace Preprocessing
{
	float computeArea(const Cloud& in);
	float computeResolution(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& in);
	float computeSupportRadius(float area);
	
	void cleanOutliers(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& in, float search_radius);

	void cleanNaNNormals(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud);

	void computeNormals(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud, float search_radius);

	void extractKeypoints(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& points, float resolution, 
							float support_radius, const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& keypoints);
}

#endif