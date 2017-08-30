#ifndef _CLOUD_H_
#define _CLOUD_H_

#include <pcl/PolygonMesh.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/point_types.h>
#include <pcl/features/feature.h>

struct Cloud; typedef struct Cloud Cloud;

//Callbacks of FeatureInitializer type should downcast
//pcl::Feature to the derived class in question and then
//set the parameters using the Cloud.
template<typename PointOutT>
using FeatureInitializer = void(*)(const struct Cloud&, pcl::Feature<pcl::PointXYZRGB,PointOutT>&);

typedef struct {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr p;
	pcl::PointCloud<pcl::Normal>::Ptr n;	
} PointNormal;

struct Cloud {
	PointNormal points, keypoints;
	std::vector<pcl::Vertices> meshes;
	
	float area, resolution, support_radius;

	//if Groundtruth is an identity matrix 
	//(check with .isIdentity()), then it is not
	//valid. It will happen for target clouds
	//(that is, a scene). Otherwise, groundtruth
	//maps this cloud to its position on the scene.
	Eigen::Matrix4f groundtruth;

	// These are the actual, correct correspondences
	// between this clouds keypoints and the target
	// keypoint. This is computed based on the
	// groundtruth transformation.
	pcl::Correspondences mapToTarget;

	//----- Operations -----
	void loadCloud(const std::string& path, const std::string& gt);
	void preprocess();

	template<typename PointOutT>
	void computeDescriptors(FeatureInitializer<PointOutT> initFeature,
							pcl::Feature<pcl::PointXYZRGB,PointOutT>& featureEstimation,
							const typename pcl::PointCloud<PointOutT>::Ptr& out) const;

};

#include "impl/cloud.hpp"

#endif