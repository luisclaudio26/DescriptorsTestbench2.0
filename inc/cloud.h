#ifndef _CLOUD_H_
#define _CLOUD_H_

#include <pcl/PolygonMesh.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/point_types.h>

typedef struct {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr p;
	pcl::PointCloud<pcl::Normal>::Ptr n;	
} PointNormal;

typedef struct {
	PointNormal points, keypoints;
	std::vector<pcl::Vertices> meshes;
	
	float area, resolution, support_radius;

	//if Groundtruth is an identity matrix 
	//(check with .isIdentity()), then it is not
	//valid. It will happen for target clouds
	//(that is, a scene). Otherwise, groundtruth
	//maps this cloud to its position on the scene.
	Eigen::Matrix4f groundtruth;

	//----- Operations -----
	void loadCloud(const std::string& path, const std::string& gt);
	void preprocess();
} Cloud;

#endif