#include "../inc/cloud.h"
#include "../inc/preprocessing.h"

#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>

void Cloud::loadCloud(const std::string& path, const std::string& gt)
{
	std::cout<<"Loading model\n";

	//load model
	pcl::PolygonMesh mesh; pcl::io::loadPolygonFilePLY(path, mesh);

	//initialize point clouds
	points.p = pcl::PointCloud<pcl::PointXYZRGB>::Ptr( new pcl::PointCloud<pcl::PointXYZRGB>() );
	points.n = pcl::PointCloud<pcl::Normal>::Ptr( new pcl::PointCloud<pcl::Normal>() );
	keypoints.p = pcl::PointCloud<pcl::PointXYZRGB>::Ptr( new pcl::PointCloud<pcl::PointXYZRGB>() );
	keypoints.n = pcl::PointCloud<pcl::Normal>::Ptr( new pcl::PointCloud<pcl::Normal>() );

	meshes = mesh.polygons; //TODO: this is a copy, but maybe could be a move
	pcl::fromPCLPointCloud2<pcl::PointXYZRGB>(mesh.cloud, *(points.p));

	//load groundtruth
}

void Cloud::preprocess()
{
	using namespace Preprocessing;

	std::cout<<"Computing basic values\n";
	resolution = computeResolution(points.p);
	area = computeArea(*this);
	support_radius = computeSupportRadius(area);
	std::cout<<"\tRes = "<<resolution<<", support radius = "<<support_radius<<std::endl;

	std::cout<<"Cleaning outliers\n";
	cleanOutliers(points.p, support_radius); //THIS OPERATION INVALIDATES THE MESH!

	std::cout<<"Computing normals\n";
	float normal_radius = resolution * Parameters::getNormalRadiusFactor();
	computeNormals(points.p, normal_radius, points.n);
	cleanNaNNormals(points.p, points.n);

	std::cout<<"Extracting keypoints\n";
	extractKeypoints(points, resolution, support_radius, keypoints);
}