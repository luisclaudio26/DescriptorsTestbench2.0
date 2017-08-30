#include "../inc/cloud.h"
#include "../inc/preprocessing.h"

#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>

static void matrixFromFile(const string& path, Eigen::Matrix4f& out)
{
	//no groundtruth. Simply assign identity
	if(path.empty())
		out<<1,0,0,0,
			 0,1,0,0,
			 0,0,1,0,
			 0,0,0,1;
	else
	{
		std::ifstream reader(path, std::ios_base::in);

		//TODO: check whether file was loaded correctly. If not, abort!
		for (int i = 0; i < 4; i++)
			for (int j = 0; j < 4; j++)
			{
				float temp; reader >> temp;
				out(i, j) = temp;
			}

		reader.close();
	}
}

//------------------------
//----- FROM CLOUD.H -----
//------------------------
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
	matrixFromFile(gt, groundtruth);
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