#include "../inc/TestCase.h"
#include "../inc/preprocessing.h"
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <string>
#include <iostream>

static void loadCloud(const std::string& path, const std::string& gt, Cloud& out)
{
	/*
	int p = path.find_last_of('/');
	std::string name = path.substr(p, path.length() - p);
	std::cout << "Loading mesh "<<path<<"...";
	*/
	
	pcl::PolygonMesh mesh;
	pcl::io::loadPolygonFilePLY(path, mesh);

	out.meshes = mesh.polygons; //TODO: this is a copy, but maybe could be a move

	out.points.p = pcl::PointCloud<pcl::PointXYZRGB>::Ptr( new pcl::PointCloud<pcl::PointXYZRGB>() );
	pcl::fromPCLPointCloud2<pcl::PointXYZRGB>(mesh.cloud, *(out.points.p));
}

static void preprocessCloud(Cloud& C)
{
	using namespace Preprocessing;

	C.resolution = computeResolution(C.points.p);
	C.area = computeArea(C);
	C.support_radius = computeSupportRadius(C.area);

	cleanOutliers(C.points.p, C.support_radius); //THIS OPERATION INVALIDATES THE MESH!

	float normal_radius = C.support_radius * Parameters::getNormalRadiusFactor();
	computeNormals(C.points.p, normal_radius, C.points.n);
	cleanNaNNormals(C.points.p, C.points.n);

	extractKeypoints(C.points, C.resolution, C.support_radius, C.keypoints);
}

//----------------------------------
//-------- FROM TESTCASE.H ---------
//----------------------------------
void TestCase::preprocess()
{
	preprocessCloud(scene);
	for(auto c = models.begin(); c != models.end(); ++c)
		preprocessCloud(*c);
}

void TestCase::loadTestCasesFromEXP(const std::string& path, std::vector<TestCase>& out)
{
	std::fstream in(path.c_str(), std::ios_base::in);
	out.clear();

	while(in.good() && !in.eof())
	{
		out.push_back( TestCase() ); 
		TestCase& cur = out.back();
		
		std::string scene_name; int n_models;
		in >> scene_name >> n_models;

		cur.name = scene_name;

		//jump straightly to the next line
		in.ignore(10, '\n');
		
		for (int i = 0; i < n_models; i++)
		{
			cur.models.push_back( Cloud() ); 
			Cloud& model = cur.models.back();

			std::string path_model, gt_path;
			std::getline(in, path_model);
			std::getline(in, gt_path);

			loadCloud(path_model, gt_path, model);
		}

		std::string scene_path; std::getline(in, scene_path);
		loadCloud(scene_path, std::string(), cur.scene);
	}

	in.close();
}