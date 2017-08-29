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
	
	out.points = pcl::PointCloud<pcl::PointXYZRGB>::Ptr( new pcl::PointCloud<pcl::PointXYZRGB>() );

	pcl::PolygonMesh mesh;
	pcl::io::loadPolygonFilePLY(path, mesh);

	pcl::fromPCLPointCloud2<pcl::PointXYZRGB>(mesh.cloud, *(out.points));
	out.meshes = mesh.polygons; //TODO: this is a copy, but maybe could be a move
}

static void preprocessCloud(Cloud& C)
{
	using namespace Preprocessing;

	C.resolution = computeResolution(C.points);
	C.area = computeArea(C);
	C.support_radius = computeSupportRadius(C.area);

	//THIS OPERATION INVALIDATES THE MESH!
	cleanOutliers(C.points, C.support_radius);
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