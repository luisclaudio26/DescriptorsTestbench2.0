#include "../inc/TestCase.h"
#include <pcl/search/kdtree.h>
#include <string>
#include <iostream>

#define OVER_PI 0.318309886

static float computeArea(const Cloud& in)
{
	//We're just summing up the area of all triangles in mesh!
	float acc = 0.0f;

	for (auto f = in.meshes.begin(); f != in.meshes.end(); ++f)
	{
		//TODO: we should do some checking in case polygon is not a triangle!
		Eigen::Vector3f v0, v1, v2;
		v0 = in.points->at(f->vertices[0]).getVector3fMap();
		v1 = in.points->at(f->vertices[1]).getVector3fMap();
		v2 = in.points->at(f->vertices[2]).getVector3fMap();

		//area of this triangle is (v1-v0) X (v2-v0) / 2
		Eigen::Vector3f a = v1 - v0;
		Eigen::Vector3f b = v2 - v0;

		acc += (a.cross(b)).norm() * 0.5f;
	}

	return acc;
}

static float computeResolution(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& in)
{
	//This code was taken from:
	// http://pointclouds.org/documentation/tutorials/correspondence_grouping.php
	double res = 0.0;
	int n_points = 0;
	int nres;
	std::vector<int> indices (2);
	std::vector<float> sqr_distances (2);
	pcl::search::KdTree<pcl::PointXYZRGB> tree;
	tree.setInputCloud (in);

	for (size_t i = 0; i < in->size (); ++i)
	{
		if (! pcl_isfinite ((*in)[i].x)) continue;

		//Considering the second neighbor since the first is the point itself.
		nres = tree.nearestKSearch (i, 2, indices, sqr_distances);
		
		if (nres == 2)
		{
			res += sqrt (sqr_distances[1]);
			++n_points;
		}
	}
		
	if (n_points != 0) res /= n_points;
	
	return res;
}

static float computeSupportRadius(float area)
{
	//we multiply it by 4.5 when we have a 2.5D model, i.e.,
	//a cloud point captured from a single point of view.
	float alpha = 0.14f;
	area *= 4.5f; //Mesh area correction
	return sqrt(area * alpha * OVER_PI);
}

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

	out.resolution = computeResolution(out.points);
	out.area = computeArea(out);
	out.support_radius = computeSupportRadius(out.area);
}

//----------------------------------
//-------- FROM TESTCASE.H ---------
//----------------------------------
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