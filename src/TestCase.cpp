#include "../inc/TestCase.h"
#include "../inc/preprocessing.h"

#include <string>
#include <iostream>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/feature.h>
#include <pcl/search/kdtree.h>

#include <pcl/features/shot_omp.h>
#include <pcl/features/impl/shot_omp.hpp>

//Callbacks of FeatureInitializer type should downcast
//pcl::Feature to the derived class in question and then
//set the parameters using the Cloud.
template<typename PointOutT>
using FeatureInitializer = void(*)(const Cloud&, pcl::Feature<pcl::PointXYZRGB,PointOutT>&);

template<typename PointOutT>
static void computeDescriptors(const Cloud& in, FeatureInitializer<PointOutT> initFeature,
								pcl::Feature<pcl::PointXYZRGB,PointOutT>& featureEstimation,
								const typename pcl::PointCloud<PointOutT>::Ptr& out)
{
	//set descriptor engine parameters
	initFeature(in, featureEstimation);

	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGB>());

	//general parameters
	featureEstimation.setSearchMethod(kdtree);
	featureEstimation.setInputCloud(in.keypoints.p);
	featureEstimation.setSearchSurface(in.points.p);
	
	featureEstimation.compute(*out);
}

template<typename PointOutT>
static void benchmarkDescriptor(const TestCase& in, FeatureInitializer<PointOutT> initFeature,
								pcl::Feature<pcl::PointXYZRGB, PointOutT>& featureEstimation)
{
	//1. Compute descriptors for keypoints in scene
	typename pcl::PointCloud<PointOutT>::Ptr scene_desc( new pcl::PointCloud<PointOutT>() );
	computeDescriptors(in.scene, initFeature, featureEstimation, scene_desc);

	std::cout<<"Computed "<<scene_desc->size()<<" descriptors for the scene\n";

	//2. Compute descriptors for keypoints in each model
}

//----------------------------------
//-------- FROM TESTCASE.H ---------
//----------------------------------
template<typename PointOutT>
void initSHOT(const Cloud& in, pcl::Feature<pcl::PointXYZRGB,PointOutT>& desc)
{
	std::cout<<"Initializing shot\n";
	
	//auto shot = dynamic_cast<pcl::SHOTEstimationOMP<pcl::PointXYZRGB, pcl::Normal, pcl::SHOT352>&>(desc);	
	
	pcl::SHOTEstimationOMP<pcl::PointXYZRGB, pcl::Normal, pcl::SHOT352>& shot = 
		(pcl::SHOTEstimationOMP<pcl::PointXYZRGB, pcl::Normal, pcl::SHOT352>&)(desc);

	shot.setRadiusSearch(in.resolution * 3.0f);
	shot.setNumberOfThreads(4);
	shot.setInputNormals(in.points.n);
}

void TestCase::descriptiveness(std::vector<PREntry>& out)
{
	//We assume preprocess() was already called, so we have
	//the keypoints inside our clouds.
	pcl::SHOTEstimationOMP<pcl::PointXYZRGB, pcl::Normal, pcl::SHOT352> shot;
    benchmarkDescriptor<pcl::SHOT352>( *this, initSHOT, shot );
}

void TestCase::visualize()
{
	pcl::visualization::PCLVisualizer viewer;
	viewer.setBackgroundColor(0.0f, 0.0f, 0.1f);

	//points
	for(auto p = scene.points.p->begin(); p != scene.points.p->end(); ++p)
		p->r = p->g = p->b = 255;

	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb_scene(scene.points.p);
	viewer.addPointCloud<pcl::PointXYZRGB>(scene.points.p, rgb_scene, "scene");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.0f, "scene");

	//keypoints	
	for(auto p = scene.keypoints.p->begin(); p != scene.keypoints.p->end(); ++p) {
		p->r = p->b = 0;
		p->g = 255;
	}

	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb_scene_kp(scene.keypoints.p);
	viewer.addPointCloud<pcl::PointXYZRGB>(scene.keypoints.p, rgb_scene_kp, "scene_kp");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3.5f, "scene_kp");

	//Won't work :( Check this after
	//viewer.addCorrespondences<pcl::PointXYZRGB>(scene, model, correspondences);

	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::milliseconds(10));
	}
}

void TestCase::preprocess()
{
	scene.preprocess();
	for(auto c = models.begin(); c != models.end(); ++c)
		c->preprocess();
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

			model.loadCloud(path_model, gt_path);
		}

		std::string scene_path; std::getline(in, scene_path);
		cur.scene.loadCloud(scene_path, std::string());
	}

	in.close();
}