#include "../inc/TestCase.h"
#include "../inc/preprocessing.h"
#include "../inc/callbacks.h"

#include <string>
#include <iostream>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/feature.h>
#include <pcl/search/kdtree.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>

//----------------------------------
//-------- FROM TESTCASE.H ---------
//----------------------------------
void TestCase::descriptiveness(std::vector<Descriptiveness::PRC>& out)
{
	//We assume preprocess() was already called, so we have
	//the keypoints inside our clouds.
	using namespace Descriptiveness;

	//precompute groundtruths
	for(auto m = models.begin(); m != models.end(); ++m)
		groundtruthCorrespondences(scene, *m);

	//benchmark each descriptor
	// TODO: Think of a better way of doing this =/
	// apparently, there's Hana library which can do
	// a for_each over a std::tuple
	PRC prcSHOT;
	pcl::SHOTEstimationOMP<pcl::PointXYZRGB, pcl::Normal, pcl::SHOT352> shot;
	descriptorPRC(distSHOT, initSHOT, shot, prcSHOT);
	out.push_back( prcSHOT ); prcSHOT.label = "SHOT";

	/*
	PRC prcFPFH;
	pcl::FPFHEstimationOMP<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> fpfh;
	descriptorPRC(distFPFH, initFPFH, fpfh, prcFPFH);
	out.push_back( prcFPFH ); prcFPFH.label = "FPFH";
	*/
}

void TestCase::visualize()
{
	pcl::visualization::PCLVisualizer viewer;
	viewer.setBackgroundColor(0.0f, 0.0f, 0.1f);

	//--------------------------------------
	//------------ TARGET CLOUD ------------
	//--------------------------------------
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

	//--------------------------------------
	//------------ SOURCE CLOUD ------------
	//--------------------------------------
	Cloud& model = models.back();

	//points
	for(auto p = model.points.p->begin(); p != model.points.p->end(); ++p)
		p->r = p->g = p->b = 128;

	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb_model(model.points.p);
	viewer.addPointCloud<pcl::PointXYZRGB>(model.points.p, rgb_model, "model");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.0f, "model");

	//keypoints	
	for(auto p = model.keypoints.p->begin(); p != model.keypoints.p->end(); ++p) {
		p->r = p->b = 0;
		p->g = 255;
	}

	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb_model_kp(model.keypoints.p);
	viewer.addPointCloud<pcl::PointXYZRGB>(model.keypoints.p, rgb_model_kp, "model_kp");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3.5f, "model_kp");

	//Won't work :( Check this after
	viewer.addCorrespondences<pcl::PointXYZRGB>(model.keypoints.p, scene.keypoints.p, model.mapToTarget);

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