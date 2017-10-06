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

	PRC prcDRINK; prcDRINK.label = "NAPS";
	DRINK3Estimation<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal, DRINKSignature> drink;
	descriptorPRC(distDRINK, initDRINK, drink, prcDRINK);
	out.push_back( prcDRINK );

	/*
	PRC prcRANDOM; prcRANDOM.label = "Random guess";
	RandomGuessDescriptor<pcl::PointXYZRGBNormal, DummyDesc> random;
	descriptorPRC(distRANDOM, initRANDOM, random, prcRANDOM);
	out.push_back( prcRANDOM );

	PRC prcBSHOT; prcBSHOT.label = "B-SHOT";
	BSHOTEstimation<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal, BSHOTDescriptor> bshot;
	descriptorPRC(distBSHOT, initBSHOT, bshot, prcBSHOT);
	out.push_back( prcBSHOT );

	PRC prcSHOT; prcSHOT.label = "SHOT";
	pcl::SHOTEstimationOMP<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal, pcl::SHOT352> shot;
	descriptorPRC(distSHOT, initSHOT, shot, prcSHOT);
	out.push_back( prcSHOT );

	PRC prcROPS; prcROPS.label = "RoPS";
	pcl::ROPSEstimation<pcl::PointXYZRGBNormal, pcl::Histogram<135>> rops;
	descriptorPRC(distROPS, initROPS, rops, prcROPS);
	out.push_back( prcROPS );

	PRC prcFPFH; prcFPFH.label = "FPFH";
	pcl::FPFHEstimationOMP<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal, pcl::FPFHSignature33> fpfh;
	descriptorPRC(distFPFH, initFPFH, fpfh, prcFPFH);
	out.push_back( prcFPFH );

	PRC prcUSC; prcUSC.label = "USC";
	pcl::UniqueShapeContext<pcl::PointXYZRGBNormal> usc;
	descriptorPRC(distUSC, initUSC, usc, prcUSC);
	out.push_back( prcUSC );
	*/
}

void TestCase::visualize(int pair)
{
	pcl::visualization::PCLVisualizer viewer;
	viewer.setBackgroundColor(0.0f, 0.0f, 0.1f);

	//--------------------------------------
	//------------ TARGET CLOUD ------------
	//--------------------------------------
	//points
	/*
	for(auto p = scene.points->begin(); p != scene.points->end(); ++p)
		p->r = p->g = p->b = 255;
	*/

	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal> rgb_scene(scene.points);
	viewer.addPointCloud<pcl::PointXYZRGBNormal>(scene.points, rgb_scene, "scene");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.0f, "scene");

	//keypoints	
	for(auto p = scene.keypoints->begin(); p != scene.keypoints->end(); ++p) {
		p->r = p->b = 0;
		p->g = 255;
	}

	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal> rgb_scene_kp(scene.keypoints);
	viewer.addPointCloud<pcl::PointXYZRGBNormal>(scene.keypoints, rgb_scene_kp, "scene_kp");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3.5f, "scene_kp");

	//--------------------------------------
	//------------ SOURCE CLOUD ------------
	//--------------------------------------
	Cloud& model = models[pair];

	//points
	/*
	for(auto p = model.points->begin(); p != model.points->end(); ++p)
		p->r = p->g = p->b = 128;
	*/

	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal> rgb_model(model.points);
	viewer.addPointCloud<pcl::PointXYZRGBNormal>(model.points, rgb_model, "model");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.0f, "model");

	//keypoints	
	for(auto p = model.keypoints->begin(); p != model.keypoints->end(); ++p) {
		p->r = p->b = 0;
		p->g = 255;
	}

	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal> rgb_model_kp(model.keypoints);
	viewer.addPointCloud<pcl::PointXYZRGBNormal>(model.keypoints, rgb_model_kp, "model_kp");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3.5f, "model_kp");

	//correspondences lines
	viewer.addCorrespondences<pcl::PointXYZRGBNormal>(model.keypoints, scene.keypoints, model.mapToTarget);

	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::milliseconds(10));
	}
}

void TestCase::preprocess()
{
	std::cout<<"--- Scene\n";
	scene.preprocess();

	int id = 0;
	for(auto c = models.begin(); c != models.end(); ++c)
	{
		std::cout<<"\n--- Model "<<id++<<std::endl;
		c->preprocess();
	}
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