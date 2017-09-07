#include "../inc/TestCase.h"
#include "../inc/preprocessing.h"
#include "../inc/descriptiveness.h"

#include <string>
#include <iostream>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/feature.h>
#include <pcl/search/kdtree.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>

#include <pcl/features/shot_omp.h>
#include <pcl/features/impl/shot_omp.hpp>

using Descriptiveness::DistanceMetric;

template<typename PointOutT>
static void evaluateDescriptiveness(const TestCase& in, 
									DistanceMetric<PointOutT> dist, FeatureInitializer<PointOutT> initFeature,
									pcl::Feature<pcl::PointXYZRGB, PointOutT>& featureEstimation)
{
	//TODO: THERE'S PROBABLY A BETTER WAY TO DO THINGS INSTEAD
	//OF LOOPING AND CREATING THESE DESCRIPTOR POINT CLOUDS.
	//PROBABLY PUTTING DESCRIPTORS INSIDE THE CLOUD, BUT THIS
	//WOULD REQUIRE TEMPLATING THE CLOUD CLASS, WHICH WOULD
	//BE A PAIN IN THE ASS.
	using namespace Descriptiveness;

	//1. Compute descriptors for keypoints in scene
	typename pcl::PointCloud<PointOutT>::Ptr scene_desc( new pcl::PointCloud<PointOutT>() );
	in.scene.computeDescriptors(initFeature, featureEstimation, scene_desc);

	std::cout<<"Computed "<<scene_desc->size()<<" descriptors for the scene\n";

	//2. Compute descriptors for keypoints in each model
	std::vector<typename pcl::PointCloud<PointOutT>::Ptr> models_desc;

	for(auto m = in.models.begin(); m != in.models.end(); ++m)
	{
		typename pcl::PointCloud<PointOutT>::Ptr model_desc( new pcl::PointCloud<PointOutT>() );
		m->computeDescriptors(initFeature, featureEstimation, model_desc);
		models_desc.push_back(model_desc);

		std::cout<<"Computed "<<model_desc->size()<<" descriptors for the model\n";
	}

	//3. Estimate correspondences.
	// determineCorrespondences() will tries to find the point in SOURCE cloud
	// which is the closest to the one in TARGET cloud. This means we end up
	// with as many correspondences as points in target cloud.
	//
	// determineReciprocalCorrespondences() tries to determineCorrespondences()
	// in both directions (source -> target, target -> cloud) and reject all
	// correspondences which do not "agree". Notice that although this seems
	// more precise (and indeed, it discards LOTS of correspondences), a simple
	// test showed that it ended up by rejecting some correct correspondences in the end.
	//
	// TODO: Test which can lead to better results.

	/*
	pcl::registration::CorrespondenceEstimation<PointOutT,PointOutT> corr_estimator;
	corr_estimator.setInputSource(models_desc.back());
	corr_estimator.setInputTarget(scene_desc);

	pcl::CorrespondencesPtr all_correspondences(new pcl::Correspondences());
	corr_estimator.determineCorrespondences(*all_correspondences);
	*/

	pcl::CorrespondencesPtr all_correspondences(new pcl::Correspondences());
	correspondenceEstimationNNDR<PointOutT>(scene_desc, models_desc.back(), dist, *all_correspondences);

	//4. Among all_correspondences, select the correct ones according to the groundtruth
	pcl::Correspondences correct;
	filterByGroundtruth(*all_correspondences, in.models.back().mapToTarget, correct);

	std::cout<<"Correct correspondences among all correspondences: "<<correct.size()<<std::endl;

	//4. Select correspondences based on NNDR. This will be used to create PRC.
	pcl::Correspondences selected_correspondences;
	filterByNNDR<PointOutT>(*all_correspondences, 0.5f, selected_correspondences);

	std::cout<<"Number of correspondences with NNDR < 0.5: "<<selected_correspondences.size()<<std::endl;

	//5. Compute statistics
}

//----------------------------------
//-------- FROM TESTCASE.H ---------
//----------------------------------
template<typename PointOutT>
void initSHOT(const Cloud& in, pcl::Feature<pcl::PointXYZRGB,PointOutT>& desc)
{
	std::cout<<"Initializing shot\n";
	
	typedef pcl::SHOTEstimationOMP<pcl::PointXYZRGB, pcl::Normal, pcl::SHOT352> SHOT_t;

	//downcasting. This is safe if you're not mixing 
	//different descriptors and init functions!!!
	SHOT_t& shot = (SHOT_t&)(desc);

	shot.setRadiusSearch(in.support_radius);
	shot.setNumberOfThreads(4);
	shot.setInputNormals(in.points.n);
}

template<typename PointOutT>
float distSHOT(const PointOutT& lhs, const PointOutT& rhs)
{
	float acc = 0.0f;
	for(int i = 0; i < 352; ++i)
		acc += pow(lhs.descriptor[i]-rhs.descriptor[i], 2.0f);
	return sqrt(acc);
}

void TestCase::descriptiveness(std::vector<PREntry>& out)
{
	//We assume preprocess() was already called, so we have
	//the keypoints inside our clouds.
	using namespace Descriptiveness;

	//Compute groundtruth correspondences
	groundtruthCorrespondences(scene, models.back());

	pcl::SHOTEstimationOMP<pcl::PointXYZRGB, pcl::Normal, pcl::SHOT352> shot;
    evaluateDescriptiveness<pcl::SHOT352>( *this, distSHOT, initSHOT, shot );
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