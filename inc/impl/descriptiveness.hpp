#include <limits>
#include "../parameters.h"

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/search/kdtree.h>

template<typename DescType>
void Descriptiveness::filterByNNDR(const pcl::Correspondences& matches, 
									float NNDR, pcl::Correspondences& out)
{
	for(auto m = matches.begin(); m != matches.end(); ++m)
		if(m->distance < NNDR) out.push_back(*m);
}

template<typename DescType>
void Descriptiveness::correspondenceEstimationNNDR(const typename pcl::PointCloud<DescType>::Ptr& target, 
													const typename pcl::PointCloud<DescType>::Ptr& source,
													DistanceMetric<DescType> distance, pcl::Correspondences& matches)
{
	//Brute-force search for closest pair of descriptors,
	//in a non-reciprocal fashion.
	//
	//TODO: I think it is possible to do this in one loop
	//by looping over it, storing the first and second NN;
	//when a closer point is found, second = first and
	//first = newFirst.
	for(int t_id = 0; t_id < target->size(); ++t_id)
	{
		DescType &t = target->at(t_id);

		int closest1st, closest2nd; 
		float dist_1st, dist_2nd; 
		dist_1st = dist_2nd = std::numeric_limits<float>::max();
		
		//1st nearest neighbor
		for(int s_id = 0; s_id < source->size(); ++s_id)
		{
			DescType &s = source->at(s_id);
			float dist = distance(t,s);

			if(dist < dist_1st) 
			{
				closest1st = s_id;
				dist_1st = dist;
			}
		}

		//2nd nearest neighbor
		for(int s_id = 0; s_id < source->size(); ++s_id)
		{
			DescType &s = source->at(s_id);
			float dist = distance(t,s);

			if(s_id != closest1st && dist < dist_2nd)
			{
				closest2nd = s_id;
				dist_2nd = dist;
			}
		}

		float NNDR = dist_1st / dist_2nd;
		matches.push_back( pcl::Correspondence(closest1st, t_id, NNDR) ); //TODO: I'm not sure about the order!!!
	}
}


static void visualizePatchCorrespondence(const Cloud& scene, const Cloud& model, const pcl::Correspondence& c)
{
	pcl::visualization::PCLVisualizer viewer;
	viewer.setBackgroundColor(0.0f, 0.0f, 0.1f);

	typedef pcl::search::KdTree<pcl::PointXYZRGBNormal> Tree;
	typedef pcl::PointCloud<pcl::PointXYZRGBNormal> RGBCloud;

	//--------------------------------------
	//------------ TARGET CLOUD ------------
	//--------------------------------------
	//extract patch
	Tree tree_s; tree_s.setInputCloud(scene.points);
	std::vector<int> indices_s; std::vector<float> dist_s;
	tree_s.radiusSearch( scene.keypoints->at(c.index_match), scene.support_radius, indices_s, dist_s );
	RGBCloud::Ptr patch_s(new RGBCloud()); copyPointCloud( *scene.points, indices_s, *patch_s );

	for(auto p = patch_s->begin(); p != patch_s->end(); ++p)
		p->r = p->g = p->b = 255;

	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal> rgb_scene(patch_s);
	viewer.addPointCloud<pcl::PointXYZRGBNormal>(patch_s, rgb_scene, "scene");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.0f, "scene");

	//--------------------------------------
	//------------ SOURCE CLOUD ------------
	//--------------------------------------
	Tree tree_m; tree_m.setInputCloud(model.points);
	std::vector<int> indices_m; std::vector<float> dist_m;
	tree_m.radiusSearch( model.keypoints->at(c.index_match), model.support_radius, indices_m, dist_m );
	RGBCloud::Ptr patch_m(new RGBCloud()); copyPointCloud( *model.points, indices_m, *patch_m );

	for(auto p = patch_m->begin(); p != patch_m->end(); ++p) {
		p->g = 255; p->r = p->b = 0;
	}

	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal> rgb_model(patch_m);
	viewer.addPointCloud<pcl::PointXYZRGBNormal>(patch_m, rgb_model, "model");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.0f, "model");

	//render
	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::milliseconds(10));
	}
}


template<typename DescType>
void Descriptiveness::evaluateDescriptiveness(const Cloud& scene, const Cloud& model, 
												const pcl::Correspondences& groundtruth,
												DistanceMetric<DescType> dist, 
												FeatureInitializer<DescType> initFeature,
												pcl::Feature<pcl::PointXYZRGBNormal, DescType>& featureEstimation,
												PRC& out)
{
	//TODO: THERE'S PROBABLY A BETTER WAY TO DO THINGS INSTEAD
	//OF LOOPING AND CREATING THESE DESCRIPTOR POINT CLOUDS.
	//PROBABLY PUTTING DESCRIPTORS INSIDE THE CLOUD, BUT THIS
	//WOULD REQUIRE TEMPLATING THE CLOUD CLASS, WHICH WOULD
	//BE A PAIN IN THE ASS.
	using namespace Descriptiveness;

	//if one of the clouds is empty, skip this case
	if(scene.keypoints->empty() || model.keypoints->empty())
	{
		std::cerr<<"\tWARNING: skipping descriptiveness evaluation because scene or model keypoint cloud is empty!\n";
		return;
	}

	//1. Compute descriptors for keypoints in scene
	typename pcl::PointCloud<DescType>::Ptr scene_desc( new pcl::PointCloud<DescType>() );
	scene.computeDescriptors(initFeature, featureEstimation, scene_desc);
	
	std::cout<<"Computed "<<scene_desc->size()<<" descriptors for the scene\n";

	//2. Compute descriptors for keypoints in model
	typename pcl::PointCloud<DescType>::Ptr model_desc( new pcl::PointCloud<DescType>() );
	model.computeDescriptors(initFeature, featureEstimation, model_desc);

	std::cout<<"Computed "<<model_desc->size()<<" descriptors for the model\n";

	//3. Estimate correspondences
	pcl::Correspondences all_correspondences;
	correspondenceEstimationNNDR<DescType>(scene_desc, model_desc, dist, all_correspondences);

	std::cout<<"Correct correspondences among all correspondences: "<<groundtruth.size()<<std::endl;

	//4. Select correspondences based on NNDR. This will be used to create PRC.
	int n = Parameters::getNSteps();
	for(int i = 1; i <= n; ++i)
	{
		float tau = i / (float)n;

		pcl::Correspondences selected_correspondences;
		filterByNNDR<DescType>(all_correspondences, tau, selected_correspondences);

		std::cout<<"\n\tNumber of correspondences with NNDR < "<<tau<<": "<<selected_correspondences.size()<<std::endl;

		//Having selected our correspondences, filter then using the groundtruth
		pcl::Correspondences selected_correct;
		filterByGroundtruth(selected_correspondences, groundtruth, selected_correct);

		std::cout<<"\tNumber of correct correspondences with NNDR < "<<tau<<": "<<selected_correct.size()<<std::endl;

		//visualize random patch pair
		static bool shown = false;
		if(!shown && tau > 0.9f)
		{
			int ind = rand() % selected_correspondences.size();
			visualizePatchCorrespondence(scene, model, selected_correspondences[ind]);
			shown = true;
		}

		//Compute statistics
		float precision = selected_correspondences.size() == 0 ?
							0 : (float)selected_correct.size() / selected_correspondences.size();
		
		float recall = (float)selected_correct.size() / groundtruth.size();

		std::cout<<"\tPrecision = "<<precision*100.0f<<"\%\tRecall = "<<recall*100.0f<<"\%\n";

		out.curve.push_back( (PREntry){precision, recall} );
	}
}