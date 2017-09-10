#include <limits>
#include "../parameters.h"

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

template<typename DescType>
void Descriptiveness::evaluateDescriptiveness(const Cloud& scene, const Cloud& model, 
												const pcl::Correspondences& groundtruth,
												DistanceMetric<DescType> dist, 
												FeatureInitializer<DescType> initFeature,
												pcl::Feature<pcl::PointXYZRGB, DescType>& featureEstimation,
												PRC& out)
{
	//TODO: THERE'S PROBABLY A BETTER WAY TO DO THINGS INSTEAD
	//OF LOOPING AND CREATING THESE DESCRIPTOR POINT CLOUDS.
	//PROBABLY PUTTING DESCRIPTORS INSIDE THE CLOUD, BUT THIS
	//WOULD REQUIRE TEMPLATING THE CLOUD CLASS, WHICH WOULD
	//BE A PAIN IN THE ASS.
	using namespace Descriptiveness;

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

	//4. Among all_correspondences, select the correct ones according to the groundtruth
	// These will be the union of True Positive and False Negative.
	pcl::Correspondences correct;
	filterByGroundtruth(all_correspondences, groundtruth, correct);

	std::cout<<"Correct correspondences among all correspondences: "<<correct.size()<<std::endl;

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

		//Compute statistics
		float precision = selected_correspondences.size() == 0 ?
							0 : (float)selected_correct.size() / selected_correspondences.size();
		
		float recall = (float)selected_correct.size() / correct.size();

		std::cout<<"\tPrecision = "<<precision*100.0f<<"\%\tRecall = "<<recall*100.0f<<"\%\n";

		out.curve.push_back( (PREntry){precision, recall} );
	}
}