template<typename DescType>
void TestCase::descriptorPRC(Descriptiveness::DistanceMetric<DescType> dist, 
								FeatureInitializer<DescType> initFeature,
								pcl::Feature<pcl::PointXYZRGBNormal, DescType>& featureEstimation,
								Descriptiveness::PRC& out)
{
	//We assume preprocess() was already called, so we have
	//the keypoints inside our clouds.
	using namespace Descriptiveness;
	
	out.curve.resize( Parameters::getNSteps() );

	//run descriptiveness evaluation for each model-scene pair
	int id = 0;
	for(auto m = models.begin(); m != models.end(); ++m)
	{
		//if we could not get any groundtruth, we cannot
		//test the descriptor! abort test
		if(m->mapToTarget.empty())
		{
			std::cout<<"\tWARNING: model "<<id++<<" has no computed groundtruth! Skipping.\n";
			return;
		}

		std::cout<<"Evaluating pair <scene, model "<<id++<<">\n";

		//PRC curve for this pair model-scene
		PRC modelPRC;
		evaluateDescriptiveness<DescType>( scene, *m, m->mapToTarget, dist, 
											initFeature, featureEstimation, modelPRC );
		out = out + modelPRC;
	}

	//compute average of PR curves
	out = out * (1.0f/models.size());
}