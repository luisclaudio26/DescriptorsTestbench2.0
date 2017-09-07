#include <limits>

template<typename DescType>
void Descriptiveness::filterByNNDR(const pcl::Correspondences& matches, 
									float NNDR, pcl::Correspondences& out)
{

}

template<typename DescType>
void Descriptiveness::correspondenceEstimationNNDR(const typename pcl::PointCloud<DescType>::Ptr& target, 
													const typename pcl::PointCloud<DescType>::Ptr& source,
													DistanceMetric<DescType> distance, pcl::Correspondences& matches)
{
	//Brute-force search for closest pair of descriptors
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