#include "../inc/descriptiveness.h"
#include <pcl/search/kdtree.h>

static bool operator==(const pcl::Correspondence& lhs, const pcl::Correspondence& rhs)
{
	return lhs.index_match == rhs.index_match && lhs.index_query == rhs.index_query;
}

//----------------------------------------
//-------- FROM DESCRIPTIVENESS.h --------
//----------------------------------------
void Descriptiveness::groundtruthCorrespondences(const Cloud& target, Cloud& source)
{
	//threshold is Half of the support radius. We square it
	//not to compute square roots.
	float threshold = target.support_radius * target.support_radius * 0.25f;

	//KD-tree for NN searching inside scene_kp
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGB>());
	kdtree->setInputCloud(target.keypoints.p);

	for(int kp = 0; kp < source.keypoints.p->size(); ++kp)
	{
		pcl::PointXYZRGB p_source = source.keypoints.p->at(kp);

		//p_target is p_source in target space
		Eigen::Vector4f v4_p = source.groundtruth * p_source.getVector4fMap();
		
		pcl::PointXYZRGB p_target;
		p_target.x = v4_p[0]; p_target.y = v4_p[1]; p_target.z = v4_p[2];

		std::vector<int> ind; std::vector<float> dist;
		kdtree->nearestKSearch(p_target, 1, ind, dist);

		if(dist[0] < threshold)
			source.mapToTarget.push_back( pcl::Correspondence(kp, ind[0], dist[0]) );
	}
}

void Descriptiveness::filterCorrectMatches(const pcl::Correspondences& groundtruth, 
										const pcl::Correspondences& matches,
										pcl::Correspondences& out)
{
	for(auto m = matches.begin(); m != matches.end(); ++m)
	{
		//copy-pasted from std::FIND implementation
		auto first = groundtruth.begin();
		while(first != groundtruth.end())
		{
			if(*first == *m)
			{
				out.push_back(*m);
				break;
			}
			++first;
		}
	}
}