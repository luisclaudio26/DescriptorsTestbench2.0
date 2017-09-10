#include "../inc/descriptiveness.h"
#include <pcl/search/kdtree.h>
#include <iostream>

static bool operator==(const pcl::Correspondence& lhs, const pcl::Correspondence& rhs)
{
	return lhs.index_match == rhs.index_match && lhs.index_query == rhs.index_query;
}

//----------------------------------------
//-------- FROM DESCRIPTIVENESS.h --------
//----------------------------------------
namespace Descriptiveness
{
	PREntry_ PREntry_::operator+(PREntry_ rhs) const
	{
		PREntry_ out;
		out.p = this->p + rhs.p;
		out.r = this->r + rhs.r;
		return out;
	}

	PREntry_ PREntry_::operator*(float s) const
	{
		PREntry_ out;
		out.p = this->p * s;
		out.r = this->r * s;
		return out;
	}

	PRC operator+(PRC& lhs, PRC& rhs)
	{
		if(lhs.curve.size() != rhs.curve.size())
		{
			std::cerr<<"\tWARNING: Adding PRCs with different sizes. Extending the smallest one.\n";
			int max_size = std::max( lhs.curve.size(), rhs.curve.size() );
			lhs.curve.resize(max_size);
			rhs.curve.resize(max_size);
		}
		
		PRC out;
		
		for(int i = 0; i < lhs.curve.size(); ++i)
			out.curve.push_back( lhs.curve.at(i) + rhs.curve.at(i) );
		
		out.label = lhs.label + rhs.label;
		
		return out; //capture by move semantics
	}

	PRC operator*(const PRC& p, float s)
	{
		PRC out;
		for(int i = 0; i < p.curve.size(); ++i)
			out.curve.push_back( p.curve.at(i) * s );
		out.label = p.label;
		return out; //capture by move semantics
	}
}

void Descriptiveness::groundtruthCorrespondences(const Cloud& target, Cloud& source)
{
	//threshold is Half of the support radius. We square it
	//not to compute square roots.
	float threshold = target.support_radius * target.support_radius * 0.25f;

	//KD-tree for NN searching inside scene_kp
	pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGBNormal>());
	kdtree->setInputCloud(target.keypoints);

	for(int kp = 0; kp < source.keypoints->size(); ++kp)
	{
		pcl::PointXYZRGBNormal p_source = source.keypoints->at(kp);

		//p_target is p_source in target space
		Eigen::Vector4f v4_p = source.groundtruth * p_source.getVector4fMap();
		
		pcl::PointXYZRGBNormal p_target;
		p_target.x = v4_p[0]; p_target.y = v4_p[1]; p_target.z = v4_p[2];

		std::vector<int> ind; std::vector<float> dist;
		kdtree->nearestKSearch(p_target, 1, ind, dist);

		if(dist[0] < threshold)
			source.mapToTarget.push_back( pcl::Correspondence(kp, ind[0], dist[0]) );
	}
}

void Descriptiveness::filterByGroundtruth(const pcl::Correspondences& groundtruth, 
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