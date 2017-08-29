#include "../inc/preprocessing.h"
#include <pcl/search/kdtree.h>
#include <pcl/PolygonMesh.h>
#include <pcl/filters/radius_outlier_removal.h>

#define OVER_PI 0.318309886

void Preprocessing::cleanOutliers(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& C, float search_radius)
{
	pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> ror(true);

	ror.setInputCloud( C );
	ror.setRadiusSearch( search_radius );
	ror.setMinNeighborsInRadius( Parameters::getMinNeighbors() );
	ror.setNegative(false); //remove anyone with less then K neighbors
	ror.filter( *C );

	std::vector<int> dummy;
	pcl::removeNaNFromPointCloud(*C, *C, dummy);
}

float Preprocessing::computeArea(const Cloud& in)
{
	//We're just summing up the area of all triangles in mesh!
	float acc = 0.0f;

	for (auto f = in.meshes.begin(); f != in.meshes.end(); ++f)
	{
		//TODO: we should do some checking in case polygon is not a triangle!
		Eigen::Vector3f v0, v1, v2;
		v0 = in.points->at(f->vertices[0]).getVector3fMap();
		v1 = in.points->at(f->vertices[1]).getVector3fMap();
		v2 = in.points->at(f->vertices[2]).getVector3fMap();

		//area of this triangle is (v1-v0) X (v2-v0) / 2
		Eigen::Vector3f a = v1 - v0;
		Eigen::Vector3f b = v2 - v0;

		acc += (a.cross(b)).norm() * 0.5f;
	}

	return acc;
}

float Preprocessing::computeResolution(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& in)
{
	//This code was taken from:
	// http://pointclouds.org/documentation/tutorials/correspondence_grouping.php
	double res = 0.0;
	int n_points = 0;
	int nres;
	std::vector<int> indices (2);
	std::vector<float> sqr_distances (2);
	pcl::search::KdTree<pcl::PointXYZRGB> tree;
	tree.setInputCloud (in);

	for (size_t i = 0; i < in->size (); ++i)
	{
		if (! pcl_isfinite ((*in)[i].x)) continue;

		//Considering the second neighbor since the first is the point itself.
		nres = tree.nearestKSearch (i, 2, indices, sqr_distances);
		
		if (nres == 2)
		{
			res += sqrt (sqr_distances[1]);
			++n_points;
		}
	}
		
	if (n_points != 0) res /= n_points;
	
	return res;
}

float Preprocessing::computeSupportRadius(float area)
{
	//we multiply it by 4.5 when we have a 2.5D model, i.e.,
	//a cloud point captured from a single point of view.
	float alpha = Parameters::getSupportRadiusAlpha();
	area *= Parameters::getMeshAreaCorrection(); //Mesh area correction
	return sqrt(area * alpha * OVER_PI);
}