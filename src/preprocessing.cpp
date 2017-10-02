#include "../inc/preprocessing.h"

#include <pcl/search/kdtree.h>
#include <pcl/PolygonMesh.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/filters/shadowpoints.h>
#include <pcl/common/common.h>

#define OVER_PI 0.318309886

static void keypointsISS(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& points, 
							float resolution, float support_radius, 
							const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& keypoints)
{
	pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGBNormal>());
	pcl::ISSKeypoint3D<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> detector;
	
	detector.setSearchMethod(kdtree);
	detector.setInputCloud(points);
	detector.setNormals(points);
	detector.setNumberOfThreads( Parameters::getNThreads() );

	// these parameters were proposed by Gioia Ballin in 
	// http://www.pointclouds.org/blog/gsoc12/gballin/iss.php
	// I didn't read the article proposing the ISS keypoint extractor,
	// so I have no idea why this is correct, but indeed it works nicely.
	detector.setSalientRadius( support_radius );
	detector.setNonMaxRadius( support_radius );
	detector.setNormalRadius( resolution * Parameters::getNormalRadiusFactor() );
	detector.setBorderRadius( 1.0f * resolution );
 
	detector.compute( *keypoints );
}

static void keypointsUniformSampling(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& points, 
										float resolution, float support_radius, 
										const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& keypoints)
{	
	pcl::UniformSampling<pcl::PointXYZRGBNormal> scene_voxelgrid;
	scene_voxelgrid.setInputCloud(points);
	scene_voxelgrid.setRadiusSearch(Parameters::getUniformSamplingDensity() * resolution);
	scene_voxelgrid.filter( *keypoints );
}

static void cameraSpaceUniformSampling(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& points, 
										float resolution, float support_radius, 
										const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& keypoints)
{
	float kp_per_res = 10.0f;

	//get bounds
	pcl::PointXYZRGBNormal min, max;
	pcl::getMinMax3D<pcl::PointXYZRGBNormal>( *points, min, max);
	float dx = max.x - min.x, dy = max.y - min.y;

	int m = dx / resolution / kp_per_res;
	int n = dy / resolution / kp_per_res;

	float over_width = m/dx, over_height = n/dy;

	//compute grid N x N
	std::vector<std::vector<pcl::PointXYZRGBNormal>> grid;
	grid.resize(m*n);

	for(auto p = points->begin(); p != points->end(); ++p)
	{
		int i = (int)((p->x - min.x) * over_width);
		int j = (int)((p->y - min.y) * over_height);

		if( i == m ) i--; if( j == n ) j--;
		grid[j*m + i].push_back(*p);
	}

	//pick a random keypoint at each cell
	for(auto c = grid.begin(); c != grid.end(); ++c)
		if(!c->empty())	keypoints->push_back( c->at(0) );
}

//------------------------------------------
//---------- FROM PREPROCESSING.H ----------
//------------------------------------------
void Preprocessing::extractKeypoints(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& points, 
									float resolution, float support_radius, 
									const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& keypoints)
{
	switch( Parameters::getKeypointMethod() )
	{
	case Parameters::UNIFORM_SAMPLING:
		keypointsUniformSampling(points, resolution, support_radius, keypoints);
		break;
	case Parameters::ISS:
		keypointsISS(points, resolution, support_radius, keypoints);
		break;
	case Parameters::CAMERA_SPACE:
		cameraSpaceUniformSampling(points, resolution, support_radius, keypoints);
		break;
	}
}

void Preprocessing::cleanNaNNormals(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud)
{
	std::vector<int> dummy;
	pcl::removeNaNNormalsFromPointCloud( *cloud, *cloud, dummy );
}

void Preprocessing::computeNormals(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud, 
									float search_radius)
{
	pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGBNormal>());
	pcl::NormalEstimationOMP<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> estimator;
	
	estimator.setInputCloud(cloud);
	estimator.setSearchMethod(kdtree);
	estimator.setRadiusSearch(search_radius);
	estimator.setNumberOfThreads(Parameters::getNThreads());
	estimator.compute(*cloud);
}

void Preprocessing::cleanOutliers(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& C, float search_radius)
{
	pcl::RadiusOutlierRemoval<pcl::PointXYZRGBNormal> ror(true);

	ror.setInputCloud( C );
	ror.setRadiusSearch( search_radius );
	ror.setMinNeighborsInRadius( Parameters::getMinNeighbors() );
	ror.setNegative(false); //remove anyone with less then K neighbors
	ror.filter( *C );

	std::vector<int> dummy;
	pcl::removeNaNFromPointCloud(*C, *C, dummy);
}

void Preprocessing::cleanShadow(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud, float threshold)
{
	//ShadowPoints cannot be done in-place
	pcl::PointCloud<pcl::PointXYZRGBNormal> out;

	pcl::ShadowPoints<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> shadow;
	
	shadow.setInputCloud(cloud); 
	shadow.setNormals(cloud);
	shadow.setThreshold( 0.1f );
	
	shadow.filter( out );

	//copy clean cloud back
	*cloud = out;
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

float Preprocessing::computeResolution(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& in)
{
	//This code was taken from:
	// http://pointclouds.org/documentation/tutorials/correspondence_grouping.php
	double res = 0.0;
	int n_points = 0;
	int nres;
	std::vector<int> indices (2);
	std::vector<float> sqr_distances (2);
	pcl::search::KdTree<pcl::PointXYZRGBNormal> tree;
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