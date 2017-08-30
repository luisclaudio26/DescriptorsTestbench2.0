#include "../inc/preprocessing.h"
#include <pcl/search/kdtree.h>
#include <pcl/PolygonMesh.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/uniform_sampling.h>

#define OVER_PI 0.318309886

void Preprocessing::extractKeypoints(const PointNormal& points, float resolution, 
									float support_radius, const PointNormal& keypoints)
{
	//Keypoints are stored in a regular cloud. We then pass
	//the original cloud (IN) as the search surface for the
	//keypoints cloud, so features will be computed for
	//keypoints but using all points in the original cloud
	//std::cout << "Extracting keypoints...";

	/*
	//--------- ISS Keypoints
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGB>());
	pcl::ISSKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZRGB> detector;
	
	detector.setInputCloud(in);
	detector.setSearchMethod(kdtree);
	detector.setNormals(normals);
	detector.setNumberOfThreads( Parameters::getNThreads() );

	//these parameters were proposed by Gioia Ballin in http://www.pointclouds.org/blog/gsoc12/gballin/iss.php
	//I didn't read the article proposing the ISS keypoint extractor,
	//so I have no idea why this is correct, but indeed it works nicely.
	detector.setSalientRadius(6 * mesh_resolution );
	detector.setNonMaxRadius(4 * mesh_resolution );
	detector.setNormalRadius(4 * mesh_resolution );
	detector.setBorderRadius(1 * mesh_resolution );
 
	detector.compute(out); */

	pcl::PointCloud<pcl::PointXYZRGB> non_filtered;
	pcl::PointCloud<pcl::Normal> non_filtered_n;

	//--------- Uniform sampling
	pcl::UniformSampling<pcl::PointXYZRGB> scene_voxelgrid;
	scene_voxelgrid.setInputCloud(points.p);
	scene_voxelgrid.setRadiusSearch(Parameters::getUniformSamplingDensity()*resolution);
	scene_voxelgrid.filter(non_filtered);

	//extract normals
	//TODO: HA! PCL IMPLEMENTATION OF UNIFORMSAMPLING WON'T FILL
	//GET REMOVED INDICES. FML. Gotta either fix this here somehow
	//or recompile the whole shit.
	pcl::copyPointCloud(*points.n, *(scene_voxelgrid.getRemovedIndices()), non_filtered_n);

	//---------- Clean keypoint cloud
	//We loop over the keypoints and eliminate all
	//points whose neighbourhood has less then 5
	//neighbours.
	//TODO: There's a function to do this
	pcl::search::KdTree<pcl::PointXYZRGB> kdtree;
	kdtree.setInputCloud(points.p);

	//std::cout<<"Keypoints before: "<<non_filtered.size()<<std::endl;

	int c = 0;
	for(int p_id = 0; p_id < non_filtered.size(); ++p_id)
	{
		pcl::PointXYZRGB& p = non_filtered[p_id];

		std::vector<int> indices; std::vector<float> dist;
		kdtree.radiusSearch(p, support_radius, indices, dist);

		if(indices.size() < 5)
		{
			//std::cout<<std::endl<<++c<<" Keypoint with less then 5 neighbours in support radius!\n";

			//remove point and corresponding normal
			non_filtered.erase(non_filtered.begin() + p_id);
			non_filtered_n.erase(non_filtered_n.begin() + p_id);

			//decrease p_id, so we effectively test the
			//next point in cloud (failing in doing so results
			//in untested points)
			p_id = p_id - 1;
		}
	}

	//copy clouds
	*keypoints.p = non_filtered;
	*keypoints.n = non_filtered_n;

	std::cout<<"N descriptors: "<<keypoints.p->size()<<std::endl;

	/*
	c = 0;
	for(auto p = out_n.begin(); p != out_n.end(); ++p)
		if(pcl::isFinite<pcl::Normal>(*p)) c++;
	std::cout<<c<<" good kp normals out of "<<out_n.size()<<std::endl;

	c = 0;
	for(auto p = out.begin(); p != out.end(); ++p)
		if(pcl::isFinite<pcl::PointXYZRGB>(*p)) c++;
	std::cout<<c<<" good kp out of "<<out.size()<<std::endl;

	c = 0;
	for(auto p = normals->begin(); p != normals->end(); ++p)
		if(pcl::isFinite<pcl::Normal>(*p)) c++;
	std::cout<<c<<" good normals out of "<<normals->size()<<std::endl;

	c = 0;
	for(auto p = in->begin(); p != in->end(); ++p)
		if(pcl::isFinite<pcl::PointXYZRGB>(*p)) c++;
	std::cout<<c<<" good points out of "<<in->size()<<std::endl; */

	//std::cout << "done. " << out.size() << " keypoints extracted." << std::endl << std::endl;

	/*
	std::cout << "These are the keypoints: ";
	for (auto id = out->indices.begin(); id != out->indices.end(); ++id)
		std::cout << *id << ", ";
	std::cout << std::endl << std::endl; 
	*/
}

void Preprocessing::cleanNaNNormals(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& p_cloud, 
									const pcl::PointCloud<pcl::Normal>::Ptr& n_cloud)
{
	/*
	//Cloud cleansing
	std::cout<<"Cleaning NaN normals. Before: "<<n_cloud->size();
	std::vector<int> mapping;
	pcl::removeNaNNormalsFromPointCloud(*n_cloud, *n_cloud, mapping);
	std::cout<<" points. After: "<<n_cloud->size()<<" points."<<std::endl;

	//loop through mapping. Keep track of the different between indexes
	//in mapping. Then difference increases, it means something was removed
	//in that point. Update different, remove point from input cloud.
	std::cout<<"Removing respective points in point cloud...";

	int lastDiff = 0; auto it = p_cloud->begin();
	for(int i = 0; i < mapping.size(); i++, it++)
	{
		if( mapping[i] - i != lastDiff )
		{
			it = p_cloud->erase( it );
			lastDiff++;
		}
	}

	std::cout<<"final cloud has "<<p_cloud->size()<<" points."<<std::endl; */

	for(int n = 0; n < n_cloud->points.size(); n++)
	{
		//skip if normal has no NaN components
		if (pcl_isfinite (n_cloud->points[n].normal_x) && 
			pcl_isfinite (n_cloud->points[n].normal_y) && 
			pcl_isfinite (n_cloud->points[n].normal_z))
			continue;

		//normal is NaN; erase it.
		n_cloud->points.erase( n_cloud->points.begin() + n );
		p_cloud->points.erase( p_cloud->points.begin() + n );

		//we need to decrease n, because after erasing the n-th point,
		//the (n+1)-th point will be in the n-th position, and so we
		//need to analyze it.
		n = n-1;
	}

	p_cloud->height = n_cloud->height = 1;
	p_cloud->width = p_cloud->points.size();
	n_cloud->width = n_cloud->points.size();
}

void Preprocessing::computeNormals(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& in,
									float search_radius, const pcl::PointCloud<pcl::Normal>::Ptr& out)
{
	//TODO: FOR 2.5D views, flip normals towards viewpoint!
	//std::cout << "Computing normals with search radius "<<search_radius<<"...";

	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGB>());
	pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> estimator;
	estimator.setInputCloud(in);
	estimator.setSearchMethod(kdtree);
	estimator.setRadiusSearch(search_radius);
	estimator.setNumberOfThreads(Parameters::getNThreads());
	estimator.compute(*out);

	//std::cout << "done." << std::endl;

	//-------------------------
	//--- Clean NaN normals ---
	//-------------------------
	//Preprocessing::cleanNaNNormals(in, out);
}

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
		v0 = in.points.p->at(f->vertices[0]).getVector3fMap();
		v1 = in.points.p->at(f->vertices[1]).getVector3fMap();
		v2 = in.points.p->at(f->vertices[2]).getVector3fMap();

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