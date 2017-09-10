#include <iostream>

#include <pcl/features/shot_omp.h>
#include <pcl/features/impl/shot_omp.hpp>

#include <pcl/features/fpfh_omp.h>
#include <pcl/features/impl/fpfh_omp.hpp>

#include <pcl/surface/gp3.h>
#include <pcl/features/rops_estimation.h>

//-----------------------------
//----------- RoPS ------------
//-----------------------------
template<typename PointOutT>
void initROPS(const Cloud& in, pcl::Feature<pcl::PointXYZRGBNormal,PointOutT>& desc)
{
	std::cout<<"Initializing RoPS\n";
	
	typedef pcl::ROPSEstimation<pcl::PointXYZRGBNormal, pcl::Histogram<135>> ROPS_t;

	// unfortunatelly we need to recompute the triangulation
	// for the cloud, because we cleaned it in the preprocessing
	// step and thus the original triangulation is not valid anymore
	//pcl::GreedyProjectionTriangulation<

	// downcasting. This is safe if you're not mixing 
	// different descriptors and init functions!!!
	ROPS_t& rops = (ROPS_t&)(desc);

	rops.setSupportRadius(in.support_radius);
	rops.setRadiusSearch(in.support_radius);
	rops.setNumberOfPartitionBins( 5 );
	rops.setNumberOfRotations( 3 );
	//rops.setTriangles();
}

template<typename PointOutT>
float distROPS(const PointOutT& lhs, const PointOutT& rhs)
{
	float acc = 0.0f;
	for(int i = 0; i < 135; ++i)
		acc += pow(lhs.histogram[i]-rhs.histogram[i], 2.0f);
	return sqrt(acc);
}

//-----------------------------
//----------- SHOT ------------
//-----------------------------
template<typename PointOutT>
void initSHOT(const Cloud& in, pcl::Feature<pcl::PointXYZRGBNormal,PointOutT>& desc)
{
	std::cout<<"Initializing SHOT\n";
	
	typedef pcl::SHOTEstimationOMP<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal, pcl::SHOT352> SHOT_t;

	//downcasting. This is safe if you're not mixing 
	//different descriptors and init functions!!!
	SHOT_t& shot = (SHOT_t&)(desc);

	shot.setRadiusSearch(in.support_radius);
	shot.setNumberOfThreads(4);
	shot.setInputNormals(in.points);
}

template<typename PointOutT>
float distSHOT(const PointOutT& lhs, const PointOutT& rhs)
{
	float acc = 0.0f;
	for(int i = 0; i < 352; ++i)
		acc += pow(lhs.descriptor[i]-rhs.descriptor[i], 2.0f);
	return sqrt(acc);
}

//-----------------------------
//----------- FPFH ------------
//-----------------------------
template<typename PointOutT>
void initFPFH(const Cloud& in, pcl::Feature<pcl::PointXYZRGBNormal,PointOutT>& desc)
{
	std::cout<<"Initializing FPFH\n";
	
	typedef pcl::FPFHEstimationOMP<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal, pcl::FPFHSignature33> FPFH_t;

	//downcasting. This is safe if you're not mixing 
	//different descriptors and init functions!!!
	FPFH_t& fpfh = (FPFH_t&)(desc);

	fpfh.setRadiusSearch(in.support_radius);
	fpfh.setNumberOfThreads(4);
	fpfh.setInputNormals(in.points);
}

template<typename PointOutT>
float distFPFH(const PointOutT& lhs, const PointOutT& rhs)
{
	float acc = 0.0f;
	for(int i = 0; i < 33; ++i)
		acc += pow(lhs.histogram[i]-rhs.histogram[i], 2.0f);
	return sqrt(acc);
}