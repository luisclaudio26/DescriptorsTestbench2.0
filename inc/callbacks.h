#include <iostream>

#include <pcl/features/shot_omp.h>
#include <pcl/features/impl/shot_omp.hpp>

#include <pcl/features/fpfh_omp.h>
#include <pcl/features/impl/fpfh_omp.hpp>

//-----------------------------
//----------- SHOT ------------
//-----------------------------
template<typename PointOutT>
void initSHOT(const Cloud& in, pcl::Feature<pcl::PointXYZRGB,PointOutT>& desc)
{
	std::cout<<"Initializing SHOT\n";
	
	typedef pcl::SHOTEstimationOMP<pcl::PointXYZRGB, pcl::Normal, pcl::SHOT352> SHOT_t;

	//downcasting. This is safe if you're not mixing 
	//different descriptors and init functions!!!
	SHOT_t& shot = (SHOT_t&)(desc);

	shot.setRadiusSearch(in.support_radius);
	shot.setNumberOfThreads(4);
	shot.setInputNormals(in.points.n);
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
void initFPFH(const Cloud& in, pcl::Feature<pcl::PointXYZRGB,PointOutT>& desc)
{
	std::cout<<"Initializing FPFH\n";
	
	typedef pcl::FPFHEstimationOMP<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> FPFH_t;

	//downcasting. This is safe if you're not mixing 
	//different descriptors and init functions!!!
	FPFH_t& fpfh = (FPFH_t&)(desc);

	fpfh.setRadiusSearch(in.support_radius);
	fpfh.setNumberOfThreads(4);
	fpfh.setInputNormals(in.points.n);
}

template<typename PointOutT>
float distFPFH(const PointOutT& lhs, const PointOutT& rhs)
{
	float acc = 0.0f;
	for(int i = 0; i < 33; ++i)
		acc += pow(lhs.histogram[i]-rhs.histogram[i], 2.0f);
	return sqrt(acc);
}