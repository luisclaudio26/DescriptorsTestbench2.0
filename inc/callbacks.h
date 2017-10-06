#include <iostream>

#include <pcl/features/shot_omp.h>
#include <pcl/features/impl/shot_omp.hpp>

#include <pcl/features/fpfh_omp.h>
#include <pcl/features/impl/fpfh_omp.hpp>

#include <pcl/surface/gp3.h>
#include <pcl/features/rops_estimation.h>

#include "descriptors/bshot.h"
#include "descriptors/impl/bshot.hpp"

#include "descriptors/drink.h"
#include "descriptors/impl/drink.hpp"

#include "descriptors/random_guess.h"

#include <pcl/features/usc.h>
#include <pcl/features/impl/usc.hpp>

//----------------------------
//----------- USC ------------
//----------------------------
template<typename PointOutT>
void initUSC(const Cloud& in, pcl::Feature<pcl::PointXYZRGBNormal, PointOutT>& desc)
{
	std::cout<<"Initializing USC\n";
	
	typedef pcl::UniqueShapeContext<pcl::PointXYZRGBNormal> USC_t;

	//downcasting. This is safe if you're not mixing 
	//different descriptors and init functions!!!
	USC_t& usc = (USC_t&)(desc);

	float r_max = in.resolution * 20.0f;
	usc.setRadiusSearch( r_max );
	usc.setMinimalRadius( 0.1f * r_max );
	usc.setLocalRadius( r_max );
	usc.setPointDensityRadius( in.resolution * 2.0f );
}

template<typename PointOutT>
float distUSC(const PointOutT& lhs, const PointOutT& rhs)
{
	float acc = 0.0f;
	for(int i = 0; i < 1960; ++i)
		acc += pow(lhs.descriptor[i]-rhs.descriptor[i], 2.0f);
	return sqrt(acc);
}

//-------------------------------
//----------- RANDOM ------------
//-------------------------------
template<typename PointOutT>
void initRANDOM(const Cloud& in, pcl::Feature<pcl::PointXYZRGBNormal,PointOutT>& desc)
{
	std::cout<<"Initializing Random Descriptor...\n";
	
	typedef RandomGuessDescriptor<pcl::PointXYZRGBNormal, DummyDesc> Random_t;

	//downcasting. This is safe if you're not mixing 
	//different descriptors and init functions!!!
	Random_t& random = (Random_t&)(desc);

	random.setRadiusSearch(in.support_radius);
}

template<typename PointOutT>
float distRANDOM(const PointOutT& lhs, const PointOutT& rhs)
{
	//just throw some random distance so our matches
	//will be also random
	return (float)rand() / RAND_MAX;
}

//------------------------------
//----------- DRINK ------------
//------------------------------
template<typename PointOutT>
void initDRINK(const Cloud& in, pcl::Feature<pcl::PointXYZRGBNormal,PointOutT>& desc)
{
	std::cout<<"Initializing DRINK\n";
	
	typedef DRINK3Estimation<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal, DRINKSignature> DRINK_t;

	//downcasting. This is safe if you're not mixing 
	//different descriptors and init functions!!!
	DRINK_t& drink = (DRINK_t&)(desc);

	drink.setRadiusSearch(in.support_radius);
	drink.setInputNormals(in.points);
}

template<typename PointOutT>
float distDRINK(const PointOutT& lhs, const PointOutT& rhs)
{
	//TODO: __builtin_popcount() is not portable!!!
	//do some #ifs here so we can make sure it compiles
	//on windows

	float dist = 0.0f;

	//ComputeDRINK
	/*
	for(int j = 0; j < DRINK_N_BINS; j++)
		dist += __builtin_popcount(lhs.histogram[j] ^ rhs.histogram[j]);
	*/

	//Test: L1 distance
	/*
	for(int j = 0; j < DRINK_N_BINS; j++)
	{
		int s = lhs.histogram[j] - rhs.histogram[j];
		dist += s > 0 ? s : -s;
	}
	*/

	//Test: L2 distance, square-rooted
	/*
	for(int j = 0; j < DRINK_N_BINS; j++)
		dist += pow(lhs.histogram[j] - rhs.histogram[j], 2.0f);
	dist = sqrt(dist);
	*/

	//Test: L2 distance, squared
	/*
	for(int j = 0; j < DRINK_N_BINS; j++)
	{
		int s = pow(lhs.histogram[j] - rhs.histogram[j], 2.0f);
		dist += s;
	}*/

	//TEST: Weight popcount based on the importance of the bit
	//It seems that this is effective. Weights should be
	//an increasing sequence.
	/*
	int weight[32];
	for(int i = 0; i < 32; i++) weight[i] = 2*i;

	for(int k = 0; k < DRINK_N_BINS; k++)
	{
		int a = lhs.histogram[k], b = rhs.histogram[k];

		for(int shift = 0; shift < 32; ++shift)
		{
			int mask = 1 << shift;
			int set = ((a & mask) ^ (b & mask)) ? 1 : 0;
			dist += set * weight[shift];
		}
	}
	*/

	//ComputeDRINK10
	/*
	for(int j = 0; j < PLANES_DESC; j++)
		dist +=  __builtin_popcount(lhs.planes[j] ^ rhs.planes[j]);
	*/

	//ComputeDRINK11
	/*
	for(int j = 0; j < N_MOMENTS_2D; j++)
		dist += pow(lhs.moments2d[j] - rhs.moments2d[j], 2.0f);
	dist = sqrt(dist);
	*/

	//ComputeDRINK12
	/*
	for(int j = 0; j < N_MOMENTS_3D; j++)
		dist += pow(lhs.moments3d[j] - rhs.moments3d[j], 2.0f);
	dist = sqrt(dist);
	*/

	//ComputeDRINK13
	for(int j = 0; j < NAPS_PLANES * NAPS_MOMENTS; j++)
		dist += pow(lhs.naps[j] - rhs.naps[j], 2.0f);
	dist = sqrt(dist);

	return dist;
}

//-------------------------------
//----------- B-SHOT ------------
//-------------------------------
template<typename PointOutT>
void initBSHOT(const Cloud& in, pcl::Feature<pcl::PointXYZRGBNormal,PointOutT>& desc)
{
	std::cout<<"Initializing B-SHOT\n";
	
	typedef BSHOTEstimation<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal, BSHOTDescriptor> BSHOT_t;

	//downcasting. This is safe if you're not mixing 
	//different descriptors and init functions!!!
	BSHOT_t& bshot = (BSHOT_t&)(desc);

	bshot.setRadiusSearch(in.support_radius);
	bshot.setInputNormals(in.points);
}

template<typename PointOutT>
float distBSHOT(const PointOutT& lhs, const PointOutT& rhs)
{
	//TODO: __builtin_popcount() is not portable!!!
	//do some #ifs here so we can make sure it compiles
	//on windows

	float acc = 0.0f;
	for(int i = 0; i < 352; ++i)
		acc += __builtin_popcount(lhs.bitset[i] ^ rhs.bitset[i]);
	return acc;
}

//-----------------------------
//----------- RoPS ------------
//-----------------------------
template<typename PointOutT>
void initROPS(const Cloud& in, pcl::Feature<pcl::PointXYZRGBNormal,PointOutT>& desc)
{
	const float PI = 3.141592653f;
	std::cout<<"Initializing RoPS\n";
	
	typedef pcl::ROPSEstimation<pcl::PointXYZRGBNormal, pcl::Histogram<135>> ROPS_t;

	//-----------------------------------
	//------ COMPUTE TRIANGULATION ------
	//-----------------------------------
	// unfortunatelly we need to recompute the triangulation
	// for the cloud, because we cleaned it in the preprocessing
	// step and thus the original triangulation is not valid anymore
	pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp3;
	
	pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBNormal>());
	std::vector<pcl::Vertices> triangles;

	gp3.setSearchRadius( in.resolution ); //Maximum edge length
	gp3.setMu(2.5f); //The NN distance multiplier to obtain the final search radius.
	gp3.setMaximumNearestNeighbors(100);
	gp3.setMaximumSurfaceAngle( PI / 4.0f );
	gp3.setMinimumAngle( PI / 18.0f );
	gp3.setMaximumAngle( 2.0f * PI / 3.0f );
	gp3.setNormalConsistency( false );
	gp3.setSearchMethod(tree);
	gp3.setInputCloud( in.points );

	std::cout<<"\tComputing triangulation\n";
	gp3.reconstruct(triangles);
	std::cout<<"\tdone\n";
	//------------------------------------

	// downcasting. This is safe if you're not mixing 
	// different descriptors and init functions!!!
	ROPS_t& rops = (ROPS_t&)(desc);

	rops.setSupportRadius(in.support_radius);
	rops.setRadiusSearch(in.support_radius);
	rops.setNumberOfPartitionBins( 5 );
	rops.setNumberOfRotations( 3 );
	rops.setTriangles( triangles );
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

	// Taken from FPFH tutorial in PCL page:
 	// IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
	fpfh.setRadiusSearch(in.support_radius * 0.3f);
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