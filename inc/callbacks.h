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
	{
		int s = pow(lhs.histogram[j] - rhs.histogram[j], 2.0f);
		dist += s;
	}
	dist = sqrt(dist);
	*/

	//Test: L2 distance, squared
	/*
	for(int j = 0; j < DRINK_N_BINS; j++)
	{
		int s = pow(lhs.histogram[j] - rhs.histogram[j], 2.0f);
		dist += s;
	}*/

	//ComputeDRINK2
	/*
	for(int j = 0; j < BIT_STRING_SIZE; j++)
		dist += __builtin_popcount(lhs.signature[j] ^ rhs.signature[j]);
	*/

	//ComputeDRINK3
	/*
	for(int j = 0; j < ORI_HIST_SIZE; j++)
	{
		dist += __builtin_popcount(lhs.oriHistX[j] ^ rhs.oriHistX[j]);
		dist += __builtin_popcount(lhs.oriHistY[j] ^ rhs.oriHistY[j]);
		dist += __builtin_popcount(lhs.oriHistZ[j] ^ rhs.oriHistZ[j]);
	} */

	//ComputeDRINK4
	/*
	for(int j = 0; j < N_LOBES; j++)
		dist += __builtin_popcount(lhs.lobeHist[j] ^ rhs.lobeHist[j]);
	*/

	//ComputeDRINK5
	/*
	for(int j = 0; j < N_PAIRS; j++)
		dist += __builtin_popcount(lhs.binarysignature[j] ^ rhs.binarysignature[j]);
	*/

	//ComputeDRINK6
	/*
	for(int j = 0; j < N_PAIRS; j++)
		dist += __builtin_popcount(lhs.binarysignature[j] ^ rhs.binarysignature[j]);
	*/

	//ComputeDRINK7
	/*
	for(int j = 0; j < N_CUTS; j++)
		dist += __builtin_popcount(lhs.radial_features[j] ^ rhs.radial_features[j]);
	*/

	//Test: L1 distance
	/*
	for(int j = 0; j < N_SECTORS; j++)
	{
		int s = lhs.radial_features[j] - rhs.radial_features[j];
		dist += s > 0 ? s : -s;
	}
	*/

	//ComputeDRINK8
	/*
	for(int j = 0; j < FSHOT; j++)
	{
		int s = pow(lhs.fshot[j] - rhs.fshot[j], 2.0f);
		dist += s;
	}
	*/

	//ComputeDRINK9
	/*
	for(int j = 0; j < PLANES_DESC; j++)
		dist +=  __builtin_popcount(lhs.planes[j] ^ rhs.planes[j]);
	*/

	//std::cout<<"d = "<<dist<<", ";

	//ComputeDRINK10
	for(int j = 0; j < PLANES_DESC; j++)
		dist +=  __builtin_popcount(lhs.planes[j] ^ rhs.planes[j]);

	//std::cout<<"d = "<<dist<<", ";

	//TEST: Weight popcount based on the importance of the bit
	//It seems that this is VERY effective! Weights should be
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