#ifndef _DRINK3_HPP_
#define _DRINK3_HPP_

#include <algorithm>
#include <cmath>
#include <chrono>
#include <stack>
#include <cfloat>
#include <iterator>
#include <pcl/features/shot_lrf.h>
#include <pcl/features/feature.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/geometry.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <fftw3.h>

template<typename PointInT, typename PointNT, typename PointOutT>
std::vector<Plane> DRINK3Estimation<PointInT,PointNT,PointOutT>::planes;

/* 1) initCompute(), deinitCompute() podem ser implementados pelas
 * classes derivando de Feature. 
 * 2) computeFeature() DEVE ser implementado.
 * 3) Os três protected e chamados na função compute() de Feature,
 * que é pública. Isso assegura que initCompute() é chamado antes
 * de computeFeature(), e deinitCompute() é chamado depois de tudo.
 * 4) SHOT cria o frame de referência local em initCompute().
 * 5) No SHOT, o cálculo de fato do descritor acontece em
 * createBinDistanceShape()! Esse é o código para observar quando
 * estiver criando o DRINK.
 * 6) Atributos importantes:
 * 	- feature_name_
 * 	- search_method_surface_
 *	- surface_
 *	- tree_
 * 	- search_parameter_
 *	- search_radius_
 *	- input_ [cloud]
 *	- indices_
 *  - PointCloudLRFConstPtr frames_; # Target of initLocalReferenceFrames
 *	- PointCloudNConstPtr normals_;
 * 7) There are metafunctions inside point_types.h which
 * check whether some point type has the desired attributes. Useful
 * for final, production code.
 * 8) For DRINK, specialize templates for benchmarking, etc.
 * Specializing the templates we can use Hamming distance for comparisons,
 * without changing existing code that much. It will be necessary to
 * implement Search class. Ideally, we could use a KD Tree that measure
 * distances using Hamming norm.
 */

template<typename PointInT, typename PointNT, typename PointOutT>
bool DRINK3Estimation<PointInT, PointNT, PointOutT>::initCompute()
{
	//invoke initCompute() method from base class
	if( !FeatureFromNormals<PointInT,PointNT,PointOutT>::initCompute() )
	{
		PCL_ERROR("[pcl::%s::initCompute] Init failed.\n", this->getClassName().c_str());
		return false;
	}

	//create local reference frame. We use SHOT's proposed
	//RF, as it claims to be unique and unambiguous.
	typename pcl::SHOTLocalReferenceFrameEstimation<PointInT>::Ptr lrf( new pcl::SHOTLocalReferenceFrameEstimation<PointInT>() );
	lrf->setRadiusSearch( this->search_radius_ );
	lrf->setInputCloud( this->input_ );
	lrf->setIndices( this->indices_ );
	lrf->setSearchSurface( this->surface_ );

	if(!FeatureWithLocalReferenceFrames<PointInT,pcl::ReferenceFrame>::initLocalReferenceFrames(this->indices_->size(), lrf))
	{
		PCL_ERROR("[DRINK] Error while initializing SHOT's local reference frame!\n");
		return false;
	}

	std::cout<<"\nComputed "<<this->frames_->size()<<" frames of reference"<<std::endl;

	return true;
}

template<typename PointInT, typename PointNT, typename PointOutT>
void DRINK3Estimation<PointInT, PointNT, PointOutT>::computeFeature(PointCloudOut& out)
{
	//loop over indices. Each one of these points
	//will have its descriptor calculated	
	for(int i = 0; i < this->indices_->size(); ++i)
		computePointDRINK11(this->indices_->at(i), i, out.at(i));

	/*
	std::cout<<"Descriptor cloud: "<<std::endl;
	for(auto it = out.begin(); it != out.end(); ++it)
	{
		for(int i = 0; i < DRINK_N_BINS; i++)
			std::cout<<it->histogram[i]<<"   ";
		std::cout<<std::endl;
	}
	*/
}

template<typename PointInT, typename PointNT, typename PointOutT>
bool DRINK3Estimation<PointInT, PointNT, PointOutT>::computePointDRINK(int id_kp, PointOutT& descriptor)
{
	//initialize descriptor
	memset(descriptor.histogram, 0, sizeof(int) * DRINK_N_BINS);

	//our current keypoint and its normal
	const PointInT& kp = this->input_->points[id_kp];

	//get all neighbours within a radius of SEARCH_RADIUS_
	std::vector<int> k_indices; std::vector<float> k_sqr_distances;
	this->tree_->radiusSearch(kp, this->search_radius_, k_indices, k_sqr_distances);

	//if number of neighbours is not enough, skip this keypoint
	if(k_indices.size() < 3)
		return false;

	float curv; Eigen::Vector4f kp_n;
	computePointNormal(*this->surface_, k_indices, kp_n, curv);
	kp_n[3] = 0.0f;

	//loop over neighbours, compute dot product of normals
	for(auto id_ngbr = k_indices.begin(); id_ngbr != k_indices.end(); ++id_ngbr)
	{
		//current neighbour and its normal
		const PointInT& ngbr = this->input_->points[*id_ngbr];
		const PointNT& ngbr_n_ = this->normals_->points[*id_ngbr];

		if(!pcl::isFinite<PointNT>(ngbr_n_))
		{
			std::cout<<"NaN normals in neighborhood! skipping"<<std::endl;
			continue;
		}

		Eigen::Vector4f ngbr_n(ngbr_n_.normal_x, ngbr_n_.normal_y, ngbr_n_.normal_z, 0.0);

		float d = kp_n.dot(ngbr_n);

		//accumulate in histogram. We normalize
		//D so it is in the range [0,1].
		//This method will crash if
		//d = 1!!! (bin will be equal to DRINK_N_BINS).
		//float n_d = (d + 1.0f) / 2.0f;
		float n_d = (d + 1.0f) / 2.0f;

		int bin = (int)(n_d * DRINK_N_BINS);
		if(bin == DRINK_N_BINS) bin--; //Awful fix to the d = 1 problem.

		descriptor.histogram[bin]++;
	}

	//Quick test on Retrieval dataset showed that
	//rotating the descriptor so to put the greatest bin
	//on the beginning is better (faster and improves PRC) 
	//then just sorting it.
	//Also, previous tests showed that sorting is better
	//then not sorting. Test on Kinect dataset showed almost no
	//difference between sorting and rotating in terms of descriptiveness,
	//so we keep choosing to rotate because it is a bit faster (about 0.01 s).
	auto max = std::max_element(std::begin(descriptor.histogram), std::end(descriptor.histogram));
	std::rotate(std::begin(descriptor.histogram), max, std::end(descriptor.histogram));

	return true;
}

template<typename PointInT, typename PointNT, typename PointOutT>
bool DRINK3Estimation<PointInT, PointNT, PointOutT>::computePointDRINK2(int id_kp, int kp_id, PointOutT& descriptor)
{
	//get transformation that maps from world space
	//to the local reference frame
	Eigen::Matrix3f world2lrf;
	pcl::ReferenceFrame lrf = this->frames_->points[kp_id];

	world2lrf.col(0)<<lrf.x_axis[0],lrf.x_axis[1],lrf.x_axis[2];
	world2lrf.col(1)<<lrf.y_axis[0],lrf.y_axis[1],lrf.y_axis[2];
	world2lrf.col(2)<<lrf.z_axis[0],lrf.z_axis[1],lrf.z_axis[2];

	Eigen::Matrix3f lrf2world = world2lrf.inverse();

	//our current keypoint
	const PointInT& kp_ = this->input_->points[id_kp];
	Eigen::Vector3f kp = kp_.getVector3fMap();

	//initialize output
	memset(descriptor.signature, 0, sizeof(int) * BIT_STRING_SIZE);

	//We'll get two sectors around the keypoint: first sector is a bit
	//"ahead" (in the Z axis) and the other is "before" (also in the Z
	//axis). The idea is to build something similar to the sampling
	//patterns (daisies) of DRINK.
	//TODO: MUST REALLY VERIFY IF THESE TRANSFORMATIONS ARE CORRECT!
	float searchRadiusOver4 = this->search_radius_ * 0.25f;
	int kNeighbors = (int)sqrt(BIT_STRING_SIZE);

	Eigen::Vector3f northKP = world2lrf * kp_.getVector3fMap();
	northKP[0] += searchRadiusOver4; 
	northKP = lrf2world * northKP;
	PointInT northKP_; northKP_.x = northKP[0];
					   northKP_.y = northKP[1];
					   northKP_.z = northKP[2];
	std::vector<int> north_indices; std::vector<float> north_sqr_dist;
	this->tree_->nearestKSearchT(northKP_, kNeighbors, 
								north_indices, north_sqr_dist);
	

	Eigen::Vector3f southKP = world2lrf * kp_.getVector3fMap();
	southKP[0] += searchRadiusOver4; 
	southKP = lrf2world * southKP;
	PointInT southKP_; southKP_.x = southKP[0];
					   southKP_.y = southKP[1];
					   southKP_.z = southKP[2];
	std::vector<int> south_indices; std::vector<float> south_sqr_dist;
	this->tree_->nearestKSearchT(southKP_, kNeighbors, 
								south_indices, south_sqr_dist);

	//std::cout<<north_indices.size()<<" points in North lobe"<<std::endl;
	//std::cout<<south_indices.size()<<" points in South lobe"<<std::endl;

	//Get pairs of points (Ni, Si), where Ni is in North lobe and Si is
	//in South lobe. Compute angle (or cosine of angle) between its normals,
	//SHOT-like. Discretize it using DRINK method.
	//We'll end up with many pairs. Maybe we can order it somehow and get
	//only the N first. These N will be part of the final descriptor
	//(for example, N string of K bits will be the final descriptor).
	std::vector<float> anglePairs;
	for(int north = 0; north < north_indices.size(); north++)
		for(int south = 0; south < south_indices.size(); south++)
		{
			//ZE POINTS
			PointInT north_p_ = this->input_->points[north];
			PointInT south_p_ = this->input_->points[south];

			//Normal of each points
			PointNT north_n_ = this->normals_->points[north];
			PointNT south_n_ = this->normals_->points[south];

			//Inefficient; is north_n is NaN, it will 
			//loop a bunch of times uselessly
			if(!pcl::isFinite<PointNT>(north_n_) || !pcl::isFinite<PointNT>(south_n_)) continue;

			Eigen::Vector3f north_n(north_n_.normal_x, north_n_.normal_y, north_n_.normal_z);
			Eigen::Vector3f south_n(south_n_.normal_x, south_n_.normal_y, south_n_.normal_z);

			Eigen::Vector3f north_p = north_p_.getVector3fMap();
			Eigen::Vector3f south_p = south_p_.getVector3fMap();

			//compute cosine of the angle between then (maybe the angle
			//itself is better)
			Eigen::Vector3f kpN = north_p - kp; //kpN = kpN * (1.0f / kpN.norm());
			Eigen::Vector3f kpS = south_p - kp; //kpS = kpS * (1.0f / kpS.norm());
			float v = kpN.dot(kpS); //Nice results
			//float v = (south_p - north_p).norm(); //BAD!
			//float v = acos(kpN.dot(kpS)); //Meh

			anglePairs.push_back( v );
		}

	//if no angle was computed, someething is wrong
	if(anglePairs.empty()) return false;

	//sort and get the first K. This is a very crude way to
	//attain some robustness: points in different clouds,
	//even if they are "the same" somehow, will have different
	//indices, so we need some property that is unique to it
	//so we always select it.
	//std::sort(anglePairs.begin(), anglePairs.end());

	//we map the [-1,1] cosine value into a integer
	//value in range [0,99]
	const float OVER_PI = 0.318309886;
	float max = *std::max_element(anglePairs.begin(), anglePairs.end());

	//std::cout<<"Bit string: \t";
	for(int i = 0; i < BIT_STRING_SIZE; i++)
	{
		//float encoding = (anglePairs[i] / max) * 512.0f; //Normalizing is not so good
		float encoding = anglePairs[i] * 512.0f;

		descriptor.signature[i] = static_cast<int>( encoding );
		//std::cout<<"("<<descriptor.signature[i]<<" "<<anglePairs[i]<<") ";
	}
	//std::cout<<endl;

	return true;
}

template<typename PointInT, typename PointNT, typename PointOutT>
bool DRINK3Estimation<PointInT, PointNT, PointOutT>::computePointDRINK3(int id_kp, PointOutT& descriptor)
{
	//AWFUL METHOD!

	//for each lobe, create a three dimensional histogram
	//of the normals. Then, discretize it somehow and output
	//bit string.

	//our current keypoint
	const PointInT& kp_ = this->input_->points[id_kp];
	Eigen::Vector3f kp = kp_.getVector3fMap();

	//initialize output
	memset(descriptor.oriHistX, 0, sizeof(int) * ORI_HIST_SIZE);
	memset(descriptor.oriHistY, 0, sizeof(int) * ORI_HIST_SIZE);
	memset(descriptor.oriHistZ, 0, sizeof(int) * ORI_HIST_SIZE);

	PointInT northKP = kp_;
	std::vector<int> north_indices; std::vector<float> north_sqr_dist;
	this->tree_->radiusSearch(northKP, this->search_radius_, 
								north_indices, north_sqr_dist);

	for(int i = 0; i < north_indices.size(); i++)
	{
		//get normal 
		PointNT north_n = this->normals_->points[i];

		int binX = (int)( (north_n.normal_x + 1.0f) * 0.5f * ORI_HIST_SIZE );
		descriptor.oriHistX[binX]++;

		int binY = (int)( (north_n.normal_y + 1.0f) * 0.5f * ORI_HIST_SIZE );
		descriptor.oriHistY[binY]++;

		int binZ = (int)( (north_n.normal_z + 1.0f) * 0.5f * ORI_HIST_SIZE );
		descriptor.oriHistZ[binZ]++;
	}

	/*
	std::cout<<"-----------------"<<std::endl;
	for(int i = 0; i < ORI_HIST_SIZE; i++)
		std::cout<<"("<<descriptor.oriHistX[i]<<", "<<descriptor.oriHistY[i]<<", "<<descriptor.oriHistZ[i]<<")"<<std::endl;*/
}

template<typename PointInT, typename PointNT, typename PointOutT>
bool DRINK3Estimation<PointInT, PointNT, PointOutT>::computePointDRINK4(int id_kp, PointOutT& descriptor)
{
	//SHAPE CONTEXTS LIKE: bad, but maybe more lobes (just like the
	//daisies from FREAK) and a unique reference frame would improve it.

	//our current keypoint
	const PointInT& kp_ = this->input_->points[id_kp];
	Eigen::Vector3f kp = kp_.getVector3fMap();

	//initialize output
	memset(descriptor.lobeHist, 0, sizeof(int) * N_LOBES);

	float searchRadiusOver4 = this->search_radius_ * 0.25f;

	//NORTH
	PointInT northKP = kp_; northKP.z += searchRadiusOver4;
	std::vector<int> north_indices; std::vector<float> north_sqr_dist;
	this->tree_->radiusSearch(northKP, searchRadiusOver4, 
								north_indices, north_sqr_dist);
	descriptor.lobeHist[0] = north_indices.size();
	
	//SOUTH
	PointInT southKP = kp_; southKP.z -= searchRadiusOver4;
	std::vector<int> south_indices; std::vector<float> south_sqr_dist;
	this->tree_->radiusSearch(southKP, searchRadiusOver4, 
								south_indices, south_sqr_dist);
	descriptor.lobeHist[1] = south_indices.size();

	//EAST
	PointInT eastKP = kp_; eastKP.x += searchRadiusOver4;
	std::vector<int> east_indices; std::vector<float> east_sqr_dist;
	this->tree_->radiusSearch(eastKP, searchRadiusOver4, 
								east_indices, east_sqr_dist);
	descriptor.lobeHist[2] = east_indices.size();

	//WEST
	PointInT westKP = kp_; westKP.x -= searchRadiusOver4;
	std::vector<int> west_indices; std::vector<float> west_sqr_dist;
	this->tree_->radiusSearch(westKP, searchRadiusOver4, 
								west_indices, west_sqr_dist);
	descriptor.lobeHist[3] = west_indices.size();

	std::cout<<"North lobe: "<<descriptor.lobeHist[0]<<", south lobe: "<<descriptor.lobeHist[1];
	std::cout<<" East lobe: "<<descriptor.lobeHist[2]<<", west lobe: "<<descriptor.lobeHist[3];
	std::cout<<std::endl;
}

template<typename PointInT, typename PointNT, typename PointOutT>
bool DRINK3Estimation<PointInT, PointNT, PointOutT>::computePointDRINK5(int id_kp, int id_lrf, PointOutT& descriptor)
{
	//This is 3DBS as it was described, but neighbourhood
	//is not angle-constrainted.

	//SHOT-like local reference frame
	pcl::ReferenceFrame lrf = this->frames_->points[id_lrf];
	Eigen::Vector3f u(lrf.x_axis[0], lrf.y_axis[0], lrf.z_axis[0]); 
	Eigen::Vector3f v(lrf.x_axis[1], lrf.y_axis[1], lrf.z_axis[1]);
	Eigen::Vector3f w(lrf.x_axis[2], lrf.y_axis[2], lrf.z_axis[2]);

	//our current keypoint and its normal
	const PointInT& kp = this->input_->points[id_kp];

	//get all neighbours within a radius of SEARCH_RADIUS_
	std::vector<int> k_indices; std::vector<float> k_sqr_distances;
	this->tree_->nearestKSearch(id_kp, N_POINTS, k_indices, k_sqr_distances);

	//output point
	memset(descriptor.binarysignature, 0, sizeof(int) * N_PAIRS);

	//loop over pairs of neighbours
	int index = 0;
	for(auto f = k_indices.begin(); f < k_indices.end(); ++f)
		for(auto s = f+1; s != k_indices.end(); ++s)
		{
			//Convert PCL points to vectors.
			//TODO: Maybe this is not necessary!
			PointInT f_pit = this->input_->points[*f];
			Eigen::Vector3f fst(f_pit.x, f_pit.y, f_pit.z);

			PointInT s_pit = this->input_->points[*s];
			Eigen::Vector3f snd(s_pit.x, s_pit.y, s_pit.z);

			//projections
			float fu, fv, fw, su, sv, sw;
			fu = fst.dot(u); fv = fst.dot(v); fw = fst.dot(w);
			su = snd.dot(u); sv = snd.dot(v); sw = snd.dot(w);

			//comparisons (-1 is a safeway to set all bits to 1)
			descriptor.binarysignature[index] = fu >= su ? -1 : 0; 
			descriptor.binarysignature[index+1] = fv >= sv ? -1 : 0;
			descriptor.binarysignature[index+2] = fw >= sw ? -1 : 0;

			index += 3;
		}

	return true;
}

template<typename PointInT, typename PointNT, typename PointOutT>
bool DRINK3Estimation<PointInT, PointNT, PointOutT>::computePointDRINK6(int id_kp, int id_lrf, PointOutT& descriptor)
{
	//This is similar to SHOT: we compare normals, and store a three-bit
	//signature comparing normals in terms of X, Y and Z.
	//Maybe we should transform normals to LRF!

	//output point
	memset(descriptor.binarysignature, 0, sizeof(int) * N_PAIRS);

	//get N_POINTS in neighbourhood
	std::vector<int> k_indices; std::vector<float> k_sqr_distances;
	this->tree_->nearestKSearch(id_kp, N_POINTS+1, k_indices, k_sqr_distances);

	//our current keypoint
	//Returning false is a hotfix. I don't know 
	//what to do when keypoint has NaN normal!
	const PointInT& kp_ = this->input_->points[id_kp];
	Eigen::Vector3f kp(kp_.x, kp_.y, kp_.z);
	
	const PointNT& kp_n_ = this->normals_->points[id_kp];
	if(!pcl::isFinite<PointNT>(kp_n_)) return false;
	Eigen::Vector3f kp_n(kp_n_.normal_x, kp_n_.normal_y, kp_n_.normal_z);

	//loop over neighbors
	int index = 0;
	for(auto ngbr_id = k_indices.begin()+1; ngbr_id < k_indices.end(); ++ngbr_id)
	{
		const PointNT& ngbr_n_ = this->normals_->points[*ngbr_id];
		
		if(!pcl::isFinite<PointNT>(ngbr_n_))
		{
			descriptor.binarysignature[index] = 0; 
			descriptor.binarysignature[index+1] = 0;
			descriptor.binarysignature[index+2] = 0;
		}
		else
		{
			Eigen::Vector3f ngbr_n(ngbr_n_.normal_x, ngbr_n_.normal_y, ngbr_n_.normal_z);
			
			descriptor.binarysignature[index] = ngbr_n[0] > kp_n[0] ? -1 : 0; 
			descriptor.binarysignature[index+1] = ngbr_n[1] > kp_n[1] ? -1 : 0;
			descriptor.binarysignature[index+2] = ngbr_n[2] > kp_n[2] ? -1 : 0;
		}

		index += 3;
	}

	return true;
}

template<typename PointInT, typename PointNT, typename PointOutT>
bool DRINK3Estimation<PointInT, PointNT, PointOutT>::computePointDRINK7(int id_kp, int id_lrf, PointOutT& descriptor)
{
	//No, this is not a good method!

	//output point
	memset(descriptor.radial_features, 0, sizeof(int) * N_CUTS);

	//SHOT-like local reference frame
	pcl::ReferenceFrame lrf = this->frames_->points[id_lrf];
	Eigen::Vector3f u(lrf.x_axis[0], lrf.y_axis[0], lrf.z_axis[0]); 
	Eigen::Vector3f v(lrf.x_axis[1], lrf.y_axis[1], lrf.z_axis[1]);
	Eigen::Vector3f w(lrf.x_axis[2], lrf.y_axis[2], lrf.z_axis[2]);

	//get all neighbours within a radius of SEARCH_RADIUS_
	std::vector<int> k_indices; std::vector<float> k_sqr_distances;
	this->tree_->radiusSearch(id_kp, this->search_radius_, k_indices, k_sqr_distances);

	//our current keypoint
	const PointInT& kp_ = this->input_->points[id_kp];
	Eigen::Vector3f kp(kp_.x, kp_.y, kp_.z);

	//compute centroid of each sector. We loop through neighbors
	//and accumulate their position to compute the centroids.
	int counts[N_SECTORS]; memset(counts, 0, sizeof(int) * N_SECTORS);
	Eigen::Vector3f centroids[N_SECTORS];
	for(int i = 0; i < N_SECTORS; i++) centroids[i] << 0, 0, 0;

	double bin_range = (2.0*PI) / ((double)N_SECTORS);

	for(auto n = k_indices.begin(); n != k_indices.end(); ++n)
	{
		//closest point to KP is itself
		if(*n == id_kp) continue;

		//convert neighbour to a vector.
		PointInT ngbr_ = this->input_->points[*n];
		Eigen::Vector3f ngbr(ngbr_.x, ngbr_.y, ngbr_.z);

		//compute angle between ngbr-keypoint and u-axis
		Eigen::Vector3f normNgbr = (ngbr-kp).normalized();
		double angle = acos( u.dot(normNgbr) );
		
		//we do this so angle is in the range [0,2pi]
		double dir = w.dot(normNgbr);
		if(dir < 0) angle += PI;

		int bin = (int)(angle / bin_range);

		centroids[bin] = centroids[bin] + ngbr;
		counts[bin]++;
	}

	for(int i = 0; i < N_SECTORS; i++)
	{
		if(counts[i] != 0)
			centroids[i] /= counts[i];
		else
			centroids[i] << 0, 0, 0;
	}

	//compute angles between angle centroids and quantize
	for(int i = 0; i < N_SECTORS; i++)
	{
		Eigen::Vector3f c = centroids[i];

		//this implies how many bits we'll use to quantize
		int q = 65536;
		descriptor.radial_features[i] = round( c[0] * q );
	}

	//std::sort(std::begin(descriptor.radial_features), std::end(descriptor.radial_features));
}

template<typename PointInT, typename PointNT, typename PointOutT>
bool DRINK3Estimation<PointInT, PointNT, PointOutT>::computePointDRINK8(int id_kp, int id_lrf, PointOutT& descriptor)
{
	//initialize descriptor
	memset(descriptor.fshot, 0, sizeof(int) * FSHOT);

	//our current keypoint and its normal
	const PointInT& kp_ = this->input_->points[id_kp];
	Eigen::Vector3f kp = kp_.getVector3fMap();

	//SHOT-like local reference frame
	pcl::ReferenceFrame lrf = this->frames_->points[id_lrf];
	if( !pcl_isfinite(lrf.x_axis[0]) ) return false;

	Eigen::Vector3f normal( lrf.z_axis[0],
							lrf.z_axis[1],
							lrf.z_axis[2]);

	Eigen::Vector3f x_axis( lrf.x_axis[0],
							lrf.x_axis[1],
							lrf.x_axis[2]);

	//get all neighbours within a radius of SEARCH_RADIUS_
	std::vector<int> k_indices; std::vector<float> k_sqr_distances;
	this->tree_->radiusSearch(kp_, this->search_radius_, k_indices, k_sqr_distances);

	//if number of neighbours is not enough, skip this keypoint
	if(k_indices.size() < 3) return false;

	//loop over neighbours, compute dot product of normals
	for(auto id_ngbr = k_indices.begin(); id_ngbr != k_indices.end(); ++id_ngbr)
	{
		//current neighbour and its normal
		const PointInT& ngbr_ = this->surface_->points[*id_ngbr];
		Eigen::Vector3f ngbr = ngbr_.getVector3fMap();

		//skip if neighbour is the keypoint itself
		if( ngbr == kp ) continue;

		const PointNT& ngbr_n_ = this->normals_->points[*id_ngbr];
		if(!pcl::isFinite<PointNT>(ngbr_n_))
		{
			std::cout<<"NaN normals in neighborhood! skipping"<<std::endl;
			continue;
		}

		Eigen::Vector3f ngbr_n(ngbr_n_.normal_x, ngbr_n_.normal_y, ngbr_n_.normal_z);

		float d = normal.dot(ngbr_n);

		//accumulate in histogram. We normalize
		//D so it is in the range [0,1].
		//This method will crash if
		//d = 1!!! (bin will be equal to FSHOT).
		//float n_d = (d + 1.0f) / 2.0f;
		float n_d = (d + 1.0f) / 2.0f;

		int bin = (int)(n_d * 30);
		if(bin == 30) bin--; //Awful fix to the d = 1 problem.

		descriptor.fshot[FSHOT_THETA * FSHOT_PHI + bin]++;

		//Mimic Unique Shape Context
		//Error with ngbr_.z close to r
		//in tuning5. Is this normal?
		//float r = sqrt(k_sqr_distances[*id_ngbr]);
	
		//phi angle. Project on tangent plane, compute
		//angle with x_axis	
		Eigen::Vector3f proj;
		pcl::geometry::project(ngbr, kp, normal, proj);
		proj -= kp;	proj.normalize();
			
		Eigen::Vector3f cross = x_axis.cross(proj);
		float phi = atan2(cross.norm(), x_axis.dot(proj));
		phi = cross.dot(normal) < 0 ? (2*PI-phi) : phi; //range [0,360]

		//compute theta
		Eigen::Vector3f no = ngbr - kp; no.normalize();
		float theta = normal.dot(no);
		theta = acos(std::min(1.0f, std::max(-1.0f, theta))); //make sure theta is in range [-1,1]
															  //output is range [0,180]

		int bin_phi = (int)(FSHOT_PHI * (phi/(2*PI)));
		int bin_theta = (int)(FSHOT_THETA * (theta/PI));
		
		descriptor.fshot[FSHOT_THETA*bin_phi + bin_theta]++;
	}

	return true;
}

//-------------------------------------------------------------------
typedef struct {
	int i; float d;
	Eigen::Vector3f pos;
} kdPoint;

struct kdNode_ {
	struct kdNode_ *l, *r;
	kdPoint *p; int n_points;
};
typedef struct kdNode_ kdNode;

static int longest_axis(const kdPoint *points, int n_points)
{
	Eigen::Vector3f min, max;

	for(int i = 0; i < 3; ++i)
	{
		min[i] = FLT_MAX;
		max[i] = -FLT_MAX;
	}

	for(int i = 0; i < n_points; ++i)
		for(int j = 0; j < 3; ++j)
		{
			max[j] = std::max(max[j], points[i].pos[j]);
			min[j] = std::min(min[j], points[i].pos[j]);
		}

	Eigen::Vector3f diff = max - min;
	int i; int longest = 0;
	for(i = 0; i < 3; ++i)
		if( diff[i] > diff[longest] ) longest = i;

	return longest;
}

template<typename PointInT, typename PointNT, typename PointOutT>
void DRINK3Estimation<PointInT,PointNT,PointOutT>::kdtree_order(const std::vector<int>& ind_in, 
																const std::vector<float> dist_in,
																const Eigen::Matrix3f& world2lrf,
																std::vector<int>& ind_out, 
																std::vector<float>& dist_out)
{
	//initialize root node
	kdNode root;
	root.n_points = ind_in.size();
	root.l = root.r = NULL;
	root.p = new kdPoint[root.n_points];

	//Build list of points inside root,
	//transforming them to local reference frame
	for(int i = 0; i < ind_in.size(); ++i)
	{
		Eigen::Vector3f pos = world2lrf * this->surface_->points[ind_in[i]].getVector3fMap();
	
		if( pos[0] != pos[0] || pos[1] != pos[1] || pos[2] != pos[2])
		{
			std::cout<<"NaN matrix! Using untransformed point.\n";

			//Dirty fix: just use untransformed point. It will add noise and
			//will mess things up; I just hope it won't be too much!
			pos = this->surface_->points[ind_in[i]].getVector3fMap();
		}

		root.p[i] = (kdPoint){ind_in[i], dist_in[i], pos};
	}

	//Build kd-tree of the points
	int l = 0;
	std::stack<kdNode*> todo; todo.push(&root);
	while(!todo.empty())
	{
		//get current node
		kdNode* cur = todo.top();
		todo.pop();

		//if node has just one point,
		//do nothing.
		if(cur->n_points > 1)
		{
			//initialize left/right children
			cur->l = new kdNode;
			cur->r = new kdNode;

			//we allocate more memory than we need,
			//otherwise we would need to loop through
			//the points two times (one to count how
			//many points we have in each side, another
			//to actually split data).
			cur->l->p = new kdPoint[cur->n_points];
			cur->r->p = new kdPoint[cur->n_points];

			//find longest axis
			int axis = longest_axis( cur->p, cur->n_points );

			//split point in the mid-point
			float split = 0.0f;
			for(int i = 0; i < cur->n_points; ++i)
				split += cur->p[i].pos[axis];
			split /= cur->n_points;

			//split point in the median
			//MAYBE IT'S NOT WORTH IT TO COPE WITH
			//THE PROBLEMS OF THIS METHOD.
			/*
			std::sort( cur->p, cur->p + cur->n_points,
						[&](const kdPoint& lhs, const kdPoint& rhs) -> bool {
							return lhs.pos[axis] < rhs.pos[axis];
						});

			//TODO: está tendo loop infinito, então provavelmente tem algum erro aqui
			//EXPLICAÇÃO: Há loop infinito porque, se houver dois pontos com coordenadas
			//iguais no eixo AXIS, nunca será possível atingir o critério de um ponto
			//por caixa.
			//SOLUÇÃO: adicionar uma variável que conta quantas vezes o loop executou
			//com os mesmos parâmetros (como fazer isso?). Se executar mais de uma vez,
			//estamos num loop infinito pela razão citada e podemos parar, deixando dois
			//pontos na caixa.
			float split = cur->p[cur->n_points/2].pos[axis];
			*/

			//loop over points, deciding to which side each
			//one belongs
			int l_i = 0, r_i = 0;
			for(int i = 0; i < cur->n_points; ++i)
				if(cur->p[i].pos[axis] < split)
					cur->l->p[l_i++] = cur->p[i];
				else
					cur->r->p[r_i++] = cur->p[i];

			cur->l->n_points = l_i;
			cur->r->n_points = r_i;
			cur->l->l = cur->l->r = NULL;
			cur->r->l = cur->r->r = NULL;

			//delete data inside this node,
			//because we copied them into
			//its children
			delete[] cur->p;

			todo.push(cur->l);
			todo.push(cur->r);
		}
	}

	//traverse kd-tree, ordering the points
	std::stack<kdNode*> dfs; dfs.push(&root);
	while(!dfs.empty())
	{
		kdNode* cur = dfs.top(); dfs.pop();

		if(cur->n_points == 1)
		{
			ind_out.push_back( cur->p[0].i );
			dist_out.push_back( cur->p[0].d );
			
			delete[] cur->p;
		}
		else
		{
			if(cur->l) dfs.push( cur->l );
			if(cur->r) dfs.push( cur->r );
		}
	}
}


template<typename PointInT, typename PointNT, typename PointOutT>
bool DRINK3Estimation<PointInT, PointNT, PointOutT>::computePointDRINK9(int id_kp, int id_lrf, PointOutT& descriptor)
{
	//initialize output
	memset(descriptor.planes, 0, sizeof(int) * PLANES_DESC);

	Eigen::Vector3f kp = this->input_->points[id_kp].getVector3fMap();

	//get transformation that maps from world space
	//to the local reference frame
	Eigen::Matrix3f lrf2world;
	pcl::ReferenceFrame lrf = this->frames_->points[id_lrf];
	
	lrf2world.col(0)<<lrf.x_axis[0],lrf.x_axis[1],lrf.x_axis[2];
	lrf2world.col(1)<<lrf.y_axis[0],lrf.y_axis[1],lrf.y_axis[2];
	lrf2world.col(2)<<lrf.z_axis[0],lrf.z_axis[1],lrf.z_axis[2];

	//skip if transformation is null
	if(lrf2world(0,0) != lrf2world(0,0))
		return false;

	Eigen::Matrix3f world2lrf = lrf2world.transpose();

	//Get K neighbours and sort them according to their distance from the center
	std::vector<int> k_indices; std::vector<float> k_sqr_distances;
	this->tree_->nearestKSearch(id_kp, N_NGBR, k_indices, k_sqr_distances);

	//This doesn't work!
	std::vector<int> indices; std::vector<float> sqr_distances;
	kdtree_order(k_indices, k_sqr_distances, world2lrf, indices, sqr_distances);

	//patch radius is simply the distance from the keypoint to the
	//farthest neighbour. We're guaranteed to have all the neighbours
	//in this range, so we "know" that no plane will be such that
	//all neighbours are to its left or its right.
	double bound = sqrt( *std::max_element(sqr_distances.begin(), sqr_distances.end()) );

	//loop over planes, decide in which side of the plane each
	//neighbour is.
	int n_ones = 0, n_zeros = 0;

	for(int i = 0; i < N_PLANES; ++i)
	{
		//Current plane
		Plane& plane = DRINK3Estimation<PointInT,PointNT,PointOutT>::planes[i];
		Eigen::Vector3f normal = world2lrf * plane.N;

		for(int j = 0; j < indices.size(); ++j)
		{			
			Eigen::Vector3f ngbr = this->surface_->points[ indices[j] ].getVector3fMap();
			
			//shift keypoint in the direction of the normal.
			//We bound the shift so it won't go to far away,
			//which would cause all points to be at the same side.
			Eigen::Vector3f center = kp + (bound * plane.O) * plane.N;

			float side = (ngbr-center).dot(normal);
			int code = side < 0 ? 0 : -1;

			if(code == -1) n_ones++;
			else n_zeros++;

			descriptor.planes[i*N_NGBR+j] = code;
		}
	}

	std::cout<<"<"<<n_zeros<<", "<<n_ones<<">, ";

	/*
	for(int i = 0; i < PLANES_DESC; i++)
		std::cout<<descriptor.planes[i];
	std::cout<<std::endl<<std::endl;
	*/

	return true;
}

//------------------------------------------------------------------
template<typename PointInT, typename PointNT, typename PointOutT>
bool DRINK3Estimation<PointInT, PointNT, PointOutT>::computePointDRINK10(int id_kp, int id_lrf, PointOutT& descriptor)
{
	//Count number of points in each side of the plane,
	//store 1 if more points are to the right side of the
	//plane (and zero otherwise).

	//initialize output
	memset(descriptor.planes, 0, sizeof(int) * PLANES_DESC);

	Eigen::Vector3f kp = this->input_->points[id_kp];

	//get transformation that maps from world space
	//to the local reference frame
	Eigen::Matrix3f world2lrf;
	pcl::ReferenceFrame lrf = this->frames_->points[id_lrf];
	
	world2lrf.row(0)<<lrf.x_axis[0],lrf.x_axis[1],lrf.x_axis[2];
	world2lrf.row(1)<<lrf.y_axis[0],lrf.y_axis[1],lrf.y_axis[2];
	world2lrf.row(2)<<lrf.z_axis[0],lrf.z_axis[1],lrf.z_axis[2];

	//skip NaN transformations
	if(world2lrf(0,0) != world2lrf(0,0))
		return false;

	//Get neighbours in a fixed radius
	std::vector<int> k_indices; std::vector<float> k_sqr_distances;
	this->tree_->radiusSearch(kp, this->search_radius_, k_indices, k_sqr_distances);

	//patch radius is simply the distance from the keypoint to the
	//farthest neighbour. We're guaranteed to have all the neighbours
	//in this range, so we "know" that no plane will be such that
	//all neighbours are to its left or its right.
	//double bound = sqrt( *std::max_element(sqr_distances.begin(), sqr_distances.end()) );

	//loop over planes, decide in which side of the plane each
	//neighbour is.
	for(int i = 0; i < N_PLANES; ++i)
	{
		//Current plane direction
		Plane& plane = DRINK3Estimation<PointInT,PointNT,PointOutT>::planes[i];
		Eigen::Vector3f normal = world2lrf * plane.N;

		int l_side = 0, r_side = 0;

		for(int j = 0; j < k_indices.size(); ++j)
		{			
			Eigen::Vector3f ngbr = this->surface_->points[ k_indices[j] ].getVector3fMap();
			
			//shift keypoint in the direction of the normal.
			//We bound the shift so it won't go to far away,
			//which would cause all points to be at the same side.
			Eigen::Vector3f center = kp;//kp + (bound * plane.O) * plane.N;

			float side = (ngbr-center).dot(normal);
			if(side < 0) l_side++;
			else r_side++;
		}

		descriptor.planes[i] = (r_side > l_side) ? 0 : -1;
	}

	/*
	std::cout<<"N points: "<<k_indices.size()<<" ";
	for(int i = 0; i < PLANES_DESC; i++)
		std::cout<<(descriptor.planes[i] == 0 ? 0 : 1);
	std::cout<<std::endl<<std::endl;
	*/

	return true;
}

template<typename PointInT, typename PointNT, typename PointOutT>
bool DRINK3Estimation<PointInT, PointNT, PointOutT>::computePointDRINK11(int id_kp, int id_lrf, PointOutT& descriptor)
{
	//Simplified RoPS descriptor
	//E se o histograma for RADIAL, daí usamos isso pra computar
	//uma DCT/FFT (que é invariante a shift ? logo seria invariante
	//a rotação?)
	//TODO: searchRadius() quando recebe o índice do keypoint procura
	//na search surface, não na nuvem de keypoints!!! Esse erro deve
	//ter causado problema em vários outros testes!

	//initialize output
	memset(descriptor.moments, 0, sizeof(float) * N_MOMENTS);

	PointInT kp = this->input_->points[id_kp];

	//translate to centroid (= kp), then rotate i.e. (Rot.Trans).x
	Eigen::Matrix4f rot;
	pcl::ReferenceFrame lrf = this->frames_->points[id_lrf];
	
	rot.col(0)<<lrf.x_axis[0],lrf.x_axis[1],lrf.x_axis[2],0.0f;
	rot.col(1)<<lrf.y_axis[0],lrf.y_axis[1],lrf.y_axis[2],0.0f;
	rot.col(2)<<lrf.z_axis[0],lrf.z_axis[1],lrf.z_axis[2],0.0f;
	rot.col(3)<<0.0f,0.0f,0.0f,1.0f;

	//skip NaN transformations
	if(rot(0,0) != rot(0,0)) return false;

	Eigen::Matrix4f trans = Eigen::Matrix4f::Identity();
	trans.col(3) = kp.getVector4fMap();

	//TODO: this can be made faster, but lets get it right first :(
	Eigen::Matrix4f world2lrf = (trans * rot).inverse();

	//Get neighbours in a fixed radius
	std::vector<int> k_indices; std::vector<float> k_sqr_distances;
	this->tree_->radiusSearch(kp, this->search_radius_, k_indices, k_sqr_distances);

	//defines the size of the bounding square for projection
	//and number of bins in 2D histogram
	//float l = 2.0f * t;
	float l = this->search_radius_; float over2l = 1.0f / (2*l);
	int n = 15; int n_points = k_indices.size();

	//project points in plane XY (perpendicular to normal, LRF's z axis)
	//and accumulate histogram
	#define AT(i,j) ((n)*(i)+(j))
	float *hist = new float[n*n]; for(int i = 0; i < n*n; ++i) hist[i] = 0;

	for(int i = 0; i < k_indices.size(); ++i)
	{
		//send point from global coordinates
		//to local reference frame
		Eigen::Vector4f p_ = world2lrf * this->surface_->points[ k_indices[i] ].getVector4fMap();

		//p coordinates should be in range [-SupportRadius, SupportRadius]
		//map it to the range [0,1]
		//Actually, coordinates should be a little less then SupportRadius (how distant?)
		Eigen::Vector4f offset; offset<<l,l,l,0.0f;
		Eigen::Vector4f p = (p_ + offset) * over2l;

		//make sure we're not picking the normal direction! I think this is correct
		float x = p[0], y = p[1];
		
		//this should not overflow, because the probability of having a point like
		//[0,0,SupportRadius] is very low.
		int u = (int)(x * n), v = (int)(y * n);

		//this will give us a normalized histogram
		hist[AT(u,v)] += 1.0f / n_points;
	}

	//compute central moments to this projection
	//E se tirasse os momentos centrais do histograma 3D?!
	float i_ = 0.0f, j_ = 0.0f;
	for(int i = 0; i < n; ++i)
		for(int j = 0; j < n; ++j)
		{
			i_ += i * hist[AT(i,j)];
			j_ += j * hist[AT(i,j)];
		}

	float u11 = 0.0f, u12 = 0.0f, u21 = 0.0f, u22 = 0.0f;
	for(int i = 0; i < n; ++i)
		for(int j = 0; j < n; ++j)
		{
			u11 += (i - i_) * (j - j_) * hist[AT(i,j)];
			u12 += (i - i_) * pow((j - j_), 2.0f) * hist[AT(i,j)];
			u21 += pow((i - i_), 2.0f) * (j - j_) * hist[AT(i,j)];
			u22 += pow((i - i_), 2.0f) * pow((j - j_), 2.0f) * hist[AT(i,j)];
		}

	//output this to descriptor
	descriptor.moments[0] = u11;
	descriptor.moments[1] = u12;
	descriptor.moments[2] = u21;
	descriptor.moments[3] = u22;

	/*
	for(int i = 0; i < n; ++i)
	{
		for(int j = 0; j < n; ++j)
			std::cout<<hist[AT(i,j)]<<" ";
		std::cout<<"\n";
	}
	std::cout<<"u11, u12, u21, u22 = "<<u11<<", "<<u12<<", "<<u21<<", "<<u22<<std::endl;
	std::cout<<"--------------------------------------------\n";
	*/

	delete[] hist;

	return true;
}

template<typename PointInT, typename PointNT, typename PointOutT>
bool DRINK3Estimation<PointInT, PointNT, PointOutT>::computePointDRINK12(int id_kp, int id_lrf, PointOutT& descriptor)
{
	//Montar histograma 2D do plano XY,
	//depois aplica FFT e tira as primeiras frequências.
	//Isso deve ser suficiente pra dar uma "descrição geral"
	//do patch

	//initialize output
	memset(descriptor.moments, 0, sizeof(float) * N_MOMENTS);

	PointInT kp = this->input_->points[id_kp];

	//translate to centroid (= kp), then rotate i.e. (Rot.Trans).x
	Eigen::Matrix4f rot;
	pcl::ReferenceFrame lrf = this->frames_->points[id_lrf];
	
	rot.col(0)<<lrf.x_axis[0],lrf.x_axis[1],lrf.x_axis[2],0.0f;
	rot.col(1)<<lrf.y_axis[0],lrf.y_axis[1],lrf.y_axis[2],0.0f;
	rot.col(2)<<lrf.z_axis[0],lrf.z_axis[1],lrf.z_axis[2],0.0f;
	rot.col(3)<<0.0f,0.0f,0.0f,1.0f;

	//skip NaN transformations
	if(rot(0,0) != rot(0,0)) return false;

	Eigen::Matrix4f trans = Eigen::Matrix4f::Identity();
	trans.col(3) = kp.getVector4fMap();

	//TODO: this can be made faster, but lets get it right first :(
	Eigen::Matrix4f world2lrf = (trans * rot).inverse();

	//Get neighbours in a fixed radius
	std::vector<int> k_indices; std::vector<float> k_sqr_distances;
	this->tree_->radiusSearch(kp, this->search_radius_, k_indices, k_sqr_distances);

	//defines the size of the bounding square for projection
	//and number of bins in 2D histogram
	//float l = 2.0f * t;
	float l = this->search_radius_; float over2l = 1.0f / (2*l);
	int n = 15; int n_points = k_indices.size();

	//project points in plane XY (perpendicular to normal, LRF's z axis)
	//and accumulate histogram
	#define AT(i,j) ((n)*(i)+(j))
	float *hist = new float[n*n]; for(int i = 0; i < n*n; ++i) hist[i] = 0;

	for(int i = 0; i < k_indices.size(); ++i)
	{
		//send point from global coordinates
		//to local reference frame
		Eigen::Vector4f p_ = world2lrf * this->surface_->points[ k_indices[i] ].getVector4fMap();

		//p coordinates should be in range [-SupportRadius, SupportRadius]
		//map it to the range [0,1]
		//Actually, coordinates should be a little less then SupportRadius (how distant?)
		Eigen::Vector4f offset; offset<<l,l,l,0.0f;
		Eigen::Vector4f p = (p_ + offset) * over2l;

		//make sure we're not picking the normal direction! I think this is correct
		float x = p[0], y = p[1];
		
		//this should not overflow, because the probability of having a point like
		//[0,0,SupportRadius] is very low.
		int u = (int)(x * n), v = (int)(y * n);

		//this will give us a normalized histogram
		hist[AT(u,v)] += 1.0f / n_points;
	}

	//compute FFT of this histogram

	//output this to descriptor

	/*
	for(int i = 0; i < n; ++i)
	{
		for(int j = 0; j < n; ++j)
			std::cout<<hist[AT(i,j)]<<" ";
		std::cout<<"\n";
	}
	std::cout<<"u11, u12, u21, u22 = "<<u11<<", "<<u12<<", "<<u21<<", "<<u22<<std::endl;
	std::cout<<"--------------------------------------------\n";
	*/

	delete[] hist;

	return true;
}

//TODO: computePointDRINK13() devia montar um histograma 3D (de preferência não-orientado),
//e a rede neural é treinada pra minimizar distâncias entre os histogramas dos patches
//transformados.

#endif