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
#include <pcl/common/common.h>

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
	{
		//computePointDRINK(this->indices_->at(i), out.at(i));
		//computePointDRINK10(this->indices_->at(i), i, out.at(i));
		//computePointDRINK11(this->indices_->at(i), i, out.at(i));
		//computePointDRINK12(this->indices_->at(i), i, out.at(i));
		computePointDRINK13(this->indices_->at(i), i, out.at(i));

		std::cout<<"\rDRINK progress: "<<(int)(i*100.0f/this->indices_->size())<<"%%";
	}
}

template<typename PointInT, typename PointNT, typename PointOutT>
bool DRINK3Estimation<PointInT, PointNT, PointOutT>::computePointDRINK(int id_kp, PointOutT& descriptor)
{
	//---- Simplified SHOT ----

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

//------------------------------------------------------------------
template<typename PointInT, typename PointNT, typename PointOutT>
bool DRINK3Estimation<PointInT, PointNT, PointOutT>::computePointDRINK10(int id_kp, int id_lrf, PointOutT& descriptor)
{
	//Count number of points in each side of the plane,
	//store 1 if more points are to the right side of the
	//plane (and zero otherwise).

	//initialize output
	memset(descriptor.planes, 0, sizeof(int) * PLANES_DESC);

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
	this->tree_->radiusSearch(kp, this->search_radius_, k_indices, k_sqr_distances, 256);

	pcl::PointCloud<PointInT> patch;
	pcl::copyPointCloud( *this->surface_, k_indices, patch );

	//patch radius is simply the distance from the keypoint to the
	//farthest neighbour. We're guaranteed to have all the neighbours
	//in this range, so we "know" that no plane will be such that
	//all neighbours are to its left or its right.
	//double bound = sqrt( *std::max_element(sqr_distances.begin(), sqr_distances.end()) );

	//loop over planes, decide in which side of the plane each
	//neighbour is.
	for(int i = 0; i < PLANES_DESC; ++i)
	{
		//Current plane direction
		Plane plane = DRINK3Estimation<PointInT,PointNT,PointOutT>::planes[i];
		Eigen::Vector4f normal = world2lrf * plane.N;

		int l_side = 0, r_side = 0;

		//shift keypoint in the direction of the normal.
		//We bound the shift so it won't go to far away,
		//which would cause all points to be at the same side.
		//kp + (bound * plane.O) * plane.N; -> OFFSET VERSION
		Eigen::Vector4f center = kp.getVector4fMap();

		for(int j = 0; j < patch.size(); ++j)
		{
			//this is inefficient and could be replaced by something faster,
			//like copying all the neighbors into a single vector
			//Eigen::Vector4f ngbr = this->surface_->points[ k_indices[j] ].getVector4fMap();
			Eigen::Vector4f ngbr = patch.points[j].getVector4fMap();
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
	memset(descriptor.moments2d, 0, sizeof(float) * N_MOMENTS_2D);

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
	descriptor.moments2d[0] = u11;
	descriptor.moments2d[1] = u12;
	descriptor.moments2d[2] = u21;
	descriptor.moments2d[3] = u22;

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
	//---- Montar histograma 3D, tirar momentos centrais ----
	//E se a gente pegar poucos keypoints, mas a região ao redor
	//é maior? Estilo o próprio SP-DOCK faz, expandir a região
	// -> o que deve inclusive ser mais rápido que só 

	//initialize output
	memset(descriptor.moments3d, 0, sizeof(float) * N_MOMENTS_3D);

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

	//get min-max of the neighborhood. We use this info to
	//define the optimal dimensions for the support of our
	//3D histogram
	float minNormal = FLT_MAX, maxNormal = -FLT_MAX;
	for(auto i = k_indices.begin(); i != k_indices.end(); ++i)
	{
		Eigen::Vector4f p = world2lrf * this->surface_->points[*i].getVector4fMap();
		if( p[2] > maxNormal ) maxNormal = p[2];
		if( p[2] < minNormal ) minNormal = p[2];
	}

	//defines the size of the bounding square for projection
	//and number of bins in 2D histogram
	float l = this->search_radius_; float over2l = 1.0f / (2*l);
	Eigen::Vector4f offset; offset<<l,l,-minNormal,0.0f;
	Eigen::Vector4f scale; scale<<over2l,over2l,0.9f/(maxNormal-minNormal),1.0f;

	int I = 8, J = 8, K = 4;
	int n_points = k_indices.size();

	//accumulate 3D rectangular histogram aligned with LRF
	#define AT_(i,j,k) ( I*J*(k) + I*(i) + (j) )
	float *hist = new float[I*J*K]; for(int i = 0; i < I*J*K; ++i) hist[i] = 0;

	for(int i = 0; i < k_indices.size(); ++i)
	{
		//send point from global coordinates
		//to local reference frame
		Eigen::Vector4f p_ = world2lrf * this->surface_->points[ k_indices[i] ].getVector4fMap();

		//p coordinates should be in range [-SupportRadius, SupportRadius]
		//map it to the range [0,1]
		//Actually, coordinates should be a little less then SupportRadius (how distant?)
		Eigen::Vector4f p = (p_ + offset).cwiseProduct(scale);

		//take coordinates
		float x = p[0], y = p[1], z = p[2];

		//this should not overflow, because the probability of having a point like
		//[0,0,SupportRadius] is very low.
		int u = (int)(x * I);
		int v = (int)(y * J);
		int w = (int)(z * K);

		//this will give us a normalized histogram
		hist[AT_(u,v,w)] += 1.0f / n_points;
	}

	//compute central moments to this projection
	float i_ = 0.0f, j_ = 0.0f, k_ = 0.0f;
	for(int k = 0; k < K; ++k)
		for(int i = 0; i < I; ++i)
			for(int j = 0; j < J; ++j)
			{
				i_ += i * hist[AT_(i,j,k)];
				j_ += j * hist[AT_(i,j,k)];
				k_ += k * hist[AT_(i,j,k)];
			}

	float u111 = 0.0f, u112 = 0.0f, u121 = 0.0f, u122 = 0.0f;
	float u211 = 0.0f, u212 = 0.0f, u221 = 0.0f, u222 = 0.0f;

	for(int k = 0; k < K; ++k)
		for(int i = 0; i < I; ++i)
			for(int j = 0; j < J; ++j)
			{
				float di = (i - i_), dj = (j - j_), dk = (k - k_);
				float e = hist[AT_(i,j,k)];

				//maybe we can reduce the number of operations somehow
				u111 += di * dj * dk * e;
				u112 += di * dj * (dk*dk) * e;
				u121 += di * (dj*dj) * dk * e;
				u122 += di * (dj*dj) * (dk*dk) * e;

				u211 += (di*di) * dj * dk * e;
				u212 += (di*di) * dj * (dk*dk) * e;
				u221 += (di*di) * (dj*dj) * dk * e;
				u222 += (di*di) * (dj*dj) * (dk*dk) * e;
			}

	//output this to descriptor
	descriptor.moments3d[0] = u111;
	descriptor.moments3d[1] = u112;
	descriptor.moments3d[2] = u121;
	descriptor.moments3d[3] = u122;
	descriptor.moments3d[4] = u211;
	descriptor.moments3d[5] = u212;
	descriptor.moments3d[6] = u221;
	descriptor.moments3d[7] = u222;

	/*
	for(int k = 0; k < n; ++k)
	{
		for(int i = 0; i < n; ++i)
		{
			for(int j = 0; j < n; ++j)
				std::cout<<hist[AT_(i,j,k)]<<" ";
			std::cout<<"\n";
		}
		std::cout<<"\n\n";
	}
	std::cout<<"---------------------------------------------\n";
	*/

	delete[] hist;

	return true;
}

template<typename PointInT, typename PointNT, typename PointOutT>
bool DRINK3Estimation<PointInT, PointNT, PointOutT>::computePointDRINK13(int id_kp, int id_lrf, PointOutT& descriptor)
{
	//Normal aligned projectional statistics
	//Project points in many planes aligned with the normal
	//Take the central moments of each one, combine in the final descriptor

	//initialize output
	memset(descriptor.naps, 0, sizeof(float) * NAPS_PLANES * NAPS_MOMENTS);

	PointInT kp = this->input_->points[id_kp];
	Eigen::Vector3f kp_ = kp.getVector3fMap();

	//check for NaNs
	pcl::ReferenceFrame lrf = this->frames_->points[id_lrf];
	if( lrf.x_axis[0] != lrf.x_axis[0] ||
		lrf.y_axis[0] != lrf.y_axis[0] ||
		lrf.z_axis[0] != lrf.z_axis[0] ) return false;
	
	Eigen::Vector3f normal, x;
	normal<<lrf.z_axis[0],lrf.z_axis[1],lrf.z_axis[2];
	x<<lrf.x_axis[0],lrf.x_axis[1],lrf.x_axis[2];

	//Get neighbours in a fixed radius
	std::vector<int> k_indices; std::vector<float> k_sqr_distances;
	this->tree_->radiusSearch(kp, this->search_radius_, k_indices, k_sqr_distances, 200 /* tests only */);

	//defines the size of the bounding square for projection
	//and number of bins in 2D histogram
	float l = this->search_radius_; float over2l = 1.0f / (2*l);
	int n = 15; int n_points = k_indices.size();

	const float TWO_PI = 6.283185307;

	//for each plane, project points onto it
	//accumulate histogram then take central moments
	for(int i = 0; i < NAPS_PLANES; ++i)
	{
		//the planes are evenly across the 0-360 range.
		//we the U vector of the plane is the normal itself,
		//the V vector is obtained by rotating the X axis
		float plane_angle = i * TWO_PI / NAPS_PLANES;

		Eigen::Quaternionf x_q( 0.0f, x[0], x[1], x[2]);
		Eigen::Quaternionf q( plane_angle, normal[0], normal[1], normal[2] );
		q.normalize();

		Eigen::Matrix<float,2,3> world2plane;
		world2plane.row(0) = normal;
		world2plane.row(1) = (q * x_q * q.inverse()).vec(); //this a quaternion rotation

		//create and initialize histogram
		#define AT(r,c) ((n)*(r)+(c))
		float *hist = new float[n*n]; 
		for(int k = 0; k < n*n; ++k) hist[k] = 0;

		//now loop over points, project points onto uv-plane,
		//build histogram and take central moments
		for(int j = 0; j < k_indices.size(); ++j)
		{
			Eigen::Vector3f ngbr = this->surface_->points[k_indices[j]].getVector3fMap();
			Eigen::Vector2f p_ = world2plane * (ngbr - kp_);

			//p coordinates should be in range [-SupportRadius, SupportRadius]
			//map it to the range [0,1]
			//Actually, coordinates should be a little less then SupportRadius (how distant?)
			Eigen::Vector2f offset; offset<<l,l;
			Eigen::Vector2f p = (p_ + offset) * over2l;
			
			//this should not overflow, because the probability of having a point like
			//[0,0,SupportRadius] is very low.
			int u = (int)(p[0] * n), v = (int)(p[1] * n);

			//this will give us a normalized histogram
			hist[AT(u,v)] += 1.0f / n_points;
		}

		//compute central moments to this projection
		float j_ = 0.0f, k_ = 0.0f;
		for(int j = 0; j < n; ++j)
			for(int k = 0; k < n; ++k)
			{
				j_ += j * hist[AT(j,k)];
				k_ += k * hist[AT(j,k)];
			}

		float u11 = 0.0f, u12 = 0.0f, u21 = 0.0f, u22 = 0.0f;

		for(int j = 0; j < n; ++j)
			for(int k = 0; k < n; ++k)
			{
				float dj = (j - j_), dk = (k - k_);
				float e = hist[AT(j,k)];

				u11 += dj * dk * e;
				u12 += dj * dk*dk * e;
				u21 += dj*dj * dk * e;
				u22 += dj*dj * dk*dk *e;
			}

		//output this to descriptor
		descriptor.naps[i*NAPS_MOMENTS+0] = u11;
		descriptor.naps[i*NAPS_MOMENTS+1] = u12;
		descriptor.naps[i*NAPS_MOMENTS+2] = u21;
		descriptor.naps[i*NAPS_MOMENTS+3] = u22;

		delete[] hist;
	}

	return true;
}

//TODO: computePointDRINK13() devia montar um histograma 3D (de preferência não-orientado),
//e a rede neural é treinada pra minimizar distâncias entre os histogramas dos patches
//transformados.

#endif