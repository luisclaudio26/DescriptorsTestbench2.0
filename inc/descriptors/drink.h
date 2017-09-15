#ifndef _DRINK3_H_
#define _DRINK3_H_

#include <pcl/point_types.h>
#include <pcl/features/feature.h>
#include <pcl/features/impl/feature.hpp>
#include <cstdlib>
using pcl::FeatureWithLocalReferenceFrames;
using pcl::FeatureFromNormals;

const double PI = 3.14159265;

#define BIT_STRING_SIZE 100
#define ORI_HIST_SIZE 10 //N bins of orientation histogram
#define N_LOBES 4

#define N_POINTS 50
#define N_PAIRS 3*N_POINTS

#define N_CUTS 24
#define N_SECTORS 2*N_CUTS

#define DRINK_N_BINS 30
#define FSHOT_PHI 10
#define FSHOT_THETA 5
#define FSHOT FSHOT_PHI*FSHOT_THETA + DRINK_N_BINS


#define N_PLANES 30
#define N_NGBR 150
//#define PLANES_DESC N_PLANES * N_NGBR  //computePointDRINK9()
#define PLANES_DESC N_PLANES 			 //computePointDRINK10()

typedef struct {
	Eigen::Vector3f N; //normal direction
	double O;		   //offset in normal direction
} Plane;

struct DRINKSignature
{
	int histogram[DRINK_N_BINS];
	int signature[BIT_STRING_SIZE];

	int oriHistX[ORI_HIST_SIZE];
	int oriHistY[ORI_HIST_SIZE];
	int oriHistZ[ORI_HIST_SIZE];

	int lobeHist[N_LOBES];

	int binarysignature[N_PAIRS];

	int radial_features[N_SECTORS];

	int fshot[FSHOT];

	int planes[PLANES_DESC];

	static void NNHamming(DRINKSignature point, 
							const pcl::PointCloud<DRINKSignature>::Ptr& cloud, 
							int& index, int& min_dist);
};

POINT_CLOUD_REGISTER_POINT_STRUCT(DRINKSignature, 
									(int[DRINK_N_BINS], histogram, histogram)
									(int[BIT_STRING_SIZE], signature, signature)
									(int[ORI_HIST_SIZE], oriHistX, oriHistX)
									(int[ORI_HIST_SIZE], oriHistY, oriHistY)
									(int[ORI_HIST_SIZE], oriHistZ, oriHistZ)
									(int[N_LOBES], lobeHist, lobeHist)
									(int[N_PAIRS], binarysignature, binarysignature)
									(int[N_SECTORS], radial_features, radial_features)
									(int[FSHOT], fshot, fshot)
									(int[PLANES_DESC], planes, planes)
									)

template<typename PointInT, typename PointNT, typename PointOutT = DRINKSignature>
class DRINK3Estimation : public FeatureFromNormals<PointInT, PointNT, PointOutT>,
						 public FeatureWithLocalReferenceFrames<PointInT, pcl::ReferenceFrame>
{
public:
	typedef typename pcl::Feature<PointInT, PointOutT>::PointCloudOut PointCloudOut;

	/** \brief Empty constructor. */
	DRINK3Estimation()
	{
		this->feature_name_ = "DRINK";

		//planes are defined only one time
		if(planes.empty())
		{
			for(int i = 0; i < N_PLANES; i++)
			{
				//get random direction for plane normal
				double u = (double)rand()/RAND_MAX;
				double v = (double)rand()/RAND_MAX;

				double theta = u * 2.0 * PI;
				double phi = acos( 2.0 * v - 1.0);

				Eigen::Vector3f N;
				N(0) = sin(theta) * cos(phi);
				N(1) = sin(theta) * sin(phi);
				N(2) = cos(phi);

				//offset is computed as a number in range [-1,1].
				//This offset is multiplied by the radius of the patch
				//in such a way that offset = 0 makes the plane cross
				//the keypoint, and -1 and 1 puts in the borders.
				double offset = (double)(rand()/RAND_MAX);

				DRINK3Estimation<PointInT,PointNT,PointOutT>::planes.push_back( (Plane){N, offset} );
			}
		}
	}

private:

	static std::vector<Plane> planes;
	void kdtree_order(const std::vector<int>& ind_in, const std::vector<float> dist_in,
						const Eigen::Matrix3f& world2lrf, std::vector<int>& ind_out, 
						std::vector<float>& dist_out);

	bool initCompute() override;
	void computeFeature(PointCloudOut& out) override;
	bool computePointDRINK(int id_kp, PointOutT& descriptor);
	bool computePointDRINK2(int id_kp, int kp_id, PointOutT& descriptor);
	bool computePointDRINK3(int id_kp, PointOutT& descriptor);
	bool computePointDRINK4(int id_kp, PointOutT& descriptor);
	bool computePointDRINK5(int id_kp, int id_lrf, PointOutT& descriptor);
	bool computePointDRINK6(int id_kp, int id_lrf, PointOutT& descriptor);
	bool computePointDRINK7(int id_kp, int id_lrf, PointOutT& descriptor);
	bool computePointDRINK8(int id_kp, int id_lrf, PointOutT& descriptor);
	bool computePointDRINK9(int id_kp, int id_lrf, PointOutT& descriptor);
	bool computePointDRINK10(int id_kp, int id_lrf, PointOutT& descriptor);
};

#include "impl/drink.hpp"

#endif