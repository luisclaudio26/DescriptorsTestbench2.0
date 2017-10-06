#ifndef _DRINK3_H_
#define _DRINK3_H_

#include <pcl/point_types.h>
#include <pcl/features/feature.h>
#include <pcl/features/impl/feature.hpp>
#include <cstdlib>
using pcl::FeatureWithLocalReferenceFrames;
using pcl::FeatureFromNormals;

const double PI = 3.14159265;

#define DRINK_N_BINS 25

#define N_MOMENTS_2D 4
#define N_MOMENTS_3D 8

#define NAPS_PLANES 8
#define NAPS_MOMENTS 4

#define PLANES_DESC 16

typedef struct {
	Eigen::Vector4f N; //normal direction
	double O;		   //offset in normal direction
} Plane;

struct DRINKSignature
{
	int histogram[DRINK_N_BINS];
	float moments2d[N_MOMENTS_2D];
	float moments3d[N_MOMENTS_3D];
	float naps[NAPS_PLANES * NAPS_MOMENTS];
	int planes[PLANES_DESC];
};

POINT_CLOUD_REGISTER_POINT_STRUCT(DRINKSignature, 
									(int[DRINK_N_BINS], histogram, histogram)
									(float[N_MOMENTS_2D], moments2d, moments2d)
									(float[N_MOMENTS_3D], moments3d, moments3d)
									(float[NAPS_PLANES*NAPS_MOMENTS], naps, naps)
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
			for(int i = 0; i < PLANES_DESC; i++)
			{
				//get random direction for plane normal
				double u = (double)rand()/RAND_MAX;
				double v = (double)rand()/RAND_MAX;

				double theta = u * 2.0 * PI;
				double phi = acos( 2.0 * v - 1.0);

				Eigen::Vector4f N;
				N(0) = sin(theta) * cos(phi);
				N(1) = sin(theta) * sin(phi);
				N(2) = cos(phi);
				N(3) = 0.0f;

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

	bool initCompute() override;
	void computeFeature(PointCloudOut& out) override;
	bool computePointDRINK(int id_kp, PointOutT& descriptor); //simplified SHOT
	bool computePointDRINK10(int id_kp, int id_lrf, PointOutT& descriptor); //Binary Splitting Planes
	bool computePointDRINK11(int id_kp, int id_lrf, PointOutT& descriptor); //simplified RoPS
	bool computePointDRINK12(int id_kp, int id_lrf, PointOutT& descriptor); //3D central moments
	bool computePointDRINK13(int id_kp, int id_lrf, PointOutT& descriptor); //Normal aligned rotational statistics
};

template<typename PointInT, typename PointNT, typename PointOutT>
std::vector<Plane> DRINK3Estimation<PointInT,PointNT,PointOutT>::planes;

#include "impl/drink.hpp"

#endif