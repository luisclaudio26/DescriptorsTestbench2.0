#ifndef _DRINK3_H_
#define _DRINK3_H_

#include <pcl/point_types.h>
#include <pcl/features/feature.h>
#include <pcl/features/impl/feature.hpp>
#include <cstdlib>
using pcl::FeatureWithLocalReferenceFrames;
using pcl::FeatureFromNormals;

const double PI = 3.14159265;

#define DRINK_N_BINS 30

#define N_PLANES_MOMENTS 2
#define N_MOMENTS_2D 4
#define N_MOMENTS_3D 8

#define NAPS_PLANES 8
#define NAPS_MOMENTS 4

typedef struct {
	Eigen::Vector3f N; //normal direction
	double O;		   //offset in normal direction
} Plane;

struct DRINKSignature
{
	int histogram[DRINK_N_BINS];
	float moments2d[N_MOMENTS_2D];
	float moments3d[N_MOMENTS_3D];
	float naps[NAPS_PLANES * NAPS_MOMENTS];
};

POINT_CLOUD_REGISTER_POINT_STRUCT(DRINKSignature, 
									(int[DRINK_N_BINS], histogram, histogram)
									(float[N_MOMENTS_2D], moments2d, moments2d)
									(float[N_MOMENTS_3D], moments3d, moments3d)
									(float[NAPS_PLANES*NAPS_MOMENTS], naps, naps)
									)

template<typename PointInT, typename PointNT, typename PointOutT = DRINKSignature>
class DRINK3Estimation : public FeatureFromNormals<PointInT, PointNT, PointOutT>,
						 public FeatureWithLocalReferenceFrames<PointInT, pcl::ReferenceFrame>
{
public:
	typedef typename pcl::Feature<PointInT, PointOutT>::PointCloudOut PointCloudOut;

private:

	bool initCompute() override;
	void computeFeature(PointCloudOut& out) override;
	bool computePointDRINK(int id_kp, PointOutT& descriptor); //simplified SHOT
	bool computePointDRINK11(int id_kp, int id_lrf, PointOutT& descriptor); //simplified RoPS
	bool computePointDRINK12(int id_kp, int id_lrf, PointOutT& descriptor); //3D central moments
	bool computePointDRINK13(int id_kp, int id_lrf, PointOutT& descriptor); //Normal aligned rotational statistics
};

#include "impl/drink.hpp"

#endif