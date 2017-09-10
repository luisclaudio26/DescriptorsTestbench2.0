#ifndef _PARAMETERS_H_
#define _PARAMETERS_H_

#include <string>
using std::string;

//maybe this is wrong, I just calculated
//it in a simple calculator
const float OVER_PI = 0.3183098861f;

//TODO: this is not cool, but... =X
class Parameters
{
public:
	enum KeypointMethod {
		UNIFORM_SAMPLING, ISS
	};

	void loadFromFile(const string& fp);

	static unsigned int getNThreads() { return Parameters::N_THREADS; }
	static float getSupportRadiusAlpha() { return Parameters::SUPPORT_RADIUS_ALPHA; }
	static int getNSteps() { return Parameters::N_STEPS_DISTANCE_THRESHOLD; }
	static float getMeshResFactor() { return Parameters::MESH_RESOLUTION_FACTOR; }
	static float getMeshAreaCorrection() { return Parameters::MESH_AREA_CORRECTION; }
	static float getNormalRadiusFactor() { return Parameters::NORMAL_SEARCH_RADIUS; }
	static float getMinNeighbors() { return Parameters::MIN_NEIGHBORS; }
	static float getRANSACThreshold() { return Parameters::RANSAC_THRESHOLD; }
	static float getUniformSamplingDensity() { return Parameters::UNIFORM_SAMPLING_DENSITY; }
	static KeypointMethod getKeypointMethod() { return KEYPOINT_METHOD; }
	
private:
	//Maximum of threads we can use when trying
	//to parallelize tasks.
	static unsigned int N_THREADS;

	//This will be used to define the search radius for descriptors
	//Original article mentions that 2% is a nice value
	static float SUPPORT_RADIUS_ALPHA;

	//to obtain a ROC curve, we vary the distance threshold
	//inbetween 0 and 1, so we decrease or increase recall value.
	//This is how many steps between 0 and 1 we'll use for the
	//experiment. In the end, this is how many experiments each
	//.descriptiveness file will have.
	static int N_STEPS_DISTANCE_THRESHOLD;

	//How many points we'll use for experiment, between 0 and 1.
	//E.g., 0.5f indicates that well reduce mesh resolution by half.
	static float MESH_RESOLUTION_FACTOR;

	//This flag has a multiplier used to estimate the mesh area
	//when we have just a single view. Guo et al says that 4.5
	//is a good value if we only a single view.
	static float MESH_AREA_CORRECTION;

	//We multiply the mesh resolution by this factor to
	//compute the search radius for normal estimation.
	static float NORMAL_SEARCH_RADIUS;

	//Minimum of neighbors that a point in a point cloud
	//must have not to be considered an outlier. This will
	//avoid computing too much NaN normals and having problems
	//with NaN/null features.
	static float MIN_NEIGHBORS;
	
	//RANSAC distance threshold
	static float RANSAC_THRESHOLD;

	//Uniform sampling density. This will control how many
	//keypoints we'll have when using UniformSampling as
	//keypoint detecor.
	static float UNIFORM_SAMPLING_DENSITY;

	//Choose Uniform Sampling or ISS Keypoints
	static KeypointMethod KEYPOINT_METHOD;


};

#endif