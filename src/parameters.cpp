#include "../inc/parameters.h"

#include <fstream>
#include <iostream>
using std::ifstream;

unsigned int Parameters::N_THREADS = 4;
float Parameters::SUPPORT_RADIUS_ALPHA = 0.01;
int Parameters::N_STEPS_DISTANCE_THRESHOLD = 40;
float Parameters::MESH_RESOLUTION_FACTOR = 1.0f;
float Parameters::MESH_AREA_CORRECTION = 4.5f;
float Parameters::NORMAL_SEARCH_RADIUS = 3.0f;
float Parameters::MIN_NEIGHBORS = 5;
float Parameters::RANSAC_THRESHOLD = 0.05f;

void Parameters::loadFromFile(const string& fp)
{
	ifstream reader(fp, std::ios_base::in);

	reader>>Parameters::N_THREADS;
	reader>>Parameters::SUPPORT_RADIUS_ALPHA;
	reader>>Parameters::N_STEPS_DISTANCE_THRESHOLD;
	reader>>Parameters::MESH_RESOLUTION_FACTOR; 
	reader>>Parameters::MESH_AREA_CORRECTION;
	reader>>Parameters::NORMAL_SEARCH_RADIUS;
	reader>>Parameters::MIN_NEIGHBORS;
	reader>>Parameters::RANSAC_THRESHOLD;

	reader.close();

	std::cout<<"Number of threads: "<<Parameters::N_THREADS<<std::endl;
	std::cout<<"Mesh area correction factor: "<<Parameters::MESH_AREA_CORRECTION<<std::endl;
	std::cout<<"Number of stes for matching threshold: "<<Parameters::N_STEPS_DISTANCE_THRESHOLD<<std::endl;
	std::cout<<"Mesh resolution resampling factor: "<<Parameters::MESH_RESOLUTION_FACTOR<<std::endl;
	std::cout<<"Support radius alpha value: "<<Parameters::SUPPORT_RADIUS_ALPHA<<std::endl;
	std::cout<<"Normal search radius factor: "<<Parameters::NORMAL_SEARCH_RADIUS<<std::endl;
	std::cout<<"Minimum number of neighbors within search radius: "<<Parameters::MIN_NEIGHBORS<<std::endl;
	std::cout<<"RANSAC Threshold: "<<Parameters::RANSAC_THRESHOLD<<std::endl;

	//we use mesh_resolution_factor inverted, so
	//to avoid divisions
	Parameters::MESH_RESOLUTION_FACTOR = 1.0f / Parameters::MESH_RESOLUTION_FACTOR;
}