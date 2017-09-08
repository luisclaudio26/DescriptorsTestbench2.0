#ifndef _TEST_CASE_H_
#define _TEST_CASE_H_

#include "cloud.h"
#include "descriptiveness.h"
#include <vector>

class TestCase
{
private:
	Cloud scene; std::vector<Cloud> models;

	//the name of this test case. For display purposes only.
	std::string name;

public:
	//Loads .EXP file and fills the OUT vector
	//with the test cases described in it
	static void loadTestCasesFromEXP(const std::string& path, std::vector<TestCase>& out);

	//Compute every information which will be used
	//in benchmarking. Normals, keypoints, etc. are
	//all calculated in this step.
	void preprocess();

	void visualize();

	//---------------------------
	//------- benchmarks --------
	//---------------------------
	void descriptiveness(std::vector<Descriptiveness::PRC>& out);
};

#endif