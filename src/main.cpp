#include <iostream>
#include <vector>
#include <fstream>
#include <string>
#include <ctime>
#include <boost/filesystem.hpp>
#include "../inc/TestCase.h"

using namespace std;
using namespace Descriptiveness;

typedef vector<PRC> PRCSet;

int main (int argc, char** argv)
{
	vector<TestCase> tests; PRCSet prc;
	TestCase::loadTestCasesFromEXP( std::string(argv[1]), tests);

	for(int i = 0; i < tests.size(); ++i)
	{
		PRCSet set;
		tests[i].preprocess();
		tests[i].descriptiveness(set);

		//accumulate set of PR curves
		if(prc.size() < set.size()) prc.resize(set.size());
		for(int j = 0; j < set.size(); ++j)
			prc[j] = prc[j] + set[j];
	}

	//take average of each PR curve
	for(auto p = prc.begin(); p != prc.end(); ++p)
		*p = *p * (1.0f / tests.size());

	//---------------------------------	
	//-------- Output results ---------
	//---------------------------------

	//Create new directory to hold results
	char dir_name[100];
	std::time_t raw_time; time(&raw_time);
	struct tm *timeinfo = localtime( &raw_time );
	strftime(dir_name, 100, "%F_%Hh%Mm%Ss", timeinfo);

	std::string dir_path = std::string("../output/") + dir_name;
	boost::filesystem::create_directories( dir_path );

	//loop over PRC curves and create files to output 'em
	for(auto p = prc.begin(); p != prc.end(); ++p)
	{
		std::string filepath(dir_path + "/" + p->label + ".prc");
		std::ofstream f(filepath);

		for(auto e = p->curve.begin(); e != p->curve.end(); ++e)
			f<<e->p<<" "<<e->r<<"\n";

		f.close();
	}

	return 0;
}