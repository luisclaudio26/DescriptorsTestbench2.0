#include <iostream>
#include <vector>
#include "../inc/TestCase.h"

using namespace std;
using namespace Descriptiveness;

int main (int argc, char** argv)
{
	vector<TestCase> tests;
	TestCase::loadTestCasesFromEXP( std::string(argv[1]), tests);

	vector<PRC> prc;
	tests.back().preprocess();
	tests.back().descriptiveness(prc);
	
	for(auto p = prc.begin(); p != prc.end(); ++p)
	{
		for(auto e = p->begin(); e != p->end(); ++e)
			cout<<"\t"<<e->p<<" "<<e->r<<"\n";
		cout<<"\t-------------------------\n";
	}

	//tests.back().visualize();

	return 0;
}