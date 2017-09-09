#include <iostream>
#include <vector>
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
		{
			prc[j].resize( set[j].size() );
			prc[j] = prc[j] + set[j];
		}
	}

	//take average of each PR curve
	for(auto p = prc.begin(); p != prc.end(); ++p)
		*p = *p * (1.0f / tests.size());
	
	//Output results
	for(auto p = prc.begin(); p != prc.end(); ++p)
	{
		for(auto e = p->begin(); e != p->end(); ++e)
			cout<<e->p<<" "<<e->r<<"\n";
		cout<<"----------------------------\n";
	}

	return 0;
}