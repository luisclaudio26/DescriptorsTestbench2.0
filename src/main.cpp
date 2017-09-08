#include <iostream>
#include "../inc/TestCase.h"
using namespace std;

int main (int argc, char** argv)
{
	std::vector<TestCase> tests;
	TestCase::loadTestCasesFromEXP( std::string(argv[1]), tests);

	Descriptiveness::PRC dummy;

	tests.back().preprocess();
	tests.back().descriptiveness(dummy);
	
	//tests.back().visualize();
	for(auto it = dummy.begin(); it != dummy.end(); ++it)
		cout<<"\t"<<it->p<<" "<<it->r<<"\n";

	return 0;
}