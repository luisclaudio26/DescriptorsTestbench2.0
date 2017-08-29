#include <iostream>
#include "../inc/TestCase.h"
using namespace std;

int main (int argc, char** argv)
{
	std::vector<TestCase> tests;
	TestCase::loadTestCasesFromEXP( std::string(argv[1]), tests);

	std::vector<PREntry> dummy;

	tests.back().preprocess();
	tests.back().descriptiveness(dummy);
	tests.back().visualize();

	return 0;
}