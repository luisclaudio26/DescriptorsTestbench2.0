#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
using namespace std;

int main(int argc, char** args)
{
	float auc = 0.0f;
	float p, r, p_last, r_last;

	cin>>p_last>>r_last;

	while( cin>>p>>r )
	{
		auc += (p + p_last)*(r - r_last)*0.5f;
		p_last = p; r_last = r;
	}

	std::cout<<auc<<"\n";

	return 0;
}