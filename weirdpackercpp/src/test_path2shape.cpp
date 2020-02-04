#include<fstream>
#include "Path.hpp"

int main(int argc,char** argv)
{
	std::ifstream inpf("../../../data/test.path");
	
	wp::path2shape(inpf,0.01);
}
