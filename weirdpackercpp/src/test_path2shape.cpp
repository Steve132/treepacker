#include<fstream>
#include "Path.hpp"
#include<iostream>

int main(int argc,char** argv)
{
	std::ifstream inpf("../../../data/test.path");
	
	wp::Shape sp=wp::path2shape(inpf,"testpath",0.01);
	std::cout << sp << std::endl;
	return 0;
}
