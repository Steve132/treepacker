#include<fstream>
#include "Path.hpp"
#include<iostream>

int main(int argc,char** argv)
{
	std::ifstream inpf("../../../data/test.path");
	
	trail::Shape sp=trail::path2shape(inpf,"testpath",0.01);
	std::cout << sp << std::endl;
	return 0;
}
