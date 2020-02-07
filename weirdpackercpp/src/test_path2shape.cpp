#include<fstream>
#include "Path.hpp"
#include "Svg.hpp"
#include<iostream>

int main(int argc,char** argv)
{
	/*std::ifstream inpf("../../../data/test.path");
	
	trail::Shape sp=trail::path2shape(inpf,"testpath",0.01);
	std::cout << sp << std::endl;*/
	
	std::ifstream inpf("../../../data/drawing.svg");
	trail::SVG drawing;
	inpf >> drawing;
	std::vector<trail::Shape> shapes=drawing.getAllShapes();
	
	for(size_t i=0;i<shapes.size();i++)
	{
		std::cout << "shape[" << i << "]:" << std::endl;
		std::cout << shapes[i] << std::endl;
	}
	return 0;
}
