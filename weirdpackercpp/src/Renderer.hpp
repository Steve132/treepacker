#ifndef RENDERER_HPP
#define RENDERER_HPP

#include<memory>
#include<Eigen/Dense>
#include<Eigen/Geometry>
#include "Triangle.hpp"
#include "BallTree.hpp"
#include "Mesh.hpp"
#include<array>


namespace wp
{
typedef std::array<std::uint8_t,3> Color;
class Renderer
{
private:
	class Impl;
	std::shared_ptr<Impl> impl;
public:
	Renderer(const Eigen::Vector2i& windowsize,float canvasheight=1.0f);
	
	void draw(const wp::Triangle& tri,const Color& color,bool outline=false);
	void draw(const ballbase<2,float>& circle,const Color& color,bool outline=false);
	void draw(const Mesh& sp,const Color& color,bool outline=false);
	void draw(const Eigen::AlignedBox2f& box,const Color& color,bool outline=false);
	
	void clear(const Color& clearcolor=Color{0,0,0});
	Eigen::Vector2f getMousePosition();
	
	void update(int n=0);
	bool isOpen();
};
}


#endif
