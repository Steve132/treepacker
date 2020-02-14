#include "Renderer.hpp"
#include "CImg.h"
using namespace cimg_library;
using namespace wp;


class Renderer::Impl
{
public:

	CImg<unsigned char> buf;
	CImgDisplay disp;
	float scale;
	Impl(const Eigen::Vector2i& windowsize,float canvasheight):
		buf(windowsize.x(),windowsize.y(),1,3),
		disp(buf),
		scale(static_cast<float>(windowsize.y()/canvasheight))
	{}
	
	void draw(const wp::Triangle& tri,const Color& color,bool outline)
	{
		Eigen::Vector2i pi[3]={
			(tri.p[0]*scale).cast<int>(),
			(tri.p[1]*scale).cast<int>(),
			(tri.p[2]*scale).cast<int>()
		};
		if(outline)
			buf.draw_triangle(pi[0].x(),pi[0].y(),pi[1].x(),pi[1].y(),pi[2].x(),pi[2].y(),&color[0],1.0f,~0U);
		else
			buf.draw_triangle(pi[0].x(),pi[0].y(),pi[1].x(),pi[1].y(),pi[2].x(),pi[2].y(),&color[0],1.0f);
	}
	void draw(const ballbase<2,float>& circle,const Color& color,bool outline)
	{
		if(outline)
			buf.draw_circle(int(circle.position.x()*scale),int(circle.position.y()*scale),int(circle.radius*scale),&color[0],1.0f,~0U);
		else
			buf.draw_circle(int(circle.position.x()*scale),int(circle.position.y()*scale),int(circle.radius*scale),&color[0],1.0f);
	}
	void draw(const Mesh& msh,const Color& color,bool outline)
	{
		for(size_t i=0;i<msh.triangles.size();i++)
		{
			draw(msh.triangles[i],color,outline);
		}
	}
	
	void draw(const Eigen::AlignedBox2f& box,const Color& color,bool outline)
	{
		if(outline)
			buf.draw_rectangle(int(box.min().x()*scale),int(box.min().y()*scale),int(box.max().x()*scale),int(box.max().y()*scale),&color[0],1.0f,~0U);
		else
			buf.draw_rectangle(int(box.min().x()*scale),int(box.min().y()*scale),int(box.max().x()*scale),int(box.max().y()*scale),&color[0],1.0f);
	}
	
	
	
	
	
	
	void clear(const Color& clearcolor)
	{
		buf.draw_rectangle(0,0,0,buf.width()-1,buf.height()-1,0,&clearcolor[0]);
	}
	Eigen::Vector2f getMousePosition()
	{
		return Eigen::Vector2f(disp.mouse_x(),disp.mouse_y())/scale;
	}
	
	void update(int n=0)
	{
		disp=buf;
		disp.wait(n);
	}
	bool isOpen()
	{
		return !disp.is_closed();
	}
	
};



Renderer::Renderer(const Eigen::Vector2i& windowsize,float canvasheight):
	impl(new Renderer::Impl(windowsize,canvasheight))
{}
	
void Renderer::draw(const wp::Triangle& tri,const Color& color,bool outline)
{
	impl->draw(tri,color,outline);
}
void Renderer::draw(const ballbase<2,float>& circle,const Color& color,bool outline)
{
	impl->draw(circle,color,outline);
}
void Renderer::draw(const Eigen::AlignedBox2f& box,const Color& color,bool outline)
{
	impl->draw(box,color,outline);
}

void Renderer::draw(const Mesh& msh,const Color& color,bool outline)
{
	impl->draw(msh,color,outline);
}
Eigen::Vector2f Renderer::getMousePosition()
{
	return impl->getMousePosition();
}
void Renderer::clear(const Color& clearcolor)
{
	impl->clear(clearcolor);
}
void Renderer::update(int n)
{
	impl->update(n);
}
bool Renderer::isOpen()
{
	return impl->isOpen();
}


