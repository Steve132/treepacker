#include "Svg.hpp"
#include "pugixml/pugixml.hpp"

class wp::SVG::Impl
{
public:
	pugi::xml_document doc;
	Impl()
	{}
	std::ostream& save(std::ostream& out) const
	{
		doc.save(out);
		return out;
	}
	std::istream& load(std::istream& inp)
	{
		doc.load(inp);
		return inp;
	}
};

wp::SVG::SVG():
	impl(new wp::SVG::Impl())
{}

std::ostream& wp::operator<<(std::ostream& out,const wp::SVG& svg)
{
	return svg.impl->save(out);
}
std::istream& wp::operator>>(std::istream& inp,wp::SVG& svg)
{
	return svg.impl->load(inp);
}

