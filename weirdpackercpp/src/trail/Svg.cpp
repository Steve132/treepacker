#include "Svg.hpp"
#include "pugixml/pugixml.hpp"

class trail::SVG::Impl
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

trail::SVG::SVG():
	impl(new trail::SVG::Impl())
{}

std::ostream& trail::operator<<(std::ostream& out,const trail::SVG& svg)
{
	return svg.impl->save(out);
}
std::istream& trail::operator>>(std::istream& inp,trail::SVG& svg)
{
	return svg.impl->load(inp);
}

