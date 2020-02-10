#include "Svg.hpp"
#include "Path.hpp"
#include "pugixml/pugixml.hpp"
#include <sstream>

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
	std::string tag2pathstr(const pugi::xml_node& node) const
	{
		
		/*
		 *	<rect x="10" y="10" width="30" height="30" stroke="black" fill="transparent" stroke-width="5"/>
		 *	<rect x="60" y="10" rx="10" ry="10" width="30" height="30" stroke="black" fill="transparent" stroke-width="5"/>
		 *	<circle cx="25" cy="75" r="20" stroke="red" fill="transparent" stroke-width="5"/>
		 *	<ellipse cx="75" cy="75" rx="20" ry="5" stroke="red" fill="transparent" stroke-width="5"/>
		 *	<line x1="10" x2="50" y1="110" y2="150" stroke="orange" stroke-width="5"/>
		 *	<polyline points="60 110 65 120 70 115 75 130 80 125 85 140 90 135 95 150 100 145"
		 *	stroke="orange" fill="transparent" stroke-width="5"/>
		 *	<polygon points="50 160 55 180 70 180 60 190 65 205 50 195 35 205 40 190 30 180 45 180"
		 *	<path d="M20,230 Q40,205 50,230 T90,230" fill="none" stroke="blue" stroke-width="5"/>		
		 */
		std::cout << node.name() << std::endl;
		std::string tag=node.name();
		std::ostringstream pout;
		if(tag=="rect")
		{
//			Eigen::Vector2d ul(node.attribute("x").value(),node.attribute("y").value());
//			Eigen::Vector2d wh(node.attribute("width").value(),node.attribute("h").value());
//			pout << "M " << ul.x() << "," << ul.y() << " W " << wh.x() << " H " << wh.y() << " W " << -wh.x() << " z";
		}
		else if(tag=="circle")
		{
				//this requires a non-path argument (arcs don't count)
		}
		else if(tag=="ellipse")
		{
			//same as circle
		}
		else if(tag=="polygon")
		{
			//pout << 
		}
		else if(tag=="path")
		{
			return node.attribute("d").value();
		}
		else if(tag=="line" || tag=="polyline")
		{
			return "";
		}
		else
		{
			return "";
		}
		return "";
	}
	std::vector<trail::Shape> getAllShapes(double segmentlen) const
	{
		const std::string xpathquery="//rect | //circle | //ellipse | //polygon | //path | //line | //polyline";
		//const std::string xpathquery="//path";
		pugi::xpath_node_set shapenodes=doc.select_nodes(xpathquery.c_str());
		std::vector<trail::Shape> shapes;
		for (pugi::xpath_node_set::const_iterator it = shapenodes.begin(); it != shapenodes.end(); ++it)
		{
			std::string rpath=tag2pathstr(it->node());
			if(rpath.size())
			{
				std::cout << "Detected Path: " << rpath << std::endl;
				std::istringstream inpathstr(rpath);
				std::string nodeid=it->node().attribute("id").value();
				if(nodeid.empty())
				{
					std::ostringstream nidstream;
					nidstream << "shape" << shapes.size();
					nodeid=nidstream.str();
				}
				shapes.push_back(trail::path2shape(inpathstr,nodeid,segmentlen));
			}
		}
		return shapes;
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
std::vector<trail::Shape> trail::SVG::getAllShapes(double segmentlen) const
{
		return impl->getAllShapes(segmentlen);
}
