add_library(poly2tri
	poly2tri/sweep/advancing_front.cc
	poly2tri/sweep/cdt.cc
	poly2tri/sweep/sweep.cc
	poly2tri/sweep/sweep_context.cc
	poly2tri/common/shapes.cc
)

add_library(pugixml
	pugixml/pugixml.cpp
)
set_target_properties(pugixml PROPERTIES CXX_STANDARD 11)
