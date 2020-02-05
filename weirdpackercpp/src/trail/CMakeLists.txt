add_subdirectory(deps)

add_library(trail
	Triangulate.cpp
	Triangulate.hpp
	Svg.cpp
	Svg.hpp
	Shape.cpp
	Shape.hpp
	Bezier.cpp
	Bezier.hpp
	Bezier.inl
	Path.cpp
	Path.hpp
)

set_target_properties(trail PROPERTIES CXX_STANDARD 11)
target_include_directories(trail PUBLIC deps)
target_link_libraries(trail PRIVATE pugixml poly2tri)



