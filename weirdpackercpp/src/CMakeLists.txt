add_subdirectory(deps)

add_library(weirdpacker
	Triangulate.cpp
	Triangulate.hpp
	Svg.cpp
	Svg.hpp
	BallTree.hpp
	Shape.cpp
	Shape.hpp
)

set_target_properties(weirdpacker PROPERTIES CXX_STANDARD 11)
target_include_directories(weirdpacker PUBLIC deps)
