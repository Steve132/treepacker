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
target_link_libraries(weirdpacker pugixml poly2tri)


add_executable(test_shape
	test_shape.cpp)
target_link_libraries(test_shape weirdpacker pthread)
set_target_properties(test_shape PROPERTIES CXX_STANDARD 11)
