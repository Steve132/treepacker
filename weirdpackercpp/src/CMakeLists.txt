add_subdirectory(deps)

add_library(weirdpacker
	Triangulate.cpp
	Triangulate.hpp
	Svg.cpp
	Svg.hpp
	BallTree.hpp
	Shape.cpp
	Shape.hpp
	Bezier.cpp
	Bezier.hpp
	Bezier.inl
)

set_target_properties(weirdpacker PROPERTIES CXX_STANDARD 11)
target_include_directories(weirdpacker PUBLIC deps)
target_link_libraries(weirdpacker pugixml poly2tri)


add_executable(test_shape
	test_shape.cpp)
target_link_libraries(test_shape weirdpacker pthread)
set_target_properties(test_shape PROPERTIES CXX_STANDARD 11)

add_executable(test_path2shape
	test_path2shape.cpp)
target_link_libraries(test_path2shape weirdpacker pthread)
set_target_properties(test_path2shape PROPERTIES CXX_STANDARD 11)
