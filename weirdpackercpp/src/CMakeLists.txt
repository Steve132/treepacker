add_subdirectory(deps)
add_subdirectory(trail)

add_library(weirdpacker
	BallTree.cpp
	BallTree.hpp
	Triangle.cpp
	Triangle.hpp
	Triangle.inl
	Mesh.cpp
	Mesh.hpp
)

set_target_properties(weirdpacker PROPERTIES CXX_STANDARD 11)
target_include_directories(weirdpacker PUBLIC trail)

target_include_directories(weirdpacker PRIVATE deps)
target_link_libraries(weirdpacker trail poly2tri)

add_executable(test_shape
	test_shape.cpp)
target_link_libraries(test_shape weirdpacker pthread)
set_target_properties(test_shape PROPERTIES CXX_STANDARD 11)

add_executable(test_path2shape
	test_path2shape.cpp)
target_link_libraries(test_path2shape weirdpacker pthread)
set_target_properties(test_path2shape PROPERTIES CXX_STANDARD 11)
