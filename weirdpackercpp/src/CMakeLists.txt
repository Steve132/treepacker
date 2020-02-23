
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O3 -march=native -DNDEBUG")

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
	Renderer.cpp
	Renderer.hpp
	Cutout.hpp
	Cutout.cpp
	CutoutSet.hpp
	CutoutSet.cpp
	Optimize.cpp
	Optimize.hpp
	OptimizerCPU.cpp
	OptimizerCPU.hpp
)

find_package(OpenMP)
if(OpenMP_CXX_FOUND)
   # target_link_libraries(MyTarget PUBLIC OpenMP::OpenMP_CXX)
endif()

set_target_properties(weirdpacker PROPERTIES CXX_STANDARD 11)
target_include_directories(weirdpacker PUBLIC trail)

target_include_directories(weirdpacker PRIVATE deps)
target_link_libraries(weirdpacker trail poly2tri X11 OpenMP::OpenMP_CXX)

add_executable(test_shape
	test_shape.cpp)
target_link_libraries(test_shape weirdpacker pthread)
set_target_properties(test_shape PROPERTIES CXX_STANDARD 11)

add_executable(test_path2shape
	test_path2shape.cpp)
target_link_libraries(test_path2shape weirdpacker pthread)
set_target_properties(test_path2shape PROPERTIES CXX_STANDARD 11)


add_executable(test_cutoutpacking
	test_cutoutpacking.cpp)
target_link_libraries(test_cutoutpacking weirdpacker OpenMP::OpenMP_CXX pthread)
set_target_properties(test_cutoutpacking PROPERTIES CXX_STANDARD 11)
target_link_libraries(test_cutoutpacking profiler )
set_target_properties(test_cutoutpacking PROPERTIES COMPILE_DEFINITIONS_RELWITHDEBINFO -DWITHGPERFTOOLS)
