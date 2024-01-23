cmake_minimum_required(VERSION 3.28.1)
project(AeroliteEngine)
add_library(AeroliteEngine 
	src/AeroVec2.cpp
	src/AeroVec3.cpp
	src/AeroWorld2D.cpp
	src/Body2D.cpp
	src/Collision2D.cpp
	src/Constraint2D.cpp
	src/Contact2D.cpp
	src/Matrix3x3.cpp
	src/Matrix3x4.cpp
	src/Particle.cpp
	src/Shape.cpp)

target_include_directories(AeroliteEngine PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_features(AeroliteEngine PUBLIC cxx_std_17)