PROJECT(RecursiveWorldToolkit)

CMAKE_MINIMUM_REQUIRED(VERSION 2.4)
if(COMMAND cmake_policy)
      cmake_policy(SET CMP0003 NEW)  # Required by CMake 2.7+
endif(COMMAND cmake_policy)

SET(LIBRARY_OUTPUT_PATH ${${PROJECT_NAME}_BINARY_DIR}/lib CACHE PATH "Output directory for libraries" )
SET(EXECUTABLE_OUTPUT_PATH ${${PROJECT_NAME}_BINARY_DIR}/bin CACHE PATH "Output directory for applications" )

# --------------------------------------------------------------------------
#   The list of "libs" which can be included can be found in:
#     http://www.mrpt.org/Libraries
#
#   The dependencies of a library are automatically added, so you only 
#    need to specify the top-most libraries your code depend on.
# --------------------------------------------------------------------------
FIND_PACKAGE( MRPT REQUIRED base maps obs gui graphs)

INCLUDE_DIRECTORIES(${${PROJECT_NAME}_SOURCE_DIR}/include)

IF(CMAKE_COMPILER_IS_GNUCXX)
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -pedantic")
ENDIF(CMAKE_COMPILER_IS_GNUCXX)

# Set optimized building:
IF(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -mtune=native")
ENDIF(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")



# --------------------------------
# TARGET : librwl
# --------------------------------
FILE(GLOB LIBHDRS "src/*.h")
FILE(GLOB LIBSRCS "src/*.cpp")
FILE(GLOB LIBHDRSPUB "include/*.h")
ADD_LIBRARY(rwl ${LIBHDRS} ${LIBSRCS} ${LIBHDRSPUB})

# --------------------------------
# TARGET : rwl-compiler
# --------------------------------
ADD_EXECUTABLE(rwl-compiler
	apps/main-compiler.cpp
	)
TARGET_LINK_LIBRARIES(rwl-compiler rwl ${MRPT_LIBS})
ADD_DEPENDENCIES(rwl-compiler rwl)

# --------------------------------
# TARGET : rwl-dataset-simulator
# --------------------------------
ADD_EXECUTABLE(rwl-dataset-simulator
	apps/main-dataset-simulator.cpp  
	)
TARGET_LINK_LIBRARIES(rwl-dataset-simulator rwl ${MRPT_LIBS})
ADD_DEPENDENCIES(rwl-dataset-simulator rwl)


