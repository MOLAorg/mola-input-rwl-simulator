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
FIND_PACKAGE( MRPT REQUIRED base maps obs gui graphs vision)

INCLUDE_DIRECTORIES(${${PROJECT_NAME}_SOURCE_DIR}/include)

IF(CMAKE_COMPILER_IS_GNUCXX)
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -pedantic")
ENDIF(CMAKE_COMPILER_IS_GNUCXX)

# Set optimized building:
IF(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -mtune=native")
ENDIF(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")



# --------------------------------
# TARGET : librwt
# --------------------------------
FILE(GLOB LIBHDRS "src/*.h")
FILE(GLOB LIBSRCS "src/*.cpp")
FILE(GLOB LIBHDRSPUB "include/*.h")
ADD_LIBRARY(rwt ${LIBHDRS} ${LIBSRCS} ${LIBHDRSPUB})

# --------------------------------
# TARGET : rwt-compiler
# --------------------------------
ADD_EXECUTABLE(rwt-compiler
	apps/main-compiler.cpp
	)
TARGET_LINK_LIBRARIES(rwt-compiler rwt ${MRPT_LIBS})
ADD_DEPENDENCIES(rwt-compiler rwt)

# --------------------------------
# TARGET : rwt-dataset-simulator
# --------------------------------
ADD_EXECUTABLE(rwt-dataset-simulator
	apps/main-dataset-simulator.cpp
	)
TARGET_LINK_LIBRARIES(rwt-dataset-simulator rwt ${MRPT_LIBS})
ADD_DEPENDENCIES(rwt-dataset-simulator rwt)


