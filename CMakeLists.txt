SET(PROJECT_NAME DSO)
CMAKE_MINIMUM_REQUIRED(VERSION 3.10)
PROJECT(${PROJECT_NAME})

#set(CMAKE_VERBOSE_MAKEFILE ON)

# set(CMAKE_CXX_STANDARD 17)

set(BUILD_TYPE Release)
#set(BUILD_TYPE RelWithDebInfo)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIE")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pie")


set(EXECUTABLE_OUTPUT_PATH bin)
set(LIBRARY_OUTPUT_PATH lib)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# required libraries
find_package(SuiteParse REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS system thread) 

# set(CMAKE_PREFIX_PATH "/usr/local/lib/cmake/Pangolin" ${CMAKE_PREFIX_PATH})
list(APPEND CMAKE_MODULE_PATH "/usr/local/lib/cmake/Pangolin")
# optional libraries
find_package(LibZip QUIET)
find_package(Pangolin QUIET)
find_package(OpenCV QUIET)
find_package(PCL 1.8 REQUIRED)

find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)

include_directories(/usr/local/cuda-11.7/targets/x86_64-linux/include)

set(VTK_DIR "/usr/local/lib/cmake/vtk-9.3") #这一句对VTK成功编译起作用了
find_package(VTK REQUIRED)
include_directories(${VTK_INCLUDE_DIRS})
# link_directories(${VTK_INCLUDE_DIRS})
# find_package(VTK REQUIRED)
# include(${VTK_USE_FILE})
# include_directories(/usr/local/include/vtk-9.3)


if(VTK_FOUND)
message("vtk found ")
# message("VTK Include Dirs: ${VTK_INCLUDE_DIRS}")
# message("VTK Include Dirs: ${VTK_USE_FILE}")
endif()


# flags
add_definitions("-DENABLE_SSE")
set(CMAKE_CXX_FLAGS
   "${SSE_FLAGS} -O3 -g -std=c++0x -march=native"
#   "${SSE_FLAGS} -O3 -g -std=c++0x -fno-omit-frame-pointer"
)

if (MSVC)
     set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /EHsc")
endif (MSVC)

# Sources files
set(dso_SOURCE_FILES
  ${PROJECT_SOURCE_DIR}/src/FullSystem/FullSystem.cpp
  ${PROJECT_SOURCE_DIR}/src/FullSystem/FullSystemOptimize.cpp
  ${PROJECT_SOURCE_DIR}/src/FullSystem/FullSystemOptPoint.cpp
  ${PROJECT_SOURCE_DIR}/src/FullSystem/FullSystemDebugStuff.cpp
  ${PROJECT_SOURCE_DIR}/src/FullSystem/FullSystemMarginalize.cpp
  ${PROJECT_SOURCE_DIR}/src/FullSystem/Residuals.cpp
  ${PROJECT_SOURCE_DIR}/src/FullSystem/CoarseTracker.cpp
  ${PROJECT_SOURCE_DIR}/src/FullSystem/CoarseInitializer.cpp
  ${PROJECT_SOURCE_DIR}/src/FullSystem/ImmaturePoint.cpp
  ${PROJECT_SOURCE_DIR}/src/FullSystem/HessianBlocks.cpp
  ${PROJECT_SOURCE_DIR}/src/FullSystem/PixelSelector2.cpp
  ${PROJECT_SOURCE_DIR}/src/OptimizationBackend/EnergyFunctional.cpp
  ${PROJECT_SOURCE_DIR}/src/OptimizationBackend/AccumulatedTopHessian.cpp
  ${PROJECT_SOURCE_DIR}/src/OptimizationBackend/AccumulatedSCHessian.cpp
  ${PROJECT_SOURCE_DIR}/src/OptimizationBackend/EnergyFunctionalStructs.cpp
  ${PROJECT_SOURCE_DIR}/src/util/settings.cpp
  ${PROJECT_SOURCE_DIR}/src/util/Undistort.cpp
  ${PROJECT_SOURCE_DIR}/src/util/globalCalib.cpp
)



include_directories(
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/thirdparty/Sophus
  ${PROJECT_SOURCE_DIR}/thirdparty/sse2neon
  ${EIGEN3_INCLUDE_DIR}
) 

#Pangolin只有0.6版本的能找到，而且出现报错信息：
#PCL与Pangolin冲突Cuda：struct cudaPointerAttributes’ has no member named ‘memoryType’
#解决方法是：把memcpy.h文件中HAVE_CUDA全部该成：HAVE_CUDA_PANGOLIN

# decide if we have pangolin
if (Pangolin_FOUND)
	message("--- found PANGOLIN, compiling dso_pangolin library.")
	include_directories( ${Pangolin_INCLUDE_DIRS} ) 
	set(dso_pangolin_SOURCE_FILES 
	  ${PROJECT_SOURCE_DIR}/src/IOWrapper/Pangolin/KeyFrameDisplay.cpp
	  ${PROJECT_SOURCE_DIR}/src/IOWrapper/Pangolin/PangolinDSOViewer.cpp)
	set(HAS_PANGOLIN 1)
else ()
	message("--- could not find PANGOLIN, not compiling dso_pangolin library.")
	message("    this means there will be no 3D display / GUI available for dso_dataset.")
	set(dso_pangolin_SOURCE_FILES )
	set(HAS_PANGOLIN 0)
endif ()

# decide if we have openCV
if (OpenCV_FOUND)
	message("--- found OpenCV, compiling dso_opencv library.")
	include_directories( ${OpenCV_INCLUDE_DIRS} )
	set(dso_opencv_SOURCE_FILES 
	  ${PROJECT_SOURCE_DIR}/src/IOWrapper/OpenCV/ImageDisplay_OpenCV.cpp
	  ${PROJECT_SOURCE_DIR}/src/IOWrapper/OpenCV/ImageRW_OpenCV.cpp)
	set(HAS_OPENCV 1)
else ()
	message("--- could not find OpenCV, not compiling dso_opencv library.")
	message("    this means there will be no image display, and image read / load functionality.")
	set(dso_opencv_SOURCE_FILES 
	  ${PROJECT_SOURCE_DIR}/src/IOWrapper/ImageDisplay_dummy.cpp
	  ${PROJECT_SOURCE_DIR}/src/IOWrapper/ImageRW_dummy.cpp)
	set(HAS_OPENCV 0)
endif ()

# decide if we have ziplib.
if (LIBZIP_LIBRARY)
	message("--- found ziplib (${LIBZIP_VERSION}), compiling with zip capability.")
	add_definitions(-DHAS_ZIPLIB=1)
	include_directories( ${LIBZIP_INCLUDE_DIR_ZIP} ${LIBZIP_INCLUDE_DIR_ZIPCONF} ) 
else()
	message("--- not found ziplib (${LIBZIP_LIBRARY}), compiling without zip capability.")
	set(LIBZIP_LIBRARY "")
endif()

if(PCL_FOUND)
	message("--- found PCL, compiling dso_pcl library.")
	include_directories(${PCL_INCLUDE_DIRS})
	link_directories(${PCL_LIBRARY_DIRS})
	add_definitions(${PCL_DEFINITIONS})
	set(dso_pcl_SOURCE_FILES ${PROJECT_SOURCE_DIR}/src/pclProcesser/pclProcessor.cpp)
	set((HAS_PCL 1))
else()
	message("could not find pcl")
	set(HAS_PCL 0)
endif()

# compile main library.
include_directories( ${CSPARSE_INCLUDE_DIR} ${CHOLMOD_INCLUDE_DIR}) 
add_library(dso ${dso_SOURCE_FILES} ${dso_opencv_SOURCE_FILES} ${dso_pangolin_SOURCE_FILES} ${dso_pcl_SOURCE_FILES})

#set_property( TARGET dso APPEND_STRING PROPERTY COMPILE_FLAGS -Wall )
target_link_libraries(dso ${OpenCV_LIBS} ${PCL_LIBRARIES} ${VTK_LIBRARIES})

if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin") # OSX
    set(BOOST_THREAD_LIBRARY boost_thread-mt)
else()
    set(BOOST_THREAD_LIBRARY boost_thread)
endif()



# build main executable (only if we have both OpenCV and Pangolin)
if (OpenCV_FOUND AND Pangolin_FOUND)
	message("--- compiling dso_dataset.")
	add_executable(dso_dataset ${PROJECT_SOURCE_DIR}/src/main_dso_pangolin.cpp )
    target_link_libraries(dso_dataset dso boost_system cxsparse ${BOOST_THREAD_LIBRARY} ${LIBZIP_LIBRARY} ${Pangolin_LIBRARIES} ${OpenCV_LIBS})
else()
	message("--- not building dso_dataset, since either don't have openCV or Pangolin.")
endif()

