cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(HWPointCloudProcess)

set(QMAKE_CXXFLAGS "-std=c++11") 

#ensure that nobody build in the source tree
if (${CMAKE_CURRENT_SOURCE_DIR} STREQUAL ${CMAKE_CURRENT_BINARY_DIR})
  message(FATAL_ERROR "Please do not use the root directory as Cmake output directory!
                       mkdir build; cd build; cmake ..")
endif()

add_compile_options(/bigobj)

include_directories(./ 
                    ./src
					./src/meshing
					./src/segmentation
                    )
#./src/dso_src					
#release as default
set(CMAKE_BUILD_TYPE "RELEASE")
Add_Definitions(-DNOMINMAX)


#CUDA
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/CUDA9.1/include")

#VCG
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/vcg)

#OpenCV
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/OpenCV/include")
link_directories("${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/OpenCV/lib")
#set(OpenCV_LIBS optimized opencv_world320
#                debug     opencv_world320d)
set(OpenCV_LIBS optimized opencv_highgui345
				optimized opencv_imgproc345
				optimized opencv_imgcodecs345
				optimized opencv_core345
				optimized opencv_features2d345
				optimized opencv_calib3d345
				optimized opencv_sfm345
				optimized opencv_line_descriptor345
				optimized opencv_img_hash345
				optimized opencv_structured_light345
				optimized opencv_fuzzy345
				optimized opencv_dnn345
				optimized opencv_rgbd345
				optimized opencv_xfeatures2d345
				optimized opencv_flann345
				optimized opencv_photo345.lib
                debug     opencv_highgui345d
				debug 	  opencv_imgproc345d
				debug 	  opencv_imgcodecs345d
				debug	  opencv_core345d
				debug 	  opencv_features2d345d
				debug     opencv_calib3d345d
				debug     opencv_sfm345d
				debug     opencv_line_descriptor345d
				debug     opencv_img_hash345d
				debug     opencv_structured_light345d
				debug     opencv_fuzzy345d
				debug     opencv_dnn345d
				debug     opencv_rgbd345d
				debug opencv_xfeatures2d345d
				debug opencv_flann345d
				debug opencv_photo345d.lib)

#Eigen3
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/eigen3")

#fmt
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/FMT/include")
link_directories("${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/FMT/lib")
set(FMT_LIBS optimized fmt
               debug   fmtd)


#Sophus
#include_directories("${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/sophus")
#include_directories("${CMAKE_CURRENT_SOURCE_DIR}/3rd_party")

#gflags
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/gflags/include")
link_directories("${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/gflags/lib")
set(GFLAGS_LIBS optimized gflags_static
				debug gflags_static_debug)
				
#glog
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/glog/include")
link_directories("${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/glog/lib")
set(GLOG_LIBS optimized glog
				debug glogd)

#ceres
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/ceres/include")
link_directories("${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/ceres/lib")
set(CERES_LIBS optimized ceres
				debug ceres-debug)

#json
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/json-develop/include")

#FLANN
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/FLANN/include")
link_directories("${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/FLANN/lib")
set(FLANN_LIBS optimized flann_s
               debug     flann_s-gd)
			   
#Zlib
#include_directories("${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/zlib/include")
#link_directories("${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/zlib/lib")
#set(ZLIB_LIBS optimized zlib
#               debug     zlibstatic)			   

#Boost
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/Boost/include/boost-1_69")
link_directories("${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/Boost/lib")
set(Boost_LIBS optimized libboost_atomic-vc140-mt-x64-1_69
               optimized libboost_chrono-vc140-mt-x64-1_69
               optimized libboost_container-vc140-mt-x64-1_69
               optimized libboost_context-vc140-mt-x64-1_69
               optimized libboost_contract-vc140-mt-x64-1_69
               optimized libboost_coroutine-vc140-mt-x64-1_69
			   optimized libboost_date_time-vc140-mt-x64-1_69
			   optimized libboost_exception-vc140-mt-x64-1_69
               debug     libboost_atomic-vc140-mt-gd-x64-1_69
			   debug     libboost_chrono-vc140-mt-gd-x64-1_69
			   debug     libboost_container-vc140-mt-gd-x64-1_69
			   debug     libboost_context-vc140-mt-gd-x64-1_69
			   debug     libboost_contract-vc140-mt-gd-x64-1_69
			   debug     libboost_coroutine-vc140-mt-gd-x64-1_69
			   debug     libboost_date_time-vc140-mt-gd-x64-1_69
			   debug     libboost_exception-vc140-mt-gd-x64-1_69)

#PCL
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/PCL/include/pcl-1.9")
link_directories("${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/PCL/lib")
set(PCL_LIBS   optimized pcl_io_release
               optimized pcl_surface_release
			   optimized pcl_common_release
			   optimized pcl_kdtree_release
			   optimized pcl_filters_release
			   optimized pcl_registration_release
			   optimized pcl_search_release
			   optimized pcl_segmentation_release
			   optimized pcl_features_release
			   debug     pcl_io_debug
               debug     pcl_surface_debug
			   debug     pcl_common_debug
			   debug     pcl_kdtree_debug
			   debug     pcl_filters_debug
			   debug     pcl_registration_debug
			   debug     pcl_search_debug
			   debug     pcl_segmentation_debug
			   debug	 pcl_features_debug
			   )

#TextureLTbC：siggraph2016_colorization
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/TextureLTbC/include")
link_directories("${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/TextureLTbC/lib/release")
set(TextureLTbC_LIBS optimized mrf)

#Tbb
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/tbb-tbb_2020/include")
link_directories("${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/tbb-tbb_2020")
set(Tbb_LIBS optimized tbb)

#rayint
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/rayint/libs")

#plane_extraction
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/plane_extraction/include")
link_directories("${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/plane_extraction/lib")
set(PlaneExtraction_LIBS optimized QRANSAC_SD_MISC_LIB
                         optimized QRANSAC_SD_PRIM_SHAPES_LIB)

#mve
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/mve/include")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/mve/3rdparty/include")
link_directories("${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/mve/lib")
link_directories("${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/mve/3rdparty/lib")
set(mve_LIBS optimized mve
             #optimized mve_dmrecon
			 #optimized mve_fssr
			 #optimized mve_math
			 #optimized mve_ogl
			 #optimized mve_sfm
			 optimized mve_util
			 optimized tiff
			 optimized jpeg
			 optimized libpng16
			 optimized zlib
			 debug mve_d
			 #debug mve_dmrecon_d
			 #debug mve_fssr_d
			 #debug mve_math_d
			 #debug mve_ogl_d
			 #debug mve_sfm_d
             debug mve_util_d
			 debug tiff_d
			 debug jpeg_d
			 debug libpng16_d
			 debug zlib_d)

#libE57
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/libE57/include")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/libE57/include/time_conversion")
link_directories("${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/libE57/lib/Release")
link_directories("${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/libE57/lib/Debug")
set(libE57_LIBS optimized E57RefImpl
                optimized time_conversion
				debug     E57RefImpl-d)

#xerces
link_directories("${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/xerces/lib/Release")
set(xerces_LIBS optimized xerces-c_3
                debug     xerces-c_3D)

# #glut
# include_directories("${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/glut3.7/include")
# link_directories("${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/glut3.7/lib")
# set(glut_LIBS optimized glut32)

#CGAL
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/CGAL-5.0/include")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/CGAL-5.0/auxiliary/gmp/include")
link_directories("${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/CGAL-5.0/auxiliary/gmp/lib")
set(CGAL_LIBS optimized libgmp-10)

#assimp
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/assimp/include")
link_directories("${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/assimp/lib")
set(ASSIMP_LIBS assimp-vc140-mt.lib)

#GL
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/GL")
link_directories("${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/GL")

#glm
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/glm/include")

#glew
# include_directories("${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/glew/include")
link_directories("${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/glew/lib/Release/x64")
set(GLEW_LIBS glew32s.lib)

#Qt
# include_directories("${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/Qt5/include")
# include_directories("${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/Qt5/include/QtCore")
# include_directories("${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/Qt5/include/QtWidgets")
# include_directories("${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/Qt5/include/QtGui")
# include_directories("${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/Qt5/include/QtOpenGL")
# include_directories("${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/Qt5/include/QtOpenGLExtensions")
# include_directories("${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/Qt5/include/QtXml")
# include_directories("${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/Qt5/include/Qt3DCore")
# include_directories("${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/Qt5/include/Qt3DExtras")
# include_directories("${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/Qt5/include/Qt3DInput")
# include_directories("${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/Qt5/include/Qt3DLogic")
# include_directories("${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/Qt5/include/Qt3DRender")
# link_directories("${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/Qt/lib")

#openmp
FIND_PACKAGE( OpenMP REQUIRED)
if(OPENMP_FOUND)
   message("OPENMP FOUND")
   set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
   set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

#target_compile_definitions(HWPointCloudProcess PRIVATE GLOG_NO_ABBREVIATED_SEVERITIES)

#Add the actural source files
add_subdirectory(src)
add_subdirectory(demo/image_generator)
add_subdirectory(demo/pointcloud_filter)
add_subdirectory(demo/query_generator)
#set_property(DIRECTORY PROPERITY VS_STARTUP_PROJECT HWPointCloudProcess)
