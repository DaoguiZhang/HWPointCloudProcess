file(GLOB src_head_list
    "*.h"
	"*.hpp"
	"CloudCompareDisplay/*.h"
	"Common/*.h"
	"meshing/*.h"
	"hw_sfm/*.h"
	"GCO/*.h"
	)
#"dso_src/*.h"
set (removed_qt_header ${src_head_list})
list(REMOVE_ITEM removed_qt_header ${CMAKE_CURRENT_SOURCE_DIR}/OpenGLWidget.h)
list(REMOVE_ITEM removed_qt_header ${CMAKE_CURRENT_SOURCE_DIR}/PolygonGLWindow.h)
list(REMOVE_ITEM removed_qt_header ${CMAKE_CURRENT_SOURCE_DIR}/PolygonWindow.h)
list(REMOVE_ITEM removed_qt_header ${CMAKE_CURRENT_SOURCE_DIR}/ImageWindow.h)


	
file(GLOB q_obj_head_list
	"CloudCompareDisplay/ccApplication.h"
	"CloudCompareDisplay/ccClipBox.h"
	"CloudCompareDisplay/ccColorRampShader.h"
	"CloudCompareDisplay/ccGLWidget.h"
	"CloudCompareDisplay/ccGLWindow.h"
	"CloudCompareDisplay/ccOctree.h"
	"CloudCompareDisplay/ccOctreeSpinBox.h"
	"CloudCompareDisplay/ccOverlayDialog.h"
	"CloudCompareDisplay/ccPickingHub.h"
	"CloudCompareDisplay/ccPointCloudLOD.h"
	"CloudCompareDisplay/ccProgressDialog.h"
	"CloudCompareDisplay/ccShader.h"
	"hw_db_tree.h"
	"mainwindow.h"
	"OpenGLWidget.h"
	"PolygonWindow.h"
	"PolygonGLWindow.h"
	"ImageWindow.h"
	"PolygonExtractionParametersWindow.h"
	)
	
file(GLOB src_cpp_list
    "*.cpp"
	"*.c"
	"CloudCompareDisplay/*.cpp"
	"Common/*.cpp"
	"meshing/*.cpp"
	"hw_sfm/*.cpp"
	"GCO/*.cpp"
	)
#"dso_src/*.cpp"
#QT
#由于生成的C++文件都是放在编译目录里的（在Shadow Building中有别于源码目录），
#所以还需要将编译目录包含到Include目录，否则编译器会找不到这些头文件
set(CMAKE_INCLUDE_CURRENT_DIR ON)

#Qt Quick程序中的QRC资源文件需要用rcc来进行预处理，生成相应.h和.cpp文件。
#QObject派生的C++类也需要通过moc进行处理。这些都是编译系统中所谓的Rules。
#CMake中通过打开下面两个选项可以自动对这些后缀的文件进行相应处理
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

#设置Qt安装路径，如"E:/programs/Qt5.12.0/5.12.0/msvc2017_64"
#set(CMAKE_PREFIX_PATH "../3rd_party/Qt5/lib/cmake")
set(CMAKE_PREFIX_PATH "D:/Qt/Qt5.8.0/5.8/msvc2015_64/lib/cmake")  
#set(CAMKE_INSTALL_PREFIX  "D:/vc_project_xht/HWBranch/install") 

#查找需要的qt库文件，最好每一个库都要写，qt也会根据依赖关系自动添加
find_package(Qt5Widgets REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5OpenGL REQUIRED)
find_package(Qt5OpenGLExtensions REQUIRED)
find_package(Qt5Xml REQUIRED)
find_package(Qt53DCore REQUIRED)
find_package(Qt53DExtras REQUIRED)
find_package(Qt53DInput REQUIRED)
find_package(Qt53DLogic REQUIRED)
find_package(Qt53DRender REQUIRED)
get_target_property(QtCore_location Qt5::Core LOCATION)
message("Qt5Core location :${QtCore_location}")
# include_directories("../3rd_party/Qt5/include")
# link_directories("../3rd_party/Qt5/lib")
#find_package(Qt5 COMPONENTS 3DCore 3DInput 3DLogic 3DRender REQUIRED)
#target_link_libraries(HWPointCloudProcess ${Qt5Widgets_LIBRARIES}) #添加Qt的链接库 3DAnimation

#查找设置当前文件夹中所有的ui文件
FILE(GLOB UI_FILES "./*.ui")

#通过Ui文件生成对应的头文件，一定要添加
qt5_wrap_ui(WRAP_FILES ${UI_FILES})

#包含Q_OBJECT的头文件，生成对应的moc_xxx.cpp
QT5_WRAP_CPP(HEADERS_MOC ${q_obj_head_list})

#包含Q_OBJECT的cpp文件，生成对应的xxx.moc
qt5_generate_moc(CloudCompareDisplay/ccPointCloudLOD.cpp ${CMAKE_CURRENT_BINARY_DIR}/ccPointCloudLOD.moc)
set(CPP_MOC ${CMAKE_CURRENT_BINARY_DIR}/ccPointCloudLOD.moc)

#添加.moc文件查找路径
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR})

#添加资源文件，非必须，一旦采用，注意修改相应的qrc文件名
#set(RCC_FILES "./mainwindow.qrc")
#qt5_add_resources(RCC_FILES mainwindow.qrc)
#set(RCC_FILES "*.qrc")

#将ui文件和生成文件整理在一个文件夹中，非必须
source_group("UI" FILES ${UI_FILES} ${WRAP_FILES} )

source_group("Q_OBJ_Header" FILES ${q_obj_head_list} )

source_group("GeneratedFiles" FILES ${HEADERS_MOC} ${CPP_MOC})


#链接器命令行，增加参数，忽略重定义
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /FORCE:MULTIPLE")

add_executable(HWPointCloudProcess ${src_head_list} 
                                   ${src_cpp_list}
								   ${HEADERS_MOC}
								   ${WRAP_FILES}
                                   ${CPP_MOC}
								   )
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../bin)    #可执行程序位于/bin/release
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_CURRENT_SOURCE_DIR}/../bin)
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_SOURCE_DIR}/../bin)


target_link_libraries(HWPointCloudProcess Qt5::Widgets Qt5::Core Qt5::Gui 
                                          Qt5::OpenGL Qt5::OpenGLExtensions
									      Qt5::Xml Qt5::3DCore Qt5::3DExtras
										  Qt5::3DInput Qt5::3DLogic Qt5::3DRender
										  )

target_link_libraries(HWPointCloudProcess
    ${PCL_LIBS}
	${OpenCV_LIBS}
	${FLANN_LIBS}
	${Boost_LIBS}
	${PlaneExtraction_LIBS}
	${TextureLTbC_LIBS}
	${mve_LIBS}
	${libE57_LIBS}
	${xerces_LIBS}
	${CGAL_LIBS}
	${glut_LIBS}
	${GFLAGS_LIBS}
	${GLOG_LIBS}
	${CERES_LIBS}
	#${FMT_LIBS}	#to do next
    opengl32
	)

#QT_NO_DEBUG
	
target_compile_definitions(HWPointCloudProcess PRIVATE GLOG_NO_ABBREVIATED_SEVERITIES )

add_subdirectory(rendering)
add_subdirectory(segmentation)