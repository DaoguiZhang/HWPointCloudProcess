#include "mainwindow.h"
#include<iostream>
#include<QDebug>
#include<QSignalMapper>
#include <random>
#include <unordered_map>
#include<vector>

//io
#include"tinyply.h"
#include <direct.h>

//CCLib Includes
#include <CloudCompareDisplay/CloudSamplingTools.h>
#include <CloudCompareDisplay/Delaunay2dMesh.h>
#include <CloudCompareDisplay/Jacobi.h>
#include <CloudCompareDisplay/MeshSamplingTools.h>
#include <CloudCompareDisplay/NormalDistribution.h>
#include <Common/ParallelSort.h>
#include <CloudCompareDisplay/PointCloud.h>
#include <CloudCompareDisplay/ScalarFieldTools.h>
#include <CloudCompareDisplay/StatisticalTestingTools.h>
#include <CloudCompareDisplay/WeibullDistribution.h>

//for tests
#include <CloudCompareDisplay/ChamferDistanceTransform.h>
#include <CloudCompareDisplay/SaitoSquaredDistanceTransform.h>

//qCC_db
#include <CloudCompareDisplay/cc2DLabel.h>
#include <CloudCompareDisplay/cc2DViewportObject.h>
#include <CloudCompareDisplay/ccCameraSensor.h>
#include <CloudCompareDisplay/ccColorScalesManager.h>
#include <CloudCompareDisplay/ccFacet.h>
#include <CloudCompareDisplay/ccFileUtils.h>
#include <CloudCompareDisplay/ccGBLSensor.h>
#include <CloudCompareDisplay/ccImage.h>
#include <CloudCompareDisplay/ccKdTree.h>
#include <CloudCompareDisplay/ccPlane.h>
#include <CloudCompareDisplay/ccProgressDialog.h>
#include <CloudCompareDisplay/ccQuadric.h>
#include <CloudCompareDisplay/ccSphere.h>
#include <CloudCompareDisplay/ccSubMesh.h>
#include <CloudCompareDisplay/ccPolyline.h>

//PICK TOOL
#include <CloudCompareDisplay/ccPickingHub.h>
#include<Helper.h>

//extract plane
#include <RansacShapeDetector.h>
#include <PlanePrimitiveShapeConstructor.h>
#include <CylinderPrimitiveShapeConstructor.h>
#include <SpherePrimitiveShapeConstructor.h>
#include <ConePrimitiveShapeConstructor.h>
#include <TorusPrimitiveShapeConstructor.h>
#include <PlanePrimitiveShape.h>
#include "nanoflann.hpp"
#include "utils.h"
#include "PointGrowAngleDis.h"

//octree
#include <Common/DgmOctree.h>
#include<CloudCompareDisplay/FastMarchingForPropagation.h>
#include "PolygonWindow.h"

// DoSamplingAction
#include "hw_to_pcl_cloud.h"

//Do Convert Cam
#include "hw_cam_model.h"

//json文件库
#include<nlohmann/json.hpp>
#include <iomanip>
//#include<z>
//test Sophus库
//#include <so3.hpp>
//#include <se3.hpp>

//#include"FullSystem.h"

#define MAX_BOX_LENGHT 1e6
#define USING_REMOVED_OBJ 1
#define USING_REMOVE_AND_MERGE 1
#define SHOW_PLANE_VIEWS 0
#define USING_EXTRACTED_PLANES 1	//set the planes(if true, save the extracted planes points)

//using namespace std;

//global static pointer (as there should only be one instance of MainWindow!)
static MainWindow* s_instance = nullptr;

//default file filter separator
//static const QString s_fileFilterSeparator(";;");

enum PickingOperation {
	NO_PICKING_OPERATION,
	PICKING_ROTATION_CENTER,
	PICKING_LEVEL_POINTS,
	PICKING_SELECT_OBJ,
};
static ccGLWindow* s_pickingWindow = nullptr;
static PickingOperation s_currentPickingOperation = NO_PICKING_OPERATION;
static std::vector<cc2DLabel*> s_levelLabels;
static ccPointCloud* s_levelMarkersCloud = nullptr;
static ccHObject* s_levelEntity = nullptr;

MainWindow::MainWindow(QWidget *parent)
	: QMainWindow(parent)
	, m_uiFrozen(false)
	, m_viewModePopupButton(nullptr)
	, m_pivotVisibilityPopupButton(nullptr)
	, m_FirstShow(true)
	, m_pickingHub(nullptr)
{
	ui.setupUi(this);

	//
	EditUIShow();
	//这边可以用setGeometry设置各个框的大小
	initial();
	//CreateOpenglWindow();
	CreateConnection();

	//opengl
	m_ccRoot1 = new ccHObject("root");
	//MDI Area
	{
		m_mdiArea = new QMdiArea(this);
		setCentralWidget(m_mdiArea);
		//connect(m_mdiArea, &QMdiArea::subWindowActivated, this, &MainWindow::updateMenus);
		connect(m_mdiArea, &QMdiArea::subWindowActivated, this, &MainWindow::on3DViewActivated);
		m_mdiArea->installEventFilter(this);
	}

	m_root_gl_ = new3DView(true);

	//picking hub
	{
		m_pickingHub = new ccPickingHub(this, this);
		connect(m_mdiArea, &QMdiArea::subWindowActivated, m_pickingHub, &ccPickingHub::onActiveWindowChanged);
	}

	//plane
	plane_state_ = false;
	m_polyobj_ = nullptr;

	////test
	//nlohmann::json myjson = nlohmann::json::array({ { "one", 1 },{ "two", 2 } });
	//char text[] = R"(
 //   {
 //       "Image": {
 //           "Width":  800,
 //           "Height": 600,
 //           "Title":  "View from 15th Floor",
 //           "Thumbnail": {
 //               "Url":    "http://www.example.com/image/481989943",
 //               "Height": 125,
 //               "Width":  100
 //           },
 //           "Animated" : false,
 //           "IDs": [116, 943, 234, 38793]
 //       }
 //   }
 //   )";
	//// parse and serialize JSON
	//nlohmann::json j_complete = nlohmann::json::parse(text);
	//std::cout << std::setw(4) << j_complete << "\n\n";
	//end test

	//std::string file_name("D:\\Huawei\\single_plane\\single.ply");
	//std::string file_name("D:\\Huawei\\simple_planc\\simple.ply");
	//std::string file_name("D:\\Huawei\\full_planc\\clean.ply");
	//std::string file_name("D:\\Huawei\\thin_plane\\plane.ply");
	//std::string file_name("D:\\Huawei\\outdoor_planc\\ground_merge.ply");
	//std::string file_name("F:\\dense_sample\\cut.ply");
	//std::string file_name("F:\\dfwg002\\dfwg.ply");
	//std::string file_name("F:\\dfwg005\\0.2downsampled.ply");
	//std::string file_name("D:\\Huawei\\e1_test\\clean.ply");
	//std::string file_name("F:\\huawei_data\\beijing_research_laboratory\\F1\\2019-12-17_14.30.00\\beijing_pointcloud_001.ply");
	//std::string file_name("D:\\Huawei\\e1_test\\subsampled.ply");
	//std::string file_name("D:\\Huawei\\hole\\roof.ply");
	//std::string file_name("F:\\huawei_data\\beijing_research_laboratory\\F1\\2019-12-17_14.30.00\\beijing_pointcloud_001_merge_polygon_opt.obj");
	//std::string file_name("D:\\Huawei\\test.obj");
	//std::string file_name("D:\\Huawei\\full_planc\\520_polygon.obj");
	//std::string file_name("D:\\Huawei\\e1_test\\subsampled_merge_polygon_opt.obj");
	//ReadObj(file_name);
	//AddViewListAItem(file_name);
	//active_objects_names_.emplace_back(db_tree_->current_element_idx_);
	//ShowPolygonAction();
	//ReadPly(file_name, false);
	////std::cout << "ReadPly done, press any key to continue;" << std::endl;
	////getchar();
	//AddViewListAItem(file_name);
	//active_objects_names_.emplace_back(db_tree_->current_element_idx_);
	//HW::HWParams::getInstance().plane_params_.ceil_height_ = 20.0;
	//DoExtractPlaneAction();
	////QString current_term_qstr = ui.DbItemListWidget->currentItem()->text();
	////db_tree_->current_element_idx_ = current_term_qstr.toStdString();
	//db_tree_->current_element_idx_ = active_objects_names_[1];
	//printf("current item: %s\n", db_tree_->current_element_idx_.c_str());
	////CopyHWPCToCCPC(db_tree_->current_element_idx_);
	//ShowAllActivedObjects();
	//clock_t start_time, end_time;
	//start_time = clock();
	//DoPolygonSnapAction();
	//end_time = clock();
	//std::cout << "DoPolygonSnapAction time: " << (double)(end_time - start_time) / CLOCKS_PER_SEC << "s" << std::endl;

#if 0

	// 沿Z轴转90度的旋转矩阵
	Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d(0, 0, 1)).toRotationMatrix();
	 
	Sophus::SO3d SO3_R(R);               // Sophus::SO(3)可以直接从旋转矩阵构造
	//Sophus::SO3d SO3_v(0.0,0.0,M_PI/2);  // 亦可从旋转向量构造
	Eigen::Quaterniond q(R);            // 或者四元数
	Sophus::SO3d SO3_q(q);


	// 上述表达方式都是等价的
	// 输出SO(3)时，以so(3)形式输出
	std::cout << "SO(3) from matrix: " << SO3_R.matrix() << endl;
	//std::cout << "SO(3) from vector: " << SO3_v.matrix() << endl;
	std::cout << "SO(3) from quaternion :" << SO3_q.matrix() << endl;

	// 使用对数映射获得它的李代数
	Eigen::Vector3d so3 = SO3_R.log();
	std::cout << "so3 = " << so3.transpose() << endl;
	// hat 为向量到反对称矩阵
	std::cout << "so3 hat=\n" << Sophus::SO3d::hat(so3) << endl;
	// 相对的，vee为反对称到向量
	std::cout << "so3 hat vee= " << Sophus::SO3d::vee(Sophus::SO3d::hat(so3)).transpose() << endl; // transpose纯粹是为了输出美观一些

	// 增量扰动模型的更新
	Eigen::Vector3d update_so3(1e-4, 0, 0); //假设更新量为这么多
	Sophus::SO3d SO3_updated = Sophus::SO3d::exp(update_so3)*SO3_R;
	std::cerr << "SO3 updated = " << SO3_updated.matrix() << endl;

	/********************萌萌的分割线*****************************/
	std::cout << "************我是分割线*************" << endl;
	// 对SE(3)操作大同小异
	Eigen::Vector3d t(1, 0, 0);           // 沿X轴平移1
	Sophus::SE3d SE3_Rt(R, t);           // 从R,t构造SE(3)
	Sophus::SE3d SE3_qt(q, t);            // 从q,t构造SE(3)
	std::cout << "SE3 from R,t= " << endl << SE3_Rt.matrix() << endl;
	std::cout << "SE3 from q,t= " << endl << SE3_qt.matrix() << endl;
	// 李代数se(3) 是一个六维向量，方便起见先typedef一下
	typedef Eigen::Matrix<double, 6, 1> Vector6d;
	Vector6d se3 = SE3_Rt.log();
	std::cout << "se3 = " << se3.transpose() << endl;
	// 观察输出，会发现在Sophus中，se(3)的平移在前，旋转在后.
	// 同样的，有hat和vee两个算符
	std::cout << "se3 hat = " << endl << Sophus::SE3d::hat(se3) << endl;
	std::cout << "se3 hat vee = " << Sophus::SE3d::vee(Sophus::SE3d::hat(se3)).transpose() << endl;

	// 最后，演示一下更新
	Vector6d update_se3; //更新量
	update_se3.setZero();
	update_se3(0, 0) = 1e-4;
	Sophus::SE3d SE3_updated = Sophus::SE3d::exp(update_se3)*SE3_Rt;
	std::cout << "SE3 updated = " << endl << SE3_updated.matrix() << endl;
#endif

}

MainWindow::~MainWindow()
{
	if (db_tree_ != NULL)
	{
		delete db_tree_;
		db_tree_ = NULL;
	}
	for (int i = 0; i < origin_elements_.size(); ++i)
	{
		if (origin_elements_[i] != NULL)
		{
			delete origin_elements_[i];
			origin_elements_[i];
		}
	}
	for (int i = 0; i < processes_vec_.size(); ++i)
	{
		if (processes_vec_[i] != NULL)
		{
			delete processes_vec_[i];
		}
	}
	//if (opengl_window_ != nullptr) delete opengl_window_;
	if (layout_ != nullptr) delete layout_;
	//if (camera_ != nullptr)delete camera_;

	//-------opengl---------//
	for (int i = 0; i < getGLWindowCount(); ++i)
	{
		getGLWindow(i)->setSceneDB(0);
	}
	m_mdiArea->closeAllSubWindows();

	if (m_ccRoot1)
	{
		delete m_ccRoot1;
		m_ccRoot1 = nullptr;
	}

	if (pop_menu_)
		delete pop_menu_;

	if (selected_ccobject)
		delete selected_ccobject;

	if (m_plane_window_)
		delete m_plane_window_;

	//恢复原来的状态
	if (m_plane_)
		delete m_plane_;

	if (loaded_scenes_cams_)
	{
		delete loaded_scenes_cams_;
		loaded_scenes_cams_ = NULL;
	}
}

void MainWindow::EditUIShow()
{
	//this->resize(1000, 800);
	//ui.DockableDbTree->resize(322, 307);
	//ui.dockWidgetContents->resize(322, 307);
	//ui.treeView_2->setGeometry(0, 10, 344, 89);
	//ui.DockableParameters->setGeometry(0, 364, 344, 600);
	//ui.dockWidgetContents_2->setGeometry(0, 10, 344, 590);
	//this->ui.toolBox->currentChanged(3);
	setWindowTitle(QStringLiteral("HW Pointcloud Process"));
	this->ui.toolBox->setCurrentIndex(0);
	this->ui.UpsampleMethodsComboBox->addItem("None");
	this->ui.UpsampleMethodsComboBox->addItem("Sample Local Plane");
	this->ui.UpsampleMethodsComboBox->addItem("Random Uniform Density");
	this->ui.UpsampleMethodsComboBox->addItem("Voxel Grid Dilation");
	this->ui.SampleLocalGroupBox->setDisabled(true);
	this->ui.RandomUniformGroupBox->setDisabled(true);
	this->ui.DbItemListWidget->setContextMenuPolicy(Qt::CustomContextMenu);

	//
	pop_menu_ = new QMenu(this);
	add_item_ = pop_menu_->addAction("add");
	delete_item_ = pop_menu_->addAction("delete");
	modify_item_ = pop_menu_->addAction("modify");
	add_item_->setEnabled(true);
	delete_item_->setDisabled(true);
	modify_item_->setDisabled(true);
	pop_menu_->setVisible(false);
}

void MainWindow::initial()
{
	first_show = true;
	db_tree_ = new HW::HWDBTree();
	//camera_ = new PerspectiveCamera();
}

//-------------------IO input-----------------//
bool MainWindow::ReadPly(const std::string& file_path, bool is_binary)
{
	//将ReadPly这一块读入变得general，这样不会出现因为格式得小问题，出现闪退现象
	std::ifstream fhd(file_path, std::ios::binary);	//(file_path, std::ios::binary)
	/*if (is_binary)
	{
		std::cerr <<"111111111111111111" << std::endl;
		fhd.open(file_path, std::ios::binary);
	}
	else
	{
		std::cerr << "22222222222222222222" << std::endl;
		fhd.open(file_path, std::ios::in);
	}*/
	if (fhd.fail()) throw std::runtime_error("failed to open " + file_path);
	//std::cout << "test" << std::endl;
	//system("pause");
	tinyply::PlyFile file_ply;
	file_ply.parse_header(fhd);

	bool has_vertex_flag = false;
	bool has_normal_flag = false;
	bool has_color_flag = false;
	bool has_alpha_flag = false;
	bool has_faces_flag = false;

	std::cout << "........................................................................\n";
	for (auto c : file_ply.get_comments()) std::cout << "Comment: " << c << std::endl;

	for (auto e : file_ply.get_elements())
	{
		std::cout << "element - " << e.name << " (" << e.size << ")" << std::endl;
		if (e.name == "face" && e.size > 0)
			has_faces_flag = true;

		for (auto p : e.properties)
		{
			if (p.name == "x")
				has_vertex_flag = true;
			if (p.name == "nx")
				has_normal_flag = true;
			if (p.name == "red")
				has_color_flag = true;
			if (p.name == "alpha")
				has_alpha_flag = true;

			std::cout << "\tproperty - " << p.name << " (" << tinyply::PropertyTable[p.propertyType].str << ")" << std::endl;
		}
	}
	std::cout << "........................................................................\n";

	//从文件头判断ply里面得元素
	std::shared_ptr<tinyply::PlyData> face_info, point_vertices, point_normal, point_color;

	//通过分析头文件，可以分开读取ply文件
	if (!has_faces_flag)
	{
		//读取点云
		HW::HWPointCloud* added_point_cloud = new HW::HWPointCloud();
		if (has_vertex_flag)
		{
			try
			{
				point_vertices = file_ply.request_properties_from_element("vertex", { "x", "y", "z" });
			}
			catch (const std::exception & e)
			{
				std::cerr << "tinyply exception: " << e.what() << std::endl;
				return false;
			}
		}
		else
		{
			std::cout << "read ply vertex failed." << std::endl;
			return false;
		}

		if (has_normal_flag)
		{
			try
			{
				point_normal = file_ply.request_properties_from_element("vertex", { "nx", "ny", "nz" });
			}
			catch (const std::exception & e)
			{
				std::cerr << "tinyply exception: " << e.what() << std::endl;
				return false;
			}
		}

		if (has_color_flag)
		{
			if (!has_alpha_flag)
			{
				try
				{
					point_color = file_ply.request_properties_from_element("vertex", { "red", "green", "blue"});
				}
				catch (const std::exception & e)
				{
					std::cerr << "tinyply exception: " << e.what() << std::endl;
					return false;
				}
			}
			else
			{
				try
				{
					point_color = file_ply.request_properties_from_element("vertex", { "red", "green", "blue", "alpha" });
				}
				catch (const std::exception & e)
				{
					std::cerr << "tinyply exception: " << e.what() << std::endl;
					return false;
				}
			}
		}

		//将主体读入到file_ply中
		file_ply.read(fhd);

		if (point_vertices) std::cout << "\tRead " << point_vertices->count << " total vertices " << std::endl;
		if (point_normal) std::cout << "\tRead " << point_normal->count << " total vertex normals " << std::endl;
		//std::system("pause");
		if (point_color) std::cout << "\tRead " << point_color->count << " total vertex color " << std::endl;
		
		//把ply读取的vertex拷贝到HWPointCloud
		added_point_cloud->CopyPointsFromPlyData(point_vertices);
		if(has_normal_flag)
			added_point_cloud->CopyNormalsFromPlyData(point_normal);
		//std::cout << "asdfsadfs" << std::endl;
		//std::system("pause");

		if (has_alpha_flag)
		{
			std::vector<uchar4> temp_color_vec;
			const size_t num_color_bytes = point_color->buffer.size_bytes();
			temp_color_vec.resize(point_color->count);
			std::memcpy(temp_color_vec.data(), point_color->buffer.get(), num_color_bytes);

			for (int i = 0; i < temp_color_vec.size(); ++i)
			{
				uchar3 one_point_color = make_uchar3(temp_color_vec[i].x, temp_color_vec[i].y, temp_color_vec[i].z);
				//system("pause");
				added_point_cloud->AddColor(one_point_color);
			}
		}
		else if(has_color_flag)
		{
			added_point_cloud->CopyColorsFromPlyData(point_color);
		}

		//将HWPointCloud传输到ccPointCloud,并且显示到系统界面
		HW::HWObject* element_add = added_point_cloud;
		element_add->SetObjectType(HW::ElementType::kHWPointCloud);
		db_tree_->current_element_idx_ = file_path;
		std::cout << "load current element idx: " << db_tree_->current_element_idx_ << std::endl;
		element_add->SetObjectName(db_tree_->current_element_idx_);
		db_tree_->AccessTreeElements().insert(std::pair<std::string, HW::HWObject*>(db_tree_->current_element_idx_, element_add));

		//插入对象
		CopyHWObjToCCObj(db_tree_->current_element_idx_);
		ShowAllActivedObjects();
	}
	else
	{
		//printf("asdfsdfsdf\n");
		//system("pause");
		//这个就是mesh,读取mesh
		HW::HWMesh* add_mesh = new HW::HWMesh();

		if (has_vertex_flag)
		{
			try
			{
				point_vertices = file_ply.request_properties_from_element("vertex", { "x", "y", "z" });
			}
			catch (const std::exception & e)
			{
				std::cerr << "tinyply exception: " << e.what() << std::endl;
				return false;
			}
		}
		else
		{
			std::cout << "read ply vertex failed." << std::endl;
			return false;
		}

		if (has_normal_flag)
		{
			try
			{
				point_normal = file_ply.request_properties_from_element("vertex", { "nx", "ny", "nz" });
			}
			catch (const std::exception & e)
			{
				std::cerr << "tinyply exception: " << e.what() << std::endl;
				return false;
			}
		}

		if (has_color_flag)
		{
			if (!has_alpha_flag)
			{
				try
				{
					point_color = file_ply.request_properties_from_element("vertex", { "red", "green", "blue" });
				}
				catch (const std::exception & e)
				{
					std::cerr << "tinyply exception: " << e.what() << std::endl;
					return false;
				}
			}
			else
			{
				try
				{
					point_color = file_ply.request_properties_from_element("vertex", { "red", "green", "blue", "alpha" });
				}
				catch (const std::exception & e)
				{
					std::cerr << "tinyply exception: " << e.what() << std::endl;
					return false;
				}
			}
		}

		face_info = file_ply.request_properties_from_element("face", { "vertex_indices" });

		//将主体读入到file_ply中
		file_ply.read(fhd);

		//read ply file to HWMesh
		if (point_vertices) std::cout << "\tRead " << point_vertices->count << " total vertices " << std::endl;
		if (point_normal) std::cout << "\tRead " << point_normal->count << " total vertex normals " << std::endl;
		if (face_info) std::cout << "\tRead " << face_info->count << " total faces " << std::endl;
		//system("pause");

		//system("pause");
		if (has_vertex_flag) {
			add_mesh->CopyPointsFromPlyData(point_vertices);
		}
		//std::system("pause");
		if (has_normal_flag) {
			add_mesh->CopyNormalsFromPlyData(point_normal);
		}
		add_mesh->CopyFacesFromPlyData(face_info);

		//system("pause");

		if (has_color_flag)
		{
			if (!has_alpha_flag)
			{
				add_mesh->CopyPointsColorsFromPlyData(point_color);
			}
			else
			{
				std::vector<uchar4> temp_color_vec;
				const size_t num_color_bytes = point_color->buffer.size_bytes();
				temp_color_vec.resize(point_color->count);
				std::memcpy(temp_color_vec.data(), point_color->buffer.get(), num_color_bytes);

				for (int i = 0; i < temp_color_vec.size(); ++i)
				{
					uchar3 one_point_color = make_uchar3(temp_color_vec[i].x, temp_color_vec[i].y, temp_color_vec[i].z);
					add_mesh->AddColor(one_point_color);
				}
			}
		}

		HW::HWObject* element_add = add_mesh;
		element_add->SetObjectType(HW::ElementType::kHWMesh);
		db_tree_->current_element_idx_ = file_path;
		std::cout << "load current element idx: " << db_tree_->current_element_idx_ << std::endl;
		element_add->SetObjectName(db_tree_->current_element_idx_);
		db_tree_->AccessTreeElements().insert(std::pair<std::string, HW::HWObject*>(db_tree_->current_element_idx_, element_add));

		CopyHWObjToCCObj(db_tree_->current_element_idx_);
		ShowAllActivedObjects();
	}

#if 0
#if 1
	std::ifstream fhd(file_path, std::ios::binary);
	if (fhd.fail()) throw std::runtime_error("failed to open " + file_path);
	//std::cout << "test" << std::endl;
	//system("pause");
	tinyply::PlyFile file_ply;
	file_ply.parse_header(fhd);
	std::shared_ptr<tinyply::PlyData> face_info, point_vertices, point_normal, point_color;
	//system("pause");
	//std::system("pause");
	face_info = file_ply.request_properties_from_element("face", { "vertex_indices" });
	//std::system("pause");
	if (!face_info->count)
	{
		HW::HWPointCloud* added_point_cloud = new HW::HWPointCloud();
		try
		{
			point_vertices = file_ply.request_properties_from_element("vertex", { "x", "y", "z" });
		}
		catch (const std::exception & e)
		{
			std::cerr << "tinyply exception: " << e.what() << std::endl;
			return false;
		}

		try
		{
			point_normal = file_ply.request_properties_from_element("vertex", { "nx", "ny", "nz" });
		}
		catch (const std::exception & e)
		{
			std::cerr << "tinyply exception: " << e.what() << std::endl;
			return false;
		}

		if (has_color)
		{
			//point_color = file_ply.request_properties_from_element("vertex", { "red", "green", "blue", "alpha" });
			try
			{
				point_color = file_ply.request_properties_from_element("vertex", { "red", "green", "blue", "alpha" });
			}
			catch (const std::exception & e)
			{
				std::cerr << "tinyply exception: " << e.what() << std::endl;
				return false;
			}
		}

		//将主体读入到file_ply中
		file_ply.read(fhd);
		//std::cout << "load current element idx: ";
		//std::system("pause");

		/*std::vector<float3> temp_vertices;
		std::vector<float3> temp_normals;

		const size_t num_vertices_bytes = point_vertices->buffer.size_bytes();
		temp_vertices.resize(point_vertices->count);
		std::memcpy(temp_vertices.data(), point_vertices->buffer.get(), num_vertices_bytes);

		const size_t num_normal_bytes = point_normal->buffer.size_bytes();
		temp_normals.resize(point_normal->count);
		std::memcpy(temp_normals.data(), point_normal->buffer.get(), num_normal_bytes);
		for (int i = 0; i < temp_vertices.size(); ++i)
		{
			added_point_cloud->AddPoint(temp_vertices[i]);
		}
		for (int i = 0; i < temp_normals.size(); ++i)
		{
			added_point_cloud->AddNormal(temp_normals[i]);
		}*/
		//printf()
		//read ply file to HWPointCloud
		//std::system("pause");
		if (point_vertices) std::cout << "\tRead " << point_vertices->count << " total vertices " << std::endl;
		//std::system("pause");
		if (point_normal) std::cout << "\tRead " << point_normal->count << " total vertex normals " << std::endl;
		//std::system("pause");
		if (point_color) std::cout << "\tRead " << point_color->count << " total vertex color " << std::endl;

		added_point_cloud->CopyPointsFromPlyData(point_vertices);
		added_point_cloud->CopyNormalsFromPlyData(point_normal);

		//printf("point_color")
#if 1
		if (has_color)
		{
			std::vector<uchar4> temp_color_vec;
			const size_t num_color_bytes = point_color->buffer.size_bytes();
			temp_color_vec.resize(point_color->count);
			std::memcpy(temp_color_vec.data(), point_color->buffer.get(), num_color_bytes);

			for (int i = 0; i < temp_color_vec.size(); ++i)
			{
				uchar3 one_point_color = make_uchar3(temp_color_vec[i].x, temp_color_vec[i].y, temp_color_vec[i].z);
				added_point_cloud->AddColor(one_point_color);
			}
		}
#endif
		//added_point_cloud->CopyColorFromPlyData(point_color);
		HW::HWObject* element_add = added_point_cloud;
		element_add->SetObjectType(HW::ElementType::kHWPointCloud);

		db_tree_->current_element_idx_ = file_path;
		std::cout << "load current element idx: " << db_tree_->current_element_idx_ << std::endl;

		db_tree_->AccessTreeElements().insert(std::pair<std::string, HW::HWObject*>(db_tree_->current_element_idx_, element_add));
		QListWidgetItem* aitem = new QListWidgetItem();
		aitem->setText(file_path.c_str());
		aitem->setCheckState(Qt::Checked);
		aitem->setFlags(Qt::ItemIsSelectable | Qt::ItemIsEditable | Qt::ItemIsUserCheckable | Qt::ItemIsEnabled);
		//增加一个Item到view list 中
		this->ui.DbItemListWidget->addItem(aitem);

		//QCheckBox *checkBox = new QCheckBox();
		//connect(checkBox, SIGNAL(stateChanged(int)), this, SLOT(SetActiveObjectsItems()));

		CopyHWObjToCCObj(db_tree_->current_element_idx_);
		ShowAllActivedObjects();

		//ShowCurrentObject(db_tree_->current_element_idx_);
	}
#endif
	else
	{
		//这个就是mesh,读取mesh
		HW::HWMesh* add_mesh = new HW::HWMesh();
		
		try
		{
			point_vertices = file_ply.request_properties_from_element("vertex", { "x", "y", "z" });
		}
		catch (const std::exception & e)
		{
			std::cerr << "tinyply exception: " << e.what() << std::endl;
			return false;
		}
		try
		{
			point_normal = file_ply.request_properties_from_element("vertex", { "nx", "ny", "nz" });
		}
		catch (const std::exception & e)
		{
			std::cerr << "tinyply exception: " << e.what() << std::endl;
			return false;
		}
		
		if (has_color)
		{
			//point_color = file_ply.request_properties_from_element("vertex", { "red", "green", "blue", "alpha" });
			try
			{
				point_color = file_ply.request_properties_from_element("vertex", { "red", "green", "blue"});
			}
			catch (const std::exception & e)
			{
				std::cerr << "tinyply exception: " << e.what() << std::endl;
				return false;
			}
		}

		/*try
		{
			point_color = file_ply.request_properties_from_element("vertex", { "red", "green", "blue"});
		}
		catch (const std::exception & e)
		{
			std::cerr << "tinyply exception: " << e.what() << std::endl;
			return false;
		}*/

		//将主体读入到file_ply中
		file_ply.read(fhd);

		//read ply file to HWPointCloud
		if (point_vertices) std::cout << "\tRead " << point_vertices->count << " total vertices " << std::endl;
		if (point_normal) std::cout << "\tRead " << point_normal->count << " total vertex normals " << std::endl;
		if (face_info) std::cout << "\tRead " << face_info->count << " total faces " << std::endl;
		//system("pause");

		std::cout << point_vertices->count << std::endl;
		std::cout << point_vertices->buffer.size_bytes() << std::endl;
		std::cout << point_normal->count << std::endl;
		std::cout << point_normal->buffer.size_bytes() << std::endl;
		//system("pause");
		add_mesh->CopyPointsFromPlyData(point_vertices);
		//std::system("pause");
		add_mesh->CopyNormalsFromPlyData(point_normal);
		add_mesh->CopyFacesFromPlyData(face_info);

		//system("pause");

		if (has_color)
		{
			add_mesh->CopyPointsColorsFromPlyData(point_color);
		}

		//fhd.close();

		HW::HWObject* element_add = add_mesh;
		element_add->SetObjectType(HW::ElementType::kHWMesh);
		db_tree_->current_element_idx_ = file_path;
		std::cout << "load current element idx: " << db_tree_->current_element_idx_ << std::endl;
		db_tree_->AccessTreeElements().insert(std::pair<std::string, HW::HWObject*>(db_tree_->current_element_idx_, element_add));
		QListWidgetItem* aitem = new QListWidgetItem();
		aitem->setText(file_path.c_str());
		aitem->setCheckState(Qt::Checked);
		aitem->setFlags(Qt::ItemIsSelectable | Qt::ItemIsEditable | Qt::ItemIsUserCheckable | Qt::ItemIsEnabled);
		//增加一个Item到view list 中
		this->ui.DbItemListWidget->addItem(aitem);

		CopyHWObjToCCObj(db_tree_->current_element_idx_);
		ShowAllActivedObjects();

		//ShowCurrentObject(db_tree_->current_element_idx_);
	}
#endif
	return true;

#if 0
	//fhd.close();
	//file_ply.~PlyFile();
	//system("pause");

	////如果这个是mesh,face_info->count
	//if (false)
	//{
	//	//分配控件
	//	HW::HWObject* element_add = new HW::HWMesh();
	//	element_add->SetObjectType(HW::ElementType::kHWMesh);
	//	element_add->ReadPly(file_path);
	//	//最后一个元素插入
	//	//db_tree_->current_element_idx_ = HW::HWParams::getInstance().unique_obj_label_;
	//	db_tree_->current_element_idx_ = file_path;
	//	std::cout << "load current element idx: " << db_tree_->current_element_idx_ << std::endl;
	//	db_tree_->AccessTreeElements().insert(std::pair<std::string, HW::HWObject*>(db_tree_->current_element_idx_, element_add));
	//	QListWidgetItem* aitem = new QListWidgetItem();
	//	aitem->setText(file_path.c_str());
	//	aitem->setCheckState(Qt::Checked);
	//	aitem->setFlags(Qt::ItemIsSelectable | Qt::ItemIsEditable | Qt::ItemIsUserCheckable | Qt::ItemIsEnabled);
	//	//增加一个Item到view list 中
	//	this->ui.DbItemListWidget->addItem(aitem);
	//	ShowCurrentObject(db_tree_->current_element_idx_);
	//}
	//else
	//{
	//	//分配控件
	//	HW::HWObject* element_add = new HW::HWPointCloud();
	//	element_add->SetObjectType(HW::ElementType::kHWPointCloud);
	//	element_add->ReadPly(file_path);
	//	//生成object唯一标志
	//	//HW::HWParams::getInstance().unique_obj_label_ = HW::HWParams::getInstance().unique_obj_label_ + 1;
	//	//生成Map结构
	//	//最后一个元素插入
	//	//db_tree_->current_element_idx_ = HW::HWParams::getInstance().unique_obj_label_;
	//	db_tree_->current_element_idx_ = file_path;
	//	std::cout << "load current element idx: " << db_tree_->current_element_idx_ << std::endl;
	//	db_tree_->AccessTreeElements().insert(std::pair<std::string, HW::HWObject*>(db_tree_->current_element_idx_, element_add));
	//	QListWidgetItem* aitem = new QListWidgetItem();
	//	aitem->setText(file_path.c_str());
	//	aitem->setCheckState(Qt::Checked);
	//	aitem->setFlags(Qt::ItemIsSelectable | Qt::ItemIsEditable | Qt::ItemIsUserCheckable | Qt::ItemIsEnabled);
	//	//增加一个Item到view list 中
	//	this->ui.DbItemListWidget->addItem(aitem);
	//	ShowCurrentObject(db_tree_->current_element_idx_);
	//}
#endif
}

bool MainWindow::ReadE57File(const std::string& filePath)
{
    HW::HWPointCloud* added_point_cloud = new HW::HWPointCloud();

    e57::Reader eReader(filePath);

    //Read and access all the e57::Data3D header information from the first scan.
    int  scanIndex = 0;	//picking the first scan

    e57::Data3D	scanHeader;		//read scan's header information
    eReader.ReadData3D(scanIndex, scanHeader);

    //_bstr_t --> const char*
    const char* scanGuid = scanHeader.guid.c_str();		//get guid

     //Get the size information about the scan.
    int64_t nColumn = 0;	        //Number of Columns in a structure scan (from "indexBounds" if structure data)
    int64_t nRow = 0;           //Number of Rows in a structure scan	
    int64_t nPointsSize = 0;    	//Number of points 
    int64_t nGroupsSize = 0;	    //Number of groups (from "groupingByLine" if present)
    int64_t nCountsSize = 0;	    //Number of points per group
    int64_t 	nCounts = 0;        //maximum point count per group    //@zk
    bool    bColumnIndex = false;    //indicates that the idElementName is "columnIndex"
    eReader.GetData3DSizes(scanIndex, nRow, nColumn, nPointsSize, nGroupsSize, nCounts, bColumnIndex);

    int64_t nSize = (nRow > 0) ? nRow : 1024;   //Pick a size for buffers

    //Setup the buffers and setup the e57::CompressedVectorReader point data object.
    double *xData = new double[nSize];
    double *yData = new double[nSize];
    double *zData = new double[nSize];

    int8_t cartesianInvalidState = -1;
    double *	intensity = NULL;
    int8_t 	isIntensityInvalid = -1;

    uint16_t *rData = new uint16_t[nSize];
    uint16_t *gData = new uint16_t[nSize];
    uint16_t *bData = new uint16_t[nSize];
    int8_t isColorInvalid = -1;

    e57::CompressedVectorReader dataReader = eReader.SetUpData3DPointsData(
        scanIndex, 	//!< scan data index 
        nSize, 		//!< size of each of the buffers given
        xData, 		//!< pointer to a buffer with the x data
        yData, 		//!< pointer to a buffer with the y data
        zData,
        &cartesianInvalidState,
        intensity,
        &isIntensityInvalid,
        rData,
        gData,
        bData,
        &isColorInvalid
    );		//!< pointer to a buffer with the z data 
    //Read the each column of data into the buffers.

    unsigned long size = 0;
    while ((size = dataReader.read()) > 0)	//Each call to dataReader.read() will retrieve the next column of data.
    {
        for (unsigned long i = 0; i < size; i++)		//x,y,z Data buffers have the next column of data.
        {
            float3 point;
            point.x = xData[i];
            point.y = yData[i];
            point.z = zData[i];
            added_point_cloud->AddPoint(point);
            uchar3 color;
            color.x = static_cast<unsigned char>(rData[i]);
            color.y = static_cast<unsigned char>(gData[i]);
            color.z = static_cast<unsigned char>(bData[i]);
            added_point_cloud->AddColor(color);
        }
    }
    //Close the reader and clean up the buffer.

    dataReader.close();
    delete xData;
    delete yData;
    delete zData;

    HW::HWObject* element_add = added_point_cloud;
    element_add->SetObjectType(HW::ElementType::kHWPointCloud);
    db_tree_->current_element_idx_ = filePath;
    std::cout << "load current element idx: " << db_tree_->current_element_idx_ << std::endl;
    element_add->SetObjectName(db_tree_->current_element_idx_);
    db_tree_->AccessTreeElements().insert(std::pair<std::string, HW::HWObject*>(db_tree_->current_element_idx_, element_add));

    //插入对象
    CopyHWObjToCCObj(db_tree_->current_element_idx_);

    //ShowAllActivedObjects();
	return true;
}
bool MainWindow::ReadObj(const std::string & file_name)
{
	HW::HWPolygon* polygon = new HW::HWPolygon();
	std::ifstream fhd(file_name);
	if (fhd.fail()) throw std::runtime_error("failed to open " + file_name);
	std::vector<float3> vertices;
	std::string str;
	bool has_vt=false;
	int count = 0;
	while (getline(fhd,str)) {
		if (str.length() == 0)
			continue;
		std::stringstream sstream(str);
		std::string obj_flag;
		sstream >> obj_flag;
		if (obj_flag == "v") {
			float3 vert;
			sstream >> vert.x >> vert.y >> vert.z;
			//vert.x -= 14094;
			//vert.y -= 3310;
			vertices.emplace_back(vert);
		}
		if (obj_flag == "vt")
			has_vt = true;
		if (obj_flag == "f") {
			int idx;
			HW::HWPlane* plane = new HW::HWPlane();
			if (has_vt) {
				std::string idxs;
				while (sstream >> idxs) {
					sscanf_s(idxs.c_str(), "%d/%d", &idx, &idx);
					plane->AddPolygonPnt(vertices[idx - 1]);
				}
			}
			else{
				while (sstream >> idx) {
					plane->AddPolygonPnt(vertices[idx - 1]);
				}
			}
			//cout << "idx: " << idx << "\n";
			if (plane->GetOuterPolygon().size() > 2) {
#ifdef LIULINGFEI
				std::cout << "Polygon: " << count << std::endl;
#endif
				count++;
				plane->initialAfterLoad();
				polygon->addPlane(plane);
			}
		}
	}

#if 0
	std::string file_path;
	file_path= file_name.substr(0, file_name.find_last_of(".")) + ".ply";
	std::ifstream file(file_path, std::ios::binary);
	if (file.fail()) throw std::runtime_error("failed to open " + file_path);
	//std::cout << "test" << std::endl;
	//system("pause");
	tinyply::PlyFile file_ply;
	file_ply.parse_header(file);

	bool has_vertex_flag = false;
	bool has_normal_flag = false;
	bool has_color_flag = false;
	bool has_alpha_flag = false;
	bool has_faces_flag = false;

	std::cout << "........................................................................\n";
	for (auto c : file_ply.get_comments()) std::cout << "Comment: " << c << std::endl;

	for (auto e : file_ply.get_elements())
	{
		std::cout << "element - " << e.name << " (" << e.size << ")" << std::endl;
		if (e.name == "face" && e.size > 0)
			has_faces_flag = true;

		for (auto p : e.properties)
		{
			if (p.name == "x")
				has_vertex_flag = true;
			if (p.name == "nx")
				has_normal_flag = true;
			if (p.name == "red")
				has_color_flag = true;
			if (p.name == "alpha")
				has_alpha_flag = true;

			std::cout << "\tproperty - " << p.name << " (" << tinyply::PropertyTable[p.propertyType].str << ")" << std::endl;
		}
	}
	std::cout << "........................................................................\n";

	//从文件头判断ply里面得元素
	std::shared_ptr<tinyply::PlyData> face_info, point_vertices, point_normal, point_color;

	//通过分析头文件，可以分开读取ply文件
	if (!has_faces_flag)
	{
		//读取点云
		HW::HWPointCloud* added_point_cloud = new HW::HWPointCloud();
		if (has_vertex_flag)
		{
			try
			{
				point_vertices = file_ply.request_properties_from_element("vertex", { "x", "y", "z" });
			}
			catch (const std::exception & e)
			{
				std::cerr << "tinyply exception: " << e.what() << std::endl;
				return false;
			}
		}
		else
		{
			std::cout << "read ply vertex failed." << std::endl;
			return false;
		}

		if (has_normal_flag)
		{
			try
			{
				point_normal = file_ply.request_properties_from_element("vertex", { "nx", "ny", "nz" });
			}
			catch (const std::exception & e)
			{
				std::cerr << "tinyply exception: " << e.what() << std::endl;
				return false;
			}
		}

		if (has_color_flag)
		{
			if (!has_alpha_flag)
			{
				try
				{
					point_color = file_ply.request_properties_from_element("vertex", { "red", "green", "blue" });
				}
				catch (const std::exception & e)
				{
					std::cerr << "tinyply exception: " << e.what() << std::endl;
					return false;
				}
			}
			else
			{
				try
				{
					point_color = file_ply.request_properties_from_element("vertex", { "red", "green", "blue", "alpha" });
				}
				catch (const std::exception & e)
				{
					std::cerr << "tinyply exception: " << e.what() << std::endl;
					return false;
				}
			}
		}

		//将主体读入到file_ply中
		file_ply.read(file);

		if (point_vertices) std::cout << "\tRead " << point_vertices->count << " total vertices " << std::endl;
		if (point_normal) std::cout << "\tRead " << point_normal->count << " total vertex normals " << std::endl;
		//std::system("pause");
		if (point_color) std::cout << "\tRead " << point_color->count << " total vertex color " << std::endl;

		//把ply读取的vertex拷贝到HWPointCloud
		added_point_cloud->CopyPointsFromPlyData(point_vertices);
		if (point_normal) added_point_cloud->CopyNormalsFromPlyData(point_normal);
		//std::cout << "asdfsadfs" << std::endl;
		//std::system("pause");

		if (has_alpha_flag)
		{
			std::vector<uchar4> temp_color_vec;
			const size_t num_color_bytes = point_color->buffer.size_bytes();
			temp_color_vec.resize(point_color->count);
			std::memcpy(temp_color_vec.data(), point_color->buffer.get(), num_color_bytes);

			for (int i = 0; i < temp_color_vec.size(); ++i)
			{
				uchar3 one_point_color = make_uchar3(temp_color_vec[i].x, temp_color_vec[i].y, temp_color_vec[i].z);
				//system("pause");
				added_point_cloud->AddColor(one_point_color);
			}
		}
		else if (has_color_flag)
		{
			added_point_cloud->CopyColorsFromPlyData(point_color);
		}
		polygon->addPointCloud(added_point_cloud);
	}
#endif

	HW::HWObject* element_add = polygon;
	element_add->SetObjectType(HW::ElementType::kHWPolygon);
	db_tree_->current_element_idx_ = file_name;
	std::cout << "load current element idx: " << db_tree_->current_element_idx_ << std::endl;
	element_add->SetObjectName(db_tree_->current_element_idx_);
	db_tree_->AccessTreeElements().insert(std::pair<std::string, HW::HWObject*>(db_tree_->current_element_idx_, element_add));

	//插入对象
	CopyHWObjToCCObj(db_tree_->current_element_idx_);
	ShowAllActivedObjects();
	return true;
}

bool MainWindow::ReadProjFile(const std::string& file_name)
{

	//保存工程文件
	proj_name_ = file_name;
	std::string filename_dir = proj_name_.substr(0, proj_name_.find_last_of("/") + 1);
	std::cerr << "filename_dir: " << filename_dir << std::endl;
	std::ifstream fh(file_name);
	nlohmann::json j_file;
	fh >> j_file;
	std::string obj_name;
	std::string ply_name;
	if (j_file.contains("obj"))
	{
		j_file["obj"].get_to(obj_name);
		obj_name = filename_dir + obj_name;
		std::cout << "obj: " << obj_name << std::endl;
	}
	if (j_file.contains("ply"))
	{
		j_file["ply"].get_to(ply_name);
		ply_name = filename_dir + ply_name;
		std::cout << "ply: " << ply_name << std::endl;
	}
	if (ReadObj(obj_name))
	{
		//将obj文件读取到工程文件里面
		proj_sub_names_.emplace_back(obj_name);
	}
	if (ReadPly(ply_name, true))
	{
		proj_sub_names_.emplace_back(ply_name);
	}
	//获取对应的obj文件
	HW::HWObject *polygon_element = db_tree_->AccessTreeElements()[obj_name];
	HW::HWObject *pnts_element = db_tree_->AccessTreeElements()[ply_name];
	if (polygon_element && pnts_element)
	{
		HW::HWPolygon* polygon = static_cast<HW::HWPolygon*>(polygon_element);
		HW::HWPointCloud* cloud_pnts = static_cast<HW::HWPointCloud*>(pnts_element);
		//polygon->addPointCloud(cloud_pnts);
		int plane_num = polygon->getPlaneVec().size();
		std::map<int, std::vector<int> > obj2ply_vec;
		std::cout << "pnts color num " << cloud_pnts->GetPointsColor().size() << std::endl;
		if (j_file.contains("obj2ply"))
		{
			nlohmann::json tmp = j_file.at("/obj2ply"_json_pointer);
			for (auto& el : tmp.items())
			{
				std::vector<int> myidx;
				std::cout << "polygon: " << el.key() << "\n";
				tmp[el.key()].get_to(myidx);
				int mykey = std::stoi(el.key());
				int mycorrectkey = mykey - 1;
				std::cout << "polygon cloud pnts num: " << myidx.size() << std::endl;
				//int polygon_num = polygon->getPlaneVec().size();
				if (mycorrectkey < 0 || mycorrectkey >= plane_num)
					continue;
				std::vector<float3> pnts_pos;
				std::vector<float3> pnts_normal;
				std::vector<float3> pnts_color;
				for (int j = 0; j < myidx.size(); ++j)
				{
					float3 pnt_pos = cloud_pnts->GetVertices()[myidx[j]];
					pnts_pos.emplace_back(pnt_pos);
					if (cloud_pnts->HasNormal())
					{
						float3 pnt_normal = cloud_pnts->GetNormal()[myidx[j]];
						pnts_normal.emplace_back(pnt_normal);
						//polygon->getPlaneVec()[j]->AddPlaneOriginPntColor
					}
					//std::cout << 
					if (cloud_pnts->HasColor())
					{
						uchar3 pnt_color_uchar = cloud_pnts->GetPointsColor()[myidx[j]];
						float3 pnt_color = make_float3(pnt_color_uchar.x / 255.0,
							pnt_color_uchar.y / 255.0, pnt_color_uchar.z / 255.0);
						pnts_color.emplace_back(pnt_color);
					}
					//polygon->getPlaneVec()[j]
				}
				//std::cerr << "pnts_color number: " << pnts_color.size() << std::endl;
				polygon->getPlaneVec()[mycorrectkey]->SetPlanePnt(pnts_pos);
				if (cloud_pnts->HasNormal())
					polygon->getPlaneVec()[mycorrectkey]->SetPlanePntNormal(pnts_normal);
				if (cloud_pnts->HasColor())
				{
					polygon->getPlaneVec()[mycorrectkey]->SetPlanePntColor(pnts_color);
				}
				//obj2ply_vec.insert(std::make_pair(mykey, myidx));
			}
		}
	}
	fh.close();

	return true;
}


//-------------------end input----------------//


//-------------------------opengl----------------------//

ccHObject* MainWindow::GetObjectByStrName(const std::string& obj_name)
{
	//
	ccHObject* obj;
	//ccPointCloud* plane_pc;
	//获取被选中的点云的名字
	for (int j = 0; j < m_ccRoot1->getChildrenNumber(); ++j)
	{
		//
		if (m_ccRoot1->getChild(j)->getName() == QString(obj_name.data()))
		{
			obj = m_ccRoot1->getChild(j)->getFirstChild();
			break;
		}
	}
	return obj;
}

HW::HWObject* MainWindow::GetCurrentObject()
{
	//获取当前读取的点云
	std::string current_name = db_tree_->current_element_idx_;

	if (current_name.empty())
		return NULL;

	HW::HWObject *obj = db_tree_->AccessTreeElements()[current_name];
	return obj;
}

void MainWindow::DenoiseHWPCFromHWPC(HW::HWObject* src, HW::HWObject* tgt,
	float& pdist, std::vector<bool>& tgt_idxs_rm)
{
	if (src != NULL && tgt != NULL)
	{
		if (src->GetObjectType() == HW::ElementType::kHWPointCloud
			&& tgt->GetObjectType() == HW::ElementType::kHWPointCloud)
		{
			HW::HWPointCloud* srcp = static_cast<HW::HWPointCloud*>(src);
			HW::HWPointCloud* tgtp = static_cast<HW::HWPointCloud*>(tgt);
			std::vector<float3> srcpnts = srcp->GetVertices();
			std::vector<float3> tgtpnts = tgtp->GetVertices();

			if (tgt_idxs_rm.size() != tgtpnts.size())
				return;

			PtCloud<float> srccloud, tgtcloud;
			for (int i = 0; i < srcpnts.size(); ++i)
			{
				srccloud.pts.push_back(PtCloud<float>::PtData(srcpnts[i].x, srcpnts[i].y, srcpnts[i].z));
			}
			for (int i = 0; i < tgtpnts.size(); ++i)
			{
				tgtcloud.pts.push_back(PtCloud<float>::PtData(tgtpnts[i].x, tgtpnts[i].y, tgtpnts[i].z));
			}

			std::cout << "building srccloud kd-tree ..." << endl;
			/*float MINVALUE = 1e-7;
			int pointNum = srccloud.pts.size();
			float scale = 0.0, magnitd = 0.0;*/

			// 1. build kd-tree
			typedef KDTreeSingleIndexAdaptor< L2_Simple_Adaptor<float, PtCloud<float> >, PtCloud<float>, 3/*dim*/ > my_kd_tree_t;
			my_kd_tree_t index(3 /*dim*/, srccloud, KDTreeSingleIndexAdaptorParams(10 /* max leaf */));
			index.buildIndex();
			std::cout << "srccloud kd-tree done, begin knn search" << std::endl;
			int k = 2;

			for (int i = 0; i < tgtcloud.pts.size(); ++i)
			{
				//std::cout << i << std::endl;
				float *query_pt = new float[3];
				query_pt[0] = tgtcloud.pts[i].x;  query_pt[1] = tgtcloud.pts[i].y;  query_pt[2] = tgtcloud.pts[i].z;
				float *dis_temp = new float[k];
				size_t *out_indices = new size_t[k];
				nanoflann::KNNResultSet<float> resultSet(k);
				resultSet.init(out_indices, dis_temp);
				index.findNeighbors(resultSet, &query_pt[0], nanoflann::SearchParams(10));
				//out_ks[i] = resultSet.size();
				float sumdist = 0.0;
				//int minidx = -1;
				for (int j = 0; j < k; ++j)
				{
					sumdist += dis_temp[j];
					//dis_temp[j]
				}
				float tmp_average_dist = sumdist / k;

				if (tmp_average_dist > pdist)
				{
					tgt_idxs_rm[i] = true;
				}
			}

			////
			//int srcrows = srcpnts.size();
			//int srccols = 3;
			//int tgtrows = tgtpnts.size();
			//int tgtcols = 3;
			//float speedup;
			//float* srcset = new float(srcrows*srccols);
			//float* tgtset = new float(tgtrows*tgtcols);
			//int nn = 4;	//这是每个点找最近邻的个数
			////FLANNParameters p;
			////FLANNParameters
			//for (int i = 0; i < srcpnts.size(); ++i)
			//{
			//	srcset[srccols*i] = srcpnts[i].x;
			//	srcset[srccols*i + 1] = srcpnts[i].y;
			//	srcset[srccols*i + 2] = srcpnts[i].z;
			//}
			//for (int i = 0; i < tgtpnts.size(); ++i)
			//{
			//	tgtset[tgtcols*i] = tgtpnts[i].x;
			//	tgtset[tgtcols*i + 1] = tgtpnts[i].y;
			//	tgtset[tgtcols*i + 2] = tgtpnts[i].z;
			//}
			//int tcount = tgtrows;
			//int* result = new int(tcount*nn);
			//float* dists = new float(tcount*nn);
			
		}
	}
}

void MainWindow::DenoiseCCPCFromCCPC(ccPointCloud* src, ccPointCloud* tgt,
	float& pdist, std::vector<bool>& tgt_idxs_rm)
{
	if (src != NULL && tgt != NULL)
	{
		PtCloud<float> srccloud, tgtcloud;
		for (int i = 0; i < src->size(); ++i)
		{
			const CCVector3* tmpp = src->getPoint(i);
			srccloud.pts.push_back(PtCloud<float>::PtData(tmpp->x, tmpp->y, tmpp->z));
		}
		for (int i = 0; i < tgt->size(); ++i)
		{
			const CCVector3* tmpp = tgt->getPoint(i);
			tgtcloud.pts.push_back(PtCloud<float>::PtData(tmpp->x, tmpp->y, tmpp->z));
		}

		std::cout << "building srccloud kd-tree ..." << endl;
		/*float MINVALUE = 1e-7;
		int pointNum = srccloud.pts.size();
		float scale = 0.0, magnitd = 0.0;*/

		// 1. build kd-tree
		typedef KDTreeSingleIndexAdaptor< L2_Simple_Adaptor<float, PtCloud<float> >, PtCloud<float>, 3/*dim*/ > my_kd_tree_t;
		my_kd_tree_t index(3 /*dim*/, srccloud, KDTreeSingleIndexAdaptorParams(10 /* max leaf */));
		index.buildIndex();
		std::cout << "srccloud kd-tree done, begin knn search" << std::endl;
		int k = 3;

		for (int i = 0; i < tgtcloud.pts.size(); ++i)
		{
			//std::cout << i << std::endl;
			float *query_pt = new float[3];
			query_pt[0] = tgtcloud.pts[i].x;  query_pt[1] = tgtcloud.pts[i].y;  query_pt[2] = tgtcloud.pts[i].z;
			float *dis_temp = new float[k];
			size_t *out_indices = new size_t[k];
			nanoflann::KNNResultSet<float> resultSet(k);
			resultSet.init(out_indices, dis_temp);
			index.findNeighbors(resultSet, &query_pt[0], nanoflann::SearchParams(10));
			//out_ks[i] = resultSet.size();
			float sumdist = 0.0;
			//int minidx = -1;
			for (int j = 0; j < k; ++j)
			{
				sumdist += dis_temp[j];
				//dis_temp[j]
			}
			float tmp_average_dist = sumdist / k;

			if (tmp_average_dist > pdist)
			{
				tgt_idxs_rm[i] = false;
			}
		}
	}
}


//void MainWindow::CreateOpenglWindow()
//{
//	//camera->SetMatrixFromCalibratedMat(exMat, inMat, config.cameraImageWidth_, config.cameraImageHeight_);
//	//camera->SetMatrixFromCalibratedMat(exMat, config.cameraRgbIntrinsicsParameter_[0], config.cameraRgbIntrinsicsParameter_[1], 
//	//	config.cameraRgbIntrinsicsParameter_[2], config.cameraRgbIntrinsicsParameter_[3], config.cameraImageWidth_, config.cameraImageHeight_);
//	camera_->SetProjectionMatrixToIdentity();
//	camera_->Perspective(40, 1, 0.4, 100);
//	camera_->SetViewMatrixToIdentity();
//	camera_->LookAt(QVector3D(0, 0, 10), QVector3D(0, 0, 0), QVector3D(0, 1, 0));
//	//camera->LookAt(QVector3D(0, 0, -3.3), QVector3D(0, 0, 1), QVector3D(0, 1, 0));
//	float fx, fy, cx, cy;
//	int width=800, height=600;
//
//	////
//	//fx = 320;
//	//fy = 320;
//	//cx = 250;
//	//cy = 250;
//	//width = 400;
//	//height = 400;
//
//
//	opengl_window_ = new OpenGLWindow(camera_);
//	//opengl_window_->resize(config.cameraImageWidth_, config.cameraImageHeight_);
//	opengl_window_->resize(width, height);
//	opengl_window_->setWindowTitle("OpenGLWindow");
//	opengl_window_->setFocusPolicy(Qt::StrongFocus);
//	opengl_window_->show();
//
//	layout_ = new QGridLayout();
//	this->ui.openglWidget->setLayout(layout_);
//	layout_->addWidget(opengl_window_);
//}

//void MainWindow::ShowOpenglObject(const std::string& select_obj)
//{
//	//先显示
//	std::vector<cv::Vec3f> vertex;
//	std::vector<cv::Vec3f> normal;
//	std::vector<cv::Vec3i> color;
//	std::vector<int> faceIndex;
//	//获取点云
//	std::string current_obj_idx = db_tree_->current_element_idx_;
//	HW::HWObject *selected_obj;
//	std::map<std::string, HW::HWObject*>::iterator iter = db_tree_->AccessTreeElements().find(current_obj_idx);
//	if (iter == std::end(db_tree_->AccessTreeElements()))
//	{
//		std::cerr << "wrong select current idx...\n" << std::endl;
//	}
//	if (iter->second->GetObjectType() == HW::ElementType::kHWPointCloud)
//	{
//		HW::HWPointCloud* in_cloud = dynamic_cast<HW::HWPointCloud*>(iter->second);
//		//向opengl传输数据
//		std::vector<float3> tmp_vertex = in_cloud->GetVertices();
//		std::vector<float3> tmp_normal = in_cloud->GetNormal();
//
//		//test
//		printf("vertex size: %d\n", tmp_vertex.size());
//		printf("vertex normal size: %d\n", tmp_normal.size());
//		//end test
//
//		size_t vertex_num = tmp_vertex.size() * sizeof(float3);
//		vertex.resize(tmp_vertex.size());
//		std::memcpy(vertex.data(), tmp_vertex.data(), vertex_num);
//		size_t normal_num = tmp_normal.size() * sizeof(float3);
//		normal.resize(tmp_normal.size());
//		std::memcpy(normal.data(), tmp_normal.data(), normal_num);
//		////test
//		//for (int i = 0; i < tmp_vertex.size(); ++i)
//		//{
//		//	std::cout << "vertex: " << tmp_vertex[i].x <<" " << tmp_vertex[i].y <<" "<< tmp_vertex[i].z << std::endl;
//		//	std::cout << "tmp vertex: " << vertex[i][0] <<" " << vertex[i][1] << " " << vertex[i][2] << std::endl;
//		//}
//		////end test
//		
//		opengl_window_->AddObject("my model", 1, GL_POINTS);
//		//
//		opengl_window_->Update("my model",
//			VertexAttributeDesc(&vertex),
//			VertexAttributeDesc(&normal),
//			VertexAttributeDesc(&GenerateColor(vertex.size(), cv::Vec4b(0, 0, 255, 255))));
//		printf("%d %d\n", vertex.size(), normal.size());
//		opengl_window_->updateGL();
//	}
//	else if (iter->second->GetObjectType() == HW::ElementType::kHWMesh)
//	{
//		//test
//		printf("show mesh...\n");
//		//end test
//
//		HW::HWMesh* in_mesh = dynamic_cast<HW::HWMesh*>(iter->second);
//		//向opengl传输数据
//		std::vector<float3> tmp_vertex = in_mesh->GetVertices();
//		std::vector<float3> tmp_normal = in_mesh->GetNormal();
//		std::vector<int3> tmp_face = in_mesh->GetFaces();
//		std::vector<uchar3> tmp_color = in_mesh->GetPointsColor();
//		//test
//		printf("vertex size: %d\n", tmp_vertex.size());
//		printf("vertex normal size: %d\n", tmp_normal.size());
//		printf("face size: %d\n", tmp_face.size());
//		//system("pause");
//		//end test
//
//		size_t vertex_num = tmp_vertex.size() * sizeof(float3);
//		vertex.resize(tmp_vertex.size());
//		std::memcpy(vertex.data(), tmp_vertex.data(), vertex_num);
//		size_t normal_num = tmp_normal.size() * sizeof(float3);
//		normal.resize(tmp_normal.size());
//		std::memcpy(normal.data(), tmp_normal.data(), normal_num);
//		size_t face_idx_num = tmp_face.size() * sizeof(int3);
//		normal.resize(3 * tmp_face.size());
//		for (int i = 0; i < tmp_face.size(); ++i)
//		{
//			faceIndex.emplace_back(tmp_face[i].x);
//			faceIndex.emplace_back(tmp_face[i].y);
//			faceIndex.emplace_back(tmp_face[i].z);
//		}
//		//std::memcpy(faceIndex.data(), tmp_face.data(), face_idx_num);
//
//		opengl_window_->AddObject("my model", 1, GL_TRIANGLES);
//		//
//		opengl_window_->Update("my model",
//			VertexAttributeDesc(&vertex),
//			VertexAttributeDesc(&normal),
//			VertexAttributeDesc(&GenerateColor(vertex.size(), cv::Vec4b(0, 0, 255, 255))),
//			&faceIndex
//		);
//		opengl_window_->updateGL();
//	}
//}

//std::vector<cv::Vec4b>& MainWindow::GenerateColor(int num, cv::Vec4b color)
//{
//	colors_.resize(num);
//	for (int i = 0; i < num; i++)
//		colors_[i] = color;
//
//	return colors_;
//}

void MainWindow::CopyHWObjToCCObj(const std::string& obj)
{
	//
	QString select_name(obj.data());
	std::cout << "select name: " << select_name.toStdString() << std::endl;

	for (int i = 0; i < m_ccRoot1->getChildrenNumber(); ++i)
	{
		ccHObject* cc_obj = m_ccRoot1->getChild(i);
		if (select_name == cc_obj->getFirstChild()->getName())
		{
			std::cout << "the copy obj existed in ccRoot" << std::endl;
			return;
		}
	}
	//从HWObject中寻找到obj,并把它传输到m_ccRoot1上
	HW::HWObject* hw_obj = db_tree_->AccessTreeElements()[obj];

	std::cout << "Get the Copy hw obj name is: " << hw_obj->GetObjectName() << std::endl;

	if (hw_obj != NULL)
	{
		if (hw_obj->GetObjectType() == HW::ElementType::kHWPointCloud)
		{
			HW::HWPointCloud* hw_cloud = static_cast<HW::HWPointCloud*>(hw_obj);
			CopyHWPCToCCPC(hw_cloud);
			//active_objects_names_.emplace_back(db_tree_->current_element_idx_);
		}
		else if (hw_obj->GetObjectType() == HW::ElementType::kHWMesh)
		{
			HW::HWMesh* hw_mesh= static_cast<HW::HWMesh*>(hw_obj);
			CopyHWMeshToCCMesh(hw_mesh);
			//active_objects_names_.emplace_back(db_tree_->current_element_idx_);
		}
		else if (hw_obj->GetObjectType() == HW::ElementType::kHWPlane)
		{
			HW::HWPlane* hw_plane = static_cast<HW::HWPlane*>(hw_obj);
			CopyHWPlanePCToCCPC(hw_plane);
		}
	}
}

//
void MainWindow::CopyHWPCToCCPC(HW::HWPointCloud* in_cloud)
{
	//printf("copy hw point cloud to ccpointclud\n");
	std::cerr << "copy hw point cloud to ccpointclud" << std::endl;
	//HW::HWPointCloud* in_cloud = dynamic_cast<HW::HWPointCloud*>(hw_obj);
	QString select_name(in_cloud->GetObjectName().data());
	//向ccPointCloud拷贝数据
	std::vector<float3> tmp_vertices = in_cloud->GetVertices();
	std::vector<float3> tmp_normals = in_cloud->GetNormal();
	std::vector<uchar3> tmp_colors = in_cloud->GetPointsColor();

	//test
	printf("vertex size: %d\n", tmp_vertices.size());
	printf("vertex normal size: %d\n", tmp_normals.size());
	printf("vertex color size: %d\n", tmp_colors.size());
	//system("pause");
	//end test

	CCVector3f min_point(1e6, 1e6, 1e6);
	CCVector3f max_point(-1e6, -1e6, -1e6);

	ccPointCloud* cloud = new ccPointCloud(select_name);

	//第一次显示的话，相机位置会变化
	if (first_show)
	{
		//顶点传输
		for (int i = 0; i < tmp_vertices.size(); ++i)
		{
			cloud->addPoint(CCVector3(tmp_vertices[i].x, tmp_vertices[i].y, tmp_vertices[i].z));
		}
		cloud->getBoundingBox(min_point, max_point);
		ccBBox cloud_box(min_point, max_point);

		std::cout << "cloud box min point: " << min_point.x << " " << min_point.y << " " << min_point.z
			<< std::endl << max_point.x << " " << max_point.y << " " << max_point.z << std::endl;

		m_root_gl_->updateConstellationCenterAndZoom(&cloud_box);

		first_show = false;
	}
	else
	{
		//顶点传输
		for (int i = 0; i < tmp_vertices.size(); ++i)
		{
			cloud->addPoint(CCVector3(tmp_vertices[i].x, tmp_vertices[i].y, tmp_vertices[i].z));
		}
	}

	if (!tmp_normals.empty())
	{
		cloud->reserveTheNormsTable();
		for (int i = 0; i < tmp_normals.size(); ++i)
		{
			//system("pause");
			cloud->addNorm(CCVector3(tmp_normals[i].x, tmp_normals[i].y, tmp_normals[i].z));
			//std::cout << "norm: " << tmp_normals[i].x << " " << tmp_normals[i].y << " " << tmp_normals[i].z << std::endl;
		}
	}

	//reserveTheNormsTable()
	if (!tmp_colors.empty())
	{
		cloud->reserveTheRGBTable();
		for (int i = 0; i < tmp_colors.size(); ++i)
		{
			//system("pause");
			//printf("%d,%d,%d", tmp_colors[i].x, tmp_colors[i].y, tmp_colors[i].z);
			cloud->addRGBColor(ccColor::Rgb(tmp_colors[i].x, tmp_colors[i].y, tmp_colors[i].z));
			//cloud->addRGBColor(ccColor::Rgb(255, 255, 0));
			//std::cout << "norm: " << tmp_normals[i].x << " " << tmp_normals[i].y << " " << tmp_normals[i].z << std::endl;
		}
	}
	else
	{
		cloud->setRGBColor(255, 255, 255);
		cloud->showNormals(true);
	}
	cloud->showColors(true);
	cloud->showSF(false); //just in case
	cloud->prepareDisplayForRefresh();

	ccHObject* show_obj = new ccHObject(select_name);
	//设置context
	show_obj->addChild(cloud);
	//show_obj->getClassID

	////look for object's parent
	//ccHObject* parentObject = show_obj->getParent();
	//if (parentObject)
	//{
	//	parentObject = m_ccRoot1;
	//}
	//else
	//{
	//	m_ccRoot1->addChild(show_obj);
	//}

	m_ccRoot1->addChild(show_obj);

	//AddCCObjectToWeidgetList(show_obj);

	//std::cout << "loaded cloud vertex number is: " << cloud->size() << std::endl;
	//active_objects_names_.emplace_back(select_name.toStdString());

#if 0
	//
	QString select_name(obj.data());
	std::cout << "select name: " << select_name.toStdString() << std::endl;

	for (int i = 0; i < m_ccRoot1->getChildrenNumber(); ++i)
	{
		ccHObject* cc_obj = m_ccRoot1->getChild(i);
		if (select_name == cc_obj->getName())
		{
			std::cout << "the copy obj existed in ccRoot" << std::endl;
			return;
		}
	}
	//从HWObject中寻找到obj,并把它传输到m_ccRoot1上
	HW::HWObject* hw_obj = db_tree_->AccessTreeElements()[obj];
	if (hw_obj != NULL)
	{
		if (hw_obj->GetObjectType() == HW::ElementType::kHWPointCloud)
		{
			printf("copy hw point cloud to ccpointclud\n");

			HW::HWPointCloud* in_cloud = dynamic_cast<HW::HWPointCloud*>(hw_obj);
			//向opengl传输数据
			std::vector<float3> tmp_vertices = in_cloud->GetVertices();
			std::vector<float3> tmp_normals = in_cloud->GetNormal();
			std::vector<uchar3> tmp_colors = in_cloud->GetPointsColor();

			//test
			printf("vertex size: %d\n", tmp_vertices.size());
			printf("vertex normal size: %d\n", tmp_normals.size());
			printf("vertex color size: %d\n", tmp_colors.size());
			//system("pause");
			//end test

			CCVector3f min_point(1e6, 1e6, 1e6);
			CCVector3f max_point(-1e6, -1e6, -1e6);

			ccPointCloud* cloud = new ccPointCloud(select_name);

			//第一次显示的话，相机位置会变化
			if (first_show)
			{
				//顶点传输
				for (int i = 0; i < tmp_vertices.size(); ++i)
				{
					cloud->addPoint(CCVector3(tmp_vertices[i].x, tmp_vertices[i].y, tmp_vertices[i].z));
				}
				cloud->getBoundingBox(min_point, max_point);
				ccBBox cloud_box(min_point, max_point);
				m_root_gl_->updateConstellationCenterAndZoom(&cloud_box);

				first_show = false;
			}
			else
			{
				//顶点传输
				for (int i = 0; i < tmp_vertices.size(); ++i)
				{
					cloud->addPoint(CCVector3(tmp_vertices[i].x, tmp_vertices[i].y, tmp_vertices[i].z));
				}
			}

			if (!tmp_normals.empty())
			{
				cloud->reserveTheNormsTable();
				for (int i = 0; i < tmp_normals.size(); ++i)
				{
					//system("pause");
					cloud->addNorm(CCVector3(tmp_normals[i].x, tmp_normals[i].y, tmp_normals[i].z));
					//std::cout << "norm: " << tmp_normals[i].x << " " << tmp_normals[i].y << " " << tmp_normals[i].z << std::endl;
				}
			}

			//reserveTheNormsTable()
			if (!tmp_colors.empty())
			{
				cloud->reserveTheRGBTable();
				for (int i = 0; i < tmp_colors.size(); ++i)
				{
					//system("pause");
					//printf("%d,%d,%d", tmp_colors[i].x, tmp_colors[i].y, tmp_colors[i].z);
					cloud->addRGBColor(ccColor::Rgb(tmp_colors[i].x, tmp_colors[i].y, tmp_colors[i].z));
					//cloud->addRGBColor(ccColor::Rgb(255, 255, 0));
					//std::cout << "norm: " << tmp_normals[i].x << " " << tmp_normals[i].y << " " << tmp_normals[i].z << std::endl;
				}
			}
			else 
			{
				cloud->setRGBColor(255, 255, 255);
				cloud->showNormals(true);
			}
			cloud->showColors(true);
			cloud->showSF(false); //just in case
			cloud->prepareDisplayForRefresh();

			ccHObject* show_obj = new ccHObject(select_name);
			//设置context
			show_obj->addChild(cloud);
			//show_obj->getClassID

			////look for object's parent
			//ccHObject* parentObject = show_obj->getParent();
			//if (parentObject)
			//{
			//	parentObject = m_ccRoot1;
			//}
			//else
			//{
			//	m_ccRoot1->addChild(show_obj);
			//}

			m_ccRoot1->addChild(show_obj);

			active_objects_names_.emplace_back(obj);
		}
		else if (hw_obj->GetObjectType() == HW::ElementType::kHWMesh)
		{
			//test
			printf("copy mesh...\n");
			//end test

			HW::HWMesh* in_mesh = dynamic_cast<HW::HWMesh*>(hw_obj);
			//向opengl传输数据
			std::vector<float3> tmp_vertices = in_mesh->GetVertices();
			std::vector<float3> tmp_normals = in_mesh->GetNormal();
			std::vector<int3> tmp_faces = in_mesh->GetFaces();
			std::vector<uchar3> tmp_colors = in_mesh->GetPointsColor();
			//test
			printf("vertex size: %d\n", tmp_vertices.size());
			printf("vertex normal size: %d\n", tmp_normals.size());
			printf("face size: %d\n", tmp_faces.size());

			CCVector3f min_point(1e6, 1e6, 1e6);
			CCVector3f max_point(-1e6, -1e6, -1e6);

			ccPointCloud* cloud = new ccPointCloud(select_name);

			//第一次显示的话，相机位置会变化
			if (first_show)
			{
				//顶点传输
				for (int i = 0; i < tmp_vertices.size(); ++i)
				{
					cloud->addPoint(CCVector3(tmp_vertices[i].x, tmp_vertices[i].y, tmp_vertices[i].z));
				}
				//test
				//CCVector3f center_point = cloud->computeGravityCenter();
				//cloud->translateGL(-center_point);
				//end test
				//compute ccbox
				cloud->getBoundingBox(min_point, max_point);
				ccBBox cloud_box(min_point, max_point);
				m_root_gl_->updateConstellationCenterAndZoom(&cloud_box);

				first_show = false;
			}
			else
			{
				//顶点传输
				for (int i = 0; i < tmp_vertices.size(); ++i)
				{
					cloud->addPoint(CCVector3(tmp_vertices[i].x, tmp_vertices[i].y, tmp_vertices[i].z));
				}
			}

			//ccPointCloud* cloud = new ccPointCloud(select_name);
			////顶点传输
			//for (int i = 0; i < tmp_vertices.size(); ++i)
			//{
			//	cloud->addPoint(CCVector3(tmp_vertices[i].x, tmp_vertices[i].y, tmp_vertices[i].z));
			//}

			if (!tmp_normals.empty())
			{
				cloud->reserveTheNormsTable();
				for (int i = 0; i < tmp_normals.size(); ++i)
				{
					cloud->addNorm(CCVector3(tmp_normals[i].x, tmp_normals[i].y, tmp_normals[i].z));
				}
			}

			//reserveTheNormsTable()
			if (!tmp_colors.empty())
			{
				cloud->reserveTheRGBTable();
				for (int i = 0; i < tmp_colors.size(); ++i)
				{
					//system("pause");
					//printf("%d,%d,%d", tmp_colors[i].x, tmp_colors[i].y, tmp_colors[i].z);
					cloud->addRGBColor(ccColor::Rgb(tmp_colors[i].x, tmp_colors[i].y, tmp_colors[i].z));

					//cloud->addRGBColor(ccColor::Rgb(255, 255, 0));
					//std::cout << "norm: " << tmp_normals[i].x << " " << tmp_normals[i].y << " " << tmp_normals[i].z << std::endl;
				}
				cloud->showNormals(false);
				m_root_gl_->setSunLight(false);
			}
			else
			{
				cloud->setRGBColor(255, 255, 255);
				cloud->showNormals(true);
				m_root_gl_->setSunLight(true);
			}
			//cloud->showColors(true);
			//cloud->showSF(false); //just in case
			//cloud->prepareDisplayForRefresh();

			//cloud->setRGBColor(255, 255, 255);
			//cloud->showColors(true);
			ccMesh* tmp_mesh = new ccMesh(cloud);
			if (!tmp_colors.empty())
			{
				tmp_mesh->showNormals(false);
			}
			else
			{
				tmp_mesh->showNormals(true);
			}
			tmp_mesh->showColors(true);

			ccGenericPointCloud* vertices = tmp_mesh->getAssociatedCloud();
			cloud = static_cast<ccPointCloud*>(vertices);
			//cloud->setRGBColor(255, 255, 255);
			if (!cloud->hasColors())
			{
				cloud->setRGBColor(255, 255, 255);
			}
			cloud->showColors(true);
			cloud->prepareDisplayForRefresh();

			for (int i = 0; i < tmp_faces.size(); ++i)
			{
				tmp_mesh->addTriangle(tmp_faces[i].x, tmp_faces[i].y, tmp_faces[i].z);
			}
			//tmp_mesh->add
			//tmp_mesh->showWired(true);
			//tmp_mesh->isShownAsWire();
			ccHObject* show_obj = new ccHObject(select_name);
			//设置context
			//ccGenericPointCloud* tmp_points = tmp_mesh->getAssociatedCloud();
			//tmp_points->getPoint()

			show_obj->addChild(tmp_mesh);
			//system("pause");
			////test for mesh trans
			//std::ofstream test_obj("D:/vc_project_xht/result_test.obj");
			//for (int i = 0; i < tmp_points->size(); ++i)
			//{
			//	test_obj << "v " << tmp_points->getPoint(i)->x << " " << tmp_points->getPoint(i)->y << " " << tmp_points->getPoint(i)->z << std::endl;
			//	//CCVector3f tmp(tmp_points->getPoint(i)->x, tmp_points->getPoint(i)->y, tmp_points->getPoint(i)->z);
			//}
			//for (int i = 0; i < tmp_points->size(); ++i)
			//{
			//	test_obj << "vn " << tmp_points->getPointNormal(i).x << " " << tmp_points->getPointNormal(i).y << " " << tmp_points->getPointNormal(i).z << std::endl;
			//}
			//for (int i = 0; i < tmp_mesh->size(); ++i)
			//{
			//	CCLib::VerticesIndexes tmp_face(tmp_mesh->getTriangleVertIndexes(i)->i[0],
			//		tmp_mesh->getTriangleVertIndexes(i)->i[1], tmp_mesh->getTriangleVertIndexes(i)->i[2]);
			//	test_obj << "f " << tmp_face.i[0] + 1 << "/0/" << tmp_face.i[0] + 1 << " "
			//		<< tmp_face.i[1] + 1 << "/0/" << tmp_face.i[1] + 1 << " "
			//		<< tmp_face.i[2] + 1 << "/0/" << tmp_face.i[2] + 1 << std::endl;
			//}
			//test_obj.close();
			////end test

			////look for object's parent
			//ccHObject* parentObject = show_obj->getParent();
			//if (parentObject)
			//{
			//	parentObject = m_ccRoot1;
			//}
			//else
			//{
			//	m_ccRoot1->addChild(show_obj);
			//}

			m_ccRoot1->addChild(show_obj);

#if 1
			if (this->ui.actionShowWired->isChecked())
			{
				ccMesh* selected_mesh = dynamic_cast<ccMesh*>(m_ccRoot1->getFirstChild()->getFirstChild());

				selected_mesh->showWired(true);
				//m_root_gl_->setWire(true);
			}
			else
			{
				ccMesh* selected_mesh = dynamic_cast<ccMesh*>(m_ccRoot1->getFirstChild()->getFirstChild());
				selected_mesh->showWired(false);
				//m_root_gl_->setSunLight(true);
			}
#endif
			active_objects_names_.emplace_back(obj);
		}
	}
#endif
}

void MainWindow::CopyHWPlanePCToCCPC(HW::HWPlane* in_cloud)
{
	printf("copy hw plane point cloud to ccpointclud\n");

	//HW::HWPointCloud* in_cloud = dynamic_cast<HW::HWPointCloud*>(hw_obj);
	QString select_name(in_cloud->GetObjectName().data());
	//向ccPointCloud拷贝数据
	std::vector<float3> tmp_vertices = in_cloud->GetPlanePnts();
	std::vector<float3> tmp_normals = in_cloud->GetPlanePntsNormal();
	std::vector<uchar3> tmp_colors = in_cloud->GetPlaneColors();
	
	//test
	printf("vertex size: %d\n", tmp_vertices.size());
	printf("vertex normal size: %d\n", tmp_normals.size());
	printf("vertex color size: %d\n", tmp_colors.size());
	//system("pause");
	//end test
	ccPointCloud* cloud = new ccPointCloud(select_name);

	//顶点传输
	for (int i = 0; i < tmp_vertices.size(); ++i)
	{
		cloud->addPoint(CCVector3(tmp_vertices[i].x, tmp_vertices[i].y, tmp_vertices[i].z));
	}
	if (!tmp_normals.empty())
	{
		cloud->reserveTheNormsTable();
		for (int i = 0; i < tmp_normals.size(); ++i)
		{
			cloud->addNorm(CCVector3(tmp_normals[i].x, tmp_normals[i].y, tmp_normals[i].z));
		}
	}
	if (!tmp_colors.empty())
	{
		cloud->reserveTheRGBTable();
		for (int i = 0; i < tmp_colors.size(); ++i)
		{
			cloud->addRGBColor(ccColor::Rgb(tmp_colors[i].x, tmp_colors[i].y, tmp_colors[i].z));
		}
	}
	else
	{
		cloud->setRGBColor(255, 255, 255);
		cloud->showNormals(true);
	}

	cloud->showColors(true);
	cloud->showSF(false); //just in case
	cloud->prepareDisplayForRefresh();

	ccHObject* show_obj = new ccHObject(select_name);
	//设置context
	show_obj->addChild(cloud);
	m_ccRoot1->addChild(show_obj);
}

void MainWindow::CopyHWPolygonToCCPolylines(HW::HWPolygon* in_polygon)
{
	//
	std::cout << "copy the polygon to polylines..." << std::endl;
	QString select_name(in_polygon->GetObjectName().data());
	std::vector<HW::HWPlane*> polygons = in_polygon->getPlaneVec();
	//开始copy这些polygons
	for (int i = 0; i < polygons.size(); ++i)
	{
		//ccPolyline* tmpline;
		//to do next...
	}
}

void MainWindow::CopyHWMeshToCCMesh(HW::HWMesh* in_mesh)
{
	//test
	printf("copy mesh...\n");
	//end test
	//HW::HWMesh* in_mesh = dynamic_cast<HW::HWMesh*>(hw_obj);
	std::string mesh_name = in_mesh->GetObjectName();
	//std::cout << "mesh name is: " << mesh_name << std::endl;
	//system("pause");

	QString mesh_name_qstr(mesh_name.data());

	//向ccMesh传输数据
	std::vector<float3> tmp_vertices = in_mesh->GetVertices();
	std::vector<float3> tmp_normals = in_mesh->GetNormal();
	std::vector<int3> tmp_faces = in_mesh->GetFaces();
	std::vector<uchar3> tmp_colors = in_mesh->GetPointsColor();
	//test
	printf("vertex size: %d\n", tmp_vertices.size());
	printf("vertex normal size: %d\n", tmp_normals.size());
	printf("face size: %d\n", tmp_faces.size());

	CCVector3f min_point(1e6, 1e6, 1e6);
	CCVector3f max_point(-1e6, -1e6, -1e6);

	ccPointCloud* cloud = new ccPointCloud(mesh_name_qstr);

	//第一次显示的话，相机位置会变化
	if (first_show)
	{
		//顶点传输
		for (int i = 0; i < tmp_vertices.size(); ++i)
		{
			cloud->addPoint(CCVector3(tmp_vertices[i].x, tmp_vertices[i].y, tmp_vertices[i].z));
		}
		//test
		//CCVector3f center_point = cloud->computeGravityCenter();
		//cloud->translateGL(-center_point);
		//end test
		//compute ccbox
		cloud->getBoundingBox(min_point, max_point);
		ccBBox cloud_box(min_point, max_point);
		m_root_gl_->updateConstellationCenterAndZoom(&cloud_box);

		first_show = false;
	}
	else
	{
		//顶点传输
		for (int i = 0; i < tmp_vertices.size(); ++i)
		{
			cloud->addPoint(CCVector3(tmp_vertices[i].x, tmp_vertices[i].y, tmp_vertices[i].z));
		}
	}

	//ccPointCloud* cloud = new ccPointCloud(select_name);
	////顶点传输
	//for (int i = 0; i < tmp_vertices.size(); ++i)
	//{
	//	cloud->addPoint(CCVector3(tmp_vertices[i].x, tmp_vertices[i].y, tmp_vertices[i].z));
	//}

	if (!tmp_normals.empty())
	{
		cloud->reserveTheNormsTable();
		for (int i = 0; i < tmp_normals.size(); ++i)
		{
			cloud->addNorm(CCVector3(tmp_normals[i].x, tmp_normals[i].y, tmp_normals[i].z));
		}
	}

	//reserveTheNormsTable()
	if (!tmp_colors.empty())
	{
		cloud->reserveTheRGBTable();
		for (int i = 0; i < tmp_colors.size(); ++i)
		{
			//system("pause");
			//printf("%d,%d,%d", tmp_colors[i].x, tmp_colors[i].y, tmp_colors[i].z);
			cloud->addRGBColor(ccColor::Rgb(tmp_colors[i].x, tmp_colors[i].y, tmp_colors[i].z));
			//cloud->addRGBColor(ccColor::Rgb(255, 255, 0));
			//std::cout << "norm: " << tmp_normals[i].x << " " << tmp_normals[i].y << " " << tmp_normals[i].z << std::endl;
		}
		cloud->showNormals(false);
		m_root_gl_->setSunLight(false);
	}
	else
	{
		cloud->setRGBColor(255, 255, 255);
		cloud->showNormals(true);
		m_root_gl_->setSunLight(true);
	}
	//cloud->showColors(true);
	//cloud->showSF(false); //just in case
	//cloud->prepareDisplayForRefresh();
	//cloud->setRGBColor(255, 255, 255);
	//cloud->showColors(true);
	ccMesh* tmp_mesh = new ccMesh(cloud);
	tmp_mesh->setName(mesh_name_qstr);

	if (!tmp_colors.empty())
	{
		tmp_mesh->showNormals(false);
	}
	else
	{
		tmp_mesh->showNormals(true);
	}

	tmp_mesh->showColors(true);
	ccGenericPointCloud* vertices = tmp_mesh->getAssociatedCloud();
	ccPointCloud* assiciate_cloud = static_cast<ccPointCloud*>(vertices);
	//cloud->setRGBColor(255, 255, 255);
	if (!assiciate_cloud->hasColors())
	{
		assiciate_cloud->setRGBColor(255, 255, 255);
	}
	assiciate_cloud->showColors(true);
	assiciate_cloud->prepareDisplayForRefresh();

	for (int i = 0; i < tmp_faces.size(); ++i)
	{
		tmp_mesh->addTriangle(tmp_faces[i].x, tmp_faces[i].y, tmp_faces[i].z);
	}
	//tmp_mesh->add
	//tmp_mesh->showWired(true);
	//tmp_mesh->isShownAsWire();
	ccHObject* show_obj = new ccHObject(mesh_name_qstr);
	//设置context
	//ccGenericPointCloud* tmp_points = tmp_mesh->getAssociatedCloud();
	//tmp_points->getPoint()

	show_obj->addChild(tmp_mesh);
	//system("pause");
	////test for mesh trans
	//std::ofstream test_obj("D:/vc_project_xht/result_test.obj");
	//for (int i = 0; i < tmp_points->size(); ++i)
	//{
	//	test_obj << "v " << tmp_points->getPoint(i)->x << " " << tmp_points->getPoint(i)->y << " " << tmp_points->getPoint(i)->z << std::endl;
	//	//CCVector3f tmp(tmp_points->getPoint(i)->x, tmp_points->getPoint(i)->y, tmp_points->getPoint(i)->z);
	//}
	//for (int i = 0; i < tmp_points->size(); ++i)
	//{
	//	test_obj << "vn " << tmp_points->getPointNormal(i).x << " " << tmp_points->getPointNormal(i).y << " " << tmp_points->getPointNormal(i).z << std::endl;
	//}
	//for (int i = 0; i < tmp_mesh->size(); ++i)
	//{
	//	CCLib::VerticesIndexes tmp_face(tmp_mesh->getTriangleVertIndexes(i)->i[0],
	//		tmp_mesh->getTriangleVertIndexes(i)->i[1], tmp_mesh->getTriangleVertIndexes(i)->i[2]);
	//	test_obj << "f " << tmp_face.i[0] + 1 << "/0/" << tmp_face.i[0] + 1 << " "
	//		<< tmp_face.i[1] + 1 << "/0/" << tmp_face.i[1] + 1 << " "
	//		<< tmp_face.i[2] + 1 << "/0/" << tmp_face.i[2] + 1 << std::endl;
	//}
	//test_obj.close();
	////end test

	////look for object's parent
	//ccHObject* parentObject = show_obj->getParent();
	//if (parentObject)
	//{
	//	parentObject = m_ccRoot1;
	//}
	//else
	//{
	//	m_ccRoot1->addChild(show_obj);
	//}

	m_ccRoot1->addChild(show_obj);

	//AddCCObjectToWeidgetList(show_obj);

#if 1

	std::string select_file_name = db_tree_->current_element_idx_;
	ccMesh* selected_mesh;

	for (int i = 0; i < m_ccRoot1->getChildrenNumber(); ++i)
	{
		if (m_ccRoot1->getChild(i)->getName() == QString(select_file_name.data()))
		{
			if (m_ccRoot1->getChild(i)->getFirstChild()->getClassID() == CC_TYPES::MESH)
			{
				selected_mesh = dynamic_cast<ccMesh*> (m_ccRoot1->getChild(i)->getFirstChild());
				break;
			}
		}
	}

	if (!selected_mesh)
		return;

	if (this->ui.actionShowWired->isChecked())
	{
		selected_mesh->showWired(true);
		//m_root_gl_->setWire(true);
	}
	else
	{
		//printf("asdfsdf\n");
		//ccMesh* selected_mesh = dynamic_cast<ccMesh*>(m_ccRoot1->getFirstChild()->getFirstChild());
		selected_mesh->showWired(false);
		//m_root_gl_->setSunLight(true);
	}
#endif
}

void MainWindow::CopyCCPCToHWPC(ccPointCloud* pc)
{
	//
	std::string hw_pc_name = pc->getName().toStdString();
	HW::HWPointCloud* add_pc = new HW::HWPointCloud();
	add_pc->SetObjectName(hw_pc_name);
	add_pc->SetObjectType(HW::kHWPointCloud);
	for (int i = 0; i < pc->size(); ++i)
	{
		const CCVector3 *p = pc->getPoint(i);
		add_pc->AddPoint(make_float3(p->x, p->y, p->z));
		if (pc->hasNormals())
		{
			const CCVector3 &n = pc->getPointNormal(i);
			add_pc->AddNormal(make_float3(n.x, n.y, n.z));
		}
		if (pc->hasColors())
		{
			const ccColor::Rgb &c = pc->getPointColor(i);
			add_pc->AddColor(make_uchar3(c.r, c.g, c.b));
		}
	}
	//加入到db_tree中,如果重名，可以改名插入db_tree
	std::pair<std::map<std::string, HW::HWObject*>::iterator, bool> pre_iter;
	pre_iter = db_tree_->AccessTreeElements().insert(std::make_pair(hw_pc_name, add_pc));
	//如果有重名，可以改一下名字插入
	if (!pre_iter.second)
	{
		std::string rename = hw_pc_name.substr(0, hw_pc_name.find_last_of(".")) + "_cloud.ply";
		pre_iter = db_tree_->AccessTreeElements().insert(std::make_pair(rename, add_pc));
		//更改ccPointCloud的名字
		const QString rename_qstr(rename.data());
		pc->setName(rename_qstr);
	}
}

void MainWindow::CopyCCPCToHWPC(ccPointCloud * pc, std::vector<int>& planes_idx, std::vector<int>& planes_iswide)
{
	std::string hw_pc_name = pc->getName().toStdString();
	HW::HWPointCloud* add_pc = new HW::HWPointCloud();
	add_pc->SetPlanesIdx(planes_idx);
	add_pc->SetPlanesIsWide(planes_iswide);
	add_pc->SetObjectName(hw_pc_name);
	add_pc->SetObjectType(HW::kHWPointCloud);
	CCVector3 bb_min, bb_max;
	pc->getBoundingBox(bb_min, bb_max);
	float3 box_min = make_float3(bb_min.x, bb_min.y, bb_min.z);
	float3 box_max = make_float3(bb_max.x, bb_max.y, bb_max.z);
	add_pc->SetBoundingBox(box_min, box_max);
	for (int i = 0; i < pc->size(); ++i)
	{
		const CCVector3 *p = pc->getPoint(i);
		add_pc->AddPoint(make_float3(p->x, p->y, p->z));
		if (pc->hasNormals())
		{
			const CCVector3 &n = pc->getPointNormal(i);
			add_pc->AddNormal(make_float3(n.x, n.y, n.z));
		}
		if (pc->hasColors())
		{
			const ccColor::Rgb &c = pc->getPointColor(i);
			add_pc->AddColor(make_uchar3(c.r, c.g, c.b));
		}
	}
	//加入到db_tree中,如果重名，可以改名插入db_tree
	std::pair<std::map<std::string, HW::HWObject*>::iterator, bool> pre_iter;
	pre_iter = db_tree_->AccessTreeElements().insert(std::make_pair(hw_pc_name, add_pc));
	//如果有重名，可以改一下名字插入
	if (!pre_iter.second)
	{
		std::string rename = hw_pc_name.substr(0, hw_pc_name.find_last_of(".")) + "_cloud.ply";
		pre_iter = db_tree_->AccessTreeElements().insert(std::make_pair(rename, add_pc));
		//更改ccPointCloud的名字
		const QString rename_qstr(rename.data());
		pc->setName(rename_qstr);
	}
}

void MainWindow::CopyCCMeshToHWMesh(ccMesh* cc_mesh)
{
	//to do next
	return;
}

void MainWindow::AddCCPCViewsList(const ccPointCloud* pc)
{
	//在UI的Weidget界面上显示
	std::string pc_name = pc->getName().toStdString();
	AddViewListAItem(pc_name);
	//将ccPointCloud点云的名字放入显示数组中
	active_objects_names_.emplace_back(pc_name);
}

void MainWindow::AddCCObjectToWeidgetList(ccObject* obj)
{
	//在UI的Weidget界面上显示
	std::string obj_name = obj->getName().toStdString();
	AddViewListAItem(obj_name);
}

void MainWindow::ConvertMatrixToELUEAngle(ccGLMatrixd cam_param)
{
	Eigen::Matrix4f test_mat;
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			test_mat(i, j) = (float)cam_param(i, j);
		}
	}

	//test_mat.col(1) *= -1;
	//test_mat.col(2) *= -1;
	//test_mat = test_mat;
	Eigen::Matrix3f R = test_mat.topLeftCorner(3, 3);
	Eigen::Vector3f ea1 = R.eulerAngles(2, 1, 0);
	ea1 = ea1 / M_PI * 180;
	std::cout << ea1.x() << std::endl;
	std::cout << ea1.y() << std::endl;
	std::cout << ea1.z() << std::endl;
	std::cout << test_mat.topRightCorner(3, 1) << std::endl << std::endl;

	/*std::cout << "rot_degree: " << rot_degree_x << " : " << rot_degree_y << " : " << rot_degree_z << std::endl;
	std::cout << "t: " << tx << " : " << ty << " : " << tz << std::endl;
	std::cout << camera_pose << std::endl;*/

}

void MainWindow::ShowAllActivedObjects()
{
#if 0
	m_ccRoot1->setDisplay_recursive(m_root_gl_);
	this->m_root_gl_->redraw();
#endif
	//ccGLCameraParameters temp_param;
	//m_root_gl_->setBaseViewMat();
	//m_root_gl_->getGLCameraParameters(temp_param);
	//m_root_gl_->set
	//ccGui::ParamStruct params;
	//ccGui::Set(params);

	//m_root_gl_->setDisplayParameters()
#if 1
	//system("pause");
	assert(m_root_gl_);
	m_ccRoot1->removeFromDisplay_recursive(m_root_gl_);
	for (int i = 0; i < m_ccRoot1->getChildrenNumber(); ++i)
	{
		bool show_flag = false;
		ccHObject* cc_obj = m_ccRoot1->getChild(i);
		for (int j = 0; j < active_objects_names_.size(); ++j)
		{
			QString obj_name(active_objects_names_[j].data());
			if (obj_name == cc_obj->getFirstChild()->getName())
			{
				//std::cout << "cc obj name is: " << active_objects_names_[j] << std::endl;
				show_flag = true;
			}
		}
		if (show_flag && cc_obj||cc_obj->isA(CC_TYPES::POLY_LINE)) {
			cc_obj->setDisplay_recursive(m_root_gl_);
		}
	}
	this->m_root_gl_->redraw();

#endif
}

void MainWindow::ShowCurrentObject(const std::string& select_obj)
{
	//
	QString select_name(select_obj.data());
	std::cout << "select name: " << select_name.toStdString() << std::endl;

	if (select_name == m_ccRoot1->getName())
	{
		std::cout << "select same name" << std::endl;
		return;
	}
	//释放原来的对象
	//to do......
	if (!m_ccRoot1->getName().isEmpty())
	{
		m_ccRoot1->removeAllChildren();
	}

	//传输数据建立新的对象
	//to do......
	//获取点云
	std::string current_obj_idx = db_tree_->current_element_idx_;
	HW::HWObject *selected_obj;
	std::map<std::string, HW::HWObject*>::iterator iter = db_tree_->AccessTreeElements().find(current_obj_idx);
	if (iter == std::end(db_tree_->AccessTreeElements()))
	{
		std::cerr << "wrong select current idx...\n" << std::endl;
	}
	if (iter->second->GetObjectType() == HW::ElementType::kHWPointCloud)
	{
		HW::HWPointCloud* in_cloud = dynamic_cast<HW::HWPointCloud*>(iter->second);
		//向opengl传输数据
		std::vector<float3> tmp_vertices = in_cloud->GetVertices();
		std::vector<float3> tmp_normals = in_cloud->GetNormal();
		std::vector<uchar3> tmp_colors = in_cloud->GetPointsColor();

		//test
		printf("vertex size: %d\n", tmp_vertices.size());
		printf("vertex normal size: %d\n", tmp_normals.size());
		printf("vertex color size: %d\n", tmp_colors.size());
		//system("pause");
		//end test

		CCVector3f min_point(1e6, 1e6, 1e6);
		CCVector3f max_point(-1e6, -1e6, -1e6);

		ccPointCloud* cloud = new ccPointCloud(select_name);

		//第一次显示的话，相机位置会变化
		if (first_show)
		{
			//顶点传输
			for (int i = 0; i < tmp_vertices.size(); ++i)
			{
				cloud->addPoint(CCVector3(tmp_vertices[i].x, tmp_vertices[i].y, tmp_vertices[i].z));

				/*if (min_point.x > tmp_vertices[i].x)
					min_point.x = tmp_vertices[i].x;
				if (min_point.y > tmp_vertices[i].y)
					min_point.y = tmp_vertices[i].y;
				if (min_point.z > tmp_vertices[i].z)
					min_point.z = tmp_vertices[i].z;
				if (max_point.x < tmp_vertices[i].x)
					max_point.x = tmp_vertices[i].x;
				if (max_point.y < tmp_vertices[i].y)
					max_point.y = tmp_vertices[i].y;
				if (max_point.z < tmp_vertices[i].z)
					max_point.z = tmp_vertices[i].z;*/
			}
			//test
			//CCVector3f center_point = cloud->computeGravityCenter();
			//cloud->translateGL(-center_point);
			//end test
			//compute ccbox
			cloud->getBoundingBox(min_point, max_point);
			ccBBox cloud_box(min_point, max_point);
			//
			std::cout << "cloud box min point: " << min_point.x << " " << min_point.y << " " << min_point.z
				<< std::endl << max_point.x << " " << max_point.y << " " << max_point.z << std::endl;

			m_root_gl_->updateConstellationCenterAndZoom(&cloud_box);

			first_show = false;
		}
		else
		{
			//顶点传输
			for (int i = 0; i < tmp_vertices.size(); ++i)
			{
				cloud->addPoint(CCVector3(tmp_vertices[i].x, tmp_vertices[i].y, tmp_vertices[i].z));
			}
		}

		//reserveTheNormsTable()
		if (!tmp_normals.empty())
		{
			cloud->reserveTheNormsTable();
			for (int i = 0; i < tmp_normals.size(); ++i)
			{
				//system("pause");
				cloud->addNorm(CCVector3(tmp_normals[i].x, tmp_normals[i].y, tmp_normals[i].z));
				//std::cout << "norm: " << tmp_normals[i].x << " " << tmp_normals[i].y << " " << tmp_normals[i].z << std::endl;
			}
		}

		//reserveTheNormsTable()
		if (!tmp_colors.empty())
		{
			cloud->reserveTheRGBTable();
			for (int i = 0; i < tmp_colors.size(); ++i)
			{
				//system("pause");
				//printf("%d,%d,%d", tmp_colors[i].x, tmp_colors[i].y, tmp_colors[i].z);
				cloud->addRGBColor(ccColor::Rgb(tmp_colors[i].x, tmp_colors[i].y, tmp_colors[i].z));

				//cloud->addRGBColor(ccColor::Rgb(255, 255, 0));
				//std::cout << "norm: " << tmp_normals[i].x << " " << tmp_normals[i].y << " " << tmp_normals[i].z << std::endl;
			}
		}
		else {
		cloud->setRGBColor(255, 255, 255);
		cloud->showNormals(true);
		}
		cloud->showColors(true);
		cloud->showSF(false); //just in case
		cloud->prepareDisplayForRefresh();

		ccHObject* show_obj = new ccHObject(select_name);
		//设置context
		show_obj->addChild(cloud);
		//look for object's parent
		ccHObject* parentObject = show_obj->getParent();
		if (parentObject)
		{
			parentObject = m_ccRoot1;
		}
		else
		{
			m_ccRoot1->addChild(show_obj);
		}

	}
	else if (iter->second->GetObjectType() == HW::ElementType::kHWMesh)
	{
		//test
		printf("show mesh...\n");
		//end test
	
		HW::HWMesh* in_mesh = dynamic_cast<HW::HWMesh*>(iter->second);
		//向opengl传输数据
		std::vector<float3> tmp_vertices = in_mesh->GetVertices();
		std::vector<float3> tmp_normals = in_mesh->GetNormal();
		std::vector<int3> tmp_faces = in_mesh->GetFaces();
		std::vector<uchar3> tmp_colors = in_mesh->GetPointsColor();
		//test
		printf("vertex size: %d\n", tmp_vertices.size());
		printf("vertex normal size: %d\n", tmp_normals.size());
		printf("face size: %d\n", tmp_faces.size());
		
		CCVector3f min_point(1e6, 1e6, 1e6);
		CCVector3f max_point(-1e6, -1e6, -1e6);

		ccPointCloud* cloud = new ccPointCloud(select_name);

		//第一次显示的话，相机位置会变化
		if (first_show)
		{
			//顶点传输
			for (int i = 0; i < tmp_vertices.size(); ++i)
			{
				cloud->addPoint(CCVector3(tmp_vertices[i].x, tmp_vertices[i].y, tmp_vertices[i].z));

				/*if (min_point.x > tmp_vertices[i].x)
					min_point.x = tmp_vertices[i].x;
				if (min_point.y > tmp_vertices[i].y)
					min_point.y = tmp_vertices[i].y;
				if (min_point.z > tmp_vertices[i].z)
					min_point.z = tmp_vertices[i].z;
				if (max_point.x < tmp_vertices[i].x)
					max_point.x = tmp_vertices[i].x;
				if (max_point.y < tmp_vertices[i].y)
					max_point.y = tmp_vertices[i].y;
				if (max_point.z < tmp_vertices[i].z)
					max_point.z = tmp_vertices[i].z;*/
			}
			//test
			//CCVector3f center_point = cloud->computeGravityCenter();
			//cloud->translateGL(-center_point);
			//end test
			//compute ccbox
			cloud->getBoundingBox(min_point, max_point);
			ccBBox cloud_box(min_point, max_point);
			m_root_gl_->updateConstellationCenterAndZoom(&cloud_box);

			first_show = false;
		}
		else
		{
			//顶点传输
			for (int i = 0; i < tmp_vertices.size(); ++i)
			{
				cloud->addPoint(CCVector3(tmp_vertices[i].x, tmp_vertices[i].y, tmp_vertices[i].z));
			}
		}

		//ccPointCloud* cloud = new ccPointCloud(select_name);
		////顶点传输
		//for (int i = 0; i < tmp_vertices.size(); ++i)
		//{
		//	cloud->addPoint(CCVector3(tmp_vertices[i].x, tmp_vertices[i].y, tmp_vertices[i].z));
		//}

		if (!tmp_normals.empty())
		{
			cloud->reserveTheNormsTable();
			for (int i = 0; i < tmp_normals.size(); ++i)
			{
				cloud->addNorm(CCVector3(tmp_normals[i].x, tmp_normals[i].y, tmp_normals[i].z));
			}
		}

		//reserveTheNormsTable()
		if (!tmp_colors.empty())
		{
			cloud->reserveTheRGBTable();
			for (int i = 0; i < tmp_colors.size(); ++i)
			{
				//system("pause");
				//printf("%d,%d,%d", tmp_colors[i].x, tmp_colors[i].y, tmp_colors[i].z);
				cloud->addRGBColor(ccColor::Rgb(tmp_colors[i].x, tmp_colors[i].y, tmp_colors[i].z));

				//cloud->addRGBColor(ccColor::Rgb(255, 255, 0));
				//std::cout << "norm: " << tmp_normals[i].x << " " << tmp_normals[i].y << " " << tmp_normals[i].z << std::endl;
			}
			cloud->showNormals(false);
			m_root_gl_->setSunLight(false);
		}
		else 
		{
			cloud->setRGBColor(255, 255, 255);
			cloud->showNormals(true);
			m_root_gl_->setSunLight(true);
		}
		//cloud->showColors(true);
		//cloud->showSF(false); //just in case
		//cloud->prepareDisplayForRefresh();


		//cloud->setRGBColor(255, 255, 255);
		//cloud->showColors(true);
		ccMesh* tmp_mesh = new ccMesh(cloud);
		if (!tmp_colors.empty())
		{
			tmp_mesh->showNormals(false);
		}
		else 
		{
			tmp_mesh->showNormals(true);
		}
		tmp_mesh->showColors(true);

		ccGenericPointCloud* vertices = tmp_mesh->getAssociatedCloud();
		cloud = static_cast<ccPointCloud*>(vertices);
		//cloud->setRGBColor(255, 255, 255);
		if (!cloud->hasColors())
		{
			cloud->setRGBColor(255, 255, 255);
		}
		cloud->showColors(true);
		cloud->prepareDisplayForRefresh();

		for (int i = 0; i < tmp_faces.size(); ++i)
		{
			tmp_mesh->addTriangle(tmp_faces[i].x, tmp_faces[i].y, tmp_faces[i].z);
		}
		//tmp_mesh->add
		//tmp_mesh->showWired(true);
		//tmp_mesh->isShownAsWire();
		ccHObject* show_obj = new ccHObject(select_name);
		//设置context
		//ccGenericPointCloud* tmp_points = tmp_mesh->getAssociatedCloud();
		//tmp_points->getPoint()

		show_obj->addChild(tmp_mesh);
		//show_obj->toFile()
		//system("pause");
		////test for mesh trans
		//std::ofstream test_obj("D:/vc_project_xht/result_test.obj");
		//for (int i = 0; i < tmp_points->size(); ++i)
		//{
		//	test_obj << "v " << tmp_points->getPoint(i)->x << " " << tmp_points->getPoint(i)->y << " " << tmp_points->getPoint(i)->z << std::endl;
		//	//CCVector3f tmp(tmp_points->getPoint(i)->x, tmp_points->getPoint(i)->y, tmp_points->getPoint(i)->z);
		//}
		//for (int i = 0; i < tmp_points->size(); ++i)
		//{
		//	test_obj << "vn " << tmp_points->getPointNormal(i).x << " " << tmp_points->getPointNormal(i).y << " " << tmp_points->getPointNormal(i).z << std::endl;
		//}
		//for (int i = 0; i < tmp_mesh->size(); ++i)
		//{
		//	CCLib::VerticesIndexes tmp_face(tmp_mesh->getTriangleVertIndexes(i)->i[0],
		//		tmp_mesh->getTriangleVertIndexes(i)->i[1], tmp_mesh->getTriangleVertIndexes(i)->i[2]);
		//	test_obj << "f " << tmp_face.i[0] + 1 << "/0/" << tmp_face.i[0] + 1 << " "
		//		<< tmp_face.i[1] + 1 << "/0/" << tmp_face.i[1] + 1 << " "
		//		<< tmp_face.i[2] + 1 << "/0/" << tmp_face.i[2] + 1 << std::endl;
		//}
		//test_obj.close();
		////end test

		//look for object's parent
		ccHObject* parentObject = show_obj->getParent();

		if (parentObject)
		{
			parentObject = m_ccRoot1;
		}
		else
		{
			m_ccRoot1->addChild(show_obj);
		}

		if (this->ui.actionShowWired->isChecked())
		{
			ccMesh* selected_mesh = dynamic_cast<ccMesh*>(m_ccRoot1->getFirstChild()->getFirstChild());

			selected_mesh->showWired(true);
			//m_root_gl_->setWire(true);
		}
		else
		{
			ccMesh* selected_mesh = dynamic_cast<ccMesh*>(m_ccRoot1->getFirstChild()->getFirstChild());
			selected_mesh->showWired(false);
			//m_root_gl_->setSunLight(true);
		}
	}

	assert(m_root_gl_);
	m_ccRoot1->setDisplay_recursive(m_root_gl_);
	this->m_root_gl_->redraw();
	////test
	/*ccGLCameraParameters camera_test;
	m_root_gl_->getGLCameraParameters(camera_test);
	printf("%lf, %lf, %lf, %f\n", camera_test.modelViewMat(0, 0), camera_test.modelViewMat(0, 1),
	camera_test.modelViewMat(0, 2), camera_test.modelViewMat(0, 3));
	printf("%lf, %lf, %lf, %f\n", camera_test.modelViewMat(1, 0), camera_test.modelViewMat(1, 1),
	camera_test.modelViewMat(1, 2), camera_test.modelViewMat(1, 3));
	printf("%lf, %lf, %lf, %f\n", camera_test.modelViewMat(2, 0), camera_test.modelViewMat(2, 1),
	camera_test.modelViewMat(2, 2), camera_test.modelViewMat(2, 3));
	printf("%lf, %lf, %lf, %f\n", camera_test.modelViewMat(3, 0), camera_test.modelViewMat(3, 1),
	camera_test.modelViewMat(3, 2), camera_test.modelViewMat(3, 3));*/
	////end test
}

ccGLWindow* MainWindow::getActiveGLWindow()
{
	if (!m_mdiArea)
	{
		return 0;
	}

	QMdiSubWindow *activeSubWindow = m_mdiArea->activeSubWindow();
	if (activeSubWindow)
	{
		return GLWindowFromWidget(activeSubWindow->widget());
	}
	else
	{
		QList<QMdiSubWindow*> subWindowList = m_mdiArea->subWindowList();
		if (!subWindowList.isEmpty())
		{
			return GLWindowFromWidget(subWindowList[0]->widget());
		}
	}

	return 0;
}

ccGLWindow* MainWindow::getGLWindow(int index) const
{
	QList<QMdiSubWindow*> subWindowList = m_mdiArea->subWindowList();
	if (index >= 0 && index < subWindowList.size())
	{
		ccGLWindow* win = GLWindowFromWidget(subWindowList[index]->widget());
		assert(win);
		return win;
	}
	else
	{
		assert(false);
		return 0;
	}
}

int MainWindow::getGLWindowCount() const
{
	return m_mdiArea ? m_mdiArea->subWindowList().size() : 0;
}

ccGLWindow* MainWindow::newPlaneView(bool allowEntitySelection)
{
	//assert(m_ccRoot && m_mdiArea);
	QWidget* viewWidget = nullptr;
	ccGLWindow* view3D = nullptr;

	createGLWindow(view3D, viewWidget);
	if (!viewWidget || !view3D)
	{
		ccLog::Error("Failed to create the 3D view");
		assert(false);
		return nullptr;
	}

	//这里可以加一些对界面的操作，包括把原来的界面disable,现有的界面进行显示操作


	return view3D;
}

ccGLWindow* MainWindow::new3DView(bool allowEntitySelection)
{

	//assert(m_ccRoot && m_mdiArea);
	QWidget* viewWidget = nullptr;
	ccGLWindow* view3D = nullptr;

	createGLWindow(view3D, viewWidget);
	if (!viewWidget || !view3D)
	{
		ccLog::Error("Failed to create the 3D view");
		assert(false);
		return nullptr;
	}

	////restore options
	//{
	//	QSettings settings;
	//	bool autoPickRotationCenter = settings.value(ccPS::AutoPickRotationCenter(), true).toBool();
	//	view3D->setAutoPickPivotAtCenter(autoPickRotationCenter);
	//}

	view3D->setAutoPickPivotAtCenter(true);
	viewWidget->setMinimumSize(400, 300);

	m_mdiArea->addSubWindow(viewWidget);

	if (allowEntitySelection)
	{
		/*connect(view3D, &ccGLWindow::entitySelectionChanged, this, [=] (ccHObject *entity) {
		m_ccRoot->selectEntity( entity );
		});

		connect(view3D, &ccGLWindow::entitiesSelectionChanged, this, [=] (std::unordered_set<int> entities){
		m_ccRoot->selectEntities( entities );
		});*/
	}

	//'echo' mode
	connect(view3D, &ccGLWindow::mouseWheelRotated, this, &MainWindow::echoMouseWheelRotate);
	//connect(view3D,	&ccGLWindow::cameraDisplaced, this, &MainWindow::echoCameraDisplaced);
	connect(view3D, &ccGLWindow::viewMatRotated, this, &MainWindow::echoBaseViewMatRotation);
	connect(view3D, &ccGLWindow::cameraPosChanged, this, &MainWindow::echoCameraPosChanged);
	connect(view3D, &ccGLWindow::pivotPointChanged, this, &MainWindow::echoPivotPointChanged);
	//connect(view3D,	&ccGLWindow::pixelSizeChanged, this, &MainWindow::echoPixelSizeChanged);
	//connect(view3D,	&QObject::destroyed, this, &MainWindow::prepareWindowDeletion);
	//connect(view3D,	&ccGLWindow::filesDropped, this, &MainWindow::addToDBAuto, Qt::QueuedConnection); //DGM: we don't want to block the 'dropEvent' method of ccGLWindow instances!
	//connect(view3D,	&ccGLWindow::newLabel, this, &MainWindow::handleNewLabel);
	//connect(view3D,	&ccGLWindow::exclusiveFullScreenToggled, this, &MainWindow::onExclusiveFullScreenToggled);
	if (m_pickingHub)
	{
		//we must notify the picking hub as well if the window is destroyed
		connect(view3D, &QObject::destroyed, m_pickingHub, &ccPickingHub::onActiveWindowDeleted);
	}
	//view3D->setSceneDB(m_ccRoot->getRootEntity());
	view3D->setSceneDB(m_ccRoot1);
	viewWidget->setAttribute(Qt::WA_DeleteOnClose);
	//m_ccRoot->updatePropertiesView();
	//QMainWindow::statusBar()->showMessage(QString("New 3D View"), 2000);

	viewWidget->showMaximized();
	viewWidget->update();

	return view3D;
}

void MainWindow::zoomOnSelectedEntities()
{
	ccGLWindow* win = nullptr;

	ccHObject tempGroup("TempGroup");
	size_t selNum = m_selectedEntities.size();
	for (size_t i = 0; i < selNum; ++i)
	{
		ccHObject *entity = m_selectedEntities[i];

		if (i == 0 || !win)
		{
			//take the first valid window as reference
			win = static_cast<ccGLWindow*>(entity->getDisplay());
		}

		if (win)
		{
			if (entity->getDisplay() == win)
			{
				tempGroup.addChild(entity, ccHObject::DP_NONE);
			}
			else if (entity->getDisplay() != nullptr)
			{
				ccLog::Error("All selected entities must be displayed in the same 3D view!");
				return;
			}
		}
	}

	if (tempGroup.getChildrenNumber() != 0)
	{
		ccBBox box = tempGroup.getDisplayBB_recursive(false, win);
		if (!box.isValid())
		{
			ccLog::Warning("Selected entities have no valid bounding-box!");
		}
		else
		{
			if (win != nullptr)
			{
				win->updateConstellationCenterAndZoom(&box);
			}
		}
	}

	refreshAll();
}

void MainWindow::setGlobalZoom()
{
	ccGLWindow* win = getActiveGLWindow();
	if (win)
		win->zoomGlobal();
}

void MainWindow::setView(CC_VIEW_ORIENTATION view)
{
	ccGLWindow* win = getActiveGLWindow();
	if (win)
	{
		win->setView(view);
	}
}

void MainWindow::toggleActiveWindowCenteredPerspective()
{
	ccGLWindow* win = getActiveGLWindow();
	if (win)
	{
		const ccViewportParameters& params = win->getViewportParameters();
		// && !checkStereoMode(win)
		if (params.perspectiveView && params.objectCenteredView) //we need to check this only if we are already in object-centered perspective mode
		{
			return;
		}
		win->togglePerspective(true);
		win->redraw(false);
		//updateViewModePopUpMenu(win);
		//updatePivotVisibilityPopUpMenu(win);
	}
}

void MainWindow::on3DViewActivated(QMdiSubWindow* mdiWin)
{
	ccGLWindow* win = mdiWin ? GLWindowFromWidget(mdiWin->widget()) : nullptr;
	//if (win)
	//{
	//	//updateViewModePopUpMenu(win);
	//	//updatePivotVisibilityPopUpMenu(win);
	//	m_UI->actionLockRotationAxis->blockSignals(true);
	//	m_UI->actionLockRotationAxis->setChecked(win->isRotationAxisLocked());
	//	m_UI->actionLockRotationAxis->blockSignals(false);
	//	m_UI->actionEnableStereo->blockSignals(true);
	//	m_UI->actionEnableStereo->setChecked(win->stereoModeIsEnabled());
	//	m_UI->actionEnableStereo->blockSignals(false);
	//	m_UI->actionExclusiveFullScreen->blockSignals(true);
	//	m_UI->actionExclusiveFullScreen->setChecked(win->exclusiveFullScreen());
	//	m_UI->actionExclusiveFullScreen->blockSignals(false);
	//	m_UI->actionShowCursor3DCoordinates->blockSignals(true);
	//	m_UI->actionShowCursor3DCoordinates->setChecked(win->cursorCoordinatesShown());
	//	m_UI->actionShowCursor3DCoordinates->blockSignals(false);
	//	m_UI->actionAutoPickRotationCenter->blockSignals(true);
	//	m_UI->actionAutoPickRotationCenter->setChecked(win->autoPickPivotAtCenter());
	//	m_UI->actionAutoPickRotationCenter->blockSignals(false);
	//}
	//m_UI->actionLockRotationAxis->setEnabled(win != nullptr);
	//m_UI->actionEnableStereo->setEnabled(win != nullptr);
	//m_UI->actionExclusiveFullScreen->setEnabled(win != nullptr);
}

void MainWindow::redrawAll(bool only2D/*=false*/)
{
	for (QMdiSubWindow *window : m_mdiArea->subWindowList())
	{
		GLWindowFromWidget(window->widget())->redraw(only2D);
	}
}

void MainWindow::refreshAll(bool only2D/*=false*/)
{
	for (QMdiSubWindow *window : m_mdiArea->subWindowList())
	{
		GLWindowFromWidget(window->widget())->refresh(only2D);
	}
}

void MainWindow::enableAll()
{
	for (QMdiSubWindow *window : m_mdiArea->subWindowList())
	{
		window->setEnabled(true);
	}
}

void MainWindow::disableAll()
{
	for (QMdiSubWindow *window : m_mdiArea->subWindowList())
	{
		window->setEnabled(false);
	}
}

void MainWindow::disableAllBut(ccGLWindow* win)
{
	//we disable all other windows
	for (QMdiSubWindow *window : m_mdiArea->subWindowList())
	{
		if (GLWindowFromWidget(window->widget()) != win)
		{
			window->setEnabled(false);
		}
	}
}

void MainWindow::echoMouseWheelRotate(float wheelDelta_deg)
{
	/*if (!m_UI->actionEnableCameraLink->isChecked())
	return;*/

	ccGLWindow* sendingWindow = dynamic_cast<ccGLWindow*>(sender());
	if (!sendingWindow)
		return;

	for (QMdiSubWindow *window : m_mdiArea->subWindowList())
	{
		ccGLWindow *child = GLWindowFromWidget(window->widget());
		if (child != sendingWindow)
		{
			child->blockSignals(true);
			child->onWheelEvent(wheelDelta_deg);
			child->blockSignals(false);
			child->redraw();
		}
	}
}

void MainWindow::echoBaseViewMatRotation(const ccGLMatrixd& rotMat)
{
	/*if (!m_UI->actionEnableCameraLink->isChecked())
	return;*/

	ccGLWindow* sendingWindow = dynamic_cast<ccGLWindow*>(sender());
	if (!sendingWindow)
		return;

	for (QMdiSubWindow *window : m_mdiArea->subWindowList())
	{
		ccGLWindow *child = GLWindowFromWidget(window->widget());
		if (child != sendingWindow)
		{
			child->blockSignals(true);
			child->rotateBaseViewMat(rotMat);
			child->blockSignals(false);
			child->redraw();
		}
	}
}

void MainWindow::echoCameraPosChanged(const CCVector3d& P)
{
	/*if (!m_UI->actionEnableCameraLink->isChecked())
	return;*/

	ccGLWindow* sendingWindow = dynamic_cast<ccGLWindow*>(sender());
	if (!sendingWindow)
		return;


	for (QMdiSubWindow *window : m_mdiArea->subWindowList())
	{
		ccGLWindow *child = GLWindowFromWidget(window->widget());
		if (child != sendingWindow)
		{
			child->blockSignals(true);
			child->setCameraPos(P);
			child->blockSignals(false);
			child->redraw();
		}
	}
}

void MainWindow::echoPivotPointChanged(const CCVector3d& P)
{
	/*if (!m_UI->actionEnableCameraLink->isChecked())
	return;*/

	ccGLWindow* sendingWindow = dynamic_cast<ccGLWindow*>(sender());
	if (!sendingWindow)
		return;

	for (QMdiSubWindow *window : m_mdiArea->subWindowList())
	{
		ccGLWindow *child = GLWindowFromWidget(window->widget());
		if (child != sendingWindow)
		{
			child->blockSignals(true);
			child->setPivotPoint(P);
			child->blockSignals(false);
			child->redraw();
		}
	}
}

///************** STATIC METHODS ******************/
//
MainWindow* MainWindow::TheInstance()
{
	if (!s_instance)
		s_instance = new MainWindow();
	return s_instance;
}

void MainWindow::DestroyInstance()
{
	delete s_instance;
	s_instance = nullptr;
}

void MainWindow::GetGLWindows(std::vector<ccGLWindow*>& glWindows)
{
	const QList<QMdiSubWindow*> windows; //= TheInstance()->m_mdiArea->subWindowList();

	if (windows.empty())
		return;

	glWindows.clear();
	glWindows.reserve(windows.size());

	for (QMdiSubWindow *window : windows)
	{
		glWindows.push_back(GLWindowFromWidget(window->widget()));
	}
}

ccGLWindow* MainWindow::GetActiveGLWindow()
{
	return TheInstance()->getActiveGLWindow();
}

ccGLWindow* MainWindow::GetGLWindow(const QString& title)
{
	const QList<QMdiSubWindow *> windows;// = TheInstance()->m_mdiArea->subWindowList();

	if (windows.empty())
		return nullptr;

	for (QMdiSubWindow *window : windows)
	{
		ccGLWindow* win = GLWindowFromWidget(window->widget());
		if (win->windowTitle() == title)
			return win;
	}

	return nullptr;
}

void MainWindow::RefreshAllGLWindow(bool only2D/*=false*/)
{
	TheInstance()->refreshAll(only2D);
}

void MainWindow::UpdateUI()
{
	//TheInstance()->updateUI();
}

void MainWindow::createGLWindow(ccGLWindow*& window, QWidget*& widget) const
{
	bool stereoMode = QSurfaceFormat::defaultFormat().stereo();

	CreateGLWindow(window, widget, stereoMode);
	assert(window && widget);
}

void MainWindow::destroyGLWindow(ccGLWindow* view3D) const
{
	if (view3D)
	{
		view3D->setParent(0);
		delete view3D;
	}
}

void MainWindow::SetCameraParams(const ccGLMatrixd& cam_pos)
{
	//
	m_root_gl_->setupProjectiveViewport(cam_pos);
}

void MainWindow::SetCameraPoseFovParams(const ccGLMatrixd& cam_pos, float fov_deg)
{
	//
	m_root_gl_->setupProjectiveViewport(cam_pos, fov_deg);
}

void MainWindow::SetCameraPoseFovArParams(const ccGLMatrixd& cam_pos, float fov_deg, float ar)
{
	//
	m_root_gl_->setupProjectiveViewport(cam_pos, fov_deg, ar);
}

//-------------------------end opengl-------------------//


void MainWindow::CreateConnection()
{
	//check two area
	//assert(db_tree_root_);
	//assert(mid_area_);
	connect(this->ui.actionEditPlane, SIGNAL(triggered()), this, SLOT(EditPlanePolygons()));
	connect(this->ui.actionImport, SIGNAL(triggered()), this, SLOT(LoadCamFileAction()));
	connect(this->ui.actionload_pnts_segs, SIGNAL(triggered()), this, SLOT(LoadHWFileSamplePntsTxtAction()));

	//RenderCurCamColorImageAction
	connect(this->ui.actionRenderColor, SIGNAL(triggered()), this, SLOT(RenderCurCamColorImageAction()));
	//RenderAllCamsColorImgsAction
	connect(this->ui.actionRenderAllColors, SIGNAL(triggered()), this, SLOT(RenderAllCamsColorImgsAction()));

	connect(this->ui.actionOpen, SIGNAL(triggered()), this, SLOT(LoadFileAction()));
	connect(this->ui.actionSave, SIGNAL(triggered()), this, SLOT(SaveFileAction()));
	connect(this->ui.actionClear, SIGNAL(triggered()), this, SLOT(ClearFileAction()));
	connect(this->ui.actionSOR, SIGNAL(triggered()), this, SLOT(ToDoSORFilterAction()));
	connect(this->ui.actionMLS, SIGNAL(triggered()), this, SLOT(ToDoMLSAction()));
	connect(this->ui.actionConstruct, SIGNAL(triggered()), this, SLOT(ToDoConstructionAction()));
	connect(this->ui.actionSimplify, SIGNAL(triggered()), this, SLOT(ToDoSimplificationAction()));
	connect(this->ui.actionEdgeSwap, SIGNAL(triggered()), this, SLOT(DoEdgeSwapAction()));

	//view list
	connect(this->ui.DbItemListWidget, SIGNAL(customContextMenuRequested(const QPoint&)), this, SLOT(ShowViewContextMenu(const QPoint&)));
	connect(this->ui.DbItemListWidget, SIGNAL(itemSelectionChanged()), this, SLOT(ChooseViewListItem()));
	//connect(this->ui.DbItemListWidget, SIGNAL(itemPressed(QListWidgetItem *)), this, SLOT(ChooseViewListItem(QListWidgetItem *)));
	connect(this->ui.DbItemListWidget, SIGNAL(itemChanged(QListWidgetItem *)), this, SLOT(SetActiveObjectsItems(QListWidgetItem *)));
	//connect(this->ui.DbItemListWidget, SIGNAL(stateChanged(int)), this, SLOT(SetActiveObjectsItems()));

	//SOR
	connect(this->ui.spinBoxPointNum, SIGNAL(editingFinished()), this, SLOT(SetSORPntNumAction()));
	connect(this->ui.doubleSpinMultiplier, SIGNAL(editingFinished()), this, SLOT(SetSORMultiplierThresholdAction()));
	connect(this->ui.buttonBoxSOR, SIGNAL(accepted()), this, SLOT(DoSORFilterAction()));
	connect(this->ui.buttonBoxSOR, SIGNAL(rejected()), this, SLOT(CancelSORFilterAction()));

	//MLS
	connect(this->ui.doubleSpinBoxSearchRadius, SIGNAL(editingFinished()), this, SLOT(SetMLSSearchRadiusAction()));
	connect(this->ui.checkBoxNormal, SIGNAL(stateChanged(int)), this, SLOT(CheckMLSComputeNormalFlagAction()));
	connect(this->ui.checkBoxPolynomial, SIGNAL(stateChanged(int)), this, SLOT(CheckMLSUsePolynomialFlagAction()));
	connect(this->ui.doubleSpinBoxOrder, SIGNAL(editingFinished()), this, SLOT(SetMLSPolygonOrderAction()));
	connect(this->ui.doubleSpinBoxGassian, SIGNAL(editingFinished()), this, SLOT(SetMLSSqrtGassionAction()));
	connect(this->ui.UpsampleMethodsComboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(SetMLSUpsampleMethodsAction(int)));
	connect(this->ui.SampleRadiusSpinBox, SIGNAL(editingFinished()), this, SLOT(SetMLSUpsampleRadiusAction()));
	connect(this->ui.SampleStepSpinBox, SIGNAL(editingFinished()), this, SLOT(SetMLSUpsampleStepSizeAction()));
	connect(this->ui.RandomUniformSpinBox, SIGNAL(editingFinished()), this, SLOT(SetMLSRandomDensityAction()));
	connect(this->ui.buttonBoxIdentifyCheck, SIGNAL(accepted()), this, SLOT(DoMLSAction()));
	connect(this->ui.buttonBoxIdentifyCheck, SIGNAL(rejected()), this, SLOT(CancelMLSAction()));

	//Extract Plane
	connect(this->ui.spinBoxPlanePointNum, SIGNAL(editingFinished()), this, SLOT(SetPlaneMinPntsNumAction()));
	connect(this->ui.doubleSpinBoxCeilHeight, SIGNAL(editingFinished()), this, SLOT(SetCeilHeightAction()));
	connect(this->ui.doubleSpinBoxPlaneMergeHeight, SIGNAL(editingFinished()), this, SLOT(SetPlaneMergeHeightAction()));
	connect(this->ui.doubleSpinBoxMaxNorm, SIGNAL(editingFinished()), this, SLOT(SetNormalDeviationAction()));
	connect(this->ui.doubleSpinBoxPlaneWidth, SIGNAL(editingFinished()), this, SLOT(SetMinPlaneWidthAction()));
	connect(this->ui.spinBox_min_merge_pnts, SIGNAL(editingFinished()), this, SLOT(SetPntsNumForMergeAction()));
	connect(this->ui.doubleSpinBox_xy, SIGNAL(editingFinished()), this, SLOT(SetXYRatioForMergeAction()));
	connect(this->ui.doubleSpinBox_area, SIGNAL(editingFinished()), this, SLOT(SetAreaForMergeAction()));
	
	connect(this->ui.buttonBoxIdentifyCheck_4, SIGNAL(accepted()), this, SLOT(DoExtractPlaneAction()));
	connect(this->ui.buttonBoxIdentifyCheck_4, SIGNAL(rejected()), this, SLOT(CancelExtractPlaneAction()));

	//plane operation
	connect(this->ui.actionPlaneOperation, SIGNAL(triggered()), this, SLOT(SetPlaneOperationState()));

	connect(this->ui.actionPolygonSnap, SIGNAL(triggered()), this, SLOT(DoPolygonSnapAction()));

	connect(this->ui.actionShowPolygon, SIGNAL(triggered()), this, SLOT(ShowPolygonAction()));
	//
	connect(this->ui.actionRemoveObjFromSecene, SIGNAL(triggered()), this, SLOT(RemoveTheObjFromSceneAction()));
	connect(this->ui.actionMergePointCloud, SIGNAL(triggered()), this, SLOT(MergeSelectedPointClouds()));
	connect(this->ui.actionMergeObj, SIGNAL(triggered()), this, SLOT(MergeSelectedObjs()));
	connect(this->ui.actionCopy2PolyView, SIGNAL(triggered()), this, SLOT(ShowPntsInPolyViewAction()));

	//Contruction
	connect(this->ui.doubleSpinBoxVoxelLength, SIGNAL(editingFinished()), this, SLOT(SetConstructionVoxelLengthAction()));
	connect(this->ui.doubleSpinBoxTruncLength, SIGNAL(editingFinished()), this, SLOT(SetContructionVolumeTruncAction()));
	connect(this->ui.spinBox_x, SIGNAL(editingFinished()), this, SLOT(SetConstructionVolumeX()));
	connect(this->ui.spinBox_y, SIGNAL(editingFinished()), this, SLOT(SetConstructionVolumeY()));
	connect(this->ui.spinBox_z, SIGNAL(editingFinished()), this, SLOT(SetConstructionVolumeZ()));
	connect(this->ui.buttonBoxIdentifyCheck_2, SIGNAL(accepted()), this, SLOT(DoConstructionAction()));
	connect(this->ui.buttonBoxIdentifyCheck_2, SIGNAL(rejected()), this, SLOT(CancelConstructionAction()));

	//Simplification
	connect(this->ui.spinBoxTargetFaces, SIGNAL(editingFinished()), this, SLOT(SetSimplifyTargetFaceNumAction()));
	connect(this->ui.doubleSpinBoxPercentage, SIGNAL(editingFinished()), this, SLOT(SetSimplifyReductionRatioAction()));
	connect(this->ui.doubleSpinBoxThreshold, SIGNAL(editingFinished()), this, SLOT(SetSimplifyQualityThresholdAction()));
	connect(this->ui.checkBoxBoundary, SIGNAL(stateChanged(int)), this, SLOT(CheckSimplifyBundryPreserveAction()));
	connect(this->ui.doubleSpinBoxBoundaryWeight, SIGNAL(editingFinished()), this, SLOT(SetSimplifyBundryWeightAction()));
	connect(this->ui.checkBoxNormal_2, SIGNAL(stateChanged(int)), this, SLOT(CheckSimplifyNormalPreserveAction()));
	connect(this->ui.checkBoxTopology, SIGNAL(stateChanged(int)), this, SLOT(CheckSimplifyToplogyPreserveAction()));
	connect(this->ui.checkBoxOptimal, SIGNAL(stateChanged(int)), this, SLOT(CheckSimplifyOptimalAction()));
	connect(this->ui.checkBoxPlanarsimplify, SIGNAL(stateChanged(int)), this, SLOT(CheckSimplifyPlanarSimplifyAction()));
	connect(this->ui.buttonBoxIdentifyCheck_3, SIGNAL(accepted()), this, SLOT(DoSimplifyAction()));
	connect(this->ui.buttonBoxIdentifyCheck_3, SIGNAL(rejected()), this, SLOT(CancelSimplifyAction()));

	//Texture
	connect(this->ui.actionTexture, SIGNAL(triggered()), this, SLOT(DoTextureMappingAction()));
	connect(this->ui.actionTexture_AfterRemoving, SIGNAL(triggered()), this, SLOT(DoTextureMappingAction()));
	//opengl
	connect(this->ui.actionSunLight, SIGNAL(triggered()), this, SLOT(SetSunLightAction()));
	connect(this->ui.actionShowWired, SIGNAL(triggered()), this, SLOT(SetShowWireAction()));

	//TOOL Action
	connect(this->ui.actionPickPoint, SIGNAL(triggered()), this, SLOT(ActivatePointPickingMode()));
	connect(this->ui.actionActionSampleInPolygon, SIGNAL(triggered()), this, SLOT(DoSamplingAction()));
	connect(this->ui.actionCompletePolygon, SIGNAL(triggered()), this, SLOT(DoCompletePolygonAction()));
	connect(this->ui.actionConvertFilesAction, SIGNAL(triggered()), this, SLOT(DoConvertFilesIntoHWFilesAction()));
}


//------------------------menu-------------------------//
void MainWindow::SetPlaneOperationState()
{
	std::cout << "set the plane operation!" << std::endl;
	if (this->ui.actionPlaneOperation->isChecked())
	{
		std::cout << "start run plane state!" << std::endl;
		plane_state_ = true;
	}
	else
	{
		std::cout << "cancel plane state!" << std::endl;
		plane_state_ = false;
	}
}

void MainWindow::DoPolygonSnapAction()
{
	printf("run Polygon Snap...\n");
	printf("current file: %s\n", db_tree_->current_element_idx_.c_str());
	if (db_tree_->current_element_idx_.empty())
	{
		printf("invalid select object...\n");
		return;
	}

	if (db_tree_->AccessTreeElements().empty())
	{
		printf("empty object...\n");
		return;
	}
	std::map<std::string, HW::HWObject*>::iterator iter = db_tree_->AccessTreeElements().find(db_tree_->current_element_idx_);
	if (iter != db_tree_->AccessTreeElements().end()&&iter->second->GetObjectType() == HW::kHWPointCloud)
	{
		HW::HWPCLFunctor* snap_process = new HW::HWSNAP();
		processes_vec_.emplace_back(snap_process);
		HW::HWObject* snap_result_obj;
		snap_process->Process(iter->second, snap_result_obj);
		//std::string resulted_file_name = iter->first.substr(0, iter->first.find_last_of(".")) + "_snapresult.ply";
		//printf("reuslted_file_name: %s\n", resulted_file_name.c_str());
		//std::map<std::string, HW::HWObject*>::iterator iter_snap = db_tree_->AccessTreeElements().find(resulted_file_name);
		//if (iter_snap != db_tree_->AccessTreeElements().end())
		//{
		//	//删除原来的，保存现在的file name
		//	//释放空间
		//	if (iter_snap->second != NULL)
		//	{
		//		delete iter_snap->second;
		//		iter_snap->second = snap_process->resulted_element_;
		//		return;
		//	}
		//}
		//else
		//{
		//	//设置对象
		//	db_tree_->current_element_idx_ = resulted_file_name;
		//	snap_process->resulted_element_->SetObjectName(db_tree_->current_element_idx_);
		//	db_tree_->AccessTreeElements().insert(std::pair<std::string, HW::HWObject*>(db_tree_->current_element_idx_, snap_process->resulted_element_));
		//}
		//CopyHWObjToCCObj(db_tree_->current_element_idx_);
		//AddViewListAItem(db_tree_->current_element_idx_);
		//active_objects_names_.emplace_back(db_tree_->current_element_idx_);
		//ShowAllActivedObjects();
	}
}

void MainWindow::ShowPolygonAction()
{
	printf("run Show Polygon...\n");
	printf("current file: %s\n", db_tree_->current_element_idx_.c_str());
	if (db_tree_->current_element_idx_.empty())
	{
		printf("invalid select object...\n");
		return;
	}

	if (db_tree_->AccessTreeElements().empty())
	{
		printf("empty object...\n");
		return;
	}
	std::map<std::string, HW::HWObject*>::iterator iter = db_tree_->AccessTreeElements().find(db_tree_->current_element_idx_);
	if (iter != db_tree_->AccessTreeElements().end() && iter->second->GetObjectType() == HW::kHWPolygon)
	{
		HW::HWPCLFunctor* showpolygon_process = new HW::HWShowPolygon();
		processes_vec_.emplace_back(showpolygon_process);
		HW::HWObject* result_obj;
		showpolygon_process->Process(iter->second, result_obj);
		#ifdef LIULINGFEI
		std::wcout << "Process done" << std::endl;
		#endif
	}
}

void MainWindow::MergeSelectedPointClouds()
{
	if (active_objects_names_.size() != 2) {
		std::cerr << "error, should select 2 point clouds to merge" << std::endl;
		return;
	}
	std::cout << "begin merge " << active_objects_names_[0] << " and " << active_objects_names_[1] << std::endl;

	std::map<std::string, HW::HWObject*>::iterator iter0 = db_tree_->AccessTreeElements().find(active_objects_names_[0]);
	std::map<std::string, HW::HWObject*>::iterator iter1 = db_tree_->AccessTreeElements().find(active_objects_names_[1]);

	HW::HWPointCloud* first_cloud;
	HW::HWPointCloud* second_cloud;
	if (iter0 != db_tree_->AccessTreeElements().end() && iter0->second->GetObjectType() == HW::kHWPointCloud &&
		iter1 != db_tree_->AccessTreeElements().end() && iter1->second->GetObjectType() == HW::kHWPointCloud)
	{
		first_cloud = dynamic_cast<HW::HWPointCloud*> (iter0->second);
		second_cloud = dynamic_cast<HW::HWPointCloud*> (iter1->second);
	}


	HW::HWPointCloud* merged = new HW::HWPointCloud();
	HW::MergeClutteredHWPointCloud(*merged, *first_cloud, *second_cloud);

	HW::HWObject* element_add = merged;
	element_add->SetObjectType(HW::ElementType::kHWPointCloud);
	db_tree_->current_element_idx_ = merged->GetObjectName();
	std::cout << "load current element idx: " << db_tree_->current_element_idx_ << std::endl;
	element_add->SetObjectName(db_tree_->current_element_idx_);
	db_tree_->AccessTreeElements().insert(std::pair<std::string, HW::HWObject*>(db_tree_->current_element_idx_, element_add));

	//插入对象
	CopyHWObjToCCObj(db_tree_->current_element_idx_);
	ShowAllActivedObjects();

	AddViewListAItem(db_tree_->current_element_idx_);
	active_objects_names_.emplace_back(db_tree_->current_element_idx_);
	std::cout << "end merge " << active_objects_names_[0] << " and " << active_objects_names_[1] << std::endl;
}

void MainWindow::MergeSelectedObjs()
{
	//merge point clouds
	if (active_objects_names_.size() < 2) {
		std::cerr << "error, should select at least 2 obj files to merge" << std::endl;
		return;
	}
	std::cout << "begin merge obj:" << std::endl;
	for (int i = 0; i < active_objects_names_.size(); ++i) {
		std::cout << active_objects_names_[i] << std::endl;
	}

	typedef std::map<std::string, HW::HWObject*>::iterator iter_type;

	std::cout << "begin search active_object_names_" << std::endl;
	std::vector<iter_type> iters(active_objects_names_.size(), db_tree_->AccessTreeElements().end());
	for (int i = 0; i < iters.size(); ++i) {
		iters[i] = db_tree_->AccessTreeElements().find(active_objects_names_[i]);
	}

	std::cout << "begin build polygons" << std::endl;
	//std::map<std::string, HW::HWObject*>::iterator iter0 = db_tree_->AccessTreeElements().find(active_objects_names_[0]);
	//std::map<std::string, HW::HWObject*>::iterator iter1 = db_tree_->AccessTreeElements().find(active_objects_names_[1]);
	std::vector<HW::HWPolygon*> polygons;
	for (const auto& it : iters) {
		if (it != db_tree_->AccessTreeElements().end() && it->second->GetObjectType() == HW::kHWPolygon) {
			HW::HWPolygon* in_polygon = dynamic_cast<HW::HWPolygon*> (it->second);
			//in_polygon->SetObjectType(HW::ElementType::kHWPolygon);
			polygons.push_back(in_polygon);
		}
	}
	HW::HWPolygon* merged = new HW::HWPolygon();
	HW::MergeHWPolygon(*merged, polygons);

	HW::HWObject* element_add = merged;
	element_add->SetObjectType(HW::ElementType::kHWPolygon);
	db_tree_->current_element_idx_ = merged->GetObjectName();
	std::cout << "load current element idx: " << db_tree_->current_element_idx_ << std::endl;
	element_add->SetObjectName(db_tree_->current_element_idx_);
	db_tree_->AccessTreeElements().insert(std::pair<std::string, HW::HWObject*>(db_tree_->current_element_idx_, element_add));

	//插入对象
	std::cout << "insert " << db_tree_->current_element_idx_ << std::endl;
	CopyHWObjToCCObj(db_tree_->current_element_idx_);
	ShowAllActivedObjects();

	AddViewListAItem(db_tree_->current_element_idx_);
	active_objects_names_.emplace_back(db_tree_->current_element_idx_);
}

//有一些bug，暂且不知道为啥显示不出来？
void MainWindow::ShowPntsInPolyViewAction()
{
	printf("run Show Pnts In POLYView...\n");
	printf("current file: %s\n", db_tree_->current_element_idx_.c_str());
	if (db_tree_->current_element_idx_.empty())
	{
		printf("invalid select object...\n");
		return;
	}

	if (db_tree_->AccessTreeElements().empty())
	{
		printf("empty object...\n");
		return;
	}
	std::map<std::string, HW::HWObject*>::iterator iter = db_tree_->AccessTreeElements().find(db_tree_->current_element_idx_);
	if (iter != db_tree_->AccessTreeElements().end() && iter->second->GetObjectType() == HW::kHWPointCloud)
	{
		//判断是否有点云
		//HW::HWPCLFunctor* showpolygon_process = new HW::HWShowPolygon();
		//processes_vec_.emplace_back(showpolygon_process);
		HW::HWObject* result_obj;
		//showpolygon_process->Process(iter->second, result_obj);
		//获取在polygon view 中显示的hwPolygon,
		int idx_polygon = -1;
		for (int i = 0; i < processes_vec_.size(); ++i)
		{
			if (processes_vec_[i] 
				&& processes_vec_[i]->resulted_element_->GetObjectType() == HW::kHWPolygon)
			{
				idx_polygon = i;
			}
		}
		
		//将点云数据传入到里面,这种不具备它们之间的一一对应关系
		if (idx_polygon!= -1 &&processes_vec_[idx_polygon]->resulted_element_ 
			&& processes_vec_[idx_polygon])
		{
			HW::HWShowPolygon* myshowpolygon = dynamic_cast<HW::HWShowPolygon*>(processes_vec_[idx_polygon]);
			if (myshowpolygon)
			{
				std::cerr << "start to load all pointcloud..." << std::endl;
				HW::HWPointCloud* pc = dynamic_cast<HW::HWPointCloud*>(iter->second);
				myshowpolygon->polyview_widget_->SetAllPointCloud(pc);
			}
		}

#ifdef LIULINGFEI
		std::wcout << "Process done" << std::endl;
#endif
	}
}

void MainWindow::EditPlanePolygons()
{
#if 0
	m_plane_window_ = new HW::HWPlaneViewGL();
	disableAll();
#endif

	if (plane_state_)
	{
		std::cout << "edit the polygons!" << std::endl;
		//获取当前的对象，可以判断一下是否为点云对象
		//增加这个点云是否是plane，然后去处理这个点云
		HW::HWObject* hw_obj = GetCurrentObject();
		if (hw_obj->GetObjectType() == HW::kHWPlane)
		{
			//统计时间
			HW::HWPlane* plane_obj = static_cast<HW::HWPlane*>(hw_obj);
			clock_t start_time, end_time;
			start_time = clock();

			plane_obj->GenerateWorldCoordToPlaneCoordMatrix();
			plane_obj->ProjectTo3DPlane();
			//std::string plane_path = "D:\\vc_project_xht\\back_up\\test_for_proj\\plane_pnts.obj";
			//plane_obj->SavePlanePointIntoOBJ(plane_path);
			//std::cout << "end test" << std::endl;
			//std::system("pause");

			plane_obj->Generate2DPlane();
			plane_obj->DoEstimateBorderFrom2DPlane();
			plane_obj->SortBorderEdges();
			plane_obj->GenerateInitial2DSortedPnts();
			plane_obj->Generate2DPloygons();
			plane_obj->MapSortedEdgePnt2PntPosIdx();
			std::cout << "end map the sorted edge pnts!!" << std::endl;

			plane_obj->ComputeImageConvertParams();
			plane_obj->GenerateEdgeImagePolygon();
			plane_obj->TriangulatePolygon2D();
			plane_obj->GenerateSampleWorldCoordPnts();
			end_time = clock();
			std::cout << "end fill hole time: " << (double)(end_time - start_time) / CLOCKS_PER_SEC << "s" << std::endl;

			plane_obj->MergeSampleAndSrcPnts();
			//plane_obj->SaveMargedPlanePntsPly("D:/vc_project_xht/back_up/test_for_plane/merged_plane_pnt.ply", HW::kBinary);

			std::string origin_name = plane_obj->GetObjectName().substr(0,
				plane_obj->GetObjectName().find("_plane")) + ".ply";

			std::cout << "plane_obj->GetObjectName() name is: " << plane_obj->GetObjectName() << std::endl;
			std::cout << "origin 111 name is: " << origin_name << std::endl;

			if (GetObjectByStrName(origin_name))
			{
				db_tree_->current_element_idx_ = origin_name;
				std::cout << "current 11111 name is: " << db_tree_->current_element_idx_ << std::endl;

				HW::HWObject* origin_obj = GetCurrentObject();
				std::cout << "current 11111 name is: " << origin_obj->GetObjectName() << std::endl;


				HW::HWPointCloud* orgin_pc = static_cast<HW::HWPointCloud*>(origin_obj);
				//把平面对象sample得到的顶点加入到原来的点云对象中
				//获取sample的顶点
				std::vector<float3> sampled_vertices = plane_obj->GetSampledWorldPnts();
				Eigen::Vector3f normals = plane_obj->ComputeAverageVertexNormal();

				ccColor::Rgb col(255, 255, 255);
				uchar3 tmp_color = make_uchar3(col.r, col.g, col.b);
				for (int i = 0; i < sampled_vertices.size(); ++i)
				{
					orgin_pc->AddPoint(sampled_vertices[i]);

					//将平面方程的法向量加入进去
					if (orgin_pc->HasNormal())
						orgin_pc->AddNormal(make_float3(normals.x(), normals.y(), normals.z()));

					if (orgin_pc->HasColor())
						orgin_pc->AddColor(tmp_color);
				}
			}
		}
		std::cout << "end all " << std::endl;
		ShowAllActivedObjects();
	}

#if 0
	std::cout << "edit the polygons!" << std::endl;
	//获取当前的对象，可以判断一下是否为点云对象
	//HW::HWObject* obj = 
	//增加这个点云是否是plane，然后去处理这个点云
	std::string current_name = db_tree_->current_element_idx_;
	ccPointCloud* obj = static_cast<ccPointCloud*>(GetObjectByStrName(current_name));
	if (true)
	{
		m_plane_ = new HW::HWPlane();
		//将点云数据传输到HWPlane对象里面
		for (int i = 0; i < obj->size(); ++i)
		{
			const CCVector3* pnt = obj->getPoint(i);
			CCVector3f pnt_normal = obj->getPointNormal(i);
			m_plane_->AddPlanePnt(pnt->x, pnt->y, pnt->z);
			m_plane_->AddPlanePntNormal(pnt_normal.x, pnt_normal.y, pnt_normal.z);
		}

		//统计时间
		clock_t start_time, end_time;
		start_time = clock();

		m_plane_->GenerateWorldCoordToPlaneCoordMatrix();
		m_plane_->ProjectTo3DPlane();
		std::string plane_path = "D:\\vc_project_xht\\back_up\\test_for_proj\\plane_pnts.obj";
		m_plane_->SavePlanePointIntoOBJ(plane_path);
		std::cout << "end test" << std::endl;
		std::system("pause");

		m_plane_->Generate2DPlane();
		m_plane_->DoEstimateBorderFrom2DPlane();
		m_plane_->SortBorderEdges();
		m_plane_->GenerateInitial2DSortedPnts();
		m_plane_->Generate2DPloygons();
		m_plane_->MapSortedEdgePnt2PntPosIdx();
		std::cout << "end map the sorted edge pnts!!" << std::endl;
		//保存
		//std::string plane_coord_path = "D:\\vc_project_xht\\back_up\\test_for_plane\\plane_obj\\remain\\plane_coord.obj";
		//std::string refine_path = "D:\\vc_project_xht\\back_up\\test_for_plane\\plane_obj\\remain\\border_pnts.obj";
		//m_plane_->SavePlaneCoordPntsIntoOBJ(plane_coord_path);
		//m_plane_->SavePlaneRefinedEdgePntsIntoOBJ(refine_path);
		m_plane_->ComputeImageConvertParams();
		m_plane_->GenerateEdgeImagePolygon();
		m_plane_->TriangulatePolygon2D();
		m_plane_->GenerateSampleWorldCoordPnts();
		end_time = clock();
		std::cout << "process fill hole time: " << (double)(end_time - start_time) / CLOCKS_PER_SEC << "s" << std::endl;

		m_plane_->MergeSampleAndSrcPnts();
		m_plane_->SaveMargedPlanePntsPly("D:/vc_project_xht/back_up/test_for_plane/merged_plane_pnt.ply", HW::kBinary);
	}
	std::cout << "end all " << std::endl;
	//system("pause");

	//to do next
	//m_plane_window_ = new HW::HWPlaneViewGL();
	//disableAll();
#endif

	return;
}

void MainWindow::LoadCamFileAction()
{
	QString strDir = QFileDialog::getExistingDirectory(
		this
		, tr("Open Directory"));
	std::cerr << "start load scenes cams..." << std::endl;
	HW::HWScenesCams* tmp_cams = new HW::HWScenesCams();
	std::string str_dir = strDir.toStdString();
	std::cerr << "str_dir: " << str_dir << std::endl;
	tmp_cams->SetCamsDir(str_dir);
	tmp_cams->SetImagesDir(str_dir);
	tmp_cams->LoadElements();
	loaded_scenes_cams_ = tmp_cams;
	std::cerr << "end loaded cams..." << std::endl;

#if 0
	//qDebug() << "test...\n";
	QString file_name_qstr = QFileDialog::getOpenFileName(this, tr("select a file"));
	if (file_name_qstr.isEmpty())
	{
		std::cout << "no file loading..." << std::endl;
		return;
	}
	std::string file_name = file_name_qstr.toStdString();
	//
	std::string suffixStr = file_name.substr(file_name.find_last_of('.') + 1);
	if (suffixStr == ".txt")
	//if (true)
	{
		std::cout << "asdfsddfsdfsdf" << std::endl;
		assert(m_root_gl_);
		if (m_root_gl_)
		{
			Eigen::Matrix4f rotation_matrix, trans_matrix;
			rotation_matrix << 0.0703577, -0.997388, -0.0163547, 0,
				0.062383, -0.0119643, 0.997981, 0,
				-0.995569, -0.0712364, 0.0613779, 0,
				0, 0, 0, 1;
			trans_matrix << 1, 0, 0, -19.7902,
				0, 1, 0, -2.6624,
				0, 0, 1, -1.2631,
				0, 0, 0, 1;
			Eigen::Matrix4f view_matrix = rotation_matrix * trans_matrix;

			Eigen::Matrix4f cam_matrix = view_matrix.inverse();

			//都是列优先
			ccGLMatrixd cc_cam_matrix(cam_matrix.data());
		
			SetCameraParams(cc_cam_matrix);

			std::cout << "end setting!!!!" << std::endl;
			ShowAllActivedObjects();
		}
	}
#endif
}

void MainWindow::LoadHWFileSamplePntsTxtAction()
{
	QString strDir = QFileDialog::getExistingDirectory(
		this
		, tr("Open Directory"));
	std::cerr << "start load sample points..." << std::endl;
	std::string str_dir = strDir.toStdString();
	std::vector<std::string> files_paths = HW::GetFilesListFromDir(str_dir);
	std::string ply_file_path;
	std::string planes_pnts_path;
	for (int i = 0; i < files_paths.size(); ++i)
	{
		if (files_paths[i].find(".ply") != std::string::npos)
		{
			ply_file_path = files_paths[i];
			break;
		}
	}
	ply_file_path = HW::GetLeftSlashPathName(ply_file_path);
	//std::string ply_file_basename = HW::GetBaseName(planes_pnts_path);
	std::string ply_file_basename_nosuffix = HW::GetPathPrefix(HW::GetBaseName(ply_file_path));
	std::cerr << "ply_file_basename_nosuffix: " << ply_file_basename_nosuffix << std::endl;
	for (int i = 0; i < files_paths.size(); ++i)
	{
		bool find_corresponding_flag = false;
		if (files_paths[i].find(".txt") != std::string::npos)
		{
			std::string tmp_plane_pnts_file_path;
			tmp_plane_pnts_file_path = HW::GetLeftSlashPathName(files_paths[i]);
			std::string plane_file_basename_nosuffix
				= HW::GetPathPrefix(HW::GetBaseName(tmp_plane_pnts_file_path));
			std::cerr << "plane_file_basename_nosuffix: " << plane_file_basename_nosuffix << std::endl;
			if (ply_file_basename_nosuffix == plane_file_basename_nosuffix)
			{
				planes_pnts_path = files_paths[i];
				find_corresponding_flag = true;
			}
		}
		if (find_corresponding_flag)
			break;
	}
	

	if (!ply_file_path.empty() && !planes_pnts_path.empty())
	{
		planes_pnts_path = HW::GetLeftSlashPathName(planes_pnts_path);
		std::cerr << "ply_file_path, planes_pnts_path: " <<
			ply_file_path << ", " << planes_pnts_path << std::endl;

		//get the file
		if (ReadPly(ply_file_path, true))
		{
			proj_sub_names_.emplace_back(ply_file_path);
		}
		//获取对应的obj文件
		HW::HWObject *pnts_element = db_tree_->AccessTreeElements()[ply_file_path];
		if (pnts_element)
		{
			HW::HWPointCloud* cloud_pnts = static_cast<HW::HWPointCloud*>(pnts_element);
			std::ifstream fhd(planes_pnts_path);
			std::vector<float3> cloud_poses = cloud_pnts->GetVertices();
			int cloud_poses_num = static_cast<int>(cloud_poses.size());
			std::cerr << "cloud_poses_num: " << cloud_poses_num << std::endl;
			std::vector<std::vector<int> > planes_idxs_vec;
			std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f> > planes_infos;
			if (fhd.is_open())
			{
				std::string str;
				std::getline(fhd, str);
				int planes_num;
				if (str.empty())
					return;
				std::stringstream ss(str);
				ss >> planes_num;
				for (int i = 0; i < planes_num; ++i)
				{
					std::string local_str;
					std::pair<Eigen::Vector3f, Eigen::Vector3f> plane_info;
					std::getline(fhd, local_str);
					std::stringstream local_ss0(local_str);
					local_ss0 >> plane_info.first[0] >> plane_info.first[1] >> plane_info.first[2]
						>> plane_info.second[0] >> plane_info.second[1] >> plane_info.second[2];
					std::getline(fhd, local_str);
					int plane_pnts_num;
					std::stringstream local_ss1(local_str);
					local_ss1 >> plane_pnts_num;
					std::vector<int> plane_pnts_idxs;
					for (int j = 0; j < plane_pnts_num; ++j)
					{
						std::string local0_str;
						std::getline(fhd, local0_str);
						int plane_pnts_idx;
						std::stringstream local0_ss1(local0_str);
						local0_ss1 >> plane_pnts_idx;
						plane_pnts_idxs.emplace_back(plane_pnts_idx);
					}
					planes_idxs_vec.emplace_back(plane_pnts_idxs);
					planes_infos.emplace_back(plane_info);
				}
				fhd.close();
			}

			std::string merge_cloud_name_prefix = HW::GetPathPrefix(ply_file_path);
			std::string merge_cloud_name = merge_cloud_name_prefix + "_merge.ply";
			
			//new cloud for sub-part
			ccPointCloud* pcMerge = new ccPointCloud(merge_cloud_name.c_str());
			pcMerge->reserveTheNormsTable();
			pcMerge->reserveTheRGBTable();
			//resort point with plane 
			std::vector<int> plane_idxs_range;
			plane_idxs_range.emplace_back(0);
			for (int i = 0; i < planes_idxs_vec.size(); ++i)
			{
				std::pair<Eigen::Vector3f, Eigen::Vector3f> plane_info = planes_infos[i];
				Eigen::Vector3f plane_n = plane_info.first;
				ccColor::Rgb tmp_col = ccColor::Generator::Random();
				int tmp_plane_pnts_num = static_cast<int>(planes_idxs_vec[i].size());
				plane_idxs_range.emplace_back(plane_idxs_range.back() + tmp_plane_pnts_num);
				for (int j = 0; j < planes_idxs_vec[i].size(); ++j)
				{
					int tmp_idx = planes_idxs_vec[i][j];
					float3 tmp_pnt = cloud_poses[tmp_idx];
					CCVector3 pos(tmp_pnt.x, tmp_pnt.y, tmp_pnt.z);
					CCVector3 norm(plane_n[0], plane_n[1], plane_n[2]);
					pcMerge->addPoint(pos);
					pcMerge->addNorm(norm);
					pcMerge->addRGBColor(tmp_col);
				}
			}

			ccPointCloud* pc;
			//获取被选中的点云的名字
			std::string current_selected_name = db_tree_->current_element_idx_;
			//std::string 
			for (int i = 0; i < m_ccRoot1->getChildrenNumber(); ++i)
			{
				//
				if (m_ccRoot1->getChild(i)->getName() == QString(current_selected_name.data()))
				{
					pc = static_cast<ccPointCloud*>(m_ccRoot1->getChild(i)->getFirstChild());
					break;
				}
			}

			const CCVector3d& globalShift = pc->getGlobalShift();
			double globalScale = pc->getGlobalScale();

			pcMerge->showColors(true);
			//pcMerge->showNormals(true);
			pcMerge->setVisible(true);
			pcMerge->setGlobalShift(globalShift);
			pcMerge->setGlobalScale(globalScale);
			std::vector<int> planes_iswide;
			planes_iswide.resize(planes_idxs_vec.size(), 1);
			CopyCCPCToHWPC(pcMerge, plane_idxs_range, planes_iswide);
			AddCCObjectToWeidgetList(pcMerge);
			pcMerge->showSF(false); //just in case
			pcMerge->prepareDisplayForRefresh();

			//把pcMerge放入m_ccRoot1中
			ccHObject* show_merge_obj = new ccHObject(pcMerge->getName());
			show_merge_obj->addChild(pcMerge);
			m_ccRoot1->addChild(show_merge_obj);
			active_objects_names_.emplace_back(pcMerge->getName().toStdString());
		}

		ShowAllActivedObjects();
	}
}

void MainWindow::RenderCurCamColorImageAction()
{
	std::cerr << "render current cam color image..." << std::endl;
	//get 
	QString strDir = QFileDialog::getExistingDirectory(
		this
		, tr("Open Directory"));
	std::cerr << "current render color image dir: " << strDir.toStdString() << std::endl;
	//ShowAllActivedObjects();
	if (m_ccRoot1 && m_root_gl_)
	{
		QImage cur_img = m_root_gl_->renderToImage();
		QString str_path = strDir + "/" + "cur_render_color.png";
		cur_img.save(str_path);
	}
	std::cerr << "end current cam color image..." << std::endl;
}

void MainWindow::RenderAllCamsColorImgsAction()
{
	std::cerr << "render all cam color images..." << std::endl;
	//get 
	QString strDir = QFileDialog::getExistingDirectory(
		this
		, tr("Open Directory"));
	std::cerr << "current render color image dir: " << strDir.toStdString() << std::endl;
	//ShowAllActivedObjects();
	if (m_ccRoot1 && m_root_gl_)
	{
		//set the loaded camera to ccdisplay scene 
		if (loaded_scenes_cams_)
		{
			//const std::vector<>
			const std::vector<CameraModel> cams_models = loaded_scenes_cams_->GetCamerasModels();
			for (int i = 0; i < cams_models.size(); ++i)
			{
				//set the loaded camera to ccdisplay
				std::cerr << "cam " << i << ": " << "rendering..." << std::endl;
				CameraModel tmp_model = cams_models[i];
				std::string tmp_path = tmp_model.path_;
				Eigen::Matrix4f tmp_cam = tmp_model.cam_pose_;
				Eigen::Matrix4d tmp_camd;
				for (int c = 0; c < 4; ++c)
				{
					for (int r = 0; r < 4; ++r)
					{
						tmp_camd(r, c) = tmp_cam(r, c);
					}
				}
				/*tmp_camd(0, 0) = tmp_cam(0, 0); tmp_camd(0, 1) = tmp_cam(0, 1);
				tmp_camd(0, 2) = tmp_cam(0, 2); tmp_camd(0, 3) = tmp_cam(0, 3);
				tmp_camd(1, 0) = tmp_cam(1, 0); tmp_camd(1, 1) = tmp_cam(1, 1);
				tmp_camd(1, 2) = tmp_cam(1, 2); tmp_camd(1, 3) = tmp_cam(1, 3);
				tmp_camd(2, 0) = tmp_cam(2, 0); tmp_camd(2, 1) = tmp_cam(2, 1);
				tmp_camd(2, 2) = tmp_cam(2, 2); tmp_camd(2, 3) = tmp_cam(2, 3);
				tmp_camd(3, 0) = tmp_cam(3, 0); tmp_camd(3, 1) = tmp_cam(3, 1);
				tmp_camd(3, 2) = tmp_cam(3, 2); tmp_camd(3, 3) = tmp_cam(3, 3);*/
				ccGLMatrixd cc_cam_matrix(tmp_camd.data());
				float fx = tmp_model.fx_;
				float fy = tmp_model.fy_;
				float cx = tmp_model.cx_;
				float v_value = 0.5 * (2 * cx) / fy;
				float fov_value = 2 * std::atan(v_value);
				std::cerr << "fov value: " << fov_value << std::endl;
				SetCameraParams(cc_cam_matrix);	//set intrinsic params(to do next...)
				//SetCameraParams(cc_cam_matrix, fov_value);	//set intrinsic params(to do next...)
				QImage cur_img = m_root_gl_->renderToImage();
				std::string cur_path = HW::GetBaseNameWithoutSuffix(tmp_path);
				QString cur_q_path = QString(cur_path.data());
				QString str_path = strDir + "/" + cur_q_path + "_render.png";
				std::cerr << "str_path: " << str_path.toStdString() << std::endl;
				cur_img.save(str_path);
				QString str_cam_path = strDir + "/" + cur_q_path + "_cam.obj";
				std::string str_cam_path_std = str_cam_path.toStdString();
				tmp_model.SaveMeOnlyIntoObj(str_cam_path_std, 0.4);
			}
		}
	}
	std::cerr << "end render all cam color images..." << std::endl;
}

void MainWindow::LoadFileAction()
{
	//qDebug() << "test...\n";
	QString file_name_qstr = QFileDialog::getOpenFileName(this, tr("select a file"));
	if (file_name_qstr.isEmpty())
	{
		std::cout << "no file loading..." << std::endl;
		return;
	}
	std::string file_name = file_name_qstr.toStdString();
	file_names_.emplace_back(file_name);

    //
    std::string suffixStr = file_name.substr(file_name.find_last_of('.') + 1);
    if (suffixStr == "e57") {
        ReadE57File(file_name);
		AddViewListAItem(file_name);
		active_objects_names_.emplace_back(db_tree_->current_element_idx_);
    }
    else if(suffixStr == "ply"){
        //这里是读ply，后续还可以读obj
        ReadPly(file_name, false);

		AddViewListAItem(file_name);
		active_objects_names_.emplace_back(db_tree_->current_element_idx_);
#if 0
	//test
	//获取当前读取的点云
	std::string current_name = db_tree_->current_element_idx_;
	HW::HWPointCloud *pc = static_cast<HW::HWPointCloud *>(db_tree_->AccessTreeElements()[current_name]);
	system("pause");

#if 1
	//获取平面
	HW::HWPlane* test_plane = new HW::HWPlane();

	std::cout << "featch the pc: " << pc->GetObjectName() << std::endl;
	for (int i = 0; i < pc->GetVertices().size(); ++i)
	{
		test_plane->AddPlanePnt(pc->GetVertices()[i].x, pc->GetVertices()[i].y, pc->GetVertices()[i].z);
		if (pc->HasNormal())
			test_plane->AddPlanePntNormal(pc->GetNormal()[i].x, pc->GetNormal()[i].y, pc->GetNormal()[i].z);
	}

	test_plane->GenerateWorldCoordToPlaneCoordMatrix();

	test_plane->Generate2DPlane();

	test_plane->DoEstimateBorderFrom2DPlane();

	test_plane->SortBorderEdges();

	test_plane->GenerateInitial2DSortedPnts();

	test_plane->Generate2DPloygons();

	system("pause");

	std::vector<float3> proj_vertices;
	proj_vertices = test_plane->GetPlaneCoordPos();
	std::vector<float3> plane_normals;

#if 0
	//获取点云的平面方程

	////获取点云的平面方程
	//std::vector<float3> tmp_vertices;
	//for (int i = 0; i < test_plane->GetPlanePnts().size(); ++i)
	//{
	//	tmp_vertices.emplace_back(make_float3(test_plane->GetPlanePnts()[i].x,
	//		test_plane->GetPlanePnts()[i].y, 1.0f));
	//}
	//std::cout << "out proj vertices is: " << tmp_vertices.size() << std::endl;
	////获取边缘的顶点，保存下来
	//const std::string edge_path1 = "D:\\vc_project_xht\\edge\\tmp_vertices.obj";
	//SaveHWPCIntoObjFile(tmp_vertices, edges_normals, edge_path1);
	//std::cout << "end save tmp obj file" << std::endl;
	//system("pause");

	std::cout << "out edge vertices is: " << proj_vertices.size() << std::endl;
	//获取边缘的顶点，保存下来
	const std::string edge_path = "D:\\vc_project_xht\\edge\\edge4.obj";

	SaveHWPCIntoObjFile(proj_vertices, edges_normals, edge_path);

	std::cout << "end save obj file" << std::endl;

	test_plane->Generate2DPloygons();

	std::cout << "test_plane->lines_pos_.size: " << test_plane->lines_pos_.size() << std::endl;

	for (int i = 0; i < test_plane->lines_pos_.size(); ++i)
	{
		//
		std::cout << "i: " << i << std::endl;
		std::string line_path = "D:\\vc_project_xht\\edge\\line_" + std::to_string(i) + ".obj";
		SaveHWPCIntoObjFile(test_plane->lines_pos_[i], edges_normals, line_path);
	}

	std::cout << "end save line pnts !" << std::endl;
	system("pause");
#endif

#if 1
	//获取点云的平面方程
	std::cout << "proj_vertices vertices is: " << proj_vertices.size() << std::endl;
	//获取平面坐标系的顶点，保存下来
	const std::string plane_path = "D:\\vc_project_xht\\edge\\plane_coord.obj";
	SaveHWPCIntoObjFile(proj_vertices, plane_normals, plane_path);
	std::cout << "end save plane obj file" << std::endl;

	//system("pause");

	std::vector<float3> edge_vertices;
	edge_vertices = test_plane->GetPlaneEdgePnts();
	std::vector<float3> edge_normals;
	//获取平面的边缘顶点，保存下来
	const std::string edge_path = "D:\\vc_project_xht\\edge\\edge_coord.obj";
	SaveHWPCIntoObjFile(edge_vertices, edge_normals, edge_path);

#endif

#endif

#if 0

	//test
	HW::HWPlane* test = new HW::HWPlane();

	std::cout << "featch the pc: " << pc->GetObjectName() << std::endl;
	for (int i = 0; i < pc->GetVertices().size(); ++i)
	{
		test->AddPlanePnt(pc->GetVertices()[i].x, pc->GetVertices()[i].y, pc->GetVertices()[i].z);
		if(pc->HasNormal())
			test->AddPlanePntNormal(pc->GetNormal()[i].x, pc->GetNormal()[i].y, pc->GetNormal()[i].z);
	}
	test->DoEstimateBorders();

	std::vector<float3> edges_vertices;
	std::vector<float3> edges_normals;

	for (int i = 0; i < test->GetPlaneEdgePnts().size(); ++i)
	{
		edges_vertices.emplace_back(test->GetPlaneEdgePnts()[i]);
	}
	std::cout << "edges vertices is: " << edges_vertices.size() << std::endl;
	//获取边缘的顶点，保存下来
	const std::string edge_path = "D:\\vc_project_xht\\edge\\edge.obj";

	SaveHWPCIntoObjFile(edges_vertices, edges_normals, edge_path);

	delete test;
	system("pause");

#endif

	//end test
#endif
		
        //ShowOpenglObject(db_tree_->current_element_idx_);
    }
	else if(suffixStr == "obj") {
		ReadObj(file_name);

		AddViewListAItem(file_name);
		active_objects_names_.emplace_back(db_tree_->current_element_idx_);
	}
	else if (suffixStr == "hw")	//这是工程文件
	{
		std::cerr << "start to load HW Project" << std::endl;
		ReadProjFile(file_name);
		
		//
		for (int i = 0; i < proj_sub_names_.size(); ++i)
		{
			AddViewListAItem(proj_sub_names_[i]);
			active_objects_names_.emplace_back(proj_sub_names_[i]);
		}
	}

#if 0
	//设置cam pos
	const ccGLMatrixd cam;
	cam<<
#endif

	ShowAllActivedObjects();
}

void MainWindow::SaveFileAction()
{
	std::cout << "start saving..." << std::endl;
	db_tree_->SaveCurrentObject(HW::kBinary);
	std::cout << "end saving..." << std::endl;

#if 0

	//获取相机位置
	ccGLCameraParameters temp_param;
	m_root_gl_->getGLCameraParameters(temp_param);

	//
	ccGLMatrixd cam_param = temp_param.modelViewMat.inverse();
	ConvertMatrixToELUEAngle(cam_param);
	std::cout << "matrix: " << std::endl;
	std::cout << cam_param(0, 0) << " " << cam_param(0, 1) << " " << cam_param(0, 2) << " " << cam_param(0, 3) << std::endl;
	std::cout << cam_param(1, 0) << " " << cam_param(1, 1) << " " << cam_param(1, 2) << " " << cam_param(1, 3) << std::endl;
	std::cout << cam_param(2, 0) << " " << cam_param(2, 1) << " " << cam_param(2, 2) << " " << cam_param(2, 3) << std::endl;
	std::cout << cam_param(3, 0) << " " << cam_param(3, 1) << " " << cam_param(3, 2) << " " << cam_param(3, 3) << std::endl;
#endif

}

void MainWindow::ClearFileAction()
{
	printf("clear all the file...\n");
	//删除list view
	int counter = this->ui.DbItemListWidget->count();
	for (int index = 0; index < counter; index++)
	{
		QListWidgetItem *item = this->ui.DbItemListWidget->takeItem(0);
		delete item;
	}
	//清空db tree里面的所有数据
	db_tree_->Clear();
	//恢复原来的状态
	if (m_plane_)
		delete m_plane_;
	if (m_plane_window_)
		delete m_plane_window_;
	//处理opengl
	if (m_ccRoot1)
		delete m_ccRoot1;
}


//------------------------view list------------------//
void MainWindow::ShowViewContextMenu(const QPoint& pos)
{
	printf("menu activate...\n");
	/*QMenu* popMenu = new QMenu(this);
	QAction* add_item = popMenu->addAction("add");
	QAction* delete_item = popMenu->addAction("delete");
	QAction* modify_item = popMenu->addAction("modify");
	add_item->setEnabled(true);
	delete_item->setDisabled(true);
	modify_item->setDisabled(true);*/
	
	pop_menu_->setVisible(true);

	//如果qlist中有一些部件被选中的话，才会出现菜单
	QList<QListWidgetItem*> list = this->ui.DbItemListWidget->selectedItems();

	bool itemChecked = false;
	for (int i = 0; i < list.size(); ++i)
	{
		if (list[i]->checkState() == Qt::Checked)
		{
			itemChecked = true;
			break;
		}
	}
	if (itemChecked)
	{
		//pop_menu_->g
		delete_item_->setEnabled(true);
		modify_item_->setEnabled(true);
	}

	connect(add_item_, SIGNAL(triggered(bool)), this, SLOT(AddItemViewAndObject()));
	connect(delete_item_, SIGNAL(triggered(bool)), this, SLOT(DeleteItemViewAndObject()));

	pop_menu_->exec(QCursor::pos()); // 菜单出现的位置为当前鼠标的位置
}

void MainWindow::DeleteItemViewAndObject()
{
	printf("delete item and object...\n");
	//delete view item
	for (int i = 0; i < this->ui.DbItemListWidget->count(); ++i)
	{
		if (this->ui.DbItemListWidget->item(i)->checkState() == Qt::Checked
			&& this->ui.DbItemListWidget->item(i)->isSelected())
		{
			//先删除对象
			std::string object_value = this->ui.DbItemListWidget->item(i)->text().toStdString();
			printf("delete object name: %s\n", object_value.c_str());
			//if()
			QListWidgetItem *item = this->ui.DbItemListWidget->takeItem(i);
			delete item;

			//获取点云名字
			std::string current_obj_idx = db_tree_->current_element_idx_;

			//删除在显示数组
			bool existed_flag = false;
			std::vector<std::string>::iterator iter = active_objects_names_.begin();
			for (; iter != active_objects_names_.end(); ++iter)
			{
				if (*iter == current_obj_idx)
				{
					existed_flag = true;
					break;
				}
			}
			if(existed_flag)
				active_objects_names_.erase(iter);

			//删除m_ccRoot1中的数据
			for (int j = 0; j < m_ccRoot1->getChildrenNumber(); ++j)
			{
				QString deleted_obj_name(current_obj_idx.data());
				if (m_ccRoot1->getChild(j)->getName() == deleted_obj_name)
				{
					m_ccRoot1->removeChild(j);
				}
			}

			//删除db_tree中的数据
			delete db_tree_->AccessTreeElements()[current_obj_idx];
			db_tree_->AccessTreeElements().erase(current_obj_idx);
			
			//if (db_tree_->AccessTreeElements().empty())
			//{
			//	//释放原来的对象
			//	//to do......
			//	if (!m_ccRoot1->getName().isEmpty())
			//	{
			//		m_ccRoot1->removeAllChildren();
			//	}
			//	assert(m_root_gl_);
			//	m_ccRoot1->setDisplay_recursive(m_root_gl_);
			//	refreshAll();
			//}

			//printf("view list remainder item11111: %d\n", this->ui.DbItemListWidget->count());
			//system("pause");
			////需要改进一下
			//if (i == 1)
			//{
			//	printf("sadfsadfsas\n");
			//	system("pause");
			//	QListWidgetItem *item = this->ui.DbItemListWidget->takeItem(0);
			//	delete item;
			//}
			//else
			//{
			//}
			//printf("view list remainder item22222: %d\n", this->ui.DbItemListWidget->count());
			//system("pause");
			//&& this->ui.DbItemListWidget->takeItem(i)->isSelected()
			/*QListWidgetItem *item = this->ui.DbItemListWidget->takeItem(i);
			this->ui.DbItemListWidget->removeItemWidget(item);
			delete item;*/
		}
	}

	//delete object
	//printf("view list remainder item: %d\n", this->ui.DbItemListWidget->count());
}

void MainWindow::AddItemViewAndObject()
{
	printf("add item and object...\n");
	LoadFileAction();
}

void MainWindow::AddViewListAItem(const std::string& item_name)
{
	QListWidgetItem* aitem = new QListWidgetItem();
	aitem->setText(item_name.c_str());
	aitem->setCheckState(Qt::Checked);
	aitem->setFlags(Qt::ItemIsSelectable | Qt::ItemIsEditable | Qt::ItemIsUserCheckable | Qt::ItemIsEnabled);
	//增加一个Item到view list 中
	this->ui.DbItemListWidget->addItem(aitem);
}

void MainWindow::ChooseViewListItem()
{
#if 1
	//choose view list item
	if (this->ui.DbItemListWidget->count() == 0)
	{
		printf("no item to  choose..\n");
		return;
	}

	printf("choose one item...\n");
	QString current_term_qstr = ui.DbItemListWidget->currentItem()->text();
	//QString current_term_qstr = item->text();
	db_tree_->current_element_idx_ = current_term_qstr.toStdString();
	printf("current item: %s\n", db_tree_->current_element_idx_.c_str());

	//CopyHWPCToCCPC(db_tree_->current_element_idx_);
	ShowAllActivedObjects();

	//ShowCurrentObject(db_tree_->current_element_idx_);
#endif
}

void MainWindow::SetActiveObjectsItems(QListWidgetItem *item)
{
	//处理对象显示问题
	std::cout << "set active object items...\n" << std::endl;
	QString item_text = item->text();
	std::string item_text_str = item_text.toStdString();
	if (item->checkState() == Qt::Checked)
	{
		bool existed_flag = false;
		for (int i = 0; i < active_objects_names_.size(); ++i)
		{
			if (active_objects_names_[i] == item_text_str)
			{
				existed_flag = true;
				break;
			}
		}
		if (!existed_flag)
			active_objects_names_.emplace_back(item_text_str);
	}
	else
	{
		std::vector<std::string>::iterator iter = active_objects_names_.begin();
		for (; iter != active_objects_names_.end(); ++iter)
		{
			if (*iter == item_text_str)
			{
				active_objects_names_.erase(iter);
				break;
			}
		}
	}

	std::cout << "active_objects_names_ size: " << active_objects_names_.size() << std::endl;

	ShowAllActivedObjects();

	//for (int i = 0; i < ui.DbItemListWidget->count(); ++i)
	//{
	//	if (ui.DbItemListWidget->item(i)->checkState() == Qt::Checked)
	//	{
	//		QString itemText = ui.DbItemListWidget->item(i)->text();
	//		//if()
	//	}
	//}
}

void MainWindow::ToDoSORFilterAction()
{
	std::cout << "to do sor filter action..." << std::endl;
	//0表示sor在toolboox中的索引
	this->ui.toolBox->setCurrentIndex(0);
}

void MainWindow::ToDoMLSAction()
{
	printf("to do moving least square action...\n");
	//1表示MLS在toolboox中的索引
	this->ui.toolBox->setCurrentIndex(1);
}

void MainWindow::ToDoConstructionAction()
{
	printf("to do construct mesh action...\n");
	//2表示Construct Mesh在toolboox中的索引
	this->ui.toolBox->setCurrentIndex(3);
}

void MainWindow::ToDoSimplificationAction()
{
	printf("to do simplify action...\n");
	//3表示Simpilfy在toolboox中的索引
	this->ui.toolBox->setCurrentIndex(4);
}

////------------------------SOR--------------------//
void MainWindow::SetSORPntNumAction()
{
	//
	printf("set point num...\n");
	int point_num = this->ui.spinBoxPointNum->value();
	printf("ponit num:%d\n", point_num);
	HW::HWParams::getInstance().sor_params_.point_num_ = point_num;
}

void MainWindow::SetSORMultiplierThresholdAction()
{
	//
	float sor_threshold = this->ui.doubleSpinMultiplier->value();
	printf("set sor gaussion multiplier threshold: %f\n", sor_threshold);
	HW::HWParams::getInstance().sor_params_.devialtion_multiplier_ = sor_threshold;
}

void MainWindow::DoSORFilterAction()
{
#if 1
	//
	printf("run SOR Filter...\n");
	printf("current file: %s\n",db_tree_->current_element_idx_.c_str());
	if (db_tree_->current_element_idx_.empty())
	{
		printf("invalid select object...\n");
		return;
	}

	if (db_tree_->AccessTreeElements().empty())
	{
		printf("empty object...\n");
		return;
	}
	std::map<std::string, HW::HWObject*>::iterator iter = db_tree_->AccessTreeElements().find(db_tree_->current_element_idx_);
	if (iter != db_tree_->AccessTreeElements().end())
	{
		HW::HWPCLFunctor* sor_process = new HW::HWSOR();
		processes_vec_.emplace_back(sor_process);
		HW::HWObject* out_sor_result;
		sor_process->Process(iter->second, out_sor_result);
		//test
		//std::string tmp_path = "D:/room_sketch/data/huawei_data/Huawei/pointCloud/test_sor.ply";
		//sor_process->resulted_element_->SavePly(tmp_path, HW::kBinary);
		//end test
		////test
		//if (sor_process == processes_vec_[0])
		//{
		//	printf("assign success...\n");
		//	return;
		//}
		////end test

		//生成唯一的对象，用路径名来识别它的唯一性
		std::string resulted_file_name = iter->first.substr(0, iter->first.find_last_of(".")) + "_sor.ply";
		printf("SOR resluted file: %s\n", resulted_file_name.c_str());
		std::map<std::string, HW::HWObject*>::iterator iter_sor = db_tree_->AccessTreeElements().find(resulted_file_name);
		if (iter_sor != db_tree_->AccessTreeElements().end())
		{
			//删除原来的，保存现在的file name
			//释放空间
			if (iter_sor->second != NULL)
			{
				delete iter_sor->second;
				iter_sor->second = sor_process->resulted_element_;
				return;
			}
		}
		else
		{
			//设置对象
			db_tree_->current_element_idx_ = resulted_file_name;
			sor_process->resulted_element_->SetObjectName(db_tree_->current_element_idx_);
			db_tree_->AccessTreeElements().insert(std::pair<std::string, HW::HWObject*>(db_tree_->current_element_idx_, sor_process->resulted_element_));
			//添加对应的item到view list 里面
			//AddViewListAItem(db_tree_->current_element_idx_);
		}
	}
	printf("end SOR Filter...\n");

	//opengl
	printf("sor current idx: %s\n", db_tree_->current_element_idx_.c_str());

	CopyHWObjToCCObj(db_tree_->current_element_idx_);
	AddViewListAItem(db_tree_->current_element_idx_);
	active_objects_names_.emplace_back(db_tree_->current_element_idx_);
	ShowAllActivedObjects();
	//ShowCurrentObject(db_tree_->current_element_idx_);
	//ShowOpenglObject(db_tree_->current_element_idx_);
#endif
}

void MainWindow::CancelSORFilterAction()
{
	//
	printf("Cancel SOR filter...\n");
	//int sor_threshold = this->ui.doubleSpinBoxThreshold->value();
}


//---------------------MLS---------------------//
void MainWindow::SetMLSSearchRadiusAction()
{
	float mls_radius = (float)this->ui.doubleSpinBoxSearchRadius->value();
	printf("set MLS radius: %f\n", mls_radius);
	HW::HWParams::getInstance().mls_params_.search_radius_ = (float)this->ui.doubleSpinBoxSearchRadius->value();
}

void MainWindow::CheckMLSComputeNormalFlagAction()
{
	if (this->ui.checkBoxNormal->checkState() == Qt::Checked)
	{
		printf("check MLS compute normal...\n");
		HW::HWParams::getInstance().mls_params_.compute_normal_flag_ = true;
	}
	else
	{
		printf("uncheck MLS compute normal...\n");
		HW::HWParams::getInstance().mls_params_.compute_normal_flag_ = false;
	}
}

void MainWindow::CheckMLSUsePolynomialFlagAction()
{
	if (this->ui.checkBoxPolynomial->checkState() == Qt::Checked)
	{
		printf("check MLS use polynomial...\n");
		HW::HWParams::getInstance().mls_params_.use_polynomial_flag_ = true;
	}
	else
	{
		printf("uncheck MLS use polynomial...\n");
		HW::HWParams::getInstance().mls_params_.use_polynomial_flag_ = false;
	}
}

void MainWindow::SetMLSPolygonOrderAction()
{
	printf("set MLS order...\n");
	HW::HWParams::getInstance().mls_params_.polynomial_order_ = 
		(float)this->ui.doubleSpinBoxOrder->value();
}

void MainWindow::SetMLSSqrtGassionAction()
{
	printf("set MLS gassion square value...\n");
	HW::HWParams::getInstance().mls_params_.gassian_param_sqrt_ =
		(float)this->ui.doubleSpinBoxGassian->value();
}

void MainWindow::SetMLSUpsampleMethodsAction(int current_idx)
{
	printf("current idx: %d\n", current_idx);
	if (this->ui.UpsampleMethodsComboBox->currentIndex() == 0)
	{
		printf("current idx: %d\n", current_idx);
		HW::HWParams::getInstance().mls_params_.upsample_methods_ = HW::kNone;
		this->ui.SampleLocalGroupBox->setDisabled(true);
		this->ui.RandomUniformGroupBox->setDisabled(true);
	}
	else if (this->ui.UpsampleMethodsComboBox->currentIndex() == 1)
	{
		printf("current idx: %d\n", current_idx);
		HW::HWParams::getInstance().mls_params_.upsample_methods_ = HW::kSampleLocalPlane;
		this->ui.SampleLocalGroupBox->setDisabled(false);
		this->ui.RandomUniformGroupBox->setDisabled(true);
	}
	else if (this->ui.UpsampleMethodsComboBox->currentIndex() == 2)
	{
		printf("current idx: %d\n", current_idx);
		HW::HWParams::getInstance().mls_params_.upsample_methods_ = HW::kRandomUniformDensity;
		this->ui.SampleLocalGroupBox->setDisabled(true);
		this->ui.RandomUniformGroupBox->setDisabled(false);
	}
	else if (this->ui.UpsampleMethodsComboBox->currentIndex() == 3)
	{
		printf("current idx: %d\n", current_idx);
		HW::HWParams::getInstance().mls_params_.upsample_methods_ = HW::kVoxelGridDilation;
		//to do next
	}
}

void MainWindow::SetMLSUpsampleRadiusAction()
{
	printf("set mls upsample radius: %f\n", (float)this->ui.SampleRadiusSpinBox->value());
	HW::HWParams::getInstance().mls_params_.upsampling_radius_
		= (float)this->ui.SampleRadiusSpinBox->value();
}

void MainWindow::SetMLSUpsampleStepSizeAction()
{
	printf("set mls upsample size: %f\n", (float)this->ui.SampleStepSpinBox->value());
	HW::HWParams::getInstance().mls_params_.upsampling_radius_
		= (float)this->ui.SampleStepSpinBox->value();
}

void MainWindow::SetMLSRandomDensityAction()
{
	printf("set mls Random Density: %d\n", (int)this->ui.RandomUniformSpinBox->value());
	HW::HWParams::getInstance().mls_params_.upsampling_radius_
		= (int)this->ui.RandomUniformSpinBox->value();
}

void MainWindow::DoMLSAction()
{
#if 1
	printf("run MLS...\n");
	printf("current mls: %s\n", db_tree_->current_element_idx_.c_str());
	if (db_tree_->current_element_idx_.empty())
	{
		printf("invalid select object...\n");
		return;
	}

	if (db_tree_->AccessTreeElements().empty())
	{
		printf("empty object...\n");
		return;
	}

	std::map<std::string, HW::HWObject*>::iterator iter = db_tree_->AccessTreeElements().find(db_tree_->current_element_idx_);
	if (iter != db_tree_->AccessTreeElements().end())
	{
		//process_ = new HW::HWMLS();
		HW::HWPCLFunctor* mls_process = new HW::HWMLS();
		processes_vec_.emplace_back(mls_process);

		HW::HWObject *out_point_clouds;
		mls_process->Process(iter->second, out_point_clouds);
		////test
		//if (out_point_clouds == NULL)
		//{
		//	printf("assign wrong data...\n");
		//	return;
		//}
		//end test
		HW::HWPointCloud* out_cloud = dynamic_cast<HW::HWPointCloud*>(mls_process->resulted_element_);
		if (out_cloud->GetVertices().empty())
		{
			printf("empty....");
			return;
		}

		////test
		//printf("out point clouds size:%d\n", out_cloud->GetVertices().size());
		//std::string tmp_path = "D:/room_sketch/data/huawei_data/Huawei/pointCloud/test_mls.ply";
		//mls_process->resulted_element_->SavePly(tmp_path, HW::kBinary);
		////end test

		//生成唯一的对象
		/*HW::HWParams::getInstance().unique_obj_label_ = HW::HWParams::getInstance().unique_obj_label_ + 1;
		db_tree_->current_element_idx_ = HW::HWParams::getInstance().unique_obj_label_;*/
		//生成唯一的对象，用路径名来识别它的唯一性
		std::string resulted_file_name = iter->first.substr(0, iter->first.find_last_of(".")) + "_mls.ply";
		printf("reuslted_file_name: %s\n", resulted_file_name.c_str());
		std::map<std::string, HW::HWObject*>::iterator iter_mls = db_tree_->AccessTreeElements().find(resulted_file_name);
		if (iter_mls != db_tree_->AccessTreeElements().end())
		{
			//删除原来的，保存现在的file name
			//释放空间
			if (iter_mls->second != NULL)
			{
				delete iter_mls->second;
				iter_mls->second = mls_process->resulted_element_;
				std::cout << "existed the mls name: " << std::endl;
				return;
			}
		}
		else
		{
			//设置对象
			db_tree_->current_element_idx_ = resulted_file_name;
			mls_process->resulted_element_->SetObjectName(db_tree_->current_element_idx_);
			db_tree_->AccessTreeElements().insert(std::pair<std::string, HW::HWObject*>(db_tree_->current_element_idx_, mls_process->resulted_element_));
			//添加对应的item到view list 里面
			//AddViewListAItem(db_tree_->current_element_idx_);
		}
		
	}
	printf("end MLS...\n");

	CopyHWObjToCCObj(db_tree_->current_element_idx_);
	AddViewListAItem(db_tree_->current_element_idx_);
	active_objects_names_.emplace_back(db_tree_->current_element_idx_);
	ShowAllActivedObjects();
#endif
	//opengl
	//ShowOpenglObject(db_tree_->current_element_idx_);
}

void MainWindow::CancelMLSAction()
{
	printf("Cancel MLS...\n");
}


//---------------------Extract plane-------------------//

void MainWindow::SetPlaneMinPntsNumAction()
{
	printf("set plane point num...\n");
	HW::HWParams::getInstance().plane_params_.plane_min_pnt_num_ =
		(float)this->ui.spinBoxPlanePointNum->value();
}

void MainWindow::SetCeilHeightAction()
{
	printf("set max distance to plane...\n");
	HW::HWParams::getInstance().plane_params_.ceil_height_ =
		(float)this->ui.doubleSpinBoxCeilHeight->value();
}

void MainWindow::SetPlaneMergeHeightAction()
{
	printf("set plane resolution...\n");
	HW::HWParams::getInstance().plane_params_.plane_merge_height_ =
		(float)this->ui.doubleSpinBoxPlaneMergeHeight->value();
}

void MainWindow::SetNormalDeviationAction()
{
	printf("set normal deviation...\n");
	HW::HWParams::getInstance().plane_params_.max_normal_deviation_ =
		(float)this->ui.doubleSpinBoxMaxNorm->value();
}

void MainWindow::SetMinPlaneWidthAction()
{
	printf("set min plane width...\n");
	HW::HWParams::getInstance().plane_params_.min_plane_width_ =
		(float)this->ui.doubleSpinBoxPlaneWidth->value();
}

void MainWindow::SetPntsNumForMergeAction()
{
	printf("set plane min num for merge %f...\n", (float)this->ui.spinBox_min_merge_pnts->value());
	HW::HWParams::getInstance().plane_params_.plane_min_pnt_num_merge_ = 
		(float)this->ui.spinBox_min_merge_pnts->value();
}

void MainWindow::SetXYRatioForMergeAction()
{
	//printf("set plane xy for merge...\n");
	printf("set plane xy for merge %f...\n", (float)this->ui.doubleSpinBox_xy->value());
	HW::HWParams::getInstance().plane_params_.xy_ratio_merge_ =
		(float)this->ui.doubleSpinBox_xy->value();
}

void MainWindow::SetAreaForMergeAction()
{
	//printf("set plane min area for merge...\n");
	printf("set plane min area for merge %f...\n", (float)this->ui.doubleSpinBox_area->value());
	HW::HWParams::getInstance().plane_params_.min_area__merge_ =
		(float)this->ui.doubleSpinBox_area->value();
}
#if 0
void MainWindow::DoExtractPlaneAction()
{
	printf("run extract plane\n");

	//convert selected ccPointCloud to pointcloud
	ccPointCloud* pc;
	//获取被选中的点云的名字
	std::string current_selected_name = db_tree_->current_element_idx_;
	//std::string 
	for (int i = 0; i < m_ccRoot1->getChildrenNumber(); ++i)
	{
		//
		if (m_ccRoot1->getChild(i)->getName() == QString(current_selected_name.data()))
		{
			pc = static_cast<ccPointCloud*>(m_ccRoot1->getChild(i)->getFirstChild());
			break;
		}
	}

	if (!pc)
	{
		printf("select no element!\n");
		return;
	}

	//system("pause");
	//ccPointCloud* pc = static_cast<ccPointCloud*>(m_ccRoot1->getFirstChild()->getFirstChild());
	//std::cout << "get the pc name: " << pc->getName().toStdString() << std::endl;
	//system("pause");
	//input cloud
	unsigned count = pc->size();
	std::cout << "pc: " << count << std::endl;
	std::string extracted_pc_name = pc->getName().toStdString();
	//std::cout << "pc name: " << extracted_pc_name << std::endl;

	//system("pause");

	bool hasNorms = pc->hasNormals();
	CCVector3 bbMin, bbMax;
	pc->getBoundingBox(bbMin, bbMax);
	const CCVector3d& globalShift = pc->getGlobalShift();
	double globalScale = pc->getGlobalScale();

	//const float scale = cloud.getScale();

	//Convert CC point cloud to RANSAC_SD type
	PointCloud cloud;
	{
		try
		{
			cloud.reserve(count);
		}
		catch (...)
		{
			printf("Not enough memory!\n");
			return;
		}

		//default point & normal
		Point Pt;
		Pt.normal[0] = 0.0;
		Pt.normal[1] = 0.0;
		Pt.normal[2] = 0.0;
		for (unsigned i = 0; i<count; ++i)
		{
			const CCVector3* P = pc->getPoint(i);
			Pt.pos[0] = static_cast<float>(P->x);
			Pt.pos[1] = static_cast<float>(P->y);
			Pt.pos[2] = static_cast<float>(P->z);
			if (hasNorms)
			{
				const CCVector3& N = pc->getPointNormal(i);
				Pt.normal[0] = static_cast<float>(N.x);
				Pt.normal[1] = static_cast<float>(N.y);
				Pt.normal[2] = static_cast<float>(N.z);
			}
			cloud.push_back(Pt);
		}

		//manually set bounding box!
		Vec3f cbbMin, cbbMax;
		cbbMin[0] = static_cast<float>(bbMin.x);
		cbbMin[1] = static_cast<float>(bbMin.y);
		cbbMin[2] = static_cast<float>(bbMin.z);
		cbbMax[0] = static_cast<float>(bbMax.x);
		cbbMax[1] = static_cast<float>(bbMax.y);
		cbbMax[2] = static_cast<float>(bbMax.z);
		cloud.setBBox(cbbMin, cbbMax);

		//输出box大小
		printf("cloud cbbMin: %f %f %f \n", cbbMin[0], cbbMin[1], cbbMin[2]);
		printf("cloud cbbMax: %f %f %f \n", cbbMax[0], cbbMax[1], cbbMax[2]);
	}

	const float scale = cloud.getScale();

	std::cout << " the cloud scale is: " << scale << std::endl;

	//import parameters from dialog
	RansacShapeDetector::Options ransacOptions;
	{
		ransacOptions.m_epsilon = HW::HWParams::getInstance().plane_params_.max_dist_plane_;
		ransacOptions.m_bitmapEpsilon = HW::HWParams::getInstance().plane_params_.plane_sample_resolution_;
		ransacOptions.m_normalThresh = std::cos(HW::HWParams::getInstance().plane_params_.max_normal_deviation_ * CC_DEG_TO_RAD);
		assert(ransacOptions.m_normalThresh >= 0);
		ransacOptions.m_probability = HW::HWParams::getInstance().plane_params_.plane_overlook_proble_;
		ransacOptions.m_minSupport = HW::HWParams::getInstance().plane_params_.plane_min_pnt_num_;
	}

	std::cout << "the least fitting is: " << ransacOptions.m_fitting << std::endl;
	//system("pause");

	//time
	clock_t start_time, end_time;
	start_time = clock();

	std::cout << "Start to Extract plane..." << std::endl;

	if (!hasNorms)
	{
		printf("Computing normals (please wait)\n");
		cloud.calcNormals(.01f * scale);

		if (pc->reserveTheNormsTable())
		{
			for (unsigned i = 0; i < count; ++i)
			{
				Vec3f& Nvi = cloud[i].normal;
				CCVector3 Ni = CCVector3::fromArray(Nvi);
				//normalize the vector in case of
				Ni.normalize();
				pc->addNorm(Ni);
			}
			pc->showNormals(true);

			//currently selected entities appearance may have changed!
			pc->prepareDisplayForRefresh_recursive();
		}
		else
		{
			printf("Not enough memory to compute normals!\n");
			return;
		}
	}

	// set which primitives are to be detected by adding the respective constructors
	RansacShapeDetector detector(ransacOptions); // the detector object

												 //只是检测平面
	detector.Add(new PlanePrimitiveShapeConstructor());

	unsigned remaining = count;
	typedef std::pair< MiscLib::RefCountPtr< PrimitiveShape >, size_t > DetectedShape;
	MiscLib::Vector< DetectedShape > shapes; // stores the detected shapes

											 //MiscLib::Vector< std::pair< MiscLib::RefCountPtr< PrimitiveShape >, size_t > > shapes; // stores the detected shapes

	std::cout << "cloud: " << cloud.size() << std::endl;
	std::cout << "ransacOptions.m_epsilon: " << ransacOptions.m_epsilon << std::endl;
	std::cout << "ransacOptions.m_bitmapEpsilon: " << ransacOptions.m_bitmapEpsilon << std::endl;
	std::cout << "ransacOptions.m_normalThresh: " << ransacOptions.m_normalThresh << std::endl;
	std::cout << "ransacOptions.m_probability: " << ransacOptions.m_probability << std::endl;
	std::cout << "ransacOptions.m_minSupport: " << ransacOptions.m_minSupport << std::endl;

	remaining = detector.Detect(cloud, 0, cloud.size(), &shapes);

	if (remaining == count)
	{
		printf("extract plane failed...\n");
		return;
	}

	std::cout << "remaining: \n" << remaining << std::endl;
	std::cout << "ccPointCloud vertex number is: " << pc->size() << std::endl;
	std::cout << "Ransac PointCloud vertex number is: " << cloud.size() << std::endl;

	end_time = clock();
	std::cout << "extract time: " << (double)(end_time - start_time) / CLOCKS_PER_SEC << "s" << std::endl;
	std::cout << "End extract plane..." << std::endl;

	std::string merge_element_name = extracted_pc_name.substr(0, extracted_pc_name.find_last_of(".")) + "_merge.ply";
	printf("the merge plane name: %s\n", merge_element_name.c_str());
	HW::HWPointCloud* merge_element_add = new HW::HWPointCloud();
	merge_element_add->SetObjectType(HW::ElementType::kHWPointCloud);
	merge_element_add->SetObjectName(merge_element_name);

	//new cloud for sub-part
	ccPointCloud* pcMerge = new ccPointCloud(merge_element_name.c_str());
	pcMerge->reserveTheNormsTable();
	pcMerge->reserveTheRGBTable();

	//HWPointCloud: set the removed sub-part
	std::string removed_element_name = extracted_pc_name.substr(0, extracted_pc_name.find_last_of(".")) + "_removed.ply";
	printf("the merge plane name: %s\n", removed_element_name.c_str());
	HW::HWPointCloud* removed_element_add = new HW::HWPointCloud();
	merge_element_add->SetObjectType(HW::ElementType::kHWPointCloud);
	merge_element_add->SetObjectName(removed_element_name);

	//ccPointCloud: a copy from HWPointCloud
	ccPointCloud* pcRemoved = new ccPointCloud(removed_element_name.c_str());
	pcRemoved->reserveTheNormsTable();
	pcRemoved->reserveTheRGBTable();


	//保存平面的名字，用于索引pc分割出的平面
	//std::vector<std::string> planes_names;
	float max_plane_box_dist = -1e6;

	if (shapes.size() > 0)
	{
		int shape_cout = 0;
		for (MiscLib::Vector<DetectedShape>::const_iterator it = shapes.begin(); it != shapes.end(); ++it)
		{
			shape_cout++;

			const PrimitiveShape* shape = it->first;
			unsigned shapePointsCount = static_cast<unsigned>(it->second);
			//too many points?!
			if (shapePointsCount > count)
			{
				printf("Inconsistent result!\n");
				break;
			}
			std::string desc;
			shape->Description(&desc);

			desc = desc + std::to_string(shape_cout) + ".ply";
			//test
			//desc = desc + std::to_string(shape_cout);
			//std::cout << "desc: " << desc << std::endl;
			//system("pause");
			//end test

			//生成唯一的对象，用路径名来识别它的唯一性
			std::string element_name = extracted_pc_name.substr(0, extracted_pc_name.find_last_of(".")) + desc;
			printf("extrated plane name: %s\n", element_name.c_str());

			//将点云存入内存中，转化为HWPointCloud
#if 0
			HW::HWPointCloud* element_add = new HW::HWPointCloud();
			element_add->SetObjectType(HW::ElementType::kHWPointCloud);
			element_add->SetObjectName(element_name);
#endif
			//new cloud for sub-part
			ccPointCloud* pcShape = new ccPointCloud(element_name.c_str());

			//we fill cloud with sub-part points
			if (!pcShape->reserve(static_cast<unsigned>(shapePointsCount)))
			{
				printf("Not enough memory!\n");
				delete pcShape;
				break;
			}

			bool saveNormals = pcShape->reserveTheNormsTable();

			bool saveColors = pcShape->reserveTheRGBTable();
			//random color
			ccColor::Rgb col = ccColor::Generator::Random();

			for (unsigned j = 0; j < shapePointsCount; ++j)
			{
				bool merge_flag = false;
				if (shapePointsCount > 0.01*count)
					merge_flag = true;

				pcShape->addPoint(CCVector3::fromArray(cloud[count - 1 - j].pos));

				////add to HWPointCloud
				//element_add->AddPoint(make_float3(cloud[count - 1 - j].pos[0],
				//	cloud[count - 1 - j].pos[1], cloud[count - 1 - j].pos[2]));

				////add vertex pos to merge pointcloud
				//if(merge_flag)
				//	pcMerge->addPoint(CCVector3::fromArray(cloud[count - 1 - j].pos));
				//else
				//	pcRemoved->addPoint(CCVector3::fromArray(cloud[count - 1 - j].pos));

				if (saveNormals)
				{
					////add to HWPointCloud
					//element_add->AddNormal(make_float3(cloud[count - 1 - j].normal[0],
					//	cloud[count - 1 - j].normal[1], cloud[count - 1 - j].normal[2]));

					pcShape->addNorm(CCVector3::fromArray(cloud[count - 1 - j].normal));
					/*if (merge_flag)
					pcMerge->addNorm(CCVector3::fromArray(cloud[count - 1 - j].normal));
					else
					pcRemoved->addNorm(CCVector3::fromArray(cloud[count - 1 - j].normal));*/
				}
				if (saveColors)
				{
					//element_add->AddColor(make_uchar3(col.r, col.g, col.b));
					pcShape->addRGBColor(col);
					/*if (merge_flag)
					pcMerge->addRGBColor(col);
					else
					pcRemoved->addRGBColor(col);*/
				}
			}


#if SHOW_PLANE_VIEWS
			//pcShape->setRGBColor(col);
			pcShape->showColors(true);
			pcShape->showNormals(saveNormals);
			pcShape->setVisible(true);
			pcShape->setGlobalShift(globalShift);
			pcShape->setGlobalScale(globalScale);

			CopyCCPCToHWPC(pcShape);
			AddCCObjectToWeidgetList(pcShape);

#if 0
			//把产生的平面点云放入dbtree,同时显示出来
			HW::HWObject* object_add = static_cast<HW::HWObject*>(element_add);

			std::map<std::string, HW::HWObject*>::iterator iter_sor = db_tree_->AccessTreeElements().find(element_name);
			if (iter_sor != db_tree_->AccessTreeElements().end())
			{
				//删除原来的，保存现在的file name
				//释放空间
				if (iter_sor->second != NULL)
				{
					std::cout << "name duplicated!" << std::endl;
					delete iter_sor->second;
					iter_sor->second = object_add;
				}
			}
			else
			{
				//设置对象
				//db_tree_->current_element_idx_ = resulted_file_name;
				db_tree_->AccessTreeElements().insert(std::pair<std::string, HW::HWObject*>(element_name, object_add));
				//添加对应的item到view list 里面
				AddViewListAItem(element_name);
			}

#endif
			//在cloudcompare里面显示这些被提取出来的平面
			pcShape->showSF(false); //just in case
			pcShape->prepareDisplayForRefresh();

			ccHObject* show_obj = new ccHObject(pcShape->getName());
			show_obj->addChild(pcShape);
			m_ccRoot1->addChild(show_obj);

			active_objects_names_.emplace_back(pcShape->getName().toStdString());
			//ShowAllActivedObjects();
#endif

#if 0
			//test
			//将ccPointcloud 转 HWPointCloud 并且保存下来
			HW::HWPointCloud *hw_pc = new HW::HWPointCloud();
			for (unsigned j = 0; j < pcShape->size(); ++j)
			{
				const CCVector3* P = pcShape->getPoint(j);
				float3 pt, pn;
				pt.x = static_cast<float>(P->x);
				pt.y = static_cast<float>(P->y);
				pt.z = static_cast<float>(P->z);
				hw_pc->AddPoint(pt);
				if (hasNorms)
				{
					const CCVector3& N = pcShape->getPointNormal(j);
					pn.x = static_cast<float>(N.x);
					pn.y = static_cast<float>(N.y);
					pn.z = static_cast<float>(N.z);
					hw_pc->AddNormal(pn);
				}
			}
			std::string file_name0 = std::to_string(shape_cout) + ".ply";
			std::string file_name = "D:/vc_project_xht/back_up/test/" + file_name0;
			std::cout << "file name: " << file_name << std::endl;
			hw_pc->SavePly(file_name, HW::kBinary);
			delete hw_pc;
			//end test
#endif

			//convert detected primitive into a CC primitive type
			ccGenericPrimitive* prim = 0;

			if (shape->Identifier() == 0)
			{
				const PlanePrimitiveShape* plane = static_cast<const PlanePrimitiveShape*>(shape);
				Vec3f G = plane->Internal().getPosition();
				Vec3f N = plane->Internal().getNormal();
				Vec3f X = plane->getXDim();
				Vec3f Y = plane->getYDim();

				//we look for real plane extents
				float minX, maxX, minY, maxY;
				for (unsigned j = 0; j<shapePointsCount; ++j)
				{
					std::pair<float, float> param;
					plane->Parameters(cloud[count - 1 - j].pos, &param);
					if (j != 0)
					{
						if (minX < param.first)
							minX = param.first;
						else if (maxX > param.first)
							maxX = param.first;
						if (minY < param.second)
							minY = param.second;
						else if (maxY > param.second)
							maxY = param.second;
					}
					else
					{
						minX = maxX = param.first;
						minY = maxY = param.second;
					}
				}

				//we recenter plane (as it is not always the case!)
				float dX = maxX - minX;
				float dY = maxY - minY;
				G += X * (minX + dX / 2);
				G += Y * (minY + dY / 2);

				//we build matrix from these vectors
				ccGLMatrix glMat(CCVector3::fromArray(X.getValue()),
					CCVector3::fromArray(Y.getValue()),
					CCVector3::fromArray(N.getValue()),
					CCVector3::fromArray(G.getValue()));

				//plane primitive
				prim = new ccPlane(dX, dY, &glMat);

#if SHOW_PLANE_VIEWS
				//is there a primitive to add to part cloud?
				if (prim)
				{
					prim->applyGLTransformation_recursive();
					pcShape->addChild(prim);
					prim->setDisplay(pcShape->getDisplay());
					prim->setColor(col);
					prim->showColors(true);
					prim->setVisible(true);
				}
#else
				//将pcShape统一管理
				pcShape->addChild(prim);
				extracted_planes_objs_.emplace_back(pcShape);
#endif
				/*if (!group)
				group = new ccHObject(QString("Ransac Detected Shapes (%1)").arg(ent->getName()));
				group->addChild(pcShape);*/

				//
				ccGLMatrix tmp_matrix;
				ccBBox tmp_plane_box;
				tmp_plane_box = prim->getOwnFitBB(tmp_matrix);
				float tmp_box_dist = tmp_plane_box.getDiagNorm();

				if (max_plane_box_dist < tmp_box_dist)
					max_plane_box_dist = tmp_box_dist;

				//这对象用于对于平面对象的内存释放
				root_planes_objs_.emplace_back(prim);
			}
			count -= shapePointsCount;
			//将平面和它的点云进行对应，用于后续的平面处理(refinement)
			//planes_names.emplace_back(element_name);
		}
	}

	//------------------------------对提取出来的平面进行筛选，获取符合条件的平面，用于后续的处理---------------//

	//-----------------------------保存天花板的点云，这样可以由于后续的merge处理---------------------//
	//获取地表平面方程，一般情况地表的点云最多
	std::vector<bool> merge_for_ceil;
	for (int i = 0; i < extracted_planes_objs_.size(); ++i)
	{
		merge_for_ceil.emplace_back(false);
	}

	assert(root_planes_objs_[0]);
	ccPlane* ground_plane = static_cast<ccPlane*> (extracted_planes_objs_[0]->getFirstChild());
	CCVector3 coeff;
	PointCoordinateType d;
	ground_plane->getEquation(coeff, d);

	std::cout << "the ground equation norm is: " << coeff.norm() << std::endl;
	int highest_plane_idx = 0;
	float highest_dist_to_ground = 0.0;

	for (int i = 0; i < extracted_planes_objs_.size(); ++i)
	{
		//因为方程系数都归一化了，所以得到的应该是百分比
		if (i != 0)
		{
			ccPointCloud* fetch_pc = extracted_planes_objs_[i];
			ccPlane* plane_out_ground = static_cast<ccPlane*>(fetch_pc->getFirstChild());

			//ccPlane* plane_out_ground = static_cast<ccPlane*> (root_planes_objs_[i]);
			CCVector3 plane_coeff;
			PointCoordinateType plane_d;
			plane_out_ground->getEquation(plane_coeff, plane_d);
			CCVector3 plane_center = plane_out_ground->getCenter();

			float dist_to_ground_plane = std::abs(plane_center.dot(coeff) - d) / coeff.norm();

			std::cout << "the dist to ground plane is: " << dist_to_ground_plane << std::endl;
			//std::abs(plane_coeff.dot(coeff)) > 0.9&&
			if (dist_to_ground_plane > 2.0)
			{
				std::cout << "the merge idx is" << i << std::endl;
				merge_for_ceil[i] = true;
				if (dist_to_ground_plane > highest_dist_to_ground && fetch_pc->size() > 5000)
				{
					highest_dist_to_ground = dist_to_ground_plane;
					highest_plane_idx = i;
				}
			}
		}
	}

	ccPointCloud* fetch_highest_pc = extracted_planes_objs_[highest_plane_idx];
	ccPlane* highest_plane_outside = static_cast<ccPlane*>(fetch_highest_pc->getFirstChild());
	CCVector3 higest_plane_coeff;
	PointCoordinateType highest_plane_d;
	highest_plane_outside->getEquation(higest_plane_coeff, highest_plane_d);

	//筛选出垂直于地面的平面
	for (int i = 0; i < extracted_planes_objs_.size(); ++i)
	{
		//方程稀疏都归一化了
		if (i != 0 || !merge_for_ceil[i] || i != highest_plane_idx)
		{
			ccPointCloud* fetch_pc = extracted_planes_objs_[i];
			ccPlane* plane_outside = static_cast<ccPlane*>(fetch_pc->getFirstChild());
			CCVector3 plane_coeff;
			PointCoordinateType plane_d;
			plane_outside->getEquation(plane_coeff, plane_d);

			if (std::abs(plane_coeff.dot(higest_plane_coeff)) > 0.3)
				continue;

			int near_ceil_count = 0;
			for (int j = 0; j < fetch_pc->size(); ++j)
			{
				const CCVector3* point = fetch_pc->getPoint(j);

				float dist_to_ceil_plane = std::abs(point->dot(higest_plane_coeff) - highest_plane_d) / higest_plane_coeff.norm();
				if (dist_to_ceil_plane < 1.0)
				{
					near_ceil_count++;
				}
			}

			std::cout << "the idx: " << i << "ceill count is: " << near_ceil_count << std::endl;

			//离天花板比较近的点，同时和天花板平行的平面
			if (near_ceil_count > 50)
			{
				merge_for_ceil[i] = true;
			}

		}
	}

	//-----------------------------完成保存天花板的点云，这样可以由于后续的merge处理---------------------//
	for (int i = 0; i < extracted_planes_objs_.size(); ++i)
	{
		//tackle the merge plane and removed obj
		ccPointCloud* fetch_pc = extracted_planes_objs_[i];
		ccPlane* prim = 0;
		prim = static_cast<ccPlane*>(extracted_planes_objs_[i]->getFirstChild());

		ccGLMatrix matrix_plane;
		ccBBox plane_box;
		plane_box = prim->getOwnFitBB(matrix_plane);
		//getOwnFitBB
		float plane_box_dist = plane_box.getDiagNorm();

		float xy_ratio = 1.0f;
		float plane_box_x = plane_box.maxCorner().x - plane_box.minCorner().x;
		float plane_box_y = plane_box.maxCorner().y - plane_box.minCorner().y;

		if (std::abs(plane_box_x) > 1e-6 || std::abs(plane_box_y) > 1e-6)
		{
			if (std::abs(plane_box_y) > std::abs(plane_box_x))
				xy_ratio = std::abs(plane_box_x) / std::abs(plane_box_y);
			else
				xy_ratio = std::abs(plane_box_y) / std::abs(plane_box_x);
		}

		float plane_area = plane_box_x * plane_box_y;

		std::cout << "the xy_ratio is: " << xy_ratio << std::endl;

		//float check_flag_value = plane_box_dist / max_plane_box_dist * 0.3 + shapePointsCount / count * 0.7;

		float check_flag_value = plane_box_dist / max_plane_box_dist;

		bool merge_flag = false;

		/*if (shapePointsCount > 0.01*count || plane_box_dist > 5.5f)
		merge_flag = true;*/
		int shape_points_count = fetch_pc->size();
		//merge_flag = true;
		if (shape_points_count > HW::HWParams::getInstance().plane_params_.plane_min_pnt_num_merge_)
			//if (shape_points_count > HW::HWParams::getInstance().plane_params_.plane_min_pnt_num_merge_
			/*|| (xy_ratio < 10 * HW::HWParams::getInstance().plane_params_.xy_ratio_merge_ && plane_area > HW::HWParams::getInstance().plane_params_.min_area__merge_)
			|| (xy_ratio < HW::HWParams::getInstance().plane_params_.xy_ratio_merge_ && shape_points_count > 2 * ransacOptions.m_minSupport)
			|| merge_for_ceil[i])*/
			merge_flag = true;

		bool saveNormals = fetch_pc->hasNormals();
		bool saveColors = fetch_pc->hasColors();

		//random color
		ccColor::Rgb col = ccColor::Generator::Random();

		for (unsigned j = 0; j < shape_points_count; ++j)
		{
			//add vertex pos to merge pointcloud
			if (merge_flag)
				pcMerge->addPoint(*fetch_pc->getPoint(j));
			else
				pcRemoved->addPoint(*fetch_pc->getPoint(j));

			if (saveNormals)
			{
				if (merge_flag)
					pcMerge->addNorm(fetch_pc->getPointNormal(j));
				else
					pcRemoved->addNorm(fetch_pc->getPointNormal(j));
			}

			if (saveColors)
			{
				if (merge_flag)
					pcMerge->addRGBColor(col);
				else
					pcRemoved->addRGBColor(col);
			}
		}
		//end tackle the merge plane and removed obj
	}
	//-----------------------------------------完成对提取出来的平面筛选-------------------------------------//

	//保存对应
	//scene_to_planes.insert(std::make_pair(extracted_pc_name, planes_names));
	//scene_to_planes_.emplace_back(std::make_pair(extracted_pc_name, planes_names));
	//保存剩余的顶点
	std::cout << "the remaining count is: " << count << std::endl;

	////system("pause");
	ccColor::Rgb tmp_color(255, 255, 255);
	for (int i = 0; i < count; ++i)
	{
		pcRemoved->addPoint(CCVector3::fromArray(cloud[count - 1 - i].pos));
		pcRemoved->addNorm(CCVector3::fromArray(cloud[count - 1 - i].normal));
		pcRemoved->addRGBColor(tmp_color);
#if 0
		pcMerge->addPoint(CCVector3::fromArray(cloud[count - 1 - i].pos));
		pcMerge->addNorm(CCVector3::fromArray(cloud[count - 1 - i].normal));
		pcMerge->addRGBColor(tmp_color);
#endif
	}
	//pcShape->setRGBColor(col);

	pcMerge->showColors(true);
	pcMerge->showNormals(true);
	pcMerge->setVisible(true);
	pcMerge->setGlobalShift(globalShift);
	pcMerge->setGlobalScale(globalScale);

	CopyCCPCToHWPC(pcMerge);
	AddCCObjectToWeidgetList(pcMerge);
	pcMerge->showSF(false); //just in case
	pcMerge->prepareDisplayForRefresh();

	//把pcMerge放入m_ccRoot1中
	ccHObject* show_merge_obj = new ccHObject(pcMerge->getName());
	show_merge_obj->addChild(pcMerge);
	m_ccRoot1->addChild(show_merge_obj);
	active_objects_names_.emplace_back(pcMerge->getName().toStdString());

#if USING_REMOVED_OBJ
	pcRemoved->showColors(true);
	pcRemoved->showNormals(true);
	pcRemoved->setVisible(true);
	pcRemoved->setGlobalShift(globalShift);
	pcRemoved->setGlobalScale(globalScale);

	CopyCCPCToHWPC(pcRemoved);
	AddCCObjectToWeidgetList(pcRemoved);
	pcRemoved->showSF(false); //just in case
	pcRemoved->prepareDisplayForRefresh();

	//把pcMerge放入m_ccRoot1中
	ccHObject* show_removed_obj = new ccHObject(pcRemoved->getName());
	show_removed_obj->addChild(pcRemoved);
	m_ccRoot1->addChild(show_removed_obj);
	active_objects_names_.emplace_back(pcRemoved->getName().toStdString());
#endif

	//
	ShowAllActivedObjects();
	std::cout << "end extract plane" << std::endl;

#if SHOW_PLANE_VIEWS

#else
	//处理释放plane
	for (int i = 0; i < root_planes_objs_.size(); ++i)
	{
		if (root_planes_objs_[i])
			delete root_planes_objs_[i];
	}
#endif
	//system("pause");
}
#endif

#if 1
void MainWindow::DoExtractPlaneAction()
{
	float dist2ceil = HW::HWParams::getInstance().plane_params_.ceil_height_;

	printf("run extract plane\n");

	//convert selected ccPointCloud to pointcloud
	ccPointCloud* pc;
	//获取被选中的点云的名字
	std::string current_selected_name = db_tree_->current_element_idx_;
	//std::string 
	for (int i = 0; i < m_ccRoot1->getChildrenNumber(); ++i)
	{
		//
		if (m_ccRoot1->getChild(i)->getName() == QString(current_selected_name.data()))
		{
			pc = static_cast<ccPointCloud*>(m_ccRoot1->getChild(i)->getFirstChild());
			break;
		}
	}

	if (!pc)
	{
		printf("select no element!\n");
		return;
	}

	//system("pause");
	//ccPointCloud* pc = static_cast<ccPointCloud*>(m_ccRoot1->getFirstChild()->getFirstChild());
	//std::cout << "get the pc name: " << pc->getName().toStdString() << std::endl;
	//system("pause");
	//input cloud
	unsigned count = pc->size();
	std::cout << "pc: " << count << std::endl;
	std::string extracted_pc_name = pc->getName().toStdString();
	//std::cout << "pc name: " << extracted_pc_name << std::endl;

	//system("pause");

	bool hasNorms = pc->hasNormals();
	CCVector3 bbMin, bbMax;
	pc->getBoundingBox(bbMin, bbMax);
	const CCVector3d& globalShift = pc->getGlobalShift();
	double globalScale = pc->getGlobalScale();
	float scene_height = bbMax.z - bbMin.z;
	//const float scale = cloud.getScale();

	//std::cout << "11111111" << std::endl;

	//Spc->com
	//Convert CC point cloud to RANSAC_SD type
	PtCloud<float> cloud, cloud_ceil;
	std::cout << "start Convert CC point cloud to RANSAC_SD type" << std::endl;
	//test
	if (hasNorms)
	{
		std::cerr << "point cloud has normals.........." << std::endl << std::endl;
	}
	//end test
	for (unsigned i = 0; i < count; ++i)
	{
		const CCVector3* P = pc->getPoint(i);
		if (hasNorms) {
			const CCVector3& Pn = pc->getPointNormal(i);
			//PtCloud<float>::PtData PT(static_cast<float>(P->x), static_cast<float>(P->y), static_cast<float>(P->z));
			//if (P->z < -0.05f)
				//continue;
// 设置天花板 @zk del
#if 0
			if (P->z < dist2ceil)
				cloud.pts.push_back(PtCloud<float>::PtData(P->x, P->y, P->z, Pn.x, Pn.y, Pn.z));
			else
				cloud_ceil.pts.push_back(PtCloud<float>::PtData(P->x, P->y, P->z, Pn.x, Pn.y, Pn.z));
#else
            cloud.pts.push_back(PtCloud<float>::PtData(P->x, P->y, P->z, Pn.x, Pn.y, Pn.z));
#endif
		}
		else {
// 设置天花板 @zk del
#if 0
			if (P->z < dist2ceil)
				cloud.pts.push_back(PtCloud<float>::PtData(P->x, P->y, P->z));
			else
				cloud_ceil.pts.push_back(PtCloud<float>::PtData(P->x, P->y, P->z));
#else
        cloud.pts.push_back(PtCloud<float>::PtData(P->x, P->y, P->z));
#endif
		}
	}
	std::cout << "Convert CC point cloud to RANSAC_SD type" << std::endl;
	count -= cloud_ceil.pts.size();

	std::cout << "cloud_ceil size: " << cloud_ceil.pts.size() << std::endl;
	std::cout << "cloud without ceil size: " << count << std::endl;

	const int k = (int)this->ui.KnnSpinBox->value();
	std::vector<PCAInfo> pcaInfos;
	PCAFunctions pcaer;
	//fixme
	std::cout << "Press any key to begin pca" << std::endl;
	pcaer.PCA(cloud, k, pcaInfos);
	std::wcout << "pca done" << std::endl;
	std::vector<std::vector<int>> clusters;
	std::vector<int> cluster_removed;
	float theta = HW::HWParams::getInstance().plane_params_.max_normal_deviation_ / 180.0 * CV_PI;
	int RMin = HW::HWParams::getInstance().plane_params_.plane_min_pnt_num_;  // minimal number of points per cluster	
	
	//std::cout << "33333333333" << std::endl;

	//time
	clock_t start_time, end_time;
	start_time = clock();

	PointGrowAngleDis segmenter(theta, RMin);
	segmenter.setData(cloud, pcaInfos);
	segmenter.run(clusters, cluster_removed);

	end_time = clock();
	std::cout << "extract time: " << (double)(end_time - start_time) / CLOCKS_PER_SEC << "s" << std::endl;
	std::vector<Eigen::Vector3f> coeff_vec = segmenter.getCoeffs();
	std::vector<Eigen::Vector3f> mean_vec = segmenter.getMeans();
	std::vector<float> d_vec = segmenter.getDs();
	std::vector<Eigen::Vector3f> boxMin_vec = segmenter.getBoxMins();
	std::vector<Eigen::Vector3f> boxMax_vec = segmenter.getBoxMaxs();

	std::string merge_element_name = extracted_pc_name.substr(0, extracted_pc_name.find_last_of(".")) + "_merge.ply";
	printf("the merge plane name: %s\n", merge_element_name.c_str());
	HW::HWPointCloud* merge_element_add = new HW::HWPointCloud();
	merge_element_add->SetObjectType(HW::ElementType::kHWPointCloud);
	merge_element_add->SetObjectName(merge_element_name);

	//new cloud for sub-part
	ccPointCloud* pcMerge = new ccPointCloud(merge_element_name.c_str());
	pcMerge->reserveTheNormsTable();
	pcMerge->reserveTheRGBTable();

	//HWPointCloud: set the removed sub-part
	std::string removed_element_name = extracted_pc_name.substr(0, extracted_pc_name.find_last_of(".")) + "_removed.ply";
	printf("the merge plane name: %s\n", removed_element_name.c_str());
	HW::HWPointCloud* removed_element_add = new HW::HWPointCloud();
	merge_element_add->SetObjectType(HW::ElementType::kHWPointCloud);
	merge_element_add->SetObjectName(removed_element_name);

	//ccPointCloud: a copy from HWPointCloud
	ccPointCloud* pcRemoved = new ccPointCloud(removed_element_name.c_str());
	pcRemoved->reserveTheNormsTable();
	pcRemoved->reserveTheRGBTable();

	std::vector<int> planes_idx;
	std::vector<int> planes_iswide;
	planes_idx.emplace_back(0);
	//保存平面的名字，用于索引pc分割出的平面
	//std::vector<std::string> planes_names;
	float max_plane_box_dist = -1e6;

	Eigen::Vector3f groud_coeff = Eigen::Vector3f(0.0f,0.0f,1.0f);

	/*Eigen::Vector3f groud_coeff = coeff_vec[0];
	if (groud_coeff.dot(groud_coeff_ideal) < 0.98)
		groud_coeff = groud_coeff_ideal;*/
	std::cout << "groud_coeff: " << groud_coeff.transpose() << "\n";
	std::cout << "the ground equation norm is: " << groud_coeff.norm() << std::endl;

	//-----------------------------完成保存天花板的点云，这样可以由于后续的merge处理---------------------//
	std::vector<int> planes_group(clusters.size(), -1);
	std::vector<int> group_type(clusters.size(), 0);
	int group_cnt = 0;
	std::cout << "start clusters" << std::endl;
	for (int i = 0; i < clusters.size(); ++i)
	{
		if (i % 1000) std::cout << i << std::endl;
		Eigen::Vector3f mean = mean_vec[i];
		Eigen::Vector3f coeff = coeff_vec[i];
		//std::cout << "coeff: " << coeff.transpose() << "\n";
		float d = d_vec[i];
		Eigen::Vector3f boxMin = boxMin_vec[i];
		Eigen::Vector3f boxMax = boxMax_vec[i];

		bool merge_flag = false;

		/*if (shapePointsCount > 0.01*count || plane_box_dist > 5.5f)
		merge_flag = true;*/
		int shape_points_count = clusters[i].size();
		//std::cout << "shape_points_count: " << shape_points_count << "\n";
		//merge_flag = true;
		float plane_height = boxMax.z() - boxMin.z();
		//std::cout << "plane_height: " << plane_height << "\n";
		float plane_width = sqrt(pow(boxMax.x() - boxMin.x(), 2) + pow(boxMax.y() - boxMin.y(), 2));
		//std::cout << "plane_width: " << plane_width << "
		float plane_merge_height = HW::HWParams::getInstance().plane_params_.plane_merge_height_;
		float min_plane_width = HW::HWParams::getInstance().plane_params_.min_plane_width_;
		//std::cout << "min_plane_width: " << min_plane_width << "\n";
		if (shape_points_count > HW::HWParams::getInstance().plane_params_.plane_min_pnt_num_merge_||((plane_height > plane_merge_height || (boxMax.z()+boxMin.z())/2 > plane_merge_height)&& plane_width>min_plane_width)
			|| (boxMin.z()>plane_merge_height&&min(boxMax.x() - boxMin.x(), boxMax.y() - boxMin.y())>min_plane_width)) {//|| (plane_height > 2 * scene_height / 3 && abs(plane_coeff.dot(coeff)) < 0.1f&&plane_width > 0.1f)) {
																									  //merge_flag = true;
			planes_group[i] = group_cnt++;
			group_type[i] = 1;//大平面
		}
#if 1
		else if (((plane_height > plane_merge_height) || boxMin.z() > plane_merge_height) && abs(coeff.dot(groud_coeff)) < 0.2f) {
			group_type[i] = 2;
			bool is_merged = false;	
			for (int j = 0; j < i; j++) {
				if (group_type[j] == 0)
					continue;
				Eigen::Vector3f boxMin_pre = boxMin_vec[j];
				Eigen::Vector3f	boxMax_pre = boxMax_vec[j];
				Eigen::Vector2f box_xymean = Eigen::Vector2f((boxMin.x() + boxMax.x()) / 2, (boxMin.y() + boxMax.y()) / 2);
				Eigen::Vector2f box_xymean_pre = Eigen::Vector2f((boxMin_pre.x() + boxMax_pre.x()) / 2, (boxMin_pre.y() + boxMax_pre.y()) / 2);
				Eigen::Vector3f coeff_pre= coeff_vec[j];
				float d_pre=d_vec[j];
				float dist1 = abs(mean.dot(coeff_pre) + d_pre);
				if (group_type[j] == 1 && dist1 < 0.03f && (boxMax.z() - boxMin_pre.z() >-0.05f  && boxMax_pre.z()- boxMin.z()>-0.05f)) {
				    //planes_group[i] = planes_group[j];
					group_type[i] = 0;
					is_merged = true;
					break;
				}
				else if (group_type[j] != 1 && dist1 <= 0.2f && (box_xymean - box_xymean_pre).norm() < 0.5f && (boxMax.z() - boxMin_pre.z() >-0.05f  && boxMax_pre.z() - boxMin.z()>-0.05f)){// && abs(coeff.dot(coeff_pre))>cos(4 * MATH_PI / 9)) {
					for (int k = 0; k < clusters[i].size(); k++)
						clusters[j].emplace_back(clusters[i][k]);
					segmenter.getPlaneInfo(clusters[j], boxMin_vec[j], boxMax_vec[j], coeff_vec[j], d_vec[j]);
					group_type[j] = 3;
					group_type[i] = 0;
					is_merged = true;
					break;
				}
			}
			if (!is_merged) {
				planes_group[i] = group_cnt++;
				group_type[i] = 2;
				for (int j = 0; j < i; j++) {
					if (group_type[j] !=2)
						continue;
					Eigen::Vector3f boxMin_pre = boxMin_vec[j];
					Eigen::Vector3f	boxMax_pre = boxMax_vec[j]; 
					Eigen::Vector3f mean_pre = mean_vec[j];
					Eigen::Vector3f coeff_pre = coeff_vec[j];
					float d_pre = d_vec[j];
					Eigen::Vector2f box_xymean = Eigen::Vector2f((boxMin.x() + boxMax.x()) / 2, (boxMin.y() + boxMax.y()) / 2);
					Eigen::Vector2f box_xymean_pre = Eigen::Vector2f((boxMin_pre.x() + boxMax_pre.x()) / 2, (boxMin_pre.y() + boxMax_pre.y()) / 2);
					float dist2 = abs(mean_pre.dot(coeff) + d);

					if (dist2 <= 0.2f && (box_xymean - box_xymean_pre).norm()<0.5f && (boxMax.z() > boxMin_pre.z() && boxMax_pre.z()>boxMin.z())){// && abs(coeff.dot(coeff_pre))>cos(4 * MATH_PI / 9)) {
						planes_group[j] = -1;
						for (int k = 0; k < clusters[j].size(); k++)
							clusters[i].emplace_back(clusters[j][k]);
						segmenter.getPlaneInfo(clusters[i], boxMin_vec[i], boxMax_vec[i], coeff_vec[i], d_vec[i]);
						group_type[i] = 3;
						group_type[j] = 0;
					}
				}
			}
		}
#endif
	}

	//将分割出来的cluster的点云进行排序，这样可以plane_idx，和点云的顺序一致，用于后续处理
	for (int i = 0; i < group_cnt ; i++) {
		int group_sum = 0;
		ccColor::Rgb col = ccColor::Generator::Random();
		int type;
		for (int j = 0; j < planes_group.size(); j++) {
			if (planes_group[j] == i) {
				type = group_type[j];
				if (type <= 0)
					continue;
				Eigen::Vector3f boxMin = boxMin_vec[j];
				Eigen::Vector3f boxMax = boxMax_vec[j];
				float plane_width = sqrt(pow(boxMax.x() - boxMin.x(), 2) + pow(boxMax.y() - boxMin.y(), 2));
				/*if (clusters[j].size()<500)
					continue;*/
				std::vector<int>& cluster = clusters[j];
				int shape_points_count = cluster.size();
				group_sum += shape_points_count;
				for (int k = 0; k < shape_points_count; k++) {
					PtCloud<float>::PtData pt = cloud.pts[cluster[k]];
					CCVector3 pos(pt.x, pt.y, pt.z);
					CCVector3 norm(pt.nx, pt.ny, pt.nz);
					pcMerge->addPoint(pos);
					pcMerge->addNorm(norm);
					pcMerge->addRGBColor(col);
				}
			}
		}
		if (group_sum != 0) {
			planes_idx.emplace_back(planes_idx.back() + group_sum);
			if (type == 1)
				planes_iswide.emplace_back(1);
			else planes_iswide.emplace_back(0);
		}
		//end tackle the merge plane and removed obj
	}

#if USING_EXTRACTED_PLANES
	std::vector<HW::HWPointCloud*> planes_points;
	for (int i = 0; i < clusters.size(); ++i)
	{
		ccColor::Rgb col = ccColor::Generator::Random();
		std::vector<int>& cluster = clusters[i];
		int shape_points_count = cluster.size();
		HW::HWPointCloud* tmp_points = new HW::HWPointCloud();
		for (int j = 0; j < shape_points_count; j++) {
			PtCloud<float>::PtData pt = cloud.pts[cluster[j]];
			float3 pos = make_float3(pt.x, pt.y, pt.z);
			float3 norm = make_float3(pt.nx, pt.ny, pt.nz);
			uchar3 color = make_uchar3(col.r, col.g, col.b);
			tmp_points->AddPoint(pos);
			tmp_points->AddNormal(norm);
			tmp_points->AddColor(color);
		}
		planes_points.emplace_back(tmp_points);
	}
	//save all the planes point clouds
	extracted_pc_name;
	std::string current_dir_str = HW::GetDirFromPathName(HW::GetLeftSlashPathName(extracted_pc_name));
	std::cerr << "current dir str: " << current_dir_str << std::endl;
	std::string current_next_dir_str = HW::EnsureTrailingSlashHW(current_dir_str) + "planes_points/";
	std::cerr << "current_next_dir_str : " << current_next_dir_str << std::endl;
	if (!boost::filesystem::is_directory(current_next_dir_str))
	{
		boost::filesystem::create_directory(current_next_dir_str);
	}
	else
	{
		std::cerr << "failed to create directory..." << std::endl;
	}
	for (int i = 0; i < planes_points.size(); ++i)
	{
		std::string current_path_str = current_next_dir_str + std::to_string(i) + ".ply";
		planes_points[i]->SavePly(current_path_str, HW::PlyFormat::kBinary);
	}
	//delete all the planes point clouds
	for (int i = 0; i < planes_points.size(); ++i)
	{
		if (planes_points[i])
		{
			delete planes_points[i];
			planes_points[i] = NULL;
		}
	}
#endif

	//添加天花板的点云(人为设置的)
	int shape_points_count = cloud_ceil.pts.size();
	ccColor::Rgb color = ccColor::Generator::Random();
	/*for (unsigned j = 0; j < shape_points_count; ++j)
	{
		PtCloud<float>::PtData pt = cloud_ceil.pts[j];
		CCVector3 pos(cloud_ceil.pts[j].x, cloud_ceil.pts[j].y, cloud_ceil.pts[j].z);
		CCVector3 norm(cloud_ceil.pts[j].nx, cloud_ceil.pts[j].ny, cloud_ceil.pts[j].nz);
		pcMerge->addPoint(pos);
		pcMerge->addNorm(norm);
		pcMerge->addRGBColor(color);
	}*/
	//planes_idx.emplace_back(planes_idx.back() + shape_points_count);
	
	for (int j = 0; j < planes_group.size(); j++) {
		if (planes_group[j] == -1) {		//表示被删选的平面	
			std::vector<int> cluster = clusters[j];
			int shape_points_count = cluster.size();
			//ccColor::Rgb col = ccColor::Generator::Random();
			ccColor::Rgb col = ccColor::black;
			for (int k = 0; k < shape_points_count; k++) {
				PtCloud<float>::PtData pt = cloud.pts[cluster[k]];
				CCVector3 pos(cloud.pts[cluster[k]].x, cloud.pts[cluster[k]].y, cloud.pts[cluster[k]].z);
				CCVector3 norm(cloud.pts[cluster[k]].nx, cloud.pts[cluster[k]].ny, cloud.pts[cluster[k]].nz);
				pcRemoved->addPoint(pos);
				pcRemoved->addNorm(norm);
				pcRemoved->addRGBColor(col);
			}
		}
	}

	//ccColor::Rgb col = ccColor::Generator::Random();
	ccColor::Rgb col = ccColor::black;
	for (int j = 0; j < cluster_removed.size(); j++) {	
		PtCloud<float>::PtData pt = cloud.pts[cluster_removed[j]];
		CCVector3 pos(cloud.pts[cluster_removed[j]].x, cloud.pts[cluster_removed[j]].y, cloud.pts[cluster_removed[j]].z);
		CCVector3 norm(cloud.pts[cluster_removed[j]].nx, cloud.pts[cluster_removed[j]].ny, cloud.pts[cluster_removed[j]].nz);
		pcRemoved->addPoint(pos);
		pcRemoved->addNorm(norm);
		pcRemoved->addRGBColor(col);
		//pcMerge->addPoint(pos);
		//pcMerge->addNorm(norm);
		//pcMerge->addRGBColor(col);
	}
	
	//-----------------------------------------完成对提取出来的平面筛选-------------------------------------//

	//保存对应
	//保存剩余的顶点
//	std::cout << "the remaining count is: " << count << std::endl;
//
//	////system("pause");
//	ccColor::Rgb tmp_color(255, 255, 255);
//	for (int i = 0; i < count; ++i)
//	{
//		pcRemoved->addPoint(CCVector3::fromArray(cloud[count - 1 - i].pos));
//		pcRemoved->addNorm(CCVector3::fromArray(cloud[count - 1 - i].normal));
//		pcRemoved->addRGBColor(tmp_color);
//#if 0
//		pcMerge->addPoint(CCVector3::fromArray(cloud[count - 1 - i].pos));
//		pcMerge->addNorm(CCVector3::fromArray(cloud[count - 1 - i].normal));
//		pcMerge->addRGBColor(tmp_color);
//#endif
//	}
	//pcShape->setRGBColor(col);

	pcMerge->showColors(true);
	//pcMerge->showNormals(true);
	pcMerge->setVisible(true);
	pcMerge->setGlobalShift(globalShift);
	pcMerge->setGlobalScale(globalScale);

	CopyCCPCToHWPC(pcMerge, planes_idx, planes_iswide);
	AddCCObjectToWeidgetList(pcMerge);
	pcMerge->showSF(false); //just in case
	pcMerge->prepareDisplayForRefresh();

	//把pcMerge放入m_ccRoot1中
	ccHObject* show_merge_obj = new ccHObject(pcMerge->getName());
	show_merge_obj->addChild(pcMerge);
	m_ccRoot1->addChild(show_merge_obj);
	active_objects_names_.emplace_back(pcMerge->getName().toStdString());

#if USING_REMOVED_OBJ
	pcRemoved->showColors(true);
	pcRemoved->showNormals(true);
	pcRemoved->setVisible(true);
	pcRemoved->setGlobalShift(globalShift);
	pcRemoved->setGlobalScale(globalScale);

	CopyCCPCToHWPC(pcRemoved);
	AddCCObjectToWeidgetList(pcRemoved);
	pcRemoved->showSF(false); //just in case
	pcRemoved->prepareDisplayForRefresh();

	//把pcMerge放入m_ccRoot1中
	ccHObject* show_removed_obj = new ccHObject(pcRemoved->getName());
	show_removed_obj->addChild(pcRemoved);
	m_ccRoot1->addChild(show_removed_obj);
	active_objects_names_.emplace_back(pcRemoved->getName().toStdString());
#endif

	//HWPointCloud: set the removed noise sub-part
	std::string noise_element_removed_name = removed_element_name.substr(0, extracted_pc_name.find_last_of(".")) + "_noise.ply";
	printf("the noise_element_removed_name plane name: %s\n", noise_element_removed_name.c_str());
	/*HW::HWPointCloud* removed_element_noise_add = new HW::HWPointCloud();
	removed_element_noise_add->SetObjectType(HW::ElementType::kHWPointCloud);
	removed_element_noise_add->SetObjectName(noise_element_removed_name);*/
	//ccPointCloud: a copy from HWPointCloud
	ccPointCloud* pcRemovednoise = new ccPointCloud(noise_element_removed_name.c_str());
	pcRemovednoise->reserveTheNormsTable();
	pcRemovednoise->reserveTheRGBTable();

	//HWPointCloud: set the removed back sub-part
	std::string back_element_removed_name = removed_element_name.substr(0, extracted_pc_name.find_last_of(".")) + "_objs.ply";
	printf("the back_element_removed_name plane name: %s\n", back_element_removed_name.c_str());
	/*HW::HWPointCloud* removed_element_back_add = new HW::HWPointCloud();
	removed_element_back_add->SetObjectType(HW::ElementType::kHWPointCloud);
	removed_element_back_add->SetObjectName(back_element_removed_name);*/
	//ccPointCloud: a copy from HWPointCloud
	ccPointCloud* pcRemovedback = new ccPointCloud(back_element_removed_name.c_str());
	pcRemovedback->reserveTheNormsTable();
	pcRemovedback->reserveTheRGBTable();

	std::cout << "end extract plane" << std::endl;
	float disttmp = 0.01;
	std::vector<bool> removed_flag;
	for (int i = 0; i < pcRemoved->size(); ++i)
	{
		removed_flag.emplace_back(true);
	}
	DenoiseCCPCFromCCPC(pcMerge, pcRemoved, disttmp, removed_flag);
	
	for (int i = 0; i < pcRemoved->size(); ++i)
	{
		if (removed_flag[i])
		{
			const CCVector3* tmpp = pcRemoved->getPoint(i);
			pcRemovednoise->addPoint(*tmpp);
			if (pcRemoved->hasNormals())
			{
				const CCVector3 tmppn = pcRemoved->getPointNormal(i);
				pcRemovednoise->addNorm(tmppn);
			}
		}
		else
		{
			const CCVector3* tmpp = pcRemoved->getPoint(i);
			pcRemovedback->addPoint(*tmpp);
			if (pcRemoved->hasNormals())
			{
				const CCVector3 tmppn = pcRemoved->getPointNormal(i);
				pcRemovedback->addNorm(tmppn);
			}
		}
	}

	pcRemovednoise->showColors(true);
	pcRemovednoise->showNormals(true);
	pcRemovednoise->setVisible(true);
	pcRemovednoise->setGlobalShift(globalShift);
	pcRemovednoise->setGlobalScale(globalScale);
	pcRemovedback->showColors(true);
	pcRemovedback->showNormals(true);
	pcRemovedback->setVisible(true);
	pcRemovedback->setGlobalShift(globalShift);
	pcRemovedback->setGlobalScale(globalScale);
	AddCCObjectToWeidgetList(pcRemovednoise);
	pcRemovednoise->showSF(false); //just in case
	pcRemovednoise->prepareDisplayForRefresh();
	AddCCObjectToWeidgetList(pcRemovedback);
	pcRemovedback->showSF(false); //just in case
	pcRemovedback->prepareDisplayForRefresh();

	//把pcMerge放入m_ccRoot1中
	ccHObject* show_Removednoise_obj = new ccHObject(pcRemovednoise->getName());
	show_Removednoise_obj->addChild(pcRemovednoise);
	m_ccRoot1->addChild(show_Removednoise_obj);
	active_objects_names_.emplace_back(pcRemovednoise->getName().toStdString());

	ccHObject* show_Removedback_obj = new ccHObject(pcRemovedback->getName());
	show_Removedback_obj->addChild(pcRemovedback);
	m_ccRoot1->addChild(show_Removedback_obj);
	active_objects_names_.emplace_back(pcRemovedback->getName().toStdString());

	CopyCCPCToHWPC(pcRemovednoise);
	CopyCCPCToHWPC(pcRemovedback);
	//SaveCCPCIntoObjFile()
	//SaveHWPCIntoObjFile(removed_pnts, removed_pnts_normals, tmppthremoved);
	//SaveHWPCIntoObjFile(remained_pnts, remained_pnts_normals, tmppthback);

	//
	ShowAllActivedObjects();
	std::cout << "end extract plane" << std::endl;

	//system("pause");
}
#endif
//去除物体
void MainWindow::RemoveTheObjFromSceneAction()
{
	float dist_threshold = 0.1;

	if (scene_to_planes_.empty())
	{
		printf("no scene selected!\n");
		return;
	}

	std::cout << "test test" << std::endl;
	//system("pause");

	//寻找被选中的那些被提取出来的平面
	//std::map<std::string, std::vector<std::string> >::iterator iter;
	std::vector<std::pair<std::string, std::vector<std::string>> >::iterator iter;

	for (iter = scene_to_planes_.begin(); iter != scene_to_planes_.end(); ++iter)
	{	
		//获取整个场景的点云
		std::string pc_name = iter->first;
		ccPointCloud* pc;

		std::cout << "the pc name is: " << pc_name << std::endl;
		//system("pause");

		pc = static_cast<ccPointCloud*>(GetObjectByStrName(pc_name));

		std::cout << "the pc size is: " << pc->size() << std::endl;
		//system("pause");

		if (!pc)
		{
			printf("select no point cloud obj!\n");
			return;
		}

		for (int i = 0; i < iter->second.size(); ++i)
		{
			//获取每一个平面的名字
			std::string obj_plane_name = iter->second[i];
			//从global 中找到平面对象
			std::cout << "the obj plane name is: " << obj_plane_name << std::endl;
			//system("pause");

			ccPointCloud* plane_pc;
			//获取被选中的点云的名字
			//std::string current_selected_name = db_tree_->current_element_idx_;
			//std::string 
			for (int j = 0; j < m_ccRoot1->getChildrenNumber(); ++j)
			{
				//
				if (m_ccRoot1->getChild(j)->getName() == QString(obj_plane_name.data()))
				{
					plane_pc = static_cast<ccPointCloud*>(m_ccRoot1->getChild(j)->getFirstChild());
					break;
				}
			}

			//获取平面方程
			if (plane_pc)
			{
				//
				ccPlane* plane_prim = static_cast<ccPlane*>(plane_pc->getChild(0));
				//std::cout << "plane 1.........." << std::endl;
				//system("pause");

				//test
				HW::HWPlane* hw_plane_prim = new HW::HWPlane(plane_prim);

				//std::cout << "plane 2..........." << std::endl;
				//system("pause");
				//end test

				CCVector3f coeff;
				PointCoordinateType d;
				
				CCVector3f plane_center = plane_prim->getCenter();
				CCVector3f plane_normal = plane_prim->getNormal();

				const float d_value = plane_center.dot(plane_normal);

				//ax + by + cz + d = 0;
				plane_prim->getEquation(coeff, d);

				std::cout << "the normal is: " << plane_normal[0] << " " << plane_normal[1] << " " << plane_normal[2] << std::endl;
				std::cout << "the d value is: " << d_value << std::endl;

				std::cout << "the coeff is: " << coeff[0] << " " << coeff[1] << " " << coeff[2] << std::endl;
				std::cout << "d is: " << d << std::endl;

#if 0
				//在平面上sample 三维顶点
				int idx = 0;
				float max_x = -10000;
				for (int k = 0; k < 3; ++k)
				{
					if (max_x > std::abs(coeff[k]))
					{
						idx = k;
					}
				}

				std::cout << "the idx is: " << idx << std::endl;

				std::vector<float3> plane_sample;

				for (int m = -500; m < 500; ++m)
				{
					for (int n = -500; n < 500; ++n)
					{
						if (idx == 0)
						{
							float x = (d_value - coeff[1] * (plane_center[1] + 0.01*m)
								- coeff[2] * (plane_center[2] + 0.01*n)) / coeff[0];
							float y = plane_center[1] + 0.01*m;
							float z = plane_center[2] + 0.01*n;
							float3 sample_pnt = make_float3(x, y, z);

							plane_sample.emplace_back(sample_pnt);
						}
						if (idx == 1)
						{
							float x = plane_center[0] + 0.01 * m;
							float y = (d_value - coeff[0] * (plane_center[0] + 0.01*m)
								- coeff[2] * (plane_center[2] + 0.01*n)) / coeff[1];
							float z = plane_center[2] + 0.01*n;
							float3 sample_pnt = make_float3(x, y, z);

							plane_sample.emplace_back(sample_pnt);
						}
						if (idx == 2)
						{
							float x = plane_center[0] + 0.01 * m;
							float y = plane_center[1] + 0.01 * n;
							float z = (d_value - coeff[0] * (plane_center[0] + 0.01*m)
								- coeff[1] * (plane_center[1] + 0.01*n)) / coeff[2];
							
							float3 sample_pnt = make_float3(x, y, z);

							plane_sample.emplace_back(sample_pnt);
						}
					}
				}

				std::string sample_path = "D:\\vc_project_xht\\room_pointcloud_plane_111" + std::to_string(i) + ".ply";

				std::cout <<"sample path is:"<< sample_path << std::endl;
				HW::HWPointCloud *sample_pc = new HW::HWPointCloud();
				for (int j = 0; j < plane_sample.size(); ++j)
				{
					sample_pc->AddPoint(make_float3(plane_sample[j].x, plane_sample[j].y, plane_sample[j].z));
				}
				sample_pc->SetObjectName(sample_path);
				sample_pc->SavePly(sample_path, HW::kBinary);
				delete sample_pc;
				system("pause");

#endif
				//获取关联的点云
				plane_prim->setAssociatedCloud(pc);
				std::cout << "end set associated cloud" << std::endl;

				//ccPointCloud* tmp = static_cast<ccPointCloud*>(plane_prim->getAssociatedCloud());
				//把平面的顶点放到平面方程中去，改一下
				for (int j = 0; j < pc->size(); ++j)
				{
					//
					if (!(j % 100000))
					{
						std::cout << "j: " << j << std::endl;
					}

					const CCVector3f* p_point = pc->getPoint(j);
					const CCVector3f p_normal = pc->getPointNormal(j);

				/*	if (j == 26)
					{
						std::cout << "end featch the 26 point" << std::endl;
						system("pause");
					}*/

					float numerator = coeff[0] * p_point->x + coeff[1] * p_point->y + coeff[2] * p_point->z - d;
					
					/*if (j == 26)
					{
						std::cout << "the numerator is: " << numerator << std::endl;
						std::cout << "the coeff norm is: " << coeff.norm() << std::endl;
						system("pause");
					}*/

					//std::cout << "the numerator is: " << numerator << std::endl;
					//std::cout << "the coeff norm is: " << coeff.norm() << std::endl;
					//system("pause");

					if (coeff.norm() < 1e-4)
						continue;

					float dist = std::abs(numerator) / coeff.norm();
					/*if (j == 26)
					{
						std::cout << "the dist is: " << dist << std::endl;
						system("pause");
					}*/
					//std::cout << "the dist is: " << dist << std::endl;
					//system("pause");

					//
					if (dist < dist_threshold)
					{
						hw_plane_prim->AddPlanePntIdxFromAssociatedPC(j);

						if (!(j % 100000))
						{
							std::cout << "end save j point: " << j << std::endl;
						}

						//test
						hw_plane_prim->AddPlanePnt(p_point->x, p_point->y, p_point->z);
						hw_plane_prim->AddPlanePntNormal(p_normal[0], p_normal[1], p_normal[2]);
						//std::cout << "end save j point: " << j << std::endl;
						//end test
					}
				}

				std::cout << "end add "<< i <<"th plane point" << std::endl;
				//system("pause");
				//test：测试出来的每一个平面。存出来看看。
				//处理每一个plan prim

				//test
				const std::vector<float3> plane_pos = hw_plane_prim->GetPlanePnts();
				HW::HWPointCloud *tmp_cloud = new HW::HWPointCloud();

				std::string plane_path = "D:\\vc_project_xht\\room_pointcloud_plane_" + std::to_string(i) + ".ply";

				tmp_cloud->SetObjectName(plane_path);

				//获取voxel，然后保存
				for (int k = 0; k < plane_pos.size(); ++k)
				{
					//
					//const CCVector3* tmp_pnt = theCloud->getPoint(propagated_pnt_idx[i]);
					//const CCVector3* tmp_pnt_normal = pi.entity->get
					tmp_cloud->AddPoint(make_float3(plane_pos[k].x, plane_pos[k].y, plane_pos[k].z));
					//tmp_cloud->AddNormal(make_float3(tmp))
				}

				tmp_cloud->SavePly(tmp_cloud->GetObjectName(), HW::kBinary);
				delete tmp_cloud;
				std::cout << "end save the plane pnts\n" << std::endl;
				//end test
				//system("pause");
			}
		}
	}
}

void MainWindow::CancelExtractPlaneAction()
{
	printf("Cancel Extract plane...\n");
}


//---------------------Contruction---------------------//
void MainWindow::SetConstructionVoxelLengthAction()
{
	//printf("set Construction Voxel Length...\n");
	float vol_voxel_length = (float)this->ui.doubleSpinBoxVoxelLength->value();
	printf("Set Construction Voxel length: %f\n", vol_voxel_length);
	HW::HWParams::getInstance().construction_params_.voxel_length_ = vol_voxel_length;
}

void MainWindow::SetContructionVolumeTruncAction()
{
	float vol_trunc_length = (float)this->ui.doubleSpinBoxTruncLength->value();
	printf("Set Construction volume trunc length: %f\n", vol_trunc_length);
	HW::HWParams::getInstance().construction_params_.volume_trunc_length_ = vol_trunc_length;
}

void MainWindow::SetConstructionVolumeX()
{
	int volume_resolution_x = (int)this->ui.spinBox_x->value();
	printf("Set Construction volume x: %d\n", volume_resolution_x);
	HW::HWParams::getInstance().construction_params_.volume_dimensions_.x = volume_resolution_x;
}

void MainWindow::SetConstructionVolumeY()
{
	int volume_resolution_y = (int)this->ui.spinBox_y->value();
	printf("Set Construction volume y: %d\n", volume_resolution_y);
	HW::HWParams::getInstance().construction_params_.volume_dimensions_.x = volume_resolution_y;
}

void MainWindow::SetConstructionVolumeZ()
{
	int volume_resolution_z = (int)this->ui.spinBox_z->value();
	printf("Set Construction volume z: %d\n", volume_resolution_z);
	HW::HWParams::getInstance().construction_params_.volume_dimensions_.x = volume_resolution_z;
}

void MainWindow::DoConstructionAction()
{
	//ReadPly("D:/room_sketch/data/huawei_data/zheda_pointcloud/zheda/PPT/pointcloud_mesh.ply", false);
#if 1
	printf("run construction...\n");
	printf("construction: %s\n", db_tree_->current_element_idx_.c_str());
	if (db_tree_->current_element_idx_.empty())
	{
		printf("invalid select object...\n");
		return;
	}

	if (db_tree_->AccessTreeElements().empty())
	{
		printf("empty object...\n");
		return;
	}

	std::map<std::string, HW::HWObject*>::iterator iter = db_tree_->AccessTreeElements().find(db_tree_->current_element_idx_);
	if (iter != db_tree_->AccessTreeElements().end())
	{
		HW::HWPCLFunctor* construct_process = new HW::HWConstruction();
		processes_vec_.emplace_back(construct_process);
		HW::HWObject* out_mesh_result;
		construct_process->Process(iter->second, out_mesh_result);
		
		////test
		//if (sor_process == processes_vec_[0])
		//{
		//	printf("assign success...\n");
		//	return;
		//}
		////end test

		//生成mesh后，将这个mesh保存到db_tree中,保存指针
		//生成唯一的对象，用路径名来识别它的唯一性
		std::string resulted_file_name = iter->first.substr(0, iter->first.find_last_of(".")) + "_mesh.ply";
		printf("reuslted_file_name: %s\n", resulted_file_name.c_str());
		//system("pause");
		//out_mesh_result->SetObjectName(resulted_file_name);
		//printf("reuslted_file_name: %s\n", resulted_file_name.c_str());
		//system("pause");
		std::map<std::string, HW::HWObject*>::iterator iter_consturction = db_tree_->AccessTreeElements().find(resulted_file_name);
		if (iter_consturction != db_tree_->AccessTreeElements().end())
		{
			//删除原来的，保存现在的file name
			//释放空间
			if (iter_consturction->second != NULL)
			{
				delete iter_consturction->second;
				iter_consturction->second = construct_process->resulted_element_;
				return;
			}
		}
		else
		{
			//设置对象
			db_tree_->current_element_idx_ = resulted_file_name;
			construct_process->resulted_element_->SetObjectName(db_tree_->current_element_idx_);
			db_tree_->AccessTreeElements().insert(std::pair<std::string, HW::HWObject*>(db_tree_->current_element_idx_, construct_process->resulted_element_));
			//iter_consturction->second->SetObjectName(db_tree_->current_element_idx_);
			//添加对应的item到view list 里面
			//AddViewListAItem(db_tree_->current_element_idx_);
		}

		//test
		//construct_process->resulted_element_->SavePly(db_tree_->current_element_idx_, HW::kBinary);
		//end test
	}

	printf("end construction...\n");
	std::cout << "after construct operation, the current element idx is: " << db_tree_->current_element_idx_ << std::endl;
	
	//opengl
	CopyHWObjToCCObj(db_tree_->current_element_idx_);
	AddViewListAItem(db_tree_->current_element_idx_);
	active_objects_names_.emplace_back(db_tree_->current_element_idx_);

	ShowAllActivedObjects();
	//ShowCurrentObject(db_tree_->current_element_idx_);
	//ShowOpenglObject(db_tree_->current_element_idx_);
#endif

}

void MainWindow::CancelConstructionAction()
{
	printf("Cancel construction...\n");
}

//--------------------Simplification----------------------//
void MainWindow::SetSimplifyTargetFaceNumAction()
{
	printf("set Simplification faces number...\n"); 
	int target_face_num = this->ui.spinBoxTargetFaces->value();
	printf("target face num:%d\n", target_face_num);
	HW::HWParams::getInstance().simplification_params_.target_faces_num_ = target_face_num;
}
void MainWindow::SetSimplifyReductionRatioAction()
{
	printf("set Simplification reduction percentage...\n");
	double target_percentage = this->ui.doubleSpinBoxPercentage->value();
	printf("target percentage:%f\n", target_percentage);
	HW::HWParams::getInstance().simplification_params_.target_percentage_ = target_percentage;
}
void MainWindow::SetSimplifyQualityThresholdAction()
{
	printf("set Simplification quallity threshold...\n");
	double quality_threshold = this->ui.doubleSpinBoxThreshold->value();
	printf("quality threshold:%f\n", quality_threshold);
	HW::HWParams::getInstance().simplification_params_.quality_threshold_ = quality_threshold;
}
void MainWindow::CheckSimplifyBundryPreserveAction()
{
	if (this->ui.checkBoxBoundary->checkState() == Qt::Checked)
	{
		printf("check Simplification boundary preserving...\n");
	}
	else
	{
		printf("uncheck Simplification boundary preserving...\n");
	}
}
void MainWindow::SetSimplifyBundryWeightAction()
{
	printf("set Simplification boudary weight...\n");
	double boudary_weight = this->ui.doubleSpinBoxBoundaryWeight->value();
	printf("boudary weight:%f\n", boudary_weight);
	HW::HWParams::getInstance().simplification_params_.boundary_preserve_weight_ = boudary_weight;
}

void MainWindow::CheckSimplifyNormalPreserveAction()
{
	if (this->ui.checkBoxNormal_2->checkState() == Qt::Checked)
	{
		printf("check Simplification normal preserving...\n");
	}
	else
	{
		printf("uncheck Simplification normal preserving...\n");
	}
	//printf("check Simplification normal preverve...\n");
}
void MainWindow::CheckSimplifyToplogyPreserveAction()
{
	if (this->ui.checkBoxTopology->checkState() == Qt::Checked)
	{
		printf("check Simplification topology preserving...\n");
	}
	else
	{
		printf("uncheck Simplification topology preserving...\n");
	}
	//printf("check Simplification topology preverve...\n");
}

void MainWindow::CheckSimplifyOptimalAction()
{
	if (this->ui.checkBoxTopology->checkState() == Qt::Checked)
	{
		printf("check Simplification optimal...\n");
	}
	else
	{
		printf("uncheck Simplification optimal...\n");
	}
	printf("check Simplification optimal...\n");
}

void MainWindow::CheckSimplifyPlanarSimplifyAction()
{
	if (this->ui.checkBoxTopology->checkState() == Qt::Checked)
	{
		printf("check Simplification planar simplify...\n");
	}
	else
	{
		printf("uncheck Simplification planar simplify...\n");
	}
	//printf("check Simplification planar simplify...\n");
}

void MainWindow::DoSimplifyAction()
{
	//Sleep(10000);
	//ReadPly("D:/room_sketch/data/huawei_data/zheda_pointcloud/zheda/PPT/pointcloud_mesh_simplify.ply",false);
#if 1
	printf("run simplification...\n");
	printf("current mls: %s\n", db_tree_->current_element_idx_.c_str());
	if (db_tree_->current_element_idx_.empty())
	{
		printf("invalid select object...\n");
		return;
	}

	if (db_tree_->AccessTreeElements().empty())
	{
		printf("empty object...\n");
		return;
	}

	std::map<std::string, HW::HWObject*>::iterator iter = db_tree_->AccessTreeElements().find(db_tree_->current_element_idx_);
	if (iter != db_tree_->AccessTreeElements().end())
	{
		HW::HWPCLFunctor* simp_process = new HW::HWSIMP();
		processes_vec_.emplace_back(simp_process);
		HW::HWObject* out_simp_result;
		simp_process->Process(iter->second, out_simp_result);
		//test
		//std::string tmp_path = "D:/room_sketch/data/huawei_data/Huawei/pointCloud/test_sor.ply";
		//sor_process->resulted_element_->SavePly(tmp_path, HW::kBinary);
		//end test
		////test
		//if (sor_process == processes_vec_[0])
		//{
		//	printf("assign success...\n");
		//	return;
		//}
		////end test

		//生成唯一的对象，用路径名来识别它的唯一性
		std::string resulted_file_name = iter->first.substr(0, iter->first.find_last_of(".")) + "simp.ply";
		printf("Simplification resluted file: %s\n", resulted_file_name.c_str());
		std::map<std::string, HW::HWObject*>::iterator iter_simp = db_tree_->AccessTreeElements().find(resulted_file_name);
		if (iter_simp != db_tree_->AccessTreeElements().end())
		{
			//删除原来的，保存现在的file name
			//释放空间
			if (iter_simp->second != NULL)
			{
				delete iter_simp->second;
				iter_simp->second = simp_process->resulted_element_;
				return;
			}
		}
		else
		{
			//设置对象
			db_tree_->current_element_idx_ = resulted_file_name;
			simp_process->resulted_element_->SetObjectName(db_tree_->current_element_idx_);
			db_tree_->AccessTreeElements().insert(std::pair<std::string, HW::HWObject*>(db_tree_->current_element_idx_, simp_process->resulted_element_));
			//添加对应的item到view list 里面
			//AddViewListAItem(db_tree_->current_element_idx_);
		}
	}
	printf("end Simplification...\n");

	//opengl
	printf("SIMP current idx: %s\n", db_tree_->current_element_idx_.c_str());
	CopyHWObjToCCObj(db_tree_->current_element_idx_);
	AddViewListAItem(db_tree_->current_element_idx_);
	active_objects_names_.emplace_back(db_tree_->current_element_idx_);
	ShowAllActivedObjects();

	//ShowCurrentObject(db_tree_->current_element_idx_);
#endif
	//ShowOpenglObject(db_tree_->current_element_idx_);
}

void MainWindow::CancelSimplifyAction()
{
	printf("cancel simplification...\n");
}
void MainWindow::DoEdgeSwapAction() {
	printf("Edge swapping......\n");
	std::map<std::string, HW::HWObject*>::iterator iter = db_tree_->AccessTreeElements().find(db_tree_->current_element_idx_);
	if (iter != db_tree_->AccessTreeElements().end())
	{
		HW::HWMesh* selected_mesh = dynamic_cast<HW::HWMesh*>(iter->second);
		selected_mesh->SetObjectName(iter->first);
		selected_mesh->edgeSwap();
	}
	printf("End edge swapping......\n");
}

//------------------------Texture Generate-------------------------//
void MainWindow::DoTextureMappingAction()
{
#if 1
	//选中这个模型并进行texture mapping
	printf("Texture mapping.........\n");

	std::map<std::string, HW::HWObject*>::iterator iter = db_tree_->AccessTreeElements().find(db_tree_->current_element_idx_);
	if (iter != db_tree_->AccessTreeElements().end())
	{
		HW::HWMesh* selected_mesh = dynamic_cast<HW::HWMesh*>(iter->second);
		bool isRemoved = false;
		if(this->ui.actionTexture_AfterRemoving->isChecked())
			isRemoved = true;
		selected_mesh->SetObjectName(iter->first);
		selected_mesh->DoTextureMapping(isRemoved);
	}
	std::cout << "end Texture..." << std::endl;
#endif
}

void MainWindow::DoSamplingAction()
{
	if (db_tree_->current_element_idx_.empty())
	{
		printf("invalid select object...\n");
		return;
	}

	if (db_tree_->AccessTreeElements().empty())
	{
		printf("empty object...\n");
		return;
	}

	std::map<std::string, HW::HWObject*>::iterator iter = db_tree_->AccessTreeElements().find(db_tree_->current_element_idx_);
	if (iter->second->GetObjectType() != HW::kHWPointCloud) {
		printf("It's not a point cloud...\n");
		return;
	}
	HW::HWPointCloud* hw_pc = dynamic_cast<HW::HWPointCloud*>(iter->second);
	int hw_pc_pnt_num_per_square = hw_pc->ComputePointsNumPerSquareMeter();
	std::cerr << "hw_pc_pnt_num_per_square: " << hw_pc_pnt_num_per_square << std::endl;
	HW::HWPlane* plane = new HW::HWPlane();
	std::cout << "polygon_: " << polygon_.size() << "\n";
	HW::HWPointCloud* hw_pc_new;
	std::string current_obj_name = db_tree_->current_element_idx_;
	string new_obj_name = current_obj_name.substr(0, current_obj_name.find_last_of(".")) + "_created"+".ply";
	bool isexisted = false;
	if (db_tree_->AccessTreeElements().count(new_obj_name)) {
		hw_pc_new = dynamic_cast<HW::HWPointCloud*>(db_tree_->AccessTreeElements()[new_obj_name]);
		isexisted = true;
	}
	else {
		hw_pc_new = new HW::HWPointCloud();
		hw_pc_new->SetObjectType(HW::kHWPointCloud);
		hw_pc_new->SetObjectName(new_obj_name);
	}
    float3 normal_avg = {0, 0, 0};
	uchar3 color= hw_pc->GetAColor(polygon_idx_[0]);
	for (int i = 0; i < polygon_idx_.size(); i++) {
		float3 normal = hw_pc->GetANormal(polygon_idx_[i]);
		normal_avg = normal_avg * i + normal;
		normal_avg /= sqrt(pow(normal_avg.x, 2) + pow(normal_avg.y, 2) + pow(normal_avg.z, 2));
	}

    bool outdoorScene = false;
    if (this->ui.actionoutdoorFlag->isChecked()) {
        outdoorScene = true;
        printf("outdoor scene!\n");
    }

	for (int i = 0; i < polygon_idx_.size(); i++) {
		plane->AddPlanePnt(polygon_[i]);
		plane->AddPlanePntNormal(normal_avg);
	}
	plane->GenerateWorldCoordToPlaneCoordMatrix();
	plane->ProjectTo3DPlane();
	plane->Generate2DPlane();
	plane->GetPolygonFromPnts();	
	std::vector<float3> sample_vertices;
	plane->DoPolygonSampling(sample_vertices, hw_pc_pnt_num_per_square);
	Eigen::Vector3f plane_n = plane->GetCameraPose().block(0, 2, 3, 1);
	bool pickedPointsOnOnePlane = true;
	//if (plane_n.x()*normal_avg.x + plane_n.y()*normal_avg.y + plane_n.z()*normal_avg.z > 0.5) {
	//	normal_avg = make_float3(plane_n.x(), plane_n.y(), plane_n.z());
	//}
 //   else {
 //       pickedPointsOnOnePlane = false;
 //       printf("-------picked points are not on the same plane!------\n");
 //   }
    for (int i = 0; i < polygon_idx_.size(); i++) {
        float3 normalPol = hw_pc->GetANormal(polygon_idx_[i]);
        Eigen::Vector3f normalPolEig(normalPol.x, normalPol.y, normalPol.z);
        Eigen::Vector3f normalPlaneEig(plane_n);
        double cosAng = normalPolEig.dot(normalPlaneEig) / (normalPolEig.norm() * normalPlaneEig.norm());
        if (cosAng < 0.5) {    // 与法向量夹角大于60度，判定不在一个平面上
            pickedPointsOnOnePlane = false;
            printf("-------picked points are noton the same plane!-------\n");
            break;
        }
    }
	if (!pickedPointsOnOnePlane) {
		// 确定法向量大小
		Eigen::Vector3f p0(sample_vertices[0].x, sample_vertices[0].y, sample_vertices[0].z);
		Eigen::Vector3f p1(sample_vertices[1].x, sample_vertices[1].y, sample_vertices[1].z);
		Eigen::Vector3f p2(sample_vertices[2].x, sample_vertices[2].y, sample_vertices[2].z);
		Eigen::Vector3f v1 = p0 - p1;
		Eigen::Vector3f v2 = p2 - p1;
		Eigen::Vector3f normal_eigen = v1.cross(v2);
		normal_eigen.normalize();

		// 确定法向量方向
		//get required fields
		std::list<std::string> req_fields;
		try {
			req_fields.push_back("xyz");
		}
		catch (const std::bad_alloc&) {
			printf("-------error: bad xyz!-------\n");
			return;
		}

		//take out the xyz info
		pcl::PCLPointCloud2::Ptr pcl_cloud2 = HW::HWTOPCLCloud(hw_pc).GetAsHWPointCloud(req_fields);
		if (!pcl_cloud2) {
			printf("-------!!!!!!!error: HWTOPCLCloud filled!!!!!!!-------\n");
		}

		//get as pcl point cloud
		pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::fromPCLPointCloud2(*pcl_cloud2, *pcl_cloud);

		// set kd-tree
		pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree(new pcl::KdTreeFLANN<pcl::PointXYZ>);
		float radius = 0.2;    // 搜索距离20cm
        if (outdoorScene) {
            radius = 2.0;
        }
        printf("kd-tree search radius is %f.\n", radius);
		kdtree->setInputCloud(pcl_cloud);

		// get neighbor directions of all picked points 
		std::vector<Eigen::Vector3f> directions;
		pcl::PointCloud<pcl::PointXYZ>::Ptr positionsCloud(new pcl::PointCloud<pcl::PointXYZ>);
		for (int i = 0; i < polygon_idx_.size(); i++) {
			float3 pointPicked = hw_pc->GetAPoint(polygon_idx_[i]);
			std::vector<int> pointIdxRadiusSearch;
			std::vector<float> pointRadiusSquaredDistance;
			pcl::PointXYZ point(pointPicked.x, pointPicked.y, pointPicked.z);
			kdtree->radiusSearch(point, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
			printf("picked point %d has %d neighbors.\n", i, pointIdxRadiusSearch.size());
			for (auto idx : pointIdxRadiusSearch) {
				pcl::PointXYZ point = pcl_cloud->points[idx];
				Eigen::Vector3f direction(point.x - pointPicked.x, point.y - pointPicked.y, point.z - pointPicked.z);
				directions.push_back(direction);
			}
		}

		// count number of inverse points
		int countInverseDirections = 0;
		for (auto d : directions) {
			if (d.dot(normal_eigen) < 0) {
				countInverseDirections++;
			}
		}
		printf("neighbors in total : %d, angle > 90 : %d.\n", directions.size(), countInverseDirections);
		if ((!outdoorScene && countInverseDirections >= directions.size() / 2)
            || (outdoorScene && countInverseDirections < directions.size() / 2)) {
			normal_eigen = -normal_eigen;
			printf("normal eigen inversed!\n");
		}
        float3 normal_comp = {0, 0, 0};
		normal_comp.x = normal_eigen[0];
		normal_comp.y = normal_eigen[1];
		normal_comp.z = normal_eigen[2];
		printf("normal computered = (%f, %f, %f)\n", normal_comp.x, normal_comp.y, normal_comp.z);
		if (plane_n.x()*normal_comp.x + plane_n.y()*normal_comp.y + plane_n.z()*normal_comp.z > 0) {
			normal_avg = make_float3(plane_n.x(), plane_n.y(), plane_n.z());
		}
		else {
			normal_avg = -make_float3(plane_n.x(), plane_n.y(), plane_n.z());
		}
        printf("normal final = (%f, %f, %f)\n", normal_avg.x, normal_avg.y, normal_avg.z);
	}

	ccPointCloud* current_pc;
    if (isexisted) {
        current_pc = static_cast<ccPointCloud*> (GetObjectByStrName(new_obj_name));
    }
	for (int i = 0; i < sample_vertices.size(); i++) {
		hw_pc_new->AddPoint(sample_vertices[i]);
		hw_pc_new->AddNormal(normal_avg);
		hw_pc_new->AddColor(color);
		if (isexisted) {
			current_pc->addPoint(CCVector3(sample_vertices[i].x, sample_vertices[i].y, sample_vertices[i].z));
			current_pc->addNorm(CCVector3(normal_avg.x, normal_avg.y, normal_avg.z));
			current_pc->addRGBColor(color.x, color.y, color.z);
		}
	}
	if (!isexisted) {
		HW::HWObject* element_add = hw_pc_new;
		element_add->SetObjectType(HW::ElementType::kHWPointCloud);
		element_add->SetObjectName(new_obj_name);
		db_tree_->AccessTreeElements().insert(std::pair<std::string, HW::HWObject*>(new_obj_name, element_add));

		//插入对象
		CopyHWObjToCCObj(new_obj_name);
		AddViewListAItem(new_obj_name);
		active_objects_names_.emplace_back(new_obj_name);
	}
	m_ccRoot1->detachChild(m_polyobj_);

	ShowAllActivedObjects();
	polygon_.clear();
	polygon_idx_.clear();
	//delete m_polyline_;
}

void MainWindow::DoCompletePolygonAction()
{
	if (m_polyline_->size() != 3) {
		printf("num of polygon vertices is not 3!\n");
		return;
	}
	ccPointCloud* pc = static_cast<ccPointCloud*>(m_polyline_->getChild(0));
	float3 vert = polygon_[0] + polygon_[2] - polygon_[1];;
	pc->addPoint(CCVector3(vert.x, vert.y, vert.z));
	pc->addNorm(pc->getPointNormal(polygon_idx_[0]));
	pc->addRGBColor(pc->getPointColor(polygon_idx_[0]));
	m_polyline_->addPointIndex(pc->size()-1);
	ShowAllActivedObjects();
	polygon_.emplace_back(vert);
	polygon_idx_.emplace_back(pc->size() - 1);
	m_polyline_->setClosed(true);
}

void MainWindow::DoConvertFilesIntoHWFilesAction()
{
	std::cout << "start to Convert..." << std::endl;
	ConvertCamModelDigram();
	std::cout << "End Converting..." << std::endl;
}

//------------------------Tool Action-------------------------------//
void MainWindow::SetSunLightAction()
{
	//ccMesh* selected_mesh = m_ccRoot1->getChild(0);
	if (this->ui.actionSunLight->isChecked())
	{
		m_root_gl_->setCustomLight(true);
	}
	else
	{
		m_root_gl_->setCustomLight(false);
	}
}

void MainWindow::SetShowWireAction()
{
	//把选中的点云显示它的网格
	std::string current_obj = db_tree_->current_element_idx_;
	QString current_obj_name(current_obj.data());
	for (int i = 0; i < m_ccRoot1->getChildrenNumber(); ++i)
	{
		if (current_obj_name == m_ccRoot1->getChild(i)->getName())
		{
			if (this->ui.actionShowWired->isChecked()
				&& m_ccRoot1->getChild(i)->getFirstChild()->getClassID() == CC_TYPES::MESH)
			{
				ccMesh* selected_mesh = dynamic_cast<ccMesh*>(m_ccRoot1->getChild(i)->getFirstChild());

				selected_mesh->showWired(true);
				//m_root_gl_->setWire(true);
			}
			else if(m_ccRoot1->getChild(i)->getFirstChild()->getClassID() == CC_TYPES::MESH)
			{
				ccMesh* selected_mesh = dynamic_cast<ccMesh*>(m_ccRoot1->getChild(i)->getFirstChild());
				selected_mesh->showWired(false);
				//m_root_gl_->setSunLight(true);
			}
		}
	}
	//update();
	this->m_root_gl_->redraw();
}

void MainWindow::enablePickingOperation(ccGLWindow* win, QString message)
{
	if (!win)
	{
		assert(false);
		return;
	}

	assert(m_pickingHub);
	if (!m_pickingHub->addListener(this))
	{
		ccLog::Error("Can't start the picking mechanism (another tool is already using it)");
		return;
	}

	//specific case: we prevent the 'point-pair based alignment' tool to process the picked point!
	//if (m_pprDlg)
	//	m_pprDlg->pause(true);

	s_pickingWindow = win;
	win->displayNewMessage(message, ccGLWindow::LOWER_LEFT_MESSAGE, true, 24 * 3600);
	win->redraw(true, false);

	//freezeUI(true);
}

void MainWindow::cancelPreviousPickingOperation(bool aborted)
{
	if (!s_pickingWindow)
		return;

	switch (s_currentPickingOperation)
	{
	case PICKING_ROTATION_CENTER:
		//nothing to do
		break;
	case PICKING_LEVEL_POINTS:
		if (s_levelMarkersCloud)
		{
			s_pickingWindow->removeFromOwnDB(s_levelMarkersCloud);
			delete s_levelMarkersCloud;
			s_levelMarkersCloud = nullptr;
		}
		break;
	default:
		assert(false);
		break;
	}

	if (aborted)
	{
		s_pickingWindow->displayNewMessage(QString(), ccGLWindow::LOWER_LEFT_MESSAGE); //clear previous messages
		s_pickingWindow->displayNewMessage("Picking operation aborted", ccGLWindow::LOWER_LEFT_MESSAGE);
	}
	s_pickingWindow->redraw(false);

	////specific case: we allow the 'point-pair based alignment' tool to process the picked point!
	//if (m_pprDlg)
	//	m_pprDlg->pause(false);

	//freezeUI(false);

	m_pickingHub->removeListener(this);

	s_pickingWindow = nullptr;
	s_currentPickingOperation = NO_PICKING_OPERATION;
}

bool MainWindow::propagationBasedSelectPoints(CCLib::GenericIndexedCloudPersist* theCloud,
	unsigned char octreeLevel,
	CCLib::DgmOctree* inputOctree,
	CCLib::GenericProgressCallback* progressCb,
	const PickedItem& pi)
{
	//
	unsigned numberOfPoints = (theCloud ? theCloud->size() : 0);
	if (numberOfPoints == 0)
	{
		return false;
	}

	//compute octree if none was provided
	CCLib::DgmOctree* theOctree = inputOctree;
	if (!theOctree)
	{
		theOctree = new CCLib::DgmOctree(theCloud);
		if (theOctree->build(progressCb) < 1)
		{
			delete theOctree;
			return false;
		}
	}

	//size
	PointCoordinateType radius = 0.02;

	CCVector3 pickedPoint = pi.P3D;

	//获取Octree的bounding box
	CCVector3 min_box, max_box;
	theOctree->getBoundingBox(min_box, max_box);
	
	float max_dist = std::max((max_box.x - min_box.x), (max_box.y - min_box.y));
	max_dist = std::max(max_dist, (max_box.z - min_box.z));

	int resolution = max_dist / radius;

	std::cout << "the resolution: " << resolution << std::endl;

	//获取volume
	VertexRearrangement *volume = new VertexRearrangement(radius, resolution);

	const CCVector3* pickedPoint1 = &pi.P3D;
	printf("asdfsafd\n");
	//system("pause");

	//获取选中顶点的所在的cell

	////初始化mask
	//std::vector<bool> mask_traverse;
	//for (int i = 0; i < theCloud->size(); ++i)
	//{
	//	mask_traverse.emplace_back(false);
	//}

	std::unordered_map<Eigen::Vector3i, bool, open3d::hash_eigen::hash<Eigen::Vector3i>>::iterator iter;
	std::unordered_map<Eigen::Vector3i, bool, open3d::hash_eigen::hash<Eigen::Vector3i>> volume_traverse_flag;

	for (int idx = 0; idx < theCloud->size(); ++idx)
	{
		const CCVector3* pnt = theCloud->getPoint(idx);
		float3 point_pos = make_float3(pnt->x, pnt->y, pnt->z);
		int3 picked_voxel = volume->LocateVolumeVoxelPos(point_pos);
		Eigen::Vector3i volume_idx(picked_voxel.x, picked_voxel.y, picked_voxel.z);
		iter = volume_traverse_flag.find(volume_idx);
		if (iter == volume_traverse_flag.end())
		{
			volume_traverse_flag.insert(std::make_pair(volume_idx, false));
		}
	}

	std::cout << "volume_traverse_flag size: " << volume_traverse_flag.size() << std::endl;

	//获取picked point的voxel
	float3 picked_point_pos = make_float3(pickedPoint1->x, pickedPoint1->y, pickedPoint1->z);
	int3 picked_voxel = volume->LocateVolumeVoxelPos(picked_point_pos);

	std::cout << "the picked voxel pos: " << picked_voxel.x << 
		" " << picked_voxel.y << " " << picked_voxel.z << std::endl;
	//system("pause");

	//保存已经获取了voxel坐标
	std::vector<int3> propagated_voxels;

	//NeighboursSet
	CCLib::DgmOctree::NeighboursSet temp_sets;
	int cell_number = theOctree->getCellNumber(0);
	//theOctree->getTheCellPosWhichIncludesThePoint()
	///int cell_depth = theOctree->get
	std::cout << "mask traverse size: " << theCloud->size() << std::endl;
	std::cout << "cell number: " << cell_number << std::endl;
	//system("pause");
	octreeLevel = theOctree->findBestLevelForAGivenNeighbourhoodSizeExtraction(radius);
	//mask_traverse[pi.itemIndex] = true;
	printf("octree level is: %d\n", octreeLevel);

	int recur_level = 0;

	propagationBasedSelectedCellPos(octreeLevel, theOctree,
		volume, picked_voxel, radius, propagated_voxels, volume_traverse_flag);

	std::cout << "picked voxel size is: " << propagated_voxels.size() << std::endl;

	//
	std::vector<int> propagated_pnt_idx;
	//在voxel里面找到顶点，然后保存它的索引即可
	for (int i = 0; i < propagated_voxels.size(); ++i)
	{
		//
		CCVector3f point_center(propagated_voxels[i].x * radius + 0.5 *radius,
			propagated_voxels[i].y * radius + 0.5 *radius, propagated_voxels[i].z * radius + 0.5 *radius);
		CCLib::DgmOctree::NeighboursSet neighbor_set;
		theOctree->getPointsInSphericalNeighbourhood(point_center, radius, neighbor_set, octreeLevel);

		for (int j = 0; j < neighbor_set.size(); ++j)
		{
			int pnt_idx = neighbor_set[j].pointIndex;
			propagated_pnt_idx.emplace_back(pnt_idx);
		}
	}
	
	std::sort(propagated_pnt_idx.begin(), propagated_pnt_idx.end());
	std::vector<int>::iterator pos;
	pos = std::unique(propagated_pnt_idx.begin(), propagated_pnt_idx.end());
	propagated_pnt_idx.erase(pos, propagated_pnt_idx.end());

	std::string current_obj_name = db_tree_->current_element_idx_;
	ccPointCloud* current_pc = static_cast<ccPointCloud*> (GetObjectByStrName(current_obj_name));
	ccColor::Rgb col(255, 255, 255);
	for (int i = 0; i < propagated_pnt_idx.size(); ++i)
	{
		current_pc->pc_deleted_idx_.emplace_back(propagated_pnt_idx[i]);
		if (current_pc->hasColors())
		{
			current_pc->setPointColor(propagated_pnt_idx[i], col);
		}
	}

	if (plane_state_)
	{
		//std::cout << "run the plane state..." << std::endl;
		////把平面抽取出来,需要先判断一下选取出来的点云是否是平面，有可能点的顶点
		//if (m_plane_)
		//{
		//	std::cout << "test111" << std::endl;
		//	delete m_plane_;
		//	m_plane_ = NULL;
		//	m_plane_ = new HW::HWPlane();
		//}
		//else
		//{
		//	std::cout << "test22222" << std::endl;
		//	m_plane_ = new HW::HWPlane();
		//}
		m_plane_ = new HW::HWPlane();
#if 1
		//获取voxel，然后保存
		for (int i = 0; i < propagated_pnt_idx.size(); ++i)
		{
			const CCVector3* pnt = current_pc->getPoint(propagated_pnt_idx[i]);
			CCVector3f pnt_normal = current_pc->getPointNormal(propagated_pnt_idx[i]);
			m_plane_->AddPlanePnt(pnt->x, pnt->y, pnt->z);
			m_plane_->AddPlanePntNormal(pnt_normal.x, pnt_normal.y, pnt_normal.z);
		}

		std::cout << "the plane cloud size is: "<< m_plane_->GetPlanePnts().size() << std::endl;

		//system("puase");

		//将m_plane_加入到dbtree里面
		//将HWPointCloud传输到ccPointCloud,并且显示到系统界面
		HW::HWObject* element_add = static_cast<HW::HWObject*>(m_plane_);
		element_add->SetObjectType(HW::ElementType::kHWPlane);

		//将平面对象放到
		std::string plane_name = current_obj_name.substr(0, current_obj_name.find_last_of(".")) + "_plane.ply";
		db_tree_->current_element_idx_ = plane_name;
		std::cout << "current plane element name is: " << db_tree_->current_element_idx_ << std::endl;
		//system("pause");

		element_add->SetObjectName(db_tree_->current_element_idx_);
		db_tree_->AccessTreeElements().insert(std::pair<std::string, HW::HWObject*>(db_tree_->current_element_idx_, element_add));

		//插入对象
		CopyHWObjToCCObj(db_tree_->current_element_idx_);
		AddViewListAItem(db_tree_->current_element_idx_);
		active_objects_names_.emplace_back(db_tree_->current_element_idx_);

		/*std::cout << "save the plane Obj name..." << std::endl;
		m_plane_->SavePlanePointIntoOBJ(plane_name);

		std::cout << "end save the plane..." << std::endl;*/

		//system("pause");

#if 0
		//统计时间
		clock_t start_time, end_time;
		start_time = clock();

		m_plane_->GenerateWorldCoordToPlaneCoordMatrix();
		m_plane_->ProjectTo3DPlane();
		std::string plane_path = "D:\\vc_project_xht\\back_up\\test_for_proj\\plane_pnts.obj";
		m_plane_->SavePlanePointIntoOBJ(plane_path);
		std::cout << "end test" << std::endl;

		m_plane_->Generate2DPlane();
		m_plane_->DoEstimateBorderFrom2DPlane();
		m_plane_->SortBorderEdges();
		m_plane_->GenerateInitial2DSortedPnts();
		m_plane_->Generate2DPloygons();
		m_plane_->MapSortedEdgePnt2PntPosIdx();
		std::cout << "end map the sorted edge pnts!!" << std::endl;
		m_plane_->ComputeImageConvertParams();
		m_plane_->GenerateEdgeImagePolygon();
		m_plane_->TriangulatePolygon2D();
		m_plane_->GenerateSampleWorldCoordPnts();
		end_time = clock();
		std::cout << "process fill hole time: " << (double)(end_time - start_time) / CLOCKS_PER_SEC << "s" << std::endl;

		m_plane_->MergeSampleAndSrcPnts();
		m_plane_->SaveMargedPlanePntsPly("D:/vc_project_xht/back_up/test_for_plane/merged_plane_pnt.ply", HW::kBinary);
#endif
		return true;
#endif
	}
	else
	{
#if 0
		//move to merge
		//找到对应的merge
		std::string find_merge_name = current_obj_name.substr(0, current_obj_name.find("_removed")) + "merge.ply";
		std::cout << "the find merge name is: " << find_merge_name << std::endl;
		ccPointCloud* merge_pc = static_cast<ccPointCloud*>(GetObjectByStrName(find_merge_name));
		if (merge_pc)
		{
			//获取voxel，然后保存
			for (int i = 0; i < propagated_pnt_idx.size(); ++i)
			{
				//
				const CCVector3* tmp_pnt = theCloud->getPoint(propagated_pnt_idx[i]);
				const CCVector3 tmp_normal = current_pc->getPointNormal(propagated_pnt_idx[i]);
				//const CCVector3* tmp_pnt_normal = pi.entity->get
				merge_pc->addPoint(*tmp_pnt);

				merge_pc->addNorm(tmp_normal);
				//tmp_cloud->AddNormal(make_float3(tmp))
			}
		}
		//将点云返回到HWP

		std::cout << "end save normal..." << std::endl;
		//end move to merge
#if 0
		//test
		HW::HWPointCloud *tmp_cloud = new HW::HWPointCloud();
		tmp_cloud->SetObjectName("D:\\vc_project_xht\\room_pointcloud_raw_selected.ply");

		//获取voxel，然后保存
		for (int i = 0; i < propagated_pnt_idx.size(); ++i)
		{
			//
			const CCVector3* tmp_pnt = theCloud->getPoint(propagated_pnt_idx[i]);
			//const CCVector3* tmp_pnt_normal = pi.entity->get
			tmp_cloud->AddPoint(make_float3(tmp_pnt->x, tmp_pnt->y, tmp_pnt->z));
			//tmp_cloud->AddNormal(make_float3(tmp))
		}

		tmp_cloud->SavePly(tmp_cloud->GetObjectName(), HW::kBinary);
		delete tmp_cloud;
		std::cout << "end save the propagation pnts\n" << std::endl;
		//end test
#endif

#endif
		return false;
	}
	
	//system("pause");
	delete volume;
}

void MainWindow::propagationBasedSelectedCellPos(
	unsigned char octreeLevel,
	CCLib::DgmOctree* inputOctree,
	VertexRearrangement* volume,
	const int3& voxelPos,
	float radius,
	std::vector<int3>& voxel_set,
	std::unordered_map<Eigen::Vector3i, bool, open3d::hash_eigen::hash<Eigen::Vector3i>>& travese_mask)
{
	int resolution = volume->GetVolumeResolution();
	float voxel_length = volume->GetVoxelLength();

	//增加Neighbor voxel 到voxel_set
	std::vector<int3> filter_neigbor_voxels;
	std::vector<int3>().swap(filter_neigbor_voxels);

	for (int i = -1; i <= 1; i++)
	{
		for (int j = -1; j <= 1; j++)
		{
			for (int k = -1; k <= 1; ++k)
			{
				if (i == 0 && j == 0 && k == 0)
					continue;
				Eigen::Vector3i neigbor_voxel(voxelPos.x + i,
					voxelPos.y + j, voxelPos.z + k);
				std::unordered_map<Eigen::Vector3i, bool, open3d::hash_eigen::hash<Eigen::Vector3i>>::iterator iter;
				//判断这个neibor没有被访问
				iter = travese_mask.find(neigbor_voxel);
				if (iter != travese_mask.end())
				{
					// 如果这个voxel里面有顶点，则保留
					if (!(*iter).second)
					{
						(*iter).second = true;
						int3 neigbor_voxel_int3 = make_int3(neigbor_voxel.x(), neigbor_voxel.y(), neigbor_voxel.z());
						filter_neigbor_voxels.emplace_back(neigbor_voxel_int3);
					}
				}
			}
		}
	}
	//
	for (int i = 0; i < filter_neigbor_voxels.size(); ++i)
	{
		//从voxel pos到 octree的3d坐标转化
		CCVector3f voxel_pos = CCVector3f(filter_neigbor_voxels[i].x * voxel_length + 0.5 * voxel_length,
			filter_neigbor_voxels[i].y * voxel_length + 0.5 * voxel_length, filter_neigbor_voxels[i].z * voxel_length + 0.5 * voxel_length);

		//NeighboursSet
		CCLib::DgmOctree::NeighboursSet temp_set;

		int set_size = inputOctree->getPointsInSphericalNeighbourhood(voxel_pos, radius,
			temp_set, octreeLevel);

		printf("voxel points number is: %d\n", set_size);

		if (set_size > 10)
		{
			//
			voxel_set.emplace_back(filter_neigbor_voxels[i]);

			propagationBasedSelectedCellPos(octreeLevel, inputOctree,
				volume, filter_neigbor_voxels[i], radius, voxel_set, travese_mask);
		}
	}
}

//
void MainWindow::onItemPicked(const PickedItem& pi)
{
	if (!s_pickingWindow || !m_pickingHub)
	{
		return;
	}

	if (!pi.entity)
	{
		return;
	}

	if (m_pickingHub->activeWindow() != s_pickingWindow)
	{
		ccLog::Warning("The point picked was picked in the wrong window");
		return;
	}

	CCVector3 pickedPoint = pi.P3D;
	int idx = pi.itemIndex;
	m_polyline_->addPointIndex(idx);
	ShowAllActivedObjects();
	//printf("asdfsdfsadfs\n");
	printf("the picked point is: %f %f %f\n", pickedPoint[0], pickedPoint[1], pickedPoint[2]);
	polygon_.emplace_back(make_float3(pickedPoint[0], pickedPoint[1], pickedPoint[2]));
	polygon_idx_.emplace_back(idx);
	//system("pause");
	//test by zdg
	if (m_polyline_)
	{
		int polyline_vers_num = m_polyline_->size();
		std::cerr << "polyline_vers_num: " << polyline_vers_num << std::endl;
	}
	//end test by zdg
	//switch (s_currentPickingOperation)
	//{
	//case PICKING_SELECT_OBJ:
	//{
	//	//获取对象
	//	if (pi.entity->isKindOf(CC_TYPES::POINT_CLOUD))
	//	{
	//		/*std::cout << "asdfsadfsafd" << std::endl;
	//		system("pause");*/
	//		ccGenericPointCloud* theCloud = static_cast<ccGenericPointCloud*>(pi.entity);
	//		ccOctree::Shared pc_octree = theCloud->getOctree();
	//		std::cout << "pc octree number: " << theCloud->size() << std::endl;
	//		unsigned numberOfPoints = (theCloud ? theCloud->size() : 0);
	//		if (numberOfPoints == 0)
	//		{
	//			return;
	//		}
	//		//获取octree level
	//		unsigned char octree_level = 5;
	//		propagationBasedSelectPoints(theCloud, octree_level, nullptr, nullptr, pi);
	//		std::cout << "test test test test test" << std::endl;
	//		//float radius = 0.0;
	//		//float minSeedDist = 0.08;
	//		//CCLib::ReferenceCloudContainer segment_lists;
	//		//std::cout << "asdfsadfsafd" << std::endl;
	//		//system("pause");
	//		//if (frontPropagationBasedSegmentation(theCloud, radius,
	//		//	minSeedDist, octree_level, segment_lists, nullptr, nullptr, true, 1.0f))
	//		//{
	//		//	std::cout << "propagation failed" << std::endl;
	//		//}
	//		//std::cout << "11111111111111111111" << std::endl;
	//		//system("pause");
	//		//char file_name[256];
	//		//
	//		////test
	//		//for (int i = 0; i < segment_lists.size(); ++i)
	//		//{
	//		//	//
	//		//	std::sprintf(file_name, "D:/vc_project_xht/test_for_segment/seg_%d.ply", i);
	//		//	HW::HWPointCloud* hw_pc = new HW::HWPointCloud();
	//		//	hw_pc->SetObjectName(file_name);
	//		//	hw_pc->SetObjectType(HW::kHWPointCloud);
	//		//	//ccPointCloud* cc_pc = new ccPointCloud()
	//		//	for (int j = 0; j < segment_lists[i]->size(); ++j)
	//		//	{
	//		//		const CCVector3* p = segment_lists[i]->getPoint(j);
	//		//		//const CCVector3* p_normal = segment_lists[i]
	//		//		hw_pc->AddPoint(make_float3(p->x, p->y, p->z));
	//		//	}
	//		//	hw_pc->SavePly(hw_pc->GetObjectName(), HW::kBinary);
	//		//}
	//		//////
	//		////if (pc_octree)
	//		////{
	//		////	Tuple3i point_cell_pos;
	//		////	//pc_octree->getCellPos();
	//		////	//pc_octree->getTheCellPosWhichIncludesThePoint(pickedPoint, point_cell_pos);
	//		////}
	//	}
	//	//
	//	//system("pause");
	//}
	//case PICKING_LEVEL_POINTS:
	//{
	//	//we only accept points picked on the right entity!
	//	//if (obj != s_levelEntity)
	//	//{
	//	//	ccLog::Warning(QString("[Level] Only points picked on '%1' are considered!").arg(s_levelEntity->getName()));
	//	//	return;
	//	//}
	//	if (!s_levelMarkersCloud)
	//	{
	//		assert(false);
	//		cancelPreviousPickingOperation(true);
	//		printf("cancel the pick operation.\n");
	//		//system("pause");
	//		return;
	//	}
	//	for (unsigned i = 0; i < s_levelMarkersCloud->size(); ++i)
	//	{
	//		const CCVector3* P = s_levelMarkersCloud->getPoint(i);
	//		if ((pickedPoint - *P).norm() < 1.0e-6)
	//		{
	//			ccLog::Warning("[Level] Point is too close from the others!");
	//			return;
	//		}
	//	}
	//	//test
	//	printf("s_levelMarkersCloud is not NULL\n");
	//	//system("pause");
	//	//end test
	//	//add the corresponding marker
	//	s_levelMarkersCloud->addPoint(pickedPoint);
	//	unsigned markerCount = s_levelMarkersCloud->size();
	//	cc2DLabel* label = new cc2DLabel();
	//	label->addPickedPoint(s_levelMarkersCloud, markerCount - 1);
	//	label->setName(QString("P#%1").arg(markerCount));
	//	label->setDisplayedIn2D(false);
	//	label->setDisplay(s_pickingWindow);
	//	label->setVisible(true);
	//	s_levelMarkersCloud->addChild(label);
	//	s_pickingWindow->redraw();
	//	if (markerCount == 3)
	//	{
	//		//we have enough points!
	//		const CCVector3* A = s_levelMarkersCloud->getPoint(0);
	//		const CCVector3* B = s_levelMarkersCloud->getPoint(1);
	//		const CCVector3* C = s_levelMarkersCloud->getPoint(2);
	//		CCVector3 X = *B - *A;
	//		CCVector3 Y = *C - *A;
	//		CCVector3 Z = X.cross(Y);
	//		//we choose 'Z' so that it points 'upward' relatively to the camera (assuming the user will be looking from the top)
	//		CCVector3d viewDir = s_pickingWindow->getCurrentViewDir();
	//		if (CCVector3d::fromArray(Z.u).dot(viewDir) > 0)
	//		{
	//			Z = -Z;
	//		}
	//		Y = Z.cross(X);
	//		X.normalize();
	//		Y.normalize();
	//		Z.normalize();
	//		ccGLMatrixd trans;
	//		double* mat = trans.data();
	//		mat[0] = X.x; mat[4] = X.y; mat[8] = X.z; mat[12] = 0;
	//		mat[1] = Y.x; mat[5] = Y.y; mat[9] = Y.z; mat[13] = 0;
	//		mat[2] = Z.x; mat[6] = Z.y; mat[10] = Z.z; mat[14] = 0;
	//		mat[3] = 0; mat[7] = 0; mat[11] = 0; mat[15] = 1;
	//		CCVector3d T = -CCVector3d::fromArray(A->u);
	//		trans.apply(T);
	//		T += CCVector3d::fromArray(A->u);
	//		trans.setTranslation(T);
	//		assert(haveOneSelection() && m_selectedEntities.front() == s_levelEntity);
	//		//applyTransformation(trans);
	//		//clear message
	//		s_pickingWindow->displayNewMessage(QString(), ccGLWindow::LOWER_LEFT_MESSAGE, false); //clear previous message
	//		s_pickingWindow->setView(CC_TOP_VIEW);
	//	}
	//	else
	//	{
	//		//we need more points!
	//		return;
	//	}
	//}
	////we use the next 'case' entry (PICKING_ROTATION_CENTER) to redefine the rotation center as well!
	//assert(s_levelMarkersCloud && s_levelMarkersCloud->size() != 0);
	//pickedPoint = *s_levelMarkersCloud->getPoint(0);
	////break;

	//case PICKING_ROTATION_CENTER:
	//{
	//	CCVector3d newPivot = CCVector3d::fromArray(pickedPoint.u);
	//	//specific case: transformation tool is enabled
	//	//if (m_transTool && m_transTool->started())
	//	//{
	//	//	m_transTool->setRotationCenter(newPivot);
	//	//	const unsigned& precision = s_pickingWindow->getDisplayParameters().displayedNumPrecision;
	//	//	s_pickingWindow->displayNewMessage(QString(), ccGLWindow::LOWER_LEFT_MESSAGE, false); //clear previous message
	//	//	s_pickingWindow->displayNewMessage(QString("Point (%1 ; %2 ; %3) set as rotation center for interactive transformation")
	//	//		.arg(pickedPoint.x, 0, 'f', precision)
	//	//		.arg(pickedPoint.y, 0, 'f', precision)
	//	//		.arg(pickedPoint.z, 0, 'f', precision),
	//	//		ccGLWindow::LOWER_LEFT_MESSAGE, true);
	//	//}
	//	//else
	//	{
	//		const ccViewportParameters& params = s_pickingWindow->getViewportParameters();
	//		if (!params.perspectiveView || params.objectCenteredView)
	//		{
	//			//apply current GL transformation (if any)
	//			pi.entity->getGLTransformation().apply(newPivot);
	//			s_pickingWindow->setPivotPoint(newPivot, true, true);
	//		}
	//	}
	//	//s_pickingWindow->redraw(); //already called by 'cancelPreviousPickingOperation' (see below)
	//}
	//break;

	//default:
	//	assert(false);
	//	break;
	//}

	//cancelPreviousPickingOperation(false);
}

void MainWindow::ActivatePointPickingMode()
{
	ccGLWindow* win = getActiveGLWindow();
	if (!win)
	{
		return;
	}
	if (this->ui.actionPickPoint->isChecked())
	{
		printf("test\n");
		//s_currentPickingOperation = PICKING_LEVEL_POINTS;
		s_currentPickingOperation = PICKING_SELECT_OBJ;
		enablePickingOperation(win, "Pick a point to be used as rotation center");
		ccPointCloud* pc;
		//获取被选中的点云的名字
		std::string current_selected_name = db_tree_->current_element_idx_;
		//std::string 
		for (int i = 0; i < m_ccRoot1->getChildrenNumber(); ++i)
		{
			//
			if (m_ccRoot1->getChild(i)->getName() == QString(current_selected_name.data()))
			{
				pc = static_cast<ccPointCloud*>(m_ccRoot1->getChild(i)->getFirstChild());
				break;
			}
		}

		if (!pc)
		{
			printf("select no element!\n");
			return;
		}
		polygon_.clear();
		polygon_idx_.clear();
		if(m_polyobj_!=nullptr)
			m_ccRoot1->detachChild(m_polyobj_);
		m_polyline_ = new ccPolyline(pc);
		m_polyline_->addChild(pc);
		string polyname = current_selected_name + "_polyline";	
		m_polyline_->setForeground(true);
		m_polyline_->setColor(ccColor::red);
		m_polyline_->showColors(true);
		m_polyline_->setWidth(8);
		m_polyline_->showVertices(true);
		m_polyline_->setDisplay(m_root_gl_);
		m_polyline_->setVisible(true);
		m_polyline_->prepareDisplayForRefresh();
		m_polyline_->refreshDisplay(true);
		m_polyobj_ = new ccHObject(polyname.data());
		m_polyobj_->addChild(m_polyline_);
		m_ccRoot1->addChild(m_polyobj_);
	}
	else
	{
		m_polyline_->setClosed(true);
		printf("cancel test\n");
		cancelPreviousPickingOperation(true);
	}
}

//mouseReleaseEvent() doPicking() processClickableItems 

void MainWindow::DeactivatePointPickingMode(bool)
{
	printf("cancel the PointPicking");
}

void MainWindow::doLevel()
{
	//picking operation already in progress
	if (s_pickingWindow)
	{
		if (s_currentPickingOperation == PICKING_LEVEL_POINTS)
		{
			cancelPreviousPickingOperation(true);
		}
		else
		{
			ccLog::Error("Stop the other picking operation first!");
		}
		return;
	}

	ccGLWindow* win = getActiveGLWindow();
	if (!win)
	{
		ccLog::Error("No active 3D view!");
		return;
	}

	/*if (!haveOneSelection())
	{
		ccLog::Error("Select an entity!");
		return;
	}*/

	//create markers cloud
	assert(!s_levelMarkersCloud);
	{
		s_levelMarkersCloud = new ccPointCloud("Level points");
		if (!s_levelMarkersCloud->reserve(3))
		{
			ccLog::Error("Not enough memory!");
			return;
		}
		win->addToOwnDB(s_levelMarkersCloud);
	}

	s_levelEntity = m_selectedEntities[0];
	s_levelLabels.clear();
	s_currentPickingOperation = PICKING_LEVEL_POINTS;

	enablePickingOperation(win, "Pick three points on the floor plane (click the Level button or press Escape to cancel)");
}

#if 1

void MainWindow::ConvertCamModelDigram()
{
	//获取当前选中的对象
	if (db_tree_->current_element_idx_.empty())
		return;

	HW::HWObject* obj = GetCurrentObject();
	std::string file_name = obj->GetObjectName();

	if (!obj)
		return;

	std::string mesh_dir = file_name.substr(0, file_name.find_last_of("/"));
	std::cout << "texture mesh dir is: " << mesh_dir << std::endl;
	std::string photo_dir = mesh_dir + "/photos/";
	std::string scene_dir = mesh_dir + "/scene/";
	std::string cam_dir = mesh_dir;
	if (_access(photo_dir.c_str(), 0) == -1)
	{
		std::cout << "teture mesh dir has no pano directory~" << std::endl;
		return;
	}
	if (_access(scene_dir.c_str(), 0) == -1) {
		mkdir(scene_dir.c_str());
	}
	HW::HWCAMModel *cam_model = new HW::HWCAMModel();

	std::vector<std::string> all_files_path;
	cam_model->GetAllDirFiles(photo_dir, all_files_path);

	std::vector<std::string> current_images_path;
	for (int i = 0; i < all_files_path.size(); ++i)
	{
		if (all_files_path[i].find(".JPG") != std::string::npos)
			current_images_path.emplace_back(all_files_path[i]);
	}

	std::vector<std::string> cam_dir_files;
	cam_model->GetCurrentDirFiles(cam_dir, cam_dir_files);
	std::string cam_path;
	for (int i = 0; i < cam_dir_files.size(); ++i)
	{
		if (cam_dir_files[i].find(".xml") != std::string::npos)
		{
			cam_path = cam_dir_files[i];
		}
	}

	if (cam_path.empty())
	{
		std::cout << "no cams file existed..." << std::endl;
		delete cam_model;
		return;
	}
	//return;
	cam_model->ReadIntrinsicExtrinsic(cam_path);
	std::cout << "real paths size is: " << current_images_path.size() << std::endl;
	std::cout << "the file images paths is: " << cam_model->GetImagesPthsSize() << std::endl;
	//获取cam，然后转化cam
	cam_model->SetImageRatio(0.5);
	cam_model->SetImgOutDir(scene_dir);

	//std::cout << "asdfasdf" << std::endl;
	cam_model->ReadRealImagesPath(current_images_path);
	//std::cout << "asdfsdfsfasdfsdf" << std::endl;
	//
#if 0
	std::string cam_obj_dir = scene_dir + "/cams_obj/";
	if (_access(cam_obj_dir.c_str(), 0) == -1)
		mkdir(cam_obj_dir.c_str());
	std::vector<int> cams_idx;
	for (int i = 125; i < 200; i++)
		cams_idx.emplace_back(i);
	std::string cams_obj_path = cam_obj_dir + "cams100.obj";
	cam_model->DrawCamsPoseIntoOBJ(cams_idx, cams_obj_path);
	std::cout << "end save the cams!" << std::endl;
	return;
#endif

	cam_model->SaveCamsIntoCam();
	cam_model->SaveImagesIntoPNG();
	delete cam_model;
}
#endif

//------------------------------------------test----------------------------------//
bool MainWindow::SaveHWPCIntoObjFile(const std::vector<float3> vertices,
	const std::vector<float3> normals, const std::string& path)
{
	std::ofstream out_fh(path);
	if (vertices.empty())
	{
		printf("none vertices!\n");
		return false;
	}
	for (int i = 0; i < vertices.size(); ++i)
	{
		out_fh << "v " << vertices[i].x << " " << vertices[i].y << " " << vertices[i].z << std::endl;
	}
	for (int i = 0; i < normals.size(); ++i)
	{
		out_fh << "vn " << normals[i].x << " " << normals[i].y << " " << normals[i].z << std::endl;
	}
	return true;
}

bool MainWindow::SaveCCPCIntoObjFile(ccPointCloud* pc, const std::string& path)
{
	return true;
}

//------------------------------------------end test------------------------------//

