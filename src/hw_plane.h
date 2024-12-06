#pragma once
#include"CloudCompareDisplay\ccPlane.h"
#include"math_utils.h"
#include<iostream>
#include<pcl/console/parse.h>
#include<pcl/filters/extract_indices.h>
#include<pcl/io/pcd_io.h>
#include<pcl/point_types.h>
#include<boost/thread/thread.hpp>
#include<pcl/features/boundary.h>
#include<math.h>

#include<boost/make_shared.hpp>
#include <fstream>
#include <iostream>
#include <list>
#include <vector>

//opencv
#include<opencv2/opencv.hpp>

#include<Eigen/Dense>

#include<pcl/point_cloud.h>
#include<pcl/features/normal_3d.h>

#include<pcl/filters/covariance_sampling.h>
#include<pcl/filters/normal_space.h>
#include<pcl/kdtree/kdtree_flann.h>
#include<pcl/kdtree/kdtree.h>
//#include<pcl/segmentation/extract_polygonal_prism_data.h>
#include<pcl/filters/statistical_outlier_removal.h>

//Sor去噪
#include "hw_sor.h"
#include "CloudCompareDisplay/CloudSamplingTools.h"
#include "Common/PointCloudTpl.h"
#include "Common/ReferenceCloud.h"

//CGAL
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Alpha_shape_2.h>
#include <CGAL/Alpha_shape_vertex_base_2.h>
#include <CGAL/Alpha_shape_face_base_2.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/algorithm.h>
#include <CGAL/assertions.h>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
//#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Delaunay_mesher_2.h>
#include <CGAL/Delaunay_mesh_face_base_2.h>
#include <CGAL/Delaunay_mesh_size_criteria_2.h>

#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>
#include <CGAL/Polygon_2.h>
#include <iostream>

//ceres
//#include "ceres/ceres.h"
//#include "glog/logging.h"

//HW
#include"hw_object.h"

#include "HLBFGS.h"
#define HW_DEBUG 1

typedef CGAL::Exact_predicates_inexact_constructions_kernel  K;

typedef K::FT                                                FT;
typedef K::Point_2                                           Point_2d;
typedef K::Segment_2                                         Segment;

typedef CGAL::Alpha_shape_vertex_base_2<K>                   Vb;
typedef CGAL::Alpha_shape_face_base_2<K>                     Fb;
typedef CGAL::Triangulation_data_structure_2<Vb, Fb>          Tds;
typedef CGAL::Delaunay_triangulation_2<K, Tds>                Triangulation_2;
typedef CGAL::Alpha_shape_2<Triangulation_2>                 Alpha_shape_2;

typedef Alpha_shape_2::Alpha_shape_edges_iterator            Alpha_shape_edges_iterator;

struct FaceInfo2
{
	FaceInfo2() {}
	int nesting_level;

	bool in_domain() {
		return nesting_level % 2 == 1;
	}
};

struct PolygonExtractionParameters {
	bool operator==(const PolygonExtractionParameters &p) {
		if (p.alpha == alpha &&
			p.real_world_length_per_pixel == real_world_length_per_pixel &&
			p.dilate_half_width == dilate_half_width &&
			p.pixel_to_world_sampling_rate == pixel_to_world_sampling_rate &&
			p.smooth_region_diameter == smooth_region_diameter &&
			p.polygon_smoothing_mu == polygon_smoothing_mu &&
			p.max_theta == max_theta &&
			p.fitting_lines_pnts_number_threshold_ == fitting_lines_pnts_number_threshold_ &&
			p.min_corner_points_distance == min_corner_points_distance) {
			return true;
		}
		else {
			return false;
		}
	}

	void PrintMeOnly()
	{
		std::cerr << "alpha: " << alpha << std::endl;
		std::cerr << "real_world_length_per_pixel: " << real_world_length_per_pixel << std::endl;
		std::cerr << "dilate_half_width: " << dilate_half_width << std::endl;
		std::cerr << "pixel_to_world_sampling_rate: " << pixel_to_world_sampling_rate << std::endl;
		std::cerr << "smooth_region_diameter: " << smooth_region_diameter << std::endl;
		std::cerr << "polygon_smoothing_mu: " << polygon_smoothing_mu << std::endl;
		std::cerr << "max_theta: " << max_theta << std::endl;
		std::cerr << "fitting_lines_pnts_number_threshold_: " << fitting_lines_pnts_number_threshold_ << std::endl;
		std::cerr << "min_corner_points_distance: " << min_corner_points_distance << std::endl;
	}

	//alpha-shapes提边的参数
	//默认2*cloud_resolution_
	double alpha;

	//每个像素在真实世界中的长度，决定了图片大小和最终多边形的精度
	//默认0.01
	double real_world_length_per_pixel;

	//图像处理过程中的膨胀模板的大小
	//默认ave_pixels_
	int dilate_half_width;

	//从图像采样的采样率，表示每几个像素采样一个像素
	//默认ave_pixels_/2
	int pixel_to_world_sampling_rate;

	//平滑区域的直径
	//默认0.05
	double smooth_region_diameter;
	
	//多边形平滑的优化参数
	//默认1.0
	double polygon_smoothing_mu;

	//判断一个点的法线是否与前后两个点的法线平行的参数，如果超过这个值，则属于不平行
	//默认10
	double max_theta;

	//extract一个polygon，如果>3个顶点法向量较近，则fitting这些顶点
	int fitting_lines_pnts_number_threshold_;

	//如果多边形的相邻两个顶点之间的距离小于这个值，则用两个点的中点代替这两个点
	//默认ave_alpha_edge_len_
	float min_corner_points_distance;
};

void print_time(clock_t start_time, clock_t end_time, std::string name);

typedef CGAL::Triangulation_vertex_base_2<K>                      Vb2;
typedef CGAL::Triangulation_face_base_with_info_2<FaceInfo2, K>    Fbb;
typedef CGAL::Constrained_triangulation_face_base_2<K, Fbb>        Fb2;
typedef CGAL::Triangulation_data_structure_2<Vb2, Fb2>               TDS2;
typedef CGAL::Exact_predicates_tag                                Itag;
typedef CGAL::Constrained_Delaunay_triangulation_2<K, TDS2, Itag>  CDT;
typedef CDT::Point                                                Point2;
typedef CGAL::Polygon_2<K>                                        Polygon_2;
typedef CDT::Face_handle                                          Face_handle;


typedef CGAL::Delaunay_mesh_face_base_2<K> Fb1;
typedef CGAL::Triangulation_data_structure_2<Vb2, Fb1> Tds1;
typedef CGAL::Constrained_Delaunay_triangulation_2<K, Tds1>  CDT1;
typedef CGAL::Delaunay_mesh_size_criteria_2<CDT1>				  Criteria;
typedef CDT1::Point C1Point;
typedef CDT1::Vertex_handle                                        Vertex_handle;
//typedef CDT::                                        Edge_handle;

template <class OutputIterator>
void alpha_edges(const Alpha_shape_2& A, OutputIterator out)
{
	Alpha_shape_edges_iterator it = A.alpha_shape_edges_begin(),
		end = A.alpha_shape_edges_end();
	for (; it != end; ++it)
		*out++ = A.segment(*it);
}

template <class OutputIterator>
void alpha_vertices(const Alpha_shape_2& A, OutputIterator out)
{
	Alpha_shape_vertices_iterator it = A.Alpha_shape_vertices_begin(),
		end = A.Alpha_shape_vertices_end();
	for (; it != end; ++it)
		//*out++;
}

void mark_domains(CDT& ct,
	Face_handle start,
	int index,
	std::list<CDT::Edge>& border)
{
	if (start->info().nesting_level != -1) {
		return;
	}
	std::list<Face_handle> queue;
	queue.push_back(start);

	while (!queue.empty()) {
		Face_handle fh = queue.front();
		queue.pop_front();
		if (fh->info().nesting_level == -1) {
			fh->info().nesting_level = index;
			for (int i = 0; i < 3; i++) {
				CDT::Edge e(fh, i);
				Face_handle n = fh->neighbor(i);
				if (n->info().nesting_level == -1) {
					if (ct.is_constrained(e)) border.push_back(e);
					else queue.push_back(n);
				}
			}
		}
	}
}

//explore set of facets connected with non constrained edges,
//and attribute to each such set a nesting level.
//We start from facets incident to the infinite vertex, with a nesting
//level of 0. Then we recursively consider the non-explored facets incident 
//to constrained edges bounding the former set and increase the nesting level by 1.
//Facets in the domain are those with an odd nesting level.
void mark_domains1(CDT& cdt)
{
	for (CDT::Face_handle f : cdt.all_face_handles()) {
		f->info().nesting_level = -1;
	}

	std::list<CDT::Edge> border;
	mark_domains(cdt, cdt.infinite_face(), 0, border);
	while (!border.empty()) {
		CDT::Edge e = border.front();
		border.pop_front();
		Face_handle n = e.first->neighbor(e.second);
		if (n->info().nesting_level == -1) {
			mark_domains(cdt, n, e.first->info().nesting_level + 1, border);
		}
	}
}

class Cmp {
public:
	bool operator() (cv::Point const& _A, cv::Point const& _B) const
	{
		return _A.x < _B.x || (_A.x == _B.x && _A.y < _B.y);
	}
};


#define OUTPUT_ALPHA_SHAPE_EDGES //输出alpha-shapes得到的边缘
#define OUTPUT_IMAGE_EDGES //输出经过图像处理的边缘点
#define OUTPUT_IMAGE_EDGES_NORMAL //输出边缘及其法线
#define OUTPUT_IMAGE_EDGES_NORMAL_SMOOTHED //输出优化后的法线
#define OUTPUT_EDGES_SMOOTHED //输出优化后的边缘点
#define OUTPUT_POLYGON //输出最终得到的多边形

//不继承，改成组合
namespace HW
{

	//
	typedef std::array<float, 2> point2d_type;

	//原始polyogn的类型
	enum PolyonCanonicalType
	{
		kPolyConvex,
		kPolyNoConvex,
		kPolyConvexOther
	};

	//切割后的结构，用新的结构？
	enum SplitType
	{
		//只有一条直线剪切的时候有2个结构类型独有的
		kLOffPoly,
		kPolyOffL,

		//
		kLIM,
		kMIL,
		kLOffMOffL,
		kLIMOffL,
		kLIMidIL,
		
		kLIMDoor,
		kMILDoor,
		kLOffMOffLDoor,
		kLIMOffLDoor,
		kLIMidILDoor,

		kSPlitOther
	};

	typedef struct SplitNode
	{
		SplitType t;
		std::vector<int> s_pnts_idxs_;	//保存,需要的结构？
		std::vector<std::vector<int> > line_idxs_;	//这是多个线段组成,line_idxs_.size()为线段数量
		std::vector<bool> lines_direct_;	//true是正方向，false是负方向
		std::vector<int> t_line_idxs_;
		std::vector<Eigen::Vector3f> split_pre_cross_;	//这是和上一个node切开的点
		std::vector<Eigen::Vector3f> split_next_cross_;	//这是和下一个node切开的点

		SplitNode* next;
		SplitNode() :t(SplitType::kSPlitOther), next(NULL) {}
	} SplitNode;

	class HWPlane:public HWObject
	{
	public:

		typedef struct HWLineToPntsIdxs
		{
			Eigen::Vector3f line_fun_;
			std::vector<int> associated_pnts_idxs_;
		} HWLineToPntsIdxs;

		typedef struct HWOptiPnt2D
		{
			Eigen::Vector2f pos_;
			Eigen::Vector2f normal_ = Eigen::Vector2f(0.0, 0.0);
			Eigen::Vector3f color_ = Eigen::Vector3f(0.0, 0.0, 0.0);
		} HWOptiPnt2D;

		struct PolygonExtractionOptiEnergyWeights
		{
			float wu_lambda_ = 20.0;
			float wn_lambda_ = 0.0;
			float wa_lambda_ = 50.0;
			float ws_lambda_ = 10.0;
			float wc_lambda_ = 100.0;
		};

		struct PolygonNeighborEnergy
		{
			int lidx = -1;
			int ridx = -1;
			float energy_value = 0.0;
		};

		HWPlane();
		HWPlane(ccPlane* plane);
		//HWPlane(const HWPlane &plane);

		bool HasNormals();
		bool HasColors();
		
		~HWPlane();

		void SetPlaneName(std::string obj_name);
		void SetManuallyEdited() { manually_edited_ = true; }
		bool isManuallyEdited() { return manually_edited_; }

		void SetPlaneOutPutDir(const std::string& dir_name);

		bool Show();
		bool Save(std::string& file);
		bool SavePly(const std::string& file, const PlyFormat& type);
		bool ReadPly(const std::string& file);

		double GetArea() { return area_; }
		bool DrawImage(cv::Mat &image, const std::string& file, std::map<cv::Point2i, Point_2d, Cmp> &ptop, Eigen::Vector4d& params);
		
		/*
		目的：没有点云的情况下，设置投影到图像坐标系的参数，用于后续的图像投影操作
		*/
		bool SetImageProjCoordParams(bool cloud_flag);

		/*
		*/
		bool SetPolygonLSParams(bool cloud_flag);

		void SetPolyLSParamsAndImgProjFlag(bool flag);

		/*
		目的：提取的平面坐标系转化到图像坐标系上，用于后续的处理
		*/
		bool DrawImageFromCornerPnts2d(std::vector<Eigen::Vector2f>& pnts2d, cv::Mat& image, const std::string& file);
		
		/*
		目的：从平面坐标系转化图像坐标系，形成图像polygon
		*/
		bool DrawImageFromLineSegPnts2d(std::vector<Eigen::Vector2f>& pnts2d, cv::Mat& image, const std::string& file);

		void LiulingfeiPolygonExtraction(const std::string& file, bool all_steps);
		Eigen::Vector2d image_to_plane(int x, int y);
		Eigen::Vector2i PlanePnt2ImagePixel(Eigen::Vector2f& pnt2d);
		std::vector<cv::Point2i> GetEdgeFromImage(cv::Mat& left_image, std::vector<float2>& edge_points);
		void GetPolygonFromImage(cv::Mat& image, const std::map<cv::Point2i, Point_2d, Cmp>& ptop, const std::string& filename, const Eigen::Vector4d& params, bool all_steps);

		void GetPolygonFromImageNew(cv::Mat& image, const std::map<cv::Point2i, Point_2d, Cmp>& ptop, const std::string& filename, const Eigen::Vector4d& params, bool all_steps);

		bool FindPoint2iInPointVec(cv::Point2i pnt, std::vector<cv::Point2i>& pnts_vec);

		void WriteEdges2D(std::string filename);
		void WriteEdges3D(std::string filename);
		void WriteEdges3DWithoutNormals(std::string filename);
		void WriteOriginEdgesPnts3DByFlag(std::string filename, const std::vector<Eigen::Vector3f>& poly_pnts, 
			const std::vector<Eigen::Vector3f>& poly_pnts_normal, float len_normal, 
			std::vector<Eigen::Vector3f> poly_pnts_color,  std::vector<bool>& is_edge_vertex);
		void WritePolygonPnts2dIntoPolygonPnt3dWithNormals(const std::string filename, const std::vector<Eigen::Vector3f>& poly_pnts, 
			const std::vector<Eigen::Vector3f>& poly_pnts_normal, float len_normal);
		void WriteLinePnts2dIntoLinePnt3dWithNormals(const std::string filename, const std::vector<Eigen::Vector3f>& poly_pnts,
			const std::vector<Eigen::Vector3f>& poly_pnts_normal, float len_normal);
		void WriteSelectEdges3D(std::string filename, const std::vector<int>& selected_pnt_idx);
		void WriteSelectLines3D(std::string filename, const std::vector<int>& selected_pnt_idx);	//非polygon
		void WriteSelectEdges3DMoveFlag(std::string filename, const std::vector<int>& selected_pnt_idx, std::vector<bool>& movflagvec);
		void WriteSelectEdges2DMoveFlag(std::string filename, std::vector<Eigen::Vector2f>& poly2dpnts, std::vector<bool>& movflagvec);
		void WriteCornerPoints3D(std::string filename);
		void WriteCornerPoints2D(std::string filename);
		/*
		目的：debug，将3D的corner 顶点打印出来
		*/
		void ScoutCornerPoints3D();

		/*
		目的：debug，将2D的corner 顶点打印出来
		*/
		void ScoutCornerPoints2D();
		void ScoutPlaneVecIdxs(std::vector<int>& vecidxs);

		float GetCloudResolution() const { return cloud_resolution_;}
		PolygonExtractionParameters GetParams() const { return params_; }
		void SetParams(const PolygonExtractionParameters& p) { 
			std::cout << "params:" << std::endl;
			std::cout << "alpha: " << params_.alpha << ", " << p.alpha << std::endl;
			std::cout << "real_world_length_per_pixel: " << params_.real_world_length_per_pixel << ", " << p.real_world_length_per_pixel << std::endl;
			std::cout << "dilate_half_width: " << params_.dilate_half_width << ", " << p.dilate_half_width << std::endl;
			std::cout << "pixel_to_world_sampling_rate: " << params_.pixel_to_world_sampling_rate << ", " << p.pixel_to_world_sampling_rate << std::endl;
			std::cout << "smooth_region_diameter: " << params_.smooth_region_diameter << ", " << p.smooth_region_diameter << std::endl;
			std::cout << "polygon_smoothing_mu: " << params_.polygon_smoothing_mu << ", " << p.polygon_smoothing_mu << std::endl;
			std::cout << "max_theta: " << params_.max_theta << ", " << p.max_theta << std::endl;
			std::cout << "min_corner_points_distance: " << params_.min_corner_points_distance << ", " << p.min_corner_points_distance << std::endl;
			if (params_ == p) {
				std::cout << "new para is the same as old para" << std::endl;
			}
			else {
				std::cout << "different" << std::endl;
				if (params_.alpha != p.alpha) std::cout << "alpha different" << std::endl;
				if (params_.real_world_length_per_pixel != p.real_world_length_per_pixel) std::cout << "lengh_per_pixel different" << std::endl;
				if (params_.dilate_half_width != p.dilate_half_width) std::cout << "dilate_half_width different" << std::endl;
				if (params_.pixel_to_world_sampling_rate != p.pixel_to_world_sampling_rate) std::cout << "pixel_to_world_sampling_rate different" << std::endl;
				if (params_.smooth_region_diameter != p.smooth_region_diameter) std::cout << "smooth_region_diameter different" << std::endl;
				if (params_.polygon_smoothing_mu != p.polygon_smoothing_mu) std::cout << "polygon_smoothing_mu different" << std::endl;
				if (params_.max_theta != p.max_theta) std::cout << "max_theta different" << std::endl;
				if (params_.min_corner_points_distance != p.min_corner_points_distance) std::cout << "min_corner_points_distance different" << std::endl;
			}
			params_ = p;
		}
		PolygonExtractionParameters GetInitialParams() const { return initial_params_; }
		void extractPolygon();
		void extractPolygonAlpha();
		void extractPolygonOpti();
		void extractPolygonNoSmoothOpti();
		void SetFilename(std::string filename);
		float3 ComputeAverageCornerPnts3dPos();
		Eigen::Vector3f GetPlaneNormal();
		Eigen::Vector3f GetNormalFrom3Points(float3 a, float3 b, float3 c);
		Eigen::Vector3f GetNormalFrom3PntsEigen(Eigen::Vector3f& a, Eigen::Vector3f& b, Eigen::Vector3f& c);
		//
		float3 GetProj3DPnt(const float3& pnt);

		float2 Proj2Plane(float3& pnt);
		//将平面点云，投影到平面上，形成严格平面，但是必须先有平面方程
		void ProjectTo3DPlane();

		bool SaveMargedPlanePntsPly(const std::string& file, const PlyFormat& type);

		//从关联的点云去找满足平面条件的点云，然后保存下来
		void AddPlanePntIdxFromAssociatedPC(int idx);

		void AddPlanePnt(float x, float y, float z);
		void AddPlanePnt(float3 p);
		void SetPlanePnt(const std::vector<float3>& v);
		void AddPlanePntNormal(float x, float y, float z);
		void AddPlanePntNormal(float3 n);
		void SetPlanePntNormal(const std::vector<float3>& v);
		void SetPlanePntColor(const std::vector<float3>& c);
		void AddPlanePntColor(uchar r, uchar g, uchar b);
		void AddPlaneOriginPntColor(float3 c);
		void AddPolygonPnt(float3& n);

		const std::vector<float3>& GetOriginPnts();
		const std::vector<float3>& GetOriginPntsNormal();
		const std::vector<float3>& GetOriginPntsColor();
		const std::vector<float3>& GetPlanePnts();
		const std::vector<float3>& GetPlanePntsNormal();
		const std::vector<uchar3>& GetPlaneColors();
		const std::vector<float2>& GetPlaneEdgePnts();
		const std::vector<float2>& GetPlaneEdgePntsNormals();
		const std::vector<uchar3>& GetPlaneEdgePntsColors();
		int GetPlaneSemanticLabel();
		const float& GetPlaneWidth();
		const float& GetPlaneHeight();
		const std::vector<float2>& GetOuterPolygon2D();
		const std::vector<float2>& GetPolygonUV();
		const std::vector<float3>& GetOuterPolygon();
		Eigen::Vector3f GetOuterPolygonCenterPnts3D();
		const std::vector<float3>& GetAllPolygonVertices();
		const std::vector<float3>& GetAllRefinedPolygonVertices();
		const std::vector<float2>& GetAllRefinedPolygonUV();
		const std::vector<float3>& GetAllRefinedPolygonNormals();
		const std::vector<int3>& GetAllRefinedPolygonTris();
		const std::vector<int3>& GetTriangles();
		const std::vector<int>& GetLines();
		const std::vector<float>& GetPolygonAngels();
		const std::vector<float>& GetMinDists();
		const std::vector<Segment> GetBorderEdges();
		const Eigen::Matrix4f& GetCameraPose();
		const Eigen::Matrix4f& GetWorld2PlaneMatrix();
		void GetBoundingBox(Eigen::Vector3f& boxMin, Eigen::Vector3f& boxMax);
		bool fuseNearbyPnts();
		bool deleteCollinearPnts();

		const std::vector<float3>& GetSampledWorldPnts();
		
		Eigen::Vector3f GetOuterPolygonPnt3d(int i);

		//获取平面坐标系下的坐标
		const std::vector<float3>& GetPlaneCoordPos();
		float4 GetPlaneCoeff();

		bool HasEdgePnts();

		const ccPlane* GetAssociatedPlane();
		void SetAssociatedPlane(ccPlane* plane);

		//计算点云的平均密度
		float ComputeCloudResolution(const pcl::PointCloud<pcl::PointXY>::Ptr &cloud);
		float ComputePlaneCloudResolution2D(const pcl::PointCloud<pcl::PointXY>::Ptr &plane_cloud);

		//计算每个平方多少点云
		int ComputePointsNumPerSquareMeter();
		int EstimateBordersFromPcl(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, float re, float reforn);	//这个需要用户设置搜索半径，和角度（暂且弃用）
		bool DoEstimateBordersFromPcl();
		int EstimateBordersFromPclAngle(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::Normal>::Ptr & cloud_normal,
			int seach_k, float angle_ratio); // 只需要设置搜索近邻和角度比（MPI*angle_ratio）
		int EstimateBordersFromPclRadius(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::Normal>::Ptr & cloud_normal,
			float seach_radius, float angle_ratio);	//
		bool DoEstimateBordersFromPclAngle();

		//对边界边进行聚类
		void SortBorderEdges();
		
		void ComputeSortedVerticesListWeights();

		bool GenerateWorldCoordToPlaneCoordMatrix();
		bool GenerateWorldCoordToPlaneCoordMatrixByCornerPnts3d();

		void Generate2DPlane();

		bool DoEstimateBorderFrom2DPlane();

		bool DoEstimateBorderFrom2DPlaneOptiAlpha();
		void SortBorderEdgesFromPlaneOptiAlpha();
		bool GetPolygonPointFromSortedBorderEdges();

		float2 TransformWorld2Plane(Eigen::Vector3f& world_coord);

		void initialAfterLoad();

		/*
		读取polygon的三维坐标,然后计算它们的平面坐标系，及其它们的转换
		*/
		void SetPolygonPnts3dFromInitialPnts(std::vector<Eigen::Vector3f>& polygon3d_pnts);
		Eigen::Vector2f FittingLine(std::vector<int>& neighs);
		Eigen::Vector2f FittingLine(std::vector<float2> edge_pnts, std::vector<int>& neighs);
		Eigen::Vector2f FittingLine(float2& p1, float2& p2);
		Eigen::Vector2f FittingLineEigen(Eigen::Vector2f& p1, Eigen::Vector2f& p2);
		Eigen::Vector3f FittingLineFromPntsIdxs(const std::vector<int>& idxs);

		/*
		目的：通过两个顶点计算线的方程ax+by+c=0这样可以保证处理所有的线
		*/
		Eigen::Vector3f CreateLineFunctionFrom2Pnts(float2& p1, float2& p2);
		Eigen::Vector3f GetIntersectionPoint3dWithLine(Eigen::Vector3f L_dir, Eigen::Vector3f L_pnt);
		float2 GetIntersectionPoint(Eigen::Vector2f& l1, Eigen::Vector2f& l2);
		float dot(float2 p1, float2 p2);
		float dot(float3 p1, float3 p2);
		float PointPointDist(float2& p1, float2& p2);
		float PointLineDist(float2& p, Eigen::Vector2f& coeff);
		float PointLineSegmentDist(float2& p, float2& p1, float2& p2);
		float2 ProjectToLine(float2& p, Eigen::Vector2f& coeff);
		void SetSemanticLabel(int label);
		void SetDiameter(float diameter_max);
		void SetCornerPt(float2 pnt, int idx);
		void SetCornerPt3d(Eigen::Vector3f& pnt3d, int idx);
		void SetCornerPts(std::vector<float3>& corner_pnts_3d);
		void SetPlaneCornerPts(const std::vector<float3>& corner_pnts_3d);
		void SetMaxPolyDistThreshold(float r);
		void UpdateCornerPts(int idx1, float2& p1, int idx2, float2& p2,int idx,std::vector<int>& nomove_idxs, Eigen::Vector3f & L_dir, Eigen::Vector3f & L_point);
		
		/*
		目的：对新的plygon进行处理，用于后续的操作
		*/
		void UpdateCornerPtsFromNewPolygon(std::vector<Eigen::Vector2f>& polyon_new);
		void SetRefineTriMaxLen(float& tri_len);
		void SeetRefineTriMinDegree(float& tri_degree);

		//
		void SaveCurrentStateData();
		void BackToLastState();
		void SetCameraPose(Eigen::Matrix4f& camera_pose);
		void GetSmoothRegions();
		void NewSmoothRegions();
		void GetNeighbors();
		void GetInitialNormals();
		float GetDistToPlane(float3 pt);
		/*void OptimizeByHLBFGS(int N, double *init_x, int num_iter, int M, int T,void (*evalfunc)(int, double*, double*,
			double*, double*));*/
		double GetAngle(double x1, double y1, double x2, double y2);
		double GetTheta(double x1, double y1, double x2, double y2);
		float GetTheta(float3 e1, float3 e2);
		double GetWeight(double x1, double y1, double x2, double y2);
		void Newiteration(int iter, int call_iter, double *x, double* f, double *g, double* gnorm);
		void EvalfuncForNormalEstimation(int N, double* x, double *prev_x, double* f, double* g);
		void DoNormalEstimation();
		void EvalfuncForPolygonSmoothing(int N, double* x, double *prev_x, double* f, double* g);
		void DoPolygonSmoothing(std::string filename);
		void NewPolygonSmoothing(std::string filename);
		void DoPolygonExtraction();
		void NewPolygonExtraction();
		void NewPolygonExtractionZDG();
		void NewPolygonInitialExtractionZDG();
		void NewPolygonExtractionWithoutSmoothZDG();
		void CopyExractionEdgePntsPosToInitialEdgePnts();
		void OptimizeNewPolygonExtraction(const std::vector<HWOptiPnt2D>& pnts,
			std::vector<bool> is_edges_pnts, const std::vector<HWLineToPntsIdxs>& fun_coeffs,
			std::vector<HWLineToPntsIdxs>& fun_coeffs_opti);
		bool PolygonExtractionFromSortedLinesFunctions(const std::vector<Eigen::Vector3f>& lines_funcs, std::vector<Eigen::Vector2f>& poly_pnts);
		void ComputeSortedAllNodeNeighborEnergysFromHWLineToPntsIdxs(const std::vector<HWOptiPnt2D>& pnts, const std::vector<HWLineToPntsIdxs>& fun_coeffs,
			std::vector<PolygonNeighborEnergy>& all_neighbor_energys);
		void ComputeAllNodeNeighborLinesAngleEnergysFromHWLineToPntsIdxsIncrease(const std::vector<HWOptiPnt2D>& pnts, const std::vector<HWLineToPntsIdxs>& fun_coeffs,
			std::vector<PolygonNeighborEnergy>& all_neighbor_energys);
		void ComputeSingleOptiPntLinePntsAssociatedPntsNormal(const std::vector<HWOptiPnt2D>& pnts, const std::vector<HWLineToPntsIdxs>& fun_coeffs, int pidx, Eigen::Vector2f& pnt_normal);
		void ComputeEnergyNodeMeFunFromItsIdxs(const std::vector<HWOptiPnt2D>& pnts, HWLineToPntsIdxs& fun_coeff);

		float EuPolygonPntsToPolygonLinesDistances(const std::vector<HWOptiPnt2D>& pnts_in,
			const std::vector<HWLineToPntsIdxs>& fun_coeffs);
		float EuPolygonPntsToPolygonLineDistance(const std::vector<HWOptiPnt2D>& pnts_in,
			const HWLineToPntsIdxs& fun_coeff);
		float EnPolygonPntsNormalToPolygonLinesNormal(const std::vector<HWOptiPnt2D>& pnts_in,
			const std::vector<HWLineToPntsIdxs>& fun_coeffs);
		float EnPolygonPntsNormalToPolygonLineNormal(const std::vector<HWOptiPnt2D>& pnts_in,
			const HWLineToPntsIdxs& fun_coeff);
		float EaPolygonArea(const std::vector<HWOptiPnt2D>& pnts_in, const std::vector<HWLineToPntsIdxs>& fun_coeffs);
		float EsPolygonLinesNum(const std::vector<HWLineToPntsIdxs>& fun_coeffs, const std::vector<HWLineToPntsIdxs>& initial_fun_coeffs);
		float EcPolygonPntsInPolygonNum(const std::vector<HWOptiPnt2D>& pnts_in, const std::vector<int>& remained_pnts_idxs);
		float ComputeAllEnergyValueFromPolygonExtractedPnts(const std::vector<HWOptiPnt2D>& pnts_in, const std::vector<HWLineToPntsIdxs>& fun_coeffs, const std::vector<int>& remained_pnts_idxs);
		float ComputeNeighborPairsEuncEnergyValue(const std::vector<HWOptiPnt2D>& pnts_in, const std::vector<HWLineToPntsIdxs>& fun_coeffs, PolygonNeighborEnergy& lines_pairs_energy);
		void InsertHWLinesPairEnergyIntoHWLinesPairsVecByOrder(std::vector<PolygonNeighborEnergy>& neighbors_vec, const PolygonNeighborEnergy& lines_pairs);
		void DeleteHWLinesPairEnergyFromHWLinesPairs(std::vector<PolygonNeighborEnergy>& neighbors_vec, int idx);
		bool CirclePntsIdxsHasZeroFlag(const std::vector<int>& circle_idxs);
		void ComputeAllLinesFunsRemainedPntsIdxsFromPnts(const std::vector<HWOptiPnt2D>& pnts_in, const std::vector<HWLineToPntsIdxs>& fun_coeffs, 
			std::vector<int>& all_remained_pnts_idxs);
		void ComputeNeighborLinesFunsRemainedPntsIdxs(const std::vector<HWOptiPnt2D>& pnts_in, const std::vector<HWLineToPntsIdxs>& fun_coeffs,
			int lidx, int ridx, std::vector<int>& neighbor_remained_pnts_idxs);

		float PolygonLinesExchangePolygonLinePnts(const std::vector<HWOptiPnt2D>& pnts_in, const std::vector<HWLineToPntsIdxs>& fun_coeffs, const std::vector<int>& remained_pnts_idxs,
			int l_idx, int r_idx, std::vector<HWLineToPntsIdxs>& fun_coeffs_opti, std::vector<int>& remained_pnts_idxs_opti);
		float PolygonLineSplitIntoTwoPolygonLines(const std::vector<HWOptiPnt2D>& pnts_in, const std::vector<HWLineToPntsIdxs>& fun_coeffs, const std::vector<int>& remained_pnts_idxs,
			int p_idx, std::vector<HWLineToPntsIdxs>& fun_coeffs_opti, std::vector<int>& remained_pnts_idxs_opti);
		float PolygonLineMergeTwoPolygonIntoPolygonLine(const std::vector<HWOptiPnt2D>& pnts_in, const std::vector<HWLineToPntsIdxs>& fun_coeffs, const std::vector<int>& remained_pnts_idxs,
			int l_idx, int r_idx, std::vector<HWLineToPntsIdxs>& fun_coeffs_opti, std::vector<int>& remained_pnts_idxs_opti);
		float PolygonLineInsertLineIntoTwoPolygonLine(const std::vector<HWOptiPnt2D>& pnts_in, const std::vector<HWLineToPntsIdxs>& fun_coeffs, const std::vector<int>& remained_pnts_idxs,
			int l_idx, int r_idx, std::vector<HWLineToPntsIdxs>& fun_coeffs_opti, std::vector<int>& remained_pnts_idxs_opti);
		static bool ComparePolygonNeighborEnergyValueDecrease(const PolygonNeighborEnergy& a, const PolygonNeighborEnergy& b);
		static bool ComparePolygonNeighborEnergyValueIncrease(const PolygonNeighborEnergy& a, const PolygonNeighborEnergy& b);

		void InnerPolygonExtraction();
		void SingleInnerPolygonExtraction(int index);
		void CalcMinDists();

		bool DoPointMoving(int idx, Eigen::Vector3f& dest, float max_dist=FLT_MAX);
		bool DoPointMoving(int idx, Eigen::Vector3f & L_dir, Eigen::Vector3f & L_point, float max_dist=FLT_MAX);
		void DoPointMoving(int idx1,int idx2, Eigen::Vector3f & L_dir, Eigen::Vector3f & L_point);
		void DoPointMovingDirect(int idx, Eigen::Vector3f& dest, float max_dist=FLT_MAX);
		//将polygon中的p_idx的顶点移动到ray_pnt位置上
		bool DoMovePnt2dToNewPnt2d(int p_idx, Eigen::Vector2f& ray_pnt);
		void DoPointCreating(int idx, Eigen::Vector3f & L_dir, Eigen::Vector3f & L_point);
		//在线段ls_idx, le_idx上添加顶点pnt2d
		void DoPointCreateOnLineSeg(int ls_idx, int le_idx, Eigen::Vector2f& pnt2d);
		//添加顶点pnt2d到polygon边缘上,如果不在，就投影，如果太远就不处理
		bool PlanePntProjPolygonLine2D(Eigen::Vector2f& pnt2d, float& threshold_dist, Eigen::Vector2i& line_pnt_idx, Eigen::Vector2f& proj_pnt);
		
		void DoPolygonNormalsAlignPntsNormals();
		void DoPolygonInversing();
		void DoPolygonDeleting();
		void DoPointDeleting(int idx);
		void DoPointDeleting(int start,int end);
		void DoPointAdding(float2 pnt,int idx, bool is_before);
		void GetPolygonFromPnts();
		int IsInPolygon(float2& p);
		int IsInSelectedPolygonNew(std::vector<Eigen::Vector2f>& polypnts2d, Eigen::Vector2f& p);
		int IsInPolygon(float3& p);
		int IsPointInPolygon3D(Eigen::Vector3f& pnt);
		int IsConcaveVertex(int idx);
		int IsConcaveVertex(const std::vector<float2> &corner_points,int idx);
		int IsConcaveVertexNew(const std::vector<Eigen::Vector2f> &corner_points, int idx);
		float Pnt2PolygonMinDist2D(Eigen::Vector2f p);

		/*
		目的：判断顶点p是否在polygon中,它们在同一个坐标系
		*/
		int IsPInPolygonVec(Eigen::Vector2f& p, std::vector<Eigen::Vector2f>& in_polygon);

		/*
		目的：获取polygon中的凹点的索引
		*/
		void GetPolygonConcaveVertex2DIdx(std::vector<int>& p_idxs);

		/*
		目的：获取polygon中的凹点的顶点
		*/
		void GetPolygonConcaveVertex2DPnts(std::vector<Eigen::Vector2f> p_pnts);
		int IsConcaveEdgePoint(int idx);
		int IsConcaveEdgePoint(const std::vector<float2>& points, int idx);
		int PointMatchingJudge(float3& point,float rp,float& dist);
		float GetEdgesAngel(float3& v1, float3& v2, int edge_idx1, int edge_idx2);
		float GetDistToPolygon(Eigen::Vector3f& p);
		void DoTriangulation();
		void DoTriangulationRefined();	//最新版本的三角化文件
		void ComputeTriNormalRefined();	//最新版本的三角化文件,无效
		void ComputeTriPntsUVRefined();	//最新版本的三角化文件的uv值
		void UpdateInformation();
		void DoPolygonSampling(std::vector<float3>& sample_vertices, int density);
		float RayToPolygonDist(Eigen::Vector3f& ray_o, Eigen::Vector3f& ray_direct,int& nearest_pnt_idx);
		////计算ray的顶点和其最近的顶点
		//bool ComputeRayPntMindist2PolyPnts(Eigen::Vector3f& ray_o, Eigen::Vector3f& ray_direct,
		//	float& min_dist, int& nearest_pnt_idx);
		//对选择的polygon 它所在的平面和射线ray_o，ray_direct进行求交，得到它的坐标
		bool RayToPolyPlanePntPos(Eigen::Vector3f& ray_o, Eigen::Vector3f& ray_direct, Eigen::Vector2f& proj_p);
		bool RayToPolyIntersectionPnt(Eigen::Vector3f& ray_o, Eigen::Vector3f& ray_direct, Eigen::Vector3f& pnt);
		bool RayToPolyPlaneIntersectionPntConst(const Eigen::Vector3f& ray_o, const Eigen::Vector3f& ray_direct, Eigen::Vector3f& pnt);
		//射线和polygon进行相交
		bool RayLinePnt2PolygonPntThreshold(const Eigen::Vector3f& oc, const Eigen::Vector3f& odir, Eigen::Vector3f& pnt3d);
		bool IsDegeneration(int idx, Eigen::Vector3f& L_dir, Eigen::Vector3f& L_point);
		/*
		目的：如果true表示，这个点可以投影到直线上，它特殊处理了凹点的情况
		*/
		bool IsDegenerationIgnoreConvexPnts(int idx, Eigen::Vector3f& L_dir, Eigen::Vector3f& L_point);
		
		/*
		目的：判断线段是否在vec线段中
		*/
		bool FindLineSegIdxInLSVec(std::vector<Eigen::Vector2i>& vec, Eigen::Vector2i& l);

		/*
		目的：获取polygon中的线段i,j的长度
		*/
		float GetPolyon3DLength(int i, int j);

		/*
		目的：判断这个value是否在vec中
		*/
		bool FindValueInVec(std::vector<int>& vec, int v);

		/*
		目的：有序的顶点索引，构成polygon的边,不构成polygon环状
		*/
		void BuildLSIdxFromPolygonIdxsVec(std::vector<int>& polygon_idxs, std::vector<Eigen::Vector2i>& linesegidx);

		/*
		目的：计算polygon相对一个线的两个端点
		*/
		void ComputeTwoPointsIdxCorrespondingLine(Eigen::Vector3f& L_dir, Eigen::Vector3f& L_point, Eigen::Vector2i& pnts_idxs);

		/*
		目的：计算polygon相对一个线最左最右的两个端点 2d的坐标系
		*/
		void ComputeLeftRightPointsIdxLine2D(Eigen::Vector2f& L_dir2d, Eigen::Vector2f& L_point2d, Eigen::Vector2i& pnts_idxs);
		/*
		目的：计算polygon相对一个线最左最右的两个端点 2d的坐标系
		*/
		bool ComputeLeftRightPointsPosLine2D(Eigen::Vector2f& L_dir2d, Eigen::Vector2f& L_point2d, 
			Eigen::Vector2i& pnts_idxs, Eigen::Vector2f& lpnt, Eigen::Vector2f& rpnt);

		/*
		目的：计算polygon相对一个线最左最右的两个端点 2d的坐标系
		*/
		void ComputeLRPointsWithFlagIdxLine2D(Eigen::Vector2f& L_dir2d, Eigen::Vector2f& L_point2d, 
			std::vector<bool>& valided_vec, Eigen::Vector2i& pnts_idxs);

		/*
		目的：输入一个顶点，计算它投影于直线后相对直线的顶点L_point2d的相对位置，返回为float
		输入：inpnt2d；直线： ldir2d，lpnt2d；输出：相对位置,相对L_point2d
		*/
		float ComputeProjPRelativeDist2LPntDir2d(Eigen::Vector2f& pnt2d, Eigen::Vector2f& L_dir2d, Eigen::Vector2f& L_point2d);

		/*
		目的：计算polygon相对一个线最左最右的两个端点 2d的坐标系,输入的polygon idxs, 输出split polygon的最左右最有两个端点
		*/
		void ComputeLRPntsIdxLine2DFromSelectPoly(Eigen::Vector2f& L_dir2d, Eigen::Vector2f& L_point2d, 
			std::vector<int>& split_idxs, Eigen::Vector2i& pnts_idxs);

		/*
		目的：输入L_dir,L_point, polygon是自己的corner_pnts_, 输出不用移动的索引
		*/
		void ComputePolygonMoveIdxs2Line(Eigen::Vector3f& L_dir, Eigen::Vector3f& L_point, std::vector<int>& no_move_idxs);

		/*
		目的：获取polygon的需要移动的顶点,polygon是自己的corner_pnts_
		*/
		bool ComputePolygonMoveIdxsFromline2d(Eigen::Vector2f& ldir2d, Eigen::Vector2f& lpnt2d, std::vector<int>& move_idxs);

		/*
		目的：输入L_dir,L_point, polygon是自己的corner_pnts_, 被选择的polygon的顶点的idx，输出不用移动的索引
		*/
		void ComputeSelectedPolygonMoveIdxs2Line(Eigen::Vector3f& L_dir, Eigen::Vector3f& L_point, 
			std::vector<int>& split_idxs, std::vector<int>& no_move_idxs, Eigen::Vector2i& my_lr_idx);

		/*
		目的：输入L_dir,L_point, polygon是自己的corner_pnts_, 被选择的polygon的顶点的idx，输出不用移动的索引,以及移动的那个顶点
		的索引，它是dists2l_flag，my_lr_idx 是基于 selected_idxs的索引
		*/
		void ComputeSelectedPolygonNoMoveIdxs2LineSeam(Eigen::Vector2f& l_dir2d, Eigen::Vector2f& l_pnt2d, std::vector<int>& selected_idxs, float dist_threshold, 
			std::vector<bool>& dists2l_flag, Eigen::Vector2i& my_lr_idx);

		/*
		目的：输入ls,le, polygon是自己的corner_pnts_, 被选择的polygon的顶点的idx，输出不用移动的索引,以及移动的那个顶点
		的索引,它是dists2l_flag，my_lr_idx 是基于 selected_idxs的索引
		*/
		void ComputeSelectedPolygonNoMoveIdxs2LineSegmentSeam(Eigen::Vector2f& ls, Eigen::Vector2f& le, std::vector<int>& selected_idxs, float dist_threshold,
			std::vector<bool>& dists2l_flag, Eigen::Vector2i& my_lr_idx);

		/*
		目的：输入L_dir,L_point, polygon是自己的corner_pnts_, 被选择的polygon的顶点的idx，输出不用移动的索引,以及移动的那个顶点
		的索引
		*/
		void ComputePolygonMoveLRIdxs2LineSeam(Eigen::Vector2f& l_dir2d, Eigen::Vector2f& l_pnt2d, float dist_threshold, Eigen::Vector2i& my_lr_idx);

		/*
		目的：输入L_dir,L_point, polygon是自己的corner_pnts_, dists2l_flag, 通过这些polygon获取新的polygon pnts
		的索引
		*/
		void ComputeAllPolyNewPntsFromNoMoveIdxs2DSeam(Eigen::Vector2f& l_dir2d, Eigen::Vector2f& l_pnt2d, 
			std::vector<bool>& dists2l_flag, std::vector<Eigen::Vector2f>& polygonnewpnts);

		/*
		目的：输入L_dir,L_point, polygon是自己的selected polygon corner_pnts_, dists2l_flag, 通过这些polygon获取新的polygon pnts
		的索引
		*/
		void ComputeSelectPolyNewPntsFromNoMoveIdxs2DSeam(Eigen::Vector2f& l_dir2d, Eigen::Vector2f& l_pnt2d,
			std::vector<int>& poly_idxs, std::vector<bool>& dists2l_flag, std::vector<Eigen::Vector2f>& polygonnewpnts);

		/*
		目的：输入source_idxs target_idxs, 输出是这两个source_idxs到target_idxs在整个polygon的连接处的idx
		*/
		void ComputeTwoPolygonConnectIdx(std::vector<int>& source_idxs, std::vector<int>& target_idxs, 
			Eigen::Vector2i& connect_idxs);

		/*
		目的：获取当前的plane和目标的plane j的相交线
		输入：目标的平面的索引
		输出：相交的直线
		*/
		void GetExpandTgtIntersectionLine(int j, Eigen::Vector3f& L_dir, Eigen::Vector3f& L_point);

		/*
		目的：获取当前的plane和目标的coeff的相交线
		输入：目标的平面的索引
		输出：相交的直线
		*/
		void GetExpandTgtCoeffIntersectionLine(float4& coeff, Eigen::Vector3f& L_dir, Eigen::Vector3f& L_point);

		/*
		目的：输入L_dir,L_point, polygon是自己的corner_pnts_, 输出被直线分割的两个部分。这些非常重要，用于后续的expand操作
		*/
		bool SplitPolygonFromIntersectionLineNew(Eigen::Vector3f& L_dir, Eigen::Vector3f& L_point,
			std::vector<int>& split_idxs1, std::vector<int>& split_idxs2);

		/*
		目的：输入ldir2d,lpnt2d, polygon是自己的poly_idx, 输出被直线分割的两个部分。split_idxs1, split_idxs2是保存poly_idx下标
		*/
		bool SplitSelectedPolyFromInterLNew(Eigen::Vector2f& ldir2d, Eigen::Vector2f& lpnt2d, std::vector<int>& poly_idx,
			std::vector<int>& split_idxs1, std::vector<int>& split_idxs2);

		/*
		目的：输入L_dir,L_point, polygon是自己的corner_pnts_, 输出被直线分割的两个部分。这些非常重要，用于后续的expand操作
		*/
		bool SplitPolygonIntersectionFromLine(Eigen::Vector3f& L_dir, Eigen::Vector3f& L_point, 
			std::vector<int>& split_idxs1, std::vector<int>& split_idxs2);

		bool SplitSelectPolyFromSplitLine(Eigen::Vector2f& ldir2d, Eigen::Vector2f& lpnt2d, std::vector<int>& selected_poly, 
			std::vector<int>& splitidx1, std::vector<int>& splitidx2);

		bool ComputeNearestLineCrossPnts(Eigen::Vector2f& ldir2d, Eigen::Vector2f& lpnt2d, 
			Eigen::Vector2i& pnts_idxs, Eigen::Vector2f& cross_pnt);

		bool ComputeCrossLSIdxFromRect(std::vector<Eigen::Vector2f>& rectpnts, std::vector<int>& polygonidxs,
			Eigen::Vector2i& start_l, Eigen::Vector2i& end_l);
		/*
		目的：对rect是否交叉计算, 可能有两个交点
		*/
		int ComputeCrossPntFromRect(std::vector<Eigen::Vector2f>& rectpnts, 
			Eigen::Vector2f& ls, Eigen::Vector2f& le, Eigen::Vector2f& crosspnt0, Eigen::Vector2f& crosspnt1);

		bool ComputeSENearestLineCrossPnts(Eigen::Vector2f& ldir2d, Eigen::Vector2f& lpnt2d,
			Eigen::Vector2i& ls_pnts_idx, Eigen::Vector2f& ls_cross_pnt, 
			Eigen::Vector2i& le_pnts_idx, Eigen::Vector2f& le_cross_pnt);

		bool CheckLineSplitSrcPolygon(Eigen::Vector2f& ldir2d, Eigen::Vector2f& lpnt2d);
		
		/*
		目的：处理这个hf_poly_pnts 和 直线有无交点
		*/
		bool ComputeLine2NearestHalfPolyCrossPnt(Eigen::Vector2f& ldir2d, Eigen::Vector2f& lpnt2d, 
			std::vector<Eigen::Vector2f>& hf_poly_pnts, Eigen::Vector2f& cross_pnts);
		
		//bool CheckLineSplitSrcPolyNode(SplitNode* srcnode, Eigen::Vector2f& ldir2d, Eigen::Vector2f& lpnt2d);
		//bool CheckLineSegSplitSrcPolygon(Eigen::Vector2f& ls, Eigen::Vector2f& le);

		/*
		目的：通过t polygon 和 本身的位置，判断Src polygon是属于哪种类型
		*/
		void CheckSrcPolygonCanonicalType();

		/*
		目的：
		*/
		void DrawSrcPolygon2DImgCoord(cv::Mat& img);
		void DrawTgtPolygonLine2DImgCoord(cv::Mat& img);

		/*
		目的：计算原始的polygon 面积和它宽度；计算Tgt polygon的面积和宽度
		*/
		void ComputeSrcAndTgtParamsOnSrcCoord();

		/*
		目的：计算原始的polygon 面积和它宽度；计算Tgt polygon的面积和投影的线段
		输入：idx 当前的idx,整个model_planes_ 输出：面积poly_area, 线段, ls, le
		*/
		void ComputePolyParamsOnSrcCoord(int idx, float& poly_area, 
			Eigen::Vector2f& ls, Eigen::Vector2f& le);

		/*
		目的：输入ldir2d,lpnt2d,对corner_pnts_进行切割，形成两个半polygon,每个都是polygon
		输入：ldir2d,lpnt2d
		输出：lnode,rnode
		*/
		bool LineSplitSrcPoly(Eigen::Vector2f& ldir2d, Eigen::Vector2f& lpnt2d, 
			SplitNode* lnode, SplitNode* rnode);
		
		/*
		目的：输入ls,le 线段, 对corner_pnts_进行切割，形成两个半polygon,每个都是polygon
		输入：lidx 为t多边形的索引,线段ls,le为lidx多边形投影到原始多边形的线段
		输出：lnode,rnode
		*/
		bool LineSegSplitSrcPoly(int lidx, Eigen::Vector2f& ls, Eigen::Vector2f& le,
			SplitNode* lnode, SplitNode* rnode);

		/*
		目的：对corner_pnts_进行排序，最左下角开始排序
		输入：corner_pnts_
		输出：排序后的corner_pnts_new_
		*/
		void SortCornerPnts();

		/*
		目的：计算corner_pnts_的bounding box
		*/
		void ComputeCornerPnts3dBoxPos(Eigen::Vector3f& mincor, Eigen::Vector3f& maxcor);
		void ComputeCornerPnts2dBoxPos(Eigen::Vector2f& mincor, Eigen::Vector2f& maxcor);

		/*
		目的：计算corner_pnts_最左下角的顶点（近似），后面按照需求来处理这些最左还是最下等问题，
		它距离bounding box 最左最下角的顶点最近
		*/
		int ComputeApproxMinCorIdx();

		/*
		目的：输入ldir2d,lpnt2d,对corner_pnts_进行切割对剩余的SplitNode进行split
		输入：ldir2d,lpnt2d,node,idx表示第几个t polygon,它用于后续的剪切约束。
		输出：lnode,rnode
		*/
		bool LineSplitSrcNode(int idx, Eigen::Vector2f& ldir2d, Eigen::Vector2f& lpnt2d,
			SplitNode* node, SplitNode* lnode, SplitNode* rnode);

		/*
		目的：输入L_dir,L_point, polygon是自己的corner_pnts_, 输出被直线切割的线段
		*/
		bool ComputeWholePolyInsectLIdxsFromLine(Eigen::Vector3f& L_dir, Eigen::Vector3f& L_point, 
			Eigen::Vector2i& line1pidx, Eigen::Vector2i& line2pidx, Eigen::Vector2f& l1_pnt, Eigen::Vector2f& l2_pnt);

		/*
		目的：输入L_dir,L_point, polygon是自己的corner_pnts_, 输出被直线切割的线段
		*/
		bool AddSplitPntsIntoSplitPoly(std::vector<int>& select_polygon, Eigen::Vector2i& line1pidx, Eigen::Vector2i& line2pidx,
			Eigen::Vector2f& l1_pnt, Eigen::Vector2f& l2_pnt, std::vector<Eigen::Vector2f>& poly_addedpnts);

		/*
		目的：输入L_dir,L_point, polygon是自己的, 输出被直线切割的线段
		*/
		bool ComputeSplitPolyInsectLDistToLine(Eigen::Vector3f& L_dir, Eigen::Vector3f& L_point, Eigen::Vector2f& m_pnt, Eigen::Vector2f& m_dir,
			std::vector<Eigen::Vector2f>& polypnts2d, Eigen::Vector2f& dist_near_far, Eigen::Vector2i& near_far_idx);

		/*
		目的：输入L_dir,L_point, polygon是自己的split_idxs, 输出被直线分割的两个部分。这些非常重要，用于后续的expand操作
		*/
		bool ComputeSplitPolygonToIntersectionLine(Eigen::Vector3f& L_dir, Eigen::Vector3f& L_point,
			std::vector<int>& split_idxs, float& polygon_dist);

		/*
		目的：输入L_dir,L_point, polygon是自己的split_idxs, 输出这个polygon距离L_dir 直线的距离。
		这些非常重要，用于后续的expand操作
		*/
		bool ComputePolygonRangeDistToIntersectionLineNew(Eigen::Vector3f& L_dir, Eigen::Vector3f& L_point,
			std::vector<int>& split_idxs, float& polygon_dist);

		/*
		目的：输入L_dir,L_point, polygon是自己的corner_pnts_, 输出这个polygon距离L_dir 直线的距离差（最远减去最近）。
		这些非常重要，用于后续的expand操作
		*/
		bool ComputePolygonRangeDistToIntersectionLine(Eigen::Vector3f& L_dir, Eigen::Vector3f& L_point,
			std::vector<int>& split_idxs, float& polygon_dist);

		/*
		目的：输入l_dir2d,l_point2d, polygon是自己的corner_pnts_, 输出这个polygon距离L_dir 直线的最远的顶点。
		这些非常重要，用于后续的expand操作
		*/
		void ComputePolygonFarestDistToIntersectionLine(Eigen::Vector2f& l_dir2d, Eigen::Vector2f& l_point2d,
			int& farest_idx, float& max_dist);

		/*
		目的：输入l_dir2d,l_point2d, polygon是自己的corner_pnts_, 输出这个polygon距离L_dir 直线的最远的顶点。
		这些非常重要，用于后续的expand操作
		*/
		void ComputePolygonFarestDistToIntersectionLineSegment(Eigen::Vector2f& ls, Eigen::Vector2f& le,
			int& farest_idx, float& max_dist);

		/*
		目的：输入farestidx, polygon是自己的corner_pnts_ idx, 从farestidx开始的polygon idx。
		这些非常重要，用于后续的expand操作
		*/
		void RestartPolyIdxsFromSelectedIdx(int& farestidx, std::vector<int>& repolyidx);

		/*
		目的：输入L_dir,L_point, polygon是自己的corner_pnts_, 输出被直线分割的两个部分。这些非常重要，用于后续的expand操作
		*/
		void ComputeIdxFromSplitPolygonIntersection(Eigen::Vector3f& L_dir, Eigen::Vector3f& L_point,
			std::vector<int>& split_idxs1, std::vector<int>& split_idxs2, std::vector<int>& moved_idxs);

		/*
		目的：输入L_dir,L_point, polygon是自己的corner_pnts_, 不用移动的索引，输出新的polygon顶点2d
		*/
		void ComputeNewPolygonPntsWithNomoveIdx(Eigen::Vector3f& L_dir, Eigen::Vector3f& L_point, 
			std::vector<int>& no_move_idxs, float dist_thresh, std::vector<Eigen::Vector2f>& polygon_new);

		/*
		目的：输入L_dir,L_point, polygon是自己的corner_pnts_,
		不用移动的索引，以及被选的split polygon的lr_idx，输出新的polygon顶点2d
		*/
		int ComputeNewPolyPntsWithNomoveIdxSplitPoly(Eigen::Vector3f& L_dir, Eigen::Vector3f& L_point, std::vector<int>& split_polygon, std::vector<int>& no_move_idxs, 
			Eigen::Vector2i& lr_idx, Eigen::Vector2i& connect_idx, float dist_thresh, std::vector<Eigen::Vector2f>& polygon_new);

		/*
		目的：方程从计算世界坐标系中的顶点映射到平面坐标系后，形成平面坐标系下的方程
		*/
		void ComputeWorldLf2PlaneLf(Eigen::Vector3f& L_dir, Eigen::Vector3f& L_point, 
			Eigen::Vector2f& l_dir2d, Eigen::Vector2f& l_point2d);

		/*
		目的：计算最左最右端的端点投到直线上 形成的一个直线,其中直线为l_point2d,l_dir2d, 需要lr_idx上的方程投影到这直线上
		*/
		void ComputeVerticalLineFromTwoPnts2D(Eigen::Vector2i lr_idx, Eigen::Vector2f& l_point2d, Eigen::Vector2f& l_dir2d,
			Eigen::Vector2f& mean_pnt, Eigen::Vector2f& mean_dir);

		/*
		目的：计算线段的两个端点的垂直直线上的直线方程
		*/
		void ComputeVerticalLFunctionFromTwoPnts2D(Eigen::Vector2f& ls, Eigen::Vector2f& le,
			Eigen::Vector2f& mean_pnt, Eigen::Vector2f& mean_dir);

		/*
		目的：计算线段的两个端点的直线方程,从线段转为直线放程，表达形式为ldir,lpnt
		*/
		void ComputeLdirFunctionFromEndPnts2D(Eigen::Vector2f& ls2d, Eigen::Vector2f& le2d,
			Eigen::Vector2f& ldir2d, Eigen::Vector2f& lpnt2d);

		/*
		目的：计算线段和直线的交点，如果线段和直线相交返回true，保存交点
		*/
		bool ComputeLineSeg2DCrossLine2DPnts(Eigen::Vector2f& ls, Eigen::Vector2f& le,
			Eigen::Vector2f& mean_pnt, Eigen::Vector2f& mean_dir, Eigen::Vector2f& cross_pnt);

		Eigen::Vector2f ComputePlaneTwoLinesCrossPnt(Eigen::Vector2f& ss, Eigen::Vector2f& se,
			Eigen::Vector2f& ts, Eigen::Vector2f& te);

		bool ComputeLineSeg2DCrossLine2DPntsNew(Eigen::Vector2f& ls, Eigen::Vector2f& le,
			Eigen::Vector2f& t_pnt, Eigen::Vector2f& t_dir, Eigen::Vector2f& cross_pnt);

		bool ComputeTwoLineSegsCrossPnt(Eigen::Vector2f& ls, Eigen::Vector2f& le,
			Eigen::Vector2f& ts, Eigen::Vector2f& te, Eigen::Vector2f& cross_pnt);
		//计算两个直线的交点,如果平行返回true,否者返回false
		bool ComputeTwoLinesCrosspnt(Eigen::Vector2f& ls, Eigen::Vector2f& le,
			Eigen::Vector2f& ts, Eigen::Vector2f& te, Eigen::Vector2f& cross_pnt);

		Eigen::Vector2f ComputePlaneTwoLinesCrossPntNew(Eigen::Vector2f& sp, Eigen::Vector2f& sdir,
			Eigen::Vector2f& tp, Eigen::Vector2f& tdir);

		/*
		目的：从两个点获取函数 l_s, l_e,两个端点，获取函数f2d: f2d[0]*x + f2d[1]*y+f2d[2]=0;
		A:{a,b; c,d}X=D; 将(x0,x1);(x2,x3)带入
		*/
		bool ComputePlaneFunctionFromTwoPnts(Eigen::Vector2f& l_s, Eigen::Vector2f& l_e, Eigen::Vector3f& f2d);

		/*
		目的：计算polygon中相对一条线上，离线段上很远的点（离的远的两个端点）
		*/
		void ComputeTwoPointsIdxFarestCorrepondingLine(Eigen::Vector3f& L_dir, Eigen::Vector3f& L_point, Eigen::Vector2i& pnts_idxs);

		//bool IsDegeneration(int idx,float4& plane_coeff);
		int IsSelfIntersection(int idx, float2& p_curr);
		int IsSelfIntersection(std::vector<float2>& corner_pnts, int idx, float2& p_curr);
		int IsSelfIntersections(std::vector<float2>& corner_pnts);	//判断polygon是否有自交出现
		/*
		目的：判断polygon中，新的点移动后，是否出现交叉现象，
		idx为初始的顶点索引，p_curr为idx移动到的顶点位置，in_polygon为输入的polygon
		*/
		int IsSelfIntersectionInPolygon(std::vector<Eigen::Vector2f>& in_polygon, int idx, Eigen::Vector2f& p_curr);

		/*
		目的：判断polygon中，新的点移动后，是否出现交叉现象，
		idx为初始的顶点索引，p_curr为idx移动到的顶点位置，in_polygon为输入的polygon
		*/
		int IsSelfIntersectionInPolygonNew(std::vector<Eigen::Vector2f>& in_polygon, int idx, Eigen::Vector2f& p_curr);

		int IsSelfIntersectionsEigenNew(std::vector<Eigen::Vector2f>& corner_pnts);	//判断polygon是否有自交出现

		//计算世界坐标系下的顶点到直线的距离
		float Pnt3DDistToLine3D(Eigen::Vector3f& pnt, Eigen::Vector3f& L_dir, Eigen::Vector3f& L_point);

		/*
		目的：在平面坐标系下，顶点到到直线的距离
		*/
		float Pnt2DDistToLine2D(Eigen::Vector2f& pnt2d, Eigen::Vector2f& L_point2d, Eigen::Vector2f& L_dir2d);

		/*
		目的：在平面坐标系下，顶点到到直线的距离
		*/
		float Pnt2DDistToLine2DNew1(Eigen::Vector2f& pnt2d, Eigen::Vector2f& lpnt2d, Eigen::Vector2f& ldir2d);
		
		/*
		目的：在平面坐标系下，顶点到到直线的距离ls, le为直线上的表示方式
		*/
		float Pnt2DDistToLine2DLSLE(Eigen::Vector2f& pnt2d, Eigen::Vector2f& ls, Eigen::Vector2f& le);

		/*
		目的：计算顶点到线段的距离
		*/
		float Pnt2DToLineSegment2D(Eigen::Vector2f& pnt2d, Eigen::Vector2f& ls, Eigen::Vector2f& le);
		
		/*
		目的：在平面坐标系下，顶点到到直线的距离, 有bug
		*/
		float Pnt2DDistToLine2DNew(Eigen::Vector2f& pnt2d, Eigen::Vector2f& ldir2d, Eigen::Vector2f& lpnt2d);

		//这个是将平面上的顶点平移到直线（在平面坐标系下操作的，两个平面的相交线）
		float2 ProjToLine3D(int idx, Eigen::Vector3f& L_dir, Eigen::Vector3f& L_point);
		float2 ProjToPlane3D(int idx, float4& plane_coeff);
		float2 ProjToPlane3D(float3& pnt);
		float3 ProjToPlane(float3& pnt);
		
		Eigen::Vector3f Pnt3dProjToPlane3D(const Eigen::Vector3f& pnt);

		/*
		目的：在平面坐标系下，将顶点投影到直线上
		*/
		void Pnt2dProjLine2D(Eigen::Vector2f& pnt2d, Eigen::Vector2f& ls, Eigen::Vector2f& ldir,
			Eigen::Vector2f& proj_pnt2d);

		/*
		目的：在平面坐标系下，将顶点投影到直线上,如果是-1，则它的在线段外，0 在线段内，1 在线段外，
		保存在proj_pnt2d
		*/
		int Pnt2dProjLineSeg2D(Eigen::Vector2f& pnt2d, Eigen::Vector2f& ls, Eigen::Vector2f& le,
			Eigen::Vector2f& proj_pnt2d);

		/*
		目的：在平面坐标系下，将顶点反投到世界坐标系的三维坐标
		*/
		void Pnt2d2Pnt3D(Eigen::Vector2f& pnt2d, Eigen::Vector3f& pnt3d);

		/*
		目的：在世界坐标系下，将三维顶点投到平面标系的二维坐标
		*/
		void Pnt3d2Pnt2D(Eigen::Vector3f& pnt3d, Eigen::Vector2f& pnt2d);

		bool IsAreaIncreasing(int idx, Eigen::Vector3f& L_dir, Eigen::Vector3f& L_point);
		bool IsSelectedAreaIncreasing(std::vector<Eigen::Vector2f>& polygonpnts, int idx, Eigen::Vector2f& projpnt);

		//计算polygon 面积的大小
		float ComputePolygon2DAreaFromPnts3d();

		//计算多边形的面积，用于后续处理
		void ComputePolygon2DAreaFromCornerPnts3d();
		float GetPolygonCorner2DArea();

		bool GenerateInitial2DSortedPnts();

		void Generate2DPloygons();
		
		void GenerateSampleWorldCoordPnts();

		void MergeSampleAndSrcPnts();

		float3 Compute2DPlaneLineFunction(const float2& a, const float2& b);

		float ComputeVerticesDistIn2DPlane(const float2& a, const float2& b);

		float ComputeDistTo2DLine(const float2& a, const float3& f);

		////计算点云的协方差矩阵
		//Eigen::Matrix2f Compute2DCovarianceMatrix(const std::vector<float2>& vertices);
		//float2 Compute2DVerticesNormal(const std::vector<float2>& vertices);
		//void Compute2DMatrixEigens(const Eigen::Matrix2f& matrix,
		//	Eigen::Matrix2f& eig_values, Eigen::Matrix2f& eig_vectors);

		float Compute2DVerticesCrossValue(const float2& a, const float2& b);

		//test
		std::vector<std::vector<float3> > lines_pos_;
		std::vector<std::vector<float3> > merge_lines_pos_;
		//end test

		//提取出的边缘点和原始顶点对应，这样就只保存序号就可以了
		bool MapSortedEdgePnt2PntPosIdx();

		//
		void PaintEdgePntsIntoImgAndSave();

		//设置绘制图片的大小
		void SetImageSize(int w, int h);

		void ComputeImageConvertParams();

		void GenerateEdgeImagePolygon();

		//坐标系转化,从平面坐标系转化到图像坐标系
		cv::Point2f PlaneCoordPnt2ImageCoordPnt(const Point_2d& plane_2d_pnt);

		Point_2d ImageCoordPnt2PlaneCoordPnt(const cv::Point2f& image_pnt);

		//-----------------------------投影到2d平面上---------------------------//

		//计算顶点的weight
		float ComputeSortedPntsAvarageWeight(const std::vector<float>& pnts_weight);

		//triangulate polygon
		void TriangulatePolygon2D();
		
		//test
		void SamplePntsFromTriTest();
		//end test

		//---------------------------完成投影到2d平面上------------------------//
		//将一个顶点投影到2d平面上
		float3 GetAPtProjectedToPlane(float3 pt);

		//计算点云的平均位置
		float3 ComputeAverageVertexPos();

		//计算点云的平均法向量，用于后面的sample顶点的法向量，后续可以用其它方法
		Eigen::Vector3f ComputeAverageVertexNormal();

		//set split structrue,-1表示它未进入剪切状态，其它表示它进入剪切状态
		void SetSplitFlag(int split_flag);
		//将数据存入到这个结构体中
		void SetSplitStructure(std::vector<HW::HWPlane*> model_planes,
			std::vector<int>& related_idxs);

		/*
		目的：构建周围的polygon的范围
		*/
		bool BuildAroundPolygonPnts2d(std::vector<std::vector<int> >& polygons_adjs);

		/*
		目的：判断线段和polygon是否有交点
		*/
		bool CheckCrossPntsWithPolygon(Eigen::Vector2f& ls, Eigen::Vector2f& le);

		/*
		目的：设置拼接线段的方向，让它和原始的src的polygon方向一致
		*/
		void SetAdjLinesDirect();

		bool CheckVecFlag(std::vector<bool>& flags);

		/*
		目的：优化polygon
		*/
		void OpitmizeAdjLinesPolygon();

		/*
		目的：设置related idx 和 polygon的方向保持一致,使得polygon能够形成
		*/
		bool SetRelatedIdxAroundSrcPoly();

		/*
		目的：通过线段构建矩形，输入包括：线段，还有宽度(需要距离来操作),逆时针
		*/
		bool ContructRectFromLS2D(Eigen::Vector2f& ls, Eigen::Vector2f& le, 
			float width, std::vector<Eigen::Vector2f>& rect_pnts);

		bool ComputeNearestDist2ParallAdjLines(int srcidx, float& dist);

		/*
		目的：判断方向,如果方向一致，则返回true否者为false
		*/
		bool CheckLineDirectWithPolygonDirect(std::vector<int>& lines_idxs);

		/*
		目的：判断方向,如果方向一致，则返回true否者为false
		*/
		void ConstructHalfPolyIdxs(Eigen::Vector2i& s2e, std::vector<int>& half_poly);

		/*
		目的：检验两个polygon是否为邻近关系
		*/
		bool CheckTwoPolysAdj(int src_idx, int tgt_idx, std::vector<std::vector<int> >& polygons_adjs);

		////参数后面在定
		//bool VoteForExpandPolygon(std::vector<Eigen::Vector3f>& pnts, 
		//	std::vector<Eigen::Vector3f>& polygon_pnts);

		//对related_poly_idxs_进行排序
		bool SortRelatedPolyIdxBasedOnMinCor();
		//end set split structure

		//void ComputePolygonFittingBoundingBox(std::vector<Eigen::Vector3f>& box_3d, float dist_epson);

		void ComputeOriginPntMaxDist2Plane(float& dist_max_normal, int& idx_normal,
			float& dist_max_inverse_normal, int& idx_inverse_normal);

		float ComputeUEEnergyPnts2Plane();

#if HW_DEBUG
		void SavePlaneCoordPntsIntoOBJ(const std::string path);
		void SavePlaneInitialEdgePntsIntoOBJ(const std::string& path);
		void SavePlaneRefinedEdgePntsIntoOBJ(const std::string& path);
		void SavePlanePointIntoOBJ(const std::string path);
		//0,1,2...
		void SavePnts2dIntoPnts3DOBJ(const std::string& path, 
			const std::vector<Eigen::Vector2f>& pnts2d);
		//0,1->line0; 2,3->line1...
		void SaveLinesPnts2dIntoLinesPnts3DOBJ(const std::string& path,
			const std::vector<Eigen::Vector2f>& pnts2d);
		//0,1->line0; 1,2->line1... end,0->linei
		void SavePolygonLines2dIntoPolygonLines3DOBJ(const std::string& path,
			const std::vector<Eigen::Vector2f>& pnts2d);

		void SaveHWOptiPnts2dIntoPolygonLineObj(const std::string& path, const std::vector<HWOptiPnt2D>& pnts);

		void SaveLinesPnts2dIntoLinesPnt3DOBJWithNormal(const std::string& path, const std::vector<Eigen::Vector2f>& pnts_pos,
			const std::vector<Eigen::Vector2f>& pnts_normals);

		void SavePnts3dIntoObj(const std::string& path, const std::vector<Eigen::Vector3f>& pnts3d);
#endif

	private:
		void ComputeCornerPnts3dFromCornerPnts2d();
		void ComputeArea();
		float ComputeTriangleAreaFrom3Points2d(float2 a, float2 b, float2 c);

		void UpdateRefinedSortedBorderPnts2D();

		bool CheckSameSegmentEdges(Segment& a, Segment& b);

		bool CheckSameSegmentPnts(Point_2d& a, Point_2d& b);

		//证明两个顶点的距离在threshold范围内
		bool CheckThresholdTwoPnts(double threshold, Point_2d& a, Point_2d& b);

		//对平面点云进行sor filter
		void PointCloudSORFilter();
		
		//计算投影到2d点云的box
		void Compute2DPntsBoxPos(float2& min_corner, float2& max_corner);

		//计算点云的协方差矩阵
		Eigen::Matrix3f ComputeCovarianceMatrix(const std::vector<float3>& vertices);

		//计算矩阵的Eigen
		void ComputeMatrixEigens(const Eigen::Matrix3f& matrix,
			Eigen::Matrix3f& eig_values, Eigen::Matrix3f& eig_vectors);

		//------------------------对三角化的Polygon进行采样点------------------//
		bool SamplePntsFromPolygon();
		void SamplePntsFromTri(std::vector<Point2>& result_pnt, 
			std::vector<Point2> tri_pnts, int point_density);
		//
		ccPlane* associated_cc_plane_;

		HWPointCloud* filtered_plane_pc;

		//平面方程ax + by + cz + d = 0;
		float4 coeff_;
		//float d_;
		Eigen::Vector3f box_min_;
		Eigen::Vector3f box_max_;
		Eigen::Vector2f boxMin_2d_;
		Eigen::Vector2f boxMax_2d_;
		float plane_width_;
		float plane_height_;

		std::string plane_obj_name_;
		std::string plane_output_dir_;

		//平面坐标系最左边点和最右边点
		float left_bottom_;
		float right_top_;

		//平面的顶点索引
		std::vector<int> pnts_idx_;
		std::vector<float3> pnts_pos_origin_;
		std::vector<float3> pnts_normal_origin_;
		std::vector<float3> pnts_color_origin_;
		std::vector<float3> pnts_pos_;
		std::vector<float3> pnts_normal_;
		std::vector<uchar3> pnts_color_;

		//平面的边缘顶点
		std::vector<int> edge_pnts_idx_;
		std::vector<float2> edge_pnts_pos_;
		std::vector<std::vector<float2>> inner_edge_pnts_pos_;

		//---------------OptimizeNewPolygonExtraction-------------//
		float polygon_neighbor_lines_angle_threhold_ = 10.0f;
		std::vector<HWOptiPnt2D> initial_edge_pnts_pos_;
		int edges_pnts_fitting_num_threshold_ = 2;
		int initial_edges_pnts_num_;
		std::vector<HWLineToPntsIdxs> initial_funs_coeffs_;
		int initial_edge_pnts_in_lines_num_;
		PolygonExtractionOptiEnergyWeights polygon_extract_energy_weights_;
		int polygon_extract_opti_iterators_num_ = 100;
		std::vector<HWLineToPntsIdxs> funs_coeffs_optimized_;
		std::vector<HWOptiPnt2D> polygon_corner_pnts_optimized_;
		//---------------End OptimizeNewPolygonExtraction-------------//

		int edge_pnts_pos_num_;
		int inner_polygon_num_;
		std::vector<int> inner_edge_pnts_pos_num_;
		std::vector<float2> edge_pnts_normal_;
		std::vector<std::vector<float2>> inner_edge_pnts_normal_;
		std::vector<uchar3> edge_pnts_color_;
		std::vector<int> edge_pnts_color_index_;
		std::vector<uchar3> colors_;

		Eigen::Vector3f average_pnts_normal_;
		float diameter_max_;
		std::vector<std::vector<int>> edge_pnts_smooth_regions_;
		std::vector<std::vector<std::vector<int>>> inner_edge_pnts_smooth_regions_;
		std::vector<std::vector<int>> edge_pnts_neighs_;
		std::vector<std::vector<std::vector<int>>> inner_edge_pnts_neighs_;
		std::vector<Eigen::Vector2f> edges_;
		std::vector<std::vector<Eigen::Vector2f>> inner_edges_;
		std::vector<int> edge_idx_;
		std::vector<float2> corner_pnts_;
		std::vector<std::vector<float2>> inner_corner_pnts_;
		std::vector<float2> all_corner_pnts_;
		std::vector<float2> corner_pnts_uv_;
		std::vector<float> corner_angels_;
		std::vector<float> min_dists_;
		std::vector<float3> corner_pnts_3d_;
		std::vector<std::vector<float3>> inner_corner_pnts_3d_;
		std::vector<float3> all_corner_pnts_3d_;
		std::vector<int> contour_index_;
		std::vector<int3> triangles_idx_;
		std::vector<int> lines_idx_;

		std::vector<float2> corner_pnts_last_state_;
		std::vector<float> corner_angels_last_state_;
		std::vector<float> min_dists_last_state_;
		std::vector<float3> corner_pnts_3d_last_state_;
		std::vector<int3> triangles_idx_last_state_;

		//std::vector<Eigen::Vector3f> corner_pnts_fitting_3d_;

		//------------这些都是需要的，后续要去掉一些不可用的东西-----------------------//
		//传化到平面物体坐标系下的坐标
		std::vector<float3> plane_coord_pos_;
		std::vector<float3> plane_coord_pnts_normal_;

		//这些都是在2d投影上，保存边缘点连起来的边
		std::vector<Segment> border_edges_;
		
		//这个从原始的边缘边合成闭合的边缘边
		std::vector<Segment> sorted_border_edges_2d_;

		//它是边sorted_border_edges_2d_取其中的开头的顶点得到，这个顶点通过Polygon可以得到简化后的polygon
		std::vector<Point_2d> sorted_border_pnts_2d_;
		std::vector<float> sorted_border_pnts_weight_2d_;
		std::vector<int> sbp_to_pnts_pos;
		bool map_pnts_flag = false;

		//转化到图像坐标系中，需要平移x,y的坐标,将平面坐标转移到正值,然后再平移顶点
		float move_x_ = 0.0;
		float move_y_ = 0.0;
		float polygon2image_scale_ = 1.0f;
		//
		int image_height_;
		int image_width_;

		float distance_per_pixel_ = 0.0f;

		//
		Polygon_2 image_coord_border_pnts_;
		//Insert the polygons into a constrained triangulation
		CDT cdt_;
		CDT cdt_polygon_;
		std::vector<Point2> sample_polygon_vertices_;

		float plane_r_tri_len_;
		float plane_r_tri_degree_;
		std::vector<float3> polygon_tri_refine_pnts_;
		std::vector<float3> polygon_tri_refine_normals_;
		std::vector<float2> polygon_tri_refine_uvs_;
		std::vector<int3> polygon_tri_refine_idx_;

		std::vector<float3> sample_world_coord_vertices_;
		//采样的顶点和原始的顶点合并
		std::vector<float3> merged_pnts_pos_;
		std::vector<float3> merged_pnts_normal_;
		std::vector<uchar3> merged_pnts_color_;

		float cloud_resolution_=0.0f;
		float ave_alpha_edge_len_=0.0f;
		float ave_pixels_=0.0f;
		float2 alpha_edges_x_;
		float2 alpha_edges_y_;
		PolygonExtractionParameters params_;
		PolygonExtractionParameters initial_params_;
		int edge_size_;
		std::string filename_;
		double min_value_ = 1e-6;
		Eigen::Vector3f plane_normal_;
		int semantic_label_;
		//是否被手动修改过
		bool manually_edited_ = false;
		float corners_area_2d_ = 0.0;

		float poly_dist_threshold_;
		float r_poly_dist_max_;

		//晚上做一些数据。明天早上实现这个算法。
		std::vector<HW::HWPlane*> model_planes_;	//等效于planes_vec_
		int splited_label_ = -1;	//当前idx
		//对多边形进行切割，切割它的平面索引
		std::vector<int> related_poly_idxs_;	//这是接近x方向的关联的polygon
		std::pair<Eigen::Vector2f, Eigen::Vector2f> corners_width_2d_;
		std::vector<float> related_poly_corners_areas_2d_;
		std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f> > related_poly_corners_widths_2d_;	//ls 和 le
		std::vector<std::vector<int> > related_polys_around_pnts_idx_;	//这个是每个polygon 的idx 对应的polygon,它和related_poly_idxs_顺序一致
		std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f> > related_polys_around_pnts_pos_;
		std::vector<Eigen::Vector2f> around_polygon_pnts_new_;	//围绕在polygon周围的pnts,先整体拼接
		std::vector<int> sorted_around_related_poly_idxs_;	//相对于related_polys_around_pnts_idx_下表的polygon
		bool around_polygon_pnts_closed_ = false;
		//平面坐标系投影到图像坐标系，后面用于处理图像
		int plane_image_height_;
		int plane_image_width_;

		//表示polygon 已经被重新排序了
		bool resorted_sor_corner_flag_ = false;
		bool related_param_flag = false;

		//设置polygon的类型:凸或者非凸
		PolyonCanonicalType polygon_cannoical_;
		//std::vector<SplitNode*> split_poly_nodes_;
		//处理后新的polygon
		std::vector<Eigen::Vector3f> poly_pnts_3d_new_;

		//-------------------------------end-------------------------------------//

		//没有平面边缘的顶点，其实顶点位置可以不用要的
		std::vector<int> pnts_idx_without_edge_;

		//世界坐标系到平面坐标系的矩阵
		Eigen::Matrix4f world_to_plane_;
		//相当于一个相机坐标到世界坐标的变换矩阵，左上角3x3矩阵的每一列代表相机中的每个轴的方向
		Eigen::Matrix4f plane_to_world_;	//cam pos

		float area_ = 0;
	};
}
