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

//Sorȥ��
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

	//alpha-shapes��ߵĲ���
	//Ĭ��2*cloud_resolution_
	double alpha;

	//ÿ����������ʵ�����еĳ��ȣ�������ͼƬ��С�����ն���εľ���
	//Ĭ��0.01
	double real_world_length_per_pixel;

	//ͼ��������е�����ģ��Ĵ�С
	//Ĭ��ave_pixels_
	int dilate_half_width;

	//��ͼ������Ĳ����ʣ���ʾÿ�������ز���һ������
	//Ĭ��ave_pixels_/2
	int pixel_to_world_sampling_rate;

	//ƽ�������ֱ��
	//Ĭ��0.05
	double smooth_region_diameter;
	
	//�����ƽ�����Ż�����
	//Ĭ��1.0
	double polygon_smoothing_mu;

	//�ж�һ����ķ����Ƿ���ǰ��������ķ���ƽ�еĲ���������������ֵ�������ڲ�ƽ��
	//Ĭ��10
	double max_theta;

	//extractһ��polygon�����>3�����㷨�����Ͻ�����fitting��Щ����
	int fitting_lines_pnts_number_threshold_;

	//�������ε�������������֮��ľ���С�����ֵ��������������е������������
	//Ĭ��ave_alpha_edge_len_
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


#define OUTPUT_ALPHA_SHAPE_EDGES //���alpha-shapes�õ��ı�Ե
#define OUTPUT_IMAGE_EDGES //�������ͼ����ı�Ե��
#define OUTPUT_IMAGE_EDGES_NORMAL //�����Ե���䷨��
#define OUTPUT_IMAGE_EDGES_NORMAL_SMOOTHED //����Ż���ķ���
#define OUTPUT_EDGES_SMOOTHED //����Ż���ı�Ե��
#define OUTPUT_POLYGON //������յõ��Ķ����

//���̳У��ĳ����
namespace HW
{

	//
	typedef std::array<float, 2> point2d_type;

	//ԭʼpolyogn������
	enum PolyonCanonicalType
	{
		kPolyConvex,
		kPolyNoConvex,
		kPolyConvexOther
	};

	//�и��Ľṹ�����µĽṹ��
	enum SplitType
	{
		//ֻ��һ��ֱ�߼��е�ʱ����2���ṹ���Ͷ��е�
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
		std::vector<int> s_pnts_idxs_;	//����,��Ҫ�Ľṹ��
		std::vector<std::vector<int> > line_idxs_;	//���Ƕ���߶����,line_idxs_.size()Ϊ�߶�����
		std::vector<bool> lines_direct_;	//true��������false�Ǹ�����
		std::vector<int> t_line_idxs_;
		std::vector<Eigen::Vector3f> split_pre_cross_;	//���Ǻ���һ��node�п��ĵ�
		std::vector<Eigen::Vector3f> split_next_cross_;	//���Ǻ���һ��node�п��ĵ�

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
		Ŀ�ģ�û�е��Ƶ�����£�����ͶӰ��ͼ������ϵ�Ĳ��������ں�����ͼ��ͶӰ����
		*/
		bool SetImageProjCoordParams(bool cloud_flag);

		/*
		*/
		bool SetPolygonLSParams(bool cloud_flag);

		void SetPolyLSParamsAndImgProjFlag(bool flag);

		/*
		Ŀ�ģ���ȡ��ƽ������ϵת����ͼ������ϵ�ϣ����ں����Ĵ���
		*/
		bool DrawImageFromCornerPnts2d(std::vector<Eigen::Vector2f>& pnts2d, cv::Mat& image, const std::string& file);
		
		/*
		Ŀ�ģ���ƽ������ϵת��ͼ������ϵ���γ�ͼ��polygon
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
		void WriteSelectLines3D(std::string filename, const std::vector<int>& selected_pnt_idx);	//��polygon
		void WriteSelectEdges3DMoveFlag(std::string filename, const std::vector<int>& selected_pnt_idx, std::vector<bool>& movflagvec);
		void WriteSelectEdges2DMoveFlag(std::string filename, std::vector<Eigen::Vector2f>& poly2dpnts, std::vector<bool>& movflagvec);
		void WriteCornerPoints3D(std::string filename);
		void WriteCornerPoints2D(std::string filename);
		/*
		Ŀ�ģ�debug����3D��corner �����ӡ����
		*/
		void ScoutCornerPoints3D();

		/*
		Ŀ�ģ�debug����2D��corner �����ӡ����
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
		//��ƽ����ƣ�ͶӰ��ƽ���ϣ��γ��ϸ�ƽ�棬���Ǳ�������ƽ�淽��
		void ProjectTo3DPlane();

		bool SaveMargedPlanePntsPly(const std::string& file, const PlyFormat& type);

		//�ӹ����ĵ���ȥ������ƽ�������ĵ��ƣ�Ȼ�󱣴�����
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

		//��ȡƽ������ϵ�µ�����
		const std::vector<float3>& GetPlaneCoordPos();
		float4 GetPlaneCoeff();

		bool HasEdgePnts();

		const ccPlane* GetAssociatedPlane();
		void SetAssociatedPlane(ccPlane* plane);

		//������Ƶ�ƽ���ܶ�
		float ComputeCloudResolution(const pcl::PointCloud<pcl::PointXY>::Ptr &cloud);
		float ComputePlaneCloudResolution2D(const pcl::PointCloud<pcl::PointXY>::Ptr &plane_cloud);

		//����ÿ��ƽ�����ٵ���
		int ComputePointsNumPerSquareMeter();
		int EstimateBordersFromPcl(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, float re, float reforn);	//�����Ҫ�û����������뾶���ͽǶȣ��������ã�
		bool DoEstimateBordersFromPcl();
		int EstimateBordersFromPclAngle(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::Normal>::Ptr & cloud_normal,
			int seach_k, float angle_ratio); // ֻ��Ҫ�����������ںͽǶȱȣ�MPI*angle_ratio��
		int EstimateBordersFromPclRadius(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::Normal>::Ptr & cloud_normal,
			float seach_radius, float angle_ratio);	//
		bool DoEstimateBordersFromPclAngle();

		//�Ա߽�߽��о���
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
		��ȡpolygon����ά����,Ȼ��������ǵ�ƽ������ϵ���������ǵ�ת��
		*/
		void SetPolygonPnts3dFromInitialPnts(std::vector<Eigen::Vector3f>& polygon3d_pnts);
		Eigen::Vector2f FittingLine(std::vector<int>& neighs);
		Eigen::Vector2f FittingLine(std::vector<float2> edge_pnts, std::vector<int>& neighs);
		Eigen::Vector2f FittingLine(float2& p1, float2& p2);
		Eigen::Vector2f FittingLineEigen(Eigen::Vector2f& p1, Eigen::Vector2f& p2);
		Eigen::Vector3f FittingLineFromPntsIdxs(const std::vector<int>& idxs);

		/*
		Ŀ�ģ�ͨ��������������ߵķ���ax+by+c=0�������Ա�֤�������е���
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
		Ŀ�ģ����µ�plygon���д������ں����Ĳ���
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
		//��polygon�е�p_idx�Ķ����ƶ���ray_pntλ����
		bool DoMovePnt2dToNewPnt2d(int p_idx, Eigen::Vector2f& ray_pnt);
		void DoPointCreating(int idx, Eigen::Vector3f & L_dir, Eigen::Vector3f & L_point);
		//���߶�ls_idx, le_idx����Ӷ���pnt2d
		void DoPointCreateOnLineSeg(int ls_idx, int le_idx, Eigen::Vector2f& pnt2d);
		//��Ӷ���pnt2d��polygon��Ե��,������ڣ���ͶӰ�����̫Զ�Ͳ�����
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
		Ŀ�ģ��ж϶���p�Ƿ���polygon��,������ͬһ������ϵ
		*/
		int IsPInPolygonVec(Eigen::Vector2f& p, std::vector<Eigen::Vector2f>& in_polygon);

		/*
		Ŀ�ģ���ȡpolygon�еİ��������
		*/
		void GetPolygonConcaveVertex2DIdx(std::vector<int>& p_idxs);

		/*
		Ŀ�ģ���ȡpolygon�еİ���Ķ���
		*/
		void GetPolygonConcaveVertex2DPnts(std::vector<Eigen::Vector2f> p_pnts);
		int IsConcaveEdgePoint(int idx);
		int IsConcaveEdgePoint(const std::vector<float2>& points, int idx);
		int PointMatchingJudge(float3& point,float rp,float& dist);
		float GetEdgesAngel(float3& v1, float3& v2, int edge_idx1, int edge_idx2);
		float GetDistToPolygon(Eigen::Vector3f& p);
		void DoTriangulation();
		void DoTriangulationRefined();	//���°汾�����ǻ��ļ�
		void ComputeTriNormalRefined();	//���°汾�����ǻ��ļ�,��Ч
		void ComputeTriPntsUVRefined();	//���°汾�����ǻ��ļ���uvֵ
		void UpdateInformation();
		void DoPolygonSampling(std::vector<float3>& sample_vertices, int density);
		float RayToPolygonDist(Eigen::Vector3f& ray_o, Eigen::Vector3f& ray_direct,int& nearest_pnt_idx);
		////����ray�Ķ����������Ķ���
		//bool ComputeRayPntMindist2PolyPnts(Eigen::Vector3f& ray_o, Eigen::Vector3f& ray_direct,
		//	float& min_dist, int& nearest_pnt_idx);
		//��ѡ���polygon �����ڵ�ƽ�������ray_o��ray_direct�����󽻣��õ���������
		bool RayToPolyPlanePntPos(Eigen::Vector3f& ray_o, Eigen::Vector3f& ray_direct, Eigen::Vector2f& proj_p);
		bool RayToPolyIntersectionPnt(Eigen::Vector3f& ray_o, Eigen::Vector3f& ray_direct, Eigen::Vector3f& pnt);
		bool RayToPolyPlaneIntersectionPntConst(const Eigen::Vector3f& ray_o, const Eigen::Vector3f& ray_direct, Eigen::Vector3f& pnt);
		//���ߺ�polygon�����ཻ
		bool RayLinePnt2PolygonPntThreshold(const Eigen::Vector3f& oc, const Eigen::Vector3f& odir, Eigen::Vector3f& pnt3d);
		bool IsDegeneration(int idx, Eigen::Vector3f& L_dir, Eigen::Vector3f& L_point);
		/*
		Ŀ�ģ����true��ʾ����������ͶӰ��ֱ���ϣ������⴦���˰�������
		*/
		bool IsDegenerationIgnoreConvexPnts(int idx, Eigen::Vector3f& L_dir, Eigen::Vector3f& L_point);
		
		/*
		Ŀ�ģ��ж��߶��Ƿ���vec�߶���
		*/
		bool FindLineSegIdxInLSVec(std::vector<Eigen::Vector2i>& vec, Eigen::Vector2i& l);

		/*
		Ŀ�ģ���ȡpolygon�е��߶�i,j�ĳ���
		*/
		float GetPolyon3DLength(int i, int j);

		/*
		Ŀ�ģ��ж����value�Ƿ���vec��
		*/
		bool FindValueInVec(std::vector<int>& vec, int v);

		/*
		Ŀ�ģ�����Ķ�������������polygon�ı�,������polygon��״
		*/
		void BuildLSIdxFromPolygonIdxsVec(std::vector<int>& polygon_idxs, std::vector<Eigen::Vector2i>& linesegidx);

		/*
		Ŀ�ģ�����polygon���һ���ߵ������˵�
		*/
		void ComputeTwoPointsIdxCorrespondingLine(Eigen::Vector3f& L_dir, Eigen::Vector3f& L_point, Eigen::Vector2i& pnts_idxs);

		/*
		Ŀ�ģ�����polygon���һ�����������ҵ������˵� 2d������ϵ
		*/
		void ComputeLeftRightPointsIdxLine2D(Eigen::Vector2f& L_dir2d, Eigen::Vector2f& L_point2d, Eigen::Vector2i& pnts_idxs);
		/*
		Ŀ�ģ�����polygon���һ�����������ҵ������˵� 2d������ϵ
		*/
		bool ComputeLeftRightPointsPosLine2D(Eigen::Vector2f& L_dir2d, Eigen::Vector2f& L_point2d, 
			Eigen::Vector2i& pnts_idxs, Eigen::Vector2f& lpnt, Eigen::Vector2f& rpnt);

		/*
		Ŀ�ģ�����polygon���һ�����������ҵ������˵� 2d������ϵ
		*/
		void ComputeLRPointsWithFlagIdxLine2D(Eigen::Vector2f& L_dir2d, Eigen::Vector2f& L_point2d, 
			std::vector<bool>& valided_vec, Eigen::Vector2i& pnts_idxs);

		/*
		Ŀ�ģ�����һ�����㣬������ͶӰ��ֱ�ߺ����ֱ�ߵĶ���L_point2d�����λ�ã�����Ϊfloat
		���룺inpnt2d��ֱ�ߣ� ldir2d��lpnt2d����������λ��,���L_point2d
		*/
		float ComputeProjPRelativeDist2LPntDir2d(Eigen::Vector2f& pnt2d, Eigen::Vector2f& L_dir2d, Eigen::Vector2f& L_point2d);

		/*
		Ŀ�ģ�����polygon���һ�����������ҵ������˵� 2d������ϵ,�����polygon idxs, ���split polygon�����������������˵�
		*/
		void ComputeLRPntsIdxLine2DFromSelectPoly(Eigen::Vector2f& L_dir2d, Eigen::Vector2f& L_point2d, 
			std::vector<int>& split_idxs, Eigen::Vector2i& pnts_idxs);

		/*
		Ŀ�ģ�����L_dir,L_point, polygon���Լ���corner_pnts_, ��������ƶ�������
		*/
		void ComputePolygonMoveIdxs2Line(Eigen::Vector3f& L_dir, Eigen::Vector3f& L_point, std::vector<int>& no_move_idxs);

		/*
		Ŀ�ģ���ȡpolygon����Ҫ�ƶ��Ķ���,polygon���Լ���corner_pnts_
		*/
		bool ComputePolygonMoveIdxsFromline2d(Eigen::Vector2f& ldir2d, Eigen::Vector2f& lpnt2d, std::vector<int>& move_idxs);

		/*
		Ŀ�ģ�����L_dir,L_point, polygon���Լ���corner_pnts_, ��ѡ���polygon�Ķ����idx����������ƶ�������
		*/
		void ComputeSelectedPolygonMoveIdxs2Line(Eigen::Vector3f& L_dir, Eigen::Vector3f& L_point, 
			std::vector<int>& split_idxs, std::vector<int>& no_move_idxs, Eigen::Vector2i& my_lr_idx);

		/*
		Ŀ�ģ�����L_dir,L_point, polygon���Լ���corner_pnts_, ��ѡ���polygon�Ķ����idx����������ƶ�������,�Լ��ƶ����Ǹ�����
		������������dists2l_flag��my_lr_idx �ǻ��� selected_idxs������
		*/
		void ComputeSelectedPolygonNoMoveIdxs2LineSeam(Eigen::Vector2f& l_dir2d, Eigen::Vector2f& l_pnt2d, std::vector<int>& selected_idxs, float dist_threshold, 
			std::vector<bool>& dists2l_flag, Eigen::Vector2i& my_lr_idx);

		/*
		Ŀ�ģ�����ls,le, polygon���Լ���corner_pnts_, ��ѡ���polygon�Ķ����idx����������ƶ�������,�Լ��ƶ����Ǹ�����
		������,����dists2l_flag��my_lr_idx �ǻ��� selected_idxs������
		*/
		void ComputeSelectedPolygonNoMoveIdxs2LineSegmentSeam(Eigen::Vector2f& ls, Eigen::Vector2f& le, std::vector<int>& selected_idxs, float dist_threshold,
			std::vector<bool>& dists2l_flag, Eigen::Vector2i& my_lr_idx);

		/*
		Ŀ�ģ�����L_dir,L_point, polygon���Լ���corner_pnts_, ��ѡ���polygon�Ķ����idx����������ƶ�������,�Լ��ƶ����Ǹ�����
		������
		*/
		void ComputePolygonMoveLRIdxs2LineSeam(Eigen::Vector2f& l_dir2d, Eigen::Vector2f& l_pnt2d, float dist_threshold, Eigen::Vector2i& my_lr_idx);

		/*
		Ŀ�ģ�����L_dir,L_point, polygon���Լ���corner_pnts_, dists2l_flag, ͨ����Щpolygon��ȡ�µ�polygon pnts
		������
		*/
		void ComputeAllPolyNewPntsFromNoMoveIdxs2DSeam(Eigen::Vector2f& l_dir2d, Eigen::Vector2f& l_pnt2d, 
			std::vector<bool>& dists2l_flag, std::vector<Eigen::Vector2f>& polygonnewpnts);

		/*
		Ŀ�ģ�����L_dir,L_point, polygon���Լ���selected polygon corner_pnts_, dists2l_flag, ͨ����Щpolygon��ȡ�µ�polygon pnts
		������
		*/
		void ComputeSelectPolyNewPntsFromNoMoveIdxs2DSeam(Eigen::Vector2f& l_dir2d, Eigen::Vector2f& l_pnt2d,
			std::vector<int>& poly_idxs, std::vector<bool>& dists2l_flag, std::vector<Eigen::Vector2f>& polygonnewpnts);

		/*
		Ŀ�ģ�����source_idxs target_idxs, �����������source_idxs��target_idxs������polygon�����Ӵ���idx
		*/
		void ComputeTwoPolygonConnectIdx(std::vector<int>& source_idxs, std::vector<int>& target_idxs, 
			Eigen::Vector2i& connect_idxs);

		/*
		Ŀ�ģ���ȡ��ǰ��plane��Ŀ���plane j���ཻ��
		���룺Ŀ���ƽ�������
		������ཻ��ֱ��
		*/
		void GetExpandTgtIntersectionLine(int j, Eigen::Vector3f& L_dir, Eigen::Vector3f& L_point);

		/*
		Ŀ�ģ���ȡ��ǰ��plane��Ŀ���coeff���ཻ��
		���룺Ŀ���ƽ�������
		������ཻ��ֱ��
		*/
		void GetExpandTgtCoeffIntersectionLine(float4& coeff, Eigen::Vector3f& L_dir, Eigen::Vector3f& L_point);

		/*
		Ŀ�ģ�����L_dir,L_point, polygon���Լ���corner_pnts_, �����ֱ�߷ָ���������֡���Щ�ǳ���Ҫ�����ں�����expand����
		*/
		bool SplitPolygonFromIntersectionLineNew(Eigen::Vector3f& L_dir, Eigen::Vector3f& L_point,
			std::vector<int>& split_idxs1, std::vector<int>& split_idxs2);

		/*
		Ŀ�ģ�����ldir2d,lpnt2d, polygon���Լ���poly_idx, �����ֱ�߷ָ���������֡�split_idxs1, split_idxs2�Ǳ���poly_idx�±�
		*/
		bool SplitSelectedPolyFromInterLNew(Eigen::Vector2f& ldir2d, Eigen::Vector2f& lpnt2d, std::vector<int>& poly_idx,
			std::vector<int>& split_idxs1, std::vector<int>& split_idxs2);

		/*
		Ŀ�ģ�����L_dir,L_point, polygon���Լ���corner_pnts_, �����ֱ�߷ָ���������֡���Щ�ǳ���Ҫ�����ں�����expand����
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
		Ŀ�ģ���rect�Ƿ񽻲����, ��������������
		*/
		int ComputeCrossPntFromRect(std::vector<Eigen::Vector2f>& rectpnts, 
			Eigen::Vector2f& ls, Eigen::Vector2f& le, Eigen::Vector2f& crosspnt0, Eigen::Vector2f& crosspnt1);

		bool ComputeSENearestLineCrossPnts(Eigen::Vector2f& ldir2d, Eigen::Vector2f& lpnt2d,
			Eigen::Vector2i& ls_pnts_idx, Eigen::Vector2f& ls_cross_pnt, 
			Eigen::Vector2i& le_pnts_idx, Eigen::Vector2f& le_cross_pnt);

		bool CheckLineSplitSrcPolygon(Eigen::Vector2f& ldir2d, Eigen::Vector2f& lpnt2d);
		
		/*
		Ŀ�ģ��������hf_poly_pnts �� ֱ�����޽���
		*/
		bool ComputeLine2NearestHalfPolyCrossPnt(Eigen::Vector2f& ldir2d, Eigen::Vector2f& lpnt2d, 
			std::vector<Eigen::Vector2f>& hf_poly_pnts, Eigen::Vector2f& cross_pnts);
		
		//bool CheckLineSplitSrcPolyNode(SplitNode* srcnode, Eigen::Vector2f& ldir2d, Eigen::Vector2f& lpnt2d);
		//bool CheckLineSegSplitSrcPolygon(Eigen::Vector2f& ls, Eigen::Vector2f& le);

		/*
		Ŀ�ģ�ͨ��t polygon �� �����λ�ã��ж�Src polygon��������������
		*/
		void CheckSrcPolygonCanonicalType();

		/*
		Ŀ�ģ�
		*/
		void DrawSrcPolygon2DImgCoord(cv::Mat& img);
		void DrawTgtPolygonLine2DImgCoord(cv::Mat& img);

		/*
		Ŀ�ģ�����ԭʼ��polygon ���������ȣ�����Tgt polygon������Ϳ��
		*/
		void ComputeSrcAndTgtParamsOnSrcCoord();

		/*
		Ŀ�ģ�����ԭʼ��polygon ���������ȣ�����Tgt polygon�������ͶӰ���߶�
		���룺idx ��ǰ��idx,����model_planes_ ��������poly_area, �߶�, ls, le
		*/
		void ComputePolyParamsOnSrcCoord(int idx, float& poly_area, 
			Eigen::Vector2f& ls, Eigen::Vector2f& le);

		/*
		Ŀ�ģ�����ldir2d,lpnt2d,��corner_pnts_�����и�γ�������polygon,ÿ������polygon
		���룺ldir2d,lpnt2d
		�����lnode,rnode
		*/
		bool LineSplitSrcPoly(Eigen::Vector2f& ldir2d, Eigen::Vector2f& lpnt2d, 
			SplitNode* lnode, SplitNode* rnode);
		
		/*
		Ŀ�ģ�����ls,le �߶�, ��corner_pnts_�����и�γ�������polygon,ÿ������polygon
		���룺lidx Ϊt����ε�����,�߶�ls,leΪlidx�����ͶӰ��ԭʼ����ε��߶�
		�����lnode,rnode
		*/
		bool LineSegSplitSrcPoly(int lidx, Eigen::Vector2f& ls, Eigen::Vector2f& le,
			SplitNode* lnode, SplitNode* rnode);

		/*
		Ŀ�ģ���corner_pnts_�������������½ǿ�ʼ����
		���룺corner_pnts_
		�����������corner_pnts_new_
		*/
		void SortCornerPnts();

		/*
		Ŀ�ģ�����corner_pnts_��bounding box
		*/
		void ComputeCornerPnts3dBoxPos(Eigen::Vector3f& mincor, Eigen::Vector3f& maxcor);
		void ComputeCornerPnts2dBoxPos(Eigen::Vector2f& mincor, Eigen::Vector2f& maxcor);

		/*
		Ŀ�ģ�����corner_pnts_�����½ǵĶ��㣨���ƣ������水��������������Щ���������µ����⣬
		������bounding box �������½ǵĶ������
		*/
		int ComputeApproxMinCorIdx();

		/*
		Ŀ�ģ�����ldir2d,lpnt2d,��corner_pnts_�����и��ʣ���SplitNode����split
		���룺ldir2d,lpnt2d,node,idx��ʾ�ڼ���t polygon,�����ں����ļ���Լ����
		�����lnode,rnode
		*/
		bool LineSplitSrcNode(int idx, Eigen::Vector2f& ldir2d, Eigen::Vector2f& lpnt2d,
			SplitNode* node, SplitNode* lnode, SplitNode* rnode);

		/*
		Ŀ�ģ�����L_dir,L_point, polygon���Լ���corner_pnts_, �����ֱ���и���߶�
		*/
		bool ComputeWholePolyInsectLIdxsFromLine(Eigen::Vector3f& L_dir, Eigen::Vector3f& L_point, 
			Eigen::Vector2i& line1pidx, Eigen::Vector2i& line2pidx, Eigen::Vector2f& l1_pnt, Eigen::Vector2f& l2_pnt);

		/*
		Ŀ�ģ�����L_dir,L_point, polygon���Լ���corner_pnts_, �����ֱ���и���߶�
		*/
		bool AddSplitPntsIntoSplitPoly(std::vector<int>& select_polygon, Eigen::Vector2i& line1pidx, Eigen::Vector2i& line2pidx,
			Eigen::Vector2f& l1_pnt, Eigen::Vector2f& l2_pnt, std::vector<Eigen::Vector2f>& poly_addedpnts);

		/*
		Ŀ�ģ�����L_dir,L_point, polygon���Լ���, �����ֱ���и���߶�
		*/
		bool ComputeSplitPolyInsectLDistToLine(Eigen::Vector3f& L_dir, Eigen::Vector3f& L_point, Eigen::Vector2f& m_pnt, Eigen::Vector2f& m_dir,
			std::vector<Eigen::Vector2f>& polypnts2d, Eigen::Vector2f& dist_near_far, Eigen::Vector2i& near_far_idx);

		/*
		Ŀ�ģ�����L_dir,L_point, polygon���Լ���split_idxs, �����ֱ�߷ָ���������֡���Щ�ǳ���Ҫ�����ں�����expand����
		*/
		bool ComputeSplitPolygonToIntersectionLine(Eigen::Vector3f& L_dir, Eigen::Vector3f& L_point,
			std::vector<int>& split_idxs, float& polygon_dist);

		/*
		Ŀ�ģ�����L_dir,L_point, polygon���Լ���split_idxs, ������polygon����L_dir ֱ�ߵľ��롣
		��Щ�ǳ���Ҫ�����ں�����expand����
		*/
		bool ComputePolygonRangeDistToIntersectionLineNew(Eigen::Vector3f& L_dir, Eigen::Vector3f& L_point,
			std::vector<int>& split_idxs, float& polygon_dist);

		/*
		Ŀ�ģ�����L_dir,L_point, polygon���Լ���corner_pnts_, ������polygon����L_dir ֱ�ߵľ�����Զ��ȥ�������
		��Щ�ǳ���Ҫ�����ں�����expand����
		*/
		bool ComputePolygonRangeDistToIntersectionLine(Eigen::Vector3f& L_dir, Eigen::Vector3f& L_point,
			std::vector<int>& split_idxs, float& polygon_dist);

		/*
		Ŀ�ģ�����l_dir2d,l_point2d, polygon���Լ���corner_pnts_, ������polygon����L_dir ֱ�ߵ���Զ�Ķ��㡣
		��Щ�ǳ���Ҫ�����ں�����expand����
		*/
		void ComputePolygonFarestDistToIntersectionLine(Eigen::Vector2f& l_dir2d, Eigen::Vector2f& l_point2d,
			int& farest_idx, float& max_dist);

		/*
		Ŀ�ģ�����l_dir2d,l_point2d, polygon���Լ���corner_pnts_, ������polygon����L_dir ֱ�ߵ���Զ�Ķ��㡣
		��Щ�ǳ���Ҫ�����ں�����expand����
		*/
		void ComputePolygonFarestDistToIntersectionLineSegment(Eigen::Vector2f& ls, Eigen::Vector2f& le,
			int& farest_idx, float& max_dist);

		/*
		Ŀ�ģ�����farestidx, polygon���Լ���corner_pnts_ idx, ��farestidx��ʼ��polygon idx��
		��Щ�ǳ���Ҫ�����ں�����expand����
		*/
		void RestartPolyIdxsFromSelectedIdx(int& farestidx, std::vector<int>& repolyidx);

		/*
		Ŀ�ģ�����L_dir,L_point, polygon���Լ���corner_pnts_, �����ֱ�߷ָ���������֡���Щ�ǳ���Ҫ�����ں�����expand����
		*/
		void ComputeIdxFromSplitPolygonIntersection(Eigen::Vector3f& L_dir, Eigen::Vector3f& L_point,
			std::vector<int>& split_idxs1, std::vector<int>& split_idxs2, std::vector<int>& moved_idxs);

		/*
		Ŀ�ģ�����L_dir,L_point, polygon���Լ���corner_pnts_, �����ƶ�������������µ�polygon����2d
		*/
		void ComputeNewPolygonPntsWithNomoveIdx(Eigen::Vector3f& L_dir, Eigen::Vector3f& L_point, 
			std::vector<int>& no_move_idxs, float dist_thresh, std::vector<Eigen::Vector2f>& polygon_new);

		/*
		Ŀ�ģ�����L_dir,L_point, polygon���Լ���corner_pnts_,
		�����ƶ����������Լ���ѡ��split polygon��lr_idx������µ�polygon����2d
		*/
		int ComputeNewPolyPntsWithNomoveIdxSplitPoly(Eigen::Vector3f& L_dir, Eigen::Vector3f& L_point, std::vector<int>& split_polygon, std::vector<int>& no_move_idxs, 
			Eigen::Vector2i& lr_idx, Eigen::Vector2i& connect_idx, float dist_thresh, std::vector<Eigen::Vector2f>& polygon_new);

		/*
		Ŀ�ģ����̴Ӽ�����������ϵ�еĶ���ӳ�䵽ƽ������ϵ���γ�ƽ������ϵ�µķ���
		*/
		void ComputeWorldLf2PlaneLf(Eigen::Vector3f& L_dir, Eigen::Vector3f& L_point, 
			Eigen::Vector2f& l_dir2d, Eigen::Vector2f& l_point2d);

		/*
		Ŀ�ģ������������Ҷ˵Ķ˵�Ͷ��ֱ���� �γɵ�һ��ֱ��,����ֱ��Ϊl_point2d,l_dir2d, ��Ҫlr_idx�ϵķ���ͶӰ����ֱ����
		*/
		void ComputeVerticalLineFromTwoPnts2D(Eigen::Vector2i lr_idx, Eigen::Vector2f& l_point2d, Eigen::Vector2f& l_dir2d,
			Eigen::Vector2f& mean_pnt, Eigen::Vector2f& mean_dir);

		/*
		Ŀ�ģ������߶ε������˵�Ĵ�ֱֱ���ϵ�ֱ�߷���
		*/
		void ComputeVerticalLFunctionFromTwoPnts2D(Eigen::Vector2f& ls, Eigen::Vector2f& le,
			Eigen::Vector2f& mean_pnt, Eigen::Vector2f& mean_dir);

		/*
		Ŀ�ģ������߶ε������˵��ֱ�߷���,���߶�תΪֱ�߷ų̣������ʽΪldir,lpnt
		*/
		void ComputeLdirFunctionFromEndPnts2D(Eigen::Vector2f& ls2d, Eigen::Vector2f& le2d,
			Eigen::Vector2f& ldir2d, Eigen::Vector2f& lpnt2d);

		/*
		Ŀ�ģ������߶κ�ֱ�ߵĽ��㣬����߶κ�ֱ���ཻ����true�����潻��
		*/
		bool ComputeLineSeg2DCrossLine2DPnts(Eigen::Vector2f& ls, Eigen::Vector2f& le,
			Eigen::Vector2f& mean_pnt, Eigen::Vector2f& mean_dir, Eigen::Vector2f& cross_pnt);

		Eigen::Vector2f ComputePlaneTwoLinesCrossPnt(Eigen::Vector2f& ss, Eigen::Vector2f& se,
			Eigen::Vector2f& ts, Eigen::Vector2f& te);

		bool ComputeLineSeg2DCrossLine2DPntsNew(Eigen::Vector2f& ls, Eigen::Vector2f& le,
			Eigen::Vector2f& t_pnt, Eigen::Vector2f& t_dir, Eigen::Vector2f& cross_pnt);

		bool ComputeTwoLineSegsCrossPnt(Eigen::Vector2f& ls, Eigen::Vector2f& le,
			Eigen::Vector2f& ts, Eigen::Vector2f& te, Eigen::Vector2f& cross_pnt);
		//��������ֱ�ߵĽ���,���ƽ�з���true,���߷���false
		bool ComputeTwoLinesCrosspnt(Eigen::Vector2f& ls, Eigen::Vector2f& le,
			Eigen::Vector2f& ts, Eigen::Vector2f& te, Eigen::Vector2f& cross_pnt);

		Eigen::Vector2f ComputePlaneTwoLinesCrossPntNew(Eigen::Vector2f& sp, Eigen::Vector2f& sdir,
			Eigen::Vector2f& tp, Eigen::Vector2f& tdir);

		/*
		Ŀ�ģ����������ȡ���� l_s, l_e,�����˵㣬��ȡ����f2d: f2d[0]*x + f2d[1]*y+f2d[2]=0;
		A:{a,b; c,d}X=D; ��(x0,x1);(x2,x3)����
		*/
		bool ComputePlaneFunctionFromTwoPnts(Eigen::Vector2f& l_s, Eigen::Vector2f& l_e, Eigen::Vector3f& f2d);

		/*
		Ŀ�ģ�����polygon�����һ�����ϣ����߶��Ϻ�Զ�ĵ㣨���Զ�������˵㣩
		*/
		void ComputeTwoPointsIdxFarestCorrepondingLine(Eigen::Vector3f& L_dir, Eigen::Vector3f& L_point, Eigen::Vector2i& pnts_idxs);

		//bool IsDegeneration(int idx,float4& plane_coeff);
		int IsSelfIntersection(int idx, float2& p_curr);
		int IsSelfIntersection(std::vector<float2>& corner_pnts, int idx, float2& p_curr);
		int IsSelfIntersections(std::vector<float2>& corner_pnts);	//�ж�polygon�Ƿ����Խ�����
		/*
		Ŀ�ģ��ж�polygon�У��µĵ��ƶ����Ƿ���ֽ�������
		idxΪ��ʼ�Ķ���������p_currΪidx�ƶ����Ķ���λ�ã�in_polygonΪ�����polygon
		*/
		int IsSelfIntersectionInPolygon(std::vector<Eigen::Vector2f>& in_polygon, int idx, Eigen::Vector2f& p_curr);

		/*
		Ŀ�ģ��ж�polygon�У��µĵ��ƶ����Ƿ���ֽ�������
		idxΪ��ʼ�Ķ���������p_currΪidx�ƶ����Ķ���λ�ã�in_polygonΪ�����polygon
		*/
		int IsSelfIntersectionInPolygonNew(std::vector<Eigen::Vector2f>& in_polygon, int idx, Eigen::Vector2f& p_curr);

		int IsSelfIntersectionsEigenNew(std::vector<Eigen::Vector2f>& corner_pnts);	//�ж�polygon�Ƿ����Խ�����

		//������������ϵ�µĶ��㵽ֱ�ߵľ���
		float Pnt3DDistToLine3D(Eigen::Vector3f& pnt, Eigen::Vector3f& L_dir, Eigen::Vector3f& L_point);

		/*
		Ŀ�ģ���ƽ������ϵ�£����㵽��ֱ�ߵľ���
		*/
		float Pnt2DDistToLine2D(Eigen::Vector2f& pnt2d, Eigen::Vector2f& L_point2d, Eigen::Vector2f& L_dir2d);

		/*
		Ŀ�ģ���ƽ������ϵ�£����㵽��ֱ�ߵľ���
		*/
		float Pnt2DDistToLine2DNew1(Eigen::Vector2f& pnt2d, Eigen::Vector2f& lpnt2d, Eigen::Vector2f& ldir2d);
		
		/*
		Ŀ�ģ���ƽ������ϵ�£����㵽��ֱ�ߵľ���ls, leΪֱ���ϵı�ʾ��ʽ
		*/
		float Pnt2DDistToLine2DLSLE(Eigen::Vector2f& pnt2d, Eigen::Vector2f& ls, Eigen::Vector2f& le);

		/*
		Ŀ�ģ����㶥�㵽�߶εľ���
		*/
		float Pnt2DToLineSegment2D(Eigen::Vector2f& pnt2d, Eigen::Vector2f& ls, Eigen::Vector2f& le);
		
		/*
		Ŀ�ģ���ƽ������ϵ�£����㵽��ֱ�ߵľ���, ��bug
		*/
		float Pnt2DDistToLine2DNew(Eigen::Vector2f& pnt2d, Eigen::Vector2f& ldir2d, Eigen::Vector2f& lpnt2d);

		//����ǽ�ƽ���ϵĶ���ƽ�Ƶ�ֱ�ߣ���ƽ������ϵ�²����ģ�����ƽ����ཻ�ߣ�
		float2 ProjToLine3D(int idx, Eigen::Vector3f& L_dir, Eigen::Vector3f& L_point);
		float2 ProjToPlane3D(int idx, float4& plane_coeff);
		float2 ProjToPlane3D(float3& pnt);
		float3 ProjToPlane(float3& pnt);
		
		Eigen::Vector3f Pnt3dProjToPlane3D(const Eigen::Vector3f& pnt);

		/*
		Ŀ�ģ���ƽ������ϵ�£�������ͶӰ��ֱ����
		*/
		void Pnt2dProjLine2D(Eigen::Vector2f& pnt2d, Eigen::Vector2f& ls, Eigen::Vector2f& ldir,
			Eigen::Vector2f& proj_pnt2d);

		/*
		Ŀ�ģ���ƽ������ϵ�£�������ͶӰ��ֱ����,�����-1�����������߶��⣬0 ���߶��ڣ�1 ���߶��⣬
		������proj_pnt2d
		*/
		int Pnt2dProjLineSeg2D(Eigen::Vector2f& pnt2d, Eigen::Vector2f& ls, Eigen::Vector2f& le,
			Eigen::Vector2f& proj_pnt2d);

		/*
		Ŀ�ģ���ƽ������ϵ�£������㷴Ͷ����������ϵ����ά����
		*/
		void Pnt2d2Pnt3D(Eigen::Vector2f& pnt2d, Eigen::Vector3f& pnt3d);

		/*
		Ŀ�ģ�����������ϵ�£�����ά����Ͷ��ƽ���ϵ�Ķ�ά����
		*/
		void Pnt3d2Pnt2D(Eigen::Vector3f& pnt3d, Eigen::Vector2f& pnt2d);

		bool IsAreaIncreasing(int idx, Eigen::Vector3f& L_dir, Eigen::Vector3f& L_point);
		bool IsSelectedAreaIncreasing(std::vector<Eigen::Vector2f>& polygonpnts, int idx, Eigen::Vector2f& projpnt);

		//����polygon ����Ĵ�С
		float ComputePolygon2DAreaFromPnts3d();

		//�������ε���������ں�������
		void ComputePolygon2DAreaFromCornerPnts3d();
		float GetPolygonCorner2DArea();

		bool GenerateInitial2DSortedPnts();

		void Generate2DPloygons();
		
		void GenerateSampleWorldCoordPnts();

		void MergeSampleAndSrcPnts();

		float3 Compute2DPlaneLineFunction(const float2& a, const float2& b);

		float ComputeVerticesDistIn2DPlane(const float2& a, const float2& b);

		float ComputeDistTo2DLine(const float2& a, const float3& f);

		////������Ƶ�Э�������
		//Eigen::Matrix2f Compute2DCovarianceMatrix(const std::vector<float2>& vertices);
		//float2 Compute2DVerticesNormal(const std::vector<float2>& vertices);
		//void Compute2DMatrixEigens(const Eigen::Matrix2f& matrix,
		//	Eigen::Matrix2f& eig_values, Eigen::Matrix2f& eig_vectors);

		float Compute2DVerticesCrossValue(const float2& a, const float2& b);

		//test
		std::vector<std::vector<float3> > lines_pos_;
		std::vector<std::vector<float3> > merge_lines_pos_;
		//end test

		//��ȡ���ı�Ե���ԭʼ�����Ӧ��������ֻ������žͿ�����
		bool MapSortedEdgePnt2PntPosIdx();

		//
		void PaintEdgePntsIntoImgAndSave();

		//���û���ͼƬ�Ĵ�С
		void SetImageSize(int w, int h);

		void ComputeImageConvertParams();

		void GenerateEdgeImagePolygon();

		//����ϵת��,��ƽ������ϵת����ͼ������ϵ
		cv::Point2f PlaneCoordPnt2ImageCoordPnt(const Point_2d& plane_2d_pnt);

		Point_2d ImageCoordPnt2PlaneCoordPnt(const cv::Point2f& image_pnt);

		//-----------------------------ͶӰ��2dƽ����---------------------------//

		//���㶥���weight
		float ComputeSortedPntsAvarageWeight(const std::vector<float>& pnts_weight);

		//triangulate polygon
		void TriangulatePolygon2D();
		
		//test
		void SamplePntsFromTriTest();
		//end test

		//---------------------------���ͶӰ��2dƽ����------------------------//
		//��һ������ͶӰ��2dƽ����
		float3 GetAPtProjectedToPlane(float3 pt);

		//������Ƶ�ƽ��λ��
		float3 ComputeAverageVertexPos();

		//������Ƶ�ƽ�������������ں����sample����ķ�������������������������
		Eigen::Vector3f ComputeAverageVertexNormal();

		//set split structrue,-1��ʾ��δ�������״̬��������ʾ���������״̬
		void SetSplitFlag(int split_flag);
		//�����ݴ��뵽����ṹ����
		void SetSplitStructure(std::vector<HW::HWPlane*> model_planes,
			std::vector<int>& related_idxs);

		/*
		Ŀ�ģ�������Χ��polygon�ķ�Χ
		*/
		bool BuildAroundPolygonPnts2d(std::vector<std::vector<int> >& polygons_adjs);

		/*
		Ŀ�ģ��ж��߶κ�polygon�Ƿ��н���
		*/
		bool CheckCrossPntsWithPolygon(Eigen::Vector2f& ls, Eigen::Vector2f& le);

		/*
		Ŀ�ģ�����ƴ���߶εķ���������ԭʼ��src��polygon����һ��
		*/
		void SetAdjLinesDirect();

		bool CheckVecFlag(std::vector<bool>& flags);

		/*
		Ŀ�ģ��Ż�polygon
		*/
		void OpitmizeAdjLinesPolygon();

		/*
		Ŀ�ģ�����related idx �� polygon�ķ��򱣳�һ��,ʹ��polygon�ܹ��γ�
		*/
		bool SetRelatedIdxAroundSrcPoly();

		/*
		Ŀ�ģ�ͨ���߶ι������Σ�����������߶Σ����п��(��Ҫ����������),��ʱ��
		*/
		bool ContructRectFromLS2D(Eigen::Vector2f& ls, Eigen::Vector2f& le, 
			float width, std::vector<Eigen::Vector2f>& rect_pnts);

		bool ComputeNearestDist2ParallAdjLines(int srcidx, float& dist);

		/*
		Ŀ�ģ��жϷ���,�������һ�£��򷵻�true����Ϊfalse
		*/
		bool CheckLineDirectWithPolygonDirect(std::vector<int>& lines_idxs);

		/*
		Ŀ�ģ��жϷ���,�������һ�£��򷵻�true����Ϊfalse
		*/
		void ConstructHalfPolyIdxs(Eigen::Vector2i& s2e, std::vector<int>& half_poly);

		/*
		Ŀ�ģ���������polygon�Ƿ�Ϊ�ڽ���ϵ
		*/
		bool CheckTwoPolysAdj(int src_idx, int tgt_idx, std::vector<std::vector<int> >& polygons_adjs);

		////���������ڶ�
		//bool VoteForExpandPolygon(std::vector<Eigen::Vector3f>& pnts, 
		//	std::vector<Eigen::Vector3f>& polygon_pnts);

		//��related_poly_idxs_��������
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

		//֤����������ľ�����threshold��Χ��
		bool CheckThresholdTwoPnts(double threshold, Point_2d& a, Point_2d& b);

		//��ƽ����ƽ���sor filter
		void PointCloudSORFilter();
		
		//����ͶӰ��2d���Ƶ�box
		void Compute2DPntsBoxPos(float2& min_corner, float2& max_corner);

		//������Ƶ�Э�������
		Eigen::Matrix3f ComputeCovarianceMatrix(const std::vector<float3>& vertices);

		//��������Eigen
		void ComputeMatrixEigens(const Eigen::Matrix3f& matrix,
			Eigen::Matrix3f& eig_values, Eigen::Matrix3f& eig_vectors);

		//------------------------�����ǻ���Polygon���в�����------------------//
		bool SamplePntsFromPolygon();
		void SamplePntsFromTri(std::vector<Point2>& result_pnt, 
			std::vector<Point2> tri_pnts, int point_density);
		//
		ccPlane* associated_cc_plane_;

		HWPointCloud* filtered_plane_pc;

		//ƽ�淽��ax + by + cz + d = 0;
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

		//ƽ������ϵ����ߵ�����ұߵ�
		float left_bottom_;
		float right_top_;

		//ƽ��Ķ�������
		std::vector<int> pnts_idx_;
		std::vector<float3> pnts_pos_origin_;
		std::vector<float3> pnts_normal_origin_;
		std::vector<float3> pnts_color_origin_;
		std::vector<float3> pnts_pos_;
		std::vector<float3> pnts_normal_;
		std::vector<uchar3> pnts_color_;

		//ƽ��ı�Ե����
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

		//------------��Щ������Ҫ�ģ�����Ҫȥ��һЩ�����õĶ���-----------------------//
		//������ƽ����������ϵ�µ�����
		std::vector<float3> plane_coord_pos_;
		std::vector<float3> plane_coord_pnts_normal_;

		//��Щ������2dͶӰ�ϣ������Ե���������ı�
		std::vector<Segment> border_edges_;
		
		//�����ԭʼ�ı�Ե�ߺϳɱպϵı�Ե��
		std::vector<Segment> sorted_border_edges_2d_;

		//���Ǳ�sorted_border_edges_2d_ȡ���еĿ�ͷ�Ķ���õ����������ͨ��Polygon���Եõ��򻯺��polygon
		std::vector<Point_2d> sorted_border_pnts_2d_;
		std::vector<float> sorted_border_pnts_weight_2d_;
		std::vector<int> sbp_to_pnts_pos;
		bool map_pnts_flag = false;

		//ת����ͼ������ϵ�У���Ҫƽ��x,y������,��ƽ������ת�Ƶ���ֵ,Ȼ����ƽ�ƶ���
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
		//�����Ķ����ԭʼ�Ķ���ϲ�
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
		//�Ƿ��ֶ��޸Ĺ�
		bool manually_edited_ = false;
		float corners_area_2d_ = 0.0;

		float poly_dist_threshold_;
		float r_poly_dist_max_;

		//������һЩ���ݡ���������ʵ������㷨��
		std::vector<HW::HWPlane*> model_planes_;	//��Ч��planes_vec_
		int splited_label_ = -1;	//��ǰidx
		//�Զ���ν����и�и�����ƽ������
		std::vector<int> related_poly_idxs_;	//���ǽӽ�x����Ĺ�����polygon
		std::pair<Eigen::Vector2f, Eigen::Vector2f> corners_width_2d_;
		std::vector<float> related_poly_corners_areas_2d_;
		std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f> > related_poly_corners_widths_2d_;	//ls �� le
		std::vector<std::vector<int> > related_polys_around_pnts_idx_;	//�����ÿ��polygon ��idx ��Ӧ��polygon,����related_poly_idxs_˳��һ��
		std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f> > related_polys_around_pnts_pos_;
		std::vector<Eigen::Vector2f> around_polygon_pnts_new_;	//Χ����polygon��Χ��pnts,������ƴ��
		std::vector<int> sorted_around_related_poly_idxs_;	//�����related_polys_around_pnts_idx_�±��polygon
		bool around_polygon_pnts_closed_ = false;
		//ƽ������ϵͶӰ��ͼ������ϵ���������ڴ���ͼ��
		int plane_image_height_;
		int plane_image_width_;

		//��ʾpolygon �Ѿ�������������
		bool resorted_sor_corner_flag_ = false;
		bool related_param_flag = false;

		//����polygon������:͹���߷�͹
		PolyonCanonicalType polygon_cannoical_;
		//std::vector<SplitNode*> split_poly_nodes_;
		//������µ�polygon
		std::vector<Eigen::Vector3f> poly_pnts_3d_new_;

		//-------------------------------end-------------------------------------//

		//û��ƽ���Ե�Ķ��㣬��ʵ����λ�ÿ��Բ���Ҫ��
		std::vector<int> pnts_idx_without_edge_;

		//��������ϵ��ƽ������ϵ�ľ���
		Eigen::Matrix4f world_to_plane_;
		//�൱��һ��������굽��������ı任�������Ͻ�3x3�����ÿһ�д�������е�ÿ����ķ���
		Eigen::Matrix4f plane_to_world_;	//cam pos

		float area_ = 0;
	};
}
