#pragma once
#ifndef HW_POINT_CLOUD_H
#define HW_POINT_CLOUD_H

//pcl
#include <pcl/surface/mls.h>

#include "hw_object.h"
#include <CloudCompareDisplay/ccColorTypes.h>

namespace HW
{
	class HWPointCloud:public HWObject
	{
	public:
		friend void MergeClutteredHWPointCloud(HWPointCloud& target, HWPointCloud& c1, HWPointCloud& c2);
		HWPointCloud();
		HWPointCloud(const std::vector<float3>& in_points);
		HWPointCloud(const std::vector<float3>& in_points, const std::vector<float3>& in_normal);
		//bool ReadPly();
		~HWPointCloud();

		bool HasNormal();
		bool HasColor();

		//copy属性
		void CopyPointsFromPlyData(std::shared_ptr<tinyply::PlyData>& point_element);

		void CopyNormalsFromPlyData(std::shared_ptr<tinyply::PlyData>& normal_element);

		void CopyColorsFromPlyData(std::shared_ptr<tinyply::PlyData>& color_element);

		void AddPoint(float3& point);
		void SetPoint(const std::vector<float3>& v);
		void AddNormal(float3& point_normal);
		void SetNormal(const std::vector<float3>& v);
		void AddColor(uchar3& point_color);
		void SetColor(uchar3& point_color,int idx);
		void SetColor(const std::vector<uchar3>& v);
		void SetSemantic(const std::vector<int>& sem);
		void SetPlanesIdx(std::vector<int>& planes_idx);
		void SetPlanesIsWide(std::vector<int>& planes_iswide);
		void SetBoundingBox(float3& box_min, float3& box_max);

		float3 GetAPoint(int idx);
		float3 GetANormal(int idx);
		uchar3 GetAColor(int idx);

		const std::vector<float3>& GetVertices();
		const std::vector<float3>& GetNormal();
		const std::vector<uchar3>& GetPointsColor();
		const std::vector<int>& GetPointsSemanticLabel();
		int GetSemanticLabel();
		const std::vector<int>& GetPlanesIdx();
		const std::vector<int>& GetPlanesIsWide();
		void getBoundingBox(float3& box_min, float3& box_max);

		bool SetAPoint(float3 in_point, int idx);

		bool ReadPly(const std::string& file);

		void AddRandoNormNoise();

		bool Show();
		bool Save(std::string& file);
		bool SavePly(const std::string& file, const PlyFormat& type);
		void AllocateInfoSpace(int num);
		void RemoveInfoSpace();
		void InitGeometricInfo();
		bool IsCluttered() const { return !planes_idx_.empty(); }
		void Merge(HWPointCloud& pc);

		//计算点云的平均密度
		float ComputePointCloudResolution(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
		int ComputePointsNumPerSquareMeter();

	private:
		Eigen::Matrix3f ComputeCovarianceMatrix(int id, int start_idx, int end_idx);
		void ComputeAvePointAndNormal(int id, int start_idx, int end_idx);
		void ComputeBoundingBox(int id, int start_idx, int end_idx);
		void ComputeWorldToPlaneMatrix(int id, int start_idx, int end_idx);
		void ComputeSingleGeometricInfo(int id);
		static bool ShouldMerge(const HWPointCloud & c1, const HWPointCloud & c2, int a, int b);
		int points_num_ = 0;
		std::vector<float3> points_;
		std::vector<uchar3> points_color_;
		std::vector<int> points_semantic_label_;
		std::vector<float3> normal_;
		std::vector<int> planes_idx_;	//[0,12,17,...]0表示初始平面点云索引 12-0为对一个平面的点云数量，17-12为下个平面的点云数量
		std::vector<int> planes_iswide_;
		float3 box_min_;
		float3 box_max_;

		std::vector<float3> ave_normals_;
		std::vector<float3> ave_points_;
		std::vector<float3> plane_normals_;
		std::vector<Eigen::Matrix4f> world_to_plane_;
		std::vector<float3> bounding_box_mins_;
		std::vector<float3> bounding_box_maxs_;
	};

	void MergeClutteredHWPointCloud(HWPointCloud& target, HWPointCloud& c1, HWPointCloud& c2);
}

#endif
