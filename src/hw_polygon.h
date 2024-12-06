#pragma once
#include "hw_plane.h"
#include <vector>
#include <string>
#include <iostream>

namespace HW
{
	class HWPolygon : public HWObject 
	{
	public:
		friend void MergeHWPolygon(HWPolygon& target, std::vector<HWPolygon*> sources);
		HWPolygon();
		HWPolygon(std::vector<HWPlane*>& planes, HWPointCloud* point_cloud):
			planes_vec_(planes), point_cloud_(point_cloud) {}
		~HWPolygon();
		bool Show();
		bool Save(std::string& file);
		bool SavePly(const std::string& file, const PlyFormat& type);
		bool ReadPly(const std::string& file);
		void addPlane(HWPlane* plane);
		void addPointCloud(HWPointCloud* pc);
		const std::vector<HWPlane*>& getPlaneVec();
		HWPointCloud* getPointCloud();
		void SetPlane2PCFlag(bool flag);	//设置点云和polygon是否具有对应关系
		bool GetPlane2PCFlag();

		int GetPolygonsNum();
		HW::HWPlane* GetHWPlane(int idx);
		void SetRmax(float r);
		float GetRmax();

	private:

		float r_max_v_;
		bool corresponding_flag_ = false;
		std::vector<HWPlane*> planes_vec_;
		HWPointCloud* point_cloud_;
	};

	void MergeHWPolygon(HWPolygon& target, std::vector<HWPolygon*> sources);
}