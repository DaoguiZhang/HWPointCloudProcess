#include "hw_polygon.h"

namespace HW
{
	HWPolygon::HWPolygon()
	{
		point_cloud_ = nullptr;
	}
	HWPolygon::~HWPolygon()
	{
		for (int i = 0; i < planes_vec_.size(); i++) {
			if (planes_vec_[i] != nullptr) {
				delete planes_vec_[i];
				planes_vec_[i] = nullptr;
			}
		}
		delete point_cloud_;
		point_cloud_ = nullptr;
	}
	bool HWPolygon::Show()
	{
		return false;
	}
	bool HWPolygon::Save(std::string & file)
	{
		return false;
	}
	bool HWPolygon::SavePly(const std::string & file, const PlyFormat & type)
	{
		return false;
	}
	bool HWPolygon::ReadPly(const std::string & file)
	{
		return false;
	}
	void HWPolygon::addPlane(HWPlane * plane)
	{
		planes_vec_.emplace_back(plane);
	}
	void HWPolygon::addPointCloud(HWPointCloud * pc)
	{
		point_cloud_ = pc;
	}
	const std::vector<HWPlane*>& HWPolygon::getPlaneVec()
	{
		return planes_vec_;
	}
	HWPointCloud * HWPolygon::getPointCloud()
	{
		return point_cloud_;
	}

	void HWPolygon::SetPlane2PCFlag(bool flag)
	{
		corresponding_flag_ = flag;
	}

	bool HWPolygon::GetPlane2PCFlag()
	{
		return corresponding_flag_;
	}

	int HWPolygon::GetPolygonsNum()
	{
		return int(planes_vec_.size());
	}

	HW::HWPlane* HWPolygon::GetHWPlane(int idx)
	{
		if (idx >= planes_vec_.size() || idx < 0)
			return NULL;
		return planes_vec_[idx];
	}

	void HWPolygon::SetRmax(float r)
	{
		r_max_v_ = r;
	}

	float HWPolygon::GetRmax()
	{
		return r_max_v_;
	}

	/*PolygonWindow* HWPolygon::GetShowObjWidgetPtr()
	{
		return associated_show_widget_;
	}*/

	void MergeHWPolygon(HWPolygon& target, std::vector<HWPolygon*> sources)
	{
		std::cout << "begin MergeHWPolygon" << std::endl;
		if (sources.size() < 2) return;
		std::vector<HWPlane*> planes;
		HWPointCloud* point_cloud = new HWPointCloud();
		std::string name;

		for (int i = 0; i < sources.size(); ++i) {
			HWPolygon* curr_polygon = sources[i];
			for (int j = 0; j < curr_polygon->planes_vec_.size(); ++j) {
				HWPlane* curr_plane = new HWPlane();
				*curr_plane = *(curr_polygon->planes_vec_[j]);
				planes.push_back(curr_plane);
			}
			//planes.insert(planes.end(), sources[i]->planes_vec_.begin(), sources[i]->planes_vec_.end());
			if (sources[i]->point_cloud_) point_cloud->Merge(*(sources[i]->point_cloud_));
		}
		target.planes_vec_ = planes;
		target.point_cloud_ = nullptr;
		std::cout << "planes.size() = " << planes.size() << std::endl;

		std::string new_name = sources[0]->GetObjectName().substr(0, sources[0]->GetObjectName().find_last_of("."));
		for (int i = 1; i < sources.size(); ++i) {
			std::string curr_name = sources[i]->GetObjectName().substr(
				sources[i]->GetObjectName().find_last_of("/\\") + 1,
				sources[i]->GetObjectName().find_last_of(".") - sources[i]->GetObjectName().find_last_of("/\\") - 1
			);
			std::cout << "curr_name = " << curr_name << std::endl;
			new_name += ("_" + curr_name);
		}
		new_name += ".obj";
		std::cout << "new_name = " << new_name << std::endl;
		target.SetObjectName(new_name);
		std::cout << "end MergeHWPolygon" << std::endl;
	}
}
