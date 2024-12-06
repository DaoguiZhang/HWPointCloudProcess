#include"hw_scene_proj_layout3d.h"

namespace HW
{
	HWSceneProjLayout3D::HWSceneProjLayout3D()
	{
	}

	HWSceneProjLayout3D::~HWSceneProjLayout3D()
	{
	}

	void HWSceneProjLayout3D::SetSceneLayout3D(const std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f> >& pnts)
	{
		scenes_layout3d_.resize(pnts.size());
		for (int i = 0; i < pnts.size(); ++i)
		{
			scenes_layout3d_[i] = pnts[i];
		}
	}

	void HWSceneProjLayout3D::SetSceneLayout3DValid(const std::vector<std::pair<bool, bool> > linesvalid)
	{
		lines_3d_valid_.resize(linesvalid.size());
		for (int i = 0; i < linesvalid.size(); ++i)
		{
			lines_3d_valid_[i] = linesvalid[i];
		}
	}

	const std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f> >& HWSceneProjLayout3D::GetSceneLayout3D()
	{
		return scenes_layout3d_;
	}

	const std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f> >& HWSceneProjLayout3D::GetSceneLayout3D() const
	{
		return scenes_layout3d_;
	}

	int HWSceneProjLayout3D::GetLayout2dId()
	{
		return layout2d_id_;
	}

	void HWSceneProjLayout3D::SetLayout2dId(int ly2did)
	{
		layout2d_id_ = ly2did;
	}

	HWSceneProjLayout3D HWSceneProjLayout3D::operator = (const HWSceneProjLayout3D& other)
	{
		this->layout2d_id_ = other.layout2d_id_;
		this->lines_3d_valid_ = other.lines_3d_valid_;
		this->scenes_layout3d_ = other.scenes_layout3d_;
		return *this;
	}
}