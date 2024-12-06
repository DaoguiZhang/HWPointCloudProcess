#pragma once

#ifndef HW_SCENE_PROJ_LAYOUT3D_H
#define HW_SCENE_PROJ_LAYOUT3D_H

#include<vector>
#include<string>
#include<Eigen/core>

namespace HW
{
	class HWSceneProjLayout3D
	{
	public:
		HWSceneProjLayout3D();

		~HWSceneProjLayout3D();
		void SetSceneLayout3D(const std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f> >& pnts);
		void SetSceneLayout3DValid(const std::vector<std::pair<bool, bool> > linesvalid);

		const std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f> >& GetSceneLayout3D();
		const std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f> >& GetSceneLayout3D() const;
		int GetLayout2dId();
		void SetLayout2dId(int ly2did);
		HWSceneProjLayout3D operator = (const HWSceneProjLayout3D& other);

		int layout2d_id_;
	private:
		std::vector<std::pair<bool, bool> > lines_3d_valid_;
		std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f> > scenes_layout3d_;
	};
}

#endif