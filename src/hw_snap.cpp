#include "hw_snap.h"

namespace HW
{
	HWSNAP::HWSNAP()
	{
	}

	HWSNAP::~HWSNAP()
	{
	}



	void HWSNAP::Process(HWObject* in_element, HWObject* out_element) {
		std::cout << "start snap process" << std::endl;
		HWPointCloud* in_pointcloud = dynamic_cast<HWPointCloud*> (in_element);
		if (!in_pointcloud->IsCluttered()) return;

		std::string original_name = in_pointcloud->GetObjectName();
		std::cout << "original name = " << original_name << std::endl;
		std::string head_name = original_name.substr(0, original_name.find_last_of("."));
		std::cout << "head name = " << head_name << std::endl;
		std::string new_name = head_name + ".obj";
		std::cout << "new_name = " << new_name << std::endl;

		std::vector<int> planes_idx = in_pointcloud->GetPlanesIdx();
		std::vector<int> planes_iswide = in_pointcloud->GetPlanesIsWide();
		std::vector<float3> points_vec = in_pointcloud->GetVertices();
		//std::vector<int> semantic_labels = in_pointcloud->GetPointsSemanticLabel();
		int semantic_label = in_pointcloud->GetSemanticLabel();
		std::vector<float3> normals_vec = in_pointcloud->GetNormal();
		std::vector<uchar3> color_vec = in_pointcloud->GetPointsColor();
		std::cout << "planes_idx.size() = " << planes_idx.size() << std::endl;
		for (const auto& a : planes_idx) {
			std::cout << a << ", ";
		}
		std::cout << std::endl;
		/*for (int i = 0; i < 1; i++) {
		HWPlane* plane = new HWPlane();
		for (int j = 0; j < points_vec.size(); j++) {
			plane->AddPlanePnt(points_vec[j]);
			plane->AddPlanePntNormal(normals_vec[j]);
			plane->SetDiameter(0.05f);
		}*/
		for (int i = 0; i < planes_idx.size() - 1; i++) {
			HWPlane* plane = new HWPlane();
			for (int j = planes_idx[i]; j < planes_idx[i + 1]; j++) {
				plane->AddPlanePnt(points_vec[j]);
				plane->AddPlanePntNormal(normals_vec[j]);
				//Liulingfei&&zdg
				if (!color_vec.empty())
				{
					//uchar3 c = color_vec[j];
					//std::cout << "c: " << (int)(c.x) << " " << (int)(c.y) << " " << (int)(c.z) << std::endl;
					int3 c_i = make_int3((int)(color_vec[j].x), (int)(color_vec[j].y), (int)(color_vec[j].z));
					//std::cout << "c_i: " << c_i.x << " " << c_i.y << " " << c_i.z << std::endl;
					float c_x = (float)(c_i.x / 255.0);
					float c_y = (float)(c_i.y / 255.0);
					float c_z = (float)(c_i.z / 255.0);
					float3 c_f = make_float3(c_x, c_y, c_z);
					//std::cout << "c_f: " << c_f.x << " " << c_f.y << " " << c_f.z << std::endl;
					plane->AddPlaneOriginPntColor(c_f);
				}
				plane->SetSemanticLabel(semantic_label);
				//AddPlaneOriginPntColor()
			}

			//if (planes_iswide[i])
				//plane->SetDiameter(0.05);
			//else
				//plane->SetDiameter(0.05f);
			plane->SetDiameter(0.05);

			planes_vec_.emplace_back(plane);
			std::cout << "run snap process: the plane cloud size is " << plane->GetPlanePnts().size() << std::endl;
		}

		//system("pause");
		HWPlane* plane_virtual = new HWPlane();
		float3 bb_min, bb_max;
		in_pointcloud->getBoundingBox(bb_min, bb_max);
		//std::vector<float3> polygon_virtual;
		//float ceil_height = HW::HWParams::getInstance().plane_params_.ceil_height_;
		//polygon_virtual.emplace_back(make_float3(bb_min.x, bb_min.y, ceil_height));
		//polygon_virtual.emplace_back(make_float3(bb_min.x, bb_max.y, ceil_height));
		//polygon_virtual.emplace_back(make_float3(bb_max.x, bb_max.y, ceil_height));
		//polygon_virtual.emplace_back(make_float3(bb_max.x, bb_min.y, ceil_height));
		//Eigen::Matrix4f cam_pose_virtual;
		//cam_pose_virtual.setIdentity();
		//cam_pose_virtual(0, 3) = (bb_min.x + bb_max.x) / 2;
		//cam_pose_virtual(1, 3) = (bb_min.y + bb_max.y) / 2;
		//cam_pose_virtual(2, 3) = ceil_height;
		//plane_virtual->SetCameraPose(cam_pose_virtual);
		//plane_virtual->SetCornerPts(polygon_virtual);
		//planes_vec_.emplace_back(plane_virtual);
//#pragma omp parallel for, planes_vec_.size()
		std::vector<HWPlane*> cur_planes;	//保存实际上具有平面的polygon，对于没有提取出来的polygon删除掉
		for (int i = 0; i < planes_vec_.size(); i++) {
			//if (i != 349) continue;
			std::stringstream ss;
			std::string index;
			ss << i;
			ss >> index;
			std::string filename = in_element->GetObjectName() + index;
			std::cout << std::endl << "i = " << i << std::endl;
			if (planes_iswide[i]) std::cout << "plane is wide" << std::endl;
			else std::cout << "plane is not wide" << std::endl;
			planes_vec_[i]->SetFilename(filename);
			planes_vec_[i]->extractPolygon();
			if (planes_vec_[i]->GetOuterPolygon().empty())
			{
				std::cout << "polygon " << i << "extract failed failed" << std::endl;
				delete planes_vec_[i];
				planes_vec_[i] = NULL;
			}
			else
			{
				cur_planes.emplace_back(planes_vec_[i]);
			}
			std::cout << "extract polygon done" << std::endl;
		}
		//去除掉
		planes_vec_.clear();
		for (int i = 0; i < cur_planes.size(); ++i)
		{
			planes_vec_.emplace_back(cur_planes[i]);
		}

		for (int i = 0; i < planes_vec_.size(); ++i)
		{
			int corner_num = planes_vec_[i]->GetOuterPolygon().size();
			std::cerr << "the " << i << " corner num: " << corner_num << std::endl;
		}

		std::cout << "first loop done" << std::endl;
		/*std::vector<int> polygon_idxs;
		polygon_idxs.emplace_back(0);
		for (int i = 0; i < planes_vec_.size(); i++) {
			int num = planes_vec_[i]->GetOuterPolygon().size();
			polygon_idxs.emplace_back(polygon_idxs.back() + num);

		}
		std::ofstream f0("g:\\xht\\huawei\\2019-04-28_14.54.52\\corner_pnts_3d1.obj");
		for (int i = 0; i < planes_vec_.size(); i++) {
			std::vector<float3> pnts = planes_vec_[i]->GetOuterPolygon();
			for (int j = 0; j < pnts.size(); ++j)
			{
				f0 << "v " << pnts[j].x << " " << pnts[j].y << " " << pnts[j].z << "\n";
			}
		}
		for (int i = 0; i < polygon_idxs.size() - 1; i++) {
			if (polygon_idxs[i] == polygon_idxs[i + 1])
				continue;
			f0 << "f ";
			for (int j = polygon_idxs[i]; j < polygon_idxs[i + 1]; j++) {
				f0 << j + 1 << " ";
			}
			f0 << "\n";
		}
		f0.close();*/
#if 0
		std::cout << "planes_vec_.size(): " << planes_vec_.size() << "\n";
		for (int i = 0; i < planes_vec_.size() - 1; i++) {
			if (planes_iswide[i])
				continue;
			Eigen::Matrix4f camera_pose = planes_vec_[i]->GetCameraPose();
			Eigen::Vector3f average_pos = camera_pose.block(0, 3, 3, 1);
			Eigen::Vector3f box_min, box_max;
			planes_vec_[i]->GetBoundingBox(box_min, box_max);
			float one_third_height = box_min.z() + (box_max.z() - box_min.z()) / 3;
			float two_third_height = box_min.z() + 2*(box_max.z() - box_min.z()) / 3;
			float4 coeff = planes_vec_[i]->GetPlaneCoeff();
			Eigen::Vector3f normal(coeff.x, coeff.y, coeff.z);
			int horizontal_low_idx = -1;
			float horizontal_low_height = -FLT_MAX;
			int horizontal_high_idx = -1;
			float horizontal_high_height = cam_pose_virtual(2, 3);
			std::vector<int> vertical_idxs;

			for (int j = 0; j < planes_vec_.size(); j++) {
				if (!planes_iswide[j] && j!= planes_vec_.size()-1)
					continue;
				Eigen::Matrix4f camera_pose_j = planes_vec_[j]->GetCameraPose();
				float4 coeff_j = planes_vec_[j]->GetPlaneCoeff();
				Eigen::Vector3f normal_j(coeff_j.x, coeff_j.y, coeff_j.z);
				if (abs(coeff_j.z) > 0.9) {
					float dist;
					float2 proj = planes_vec_[j]->ProjToPlane3D(make_float3(average_pos.x(), average_pos.y(), camera_pose_j(2, 3)));
					/*if (planes_vec_[j]->IsInPolygon(proj))
						dist = 0;
					else*/
						dist = planes_vec_[j]->GetDistToPolygon(Eigen::Vector3f(average_pos.x(), average_pos.y(), camera_pose_j(2, 3)));
					if (dist < 0.2f&&horizontal_low_height < camera_pose_j(2, 3) && one_third_height>camera_pose_j(2, 3)) {
						horizontal_low_idx = j;
						horizontal_low_height = camera_pose_j(2, 3);
					}
					else if (dist < 0.2f&&horizontal_high_height > camera_pose_j(2, 3) && two_third_height < camera_pose_j(2, 3)){
						horizontal_high_idx = j;
						horizontal_high_height = camera_pose_j(2, 3);
					}
				}
				else {
					float dist = planes_vec_[j]->GetDistToPolygon(Eigen::Vector3f(average_pos.x(), average_pos.y(), horizontal_high_height - 1));
					/*Eigen::Vector3f L_dir, L_point;
					float4 coeff1 = planes_vec_[i]->GetPlaneCoeff();
					float4 coeff2 = planes_vec_[j]->GetPlaneCoeff();
					Eigen::Vector3f n1(coeff1.x, coeff1.y, coeff1.z);
					Eigen::Vector3f n2(coeff2.x, coeff2.y, coeff2.z);
					L_dir = n1.cross(n2);
					int flag[3] = { 1,1,1 };
					for (int k = 0; k < 3; k++) {
						if (abs(n1(k)) > 0.9 || abs(n2(k)) > 0.9)
							flag[k] = 0;
					}
					Eigen::Matrix2f A;
					Eigen::Vector3f pos = planes_vec_[i]->GetCameraPose().topRightCorner(3, 1);
					if (flag[0] == 1) {
						A << n1[1], n1[2], n2[1], n2[2];
						Eigen::Vector2f B(-coeff1.w - pos[0] * n1[0], -coeff2.w - pos[0] * n2[0]);
						Eigen::Vector2f point = A.inverse()*B;
						L_point = Eigen::Vector3f(pos[0], point(0), point(1));
					}
					else if (flag[1] == 1) {
						A << n1[0], n1[2], n2[0], n2[2];
						Eigen::Vector2f B(-coeff1.w - pos[1] * n1[1], -coeff2.w - pos[1] * n2[1]);
						Eigen::Vector2f point = A.inverse()*B;
						L_point = Eigen::Vector3f(point(0), pos[1], point(1));
					}
					else {
						A << n1[0], n1[1], n2[0], n2[1];
						Eigen::Vector2f B(-coeff1.w - pos[2] * n1[2], -coeff2.w - pos[2] * n2[2]);
						Eigen::Vector2f point = A.inverse()*B;
						L_point = Eigen::Vector3f(point(0), point(1), pos[2]);
					}
					Eigen::Vector3f e1 = average_pos - L_point;
					float length = abs(e1.dot(L_dir));
					float dist = sqrt(e1.squaredNorm() - length*length);*/
					Eigen::Vector3f normal_j(coeff_j.x, coeff_j.y, coeff_j.z);
					if (dist < 0.3f&&abs(normal.dot(normal_j)) < 0.95){
						/*Eigen::Vector3f average_pos1 = planes_vec_[j]->GetCameraPose().block(0, 3, 3, 1);
						Eigen::Vector3f normal1 = planes_vec_[j]->GetCameraPose().block(0, 2, 3, 1);
						bool flag = true;
						for (int k = 0; k < vertical_idxs.size(); k++) {
							Eigen::Vector3f average_pos2 = planes_vec_[vertical_idxs[k]]->GetCameraPose().block(0, 3, 3, 1);
							Eigen::Vector3f normal2 = planes_vec_[vertical_idxs[k]]->GetCameraPose().block(0, 2, 3, 1);
							float xy_dist = sqrt(pow(average_pos1(0) - average_pos2(0), 2) + pow(average_pos1(1) - average_pos2(1), 2));
							if (xy_dist < 0.2&&abs(normal1.dot(normal2) > 0.9))
								flag = false;
						}
						if(flag)*/
							vertical_idxs.emplace_back(j);
					}
				}
				
			}
			if (horizontal_low_idx == -1) {
				horizontal_low_idx = 0;
				horizontal_low_height = 0;
			}
			/*if (horizontal_high_idx == -1) {
				horizontal_high_idx = 0;
				horizontal_high_height = 8.0;
			}*/

			std::cout << "horizontal_low_height:" << horizontal_low_height << "\n";
			std::cout << "horizontal_high_height:" << horizontal_high_height << "\n";
			std::vector<float3> pos_vec;
			if (vertical_idxs.size() != 2) {
				std::vector<int> idx_temp;
				for (int j = 0; j < vertical_idxs.size(); j++) {
					Eigen::Matrix4f camera_pose_j = planes_vec_[j]->GetCameraPose();
					if (camera_pose_j(2, 3) >= horizontal_low_height&&horizontal_low_height <= horizontal_high_height)
						idx_temp.emplace_back(vertical_idxs[j]);
				}
				vertical_idxs = idx_temp;
			}
			std::cout << "i vertical_idxs.size: " << i << "  " << vertical_idxs.size() << std::endl;
			if (vertical_idxs.size() != 2) {
				/*planes_vec_[i]->ProjectTo3DPlane();
				planes_vec_[i]->Generate2DPlane();
				planes_vec_[i]->DoEstimateBorderFrom2DPlane();
				planes_vec_[i]->SortBorderEdges();
				planes_vec_[i]->GenerateInitial2DSortedPnts();
				planes_vec_[i]->GetSmoothRegions();
				planes_vec_[i]->GetNeighbors();
				planes_vec_[i]->GetInitialNormals();
				planes_vec_[i]->DoNormalEstimation();
				planes_vec_[i]->DoPolygonSmoothing();
				planes_vec_[i]->DoPolygonExtraction();
				planes_vec_[i]->CalcMinDists();*/
				continue;
			}
			bool flag = true;
			for (int j = 0; j < vertical_idxs.size(); j++) {
				float4 coeff_j = planes_vec_[vertical_idxs[j]]->GetPlaneCoeff();
				Eigen::Vector3f normal_j(coeff_j.x, coeff_j.y, coeff_j.z);
				Eigen::Matrix3f A;
				A << normal(0), normal(1), normal(2), normal_j(0), normal_j(1), normal_j(2), 0, 0, 1;
				Eigen::Vector3f B;
				B << -coeff.w, -coeff_j.w, horizontal_low_height;
				Eigen::Vector3f X = A.inverse()*B;
				float3 pos;
				pos.x = X.x();
				pos.y = X.y();
				if (0) {
					std::cout << "vertical_idxs[0]: " << vertical_idxs[0] << "\n";
					std::cout << "vertical_idxs[1]: " << vertical_idxs[1] << "\n";
					float dist = planes_vec_[vertical_idxs[0]]->GetDistToPolygon(Eigen::Vector3f(average_pos.x(), average_pos.y(), average_pos.z()));
					std::cout << "dist1: " << dist << "\n";
					dist = planes_vec_[vertical_idxs[1]]->GetDistToPolygon(Eigen::Vector3f(average_pos.x(), average_pos.y(), average_pos.z()));
					std::cout << "dist2: " << dist << "\n";
					flag = false;
					break;
				}
				if (j % 2 == 0)
					pos.z = horizontal_low_height;
				else
					pos.z = horizontal_high_height;
				pos_vec.emplace_back(pos);
				/*std::cout << "normal: " << normal.transpose() << "\n";
				std::cout << "normal_j: " << normal_j.transpose() << "\n";
				std::cout << "pos: " << pos.x<<" " << pos.y << " " << pos.z << " " << "\n";*/
				std::cout << "pos: " << pos.x << " " << pos.y << " " << pos.z << " " << "\n";
				if (j % 2 == 1)
					pos.z = horizontal_low_height;
				else
					pos.z = horizontal_high_height;
				pos_vec.emplace_back(pos);
			}
			if (!flag)
				break;
			if (abs(pos_vec[0].x- pos_vec[2].x)>1) {
				std::cout << "vertical_idxs[0]: " << vertical_idxs[0] << "\n";
				std::cout << "vertical_idxs[1]: " << vertical_idxs[1] << "\n";
				float dist = planes_vec_[vertical_idxs[0]]->GetDistToPolygon(Eigen::Vector3f(average_pos.x(), average_pos.y(), average_pos.z()));
				std::cout << "dist1: " << dist << "\n";
				dist = planes_vec_[vertical_idxs[1]]->GetDistToPolygon(Eigen::Vector3f(average_pos.x(), average_pos.y(), average_pos.z()));
				std::cout << "dist2: " << dist << "\n";
				//system("pause");
				//flag = false;
				//break;
			}
			planes_vec_[i]->SetCornerPts(pos_vec);
			planes_vec_[i]->CalcMinDists();
		}
#endif
		widget = new PolygonWindow();
		widget->resize(200, 100);
		widget->move(10, 10);		
		widget->SetData(in_pointcloud, planes_vec_);
		widget->show();
		std::cout << "widget->show() done" << std::endl;
	}

	HWPlane* HWSNAP::GetPlaneObject(int i)
	{
		if (i >= planes_vec_.size() || i < 0)
			return NULL;
		return planes_vec_[i];
	}

	void HWSNAP::SetProcessType(ProcessType& my_type)
	{
		//
		type_process_ = my_type;
	}

	ProcessType HWSNAP::GetProcessType()
	{
		return type_process_;
	}
}
