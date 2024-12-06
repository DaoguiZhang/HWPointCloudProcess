#include "hw_construction.h"
#include <fstream>
#include <opencv2\core.hpp>
#include <opencv2\imgcodecs.hpp>
namespace HW
{
	HWConstruction::HWConstruction()
	{}
	HWConstruction::~HWConstruction()
	{}

	void HWConstruction::Process(HWObject* in_element, HWObject* out_element)
	{

		//设置tsdf参数?
		HWPointCloud* in_pointcloud = dynamic_cast<HWPointCloud*> (in_element);
		open3d::ScalableTSDFVolume *volume = new open3d::ScalableTSDFVolume();

		volume->voxel_length_ = HWParams::getInstance().construction_params_.voxel_length_;
		volume->sdf_trunc_ = HWParams::getInstance().construction_params_.volume_trunc_length_;
		//这个参数需要重新考虑
		int volume_resolution = HWParams::getInstance().construction_params_.volume_dimensions_.x;

		//test
		printf("volume voxel length: %f\n", volume->voxel_length_);
		printf("volume sdf trunc: %f\n", volume->sdf_trunc_);
		printf("volume resolution: %d\n", volume_resolution);
		//end test

		VertexRearrangement  sort(volume->voxel_length_, volume_resolution);
		sort.AggregateVertexWithinVoxel(in_pointcloud->GetVertices());
		std::vector<int> volume_vertex_ordered_idx = sort.GetVolumeVertexOrderedIdx();
		std::vector<voxel_info> voxel_info_vec = sort.GetVoxelInfoVec();
		volume->volume_units_.rehash(10000000);
		std::vector<open3d::ScalableTSDFVolume::VolumeUnit *> volume_unit_vec(voxel_info_vec.size());

		//time
		clock_t start_time, end_time;
		start_time = clock();

		for (int voxel_idx = 0; voxel_idx < voxel_info_vec.size(); voxel_idx++) {
			voxel_info &voxel = voxel_info_vec[voxel_idx];
			int3 &voxel_pos = voxel.voxel_pos;
			Eigen::Vector3i index(voxel_pos.x, voxel_pos.y, voxel_pos.z);

			volume_unit_vec[voxel_idx] = new open3d::ScalableTSDFVolume::VolumeUnit(volume->voxel_length_,
				volume->sdf_trunc_,
				index.cast<double>() * volume->voxel_length_);
			volume_unit_vec[voxel_idx]->index_ = index;
		}
        printf("Construction step 1.\n");

		for (int voxel_idx = 0; voxel_idx < voxel_info_vec.size(); voxel_idx++) {
			voxel_info &voxel = voxel_info_vec[voxel_idx];
			int3 &voxel_pos = voxel.voxel_pos;
			open3d::ScalableTSDFVolume::VolumeUnit *&unit = volume->volume_units_[Eigen::Vector3i(voxel_pos.x, voxel_pos.y, voxel_pos.z)];
			unit = volume_unit_vec[voxel_idx];
		}
        printf("Construction step 2.\n");

		std::unordered_map<Eigen::Vector3i, open3d::ScalableTSDFVolume::VolumeUnit*,
			open3d::hash_eigen::hash<Eigen::Vector3i>>::iterator iter;
		for (int voxel_idx = 0; voxel_idx < voxel_info_vec.size(); voxel_idx++) {
			voxel_info &voxel = voxel_info_vec[voxel_idx];
			int3 &voxel_pos = voxel.voxel_pos;

			for (auto x = voxel_pos.x - 1; x <= voxel_pos.x + 1; x++) {
				for (auto y = voxel_pos.y - 1; y <= voxel_pos.y + 1; y++) {
					for (auto z = voxel_pos.z - 1; z <= voxel_pos.z + 1; z++) {
						Eigen::Vector3i index(x, y, z);
						iter = volume->volume_units_.find(index);
						if (iter == volume->volume_units_.end()) {
							open3d::ScalableTSDFVolume::VolumeUnit* unit = new open3d::ScalableTSDFVolume::VolumeUnit(volume->voxel_length_,
								volume->sdf_trunc_,
								index.cast<double>() * volume->voxel_length_);
							unit->index_ = index;
							volume->volume_units_.insert(std::make_pair(index, unit));
						}
					}
				}
			}
		}
        printf("Construction step 3.\n");

		volume->IntegratePointCloud(in_pointcloud->GetVertices(),
			in_pointcloud->GetNormal(),
			volume_vertex_ordered_idx,
			voxel_info_vec,
			1.0f,
			volume->sdf_trunc_);

		end_time = clock();
		std::cout << "integrate time: " << (double)(end_time - start_time) / CLOCKS_PER_SEC << "s" << std::endl;

		//march cube
		start_time = clock();

		HWMesh *output_mesh = volume->ExtractHWTriMesh();

		end_time = clock();
		std::cout << "march cube time: " << (double)(end_time - start_time) / CLOCKS_PER_SEC << "s" << std::endl;
		//printf("Texture mapping.........\n");
		//std::vector<float3> vertices = output_mesh->GetVertices();
		//std::vector<float> vertices_camera_distance;
		//for (int i = 0; i < vertices.size(); i++) {
		//	uchar3 color;
		//	color.x = 0;
		//	color.y = 0;
		//	color.y = 0;
		//	output_mesh->AddColor(color);
		//}
		//for (int i = 0; i < vertices.size(); i++)
		//	vertices_camera_distance.emplace_back(FLT_MAX);
		//std::string pano_path = "D:\\vc_project_xht\\roomScence\\pano\\";
		//std::string pano_info = pano_path + "pano-poses.csv";
		//std::ifstream ifs(pano_info);
		//std::string line;
		//std::getline(ifs, line);
		//while (std::getline(ifs, line)) {
		//	int pano_idx;
		//	char pano_name[100];
		//	double timestamp;
		//	double w, x, y, z;
		//	Eigen::Vector3d camera_pos;
		//	Eigen::Quaterniond quaternion;
		//	sscanf(line.c_str(), "%d; %[^;]; %lf; %lf; %lf; %lf; %lf; %lf; %lf; %lf", &pano_idx, pano_name, &timestamp, &camera_pos(0),
		//		&camera_pos(1), &camera_pos(2), &quaternion.w(), &quaternion.x(), &quaternion.y(), &quaternion.z());
		//	std::string pano_image_path = pano_path + pano_name;
		//	cv::Mat pano_image = cv::imread(pano_image_path, cv::IMREAD_COLOR);
		//	int width = pano_image.cols, height = pano_image.rows;
		//	Eigen::Matrix3d rotation_matrix;
		//	rotation_matrix = quaternion.matrix();
		//	for (int i = 0; i < vertices.size(); i++) {
		//		Eigen::Vector3d vert;
		//		vert(0) = vertices[i].x;
		//		vert(1) = vertices[i].y;
		//		vert(2) = vertices[i].z;
		//		Eigen::Vector3d d = vert - camera_pos;//顶点到相机位置
		//		double d_norm = d.norm();
		//		/*Eigen::Vector3i vert_index;
		//		vert_index(0) = (int)(vertices[i](0) / voxel_length);
		//		vert_index(1) = (int)(vertices[i](1) / voxel_length);
		//		vert_index(2) = (int)(vertices[i](2) / voxel_length);*/
		//		Eigen::Vector3d d_xy = d - d.dot(rotation_matrix.col(2)) / rotation_matrix.col(2).dot(rotation_matrix.col(2))*rotation_matrix.col(2);
		//		double cosa = rotation_matrix.col(1).dot(d_xy) / rotation_matrix.col(1).norm() / d_xy.norm();
		//		double a = std::acos(cosa) * 180 / M_PI;
		//		double cosx = rotation_matrix.col(0).dot(d_xy);
		//		if (cosx < 0) {
		//			a = -a;
		//			if (a < -90)
		//				a = 360 + a;
		//		}
		//		double cosb = rotation_matrix.col(2).dot(d) / rotation_matrix.col(2).norm() / d_norm;
		//		double b = std::acos(cosb) * 180 / M_PI;
		//		b -= 90;
		//		int u = a / 360.0f * width + width / 4;
		//		int v = b / 90.0f * (height / 2) + height / 2;
		//		int flag = 1;
		//		Eigen::Vector3d d_normalize = d / d_norm;
		//		Eigen::Vector3d step = d_normalize  * volume->voxel_length_;
		//		int step_num = abs(d_norm) / volume->voxel_length_;
		//		for (int si = 5; si < step_num; si++) {
		//			Eigen::Vector3d pos = vert - si*step;
		//			Eigen::Vector3i index;
		//			index(0) = (int)(pos(0) / volume->voxel_length_);
		//			index(1) = (int)(pos(1) / volume->voxel_length_);
		//			index(2) = (int)(pos(2) / volume->voxel_length_);
		//			std::unordered_map<Eigen::Vector3i, open3d::ScalableTSDFVolume::VolumeUnit*,
		//				open3d::hash_eigen::hash<Eigen::Vector3i>>::iterator iter = volume->volume_units_.find(index);
		//			if (iter != volume->volume_units_.end()) {
		//				if (volume->volume_units_[index]->volume_.tsdf_ != 0) {
		//					flag = 0;
		//					break;
		//				}
		//			}
		//		}
		//		if (flag&&d_norm<vertices_camera_distance[i]) {
		//			uchar3 color;
		//			color.z = pano_image.at<cv::Vec3b>(v, u)[0];
		//			color.y = pano_image.at<cv::Vec3b>(v, u)[1];
		//			color.x = pano_image.at<cv::Vec3b>(v, u)[2];
		//			output_mesh->ChangeColor(color, i);
		//			vertices_camera_distance[i] = d_norm;
		//		}
		//	}
		//}

		//计算法向量
		output_mesh->ComputeVersNormals(true);
		//output_mesh->SetVolume(volume);

		resulted_element_ = output_mesh;
		resulted_element_->SetObjectType(kHWMesh);
		out_element = resulted_element_;
	}
}