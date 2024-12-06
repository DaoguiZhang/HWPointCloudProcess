// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2018 www.open3d.org
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.
// ----------------------------------------------------------------------------

#include "ScalableTSDFVolume.h"

#include <unordered_set>

//#include "Console.h"
//#include "PointCloud.h"
#include "UniformTSDFVolume.h"
#include "MarchingCubesConst.h"

namespace open3d {
	ScalableTSDFVolume::ScalableTSDFVolume(double voxel_length, double sdf_trunc, int volume_unit_resolution):
		//TSDFVolumeColorType color_type, int volume_unit_resolution/* = 16*/,
		//int depth_sampling_stride/* = 4*/) :
		TSDFVolume(voxel_length, sdf_trunc),
		volume_unit_resolution_(volume_unit_resolution)
		//volume_unit_length_(voxel_length * volume_unit_resolution),
		//depth_sampling_stride_(depth_sampling_stride)
	{
	}

	ScalableTSDFVolume::~ScalableTSDFVolume()
	{
	}

	void ScalableTSDFVolume::Reset()
	{
		volume_units_.clear();
	}

	//void ScalableTSDFVolume::Integrate(const RGBDImage &image,
	//	const PinholeCameraIntrinsic &intrinsic,
	//	const Eigen::Matrix4d &extrinsic)
	//{
	//	if ((image.depth_.num_of_channels_ != 1) ||
	//		(image.depth_.bytes_per_channel_ != 4) ||
	//		(image.depth_.width_ != intrinsic.width_) ||
	//		(image.depth_.height_ != intrinsic.height_) ||
	//		(color_type_ == TSDFVolumeColorType::RGB8 &&
	//			image.color_.num_of_channels_ != 3) ||
	//			(color_type_ == TSDFVolumeColorType::RGB8 &&
	//				image.color_.bytes_per_channel_ != 1) ||
	//				(color_type_ == TSDFVolumeColorType::Gray32 &&
	//					image.color_.num_of_channels_ != 1) ||
	//					(color_type_ == TSDFVolumeColorType::Gray32 &&
	//						image.color_.bytes_per_channel_ != 4) ||
	//						(color_type_ != TSDFVolumeColorType::None &&
	//							image.color_.width_ != intrinsic.width_) ||
	//							(color_type_ != TSDFVolumeColorType::None &&
	//								image.color_.height_ != intrinsic.height_)) {
	//		//PrintWarning("[ScalableTSDFVolume::Integrate] Unsupported image format.\n");
	//		//return;
	//	}
	//	auto depth2cameradistance = CreateDepthToCameraDistanceMultiplierFloatImage(
	//		intrinsic);
	//	auto pointcloud = CreatePointCloudFromDepthImage(image.depth_, intrinsic,
	//		extrinsic, 1000.0, 1000.0, depth_sampling_stride_);
	//	std::unordered_set<Eigen::Vector3i, hash_eigen::hash<Eigen::Vector3i>>
	//		touched_volume_units_;
	//	for (const auto &point : pointcloud->points_) {
	//		auto min_bound = LocateVolumeUnit(point - Eigen::Vector3d(
	//			sdf_trunc_, sdf_trunc_, sdf_trunc_));
	//		auto max_bound = LocateVolumeUnit(point + Eigen::Vector3d(
	//			sdf_trunc_, sdf_trunc_, sdf_trunc_));
	//		for (auto x = min_bound(0); x <= max_bound(0); x++) {
	//			for (auto y = min_bound(1); y <= max_bound(1); y++) {
	//				for (auto z = min_bound(2); z <= max_bound(2); z++) {
	//					auto loc = Eigen::Vector3i(x, y, z);
	//					if (touched_volume_units_.find(loc) ==
	//						touched_volume_units_.end()) {
	//						touched_volume_units_.insert(loc);
	//						auto volume = OpenVolumeUnit(Eigen::Vector3i(x, y, z));
	//						volume->IntegrateWithDepthToCameraDistanceMultiplier(
	//							image, intrinsic, extrinsic,
	//							*depth2cameradistance);
	//					}
	//				}
	//			}
	//		}
	//	}
	//}
//
//	void ScalableTSDFVolume::IntegratePointCloud(std::vector<Eigen::Vector3f> vertex_pos_vec,
//		std::vector<Eigen::Vector3f> vertex_normal_vec,
//		float point_cloud_weight,
//		double sdf_trunc)
//	{
//#if 0
//		vertex_normal_vec[0] = Eigen::Vector3f(1, 0, 0);
//		vertex_pos_vec[0] = Eigen::Vector3f(0, 0, 0);
//		char path[256];
//		std::ofstream fs;
//		sprintf(path, "F:/ElasticFusion-master/room1024_20190314/test.ply", 0);
//		fs.open(path);
//		fs << "ply";
//		fs << "\nformat " << "ascii" << " 1.0";
//		fs << "\nelement vertex " << 100;
//		fs << "\nproperty float x"
//			"\nproperty float y"
//			"\nproperty float z";
//		fs << "\nproperty uchar red"
//			"\nproperty uchar green"
//			"\nproperty uchar blue";
//		fs << "\nend_header\n";
//
//		for (int i = 0; i < 100; ++i) {
//			Eigen::Vector3f temp = vertex_pos_vec[0] + 0.001 * i * vertex_normal_vec[0];
//			fs << temp.x() << " " << temp.y() << " " << temp.z() << " "
//				<< (int)0 << " " << (int)240 << " " << (int)0 << std::endl;
//		}
//		fs.close();
//#endif
//		for (int vertex_idx = 0; vertex_idx < vertex_pos_vec.size(); ++vertex_idx) {
//			//for (int vertex_idx = 0; vertex_idx < 1; ++vertex_idx) {
//			if (vertex_idx % 100000 == 99999)
//				printf("%d: %d\n", vertex_pos_vec.size(), vertex_idx);
//#if 0
//			printf("%f %f %f\n", vertex_normal_vec[vertex_idx].x(),
//				vertex_normal_vec[vertex_idx].y(),
//				vertex_normal_vec[vertex_idx].z());
//#endif
//#if 0
//			auto min_bound = LocateVolumeUnitFloat(vertex_pos_vec[vertex_idx] - Eigen::Vector3f(
//				sdf_trunc_ * 1, sdf_trunc_ * 1, sdf_trunc_));
//			auto max_bound = LocateVolumeUnitFloat(vertex_pos_vec[vertex_idx] + Eigen::Vector3f(
//				sdf_trunc_ * 1, sdf_trunc_ * 1, sdf_trunc_));
//#endif
//#if 1
//			auto min_bound = LocateVolumeUnitFloat(vertex_pos_vec[vertex_idx] - 2 * Eigen::Vector3f(
//				volume_unit_length_, volume_unit_length_, volume_unit_length_));
//			auto max_bound = LocateVolumeUnitFloat(vertex_pos_vec[vertex_idx] + 2 * Eigen::Vector3f(
//				volume_unit_length_, volume_unit_length_, volume_unit_length_));
//#endif
//			for (auto x = min_bound(0); x <= max_bound(0); x++) {
//				for (auto y = min_bound(1); y <= max_bound(1); y++) {
//					for (auto z = min_bound(2); z <= max_bound(2); z++) {
//						auto volume = OpenVolumeUnit(Eigen::Vector3i(x, y, z));
//						volume->IntegrateVertex(vertex_pos_vec[vertex_idx],
//							vertex_normal_vec[vertex_idx],
//							point_cloud_weight,
//							sdf_trunc);
//					}
//				}
//			}
//		}
//	}

	void ScalableTSDFVolume::IntegratePointCloud(const std::vector<float3>& vertex_pos_vec,
		const std::vector<float3>& vertex_normal_vec,
		std::vector<int>& voxel_vertex_ordered_idx,
		std::vector<voxel_info>& voxel_info_vec,
		float point_cloud_weight,
		double sdf_trunc)
    {
        // check input
        if (vertex_pos_vec.empty()) {
            printf("error: vertex_pos_vec is empty!\n");
            return;
        }
        if (vertex_normal_vec.empty()) {
            printf("error: vertex_normal_vec is empty!\n");
            return;
        }
        if (vertex_pos_vec.empty()) {
            printf("error: voxel_vertex_ordered_idx is empty!\n");
            return;
        }
        if (vertex_pos_vec.empty()) {
            printf("error: voxel_info_vec is empty!\n");
            return;
        }

		int count = 0 ,cnt = 1;
		for (int voxel_idx = 0; voxel_idx < voxel_info_vec.size(); voxel_idx++) {
			voxel_info &voxel = voxel_info_vec[voxel_idx];
			count += voxel.idx_length;
			if (count > cnt * 1000000) {
				cnt++;
				printf("%d / %d\n", count, vertex_pos_vec.size());
			}
			int3 voxel_pos= voxel.voxel_pos;
			//if (voxel.idx_length != 1) {
			//	//printf("vertex:%f %f %f\n", voxel_vertex_pos_vec[0].x, voxel_vertex_pos_vec[0].y, voxel_vertex_pos_vec[0].z);
			//	//printf("voxel:%d %d %d\n", voxel_pos.x, voxel_pos.y, voxel_pos.z);
			//	printf("%d\n", voxel.idx_length);
			//	getchar();
			//}
			//#ifdef _OPENMP
			//int x_vec[27] = { -1,0,1,-1,0,1,-1,0,1,-1,0,1 ,-1,0,1 ,-1,0,1 ,-1,0,1 ,-1,0,1 ,-1,0,1 };
			//int y_vec[27] = { -1,-1,-1,0,0,0,1,1,1,-1,-1,-1,0,0,0,1,1,1, -1,-1,-1,0,0,0,1,1,1 };
			//int z_vec[27] = { -1,-1,-1,-1,-1,-1,-1,-1,-1,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1 };
			  #pragma omp parallel for schedule(static)
			for (int i = 0; i < 27; i++) {
				int x = i % 3 - 1;
				int y = i / 3 % 3 - 1;
				int z = i / 9 - 1;
				UniformTSDFVolume &volume = volume_units_[Eigen::Vector3i(x + voxel_pos.x, y + voxel_pos.y, z + voxel_pos.z)]->volume_;
							
				volume.IntegrateVertex(vertex_pos_vec,
							vertex_normal_vec,
							voxel_vertex_ordered_idx,
							voxel,
							point_cloud_weight,
							sdf_trunc);
			}
		}
	}
	
	std::shared_ptr<TriangleMesh> ScalableTSDFVolume::ExtractTriangleMesh()
	{
		// implementation of marching cubes, based on
		// http://paulbourke.net/geometry/polygonise/
		auto mesh = std::make_shared<TriangleMesh>();
		double half_voxel_length = voxel_length_ * 0.5;
		std::unordered_map<Eigen::Vector4i, int, hash_eigen::hash<Eigen::Vector4i>>
			edgeindex_to_vertexindex;
		int edge_to_index[12];
		for (const auto &unit : volume_units_) {
			if (1) {
				const auto &volume0 = unit.second->volume_;
				const auto &index0 = unit.second->index_;
				/*for (int x = 0; x < 1; x++) {
					for (int y = 0; y < 1; y++) {
						for (int z = 0; z < 1; z++) {*/
							//Eigen::Vector3i idx0(0, 0, 0);
							int cube_index = 0;
							float w[8];
							float f[8];
							Eigen::Vector3d c[8];
							for (int i = 0; i < 8; i++) {
								//Eigen::Vector3i index1 = index0;
								Eigen::Vector3i idx1 = index0 + shift[i];
								//if (idx1(0) < volume_unit_resolution_ &&
								//	idx1(1) < volume_unit_resolution_ &&
								//	idx1(2) < volume_unit_resolution_) {
								//	w[i] = volume0.weight_;
								//	f[i] = volume0.tsdf_;
								//	/*if (color_type_ == TSDFVolumeColorType::RGB8)
								//		c[i] = volume0.color_.cast<double>() / 255.0;
								//	else if (color_type_ ==
								//		TSDFVolumeColorType::Gray32);
								//		c[i] = volume0.color_.cast<double>();*/
								//}
								//else {
								//	for (int j = 0; j < 3; j++) {
								//		if (idx1(j) >= volume_unit_resolution_) {
								//			idx1(j) -= volume_unit_resolution_;
								//			index1(j) += 1;
								//		}
								//	}
									auto unit_itr1 = volume_units_.find(idx1);
									if (unit_itr1 == volume_units_.end()) {
										w[i] = 0.0f;  f[i] = 0.0f;
									}
									else {
										const auto &volume1 =
											unit_itr1->second->volume_;
										w[i] = volume1.weight_;
										f[i] = volume1.tsdf_;
										/*if (color_type_ ==
											TSDFVolumeColorType::RGB8)
											c[i] = volume1.color_.cast<double>() / 255.0;
										else if (color_type_ ==
											TSDFVolumeColorType::Gray32)
											c[i] = volume1.color_.cast<double>();*/
									}
								//}
								if (w[i] == 0.0f) {
									cube_index = 0;
									break;
								}
								else {
									if (f[i] < 0.0f) {
										cube_index |= (1 << i);
									}
								}
							}

							if (cube_index == 0 || cube_index == 255) {
								continue;
							}

							for (int i = 0; i < 12; i++) {
								if (edge_table[cube_index] & (1 << i)) {
									Eigen::Vector4i edge_index = Eigen::Vector4i(
										index0(0), index0(1), index0(2), 0) *
										volume_unit_resolution_ +
										Eigen::Vector4i(0, 0, 0, 0) +
										edge_shift[i];
									if (edgeindex_to_vertexindex.find(edge_index) ==
										edgeindex_to_vertexindex.end()) {
										edge_to_index[i] =
											(int)mesh->vertices_.size();
										edgeindex_to_vertexindex[edge_index] =
											(int)mesh->vertices_.size();
										Eigen::Vector3d pt(
											half_voxel_length +
											voxel_length_ * edge_index(0),
											half_voxel_length +
											voxel_length_ * edge_index(1),
											half_voxel_length +
											voxel_length_ * edge_index(2));
										double f0 = std::abs((double)f[
											edge_to_vert[i][0]]);
										double f1 = std::abs((double)f[
											edge_to_vert[i][1]]);
										pt(edge_index(3)) += f0 * voxel_length_ /
											(f0 + f1);
										mesh->vertices_.push_back(pt);
										/*if (color_type_ !=
											TSDFVolumeColorType::None) {
											const auto &c0 = c[edge_to_vert[i][0]];
											const auto &c1 = c[edge_to_vert[i][1]];
											mesh->vertex_colors_.push_back(
												(f1 * c0 + f0 * c1) / (f0 + f1));
										}*/
									}
									else {
										edge_to_index[i] =
											edgeindex_to_vertexindex[
												edge_index];
									}
								}
							}
							for (int i = 0; tri_table[cube_index][i] != -1; i += 3)
							{
								mesh->triangles_.push_back(Eigen::Vector3i(
									edge_to_index[tri_table[cube_index][i]],
									edge_to_index[tri_table[cube_index][i + 2]],
									edge_to_index[tri_table[cube_index][i + 1]]));
							}
						/*}
					}
				}*/
			}
		}
		return mesh;
	}

	HW::HWMesh* ScalableTSDFVolume::ExtractHWTriMesh()
	{
		// implementation of marching cubes, based on
		// http://paulbourke.net/geometry/polygonise/
		//auto mesh = std::make_shared<HW::HWMesh>();
		HW::HWMesh* mesh = new HW::HWMesh();
		double half_voxel_length = voxel_length_ * 0.5;
		std::unordered_map<Eigen::Vector4i, int, hash_eigen::hash<Eigen::Vector4i>>
			edgeindex_to_vertexindex;
		int edge_to_index[12];
		for (const auto &unit : volume_units_)
		{
			const auto &volume0 = unit.second->volume_;
			const auto &index0 = unit.second->index_;
			int cube_index = 0;
			float w[8];
			float f[8];
			Eigen::Vector3d c[8];

			for (int i = 0; i < 8; i++) {
				Eigen::Vector3i idx1 = index0 + shift[i];
				auto unit_itr1 = volume_units_.find(idx1);

				if (unit_itr1 == volume_units_.end()) {
					w[i] = 0.0f;  f[i] = 0.0f;
				}
				else 
				{
					const auto &volume1 =
						unit_itr1->second->volume_;
					w[i] = volume1.weight_;
					f[i] = volume1.tsdf_;
				}

				if (w[i] == 0.0f) {
					cube_index = 0;
					break;
				}
				else {
					if (f[i] < 0.0f) {
						cube_index |= (1 << i);
					}
				}
			}

			if (cube_index == 0 || cube_index == 255) {
				continue;
			}

			for (int i = 0; i < 12; i++) {
				if (edge_table[cube_index] & (1 << i)) {
					Eigen::Vector4i edge_index = Eigen::Vector4i(
						index0(0), index0(1), index0(2), 0) *
						volume_unit_resolution_ +
						Eigen::Vector4i(0, 0, 0, 0) +
						edge_shift[i];
					if (edgeindex_to_vertexindex.find(edge_index) ==
						edgeindex_to_vertexindex.end()) {
						edge_to_index[i] =
							(int)mesh->GetVertices().size();
						edgeindex_to_vertexindex[edge_index] =
							(int)mesh->GetVertices().size();

						float3 pt;
						pt.x = half_voxel_length +
							voxel_length_ * edge_index(0);
						pt.y = half_voxel_length +
							voxel_length_ * edge_index(1);
						pt.z = half_voxel_length +
							voxel_length_ * edge_index(2);

						/*Eigen::Vector3d pt(
							half_voxel_length +
							voxel_length_ * edge_index(0),
							half_voxel_length +
							voxel_length_ * edge_index(1),
							half_voxel_length +
							voxel_length_ * edge_index(2));*/

						double f0 = std::abs((double)f[
							edge_to_vert[i][0]]);
						double f1 = std::abs((double)f[
							edge_to_vert[i][1]]);

						if(edge_index(3) == 0)
							pt.x += f0 * voxel_length_ / (f0 + f1);
						else if(edge_index(3) == 1)
							pt.y += f0 * voxel_length_ / (f0 + f1);
						else if (edge_index(3) == 2)
							pt.z += f0 * voxel_length_ / (f0 + f1);

						/*pt(edge_index(3)) += f0 * voxel_length_ /
							(f0 + f1);*/

						mesh->AddPoint(pt);
						//mesh->GetVertices().push_back(pt);
					}
					else {
						edge_to_index[i] =
							edgeindex_to_vertexindex[
								edge_index];
					}
				}
			}

			for (int i = 0; tri_table[cube_index][i] != -1; i += 3)
			{
				int3 pt_idx;
				pt_idx.x = edge_to_index[tri_table[cube_index][i]];
				pt_idx.y = edge_to_index[tri_table[cube_index][i + 2]];
				pt_idx.z = edge_to_index[tri_table[cube_index][i + 1]];
				mesh->AddFaceIdx(pt_idx);
				/*mesh->AddFaceIdxInt(
					edge_to_index[tri_table[cube_index][i]],
					edge_to_index[tri_table[cube_index][i + 2]],
					edge_to_index[tri_table[cube_index][i + 1]]
				);*/
			}

		}
		return mesh;
	}
}    // namespace open3d
