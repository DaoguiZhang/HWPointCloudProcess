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

#include "UniformTSDFVolume.h"

#include <unordered_map>
#include <thread>

//#include <Core/Utility/Helper.h>
//#include <Core/Integration/MarchingCubesConst.h>
#include "Helper.h"
#include "MarchingCubesConst.h"

namespace open3d {

	UniformTSDFVolume::UniformTSDFVolume(double voxel_length,
		double sdf_trunc,
		const Eigen::Vector3d &origin/* = Eigen::Vector3d::Zero()*/) :
		TSDFVolume(voxel_length, sdf_trunc),
		origin_(origin), //length_(length), resolution_(resolution),
		//voxel_num_(resolution * resolution * resolution),
		tsdf_( 0.0f), weight_(0.0f)
		//color_(olor_type != TSDFVolumeColorType::None ? voxel_num_ : 0,
			//Eigen::Vector3f::Zero())
	{
	}

	UniformTSDFVolume::~UniformTSDFVolume()
	{
	}

	void UniformTSDFVolume::Reset()
	{
		tsdf_ = 0.0f;
		weight_ = 0.0f;
		/*if (color_type_ != TSDFVolumeColorType::None) {
			std::memset(color_.data(), 0, voxel_num_ * 12);
		}*/
	}

	std::shared_ptr<TriangleMesh> UniformTSDFVolume::ExtractTriangleMesh()
	{
		// implementation of marching cubes, based on
		// http://paulbourke.net/geometry/polygonise/
		auto mesh = std::make_shared<TriangleMesh>();
		double half_voxel_length = voxel_length_ * 0.5;
		std::unordered_map<Eigen::Vector4i, int, hash_eigen::hash<Eigen::Vector4i>>
			edgeindex_to_vertexindex;
		int edge_to_index[12];
					int cube_index = 0;
					float f[8];
					Eigen::Vector3d c[8];
					for (int i = 0; i < 8; i++) {
						Eigen::Vector3i idx =  shift[i];
						if (weight_ == 0.0f) {
							cube_index = 0;
							break;
						}
						else {
							f[i] = tsdf_;
							if (f[i] < 0.0f) {
								cube_index |= (1 << i);
							}
							/*if (color_type_ == TSDFVolumeColorType::RGB8) {
								c[i] = color_.cast<double>() / 255.0;
							}
							else if (color_type_ ==
								TSDFVolumeColorType::Gray32) {
								c[i] = color_.cast<double>();
							}*/
						}
					}
					if (cube_index == 0 || cube_index == 255) {
						//break;
					}
					for (int i = 0; i < 12; i++) {
						if (edge_table[cube_index] & (1 << i)) {
							Eigen::Vector4i edge_index =
								Eigen::Vector4i(0, 0, 0, 0) + edge_shift[i];
							if (edgeindex_to_vertexindex.find(edge_index) ==
								edgeindex_to_vertexindex.end()) {
								edge_to_index[i] = (int)mesh->vertices_.size();
								edgeindex_to_vertexindex[edge_index] =
									(int)mesh->vertices_.size();
								Eigen::Vector3d pt(
									half_voxel_length +
									voxel_length_ * edge_index(0),
									half_voxel_length +
									voxel_length_ * edge_index(1),
									half_voxel_length +
									voxel_length_ * edge_index(2));
								double f0 = std::abs((double)f[edge_to_vert[i][0]]);
								double f1 = std::abs((double)f[edge_to_vert[i][1]]);
								pt(edge_index(3)) += f0 * voxel_length_ / (f0 + f1);
								mesh->vertices_.push_back(pt + origin_);
								/*if (color_type_ != TSDFVolumeColorType::None) {
									const auto &c0 = c[edge_to_vert[i][0]];
									const auto &c1 = c[edge_to_vert[i][1]];
									mesh->vertex_colors_.push_back(
										(f1 * c0 + f0 * c1) / (f0 + f1));
								}*/
							}
							else {
								edge_to_index[i] = edgeindex_to_vertexindex.find(
									edge_index)->second;
							}
						}
					}
					for (int i = 0; tri_table[cube_index][i] != -1; i += 3) {
						mesh->triangles_.push_back(Eigen::Vector3i(
							edge_to_index[tri_table[cube_index][i]],
							edge_to_index[tri_table[cube_index][i + 2]],
							edge_to_index[tri_table[cube_index][i + 1]]));
		}
		return mesh;
	}



	//std::shared_ptr<PointCloud> UniformTSDFVolume::ExtractVoxelPointCloud()
	//{
	//	auto voxel = std::make_shared<PointCloud>();
	//	double half_voxel_length = voxel_length_ * 0.5;
	//	float *p_tsdf = (float *)tsdf_.data();
	//	float *p_weight = (float *)weight_.data();
	//	float *p_color = (float *)color_.data();
	//	for (int x = 0; x < resolution_; x++) {
	//		for (int y = 0; y < resolution_; y++) {
	//			Eigen::Vector3d pt(
	//				half_voxel_length + voxel_length_ * x,
	//				half_voxel_length + voxel_length_ * y,
	//				half_voxel_length);
	//			for (int z = 0; z < resolution_; z++, pt(2) += voxel_length_,
	//				p_tsdf++, p_weight++, p_color += 3) {
	//				if (*p_weight != 0.0f && *p_tsdf < 0.98f &&
	//					*p_tsdf >= -0.98f) {
	//					voxel->points_.push_back(pt + origin_);
	//					double c = (static_cast<double>(*p_tsdf) + 1.0) * 0.5;
	//					voxel->colors_.push_back(Eigen::Vector3d(c, c, c));
	//				}
	//			}
	//		}
	//	}
	//	return voxel;
	//}

//	void UniformTSDFVolume::IntegrateWithDepthToCameraDistanceMultiplier(
//		const RGBDImage &image, const PinholeCameraIntrinsic &intrinsic,
//		const Eigen::Matrix4d &extrinsic,
//		const Image &depth_to_camera_distance_multiplier)
//	{
//		const float fx = static_cast<float>(intrinsic.GetFocalLength().first);
//		const float fy = static_cast<float>(intrinsic.GetFocalLength().second);
//		const float cx = static_cast<float>(intrinsic.GetPrincipalPoint().first);
//		const float cy = static_cast<float>(intrinsic.GetPrincipalPoint().second);
//		const Eigen::Matrix4f extrinsic_f = extrinsic.cast<float>();
//		const float voxel_length_f = static_cast<float>(voxel_length_);
//		const float half_voxel_length_f = voxel_length_f * 0.5f;
//		const float sdf_trunc_f = static_cast<float>(sdf_trunc_);
//		const float sdf_trunc_inv_f = 1.0f / sdf_trunc_f;
//		const Eigen::Matrix4f extrinsic_scaled_f = extrinsic_f *
//			voxel_length_f;
//		const float safe_width_f = intrinsic.width_ - 0.0001f;
//		const float safe_height_f = intrinsic.height_ - 0.0001f;
//
//#ifdef _OPENMP
//#pragma omp parallel for schedule(static)
//#endif
//		for (int x = 0; x < resolution_; x++) {
//			for (int y = 0; y < resolution_; y++) {
//				int idx_shift = x * resolution_ * resolution_ + y * resolution_;
//				float *p_tsdf = (float *)tsdf_.data() + idx_shift;
//				float *p_weight = (float *)weight_.data() + idx_shift;
//				float *p_color = (float *)color_.data() + idx_shift * 3;
//				Eigen::Vector4f voxel_pt_camera = extrinsic_f * Eigen::Vector4f(
//					half_voxel_length_f + voxel_length_f * x +
//					(float)origin_(0),
//					half_voxel_length_f + voxel_length_f * y +
//					(float)origin_(1),
//					half_voxel_length_f + (float)origin_(2),
//					1.0f);
//				for (int z = 0; z < resolution_; z++,
//					voxel_pt_camera(0) += extrinsic_scaled_f(0, 2),
//					voxel_pt_camera(1) += extrinsic_scaled_f(1, 2),
//					voxel_pt_camera(2) += extrinsic_scaled_f(2, 2),
//					p_tsdf++, p_weight++, p_color += 3) {
//					if (voxel_pt_camera(2) > 0) {
//						float u_f = voxel_pt_camera(0) * fx /
//							voxel_pt_camera(2) + cx + 0.5f;
//						float v_f = voxel_pt_camera(1) * fy /
//							voxel_pt_camera(2) + cy + 0.5f;
//						if (u_f >= 0.0001f && u_f < safe_width_f &&
//							v_f >= 0.0001f && v_f < safe_height_f) {
//							int u = (int)u_f;
//							int v = (int)v_f;
//							float d = *PointerAt<float>(image.depth_, u, v);
//							if (d > 0.0f) {
//								float sdf = (d - voxel_pt_camera(2)) * (
//									*PointerAt<float>(
//										depth_to_camera_distance_multiplier,
//										u, v));
//								if (sdf > -sdf_trunc_f) {
//									// integrate
//									float tsdf = std::min(1.0f,
//										sdf * sdf_trunc_inv_f);
//									*p_tsdf = ((*p_tsdf) * (*p_weight) + tsdf) /
//										(*p_weight + 1.0f);
//									if (color_type_ ==
//										TSDFVolumeColorType::RGB8) {
//										const uint8_t *rgb = PointerAt<uint8_t>(
//											image.color_, u, v, 0);
//										p_color[0] = (p_color[0] *
//											(*p_weight) + rgb[0]) /
//											(*p_weight + 1.0f);
//										p_color[1] = (p_color[1] *
//											(*p_weight) + rgb[1]) /
//											(*p_weight + 1.0f);
//										p_color[2] = (p_color[2] *
//											(*p_weight) + rgb[2]) /
//											(*p_weight + 1.0f);
//									}
//									else if (color_type_ ==
//										TSDFVolumeColorType::Gray32) {
//										const float *intensity = PointerAt<float>(
//											image.color_, u, v, 0);
//										// PrintError("intensity : %f\n", *intensity);
//										p_color[0] = (p_color[0] *
//											(*p_weight) + *intensity) /
//											(*p_weight + 1.0f);
//										p_color[1] = (p_color[1] *
//											(*p_weight) + *intensity) /
//											(*p_weight + 1.0f);
//										p_color[2] = (p_color[2] *
//											(*p_weight) + *intensity) /
//											(*p_weight + 1.0f);
//									}
//									*p_weight += 1.0f;
//								}
//							}
//						}
//					}
//				}
//			}
//		}
//	}
//
	void UniformTSDFVolume::IntegrateVertex(Eigen::Vector3f vertex_pos,
		Eigen::Vector3f vertex_normal,
		float point_cloud_weight,
		double sdf_trunc)
	{
		const float voxel_length_f = static_cast<float>(voxel_length_);
		const float half_voxel_length_f = voxel_length_f * 0.5f;
		//const float sdf_trunc_f = static_cast<float>(sdf_trunc_);
		const float sdf_trunc_f = static_cast<float>(sdf_trunc);
		const float sdf_trunc_inv_f = 1.0f / sdf_trunc_f;

		//#ifdef _OPENMP
		//#pragma omp parallel for schedule(static)
		//#endif
		/*int resolutionsqr = resolution_*resolution_;
		int x_start = -resolutionsqr, y_start;*/
		//for (int x = 0; x < resolution_; x++) {
		/*x_start += resolutionsqr;
		y_start = x_start - resolution_;*/
		//for (int y = 0; y < resolution_; y++) {
		Eigen::Vector3f voxel_pt = Eigen::Vector3f(
			half_voxel_length_f + (float)origin_(0),
			half_voxel_length_f + (float)origin_(1),
			half_voxel_length_f + (float)origin_(2));


		Eigen::Vector3f vertex_pos_to_voxel_pt = vertex_pos - voxel_pt;
		float signed_dist1 = vertex_pos_to_voxel_pt.dot(vertex_normal);
		float dist2 = sqrt(vertex_pos_to_voxel_pt.dot(vertex_pos_to_voxel_pt) - signed_dist1 * signed_dist1);

		float sdf = signed_dist1;
		float weight = point_cloud_weight *
			(1.0f / (1.0f + exp(dist2 / (0.1f * voxel_length_f))) - 0.5f) * 2.0f;
		if (dist2 < voxel_length_f * 2 && sdf > -sdf_trunc_f && sdf < sdf_trunc_f) {

			float tsdf = sdf * sdf_trunc_inv_f;
			
			tsdf_ = (tsdf_ * weight_ + tsdf) /
				(weight_ + weight);
			weight_ += weight;
		}
	}

	void UniformTSDFVolume::IntegrateVertex(const std::vector<float3>& vertex_pos_vec,
		const std::vector<float3>& vertex_normal_vec,
		std::vector<int>& voxel_vertex_ordered_idx,
		voxel_info& voxel,
		float point_cloud_weight,
		double sdf_trunc) {
		const float voxel_length_f = static_cast<float>(voxel_length_);
		const float half_voxel_length_f = voxel_length_f * 0.5f;
		const float sdf_trunc_f = static_cast<float>(sdf_trunc);
		const float sdf_trunc_inv_f = 1.0f / sdf_trunc_f;
		float3 voxel_pt;
		voxel_pt.x = half_voxel_length_f + (float)origin_(0);
		voxel_pt.y = half_voxel_length_f + (float)origin_(1);
		voxel_pt.z = half_voxel_length_f + (float)origin_(2);
		for (int i = voxel.idx_start; i < voxel.idx_start + voxel.idx_length; i++) {
			float3 voxel_vertex_pos = vertex_pos_vec[voxel_vertex_ordered_idx[i]];
			float3 voxel_normal_pos = vertex_normal_vec[voxel_vertex_ordered_idx[i]];
			float3 vertex_pos_to_voxel_pt = voxel_vertex_pos - voxel_pt;
			float signed_dist1 = vertex_pos_to_voxel_pt.x*voxel_normal_pos.x
				+ vertex_pos_to_voxel_pt.y*voxel_normal_pos.y
				+ vertex_pos_to_voxel_pt.z*voxel_normal_pos.z;
			float dist2 = sqrt(vertex_pos_to_voxel_pt.x*vertex_pos_to_voxel_pt.x 
				+ vertex_pos_to_voxel_pt.y*vertex_pos_to_voxel_pt.y 
				+ vertex_pos_to_voxel_pt.z*vertex_pos_to_voxel_pt.z - signed_dist1 * signed_dist1);

			float sdf = signed_dist1;
			float weight = point_cloud_weight *
				(1.0f / (1.0f + exp(dist2 / (0.1f * voxel_length_f))) - 0.5f) * 2.0f;
			//printf("%f: %f\n", sdf, weight);
			if (dist2 < voxel_length_f * 2 && sdf > -sdf_trunc_f && sdf < sdf_trunc_f) {

				float tsdf = sdf * sdf_trunc_inv_f;

				tsdf_ = (tsdf_ * weight_ + tsdf) /
					(weight_ + weight);
				weight_ += weight;
			//	printf("%f: %f\n", tsdf_, weight_);
			}
		}
	}

	//Eigen::Vector3d UniformTSDFVolume::GetNormalAt(const Eigen::Vector3d &p)
	//{
	//	Eigen::Vector3d n;
	//	const double half_gap = 0.99 * voxel_length_;
	//	for (int i = 0; i < 3; i++) {
	//		Eigen::Vector3d p0 = p;
	//		p0(i) -= half_gap;
	//		Eigen::Vector3d p1 = p;
	//		p1(i) += half_gap;
	//		n(i) = GetTSDFAt(p1) - GetTSDFAt(p0);
	//	}
	//	return n.normalized();
	//}

	//double UniformTSDFVolume::GetTSDFAt(const Eigen::Vector3d &p)
	//{
	//	Eigen::Vector3i idx;
	//	Eigen::Vector3d p_grid = p / voxel_length_ - Eigen::Vector3d(0.5, 0.5, 0.5);
	//	for (int i = 0; i < 3; i++) {
	//		idx(i) = (int)std::floor(p_grid(i));
	//	}
	//	Eigen::Vector3d r = p_grid - idx.cast<double>();
	//	return (1 - r(0)) * (
	//		(1 - r(1)) * (
	//		(1 - r(2)) * tsdf_[IndexOf(idx + Eigen::Vector3i(0, 0, 0))] +
	//			r(2) * tsdf_[IndexOf(idx + Eigen::Vector3i(0, 0, 1))]
	//			) + r(1) * (
	//			(1 - r(2)) * tsdf_[IndexOf(idx + Eigen::Vector3i(0, 1, 0))] +
	//				r(2) * tsdf_[IndexOf(idx + Eigen::Vector3i(0, 1, 1))]
	//				)) + r(0) * (
	//				(1 - r(1)) * (
	//					(1 - r(2)) * tsdf_[IndexOf(idx + Eigen::Vector3i(1, 0, 0))] +
	//					r(2) * tsdf_[IndexOf(idx + Eigen::Vector3i(1, 0, 1))]
	//					) + r(1) * (
	//					(1 - r(2)) * tsdf_[IndexOf(idx + Eigen::Vector3i(1, 1, 0))] +
	//						r(2) * tsdf_[IndexOf(idx + Eigen::Vector3i(1, 1, 1))]
	//						));
	//}

}    // namespace open3d
