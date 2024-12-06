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

#pragma once

#include <memory>
#include <unordered_map>
#include "TSDFVolume.h"
#include "UniformTSDFVolume.h"
#include "Helper.h"
#include "hw_mesh.h"

//#include <opencv2/opencv.hpp>
namespace HW
{
	class HWMesh;
}

namespace open3d {

class UniformTSDFVolume;

const int MAX_POINT_NUM_IN_VOXEL = 255;
/// Class that implements a more memory efficient data structure for volumetric
/// integration
/// This implementation is based on the following repository:
/// https://github.com/qianyizh/ElasticReconstruction/tree/master/Integrate
/// The reference is:
/// Q.-Y. Zhou and V. Koltun
/// Dense Scene Reconstruction with Points of Interest
/// In SIGGRAPH 2013
///
/// An observed depth pixel gives two types of information: (a) an approximation
/// of the nearby surface, and (b) empty space from the camera to the surface.
/// They induce two core concepts of volumetric integration: weighted average of
/// a truncated signed distance function (TSDF), and carving. The weighted
/// average of TSDF is great in addressing the Gaussian noise along surface
/// normal and producing a smooth surface output. The carving is great in
/// removing outlier structures like floating noise pixels and bumps along
/// structure edges.

class ScalableTSDFVolume : public TSDFVolume {
public:
    struct VolumeUnit {
    public:
        //VolumeUnit() : volume_(NULL) {}
		VolumeUnit(double voxel_length, double sdf_trunc,
			const Eigen::Vector3d &origin = Eigen::Vector3d::Zero()):
			volume_(voxel_length, sdf_trunc,origin)
		{}
    public:
        UniformTSDFVolume volume_;
        Eigen::Vector3i index_;
    };
public:
	ScalableTSDFVolume(double voxel_length = 0.02f, double sdf_trunc = 0.04f, int volume_unit_resolution = 1);
    ~ScalableTSDFVolume() override;

public:
    void Reset() override;
    
	void IntegratePointCloud(const std::vector<float3>& vertex_pos_vec,
		const std::vector<float3>& vertex_normal_vec,
		std::vector<int>& voxel_vertex_ordered_idx_,
		std::vector<voxel_info>& voxel_to_vertex_info_,
		float point_cloud_weight = 1.0f,
		double sdf_trunc = 0.1);
    //std::shared_ptr<PointCloud> ExtractPointCloud() override;
    std::shared_ptr<TriangleMesh> ExtractTriangleMesh() override;
	
	HW::HWMesh* ExtractHWTriMesh();

   // std::shared_ptr<PointCloud> ExtractVoxelPointCloud();
	
public:
    int volume_unit_resolution_;
   
	int threshold_num_;
    /// Assume the index of the volume unit is (x, y, z), then the unit spans
    /// from (x, y, z) * volume_unit_length_
    /// to (x + 1, y + 1, z + 1) * volume_unit_length_
    std::unordered_map<Eigen::Vector3i, VolumeUnit*,
            hash_eigen::hash<Eigen::Vector3i>> volume_units_;


private:

	Eigen::Vector3i LocateVolumeVoxel(const Eigen::Vector3f &point) {
		return Eigen::Vector3i((int)std::floor(point(0) / voxel_length_),
			(int)std::floor(point(1) / voxel_length_),
			(int)std::floor(point(2) / voxel_length_));
	}

	Eigen::Vector3i LocateVolumeVoxelFloat(const Eigen::Vector3f &point) {
		return Eigen::Vector3i((int)std::floor(point(0) / voxel_length_),
			(int)std::floor(point(1) / voxel_length_),
			(int)std::floor(point(2) / voxel_length_));
	}
};

}    // namespace open3d
