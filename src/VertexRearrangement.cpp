#include"VertexRearrangement.h"

VertexRearrangement::VertexRearrangement()
{}

VertexRearrangement::VertexRearrangement(float voxel_length, int volume_resolution)
{
	volume_resolution_ = volume_resolution;
	voxel_length_ = voxel_length;
}

VertexRearrangement::~VertexRearrangement()
{
	if (!volume_vertex_ordered_idx_.empty())
		volume_vertex_ordered_idx_.clear();
	if (!volume_to_vertex_info_.empty())
		volume_to_vertex_info_.clear();
	if (!volume_voxel_to_vertex_idx_.empty())
		volume_voxel_to_vertex_idx_.clear();
}

int3 VertexRearrangement::LocateVolumeVoxelPos(const float3& ver_pos)
{
	int3 voxel_pos;
	voxel_pos.x = (int)(std::floor(ver_pos.x / voxel_length_));
	voxel_pos.y = (int)(std::floor(ver_pos.y / voxel_length_));
	voxel_pos.z = (int)(std::floor(ver_pos.z / voxel_length_));
	return voxel_pos;
}

void VertexRearrangement::OpenVolumeVoxel(const int3 &index, int ver_idx)
{
	long hash_key = index.x*volume_resolution_*volume_resolution_ + index.y*volume_resolution_ + index.z;
	
	auto& voxel = volume_voxel_to_vertex_idx_[hash_key];
	if (voxel.empty())
	{
		voxel.emplace_back(index.x);
		voxel.emplace_back(index.y);
		voxel.emplace_back(index.z);
	}

	if (voxel.size() == voxel.capacity() && voxel.size() > 50)
	{
		voxel.reserve(voxel.size() + 10);
	}
	voxel.emplace_back(ver_idx);
}

void VertexRearrangement::AggregateVertexWithinVoxel(const std::vector<float3>& vertex_pos_vec)
{
	clock_t start_time, end_time;
	start_time = clock();
	//设置idx大小
	volume_vertex_ordered_idx_.resize(vertex_pos_vec.size());
	for (int i = 0; i < vertex_pos_vec.size(); ++i)
	{
		int3 vertex_voxel_pos = LocateVolumeVoxelPos(vertex_pos_vec[i]);
		
		OpenVolumeVoxel(vertex_voxel_pos, i);
	}
	end_time = clock();
	std::cout << "openVolumeVoxel time: " << (double)(end_time - start_time) / CLOCKS_PER_SEC << "s" << std::endl;

	//排序
	start_time = clock();

	int current_idx = 0;
	long resolution_sqrt = volume_resolution_*volume_resolution_;
	for (std::unordered_map<long, std::vector<int>>::iterator it = volume_voxel_to_vertex_idx_.begin();
		it != volume_voxel_to_vertex_idx_.end(); ++it)
	{
		if (it->second.empty())
			continue;
		voxel_info tmp_voxel_info;
		tmp_voxel_info.idx_start = current_idx;
		tmp_voxel_info.idx_length = it->second.size() - 3;
		//前三个存(x,y,z)
		tmp_voxel_info.voxel_pos.x = it->second[0];
		tmp_voxel_info.voxel_pos.y = it->second[1];
		tmp_voxel_info.voxel_pos.z = it->second[2];
		//后面存resolution
		for (int i = 3; i < it->second.size(); ++i)
		{
			volume_vertex_ordered_idx_[current_idx] = it->second[i];
			current_idx++;
		}
		volume_to_vertex_info_.emplace_back(tmp_voxel_info);
	}
	end_time = clock();
	std::cout << "write to volume time: " << (double)(end_time - start_time) / CLOCKS_PER_SEC << "s" << std::endl;

	std::unordered_map<long, std::vector<int> >::iterator iter;
	for (iter = volume_voxel_to_vertex_idx_.begin(); iter != volume_voxel_to_vertex_idx_.end(); ++iter) {
		std::vector<int>().swap(iter->second);
	}
	//volume_voxel_to_vertex_idx_.clear();
}

std::vector<int>& VertexRearrangement::GetVolumeVertexOrderedIdx()
{
	return volume_vertex_ordered_idx_;
}

std::vector<voxel_info>& VertexRearrangement::GetVoxelInfoVec()
{
	return volume_to_vertex_info_;
}

int VertexRearrangement::GetVolumeResolution()
{
	return volume_resolution_;
}

float VertexRearrangement::GetVoxelLength()
{
	return voxel_length_;
}