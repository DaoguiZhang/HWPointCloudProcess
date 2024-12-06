#pragma once
#include<vector>
#include<iostream>
#include<unordered_map>
#include<ctime>
#include"math_utils.h"

struct voxel_info {
	int3 voxel_pos;
	int idx_start;
	int idx_length;
};

class VertexRearrangement {
public:
	VertexRearrangement();
	VertexRearrangement(float voxel_length, int volume_resolution);
	~VertexRearrangement();

	/*
	aggregate the vertex pos: the vertex in same voxel should aggregate
	*/
	void AggregateVertexWithinVoxel(const std::vector<float3>& vertex_pos_vec);

	int3 LocateVolumeVoxelPos(const float3& ver_pos);
	void OpenVolumeVoxel(const int3 &index, int ver_idx);

	int GetVolumeResolution();
	float GetVoxelLength();

	std::vector<int>& GetVolumeVertexOrderedIdx();
	std::vector<voxel_info>& GetVoxelInfoVec();

	std::vector<int> volume_vertex_ordered_idx_;
	std::vector<voxel_info> volume_to_vertex_info_;

private:
	std::unordered_map<long, std::vector<int> > volume_voxel_to_vertex_idx_;

	int volume_resolution_;
	float voxel_length_;
};