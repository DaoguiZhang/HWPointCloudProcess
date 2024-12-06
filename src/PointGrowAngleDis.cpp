#include "PointGrowAngleDis.h"
#include <fstream>
#include <stdio.h>
#include <omp.h>
#include "hw_cmns.h"
#include"hw_algorithms.h"

using namespace std;

PointGrowAngleDis::PointGrowAngleDis(float theta, int Rmin)
{
	this->theta = theta;
	this->Rmin = Rmin;
	this->all_pnts_plane_.plane_normal_ = Eigen::Vector3f(0.0, 0.0, 0.0);
	this->pnt_to_plane_dist_threhold_ = 0.0;
	single_polygon_pnts_flag_ = false;
}

PointGrowAngleDis::~PointGrowAngleDis()
{
}

void PointGrowAngleDis::setData(PtCloud<float> &data, std::vector<PCAInfo> &infos)
{
	this->pointData = data;
	this->pointNum = data.pts.size();
	this->pcaInfos = infos;
}

void PointGrowAngleDis::SetInitialPnt2PlaneThreshold(const float& intial_threshold)
{
	pnt_to_plane_dist_threhold_ = intial_threshold;
}

void PointGrowAngleDis::SetSinglePolygonFlag(bool flag)
{
	single_polygon_pnts_flag_ = flag;
}

void PointGrowAngleDis::SetGrowPointsSeeds(const std::vector<Eigen::Vector3f>& pnts_seeds)
{
	points_grow_seed_ = pnts_seeds;
}

void PointGrowAngleDis::SetGrowPontsSeedsIdxs(const std::vector<int>& seed_idxs)
{
	points_seed_idxs_ = seed_idxs;
}

void PointGrowAngleDis::SetAllPntsPlaneCeoff(const Eigen::Vector3f& center_pnt, const Eigen::Vector3f& plane_normal)
{
	all_pnts_plane_.center_pnt_ = center_pnt;
	all_pnts_plane_.plane_normal_ = plane_normal;
}

void PointGrowAngleDis::run(std::vector<std::vector<int> > &clusters, std::vector<int> &cluster_removed)
{
	float b = 1.4826;

	// sort the data points according to their curvature
	std::vector<std::pair<int, float> > idxSorted(this->pointNum);
	for (int i = 0; i<this->pointNum; ++i)
	{
		idxSorted[i].first = i;
		idxSorted[i].second = pcaInfos[i].lambda0;
	}
	std::cout << "start sorting" << std::endl;
	std::sort(idxSorted.begin(), idxSorted.end(), [](const std::pair<int, float>& lhs, const std::pair<int, float>& rhs) { return lhs.second < rhs.second; });
	std::cout << "end sorting" << std::endl;

	// begin region growing
	std::vector<int> used(this->pointNum, 0);
	for (int i = 0; i<this->pointNum; ++i)
	{
		if (used[i])
		{
			continue;
		}

		if (i % 10000 == 0)
		{
			cout << i << endl;
		}

		//
		std::vector<int> clusterNew;
		clusterNew.push_back(idxSorted[i].first);
		cv::Matx31d normalStart = pcaInfos[idxSorted[i].first].normal;

		int count = 0;
		while (count < clusterNew.size())
		{
			int idxSeed = clusterNew[count];
			int num = pcaInfos[idxSeed].idxIn.size();
			cv::Matx31d normalSeed = pcaInfos[idxSeed].normal;

			// EDth
			std::vector<float> EDs(num);
			for (int j = 0; j<num; ++j)
			{
				int idx = pcaInfos[idxSeed].idxIn[j];
				float dx = this->pointData.pts[idxSeed].x - this->pointData.pts[idx].x;
				float dy = this->pointData.pts[idxSeed].y - this->pointData.pts[idx].y;
				float dz = this->pointData.pts[idxSeed].z - this->pointData.pts[idx].z;

				EDs[j] = sqrt(dx * dx + dy * dy + dz * dz);
			}
			std::sort(EDs.begin(), EDs.end(), [](const float& lhs, const float& rhs) { return lhs < rhs; });
			float EDth = EDs[EDs.size() / 2];

			// ODth
			cv::Matx31d h_mean(0, 0, 0);
			for (int j = 0; j < num; ++j)
			{
				int idx = pcaInfos[idxSeed].idxIn[j];
				h_mean += cv::Matx31d(this->pointData.pts[idx].x, this->pointData.pts[idx].y, this->pointData.pts[idx].z);
			}
			h_mean *= (1.0 / num);
			std::vector<float> ODs(num);
			for (int j = 0; j < num; ++j)
			{
				int idx = pcaInfos[idxSeed].idxIn[j];
				cv::Matx31d pt(this->pointData.pts[idx].x, this->pointData.pts[idx].y, this->pointData.pts[idx].z);
				cv::Matx<float, 1, 1> OD_mat = (pt - h_mean).t() * pcaInfos[idxSeed].normal;
				float OD = fabs(OD_mat.val[0]);
				ODs[j] = OD;
			}

			// calculate the Rz-score for all points using ODs
			std::vector<float> sorted_ODs(ODs.begin(), ODs.end());
			float median_OD = meadian(sorted_ODs);
			std::vector<float>().swap(sorted_ODs);

			std::vector<float> abs_diff_ODs(num);
			for (int j = 0; j < num; ++j)
			{
				abs_diff_ODs[j] = fabs(ODs[j] - median_OD);
			}
			float MAD_OD = b * meadian(abs_diff_ODs);
			float ODth = median_OD + 2.0 * MAD_OD;

			// point cloud collection
			for (int j = 0; j < num; ++j)
			{
				int idx = pcaInfos[idxSeed].idxIn[j];
				if (used[idx])
				{
					continue;
				}

				if (ODs[j] < ODth && EDs[j] < EDth)
				{
					cv::Matx31d normalCur = pcaInfos[idx].normal;
					float angle = acos(normalCur.val[0] * normalStart.val[0]
						+ normalCur.val[1] * normalStart.val[1]
						+ normalCur.val[2] * normalStart.val[2]);
					if (angle != angle)
					{
						continue;
					}

					if (min(angle, float(CV_PI - angle)) < this->theta)
					{
						clusterNew.push_back(idx);
						used[idx] = 1;
					}
				}
			}

			count++;
		}

		if (clusterNew.size() > this->Rmin)
		{
			clusters.push_back(clusterNew);
		}
		else {
			cluster_removed.insert(cluster_removed.end(), clusterNew.begin(), clusterNew.end());
		}
	}
	cout << " number of clusters : " << clusters.size() << endl;
	std::sort(clusters.begin(), clusters.end(), [](const vector<int> &a, const vector<int> &b) { return a.size() > b.size(); });
	getPlaneInfo(clusters);
}

void PointGrowAngleDis::runGrowSegmentsFromSeeds(std::vector<std::vector<int> >& clusters, std::vector<int>& cluster_removed)
{
	std::cerr << "to do next..." << std::endl;
	std::vector<std::vector<int> > intial_seed_clusters;
	std::vector<int> points_seed_idx(points_grow_seed_.size(), -1);
	if (all_pnts_plane_.plane_normal_.norm() < HW::KMIN_FLOAT_THRESHOLD)
	{
		return;
	}
	for (int i = 0; i < points_grow_seed_.size(); ++i)
	{
		Eigen::Vector3f pnt_seed = points_grow_seed_[i];
		for (int j = 0; j < pointData.pts.size(); ++j)
		{
			Eigen::Vector3f tmp_pnt = Eigen::Vector3f(pointData.pts[j].x,
				pointData.pts[j].z, pointData.pts[j].z);
			if (HW::CheckTwoPnt3dSame(pnt_seed, tmp_pnt))
			{
				points_seed_idx[i] = j;
			}
		}
	}
	std::vector<PlaneFuncInfo> current_initial_planes;
	for (int i = 0; i < points_grow_seed_.size(); ++i)
	{
		PlaneFuncInfo cur_plane;
		cur_plane.center_pnt_ = points_grow_seed_[i];
		cur_plane.plane_normal_ = all_pnts_plane_.plane_normal_;
	}
	intial_seed_clusters.resize(current_initial_planes.size());
	std::vector<int> initial_pnts_labels_;
	initial_pnts_labels_.resize(pointNum, -1);
	std::vector<bool> used_pnts(pointNum, false);
	//get points cluster based on the pnts distance to plane (threshold)
	for (int i = 0; i < points_seed_idx.size(); ++i)
	{
		if (points_seed_idx[i] == -1)
		{
			continue;
		}
		std::vector<int> signle_cluster;
		for (int j = 0; j < pointData.pts.size(); ++j)
		{
			Eigen::Vector3f pnt_pos =
				Eigen::Vector3f(pointData.pts[j].x, pointData.pts[j].y, pointData.pts[j].z);
			Eigen::Vector3f pnt_c = current_initial_planes[i].center_pnt_;
			Eigen::Vector3f pnt_normal = current_initial_planes[i].plane_normal_;
			float pnt2plane_dist = HW::Pnt3d2Plane3DCNDist(pnt_pos, pnt_c, pnt_normal);
			if (pnt2plane_dist < pnt_to_plane_dist_threhold_)
			{
				signle_cluster.emplace_back(j);
				initial_pnts_labels_[j] = i;
			}
		}
		intial_seed_clusters[i] = signle_cluster;
	}

	// sort the data points according to their curvature
	std::vector<std::pair<int, float> > idxSorted(this->pointNum);
	for (int i = 0; i<this->pointNum; ++i)
	{
		idxSorted[i].first = i;
		idxSorted[i].second = pcaInfos[i].lambda0;
	}
	//curvature is increasing order
	std::cout << "start sorting" << std::endl;
	std::sort(idxSorted.begin(), idxSorted.end(), [](const std::pair<int, float>& lhs, const std::pair<int, float>& rhs) { return lhs.second < rhs.second; });
	std::cout << "end sorting" << std::endl;

	//split the polygon based on seed
	//to do next...
	
}

void PointGrowAngleDis::runSingleOriginalPlaneGrowSegmentsFromTwoSeeds(std::vector<std::vector<int> >& clusters, std::vector<int>& cluster_removed)
{
	if (!single_polygon_pnts_flag_)
	{
		std::cerr << "single polygon flag error..." << std::endl;
		return;
	}
	if (points_seed_idxs_.size() != 2)
	{
		std::cerr << "point seed num: " << points_seed_idxs_.size() << std::endl;
		return;
	}
	
	std::vector<std::vector<int> > intial_seed_clusters;
	if (all_pnts_plane_.plane_normal_.norm() < HW::KMIN_FLOAT_THRESHOLD)
	{
		ComputePlaneFromAllPnts(all_pnts_plane_.center_pnt_, all_pnts_plane_.plane_normal_);
	}
	std::vector<Eigen::Vector3f> points_seeds;
	for (int i = 0; i < points_seed_idxs_.size(); ++i)
	{
		int idx = points_seed_idxs_[i];
		Eigen::Vector3f tmp_pnt = Eigen::Vector3f(pointData.pts[idx].x,
			pointData.pts[idx].y, pointData.pts[idx].z);
		points_seeds.emplace_back(tmp_pnt);
	}
	std::vector<PlaneFuncInfo> current_initial_planes;
	for (int i = 0; i < points_seeds.size(); ++i)
	{
		PlaneFuncInfo cur_plane;
		cur_plane.center_pnt_ = points_seeds[i];
		cur_plane.plane_normal_ = all_pnts_plane_.plane_normal_;
		current_initial_planes.emplace_back(cur_plane);
	}
	intial_seed_clusters.resize(current_initial_planes.size());
	std::vector<int> initial_pnts_labels;
	intial_seed_clusters[0].clear();
	intial_seed_clusters[1].clear();
	initial_pnts_labels.resize(pointNum, -1);

	//std::vector<std::vector<int> > intial_seed_clusters;
	//std::vector<int> seed_cluster0;
	//std::vector<int> seed_cluster1;
	//get points cluster based on the pnts distance to plane (threshold)
	for (int j = 0; j < pointData.pts.size(); ++j)
	{
		Eigen::Vector3f pnt_pos =
			Eigen::Vector3f(pointData.pts[j].x, pointData.pts[j].y, pointData.pts[j].z);
		Eigen::Vector3f pnt0_c = current_initial_planes[0].center_pnt_;
		Eigen::Vector3f pnt0_normal = current_initial_planes[0].plane_normal_;
		Eigen::Vector3f pnt1_c = current_initial_planes[1].center_pnt_;
		Eigen::Vector3f pnt1_normal = current_initial_planes[1].plane_normal_;
		float pnt2plane0_dist = HW::Pnt3d2Plane3DCNDist(pnt_pos, pnt0_c, pnt0_normal);
		float pnt2plane1_dist = HW::Pnt3d2Plane3DCNDist(pnt_pos, pnt1_c, pnt1_normal);
		if (pnt2plane0_dist < pnt2plane1_dist)
		{
			initial_pnts_labels[j] = 0;
			//intial_seed_clusters[0].emplace_back(j);
		}
		else
		{
			initial_pnts_labels[j] = 1;
			//intial_seed_clusters[1].emplace_back(j);
		}
	}

	////show the pnt cloud
	//std::cerr << "test..." << std::endl;
	//clusters.clear();
	//clusters.emplace_back(intial_seed_clusters[0]);
	//clusters.emplace_back(intial_seed_clusters[1]);
	//std::cerr << "end test..." << std::endl;
	//return;

	std::vector<bool> idx_used(pointData.pts.size(), false);
	int k = 15;
	std::vector<int> grow_pnts_idx0;
	std::cerr << "initial_pnts_labels 0: " << initial_pnts_labels[points_seed_idxs_[0]] << std::endl;
	std::cerr << "initial_pnts_labels 1: " << initial_pnts_labels[points_seed_idxs_[1]] << std::endl;
	/*runGrowPntsIdxFromSingleIdxRecursive(points_seed_idxs_[0], idx_used, k,
		initial_pnts_labels, grow_pnts_idx0);*/
	runGrowPntsIdxFromSingleIdx(points_seed_idxs_[0], k, initial_pnts_labels, grow_pnts_idx0);
	
	////show the pnt cloud
	//std::cerr << "test..." << std::endl;
	//clusters.clear();
	//clusters.emplace_back(grow_pnts_idx0);
	//clusters.emplace_back(grow_pnts_idx0);
	//std::cerr << "end test..." << std::endl;
	//return;

	//copy left to intial_seed_clusters
	std::vector<int> initial_pnts_labels_grow_one;
	initial_pnts_labels_grow_one.resize(initial_pnts_labels.size(), -1);
	for (int i = 0; i < grow_pnts_idx0.size(); ++i)
	{
		initial_pnts_labels_grow_one[grow_pnts_idx0[i]] = 0;
	}
	for (int i = 0; i < initial_pnts_labels_grow_one.size(); ++i)
	{
		if (initial_pnts_labels_grow_one[i] != 0)
		{
			initial_pnts_labels_grow_one[i] = 1;
		}
	}
#if 0
	for (int i = 0; i < initial_pnts_labels_grow_one.size(); ++i)
	{
		if (initial_pnts_labels_grow_one[i] == 0)
		{
			intial_seed_clusters[0].emplace_back(i);
		}
		if (initial_pnts_labels_grow_one[i] == 1)
		{
			intial_seed_clusters[1].emplace_back(i);
		}
	}
	//show the pnt cloud
	std::cerr << "test..." << std::endl;
	clusters.clear();
	clusters.emplace_back(intial_seed_clusters[0]);
	clusters.emplace_back(intial_seed_clusters[1]);
	std::cerr << "end test..." << std::endl;
	return;
#endif
	std::vector<int> grow_pnts_idx1;
	idx_used.resize(pointData.pts.size(), false);
	std::cerr << "points_seed_idxs_[1]: " << points_seed_idxs_[1] << std::endl;
	runGrowPntsIdxFromSingleIdx(points_seed_idxs_[1], k, 
		initial_pnts_labels_grow_one, grow_pnts_idx1);
	/*runGrowPntsIdxFromSingleIdxRecursive(points_seed_idxs_[1], idx_used, k,
		initial_pnts_labels_grow_one, grow_pnts_idx1);*/
	////show the pnt cloud
	//std::cerr << "test..." << std::endl;
	//clusters.clear();
	//clusters.emplace_back(grow_pnts_idx1);
	//clusters.emplace_back(grow_pnts_idx1);
	//std::cerr << "end test..." << std::endl;
	//return;

	std::vector<int> initial_pnts_labels_grow_two;
	initial_pnts_labels_grow_two.resize(initial_pnts_labels.size(), -1);
	for (int i = 0; i < grow_pnts_idx1.size(); ++i)
	{
		initial_pnts_labels_grow_two[grow_pnts_idx1[i]] = 1;
	}
	for (int i = 0; i < initial_pnts_labels_grow_two.size(); ++i)
	{
		if (initial_pnts_labels_grow_two[i] != 1)
		{
			initial_pnts_labels_grow_two[i] = 0;
		}
	}
	for (int i = 0; i < initial_pnts_labels_grow_two.size(); ++i)
	{
		if (!initial_pnts_labels_grow_two[i])
		{
			intial_seed_clusters[0].emplace_back(i);
		}	
		if (initial_pnts_labels_grow_two[i] == 1)
		{
			intial_seed_clusters[1].emplace_back(i);
		}
	}

	////show the pnt cloud
	//std::cerr << "test..." << std::endl;
	//clusters.clear();
	//clusters.emplace_back(intial_seed_clusters[0]);
	//clusters.emplace_back(intial_seed_clusters[1]);
	//std::cerr << "end test..." << std::endl;
	//return;

	/*for (int i = 0; i < points_seed_idxs_.size(); ++i)
	{
		if (points_seed_idxs_[i] == -1)
		{
			continue;
		}
		std::vector<int> signle_cluster;
		for (int j = 0; j < pointData.pts.size(); ++j)
		{
			Eigen::Vector3f pnt_pos =
				Eigen::Vector3f(pointData.pts[j].x, pointData.pts[j].y, pointData.pts[j].z);
			Eigen::Vector3f pnt_c = current_initial_planes[i].center_pnt_;
			Eigen::Vector3f pnt_normal = current_initial_planes[i].plane_normal_;
			float pnt2plane_dist = HW::Pnt3d2Plane3DCNDist(pnt_pos, pnt_c, pnt_normal);
			if (pnt2plane_dist < pnt_to_plane_dist_threhold_)
			{
				signle_cluster.emplace_back(j);
				initial_pnts_labels_[j] = i;
			}
		}
		intial_seed_clusters[i] = signle_cluster;
	}*/

	////get avarage pnt dist to plane
	//ComputePlaneFromSingleClusterIdx(intial_seed_clusters[0], 
	//	current_initial_planes[0].center_pnt_, current_initial_planes[0].plane_normal_);
	//ComputePlaneFromSingleClusterIdx(intial_seed_clusters[1], 
	//	current_initial_planes[1].center_pnt_, current_initial_planes[1].plane_normal_);
	//float cluster0_value = GetUfEnergyValueFromSinglePlaneIdxs(intial_seed_clusters[0],
	//	current_initial_planes[0].center_pnt_, current_initial_planes[0].plane_normal_);
	//float cluster1_value = GetUfEnergyValueFromSinglePlaneIdxs(intial_seed_clusters[1],
	//	current_initial_planes[1].center_pnt_, current_initial_planes[1].plane_normal_);
	//float avarage_cluster0_dist_to_plane = cluster0_value / intial_seed_clusters[0].size();
	//float avarage_cluster1_dist_to_plane = cluster1_value / intial_seed_clusters[1].size();

	std::vector<int> cur_cluster0 = intial_seed_clusters[0];
	std::vector<int> cur_cluster1 = intial_seed_clusters[1];
	std::vector<int> pre_cluster0 = cur_cluster0;
	std::vector<int> pre_cluster1 = cur_cluster1;
	std::vector<int> cur_clusters_label = initial_pnts_labels_grow_two;
	std::vector<int> pre_clusters_label = initial_pnts_labels_grow_two;
	std::vector<PlaneFuncInfo> cur_planes;
	cur_planes.resize(2);
	float cur_cluster0_value, cur_cluster1_value;
	float pre_sum_value, cur_sum_value;
	bool grow_processing = true;
	int grow_step_num = 0;

	ComputePlaneFromSingleClusterIdx(cur_cluster0,
		cur_planes[0].center_pnt_, cur_planes[0].plane_normal_);
	ComputePlaneFromSingleClusterIdx(cur_cluster1,
		cur_planes[1].center_pnt_, cur_planes[1].plane_normal_);
	cur_cluster0_value = GetUfEnergyValueFromSinglePlaneIdxs(cur_cluster0,
		cur_planes[0].center_pnt_, cur_planes[0].plane_normal_);
	cur_cluster1_value = GetUfEnergyValueFromSinglePlaneIdxs(cur_cluster1,
		cur_planes[1].center_pnt_, cur_planes[1].plane_normal_);
	cur_sum_value = cur_cluster0_value + cur_cluster1_value;

	float avarage_cluster0_dist_to_plane, avarage_cluster1_dist_to_plane;
	do
	{
		pre_sum_value = cur_sum_value;
		pre_clusters_label = cur_clusters_label;
		avarage_cluster0_dist_to_plane = cur_cluster0_value / cur_cluster0.size();
		avarage_cluster1_dist_to_plane = cur_cluster1_value / cur_cluster1.size();
		if (avarage_cluster0_dist_to_plane < avarage_cluster1_dist_to_plane)
		{
			//cluster0 grow to cluster1
			for (int i = 0; i < cur_cluster0.size(); ++i)
			{
				int idx = cur_cluster0[i];
				//pcaInfos[idx].idxIn;
				std::vector<int> neigh_idxs = pcaInfos[idx].idxIn;
				for (int j = 0; j < neigh_idxs.size(); ++j)
				{
					if (cur_clusters_label[neigh_idxs[j]] != 0)
					{
						Eigen::Vector3f pnt_neighbor = Eigen::Vector3f(pointData.pts[neigh_idxs[j]].x, 
							pointData.pts[neigh_idxs[j]].y, pointData.pts[neigh_idxs[j]].z);
						float pnt2plane0_dist = HW::Pnt3d2Plane3DCNDist(pnt_neighbor, 
							cur_planes[0].center_pnt_, cur_planes[0].plane_normal_);
						float pnt2plane1_dist = HW::Pnt3d2Plane3DCNDist(pnt_neighbor,
							cur_planes[1].center_pnt_, cur_planes[1].plane_normal_);
						if (pnt2plane0_dist < pnt2plane1_dist)
						{
							//std::cerr << "000000" << std::endl;
							cur_clusters_label[neigh_idxs[j]] = 0;
						}	
					}
				}
			}
		}
		else
		{
			//cluster1 grow to cluster0
			for (int i = 0; i < cur_cluster1.size(); ++i)
			{
				int idx = cur_cluster1[i];
				//pcaInfos[idx].idxIn;
				std::vector<int> neigh_idxs = pcaInfos[idx].idxIn;
				for (int j = 0; j < neigh_idxs.size(); ++j)
				{
					if (cur_clusters_label[neigh_idxs[j]] != 1)
					{
						Eigen::Vector3f pnt_neighbor = Eigen::Vector3f(pointData.pts[neigh_idxs[j]].x,
							pointData.pts[neigh_idxs[j]].y, pointData.pts[neigh_idxs[j]].z);
						float pnt2plane0_dist = HW::Pnt3d2Plane3DCNDist(pnt_neighbor,
							cur_planes[0].center_pnt_, cur_planes[0].plane_normal_);
						float pnt2plane1_dist = HW::Pnt3d2Plane3DCNDist(pnt_neighbor,
							cur_planes[1].center_pnt_, cur_planes[1].plane_normal_);
						if (pnt2plane0_dist > pnt2plane1_dist)
						{
							//std::cerr << "1111111" << std::endl;
							cur_clusters_label[neigh_idxs[j]] = 1;
						}
					}
				}
			}
		}
		pre_cluster0 = cur_cluster0;
		pre_cluster1 = cur_cluster1;
		cur_cluster0.clear();
		cur_cluster1.clear();
		//new cluster
		for (int i = 0; i < cur_clusters_label.size(); ++i)
		{
			if (cur_clusters_label[i] == 0)
			{
				cur_cluster0.emplace_back(i);
			}
			else if (cur_clusters_label[i] == 1)
			{
				cur_cluster1.emplace_back(i);
			}
			else
			{
				std::cerr << "none..." << std::endl;
			}
		}

		ComputePlaneFromSingleClusterIdx(cur_cluster0,
			cur_planes[0].center_pnt_, cur_planes[0].plane_normal_);
		ComputePlaneFromSingleClusterIdx(cur_cluster1,
			cur_planes[1].center_pnt_, cur_planes[1].plane_normal_);
		/*std::cerr << "cur_planes[0].center_pnt_: " << cur_planes[0].center_pnt_.transpose() << ", " <<
			"cur_planes[0].plane_normal_: " << cur_planes[0].plane_normal_.transpose() << std::endl;
		std::cerr << "cur_planes[0].center_pnt_: " << cur_planes[1].center_pnt_.transpose() << ", " << 
			"cur_planes[0].plane_normal_: " << cur_planes[1].plane_normal_.transpose() << std::endl;*/

		cur_cluster0_value = GetUfEnergyValueFromSinglePlaneIdxs(cur_cluster0,
			cur_planes[0].center_pnt_, cur_planes[0].plane_normal_);
		cur_cluster1_value = GetUfEnergyValueFromSinglePlaneIdxs(cur_cluster1,
			cur_planes[1].center_pnt_, cur_planes[1].plane_normal_);
		cur_sum_value = cur_cluster0_value + cur_cluster1_value;

		std::cerr << "pre_sum_value: " << pre_sum_value << std::endl;
		std::cerr << "cur_sum_value: " << cur_sum_value << std::endl;
		if (cur_sum_value > pre_sum_value)
		{
			grow_processing = false;
			cur_cluster0 = pre_cluster0;
			cur_cluster1 = pre_cluster1;
			cur_clusters_label = pre_clusters_label;
		}
		grow_step_num = grow_step_num + 1;
		std::cerr << "grow_step_num: " << grow_step_num << std::endl;
		if (grow_step_num > 50)
		{
			grow_processing = false;
		}
	} while (grow_processing);

	/*clusters.clear();
	clusters.emplace_back(cur_cluster0);
	clusters.emplace_back(cur_cluster1);
	return;*/


//	//test
//	std::vector<int> cur_split_pnts_idx0, cur_split_pnts_idx1;
//	for (int i = 0; i < cur_clusters_label.size(); ++i)
//	{
//		if (cur_clusters_label[i] == 0)
//		{
//			cur_split_pnts_idx0.emplace_back(i);
//		}
//		else if (cur_clusters_label[i] == 1)
//		{
//			cur_split_pnts_idx1.emplace_back(i);
//		}
//		else
//		{
//			std::cerr << "none..." << std::endl;
//		}
//	}
//	clusters.clear();
//	clusters.emplace_back(cur_split_pnts_idx0);
//	clusters.emplace_back(cur_split_pnts_idx1);
//	return;
//	std::cerr << "End test 0..." << std::endl;
//	//end test

	//growth 来判断连通性,
	std::vector<int> cur_grow_pnts_idx0;
	std::vector<int> cur_grow_pnts_idx1;
	int initial_cur_cluster0_idx = cur_cluster0[0];
	runGrowPntsIdxFromSingleIdx(initial_cur_cluster0_idx, k,
		cur_clusters_label, cur_grow_pnts_idx0);
	int initial_cur_cluster1_idx = cur_cluster1[0];
	runGrowPntsIdxFromSingleIdx(initial_cur_cluster1_idx, k,
		cur_clusters_label, cur_grow_pnts_idx1);
	
	/*clusters.clear();
	clusters.emplace_back(cur_grow_pnts_idx0);
	clusters.emplace_back(cur_grow_pnts_idx1);
	std::cerr << "End test..." << std::endl;
	return;*/

	//cur_clusters_label; get Max clusters points
	/*-------------start get 1/2 max cluster points--------------*/
	std::vector<int> tmp_all_idxs_label = cur_clusters_label;
	if (cur_grow_pnts_idx0.size() < cur_cluster0.size() / 2)
	{
		//get max grow pnts
		std::vector<int> cur_grow_pnts_idx0_new;
		for (int i = 0; i < cur_grow_pnts_idx0.size(); ++i)
		{
			tmp_all_idxs_label[cur_grow_pnts_idx0[i]] = -1;
		}
		for (int i = 0; i < tmp_all_idxs_label.size(); ++i)
		{
			if (tmp_all_idxs_label[i] == 0)
			{
				cur_grow_pnts_idx0_new.emplace_back(i);
			}
		}
		cur_grow_pnts_idx0 = cur_grow_pnts_idx0_new;
		initial_cur_cluster0_idx = cur_grow_pnts_idx0[0];
	}

	tmp_all_idxs_label = cur_clusters_label;	
	if (cur_grow_pnts_idx1.size() < cur_cluster1.size() / 2)
	{
		//get max grow pnts
		std::vector<int> cur_grow_pnts_idx1_new;
		for (int i = 0; i < cur_grow_pnts_idx0.size(); ++i)
		{
			tmp_all_idxs_label[cur_grow_pnts_idx0[i]] = -1;
		}
		for (int i = 0; i < tmp_all_idxs_label.size(); ++i)
		{
			if (tmp_all_idxs_label[i] == 1)
			{
				cur_grow_pnts_idx1_new.emplace_back(i);
			}
		}
		cur_grow_pnts_idx1 = cur_grow_pnts_idx1_new;
		initial_cur_cluster1_idx = cur_grow_pnts_idx1[0];
	}
	
	/*clusters.clear();
	clusters.emplace_back(cur_grow_pnts_idx0);
	clusters.emplace_back(cur_grow_pnts_idx1);
	return;*/
	/*-------------end get 1/2 max cluster points--------------*/

	tmp_all_idxs_label.resize(cur_clusters_label.size(), -1);
	for (int i = 0; i < cur_grow_pnts_idx0.size(); ++i)
	{
		tmp_all_idxs_label[cur_grow_pnts_idx0[i]] = 0;
	}
	for (int i = 0; i < cur_grow_pnts_idx1.size(); ++i)
	{
		tmp_all_idxs_label[cur_grow_pnts_idx1[i]] = 1;
	}
	for (int i = 0; i < tmp_all_idxs_label.size(); ++i)
	{
		if (tmp_all_idxs_label[i] != 1
			|| tmp_all_idxs_label[i] != 0)
		{
			tmp_all_idxs_label[i] = -1;
		}
	}

	std::vector<int> cur_grow_pnts_idx0_0, cur_grow_pnts_idx1_1;
	runGrowPntsIdxFromSelectedIdxsWithLabel(cur_grow_pnts_idx0, k*2, 0,
		tmp_all_idxs_label, cur_grow_pnts_idx0_0);
	runGrowPntsIdxFromSelectedIdxsWithLabel(cur_grow_pnts_idx1, k*2, 1,
		tmp_all_idxs_label, cur_grow_pnts_idx1_1);

	/*runGrowPntsIdxFromSingleIdxWithLabel(initial_cur_cluster0_idx, k,
		tmp_all_idxs_label, cur_grow_pnts_idx0_0);
	runGrowPntsIdxFromSingleIdxWithLabel(initial_cur_cluster1_idx, k,
		tmp_all_idxs_label, cur_grow_pnts_idx1_1);*/

	std::cerr << "cur_cluster0 num, cur_cluster1 num: " << cur_cluster0.size()
		<< ", " << cur_cluster1.size() << std::endl;
	std::cerr << "cur_grow_pnts_idx0 num, cur_grow_pnts_idx1 num: " << cur_grow_pnts_idx0.size()
		<< ", " << cur_grow_pnts_idx1.size() << std::endl;
	std::cerr << "cur_grow_pnts_idx0_0 num, cur_grow_pnts_idx1_1 num: " << cur_grow_pnts_idx0_0.size()
		<< ", " << cur_grow_pnts_idx1_1.size() << std::endl;

	clusters.clear();
	clusters.emplace_back(cur_grow_pnts_idx0_0);
	clusters.emplace_back(cur_grow_pnts_idx1_1);
	return;

#if 0
	if (cur_grow_pnts_idx0.size() == cur_cluster0.size()
		&& cur_grow_pnts_idx1.size() == cur_cluster1.size())
	{
		clusters.clear();
		clusters.emplace_back(cur_cluster0);
		clusters.emplace_back(cur_cluster1);
	}
	else if (cur_grow_pnts_idx0.size() != cur_cluster0.size()
		&& cur_grow_pnts_idx1.size() == cur_cluster1.size())
	{
		//cur_clusters_label
		std::vector<int> tmp_all_idxs_label = cur_clusters_label;
		for (int i = 0; i < cur_grow_pnts_idx0.size(); ++i)
		{
			tmp_all_idxs_label[cur_grow_pnts_idx0[i]] = -1;
		}
		std::vector<int> cur_grow_pnts_idx0_0, cur_grow_pnts_idx1_1;
		if (cur_grow_pnts_idx0.size() > cur_cluster0.size() / 2)
		{		
			for (int i = 0; i < tmp_all_idxs_label.size(); ++i)
			{
				if (tmp_all_idxs_label[i] == -1)
				{
					tmp_all_idxs_label[i] = 0;
				}
				else if(tmp_all_idxs_label[i] == 0)
				{
					tmp_all_idxs_label[i] = 1;
				}
				else
				{
					tmp_all_idxs_label[i] = 1;
				}
			}
			for (int i = 0; i < tmp_all_idxs_label.size(); ++i)
			{
				if (tmp_all_idxs_label[i] == 0)
				{
					cur_grow_pnts_idx0_0.emplace_back(i);
				}
				if (tmp_all_idxs_label[i] == 1)
				{
					cur_grow_pnts_idx1_1.emplace_back(i);
				}
			}
		}
		else
		{
			for (int i = 0; i < tmp_all_idxs_label.size(); ++i)
			{
				if (tmp_all_idxs_label[i] == -1)
				{
					tmp_all_idxs_label[i] = 1;
				}
				else if (tmp_all_idxs_label[i] == 0)
				{
					tmp_all_idxs_label[i] = 0;
				}
				else
				{
					tmp_all_idxs_label[i] = 1;
				}
			}
			for (int i = 0; i < tmp_all_idxs_label.size(); ++i)
			{
				if (tmp_all_idxs_label[i] == 0)
				{
					cur_grow_pnts_idx0_0.emplace_back(i);
				}
				if (tmp_all_idxs_label[i] == 1)
				{
					cur_grow_pnts_idx1_1.emplace_back(i);
				}
			}
		}
		clusters.clear();
		clusters.emplace_back(cur_grow_pnts_idx0_0);
		clusters.emplace_back(cur_grow_pnts_idx1_1);
	}
	else if(cur_grow_pnts_idx0.size() == cur_cluster0.size()
		&& cur_grow_pnts_idx1.size() != cur_cluster1.size())
	{
		//cur_clusters_label
		std::vector<int> tmp_all_idxs_label = cur_clusters_label;
		for (int i = 0; i < cur_grow_pnts_idx1.size(); ++i)
		{
			tmp_all_idxs_label[cur_grow_pnts_idx1[i]] = -1;
		}
		std::vector<int> cur_grow_pnts_idx0_0, cur_grow_pnts_idx1_1;
		if (cur_grow_pnts_idx1.size() > cur_cluster1.size() / 2)
		{
			for (int i = 0; i < tmp_all_idxs_label.size(); ++i)
			{
				if (tmp_all_idxs_label[i] == -1)
				{
					tmp_all_idxs_label[i] = 1;
				}
				else if (tmp_all_idxs_label[i] == 1)
				{
					tmp_all_idxs_label[i] = 0;
				}
				else
				{
					tmp_all_idxs_label[i] = 0;
				}
			}
			for (int i = 0; i < tmp_all_idxs_label.size(); ++i)
			{
				if (tmp_all_idxs_label[i] == 0)
				{
					cur_grow_pnts_idx0_0.emplace_back(i);
				}
				if (tmp_all_idxs_label[i] == 1)
				{
					cur_grow_pnts_idx1_1.emplace_back(i);
				}
			}
		}
		else
		{
			for (int i = 0; i < tmp_all_idxs_label.size(); ++i)
			{
				if (tmp_all_idxs_label[i] == -1)
				{
					tmp_all_idxs_label[i] = 0;
				}
				else if (tmp_all_idxs_label[i] == 1)
				{
					tmp_all_idxs_label[i] = 1;
				}
				else
				{
					tmp_all_idxs_label[i] = 0;
				}
			}
			for (int i = 0; i < tmp_all_idxs_label.size(); ++i)
			{
				if (tmp_all_idxs_label[i] == 0)
				{
					cur_grow_pnts_idx0_0.emplace_back(i);
				}
				if (tmp_all_idxs_label[i] == 1)
				{
					cur_grow_pnts_idx1_1.emplace_back(i);
				}
			}
		}
		clusters.clear();
		clusters.emplace_back(cur_grow_pnts_idx0_0);
		clusters.emplace_back(cur_grow_pnts_idx1_1);
	}
	else
	{
		//cur_clusters_label
		std::vector<int> tmp_all_idxs_label = cur_clusters_label;
		for (int i = 0; i < cur_grow_pnts_idx0.size(); ++i)
		{
			tmp_all_idxs_label[cur_grow_pnts_idx0[i]] = -1;
		}
		std::vector<int> cur_grow_pnts_idx0_0, cur_grow_pnts_idx1_1;
		if (cur_grow_pnts_idx0.size() > cur_cluster0.size() / 2)
		{
			for (int i = 0; i < tmp_all_idxs_label.size(); ++i)
			{
				if (tmp_all_idxs_label[i] == -1)
				{
					tmp_all_idxs_label[i] = 0;
				}
				else if (tmp_all_idxs_label[i] == 0)
				{
					tmp_all_idxs_label[i] = 1;
				}
				else
				{
					tmp_all_idxs_label[i] = 1;
				}
			}
			for (int i = 0; i < tmp_all_idxs_label.size(); ++i)
			{
				if (tmp_all_idxs_label[i] == 0)
				{
					cur_grow_pnts_idx0_0.emplace_back(i);
				}
				if (tmp_all_idxs_label[i] == 1)
				{
					cur_grow_pnts_idx1_1.emplace_back(i);
				}
			}
		}
		else
		{
			for (int i = 0; i < tmp_all_idxs_label.size(); ++i)
			{
				if (tmp_all_idxs_label[i] == -1)
				{
					tmp_all_idxs_label[i] = 1;
				}
				else if (tmp_all_idxs_label[i] == 0)
				{
					tmp_all_idxs_label[i] = 0;
				}
				else
				{
					tmp_all_idxs_label[i] = 1;
				}
			}
			for (int i = 0; i < tmp_all_idxs_label.size(); ++i)
			{
				if (tmp_all_idxs_label[i] == 0)
				{
					cur_grow_pnts_idx0_0.emplace_back(i);
				}
				if (tmp_all_idxs_label[i] == 1)
				{
					cur_grow_pnts_idx1_1.emplace_back(i);
				}
			}
		}

		clusters.clear();
		clusters.emplace_back(cur_grow_pnts_idx0_0);
		clusters.emplace_back(cur_grow_pnts_idx1_1);
		return;
	}
#endif

	//// sort the data points according to their curvature
	//std::vector<std::pair<int, float> > idxSorted(this->pointNum);
	//for (int i = 0; i<this->pointNum; ++i)
	//{
	//	idxSorted[i].first = i;
	//	idxSorted[i].second = pcaInfos[i].lambda0;
	//}
	////curvature is increasing order
	//std::cout << "start sorting" << std::endl;
	//std::sort(idxSorted.begin(), idxSorted.end(), [](const std::pair<int, float>& lhs, const std::pair<int, float>& rhs) { return lhs.second < rhs.second; });
	//std::cout << "end sorting" << std::endl;
	//split the polygon based on seed
	//to do next...
}

void PointGrowAngleDis::runSingleOriginalPlaneGrowSegmentsFromTwoSeedsNew(std::vector<std::vector<int> >& clusters, std::vector<int>& cluster_removed)
{
	if (!single_polygon_pnts_flag_)
	{
		std::cerr << "single polygon flag error..." << std::endl;
		return;
	}
	if (points_seed_idxs_.size() != 2)
	{
		std::cerr << "point seed num: " << points_seed_idxs_.size() << std::endl;
		return;
	}

	std::vector<std::vector<int> > intial_seed_clusters;
	if (all_pnts_plane_.plane_normal_.norm() < HW::KMIN_FLOAT_THRESHOLD)
	{
		ComputePlaneFromAllPnts(all_pnts_plane_.center_pnt_, all_pnts_plane_.plane_normal_);
	}
	std::vector<Eigen::Vector3f> points_seeds;
	for (int i = 0; i < points_seed_idxs_.size(); ++i)
	{
		int idx = points_seed_idxs_[i];
		Eigen::Vector3f tmp_pnt = Eigen::Vector3f(pointData.pts[idx].x,
			pointData.pts[idx].y, pointData.pts[idx].z);
		points_seeds.emplace_back(tmp_pnt);
	}
	std::vector<PlaneFuncInfo> current_initial_planes;
	for (int i = 0; i < points_seeds.size(); ++i)
	{
		PlaneFuncInfo cur_plane;
		cur_plane.center_pnt_ = points_seeds[i];
		cur_plane.plane_normal_ = all_pnts_plane_.plane_normal_;
		current_initial_planes.emplace_back(cur_plane);
	}
	intial_seed_clusters.resize(current_initial_planes.size());
	std::vector<int> initial_pnts_labels;
	intial_seed_clusters[0].clear();
	intial_seed_clusters[1].clear();
	initial_pnts_labels.resize(pointNum, -1);

	//std::vector<std::vector<int> > intial_seed_clusters;
	//std::vector<int> seed_cluster0;
	//std::vector<int> seed_cluster1;
	//get points cluster based on the pnts distance to plane (threshold)
	for (int j = 0; j < pointData.pts.size(); ++j)
	{
		Eigen::Vector3f pnt_pos =
			Eigen::Vector3f(pointData.pts[j].x, pointData.pts[j].y, pointData.pts[j].z);
		Eigen::Vector3f pnt0_c = current_initial_planes[0].center_pnt_;
		Eigen::Vector3f pnt0_normal = current_initial_planes[0].plane_normal_;
		Eigen::Vector3f pnt1_c = current_initial_planes[1].center_pnt_;
		Eigen::Vector3f pnt1_normal = current_initial_planes[1].plane_normal_;
		float pnt2plane0_dist = HW::Pnt3d2Plane3DCNDist(pnt_pos, pnt0_c, pnt0_normal);
		float pnt2plane1_dist = HW::Pnt3d2Plane3DCNDist(pnt_pos, pnt1_c, pnt1_normal);
		if (pnt2plane0_dist < pnt2plane1_dist)
		{
			initial_pnts_labels[j] = 0;
			//intial_seed_clusters[0].emplace_back(j);
		}
		else
		{
			initial_pnts_labels[j] = 1;
			//intial_seed_clusters[1].emplace_back(j);
		}
	}

	////show the pnt cloud
	//std::cerr << "test..." << std::endl;
	//clusters.clear();
	//clusters.emplace_back(intial_seed_clusters[0]);
	//clusters.emplace_back(intial_seed_clusters[1]);
	//std::cerr << "end test..." << std::endl;
	//return;

	std::vector<bool> idx_used(pointData.pts.size(), false);
	int k = 15;
	std::vector<int> grow_pnts_idx0;
	std::cerr << "initial_pnts_labels 0: " << initial_pnts_labels[points_seed_idxs_[0]] << std::endl;
	std::cerr << "initial_pnts_labels 1: " << initial_pnts_labels[points_seed_idxs_[1]] << std::endl;
	/*runGrowPntsIdxFromSingleIdxRecursive(points_seed_idxs_[0], idx_used, k,
	initial_pnts_labels, grow_pnts_idx0);*/
	runGrowPntsIdxFromSingleIdx(points_seed_idxs_[0], k, initial_pnts_labels, grow_pnts_idx0);
	////show the pnt cloud
	//std::cerr << "test..." << std::endl;
	//clusters.clear();
	//clusters.emplace_back(grow_pnts_idx0);
	//clusters.emplace_back(grow_pnts_idx0);
	//std::cerr << "end test..." << std::endl;
	//return;

	//copy left to intial_seed_clusters
	std::vector<int> initial_pnts_labels_grow_one;
	initial_pnts_labels_grow_one.resize(initial_pnts_labels.size(), -1);
	for (int i = 0; i < grow_pnts_idx0.size(); ++i)
	{
		initial_pnts_labels_grow_one[grow_pnts_idx0[i]] = 0;
	}
	for (int i = 0; i < initial_pnts_labels_grow_one.size(); ++i)
	{
		if (initial_pnts_labels_grow_one[i] != 0)
		{
			initial_pnts_labels_grow_one[i] = 1;
		}
	}
#if 0
	for (int i = 0; i < initial_pnts_labels_grow_one.size(); ++i)
	{
		if (initial_pnts_labels_grow_one[i] == 0)
		{
			intial_seed_clusters[0].emplace_back(i);
		}
		if (initial_pnts_labels_grow_one[i] == 1)
		{
			intial_seed_clusters[1].emplace_back(i);
		}
	}
	//show the pnt cloud
	std::cerr << "test..." << std::endl;
	clusters.clear();
	clusters.emplace_back(intial_seed_clusters[0]);
	clusters.emplace_back(intial_seed_clusters[1]);
	std::cerr << "end test..." << std::endl;
	return;
#endif
	std::vector<int> grow_pnts_idx1;
	idx_used.resize(pointData.pts.size(), false);
	std::cerr << "points_seed_idxs_[1]: " << points_seed_idxs_[1] << std::endl;
	runGrowPntsIdxFromSingleIdx(points_seed_idxs_[1], k,
		initial_pnts_labels_grow_one, grow_pnts_idx1);
	/*runGrowPntsIdxFromSingleIdxRecursive(points_seed_idxs_[1], idx_used, k,
	initial_pnts_labels_grow_one, grow_pnts_idx1);*/
	////show the pnt cloud
	//std::cerr << "test..." << std::endl;
	//clusters.clear();
	//clusters.emplace_back(grow_pnts_idx1);
	//clusters.emplace_back(grow_pnts_idx1);
	//std::cerr << "end test..." << std::endl;
	//return;

	std::vector<int> initial_pnts_labels_grow_two;
	initial_pnts_labels_grow_two.resize(initial_pnts_labels.size(), -1);
	for (int i = 0; i < grow_pnts_idx1.size(); ++i)
	{
		initial_pnts_labels_grow_two[grow_pnts_idx1[i]] = 1;
	}
	for (int i = 0; i < initial_pnts_labels_grow_two.size(); ++i)
	{
		if (initial_pnts_labels_grow_two[i] != 1)
		{
			initial_pnts_labels_grow_two[i] = 0;
		}
	}
	for (int i = 0; i < initial_pnts_labels_grow_two.size(); ++i)
	{
		if (!initial_pnts_labels_grow_two[i])
		{
			intial_seed_clusters[0].emplace_back(i);
		}
		if (initial_pnts_labels_grow_two[i] == 1)
		{
			intial_seed_clusters[1].emplace_back(i);
		}
	}

	////show the pnt cloud
	//std::cerr << "test..." << std::endl;
	//clusters.clear();
	//clusters.emplace_back(intial_seed_clusters[0]);
	//clusters.emplace_back(intial_seed_clusters[1]);
	//std::cerr << "end test..." << std::endl;
	//return;

	/*for (int i = 0; i < points_seed_idxs_.size(); ++i)
	{
	if (points_seed_idxs_[i] == -1)
	{
	continue;
	}
	std::vector<int> signle_cluster;
	for (int j = 0; j < pointData.pts.size(); ++j)
	{
	Eigen::Vector3f pnt_pos =
	Eigen::Vector3f(pointData.pts[j].x, pointData.pts[j].y, pointData.pts[j].z);
	Eigen::Vector3f pnt_c = current_initial_planes[i].center_pnt_;
	Eigen::Vector3f pnt_normal = current_initial_planes[i].plane_normal_;
	float pnt2plane_dist = HW::Pnt3d2Plane3DCNDist(pnt_pos, pnt_c, pnt_normal);
	if (pnt2plane_dist < pnt_to_plane_dist_threhold_)
	{
	signle_cluster.emplace_back(j);
	initial_pnts_labels_[j] = i;
	}
	}
	intial_seed_clusters[i] = signle_cluster;
	}*/

	std::vector<int> cur_cluster0 = intial_seed_clusters[0];
	std::vector<int> cur_cluster1 = intial_seed_clusters[1];
	std::vector<int> pre_cluster0 = cur_cluster0;
	std::vector<int> pre_cluster1 = cur_cluster1;
	std::vector<int> cur_clusters_label = initial_pnts_labels_grow_two;
	std::vector<int> pre_clusters_label = initial_pnts_labels_grow_two;
	std::vector<PlaneFuncInfo> cur_planes;
	cur_planes.resize(2);
	float cur_cluster0_value, cur_cluster1_value;
	float pre_sum_value, cur_sum_value;
	bool grow_processing = true;
	int grow_step_num = 0;

	ComputePlaneFromSingleClusterIdx(cur_cluster0,
		cur_planes[0].center_pnt_, cur_planes[0].plane_normal_);
	ComputePlaneFromSingleClusterIdx(cur_cluster1,
		cur_planes[1].center_pnt_, cur_planes[1].plane_normal_);
	cur_cluster0_value = GetUfEnergyValueFromSinglePlaneIdxs(cur_cluster0,
		cur_planes[0].center_pnt_, cur_planes[0].plane_normal_);
	cur_cluster1_value = GetUfEnergyValueFromSinglePlaneIdxs(cur_cluster1,
		cur_planes[1].center_pnt_, cur_planes[1].plane_normal_);
	cur_sum_value = cur_cluster0_value + cur_cluster1_value;

	float avarage_cluster0_dist_to_plane, avarage_cluster1_dist_to_plane;
	bool cluster0_grow_cluster1_flag = true; //to do next...
	do
	{
		pre_sum_value = cur_sum_value;
		pre_clusters_label = cur_clusters_label;
		avarage_cluster0_dist_to_plane = cur_cluster0_value / cur_cluster0.size();
		avarage_cluster1_dist_to_plane = cur_cluster1_value / cur_cluster1.size();
		if (avarage_cluster0_dist_to_plane < avarage_cluster1_dist_to_plane)
		{
			//cluster0 grow to cluster1
			for (int i = 0; i < cur_cluster0.size(); ++i)
			{
				int idx = cur_cluster0[i];
				//pcaInfos[idx].idxIn;
				std::vector<int> neigh_idxs = pcaInfos[idx].idxIn;
				for (int j = 0; j < neigh_idxs.size(); ++j)
				{
					if (cur_clusters_label[neigh_idxs[j]] != 0)
					{
						Eigen::Vector3f pnt_neighbor = Eigen::Vector3f(pointData.pts[neigh_idxs[j]].x,
							pointData.pts[neigh_idxs[j]].y, pointData.pts[neigh_idxs[j]].z);
						float pnt2plane0_dist = HW::Pnt3d2Plane3DCNDist(pnt_neighbor,
							cur_planes[0].center_pnt_, cur_planes[0].plane_normal_);
						float pnt2plane1_dist = HW::Pnt3d2Plane3DCNDist(pnt_neighbor,
							cur_planes[1].center_pnt_, cur_planes[1].plane_normal_);
						if (pnt2plane0_dist < pnt2plane1_dist)
						{
							//std::cerr << "000000" << std::endl;
							cur_clusters_label[neigh_idxs[j]] = 0;
						}
					}
				}
			}
		}
		else
		{
			//cluster1 grow to cluster0
			for (int i = 0; i < cur_cluster1.size(); ++i)
			{
				int idx = cur_cluster1[i];
				//pcaInfos[idx].idxIn;
				std::vector<int> neigh_idxs = pcaInfos[idx].idxIn;
				for (int j = 0; j < neigh_idxs.size(); ++j)
				{
					if (cur_clusters_label[neigh_idxs[j]] != 1)
					{
						Eigen::Vector3f pnt_neighbor = Eigen::Vector3f(pointData.pts[neigh_idxs[j]].x,
							pointData.pts[neigh_idxs[j]].y, pointData.pts[neigh_idxs[j]].z);
						float pnt2plane0_dist = HW::Pnt3d2Plane3DCNDist(pnt_neighbor,
							cur_planes[0].center_pnt_, cur_planes[0].plane_normal_);
						float pnt2plane1_dist = HW::Pnt3d2Plane3DCNDist(pnt_neighbor,
							cur_planes[1].center_pnt_, cur_planes[1].plane_normal_);
						if (pnt2plane0_dist > pnt2plane1_dist)
						{
							//std::cerr << "1111111" << std::endl;
							cur_clusters_label[neigh_idxs[j]] = 1;
						}
					}
				}
			}
		}
		pre_cluster0 = cur_cluster0;
		pre_cluster1 = cur_cluster1;
		cur_cluster0.clear();
		cur_cluster1.clear();
		//new cluster
		for (int i = 0; i < cur_clusters_label.size(); ++i)
		{
			if (cur_clusters_label[i] == 0)
			{
				cur_cluster0.emplace_back(i);
			}
			else if (cur_clusters_label[i] == 1)
			{
				cur_cluster1.emplace_back(i);
			}
			else
			{
				std::cerr << "none..." << std::endl;
			}
		}

		ComputePlaneFromSingleClusterIdx(cur_cluster0,
			cur_planes[0].center_pnt_, cur_planes[0].plane_normal_);
		ComputePlaneFromSingleClusterIdx(cur_cluster1,
			cur_planes[1].center_pnt_, cur_planes[1].plane_normal_);
		/*std::cerr << "cur_planes[0].center_pnt_: " << cur_planes[0].center_pnt_.transpose() << ", " <<
		"cur_planes[0].plane_normal_: " << cur_planes[0].plane_normal_.transpose() << std::endl;
		std::cerr << "cur_planes[0].center_pnt_: " << cur_planes[1].center_pnt_.transpose() << ", " <<
		"cur_planes[0].plane_normal_: " << cur_planes[1].plane_normal_.transpose() << std::endl;*/

		cur_cluster0_value = GetUfEnergyValueFromSinglePlaneIdxs(cur_cluster0,
			cur_planes[0].center_pnt_, cur_planes[0].plane_normal_);
		cur_cluster1_value = GetUfEnergyValueFromSinglePlaneIdxs(cur_cluster1,
			cur_planes[1].center_pnt_, cur_planes[1].plane_normal_);
		cur_sum_value = cur_cluster0_value + cur_cluster1_value;

		std::cerr << "pre_sum_value: " << pre_sum_value << std::endl;
		std::cerr << "cur_sum_value: " << cur_sum_value << std::endl;
		if (cur_sum_value > pre_sum_value)
		{
			grow_processing = false;
			cur_cluster0 = pre_cluster0;
			cur_cluster1 = pre_cluster1;
			cur_clusters_label = pre_clusters_label;
		}
		grow_step_num = grow_step_num + 1;
		std::cerr << "grow_step_num: " << grow_step_num << std::endl;
		if (grow_step_num > 50)
		{
			grow_processing = false;
		}
	} while (grow_processing);

	/*clusters.clear();
	clusters.emplace_back(cur_cluster0);
	clusters.emplace_back(cur_cluster1);
	return;*/


	//	//test
	//	std::vector<int> cur_split_pnts_idx0, cur_split_pnts_idx1;
	//	for (int i = 0; i < cur_clusters_label.size(); ++i)
	//	{
	//		if (cur_clusters_label[i] == 0)
	//		{
	//			cur_split_pnts_idx0.emplace_back(i);
	//		}
	//		else if (cur_clusters_label[i] == 1)
	//		{
	//			cur_split_pnts_idx1.emplace_back(i);
	//		}
	//		else
	//		{
	//			std::cerr << "none..." << std::endl;
	//		}
	//	}
	//	clusters.clear();
	//	clusters.emplace_back(cur_split_pnts_idx0);
	//	clusters.emplace_back(cur_split_pnts_idx1);
	//	return;
	//	std::cerr << "End test 0..." << std::endl;
	//	//end test

	//growth 来判断连通性,
	std::vector<int> cur_grow_pnts_idx0;
	std::vector<int> cur_grow_pnts_idx1;
	int initial_cur_cluster0_idx = cur_cluster0[0];
	runGrowPntsIdxFromSingleIdx(initial_cur_cluster0_idx, k,
		cur_clusters_label, cur_grow_pnts_idx0);
	int initial_cur_cluster1_idx = cur_cluster1[0];
	runGrowPntsIdxFromSingleIdx(initial_cur_cluster1_idx, k,
		cur_clusters_label, cur_grow_pnts_idx1);

	/*clusters.clear();
	clusters.emplace_back(cur_grow_pnts_idx0);
	clusters.emplace_back(cur_grow_pnts_idx1);
	std::cerr << "End test..." << std::endl;
	return;*/

	//cur_clusters_label; get Max clusters points
	/*-------------start get 1/2 max cluster points--------------*/
	std::vector<int> tmp_all_idxs_label = cur_clusters_label;
	if (cur_grow_pnts_idx0.size() < cur_cluster0.size() / 2)
	{
		//get max grow pnts
		std::vector<int> cur_grow_pnts_idx0_new;
		for (int i = 0; i < cur_grow_pnts_idx0.size(); ++i)
		{
			tmp_all_idxs_label[cur_grow_pnts_idx0[i]] = -1;
		}
		for (int i = 0; i < tmp_all_idxs_label.size(); ++i)
		{
			if (tmp_all_idxs_label[i] == 0)
			{
				cur_grow_pnts_idx0_new.emplace_back(i);
			}
		}
		cur_grow_pnts_idx0 = cur_grow_pnts_idx0_new;
		initial_cur_cluster0_idx = cur_grow_pnts_idx0[0];
	}

	tmp_all_idxs_label = cur_clusters_label;
	if (cur_grow_pnts_idx1.size() < cur_cluster1.size() / 2)
	{
		//get max grow pnts
		std::vector<int> cur_grow_pnts_idx1_new;
		for (int i = 0; i < cur_grow_pnts_idx0.size(); ++i)
		{
			tmp_all_idxs_label[cur_grow_pnts_idx0[i]] = -1;
		}
		for (int i = 0; i < tmp_all_idxs_label.size(); ++i)
		{
			if (tmp_all_idxs_label[i] == 1)
			{
				cur_grow_pnts_idx1_new.emplace_back(i);
			}
		}
		cur_grow_pnts_idx1 = cur_grow_pnts_idx1_new;
		initial_cur_cluster1_idx = cur_grow_pnts_idx1[0];
	}

	/*clusters.clear();
	clusters.emplace_back(cur_grow_pnts_idx0);
	clusters.emplace_back(cur_grow_pnts_idx1);
	return;*/
	/*-------------end get 1/2 max cluster points--------------*/

	tmp_all_idxs_label.resize(cur_clusters_label.size(), -1);
	for (int i = 0; i < cur_grow_pnts_idx0.size(); ++i)
	{
		tmp_all_idxs_label[cur_grow_pnts_idx0[i]] = 0;
	}
	for (int i = 0; i < cur_grow_pnts_idx1.size(); ++i)
	{
		tmp_all_idxs_label[cur_grow_pnts_idx1[i]] = 1;
	}
	for (int i = 0; i < tmp_all_idxs_label.size(); ++i)
	{
		if (tmp_all_idxs_label[i] != 1
			|| tmp_all_idxs_label[i] != 0)
		{
			tmp_all_idxs_label[i] = -1;
		}
	}

	std::vector<int> cur_grow_pnts_idx0_0, cur_grow_pnts_idx1_1;
	runGrowPntsIdxFromSelectedIdxsWithLabel(cur_grow_pnts_idx0, k * 2, 0,
		tmp_all_idxs_label, cur_grow_pnts_idx0_0);
	runGrowPntsIdxFromSelectedIdxsWithLabel(cur_grow_pnts_idx1, k * 2, 1,
		tmp_all_idxs_label, cur_grow_pnts_idx1_1);

	std::cerr << "cur_cluster0 num, cur_cluster1 num: " << cur_cluster0.size()
		<< ", " << cur_cluster1.size() << std::endl;
	std::cerr << "cur_grow_pnts_idx0 num, cur_grow_pnts_idx1 num: " << cur_grow_pnts_idx0.size()
		<< ", " << cur_grow_pnts_idx1.size() << std::endl;
	std::cerr << "cur_grow_pnts_idx0_0 num, cur_grow_pnts_idx1_1 num: " << cur_grow_pnts_idx0_0.size()
		<< ", " << cur_grow_pnts_idx1_1.size() << std::endl;

	clusters.clear();
	clusters.emplace_back(cur_grow_pnts_idx0_0);
	clusters.emplace_back(cur_grow_pnts_idx1_1);
	return;
}

void PointGrowAngleDis::runGrowPntsIdxFromSingleIdxRecursive(int seed_idx, std::vector<bool>& used_idxs, int k,
	const std::vector<int>& all_pnts_labels, std::vector<int>& grow_pnts_idx)
{
	if (seed_idx >= used_idxs.size() ||
		seed_idx < 0)
	{
		return;
	}
	if (used_idxs[seed_idx])
	{
		return;
	}
	grow_pnts_idx.emplace_back(seed_idx);
	used_idxs[seed_idx] = true;
	if (seed_idx >= pcaInfos.size())
	{
		return;
	}
	std::vector<int> neigh_idxs = pcaInfos[seed_idx].idxIn;
	//check the point if it is in the plane one side
	int neigh_num = static_cast<int>(neigh_idxs.size());
	int k_min = std::min(k, neigh_num);
	//std::cerr << "neigh_num, k: " << neigh_num << ", " << k << std::endl;
	for (int i = 0; i < k_min; ++i)
	{
		//find k
		int neigh_idx = neigh_idxs[i];
		if (neigh_idx >= all_pnts_labels.size() ||
			neigh_idx < 0)
		{
			continue;
		}
		if (all_pnts_labels[seed_idx] == all_pnts_labels[neigh_idx]
			&& neigh_idx != seed_idx)
		{
			runGrowPntsIdxFromSingleIdxRecursive(neigh_idx,
				used_idxs, k, all_pnts_labels, grow_pnts_idx);
		}
		else
		{
			used_idxs[neigh_idx] = true;
		}
	}
}

void PointGrowAngleDis::runGrowPntsIdxFromSingleIdx(int seed_idx, int k, const std::vector<int>& all_pnts_labels, 
	std::vector<int>& grow_pnts_idx)
{
	// begin region growing
	std::vector<int> used_idxs(all_pnts_labels.size(), false);
	std::vector<int> clusterNew;
	clusterNew.push_back(seed_idx);
	//cv::Matx31d normalStart = pcaInfos[seed_idx].normal;
	int count = 0;
	while (count < clusterNew.size())
	{
		//std::cerr << "count: " << count << std::endl;
		int idxSeed = clusterNew[count];
		int num = pcaInfos[idxSeed].idxIn.size();
		int k_min = std::min(num, k);
		// neighbor idxs
		//std::cerr << "num, k: " << num << ", " << k << std::endl;
		for (int j = 0; j < k_min; ++j)
		{
			int neighbor_idx = pcaInfos[idxSeed].idxIn[j];
			if (all_pnts_labels[idxSeed] == all_pnts_labels[neighbor_idx]
				&& neighbor_idx != idxSeed)
			{
				if (!used_idxs[neighbor_idx])
				{
					clusterNew.emplace_back(neighbor_idx);
					used_idxs[neighbor_idx] = true;
				}
			}
		}
		count++;

		/*if (used_idxs[idxSeed])
		{
			count++;
		}
		else
		{	
		}*/
	}
	grow_pnts_idx = clusterNew;
}

void PointGrowAngleDis::runGrowPntsIdxFromSingleIdxWithLabel(int seed_idx, int k, const std::vector<int>& all_pnts_labels,
	std::vector<int>& grow_pnts_idx)
{
	// begin region growing
	std::vector<bool> used_idxs(all_pnts_labels.size(), false);
	std::vector<int> clusterNew;
	clusterNew.push_back(seed_idx);
	//cv::Matx31d normalStart = pcaInfos[seed_idx].normal;
	int count = 0;
	while (count < clusterNew.size())
	{
		//std::cerr << "count: " << count << std::endl;
		int idxSeed = clusterNew[count];
		int num = pcaInfos[idxSeed].idxIn.size();
		int k_min = std::min(num, k);
		// neighbor idxs
		//std::cerr << "num, k: " << num << ", " << k << std::endl;
		for (int j = 0; j < k_min; ++j)
		{
			int neighbor_idx = pcaInfos[idxSeed].idxIn[j];
			if ((all_pnts_labels[idxSeed] == all_pnts_labels[neighbor_idx]
				|| all_pnts_labels[neighbor_idx] == -1) && neighbor_idx != idxSeed)
			{
				if (!used_idxs[neighbor_idx])
				{
					clusterNew.emplace_back(neighbor_idx);
					used_idxs[neighbor_idx] = true;
				}
			}
		}
		count++;
	}
	grow_pnts_idx = clusterNew;
}

void PointGrowAngleDis::runGrowPntsIdxFromSelectedIdxsWithLabel(const std::vector<int>& origin_idxs, int k, int idx_label,
	const std::vector<int>& all_pnts_labels, std::vector<int>& grow_pnts_idx)
{
	// begin region growing
	std::vector<bool> used_idxs(all_pnts_labels.size(), false);
	std::vector<int> all_cluster_pnts_idxs;
	for (int i = 0; i < origin_idxs.size(); ++i)
	{
		if (used_idxs[origin_idxs[i]])
			continue;
		std::vector<int> clusterNew;
		clusterNew.push_back(origin_idxs[i]);
		//cv::Matx31d normalStart = pcaInfos[seed_idx].normal;
		int count = 0;
		while (count < clusterNew.size())
		{
			//std::cerr << "count: " << count << std::endl;
			int idxSeed = clusterNew[count];
			int num = pcaInfos[idxSeed].idxAll.size();
			int k_min = std::min(k, num);
			// neighbor idxs
			//std::cerr << "num, k: " << num << ", " << k << std::endl;
			for (int j = 0; j < k_min; ++j)
			{
				int neighbor_idx = pcaInfos[idxSeed].idxAll[j];
				if (idx_label == all_pnts_labels[neighbor_idx] && neighbor_idx != idxSeed)
				{
					if (!used_idxs[neighbor_idx])
					{
						//std::cerr << neighbor_idx << ": all_pnts_labels[idxSeed]" << std::endl;
						clusterNew.emplace_back(neighbor_idx);
						used_idxs[neighbor_idx] = true;
					}
				}
				if (all_pnts_labels[neighbor_idx] == -1 && neighbor_idx != idxSeed)
				{
					if (!used_idxs[neighbor_idx])
					{
						std::cerr << neighbor_idx << ": -1" << std::endl;
						clusterNew.emplace_back(neighbor_idx);
						used_idxs[neighbor_idx] = true;
					}
				}
			}
			count++;
		}
		for (int j = 0; j < clusterNew.size(); ++j)
		{
			all_cluster_pnts_idxs.emplace_back(clusterNew[j]);
		}
		//all_cluster_pnts_idxs.insert(all_cluster_pnts_idxs.end(), clusterNew.begin(), clusterNew.end());
	}
	grow_pnts_idx = all_cluster_pnts_idxs;
}

float PointGrowAngleDis::meadian(std::vector<float> &dataset)
{
	std::sort(dataset.begin(), dataset.end(), [](const float& lhs, const float& rhs) { return lhs < rhs; });

	return dataset[dataset.size() / 2];
}

void PointGrowAngleDis::getPlaneInfo(std::vector<std::vector<int>>& clusters)
{
	for (int i = 0; i < clusters.size(); i++) {
		int num_vers = clusters[i].size();
		Eigen::Matrix3f covariance_matrix = Eigen::Matrix3f::Zero(3, 3);
		Eigen::Vector3f mean;
		Eigen::Vector3f boxMin(FLT_MAX, FLT_MAX, FLT_MAX), boxMax(-FLT_MAX, -FLT_MAX, -FLT_MAX);
		for (int j = 0; j < num_vers; j++) {
			int idx = clusters[i][j];
			mean(0) += pointData.pts[idx].x;
			mean(1) += pointData.pts[idx].y;
			mean(2) += pointData.pts[idx].z;
			if (pointData.pts[idx].x < boxMin(0))
				boxMin(0) = pointData.pts[idx].x;
			if (pointData.pts[idx].y < boxMin(1))
				boxMin(1) = pointData.pts[idx].y;
			if (pointData.pts[idx].z < boxMin(2))
				boxMin(2) = pointData.pts[idx].z;
			if (pointData.pts[idx].x > boxMax(0))
				boxMax(0) = pointData.pts[idx].x;
			if (pointData.pts[idx].y > boxMax(1))
				boxMax(1) = pointData.pts[idx].y;
			if (pointData.pts[idx].z > boxMax(2))
				boxMax(2) = pointData.pts[idx].z;
		}
		if (num_vers != 0) {
			mean = mean / num_vers;
		}
		mean_vec.emplace_back(mean);
		for (int j = 0; j < num_vers; ++j)
		{
			//
			int idx = clusters[i][j];
			Eigen::Vector3f vertex(pointData.pts[idx].x, pointData.pts[idx].y, pointData.pts[idx].z);
			Eigen::Matrix3f multiplied_matrix = (vertex - mean)*(vertex - mean).transpose();
			//std::cout << "temp matrix: \n" << temp_matrix << std::endl;
			covariance_matrix = covariance_matrix + multiplied_matrix;
		}

		covariance_matrix = covariance_matrix / num_vers;

		boxMin_vec.emplace_back(boxMin);
		boxMax_vec.emplace_back(boxMax);
		Eigen::Matrix3f eigen_values = Eigen::Matrix3f::Zero(3, 3);
		Eigen::Matrix3f eigen_vector = Eigen::Matrix3f::Zero(3, 3);
		Eigen::EigenSolver<Eigen::Matrix3f> es(covariance_matrix);
		eigen_values = es.pseudoEigenvalueMatrix();
		eigen_vector = es.pseudoEigenvectors();
		int min_eigen_idx = -1;
		float min_value = FLT_MAX;

		for (int j = 0; j < 3; ++j)
		{
			if (min_value > eigen_values(j, j))
			{
				min_value = eigen_values(j, j);
				min_eigen_idx = j;
			}
		}
		//ax + by + cz + d = 0
		Eigen::Vector3f coeff;
		coeff.x() = eigen_vector(0, min_eigen_idx);
		coeff.y() = eigen_vector(1, min_eigen_idx);
		coeff.z() = eigen_vector(2, min_eigen_idx);
		float d = -(coeff.x() * mean.x() + coeff.y() * mean.y() + coeff.z() * mean.z());
		coeff_vec.emplace_back(coeff);
		d_vec.emplace_back(d);
	}
}

void PointGrowAngleDis::getPlaneInfo(std::vector<int>& cluster, Eigen::Vector3f & boxMin, Eigen::Vector3f & boxMax, Eigen::Vector3f & coeff, float & d)
{
	int num_vers = cluster.size();
	Eigen::Matrix3f covariance_matrix = Eigen::Matrix3f::Zero(3, 3);
	Eigen::Vector3f mean;
	for (int j = 0; j < num_vers; j++) {
		int idx = cluster[j];
		mean(0) += pointData.pts[idx].x;
		mean(1) += pointData.pts[idx].y;
		mean(2) += pointData.pts[idx].z;
		if (pointData.pts[idx].x < boxMin(0))
			boxMin(0) = pointData.pts[idx].x;
		if (pointData.pts[idx].y < boxMin(1))
			boxMin(1) = pointData.pts[idx].y;
		if (pointData.pts[idx].z < boxMin(2))
			boxMin(2) = pointData.pts[idx].z;
		if (pointData.pts[idx].x > boxMax(0))
			boxMax(0) = pointData.pts[idx].x;
		if (pointData.pts[idx].y > boxMax(1))
			boxMax(1) = pointData.pts[idx].y;
		if (pointData.pts[idx].z > boxMax(2))
			boxMax(2) = pointData.pts[idx].z;
	}
	if (num_vers != 0) {
		mean = mean / num_vers;
	}
	
	for (int j = 0; j < num_vers; ++j)
	{
		//
		int idx = cluster[j];
		Eigen::Vector3f vertex(pointData.pts[idx].x, pointData.pts[idx].y, pointData.pts[idx].z);
		Eigen::Matrix3f multiplied_matrix = (vertex - mean)*(vertex - mean).transpose();
		//std::cout << "temp matrix: \n" << temp_matrix << std::endl;
		covariance_matrix = covariance_matrix + multiplied_matrix;
	}

	covariance_matrix = covariance_matrix / num_vers;

	Eigen::Matrix3f eigen_values = Eigen::Matrix3f::Zero(3, 3);
	Eigen::Matrix3f eigen_vector = Eigen::Matrix3f::Zero(3, 3);
	Eigen::EigenSolver<Eigen::Matrix3f> es(covariance_matrix);
	eigen_values = es.pseudoEigenvalueMatrix();
	eigen_vector = es.pseudoEigenvectors();
	int min_eigen_idx = -1;
	float min_value = FLT_MAX;

	for (int j = 0; j < 3; ++j)
	{
		if (min_value > eigen_values(j, j))
		{
			min_value = eigen_values(j, j);
			min_eigen_idx = j;
		}
	}
	//ax + by + cz + d = 0
	coeff.x() = eigen_vector(0, min_eigen_idx);
	coeff.y() = eigen_vector(1, min_eigen_idx);
	coeff.z() = eigen_vector(2, min_eigen_idx);
    d = -(coeff.x() * mean.x() + coeff.y() * mean.y() + coeff.z() * mean.z());
}

const std::vector<Eigen::Vector3f>& PointGrowAngleDis::getCoeffs()
{
	return coeff_vec;
}

const std::vector<Eigen::Vector3f>& PointGrowAngleDis::getMeans()
{
	return mean_vec;
}

const std::vector<float>& PointGrowAngleDis::getDs()
{
	return d_vec;
}

const std::vector<Eigen::Vector3f>& PointGrowAngleDis::getBoxMins()
{
	return boxMin_vec;
}

const std::vector<Eigen::Vector3f>& PointGrowAngleDis::getBoxMaxs()
{
	return boxMax_vec;
}

void PointGrowAngleDis::ComputePlaneFromAllPnts(Eigen::Vector3f& center_pnt, Eigen::Vector3f& plane_normal)
{
	std::vector<Eigen::Vector3f> all_pnts;
	for (int i = 0; i < pointData.pts.size(); ++i)
	{
		Eigen::Vector3f tmp_pnt = Eigen::Vector3f(pointData.pts[i].x, pointData.pts[i].y, 
			pointData.pts[i].z);
		all_pnts.emplace_back(tmp_pnt);
	}
	ComputePlaneFromSingleCluterPnts(all_pnts, center_pnt, plane_normal);
}

void PointGrowAngleDis::ComputePlaneFromSingleCluterPnts(const std::vector<Eigen::Vector3f>& pnts,
	Eigen::Vector3f& center_pnt, Eigen::Vector3f& plane_normal)
{
	int num_vers = pnts.size();
	Eigen::Matrix3f covariance_matrix = Eigen::Matrix3f::Zero(3, 3);
	Eigen::Vector3f mean(0.0, 0.0, 0.0);
	for (int j = 0; j < num_vers; j++) {
		mean(0) += pnts[j][0];
		mean(1) += pnts[j][1];
		mean(2) += pnts[j][2];
	}
	if (num_vers != 0) {
		mean = mean / num_vers;
	}

	for (int j = 0; j < num_vers; ++j)
	{
		//int idx = cluster[j];
		Eigen::Vector3f vertex(pnts[j][0], pnts[j][1], pnts[j][2]);
		Eigen::Matrix3f multiplied_matrix = (vertex - mean)*(vertex - mean).transpose();
		//std::cout << "temp matrix: \n" << temp_matrix << std::endl;
		covariance_matrix = covariance_matrix + multiplied_matrix;
	}
	covariance_matrix = covariance_matrix / num_vers;
	Eigen::Matrix3f eigen_values = Eigen::Matrix3f::Zero(3, 3);
	Eigen::Matrix3f eigen_vector = Eigen::Matrix3f::Zero(3, 3);
	Eigen::EigenSolver<Eigen::Matrix3f> es(covariance_matrix);
	eigen_values = es.pseudoEigenvalueMatrix();
	eigen_vector = es.pseudoEigenvectors();
	int min_eigen_idx = -1;
	float min_value = FLT_MAX;
	//some thing wrong, to do next...
	for (int j = 0; j < 3; ++j)
	{
		if (min_value > eigen_values(j, j))
		{
			min_value = eigen_values(j, j);
			min_eigen_idx = j;
		}
	}
	center_pnt = mean;

	//ax + by + cz + d = 0
	plane_normal[0] = eigen_vector(0, min_eigen_idx);
	plane_normal[1] = eigen_vector(1, min_eigen_idx);
	plane_normal[2] = eigen_vector(2, min_eigen_idx);
	//d = -(coeff.x() * mean.x() + coeff.y() * mean.y() + coeff.z() * mean.z());
}

void PointGrowAngleDis::ComputePlaneFromSingleClusterIdx(const std::vector<int>& cluster,
	Eigen::Vector3f& center_pnt, Eigen::Vector3f& plane_normal)
{
	int num_vers = static_cast<int>(cluster.size());
	Eigen::Matrix3f covariance_matrix = Eigen::Matrix3f::Zero(3, 3);
	Eigen::Vector3f mean(0.0, 0.0, 0.0);
	for (int j = 0; j < num_vers; j++) {
		int idx = cluster[j];
		mean(0) += pointData.pts[idx].x;
		mean(1) += pointData.pts[idx].y;
		mean(2) += pointData.pts[idx].z;
	}
	if (num_vers != 0) {
		mean = mean / num_vers;
	}
	for (int j = 0; j < num_vers; ++j)
	{
		int idx = cluster[j];
		Eigen::Vector3f vertex(pointData.pts[idx].x, pointData.pts[idx].y, pointData.pts[idx].z);
		Eigen::Matrix3f multiplied_matrix = (vertex - mean)*(vertex - mean).transpose();
		//std::cout << "temp matrix: \n" << temp_matrix << std::endl;
		covariance_matrix = covariance_matrix + multiplied_matrix;
	}
	covariance_matrix = covariance_matrix / num_vers;
	Eigen::Matrix3f eigen_values = Eigen::Matrix3f::Zero(3, 3);
	Eigen::Matrix3f eigen_vector = Eigen::Matrix3f::Zero(3, 3);
	Eigen::EigenSolver<Eigen::Matrix3f> es(covariance_matrix);
	eigen_values = es.pseudoEigenvalueMatrix();
	eigen_vector = es.pseudoEigenvectors();
	int min_eigen_idx = -1;
	float min_value = FLT_MAX;
	//some thing wrong...
	for (int j = 0; j < 3; ++j)
	{
		if (min_value > eigen_values(j, j))
		{
			min_value = eigen_values(j, j);
			min_eigen_idx = j;
		}
	}
	//ax + by + cz + d = 0
	plane_normal[0] = eigen_vector(0, min_eigen_idx);
	plane_normal[1] = eigen_vector(1, min_eigen_idx);
	plane_normal[2] = eigen_vector(2, min_eigen_idx);
	center_pnt = mean;
}

float PointGrowAngleDis::GetAllEnergyFromPlanes()
{
	return 1.0;
}

float PointGrowAngleDis::GetSingleEnergyFromPlanes(int idx)
{
	return 1.0;
}

float PointGrowAngleDis::GetUfEnergyValueFromSinglePlaneFunction(const std::vector<Eigen::Vector3f>& pnts,
	const Eigen::Vector3f& center_pnt, const Eigen::Vector3f& plane_normal)
{
	if (pnts.empty())
	{
		return std::numeric_limits<float>::max();
	}
	int num_vers = static_cast<int>(pnts.size());
	float sum_value = 0.0;
	for (int i = 0; i < pnts.size(); ++i)
	{
		float tmp_dist = HW::Pnt3d2Plane3DCNDist(pnts[i], center_pnt, plane_normal);
		sum_value += tmp_dist;
	}
	return sum_value;
}

float PointGrowAngleDis::GetUfEnergyValueFromSinglePlaneIdxs(const std::vector<int>& cluster_idxs,
	const Eigen::Vector3f& center_pnt, const Eigen::Vector3f& plane_normal)
{
	if (cluster_idxs.empty())
	{
		return std::numeric_limits<float>::max();
	}
	std::vector<Eigen::Vector3f> pnts;
	for (int i = 0; i < cluster_idxs.size(); ++i)
	{
		int idx = cluster_idxs[i];
		Eigen::Vector3f tmp_pnt = Eigen::Vector3f(pointData.pts[idx].x, 
			pointData.pts[idx].y, pointData.pts[idx].z);
		pnts.emplace_back(tmp_pnt);
	}
	float sum_value = GetUfEnergyValueFromSinglePlaneFunction(pnts, center_pnt, plane_normal);
	return sum_value;
}

void PointGrowAngleDis::UpdatePlanesFromPointsRemoved()
{

}
