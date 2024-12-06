//  Add point-to-plane distance to constrict the angle-only point growing algorithm

#ifndef _POINT_GROW_ANGLE_DIS_H_
#define _POINT_GROW_ANGLE_DIS_H_
#pragma once

#include "PCAFunctions.h"
#include "opencv/cv.h"
#include "Eigen/Dense"

class PointGrowAngleDis 
{
public:

	typedef struct PlaneFuncInfo
	{
		Eigen::Vector3f center_pnt_;
		Eigen::Vector3f plane_normal_;
	}PlaneFuncInfo;

	PointGrowAngleDis( float theta, int Rmin );
	~PointGrowAngleDis();

	void run( std::vector<std::vector<int> > &clusters, std::vector<int> &cluster_removed);

	void runGrowSegmentsFromSeeds(std::vector<std::vector<int> >& clusters, std::vector<int>& cluster_removed);
	//split one polygon pnt cloud into two polygon cloud
	void runSingleOriginalPlaneGrowSegmentsFromTwoSeeds(std::vector<std::vector<int> >& clusters, std::vector<int>& cluster_removed);
	void runSingleOriginalPlaneGrowSegmentsFromTwoSeedsNew(std::vector<std::vector<int> >& clusters, std::vector<int>& cluster_removed);

	//void runInitialPntsSegmentsFromTwoSeedsByGrowth(std::vector<int>& cluster_idxs0, std::vector<int>& cluster_idxs1);
	void runGrowPntsIdxFromSingleIdxRecursive(int seed_idx, std::vector<bool>& used_idxs, int k,
		const std::vector<int>& all_pnts_labels, std::vector<int>& grow_pnts_idx);
	void runGrowPntsIdxFromSingleIdx(int seed_idx, int k, const std::vector<int>& all_pnts_labels, 
		std::vector<int>& grow_pnts_idx);
	void runGrowPntsIdxFromSingleIdxWithLabel(int seed_idx, int k, const std::vector<int>& all_pnts_labels,
		std::vector<int>& grow_pnts_idx);
	void runGrowPntsIdxFromSelectedIdxsWithLabel(const std::vector<int>& origin_idxs, int k, int idx_label,
		const std::vector<int>& all_pnts_labels, std::vector<int>& grow_pnts_idx);

	void setData(PtCloud<float> &data, std::vector<PCAInfo> &pcaInfos);
	void SetInitialPnt2PlaneThreshold(const float& intial_threshold);
	void SetSinglePolygonFlag(bool flag);
	void SetGrowPointsSeeds(const std::vector<Eigen::Vector3f>& pnts_seeds);
	void SetGrowPontsSeedsIdxs(const std::vector<int>& seed_idxs);
	void SetAllPntsPlaneCeoff(const Eigen::Vector3f& center_pnt, const Eigen::Vector3f& plane_normal);

	float meadian( std::vector<float> &dataset );

	void getPlaneInfo(std::vector<std::vector<int> > &clusters);
	void getPlaneInfo(std::vector<int> &cluster, Eigen::Vector3f &boxMin, Eigen::Vector3f &boxMax, Eigen::Vector3f &coeff,float& d);

	const std::vector<Eigen::Vector3f>& getCoeffs();
	const std::vector<Eigen::Vector3f>& getMeans();
	const std::vector<float>& getDs();
	const std::vector<Eigen::Vector3f>& getBoxMins();
	const std::vector<Eigen::Vector3f>& getBoxMaxs();

	void ComputePlaneFromAllPnts(Eigen::Vector3f& center_pnt, Eigen::Vector3f& plane_normal);
	void ComputePlaneFromSingleCluterPnts(const std::vector<Eigen::Vector3f>& pnts, 
		Eigen::Vector3f& center_pnt, Eigen::Vector3f& plane_normal);
	void ComputePlaneFromSingleClusterIdx(const std::vector<int>& cluster,
		Eigen::Vector3f& center_pnt, Eigen::Vector3f& plane_normal);

	float GetAllEnergyFromPlanes();

	float GetSingleEnergyFromPlanes(int idx);

	float GetUfEnergyValueFromSinglePlaneFunction(const std::vector<Eigen::Vector3f>& pnts, 
		const Eigen::Vector3f& center_pnt, const Eigen::Vector3f& plane_normal);

	//float GetRatioPntsConvertCloud();

	float GetUfEnergyValueFromSinglePlaneIdxs(const std::vector<int>& cluster_idxs,
		const Eigen::Vector3f& center_pnt, const Eigen::Vector3f& plane_normal);

	void UpdatePlanesFromPointsRemoved();

private:
	
	bool single_polygon_pnts_flag_;

	float theta; 
	int Rmin;

	int pointNum;
	PtCloud<float> pointData;
	std::vector<PCAInfo> pcaInfos;
	std::vector<Eigen::Vector3f> mean_vec;
	std::vector<Eigen::Vector3f> coeff_vec;

	PlaneFuncInfo all_pnts_plane_;
	float pnt_to_plane_dist_threhold_;

	std::vector<float> d_vec;
	std::vector<Eigen::Vector3f> boxMin_vec;
	std::vector<Eigen::Vector3f> boxMax_vec;
	std::vector<Eigen::Vector3f> points_grow_seed_;

	std::vector<int> points_seed_idxs_;
};

#endif // _POINT_GROW_ANGLE_DIS_H_
