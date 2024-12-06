#include"hw_lines_cluster.h"
#include"hw_algorithms.h"

namespace HW
{
	LineDistMeasure CalculateLineSegs2DDistanceMeasure(const LinePnt2D& line_pointCore, const LinePnt2D& line_pointTarget)
	{
		Eigen::Vector2f src_ls = line_pointCore.line_.first;
		Eigen::Vector2f src_le = line_pointCore.line_.second;
		Eigen::Vector2f src_dir = src_le - src_ls;
		float src_line_length = src_dir.norm();
		src_dir.normalize();
		Eigen::Vector2f tgt_ls = line_pointTarget.line_.first;
		Eigen::Vector2f tgt_le = line_pointTarget.line_.second;
		Eigen::Vector2f tgt_dir = tgt_le - tgt_ls;
		float tgt_line_length = tgt_dir.norm();
		tgt_dir.normalize();
		float st_angle = ComputeAngleFromTwoLinesVector2D(src_dir, tgt_dir);
		if (st_angle > 90.0)
			st_angle = 180.0 - st_angle;
		float s2t_sl_dist, s2t_el_dist, t2s_sl_dist, t2s_el_dist;
		s2t_sl_dist = PntDist2Line2D(src_ls, tgt_ls, tgt_le);
		s2t_el_dist = PntDist2Line2D(src_le, tgt_ls, tgt_le);
		t2s_sl_dist = PntDist2Line2D(tgt_ls, src_ls, src_le);
		t2s_el_dist = PntDist2Line2D(tgt_le, src_ls, src_le);
		float dist_min = (s2t_sl_dist + s2t_el_dist + t2s_sl_dist + t2s_el_dist) / 4.0;
		//overlap lines (no consider...,to do next...)
		LineDistMeasure tmp_measure;
		tmp_measure.angle_measure_ = st_angle;
		tmp_measure.dist_measure_ = dist_min;
		return tmp_measure;
	}

	int LinesHWDBSCAN::Run()
	{
		int clusterID = 1;
		std::vector<LinePnt2D>::iterator iter = lines_points_.begin();
		//int lines_num = static_cast<int>(lines_points_.size());
		//printf("lines icount: %d\n", lines_num);
		int icount = 0; 
		for (; iter != lines_points_.end(); ++iter)
		{
			//// << "icount: " << icount++ << std::endl;
			//icount++;
			//printf("icount: %d", icount);
			if (iter->clusterID == hw_none_classifed_id)
			{
				if (ExpandCluster(*iter, clusterID))
				{
					clusterID += 1;
				}
			}
		}
		return clusterID;
	}

	std::vector<int> LinesHWDBSCAN::CalculateClusterFromLinesPnts(LinePnt2D point)
	{
		int index = 0;
		std::vector<LinePnt2D>::iterator iter;
		std::vector<int> clusterIndex;
		for (iter = lines_points_.begin(); iter != lines_points_.end(); ++iter)
		{
			LineDistMeasure tmp_dist;
			if (line2line_dist_state_)
			{
				tmp_dist = CalculateLineDistanceMax(point, *iter);
				//tmp_dist = CalculateLineDistance(point, *iter);
			}
			else
			{
				tmp_dist = CalculateDistance(point, *iter);
			}
			/*printf("src line -> (%f, %f), (%f, %f): \n", point.line_.first[0], point.line_.first[1], 
				point.line_.second[0], point.line_.second[1]);
			printf("tgt line -> (%f, %f), (%f, %f): \n", iter->line_.first[0], iter->line_.first[1],
				iter->line_.second[0], iter->line_.second[1]);
			printf("tmp_dist angle, dist -> %f, %f: \n", tmp_dist.angle_measure_, tmp_dist.dist_measure_);*/
			if (tmp_dist.angle_measure_ <= line_angle_epsilon_
				&& tmp_dist.dist_measure_ <= line_epsilon_)
			{
				clusterIndex.push_back(index);
			}
			index++;
		}
		return clusterIndex;
	}

	void LinesHWDBSCAN::SetLine2LineMeasureState(bool line2line_state)
	{
		line2line_dist_state_ = line2line_state;
	}

	bool LinesHWDBSCAN::ExpandCluster(LinePnt2D& line_points, int clusterID)
	{
		std::vector<int> clusterSeeds = CalculateClusterFromLinesPnts(line_points);

		if (clusterSeeds.size() < min_lines_size_)
		{
			line_points.clusterID = hw_classifed_noise_id;
			//printf("noise id...\n");
			return false;
		}
		else
		{
			//delete the origin lines
			int index = 0, indexCorePoint = 0;
			std::vector<int>::iterator iterSeeds;
			for (iterSeeds = clusterSeeds.begin(); iterSeeds != clusterSeeds.end(); ++iterSeeds)
			{
				lines_points_.at(*iterSeeds).clusterID = clusterID;
				if (ValidateTwoLinesSame(lines_points_.at(*iterSeeds), line_points))
				{
					indexCorePoint = index;
				}
				++index;
			}
			clusterSeeds.erase(clusterSeeds.begin() + indexCorePoint);
			for (std::vector<int>::size_type i = 0, n = clusterSeeds.size(); i < n; ++i)
			{
				//std::vector<int> clusterNeighors = CalculateClusterFromLinesPnts(lines_points_.at(clusterSeeds[i]));
				std::vector<int> clusterNeighors = CalculateClusterFromLinesPnts(lines_points_[clusterSeeds[i]]);
				//printf("clusterNeighors num: %d\n", clusterNeighors.size());
				//printf("min_lines_size_: %d\n", min_lines_size_);
				if (clusterNeighors.size() >= min_lines_size_)
				{
					std::vector<int>::iterator iterNeighors;
					for (iterNeighors = clusterNeighors.begin(); iterNeighors != clusterNeighors.end(); ++iterNeighors)
					{
						if (lines_points_.at(*iterNeighors).clusterID == hw_none_classifed_id
							|| lines_points_.at(*iterNeighors).clusterID == hw_classifed_noise_id)
						{
							if (lines_points_.at(*iterNeighors).clusterID == hw_none_classifed_id)
							{
								clusterSeeds.push_back(*iterNeighors);
								n = clusterSeeds.size();
							}
							lines_points_.at(*iterNeighors).clusterID = clusterID;
						}
					}
				}
			}

			return true;
		}
	}

	//important...
	inline LineDistMeasure LinesHWDBSCAN::CalculateDistance(const LinePnt2D& line_pointCore, const LinePnt2D& line_pointTarget)
	{
		Eigen::Vector2f src_ls = line_pointCore.line_.first;
		Eigen::Vector2f src_le = line_pointCore.line_.second;
		Eigen::Vector2f src_dir = src_le - src_ls;
		float src_line_length = src_dir.norm();
		src_dir.normalize();
		Eigen::Vector2f tgt_ls = line_pointTarget.line_.first;
		Eigen::Vector2f tgt_le = line_pointTarget.line_.second;
		Eigen::Vector2f tgt_dir = tgt_le - tgt_ls;
		float tgt_line_length = tgt_dir.norm();
		tgt_dir.normalize();
		float st_angle = ComputeAngleFromTwoLinesVector2D(src_dir, tgt_dir);
		if (st_angle > 90.0)
			st_angle = 180.0 - st_angle;
		float sl_dist, el_dist;
		if (src_line_length < tgt_line_length)
		{
			sl_dist = PntDist2LineSegment2D(src_ls, tgt_ls, tgt_le);
			el_dist = PntDist2LineSegment2D(src_le, tgt_ls, tgt_le);
		}
		else
		{
			sl_dist = PntDist2LineSegment2D(tgt_ls, src_ls, src_le);
			el_dist = PntDist2LineSegment2D(tgt_le, src_ls, src_le);
		}
		float dist_min = sl_dist;
		if (el_dist < sl_dist)
		{
			dist_min = el_dist;
		}
		//overlap lines (no consider...,to do next...)
		LineDistMeasure tmp_measure;
		tmp_measure.angle_measure_ = st_angle;
		tmp_measure.dist_measure_ = dist_min;
		return tmp_measure;
	}

	inline LineDistMeasure LinesHWDBSCAN::CalculateLineDistance(const LinePnt2D& line_pointCore, const LinePnt2D& line_pointTarget)
	{
		Eigen::Vector2f src_ls = line_pointCore.line_.first;
		Eigen::Vector2f src_le = line_pointCore.line_.second;
		Eigen::Vector2f src_dir = src_le - src_ls;
		float src_line_length = src_dir.norm();
		src_dir.normalize();
		Eigen::Vector2f tgt_ls = line_pointTarget.line_.first;
		Eigen::Vector2f tgt_le = line_pointTarget.line_.second;
		Eigen::Vector2f tgt_dir = tgt_le - tgt_ls;
		float tgt_line_length = tgt_dir.norm();
		tgt_dir.normalize();
		float st_angle = ComputeAngleFromTwoLinesVector2D(src_dir, tgt_dir);
		if (st_angle > 90.0)
			st_angle = 180.0 - st_angle;
		float sl_dist, el_dist;
		if (src_line_length < tgt_line_length)
		{
			sl_dist = PntDist2Line2D(src_ls, tgt_ls, tgt_le);
			el_dist = PntDist2Line2D(src_le, tgt_ls, tgt_le);
		}
		else
		{
			sl_dist = PntDist2Line2D(tgt_ls, src_ls, src_le);
			el_dist = PntDist2Line2D(tgt_le, src_ls, src_le);
		}
		float dist_max = sl_dist;
		if (el_dist > sl_dist)
		{
			dist_max = el_dist;
		}
		LineDistMeasure tmp_measure;
		tmp_measure.angle_measure_ = st_angle;
		tmp_measure.dist_measure_ = dist_max;
		return tmp_measure;
	}

	inline LineDistMeasure LinesHWDBSCAN::CalculateLineDistanceMax(const LinePnt2D& line_pointCore, const LinePnt2D& line_pointTarget)
	{
		Eigen::Vector2f src_ls = line_pointCore.line_.first;
		Eigen::Vector2f src_le = line_pointCore.line_.second;
		Eigen::Vector2f src_dir = src_le - src_ls;
		float src_line_length = src_dir.norm();
		src_dir.normalize();
		Eigen::Vector2f tgt_ls = line_pointTarget.line_.first;
		Eigen::Vector2f tgt_le = line_pointTarget.line_.second;
		Eigen::Vector2f tgt_dir = tgt_le - tgt_ls;
		float tgt_line_length = tgt_dir.norm();
		tgt_dir.normalize();
		float st_angle = ComputeAngleFromTwoLinesVector2D(src_dir, tgt_dir);
		if (st_angle > 90.0)
			st_angle = 180.0 - st_angle;
		std::vector<float> pnts2line_dists;
		float pnt2dist;
		pnt2dist = PntDist2Line2D(src_ls, tgt_ls, tgt_le);
		pnts2line_dists.emplace_back(pnt2dist);
		pnt2dist = PntDist2Line2D(src_le, tgt_ls, tgt_le);
		pnts2line_dists.emplace_back(pnt2dist);
		pnt2dist = PntDist2Line2D(tgt_ls, src_ls, src_le);
		pnts2line_dists.emplace_back(pnt2dist);
		pnt2dist = PntDist2Line2D(tgt_le, src_ls, src_le);
		pnts2line_dists.emplace_back(pnt2dist);
		float dist_max = pnts2line_dists[0];
		for (int i = 1; i < pnts2line_dists.size(); ++i)
		{
			if (dist_max < pnts2line_dists[i])
			{
				dist_max = pnts2line_dists[i];
			}
		}
		LineDistMeasure tmp_measure;
		tmp_measure.angle_measure_ = st_angle;
		tmp_measure.dist_measure_ = dist_max;
		return tmp_measure;
	}

	bool LinesHWDBSCAN::ValidateTwoLinesSame(const LinePnt2D& lp1, const LinePnt2D& lp2)
	{
		if (ValidateTwoPntsSame(lp1.line_.first, lp2.line_.first)
			&& ValidateTwoPntsSame(lp1.line_.second, lp2.line_.second))
		{
			return true;
		}
		if (ValidateTwoPntsSame(lp1.line_.first, lp2.line_.second)
			&& ValidateTwoPntsSame(lp1.line_.second, lp2.line_.first))
		{
			return true;
		}
		return false;
	}

	bool LinesHWDBSCAN::ValidateTwoPntsSame(const Eigen::Vector2f& p1, const Eigen::Vector2f& p2)
	{
		if (std::abs(p1[0] - p2[0]) < KMIN_FLOAT_THRESHOLD_COARSE
			&& std::abs(p1[1] - p2[1]) < KMIN_FLOAT_THRESHOLD_COARSE)
		{
			return true;
		}
		return false;
	}
}