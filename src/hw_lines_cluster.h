#pragma once
#ifndef HW_LINES_CLUSTER_H
#define HW_LINES_CLUSTER_H
#include"hw_cmns.h"

namespace HW
{
	const int hw_none_classifed_id = -1;
	const int hw_classifed_noise_id = 0;

	typedef struct LinePnt2D
	{
		std::pair<Eigen::Vector2f, Eigen::Vector2f> line_;
		int clusterID = hw_none_classifed_id;  // clustered ID

		LinePnt2D operator = (const LinePnt2D& other)
		{
			this->line_ = other.line_;
			this->clusterID = other.clusterID;
			return *this;
		}
	}LinePnt2D;

	typedef struct LineDistMeasure
	{
		float angle_measure_;
		float dist_measure_;
		bool operator < (const LineDistMeasure& other)
		{
			if ((this->angle_measure_ + this->dist_measure_)
				< (other.angle_measure_ + other.dist_measure_))
			{
				return true;
			}
			return false;
		}
	} LineDistMeasure;

	LineDistMeasure CalculateLineSegs2DDistanceMeasure(const LinePnt2D& line_pointCore, const LinePnt2D& line_pointTarget);
	
	class LinesHWDBSCAN {

	public:
		LinesHWDBSCAN(unsigned int minLines, float eps, float angle_eps,
			std::vector<LinePnt2D> lines) {
			min_lines_size_ = minLines;
			line_epsilon_ = eps;
			line_angle_epsilon_ = angle_eps;
			lines_points_ = lines;
			lines_size_ = lines.size();
			line2line_dist_state_ = false;
		}
		~LinesHWDBSCAN() {}

		int Run();
		void SetLine2LineMeasureState(bool line2line_state);
		std::vector<int> CalculateClusterFromLinesPnts(LinePnt2D point);
		bool ExpandCluster(LinePnt2D& line_points, int clusterID);
		inline LineDistMeasure CalculateDistance(const LinePnt2D& line_pointCore, const LinePnt2D& line_pointTarget);
		inline LineDistMeasure CalculateLineDistance(const LinePnt2D& line_pointCore, const LinePnt2D& line_pointTarget);
		inline LineDistMeasure CalculateLineDistanceMax(const LinePnt2D& line_pointCore, const LinePnt2D& line_pointTarget);

		int GetTotalPointSize() { return lines_size_; }
		int GetMinimumClusterSize() { return min_lines_size_; }
		int GetEpsilonSize() { return line_epsilon_; }

	public:
		std::vector<LinePnt2D> lines_points_;

	private:

		bool ValidateTwoLinesSame(const LinePnt2D& lp1, const LinePnt2D& lp2);
		bool ValidateTwoPntsSame(const Eigen::Vector2f& p1, const Eigen::Vector2f& p2);

		bool line2line_dist_state_ = false;
		unsigned int lines_size_;
		unsigned int min_lines_size_;
		float line_epsilon_;
		float line_angle_epsilon_;
	};
}

#endif