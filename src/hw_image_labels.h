#pragma once

#ifndef HW_IMAGE_LABELS_H
#define HW_IMAGE_LABELS_H

#include"hw_cmns.h"
#include"opencv2/opencv.hpp"

namespace HW
{
	struct HWRGB2Label 
	{
		Eigen::Vector3i HWRGB_;
		int label_;
	};

	struct HWImageLineRect
	{
		Eigen::Vector2f ls_;
		Eigen::Vector2f le_;
		float line_width_ = 5;

		int plane_idx0_ = -1;
		int plane_idx1_ = -1;
		/*int p0_label_;
		int p1_label_;
		int p2_label_;
		int p3_label_;*/
	};

	class HWImageLabels 
	{

	public:

		HWImageLabels();
		~HWImageLabels();
		void SetLabelsDir(const std::string& dir);
		void SetImageLineWidth(float lw);
		void LoadDataFromDir();
		const std::vector<cv::Mat>& GetImagesLabels();
		const std::vector<cv::Mat>& GetImagesLabelsIntersactionRect();
		const std::vector<std::string>& GetImagesPaths();
		//Get Images Labels
		void ComputeLabelsRectsFromImagesLabels();
		//GetPolygonFromImageNew

		const std::vector<std::vector<HWImageLineRect> >& GetHWImagesLineLabelRects();

		void ReadImageLabelFromImagesLabelPath(const std::string& path);
		int GetPlaneLabelFromRGB(const Eigen::Vector3i in_rgb);
		Eigen::Vector3i GetRGBFromPlaneLabel(int in_label);

	private:

		cv::Mat RecoverRGBImageFromImageLabels(const cv::Mat& in_image_label);
		void ComputeImageRectCornersFromCannyImage(int idx, const cv::Mat& c_img, 
			const cv::Mat& cnny_img);

		void GroupLinesSegmentsBasedOnAngleDistThreshold(const std::vector<cv::Vec4i> image_lines, 
			std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f> >& lines_groups_pnts);

		//image labels store in line_labels. the group its from image labels
		void GroupLinePntsBasedOnImageLabel(const std::vector<unsigned short int>& line_labels, 
			std::vector<std::vector<unsigned short int> >& line_groups_labels);
		void GroupLinesRectsBasedOnAssignedImagesLabels(const std::vector<HWImageLineRect>& image_lines_rect, 
			std::vector<std::vector<int> >& lines_group_idxs);
		void ExtractImageLabelsFromLinePnts(const std::vector<std::vector<unsigned short int> >& line_groups_labels,
			std::pair<int, int>& prime_labels);

		//extract images lines from image:params
		double image_lines_rho_, image_lines_theta_, image_lines_threshold_,
			image_min_line_length_, image_max_line_gap_ = 0;

		//extract images lines
		float lines_combined_angle_threshold_;
		float lines_combined_dist_threshold_;

		std::string labels_dir_;
		std::vector<std::string> images_paths_;
		std::string labels_path_;
		std::vector<HWRGB2Label> colors_to_labels_;
		std::vector<cv::Mat> images_labels_;	//unsigned short int
		std::vector<cv::Mat> images_labels_intersections_;
		std::vector<std::vector<HWImageLineRect> > images_rect_corners_;

		float images_lines_width_;
	};
}

#endif // !HW_IMAGE_LABELS_H


