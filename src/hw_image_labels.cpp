#include"hw_image_labels.h"
#include<fstream>
#include<iostream>
#include"hw_lines_cluster.h"
#include"hw_cmns.h"
#include"hw_algorithms.h"

namespace HW
{
	HWImageLabels::HWImageLabels()
	{
		images_lines_width_ = 5.0;
		//line extract params
		image_lines_rho_ = 1;
		image_lines_theta_ = CV_PI / 180;
		image_lines_threshold_ = 10;
		image_min_line_length_ = 5;
		image_max_line_gap_ = 5;
		lines_combined_angle_threshold_ = 5.0f;
		lines_combined_dist_threshold_ = 3.0f;	//像素距离
	}
	HWImageLabels::~HWImageLabels()
	{

	}
	void HWImageLabels::SetLabelsDir(const std::string& dir)
	{
		labels_dir_ = dir;
	}

	void HWImageLabels::SetImageLineWidth(float lw)
	{
		images_lines_width_ = lw;
	}

	void HWImageLabels::LoadDataFromDir()
	{
		std::cerr << "start to read images labels..." << std::endl;
		if (labels_dir_.empty())
		{
			return;
		}
		std::vector<std::string> files_list;
		files_list = GetFilesListFromDir(labels_dir_);
		for (int i = 0; i < files_list.size(); ++i)
		{
			std::cerr << files_list[i] << std::endl;
			std::string file_list_path = GetLeftSlashPathName(files_list[i]);
			if (file_list_path.find(".png") != std::string::npos ||
				file_list_path.find(".PNG") != std::string::npos ||
				file_list_path.find(".jpg") != std::string::npos ||
				file_list_path.find(".JPG") != std::string::npos ||
				file_list_path.find(".tiff") != std::string::npos ||
				file_list_path.find(".TIFF") != std::string::npos)
			{
				images_paths_.emplace_back(file_list_path);
			}
			if (file_list_path.find(".txt") != std::string::npos)
			{
				labels_path_ = file_list_path;
			}
		}
		std::sort(images_paths_.begin(), images_paths_.end());
		if (!labels_path_.empty())
		{
			ReadImageLabelFromImagesLabelPath(labels_path_);
			//get image label...
			for (int i = 0; i < images_paths_.size(); ++i)
			{
				//it is bgr(opencv),
				cv::Mat tmp_image = cv::imread(images_paths_[i]);
				//cv::namedWindow("show_color");
				//cv::imshow("show_color", tmp_image);
				//cv::waitKey(0);	//check its

				int image_width = tmp_image.cols;
				int image_height = tmp_image.rows;
				cv::Mat tmp_image_label;
				tmp_image_label.create(image_height, image_width, CV_16UC1);
				for (int r = 0; r < tmp_image_label.rows; ++r)
				{
					for (int c = 0; c < tmp_image_label.cols; ++c)
					{
						cv::Vec3b pixel_c = tmp_image.at<cv::Vec3b>(r, c);
						int tr, tg, tb;
						tr = static_cast<int>(pixel_c[2]);
						tg = static_cast<int>(pixel_c[1]);
						tb = static_cast<int>(pixel_c[0]);
						if (tr > 0 || tg > 0 || tg > 0)
						{
							//std::cerr << "color: " << tr << " " << tg
							//	<< " " << tb << std::endl;
							Eigen::Vector3i tmp_rgb = Eigen::Vector3i(tr, tg, tb);
							int tmp_label = GetPlaneLabelFromRGB(tmp_rgb);
							unsigned short int pixel_id = static_cast<int>(tmp_label);	//get from image id map...
							tmp_image_label.at<unsigned short int>(r, c) = pixel_id;
						}
						else
						{
							unsigned short int pixel_id = KMAX_USHORT_VALUE;
							tmp_image_label.at<unsigned short int>(r, c) = pixel_id;
						}
						//check the pixel label...
					}
				}
				images_labels_.emplace_back(tmp_image_label);
			}
		}
		std::cerr << "end read images labels..." << std::endl;

		////test the image labels
		//if (!images_labels_.empty())
		//{
		//	cv::Mat test_label_image = images_labels_[0];
		//	cv::Mat test_rgb_image = RecoverRGBImageFromImageLabels(test_label_image);
		//	std::string test_path = labels_dir_ + "/" + "0_rgb_rev.png";
		//	cv::imwrite(test_path, test_rgb_image);
		//}
		////end test the image labels

		ComputeLabelsRectsFromImagesLabels();
	}

	const std::vector<cv::Mat>& HWImageLabels::GetImagesLabels()
	{
		return images_labels_;
	}

	const std::vector<cv::Mat>& HWImageLabels::GetImagesLabelsIntersactionRect()
	{
		return images_labels_intersections_;
	}

	const std::vector<std::string>& HWImageLabels::GetImagesPaths()
	{
		return images_paths_;
	}

	void HWImageLabels::ComputeLabelsRectsFromImagesLabels()
	{
		if (images_labels_.empty())
		{
			std::cerr << "none images labels..." << std::endl;
			return;
		}
		images_rect_corners_.resize(images_labels_.size());
		images_labels_intersections_.resize(images_labels_.size());
		for (int i = 0; i < images_paths_.size(); ++i)
		{
			cv::Mat c_image = cv::imread(images_paths_[i]);
			cv::Mat canny_image;
			//求边缘
			cv::Canny(c_image, canny_image, 0, 10, 3, true);
#if 0	//test
			if (i == 0)
			{
					std::string test_path = labels_dir_ + "/" + "0_rgb_canny.png";
					cv::imwrite(test_path, canny_image);
			}
#endif
			//get corner from canny images
			ComputeImageRectCornersFromCannyImage(i, c_image, canny_image);
			//return;
		}
	}

	const std::vector<std::vector<HWImageLineRect> >& HWImageLabels::GetHWImagesLineLabelRects()
	{
		return images_rect_corners_;
	}

	void HWImageLabels::ReadImageLabelFromImagesLabelPath(const std::string& path)
	{
		std::ifstream fh(path);
		if (fh.is_open())
		{
			std::string str;
			while (std::getline(fh, str))
			{
				if (str.empty())
					continue;
				std::stringstream ss(str);
				int tmp_r, tmp_g, tmp_b, tmp_label;
				ss >> tmp_r >> tmp_g >> tmp_b >> tmp_label;
				std::cerr << "rgbl: " << tmp_r << " " << tmp_g << " "
					<< tmp_b << " " << tmp_label << std::endl;
				HWRGB2Label tmp_data;
				tmp_data.HWRGB_ = Eigen::Vector3i(tmp_r, tmp_g, tmp_b);
				tmp_data.label_ = tmp_label;
				colors_to_labels_.emplace_back(tmp_data);
			}
		}
	}

	int HWImageLabels::GetPlaneLabelFromRGB(const Eigen::Vector3i in_rgb)
	{
		int idx = -1;
		for (int i = 0; i < colors_to_labels_.size(); ++i)
		{
			Eigen::Vector3i tmp_rgb = colors_to_labels_[i].HWRGB_;
			if (in_rgb == tmp_rgb)
			{
				idx = i;
				break;
			}
		}
		if (idx != -1)
		{
			return colors_to_labels_[idx].label_;
		}
		else
		{
			return KMAX_USHORT_VALUE;
		}
	}

	Eigen::Vector3i HWImageLabels::GetRGBFromPlaneLabel(int in_label)
	{
		int idx = -1;
		for (int i = 0; i < colors_to_labels_.size(); ++i)
		{
			int tmp_label = colors_to_labels_[i].label_;
			if (in_label == tmp_label)
			{
				idx = i;
				break;
			}
		}
		if (idx != -1)
		{
			return colors_to_labels_[idx].HWRGB_;
		}
		else
		{
			//black
			return Eigen::Vector3i(0, 0, 0);
		}
	}

	cv::Mat HWImageLabels::RecoverRGBImageFromImageLabels(const cv::Mat& in_image_label)
	{
		cv::Mat rgb_image;
		rgb_image.create(in_image_label.rows, in_image_label.cols, CV_8UC3);
		for (int r = 0; r < in_image_label.rows; ++r)
		{
			for (int c = 0; c < in_image_label.cols; ++c)
			{
				unsigned short int tmp_pixel_label = in_image_label.at<unsigned short int>(r, c);
				int ti_pixel_label = static_cast<int>(tmp_pixel_label);
				Eigen::Vector3i tmp_pixel_rgb = GetRGBFromPlaneLabel(ti_pixel_label);
				/*if (ti_pixel_label != KMAX_USHORT_VALUE)
				{
					std::cerr << "rgbi: " << tmp_pixel_rgb[0] << ", "
						<< tmp_pixel_rgb[1] << ", " << tmp_pixel_rgb[2] << "->" << ti_pixel_label << std::endl;
				}*/
				cv::Vec3b tmp_pbgr((uchar)tmp_pixel_rgb[2], 
					(uchar)tmp_pixel_rgb[1], (uchar)tmp_pixel_rgb[0]);
				rgb_image.at<cv::Vec3b>(r, c) = tmp_pbgr;
			}
		}
		return rgb_image;
	}

	void HWImageLabels::ComputeImageRectCornersFromCannyImage(int idx, const cv::Mat& c_img,
		const cv::Mat& cnny_img)
	{
		if (idx < 0 || idx >= images_rect_corners_.size())
		{

			return;
		}
		cv::Mat lines_image = cnny_img.clone();
		std::cerr <<"lines image type: " << lines_image.type() << std::endl;
		//进行霍夫线变换
		std::vector<cv::Vec4i> image_lines;//定义一个矢量结构lines用于存放得到的线段矢量集合
		cv::HoughLinesP(lines_image, image_lines, image_lines_rho_, image_lines_theta_, image_lines_threshold_,
			image_min_line_length_, image_max_line_gap_);

		/*for (int i = 0; i < image_lines.size(); ++i)
		{
			cv::Point2i tmp_s = cv::Point2i(image_lines[i][0], image_lines[i][1]);
			cv::Point2i tmp_e = cv::Point2i(image_lines[i][2], image_lines[i][3]);
			cv::circle(lines_image, tmp_s, 4, cv::Scalar(255, 0, 0), 2);
			cv::circle(lines_image, tmp_e, 4, cv::Scalar(0, 0, 255), 2);
			cv::line(lines_image, tmp_s, tmp_e, cv::Scalar(255, 255, 0), 3);
		}
		std::string test_path = labels_dir_ + "/" + "0_line_image.png";
		cv::imwrite(test_path, lines_image);*/

		/*cv::Mat tmp_c_img = c_img.clone();
		for (int i = 0; i < image_lines.size(); ++i)
		{
			cv::Point2i tmp_s = cv::Point2i(image_lines[i][0], image_lines[i][1]);
			cv::Point2i tmp_e = cv::Point2i(image_lines[i][2], image_lines[i][3]);
			cv::circle(tmp_c_img, tmp_s, 4, cv::Scalar(255, 0, 0), 2);
			cv::circle(tmp_c_img, tmp_e, 4, cv::Scalar(0, 0, 255), 2);
			cv::line(tmp_c_img, tmp_s, tmp_e, cv::Scalar(255, 255, 0), 3);
		}
		std::string test_path = labels_dir_ + "/" + "0_line_color_image.png";
		cv::imwrite(test_path, tmp_c_img);*/

		std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f> >lines_groups_pnts;
		GroupLinesSegmentsBasedOnAngleDistThreshold(image_lines, lines_groups_pnts);

		/*for (int i = 0; i < lines_groups_pnts.size(); ++i)
		{
			cv::Point2i tmp_s = cv::Point2i(lines_groups_pnts[i].first[0], lines_groups_pnts[i].first[1]);
			cv::Point2i tmp_e = cv::Point2i(lines_groups_pnts[i].second[0], lines_groups_pnts[i].second[1]);
			cv::circle(c_img, tmp_s, 4, cv::Scalar(255, 0, 0), 2);
			cv::circle(c_img, tmp_e, 4, cv::Scalar(0, 0, 255), 2);
			cv::line(c_img, tmp_s, tmp_e, cv::Scalar(255, 255, 0), 3);
		}
		std::string test_combine_path = labels_dir_ + "/" + "0_line_combine_color_image.png";
		cv::imwrite(test_combine_path, c_img);*/

		//group the lines based on plane label
		cv::Mat labels_image = images_labels_[idx];
		std::vector<HWImageLineRect> image_lines_rects;
		std::vector<std::vector<Eigen::Vector2i> > lines_sample_pnts;
		//sample the image from ls and le
		for (int i = 0; i < lines_groups_pnts.size(); ++i)
		{
			std::vector<Eigen::Vector2i> line_sample_pnts;
			SamplePntsFromLineEndPnts2D(lines_groups_pnts[i].first, lines_groups_pnts[i].second, line_sample_pnts);
			//test
			lines_sample_pnts.emplace_back(line_sample_pnts);
			//end test
			std::vector<unsigned short int> line_labels_vec;	//single line lables vec
			for (int j = 0; j < line_sample_pnts.size(); j = j + (images_lines_width_ / 2))
			{
				Eigen::Vector2i cur_pnt = line_sample_pnts[j];
				if (cur_pnt[1] < 0 || cur_pnt[1] >= labels_image.rows
					|| cur_pnt[0] < 0 || cur_pnt[0] >= labels_image.cols)
				{
					continue;
				}
				unsigned short int cur_label = labels_image.at<unsigned short int>(cur_pnt[1], cur_pnt[0]);
				line_labels_vec.emplace_back(cur_label);
				//get the patch images_lines_width_*images_lines_width_, and set the patch
				for (int r = -images_lines_width_ / 2; r <= images_lines_width_ / 2; ++r)
				{
					for (int c = -images_lines_width_ / 2; c <= images_lines_width_ / 2; ++c)
					{
						Eigen::Vector2i cur_pnt_neighbor = Eigen::Vector2i(cur_pnt[0] + c, cur_pnt[1] + r);
						if (cur_pnt_neighbor[1] < 0 || cur_pnt_neighbor[1] >= labels_image.rows
							|| cur_pnt_neighbor[0] < 0 || cur_pnt_neighbor[0] >= labels_image.cols)
						{
							continue;
						}
						unsigned short int cur_label_neighbor = labels_image.at<unsigned short int>(cur_pnt_neighbor[1], cur_pnt_neighbor[0]);
						line_labels_vec.emplace_back(cur_label_neighbor);
					}
				}
			}
			
			std::vector<std::vector<unsigned short int> > line_groups_labels;
			int line_labels_num = static_cast<int>(line_labels_vec.size());
			GroupLinePntsBasedOnImageLabel(line_labels_vec, line_groups_labels);
			std::pair<int, int> line_label_values;
			ExtractImageLabelsFromLinePnts(line_groups_labels, line_label_values);

			HWImageLineRect tmp_image_line_rect;
			tmp_image_line_rect.ls_ = lines_groups_pnts[i].first;
			tmp_image_line_rect.le_ = lines_groups_pnts[i].second;
			tmp_image_line_rect.line_width_ = images_lines_width_;
			tmp_image_line_rect.plane_idx0_ = line_label_values.first;
			tmp_image_line_rect.plane_idx1_ = line_label_values.second;
			std::cerr << "tmp_image_line_rect labels: " << tmp_image_line_rect.plane_idx0_ << ", "
				<< tmp_image_line_rect.plane_idx1_ << std::endl;
			image_lines_rects.emplace_back(tmp_image_line_rect);
		}

		////test
		////lines_sample_pnts
		//for (int i = 0; i < lines_sample_pnts.size(); ++i)
		//{
		//	for (int j = 0; j < lines_sample_pnts[i].size(); ++j)
		//	{
		//		cv::Point2i tmp_s = cv::Point2i(lines_sample_pnts[i][j][0], lines_sample_pnts[i][j][1]);
		//		cv::circle(c_img, tmp_s, 4, cv::Scalar(255, 0, 0), 2);
		//	}
		//}
		//std::string test_sample_path = labels_dir_ + "/" + "0_line_sample_pnts_color_image.png";
		//cv::imwrite(test_sample_path, c_img);
		////test

		std::vector<HWImageLineRect> image_lines_rects_new;
		for (int i = 0; i < image_lines_rects.size(); ++i)
		{
			if (image_lines_rects[i].plane_idx0_ != KMAX_USHORT_VALUE
				&& image_lines_rects[i].plane_idx1_ != KMAX_USHORT_VALUE)
			{
				image_lines_rects_new.emplace_back(image_lines_rects[i]);
			}
		}

		/*for (int i = 0; i < image_lines_rects_new.size(); ++i)
		{
			cv::Point2i tmp_s = cv::Point2i(image_lines_rects_new[i].ls_[0], image_lines_rects_new[i].ls_[1]);
			cv::Point2i tmp_e = cv::Point2i(image_lines_rects_new[i].le_[0], image_lines_rects_new[i].le_[1]);
			cv::circle(c_img, tmp_s, 4, cv::Scalar(255, 0, 0), 2);
			cv::circle(c_img, tmp_e, 4, cv::Scalar(0, 0, 255), 2);
			cv::line(c_img, tmp_s, tmp_e, cv::Scalar(255, 255, 0), 3);
		}
		std::string test_combine_path = labels_dir_ + "/" + "0_line_filter_valid_color_image.png";
		cv::imwrite(test_combine_path, c_img);*/

		cv::Mat image_intersection = cv::Mat::zeros(c_img.size(), c_img.type());
		for (int i = 0; i < image_lines_rects_new.size(); ++i)
		{
			cv::Point2i tmp_s = cv::Point2i(image_lines_rects_new[i].ls_[0], image_lines_rects_new[i].ls_[1]);
			cv::Point2i tmp_e = cv::Point2i(image_lines_rects_new[i].le_[0], image_lines_rects_new[i].le_[1]);
			cv::line(image_intersection, tmp_s, tmp_e, cv::Scalar(255, 255, 255), images_lines_width_*2);
		}

		//std::string test_combine_new_final_path = labels_dir_ + "/" + "0_line_filter_final_color_image.png";
		//cv::imwrite(test_combine_new_final_path, image_intersection);

		images_rect_corners_[idx] = image_lines_rects_new;
		images_labels_intersections_[idx] = image_intersection.clone();

		//std::vector<std::vector<int> > image_lines_rects_group_idxs;


#if 0
		for (int i = 0; i < lines_image.rows; i++) 
		{
			for (int j = 0; j < lines_image.cols; j++) 
			{
				if (lines_image.at<uchar>(i, j) == 255) 
				{
					cv::Point2i start;
					start.x = j;
					start.y = i;
					all_edge_points.push_back(start);
				}
			}
		}
		std::cout << "start to split all lines into lines vector..." << std::endl;
		std::vector<std::vector<cv::Point2i> > image_lines_pnts;
		std::vector<bool> all_edge_points_visited(all_edge_points.size(), false);
		cv::Mat lines_image_processed = lines_image.clone();
		for (int sidx = 0; sidx < all_edge_points.size(); ++sidx)
		{
			if (all_edge_points_visited[sidx])
			{
				continue;
			}
			std::vector<cv::Point2i> line_pnts;
			cv::Mat cur_lines_image = lines_image_processed.clone();
			//extract lines
			cv::Point2i s_pnt = all_edge_points[sidx];
			line_pnts.emplace_back(s_pnt);
			while (true)
			{
				bool find_next_pnt = false;
				cv::Point2i cur_pnt = s_pnt;
				std::vector<cv::Point2i> high_pri3;
				std::vector<cv::Point2i> low_pri3;
				std::vector<cv::Point2i> high_pri5;
				std::vector<cv::Point2i> low_pri5;
				for (int i = -2; i < 3; ++i)
				{
					for (int j = -2; j < 3; ++j)
					{
						if (i == 0 && j == 0)
						{
							continue;
						}
						cv::Point2i tmp_pnt = cv::Point2i(cur_pnt.x + i, cur_pnt.y + j);
						if (i >= -1 && i <= 1 && j >= -1 && j <= 1)
						{
							if (i == 0 || j == 0)
							{
								high_pri3.emplace_back(tmp_pnt);
							}
							else
							{
								low_pri3.emplace_back(tmp_pnt);
							}
						}
						else
						{
							if ((i >= -1 && i <= 1) || (j >= -1 && j <= 1))
							{
								high_pri5.emplace_back(tmp_pnt);
							}
							else
							{
								low_pri5.emplace_back(tmp_pnt);
							}
						}
					}
				}
				std::vector<cv::Point2i> cur_pnt_neighbors;
				//insert into neighbor pnts
				cur_pnt_neighbors.insert(cur_pnt_neighbors.end, high_pri3.begin(), high_pri3.end());
				cur_pnt_neighbors.insert(cur_pnt_neighbors.end, low_pri3.begin(), low_pri3.end());
				cur_pnt_neighbors.insert(cur_pnt_neighbors.end, high_pri5.begin(), high_pri5.end());
				cur_pnt_neighbors.insert(cur_pnt_neighbors.end, low_pri5.begin(), low_pri5.end());
				//find next pnts from this neighbor pnts
				for (int i = 0; i < cur_pnt_neighbors.size(); ++i)
				{
					int row = cur_pnt_neighbors[i].y;
					int col = cur_pnt_neighbors[i].x;
					if (cur_lines_image.at<uchar>(row, col) == 255)
					{
						//selected.push_back(cur_pnt_neighbors[i]);
						cur_lines_image.at<uchar>(row, col) = 0;
						cur_pnt = cur_pnt_neighbors[i];
						find_next_pnt = true;
						line_pnts.emplace_back(cur_pnt);
						s_pnt = cur_pnt;
						break;
					}
				}

			}
			lines_image_processed = cur_lines_image.clone();
		}
		std::cout << "end split all lines into lines vector..." << std::endl;
#endif
	}

	void HWImageLabels::GroupLinesSegmentsBasedOnAngleDistThreshold(const std::vector<cv::Vec4i> image_lines,
		std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f> >& lines_groups_pnts)
	{
		lines_groups_pnts.clear();
		//combine all the lines
		std::vector<LinePnt2D> lines_segments;
		for (int i = 0; i < image_lines.size(); ++i)
		{
			LinePnt2D line_pnts;
			line_pnts.line_.first = Eigen::Vector2f(image_lines[i][0], image_lines[i][1]);
			line_pnts.line_.second = Eigen::Vector2f(image_lines[i][2], image_lines[i][3]);
			line_pnts.clusterID = hw_none_classifed_id;
			lines_segments.emplace_back(line_pnts);
		}
		int lines_min_support = 2;
		lines_combined_dist_threshold_ = 3.0;
		lines_combined_angle_threshold_ = 5.0;
		bool line2line_dist_state = true;
		LinesHWDBSCAN* lines_clusters = new LinesHWDBSCAN(lines_min_support, lines_combined_dist_threshold_,
			lines_combined_angle_threshold_, lines_segments);
		lines_clusters->SetLine2LineMeasureState(line2line_dist_state);
		int cluster_num = lines_clusters->Run();
		std::vector<LinePnt2D> lines_segments_new = lines_clusters->lines_points_;
		std::cerr << "lines_segments_new num: " << lines_segments_new.size() << std::endl;
		std::vector<std::vector<int> > lines_groups_idxs;
		for (int i = 0; i < lines_segments_new.size(); ++i)
		{
			std::vector<int> line_group_idxs;
			if (lines_segments_new[i].clusterID == hw_classifed_noise_id)
			{
				//printf("%d \n", lines_segments_new[i].clusterID);
				line_group_idxs.emplace_back(i);
				lines_groups_idxs.emplace_back(line_group_idxs);
			}
		}
		for (int i = 1; i < cluster_num; ++i)
		{
			//how to do it?
			std::vector<int> line_group_idxs;
			for (int j = 0; j < lines_segments_new.size(); ++j)
			{
				if (lines_segments_new[j].clusterID == i)
				{
					line_group_idxs.emplace_back(j);
				}
			}
			lines_groups_idxs.emplace_back(line_group_idxs);
		}
		//group lines pnts from lines_groups_idxs 
		for (int i = 0; i < lines_groups_idxs.size(); ++i)
		{
			const std::vector<int> lines_group_idxs = lines_groups_idxs[i];
			std::vector<Eigen::Vector3f> lines_group_pnts;
			for (int j = 0; j < lines_group_idxs.size(); ++j)
			{
				//line group segments
				int line_idx = lines_group_idxs[j];
				Eigen::Vector2f ls2d = lines_segments_new[line_idx].line_.first;
				Eigen::Vector2f le2d = lines_segments_new[line_idx].line_.second;
				Eigen::Vector3f ls3d = Eigen::Vector3f(ls2d[0], ls2d[1], 1.0);
				Eigen::Vector3f le3d = Eigen::Vector3f(le2d[0], le2d[1], 1.0);
				lines_group_pnts.emplace_back(ls3d);
				lines_group_pnts.emplace_back(le3d);
			}
			int lines_group_pnts_num = static_cast<int>(lines_group_pnts.size());
			if (lines_group_pnts_num >= 3)
			{
				std::pair<Eigen::Vector3f, Eigen::Vector3f> line_se;
				//fit the line
				FittingLineLsLePnts3dFromPnts3d3f(lines_group_pnts, line_se.first, line_se.second);
				std::pair<Eigen::Vector2f, Eigen::Vector2f> line_se2d;
				line_se2d.first = Eigen::Vector2f(line_se.first[0], line_se.first[1]);
				line_se2d.second = Eigen::Vector2f(line_se.second[0], line_se.second[1]);
				lines_groups_pnts.emplace_back(line_se2d);
			}
			else
			{
				std::pair<Eigen::Vector2f, Eigen::Vector2f> line_se2d;
				line_se2d.first = Eigen::Vector2f(lines_group_pnts[0][0], lines_group_pnts[0][1]);
				line_se2d.second = Eigen::Vector2f(lines_group_pnts[1][0], lines_group_pnts[1][1]);
				lines_groups_pnts.emplace_back(line_se2d);
			}
		}
	}

	void HWImageLabels::GroupLinePntsBasedOnImageLabel(const std::vector<unsigned short int>& line_labels,
		std::vector<std::vector<unsigned short int> >& line_groups_labels)
	{
		line_groups_labels.clear();
		/*int line_labels_num = static_cast<int>(line_labels.size());
		if (line_labels_num < 1)
		{
			return;
		}
		unsigned short int label_min_value, label_max_value;
		label_min_value = line_labels[0];
		label_max_value = line_labels[0];
		for (int i = 1; i < line_labels.size(); ++i)
		{
			if (label_min_value > line_labels[i])
			{
				label_min_value = line_labels[i];
			}
			if (label_max_value < line_labels[i])
			{
				label_max_value = line_labels[i];
			}
		}*/
		
		std::vector<std::vector<unsigned short int> > tmp_line_group_labels;
		for (int i = 1; i < line_labels.size(); ++i)
		{
			int idx = -1;
			for (int j = 0; j < tmp_line_group_labels.size(); ++j)
			{
				if (tmp_line_group_labels[j][0] == line_labels[i])
				{
					idx = j;
					break;
				}
			}
			if (idx != -1)
			{
				tmp_line_group_labels[idx].emplace_back(line_labels[i]);
			}
			else
			{
				std::vector<unsigned short int> tmp_label_new;
				tmp_label_new.emplace_back(line_labels[i]);
				tmp_line_group_labels.emplace_back(tmp_label_new);
			}
		}

		line_groups_labels = tmp_line_group_labels;
	}

	void HWImageLabels::GroupLinesRectsBasedOnAssignedImagesLabels(const std::vector<HWImageLineRect>& image_lines_rect,
		std::vector<std::vector<int> >& lines_group_idxs)
	{
#if 0
		std::vector<std::vector<HWImageLineRect> > image_line_rect_group_labels;
		for (int i = 0; i < image_lines_rect.size(); ++i)
		{
			std::pair<int, int> tmp_line_label = std::make_pair(image_lines_rect[i].plane_idx0_, image_lines_rect[i].plane_idx1_);
			int idx = -1;
			for (int j = 0; j < image_line_rect_group_labels.size(); ++j)
			{
				if (image_line_rect_group_labels[j][0] == line_labels[i])
				{
					idx = j;
					break;
				}
			}
			if (idx != -1)
			{
				tmp_line_group_labels[idx].emplace_back(line_labels[i]);
			}
			else
			{
				std::vector<unsigned short int> tmp_label_new;
				tmp_label_new.emplace_back(line_labels[i]);
				tmp_line_group_labels.emplace_back(tmp_label_new);
			}
		}
#endif
	}

	void HWImageLabels::ExtractImageLabelsFromLinePnts(const std::vector<std::vector<unsigned short int> >& line_groups_labels,
		std::pair<int, int>& prime_labels)
	{
		unsigned short int line_max0_label = KMAX_USHORT_VALUE;
		unsigned short int line_max1_label = KMAX_USHORT_VALUE;
		int label_num_max = 0;
		int label_num_max_idx = -1;
		for (int i = 0; i < line_groups_labels.size(); ++i)
		{
			int label_num = static_cast<int>(line_groups_labels[i].size());
			if (label_num > label_num_max)
			{
				label_num_max = label_num;
				label_num_max_idx = i;
				line_max0_label = line_groups_labels[i][0];
				break;
			}
		}
		label_num_max = 0;
		//label_num_max_idx = -1;
		for (int i = 0; i < line_groups_labels.size(); ++i)
		{
			if (i == label_num_max_idx)
				continue;
			int label_num = static_cast<int>(line_groups_labels[i].size());
			if (label_num > label_num_max)
			{
				label_num_max = label_num;
				label_num_max_idx = i;
				line_max1_label = line_groups_labels[i][0];
				/*if (line_max1_label == KMAX_USHORT_VALUE)
				{
					std::cerr << "KMAX_USHORT_VALUE" << std::endl;
				}*/
				break;
			}
		}
		prime_labels.first = line_max0_label;
		prime_labels.second = line_max1_label;
	}
}