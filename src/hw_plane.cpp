#include"hw_plane.h"
#include "rply.h"
#include <algorithm>
#include<map>
#include"hw_algorithms.h"
#include"hw_cmns.h"

#define PAINT_CAM_POS 0
//#define TEST_PLANE 1
//#define LIULINGFEI
#define ZDG_DEBUG 1

void print_time(clock_t start_time, clock_t end_time, std::string name) {
	std::cout << name << " time: " << (double)(end_time - start_time) / CLOCKS_PER_SEC << "s" << std::endl;
}

namespace HW
{

#if ZDG_DEBUG
	void print_polygon_pnts2d(std::vector<Eigen::Vector2f>& pnts)
	{
		for (int i = 0; i < pnts.size(); ++i)
		{
			std::cerr << pnts[i][0] << " " << pnts[i][1] << std::endl;
		}
	}

	void print_polygon_pnts_2d_float2(std::vector<float2>& pnts)
	{
		for (int i = 0; i < pnts.size(); ++i)
		{
			std::cerr << pnts[i].x << " " << pnts[i].y << std::endl;
		}
	}

	void print_polygon_idx(std::vector<int>& p_idxs)
	{
		for (int i = 0; i < p_idxs.size(); ++i)
		{
			std::cerr << p_idxs[i] << " ";
		}
		std::cerr << std::endl;
	}

	//const std::string SavePolygonLineDebugDir = "D:/vc_project_new/huawei_data_indoor/thesis_test/scannet_my_data/scene0062_01_opti3/";
	//const std::string SavePolygonLineDebugDir = "D:/vc_project_new/huawei_data_indoor/thesis_test/polygons_lines_test/scannet_room/scene0062_01_0/polygon_opti_test/";
	const std::string SavePolygonLineDebugDir = "D:/vc_project_new/huawei_data_indoor/thesis_test/polygons_lines_test/scannet_room/scene0062_01_0_test/polygon_opti_test/";

#endif

	HWPlane::HWPlane()
	{
		image_height_ = 750;
		image_width_ = 1000;

		uchar3 red = make_uchar3(255, 0, 0);
		uchar3 green = make_uchar3(0, 255, 0);
		uchar3 blue = make_uchar3(0, 0, 255);
		colors_.push_back(red);
		colors_.push_back(green);
		colors_.push_back(blue);

		params_.alpha = 0;
		params_.real_world_length_per_pixel = 0;
		params_.dilate_half_width = 0;
		params_.pixel_to_world_sampling_rate = 0;
		params_.smooth_region_diameter = 0;
		params_.polygon_smoothing_mu = 0;
		params_.max_theta = 0;
		params_.fitting_lines_pnts_number_threshold_ = 0;
		params_.min_corner_points_distance = 0;

		inner_polygon_num_ = 0;
		semantic_label_ = 13;
		r_poly_dist_max_ = 0.02;

		plane_r_tri_len_ = 0.2;
		plane_r_tri_degree_ = 10;
		polygon_extract_opti_iterators_num_ = 6;
		polygon_neighbor_lines_angle_threhold_ = 8;
		edges_pnts_fitting_num_threshold_ = 3;
	}


	HWPlane::HWPlane(ccPlane* plane)
	{
		associated_cc_plane_ = plane;
	}

	//HWPlane::HWPlane(const HWPlane & plane) :
	//	corner_pnts_3d_(plane.corner_pnts_3d_),
	//	//corner_pnts_(plane.corner_pnts_),
	//	pnts_pos_origin_(plane.pnts_pos_origin_),
	//	pnts_pos_(plane.pnts_pos_),
	//	plane_coord_pos_(plane.plane_coord_pos_),
	//	plane_coord_pnts_normal_(plane.plane_coord_pnts_normal_),
	//	world_to_plane_(plane.world_to_plane_),
	//	plane_to_world_(plane.plane_to_world_),
	//	coeff_(plane.coeff_)
	//{
	//}

	bool HWPlane::HasNormals()
	{
		if (!pnts_pos_.empty() && pnts_pos_.size() == pnts_normal_.size())
			return true;
		
		return false;
	}

	bool HWPlane::HasColors()
	{
		if (!pnts_pos_.empty() && pnts_pos_.size() == pnts_color_.size())
			return true;

		return false;
	}

	HWPlane::~HWPlane()
	{}

	void HWPlane::SetPlaneName(std::string obj_name)
	{
		plane_obj_name_ = obj_name;
	}

	void HWPlane::SetPlaneOutPutDir(const std::string& dir_name)
	{
		plane_output_dir_ = dir_name;
	}

	bool HWPlane::Show()
	{
		return false;
	}

	bool HWPlane::Save(std::string& file)
	{
		return false;
	}

	bool HWPlane::SavePly(const std::string& file, const PlyFormat& type)
	{
		bool write_ascii = false;
		if (type == PlyFormat::kAscci)
		{
			write_ascii = true;
		}
		p_ply ply_file = ply_create(file.c_str(),
			write_ascii ? PLY_ASCII : PLY_LITTLE_ENDIAN, NULL, 0, NULL);
		if (!ply_file) {
			printf("Write PLY failed: unable to open file: %s\n", file.c_str());
			return false;
		}
		ply_add_comment(ply_file, "Created by HW3D");
		ply_add_element(ply_file, "vertex",
			static_cast<long>(pnts_pos_.size()));
		ply_add_property(ply_file, "x", PLY_FLOAT, PLY_FLOAT, PLY_FLOAT);
		ply_add_property(ply_file, "y", PLY_FLOAT, PLY_FLOAT, PLY_FLOAT);
		ply_add_property(ply_file, "z", PLY_FLOAT, PLY_FLOAT, PLY_FLOAT);
		if (this->HasNormals()) {
			ply_add_property(ply_file, "nx", PLY_FLOAT, PLY_FLOAT, PLY_FLOAT);
			ply_add_property(ply_file, "ny", PLY_FLOAT, PLY_FLOAT, PLY_FLOAT);
			ply_add_property(ply_file, "nz", PLY_FLOAT, PLY_FLOAT, PLY_FLOAT);
		}
		if (this->HasColors()) {
			//printf()
			ply_add_property(ply_file, "red", PLY_UCHAR, PLY_UCHAR, PLY_UCHAR);
			ply_add_property(ply_file, "green", PLY_UCHAR, PLY_UCHAR, PLY_UCHAR);
			ply_add_property(ply_file, "blue", PLY_UCHAR, PLY_UCHAR, PLY_UCHAR);
			ply_add_property(ply_file, "alpha", PLY_UCHAR, PLY_UCHAR, PLY_UCHAR);
		}
		if (!ply_write_header(ply_file)) {
			printf("Write PLY failed: unable to write header.\n");
			ply_close(ply_file);
			return false;
		}

		for (size_t i = 0; i < pnts_pos_.size(); i++)
		{
			ply_write(ply_file, (float)pnts_pos_[i].x);
			ply_write(ply_file, (float)pnts_pos_[i].y);
			ply_write(ply_file, (float)pnts_pos_[i].z);
			if (this->HasNormals())
			{
				ply_write(ply_file, (float)pnts_normal_[i].x);
				ply_write(ply_file, (float)pnts_normal_[i].y);
				ply_write(ply_file, (float)pnts_normal_[i].z);
			}
			if (this->HasColors())
			{
				ply_write(ply_file, (unsigned char)std::min(255.0, std::max(0.0,
					pnts_color_[i].x * 1.0)));
				ply_write(ply_file, (unsigned char)std::min(255.0, std::max(0.0,
					pnts_color_[i].y * 1.0)));
				ply_write(ply_file, (unsigned char)std::min(255.0, std::max(0.0,
					pnts_color_[i].z * 1.0)));
				ply_write(ply_file, (unsigned char)std::min(255.0, std::max(0.0,
					0.0 * 1.0)));
			}
		}
		ply_close(ply_file);

		return true;
	}

	bool HWPlane::SaveMargedPlanePntsPly(const std::string& file, const PlyFormat& type)
	{
		bool write_ascii = false;
		if (type == PlyFormat::kAscci)
		{
			write_ascii = true;
		}
		p_ply ply_file = ply_create(file.c_str(),
			write_ascii ? PLY_ASCII : PLY_LITTLE_ENDIAN, NULL, 0, NULL);
		if (!ply_file) {
			printf("Write PLY failed: unable to open file: %s\n", file.c_str());
			return false;
		}
		ply_add_comment(ply_file, "Created by HW3D");
		ply_add_element(ply_file, "vertex",
			static_cast<long>(merged_pnts_pos_.size()));
		ply_add_property(ply_file, "x", PLY_FLOAT, PLY_FLOAT, PLY_FLOAT);
		ply_add_property(ply_file, "y", PLY_FLOAT, PLY_FLOAT, PLY_FLOAT);
		ply_add_property(ply_file, "z", PLY_FLOAT, PLY_FLOAT, PLY_FLOAT);
		if (this->HasNormals()) {
			ply_add_property(ply_file, "nx", PLY_FLOAT, PLY_FLOAT, PLY_FLOAT);
			ply_add_property(ply_file, "ny", PLY_FLOAT, PLY_FLOAT, PLY_FLOAT);
			ply_add_property(ply_file, "nz", PLY_FLOAT, PLY_FLOAT, PLY_FLOAT);
		}
		if (this->HasColors()) {
			//printf()
			ply_add_property(ply_file, "red", PLY_UCHAR, PLY_UCHAR, PLY_UCHAR);
			ply_add_property(ply_file, "green", PLY_UCHAR, PLY_UCHAR, PLY_UCHAR);
			ply_add_property(ply_file, "blue", PLY_UCHAR, PLY_UCHAR, PLY_UCHAR);
			ply_add_property(ply_file, "alpha", PLY_UCHAR, PLY_UCHAR, PLY_UCHAR);
		}
		if (!ply_write_header(ply_file)) {
			printf("Write PLY failed: unable to write header.\n");
			ply_close(ply_file);
			return false;
		}

		for (size_t i = 0; i < merged_pnts_pos_.size(); i++)
		{
			ply_write(ply_file, (float)merged_pnts_pos_[i].x);
			ply_write(ply_file, (float)merged_pnts_pos_[i].y);
			ply_write(ply_file, (float)merged_pnts_pos_[i].z);
			if (this->HasNormals())
			{
				ply_write(ply_file, (float)merged_pnts_normal_[i].x);
				ply_write(ply_file, (float)merged_pnts_normal_[i].y);
				ply_write(ply_file, (float)merged_pnts_normal_[i].z);
			}
			if (this->HasColors())
			{
				ply_write(ply_file, (unsigned char)std::min(255.0, std::max(0.0,
					pnts_color_[i].x * 1.0)));
				ply_write(ply_file, (unsigned char)std::min(255.0, std::max(0.0,
					pnts_color_[i].y * 1.0)));
				ply_write(ply_file, (unsigned char)std::min(255.0, std::max(0.0,
					pnts_color_[i].z * 1.0)));
				ply_write(ply_file, (unsigned char)std::min(255.0, std::max(0.0,
					0.0 * 1.0)));
			}
		}
		ply_close(ply_file);

		return true;
	}

	bool HWPlane::ReadPly(const std::string& file)
	{
		return false;
	}

	bool HWPlane::DrawImage(cv::Mat &image, const std::string& file, std::map<cv::Point2i, Point_2d, Cmp> &ptop, Eigen::Vector4d& min_max)
	{
		double length_per_pixel = params_.real_world_length_per_pixel;
		if (length_per_pixel == 0) {
			length_per_pixel = 0.01;
			initial_params_.real_world_length_per_pixel = 0.01;
		}
		std::cout << "length_per_pixel = " << length_per_pixel << std::endl;

		distance_per_pixel_ = length_per_pixel;
		std::cout << "distance_per_pixel_ = " << distance_per_pixel_ << std::endl;

		float width = alpha_edges_x_.y - alpha_edges_x_.x;
		float height = alpha_edges_y_.y - alpha_edges_y_.x;

		int real_w = 0;
		float longer = 0;
		float ratio = 1.0 / length_per_pixel;

		if (width > height) {
			real_w = width / length_per_pixel;
			longer = width;
		}
		else {
			real_w = height / length_per_pixel;
			longer = height;
		}


		//int real_w = plane_height_ / length_per_pixel;
		//double w_divided_by_h = (alpha_edges_x_.y - alpha_edges_x_.x) /
		//						(alpha_edges_y_.y - alpha_edges_y_.x);
		//double ratio = 0;
		//if (w_divided_by_h > 1) ratio = real_w / (alpha_edges_x_.y - alpha_edges_x_.x);
		//else ratio = real_w / (alpha_edges_y_.y - alpha_edges_y_.x);

		ave_pixels_ = ave_alpha_edge_len_ * ratio;
		std::cout << "ave_pixels_ = " << ave_pixels_ << std::endl;
		if (ave_pixels_ < 1) ave_pixels_ = 1;

		edge_size_ = 8 * (2*ave_pixels_+1);
		int w = real_w + edge_size_ * 2;
		std::cout << "image width = " << w << std::endl;
		std::cout << "x width = " << width << std::endl;
		image.create(w, w, CV_8UC1);
		//设置图像处理的长宽
		plane_image_width_ = w;
		plane_image_height_ = w;

		for (int i = 0; i < image.rows; i++) {
			for (int j = 0; j < image.cols; j++) {
				image.at<uchar>(i, j) = 0;
			}
		}

		int line_type = cv::LINE_8;
		int thickness = 1;
		cv::Scalar color(255);

		//std::map<cv::Point, Point_2d, Cmp> ptop;
		std::cout << "border_edges_.size() = " << border_edges_.size() << std::endl;
		//for (int i = 0; i < border_edges_.size(); ++i)
		//{
		//	Point_2d start = border_edges_[i].start();
		//	Point_2d end = border_edges_[i].end();
		//	//cv::Point start_point((start.x() - minx) * ratio + edge,
		//						  //(start.y() - miny) * ratio + edge);
		//	//cv::Point end_point((end.x() - minx) * ratio + edge,
		//						  //(end.y() - miny) * ratio + edge);
		//	cv::Point start_point((start.x() - alpha_edges_x_.x) * ratio + edge_size_,
		//		(start.y() - alpha_edges_y_.x) * ratio + edge_size_);
		//	cv::Point end_point((end.x() - alpha_edges_x_.x) * ratio + edge_size_,
		//		(end.y() - alpha_edges_y_.x) * ratio + edge_size_);
		//	line(image, start_point, end_point, color, thickness, line_type);
		//	ptop.insert(std::make_pair(start_point, start));
		//	ptop.insert(std::make_pair(end_point, end));

		//	//std::cout << start.x() << ", " << start.y() << std::endl;
		//	//std::cout << end.x() << ", " << end.y() << std::endl;
		//	//std::cout << start_point.x << ", " << start_point.y << std::endl;
		//	//std::cout << end_point.x << ", " << end_point.y << std::endl;
		//	//std::cout << std::endl;
		//}

		for (int i = 0; i < plane_coord_pos_.size(); i++) {
			float3 point = plane_coord_pos_[i];
			cv::Point start_point((point.x - alpha_edges_x_.x) * ratio + edge_size_,
				(point.y - alpha_edges_y_.x) * ratio + edge_size_);
			cv::circle(image, start_point, 1, color, -1);
		}

		//cv::imwrite(line_segment_file, image);

		//std::string edge_file = file + "_canny.png";
		//cv::Canny(image, image, 5, 9);
		//cv::imwrite(edge_file, image);


		return true;
	}

	bool HWPlane::SetImageProjCoordParams(bool cloud_flag)
	{
		if (!cloud_flag)
		{
			//获取polygon的长度
			double length_per_pixel = params_.real_world_length_per_pixel;
			if (length_per_pixel == 0) {
				length_per_pixel = 0.01;
				initial_params_.real_world_length_per_pixel = 0.01;
			}
			std::cout << "length_per_pixel = " << length_per_pixel << std::endl;

			distance_per_pixel_ = length_per_pixel;
			std::cout << "distance_per_pixel_ = " << distance_per_pixel_ << std::endl;

			float width = alpha_edges_x_.y - alpha_edges_x_.x;
			float height = alpha_edges_y_.y - alpha_edges_y_.x;

			int real_w = 0;
			float longer = 0;
			float ratio = 1.0 / length_per_pixel;

			if (width > height) {
				real_w = width / length_per_pixel;
				longer = width;
			}
			else {
				real_w = height / length_per_pixel;
				longer = height;
			}
			ave_pixels_ = ave_alpha_edge_len_ * ratio;
			//获取宽度
			std::cout << "ave_pixels_ = " << ave_pixels_ << std::endl;
			if (ave_pixels_ < 1) ave_pixels_ = 1;
			//添加边界长度
			edge_size_ = (4 * ave_pixels_ + 1);
			int w = real_w + edge_size_ * 2;
			std::cout << "image width = " << w << std::endl;
			std::cout << "x width = " << width << std::endl;

			//设置图像处理的长宽
			plane_image_width_ = w;
			plane_image_height_ = w;
			return true;
		}
		return true;
	}

	bool HWPlane::SetPolygonLSParams(bool cloud_flag)
	{
		if (!cloud_flag)
		{
			alpha_edges_x_.x = 1e8;
			alpha_edges_x_.y = -1e8;
			alpha_edges_y_.x = 1e8;
			alpha_edges_y_.y = -1e8;
			double total_length = 0.0;
			double total_x = 0.0;
			double total_y = 0.0;
			for (int i = 0, j = corner_pnts_.size() - 1; i < corner_pnts_.size(); j = i++)
			{
				float2 start = corner_pnts_[j % (corner_pnts_.size())];
				float2 end = corner_pnts_[i % (corner_pnts_.size())];
				Eigen::Vector2f ls = Eigen::Vector2f(start.x, start.y);
				Eigen::Vector2f le = Eigen::Vector2f(end.x, end.y);
				float dist = (le - ls).norm();
				total_length += dist;

				alpha_edges_x_.x = fminf(alpha_edges_x_.x, corner_pnts_[i].x);
				alpha_edges_x_.y = fmaxf(alpha_edges_x_.y, corner_pnts_[i].x);
				alpha_edges_y_.x = fminf(alpha_edges_y_.x, corner_pnts_[i].y);
				alpha_edges_y_.y = fmaxf(alpha_edges_y_.y, corner_pnts_[i].y);

				alpha_edges_x_.x = fminf(alpha_edges_x_.x, corner_pnts_[j].x);
				alpha_edges_x_.y = fmaxf(alpha_edges_x_.y, corner_pnts_[j].x);
				alpha_edges_y_.x = fminf(alpha_edges_y_.x, corner_pnts_[j].y);
				alpha_edges_y_.y = fmaxf(alpha_edges_y_.y, corner_pnts_[j].y);
			}
			
			std::cout << "alpha_edges_x_.x = " << alpha_edges_x_.x << std::endl;
			std::cout << "alpha_edges_x_.y = " << alpha_edges_x_.y << std::endl;
			std::cout << "alpha_edges_y_.x = " << alpha_edges_y_.x << std::endl;
			std::cout << "alpha_edges_y_.y = " << alpha_edges_y_.y << std::endl;
			ave_alpha_edge_len_ = total_length / corner_pnts_.size();
			std::cout << "ave_alpha_edge_len_ = " << ave_alpha_edge_len_ << std::endl;
			return true;
		}
		return true;
	}

	void HWPlane::SetPolyLSParamsAndImgProjFlag(bool flag)
	{
		related_param_flag = flag;
	}

	bool HWPlane::DrawImageFromCornerPnts2d(std::vector<Eigen::Vector2f>& pnts2d, cv::Mat& image, const std::string& file)
	{
		if (pnts2d.size() < 3)
			return false;
		if(image.empty())
			image.create(plane_image_width_, plane_image_height_, CV_8UC1);
		std::cerr << "plane_width_, plane_height_: " << plane_image_width_ << " " << plane_image_height_ << std::endl;
		std::cerr << "alpha_edges_x_.x, alpha_edges_y_.x: " << alpha_edges_x_.x << ", " << alpha_edges_y_.x << std::endl;
		std::cerr << "distance_per_pixel_: " << distance_per_pixel_ << std::endl;
		std::cerr << "edge_size_: " << edge_size_ << std::endl;
		//这是一个
		int line_type = cv::LINE_8;
		int thickness = 3;
		cv::Scalar color(255);
		for (int i = 0, j = pnts2d.size() - 1; i < pnts2d.size(); j = i++)
		{
			Eigen::Vector2i ls = PlanePnt2ImagePixel(pnts2d[j % (pnts2d.size())]);
			Eigen::Vector2i le = PlanePnt2ImagePixel(pnts2d[i % (pnts2d.size())]);
			//std::cerr << i << ": " << ls[0] << " " << ls[1] << ", " << le[0] << " " << le[1] << std::endl;
			cv::Point2i pls(ls[0], ls[1]);
			cv::Point2i ple(le[0], le[1]);
			cv::line(image, pls, ple, color, thickness, line_type);
		}
//#if ZDG_DEBUG
//		if(file.find(".png") != std::string::npos)
//			cv::imwrite(file, image);
//#endif
		return true;
	}

	bool HWPlane::DrawImageFromLineSegPnts2d(std::vector<Eigen::Vector2f>& pnts2d, cv::Mat& image, const std::string& file)
	{
		if (pnts2d.size() != 2)
			return false;
		if (image.empty())
			image.create(plane_width_, plane_height_, CV_8UC1);
		int line_type = cv::LINE_8;
		int thickness = 3;
		cv::Scalar color(255);
		Eigen::Vector2i ls = PlanePnt2ImagePixel(pnts2d[0]);
		Eigen::Vector2i le = PlanePnt2ImagePixel(pnts2d[1]);
		std::cerr << "related: " << ls[0] << " " << ls[1] << ", " << le[0] << " " << le[1] << std::endl;
		cv::Point2i pls(ls[0], ls[1]);
		cv::Point2i ple(le[0], le[1]);
		cv::line(image, pls, ple, color, thickness, line_type);
		return true;
	}

	void HWPlane::LiulingfeiPolygonExtraction(const std::string &file, bool all_steps = false)
	{
		//int w = 1024;
		//int edge = 4 * ave_len_;
		//int w = plane_height_ * 100 + edge;
		//std::cout << "image width = " << w << std::endl;
		//cv::Mat image = cv::Mat::zeros(w, w, CV_8UC1);
		cv::Mat image;
		std::map<cv::Point2i, Point_2d, Cmp> ptop;
		Eigen::Vector4d params;
		DrawImage(image, file, ptop, params);
#if 0
		GetPolygonFromImage(image, ptop, file, params, all_steps);
#else
		GetPolygonFromImageNew(image, ptop, file, params, all_steps);
#endif
	}

	void fill_color(cv::Mat& image, uchar color, cv::Point start_point) {
		using namespace cv;
		uchar back_color = image.at<uchar>(start_point.y, start_point.x);
		std::vector<cv::Point> stack;
		if (back_color != color) {
			stack.push_back(start_point);
		}
		int width = image.cols;
		int height = image.rows;
		while (!stack.empty()) {
			cv::Point& curr = stack.back();
			int x = curr.x, y = curr.y;
			stack.pop_back();
			if (image.at<uchar>(y, x) == back_color) {
				image.at<uchar>(y, x) = color;
				if (y > 0 && image.at<uchar>(y - 1, x) == back_color) stack.push_back(Point(x, y - 1));
				if (x > 0 && image.at<uchar>(y, x - 1) == back_color) stack.push_back(Point(x - 1, y));
				if (y < height - 1 && image.at<uchar>(y + 1, x) == back_color) stack.push_back(Point(x, y + 1));
				if (x < width - 1 && image.at<uchar>(y, x + 1) == back_color) stack.push_back(Point(x + 1, y));
			}
		}
	}


	double get_distance(const cv::Point2i& a, const cv::Point2i& b) {
		double distance;
		distance = powf((a.x - b.x), 2) + powf((a.y - b.y), 2);
		distance = sqrtf(distance);
		return distance;
	}

	void get_line(cv::Mat& image, cv::Point2i start_point, cv::Point2i& pointa, cv::Point2i& pointb) {
		std::vector<cv::Point2i> to_process;
		std::vector<cv::Point2i> line_points;
		to_process.push_back(start_point);
		line_points.push_back(start_point);
		pointa = start_point;
		pointb = start_point;

		//Vec2f ave_grad{ image.at<Vec3f>(start_point)[1],
						//image.at<Vec3f>(start_point)[2] };
		//ave_grad = normalize(ave_grad);

		cv::Vec2f ave_grad{ 0, 0 };


		float threshold = 0;
		while (!to_process.empty()) {
			cv::Point2i curr = to_process.back();
			to_process.pop_back();
			image.at<cv::Vec3f>(curr) = cv::Vec3f{ 0, -128, -128 };
			//cout << curr << "set to -128" << endl;
			for (int i = -1; i < 2; i++) {
				for (int j = -1; j < 2; j++) {
					if (i == 0 && j == 0) continue;
					cv::Point2i q(curr.x + i, curr.y + j);
					//if (line_points.find(q) != line_points.end()) continue;
					cv::Vec3f temp = image.at<cv::Vec3f>(q);
					if ((int)temp[1] == -128 && (int)temp[2] == -128) {
						//cout << "i: " << i << ", j: " << j << ", " << (float)temp[1] << ", " << (float)temp[2] << endl;
						//cout << "continue" << endl;
						continue;
					}
					cv::Vec2f grad{ temp[1], temp[2] };
					grad = normalize(grad);
					//std::cout << "threshold: " << threshold << std::endl;
					if (line_points.size() < 5 || grad.dot(ave_grad) > threshold - 0.1) {
						int num = line_points.size();
						//ave_grad = (ave_grad * num + grad) / (num + 1);
						//threshold = (threshold * num + grad.dot(ave_grad)) / (num + 1);
						ave_grad = (ave_grad * (num - 1) + grad) / (num);
						threshold = (threshold * (num - 1) + grad.dot(ave_grad)) / (num);
						ave_grad = normalize(ave_grad);
						to_process.push_back(q);
						line_points.push_back(q);
						//std::cout << q << std::endl;
					}
					else {
						line_points.push_back(q);
						std::cout << "bad: " << q << std::endl;
						std::cout << "grad = " << grad << std::endl;
						std::cout << "ave_grad = " << ave_grad << std::endl;
						std::cout << "grad.dot(ave_grad) = " << grad.dot(ave_grad) << std::endl;
					}
				}
			}
		}

		bool diagnal = (ave_grad[0] * ave_grad[1] < 0);
		if (diagnal) std::cout << "diagnal" << std::endl;
		else std::cout << "not diagnal" << std::endl;
		if (abs(ave_grad[0]) < abs(ave_grad[1])) {
			std::cout << "horizontal edge" << std::endl;
			for (auto& p : line_points) {
				if (p.x < pointa.x) pointa = p;
				else if (p.x == pointa.x) {
					if (diagnal && p.y < pointa.y) pointa = p;
					else if (!diagnal && p.y > pointa.y) pointa = p;
				}

				if (p.x > pointb.x) pointb = p;
				else if (p.x == pointb.x) {
					if (diagnal && p.y > pointb.y) pointb = p;
					else if (!diagnal && p.y < pointb.y) pointb = p;
				}
			}
		}
		else {
			std::cout << "verticle edge" << std::endl;
			for (auto& p : line_points) {
				if (p.y < pointa.y) pointa = p;
				else if (p.y == pointa.y) {
					if (diagnal && p.x < pointa.x) pointa = p;
					else if (!diagnal && p.x > pointa.x) pointa = p;
				}

				if (p.y > pointb.y) pointb = p;
				else if (p.y == pointb.y) {
					if (diagnal && p.x > pointb.x) pointb = p;
					else if (!diagnal && p.x < pointb.x) pointb = p;
				}
			}
		}

		if (get_distance(pointb, start_point) < get_distance(pointa, start_point)) {
			swap(pointa, pointb);
		}
		//std::cout << "start: " << start_point << std::endl;
		//std::cout << "pointa: " << pointa << std::endl;
		//std::cout << "pointb: " << pointb << std::endl;
		//std::cout << std::endl;
	}

	Eigen::Vector2d HWPlane::image_to_plane(int x, int y) {
		//cv::Point start_point((start.x() - centerx) * ratio + w / 2,
			//(start.y() - centery) * ratio + w / 2);
		double plane_x = (x +0.5- edge_size_) * distance_per_pixel_ + alpha_edges_x_.x;
		double plane_y = (y +0.5- edge_size_) * distance_per_pixel_ + alpha_edges_y_.x;
		return Eigen::Vector2d{ plane_x, plane_y };
	}

	Eigen::Vector2i HWPlane::PlanePnt2ImagePixel(Eigen::Vector2f& pnt2d)
	{
		//
		int pixel_x = (pnt2d[0] - alpha_edges_x_.x) / distance_per_pixel_ + edge_size_ - 0.5;
		int pixel_y = (pnt2d[1] - alpha_edges_y_.x) / distance_per_pixel_ + edge_size_ - 0.5;
		return Eigen::Vector2i(pixel_x, pixel_y);
	}

	std::vector<cv::Point2i> HWPlane::GetEdgeFromImage(cv::Mat& final_edge, std::vector<float2>& edge_points)
	{
		using std::string;
		using std::vector;
		using std::cout;
		using std::endl;
		using namespace cv;

		std::vector<Point2i> all_edge_points;
		for (int i = 0; i < final_edge.rows; i++) {
			for (int j = 0; j < final_edge.cols; j++) {
				if (final_edge.at<uchar>(i, j) == 255) {
					Point2i start;
					start.x = j;
					start.y = i;
					all_edge_points.push_back(start);
				}
			}
		}
		std::cout << "start found" << std::endl;
		//std::cout << start.x << ", " << start.y << std::endl;


		//开始寻找外边缘
		std::vector<Point2i> selected;
		//std::vector<float2> edge_points;
		int count = 0;
		for (int start_index = 0; start_index < all_edge_points.size(); start_index++) {
			Mat final_edge_copy = final_edge.clone();
			//edge_pnts_pos_.clear();
			edge_points.clear();
			selected.clear();
			count = 0;
			Point2i curr = all_edge_points[start_index];
			//1cm采样一个像素点
			//int sample_rate = 0.01 / distance_per_pixel_ + 1;
			int sample_rate = params_.pixel_to_world_sampling_rate;
			if (sample_rate == 0) {
				sample_rate = ave_pixels_ / 2;
				initial_params_.pixel_to_world_sampling_rate = ave_pixels_ / 2;
			}
			std::cout << "sample_rate = " << sample_rate << std::endl;
			if (sample_rate < 1) sample_rate = 1;
			//if (sample_rate > 4) sample_rate = 4;
			std::cout << "plane_width = " << plane_width_ << std::endl;
			std::cout << "sample_rate = " << sample_rate << std::endl;
			while (true) {
				bool found_next = false;
				std::vector<Point2i> high_pri3;
				std::vector<Point2i> low_pri3;
				std::vector<Point2i> high_pri5;
				std::vector<Point2i> low_pri5;
				for (int i = -2; i < 3; i++) {
					for (int j = -2; j < 3; j++) {
						//在3x3范围内
						if (i == 0 && j == 0) continue;
						Point2i temp(curr.x + j, curr.y + i);
						if (i >= -1 && i <= 1 && j >= -1 && j <= 1) {
							if (i == 0 || j == 0) {
								high_pri3.push_back(temp);
							}
							else {
								low_pri3.push_back(temp);
							}
						}
						//不在3x3范围内
						else {
							if ((i >= -1 && i <= 1) || (j >= -1 && j <= 1)) {
								high_pri5.push_back(temp);
							}
							else {
								low_pri5.push_back(temp);
							}
						}
					}
				}
				std::vector<Point2i> pri_vec;
				pri_vec.insert(pri_vec.end(), high_pri3.begin(), high_pri3.end());
				pri_vec.insert(pri_vec.end(), low_pri3.begin(), low_pri3.end());
				pri_vec.insert(pri_vec.end(), high_pri5.begin(), high_pri5.end());
				pri_vec.insert(pri_vec.end(), low_pri5.begin(), low_pri5.end());

				for (auto& p : pri_vec) {
					int row = p.y;
					int col = p.x;
					if (final_edge_copy.at<uchar>(row, col) == 255) {
						if (count % sample_rate == 0) {
							Eigen::Vector2d pnt = image_to_plane(col, row);
							sorted_border_pnts_2d_.emplace_back(Point_2d(pnt(0), pnt(1)));
							//edge_pnts_pos_.push_back(make_float2(pnt(0), pnt(1)));
							edge_points.push_back(make_float2(pnt(0), pnt(1)));
							selected.push_back(p);
							//std::cout << "selected.push_back " << p << std::endl;
						}
						final_edge_copy.at<uchar>(row, col) = 0;
						curr = p;
						found_next = true;
						count++;
						break;
					}
				}
				if (!found_next) {
					break;
				}

				/*
				//FIXME:找到最邻近的点后，扩张的方向可能与原方向相反
				if (!found_next) {
					std::cout << "not found next" << std::endl;
					int dist = 3;
					while (!found_next && dist < ave_pixels_) {
						//std::cout << "dist = " << dist << std::endl;
						int row = curr.x - dist;
						int col = curr.y - dist;

						//上面的行
						for (int i = 0; i < 2 * dist + 1; i++) {
							if (row < 0 || row >= final_edge_copy.rows ||
								col < 0 || col >= final_edge_copy.cols) continue;
							if (final_edge_copy.at<uchar>(row, col) == 255) {
								curr = Point2i(col, row);
								found_next = true;
								break;
							}
							col++;
						}
						//右边的列
						for (int i = 0; i < 2 * dist + 1; i++) {
							if (row < 0 || row >= final_edge_copy.rows ||
								col < 0 || col >= final_edge_copy.cols) continue;
							if (final_edge_copy.at<uchar>(row, col) == 255) {
								curr = Point2i(col, row);
								found_next = true;
								break;
							}
							row++;
						}
						//下面的行
						for (int i = 0; i < 2 * dist + 1; i++) {
							if (row < 0 || row >= final_edge_copy.rows ||
								col < 0 || col >= final_edge_copy.cols) continue;
							if (final_edge_copy.at<uchar>(row, col) == 255) {
								curr = Point2i(col, row);
								found_next = true;
								break;
							}
							col--;
						}
						//左边的列
						for (int i = 0; i < 2 * dist + 1; i++) {
							if (row < 0 || row >= final_edge_copy.rows ||
								col < 0 || col >= final_edge_copy.cols) continue;
							if (final_edge_copy.at<uchar>(row, col) == 255) {
								curr = Point2i(col, row);
								found_next = true;
								break;
							}
							row--;
						}
						dist++;
					}

					if (!found_next) break;
					Eigen::Vector2d pnt = image_to_plane(curr.x, curr.y);
					sorted_border_pnts_2d_.emplace_back(Point_2d(pnt(0), pnt(1)));
					//edge_pnts_pos_.push_back(make_float2(pnt(0), pnt(1)));
					edge_points.push_back(make_float2(pnt(0), pnt(1)));
					selected.push_back(curr);
					final_edge_copy.at<uchar>(curr.y, curr.x) = 0;
					count++;
				}
				*/
			}
			if (count > all_edge_points.size() / 10) {
				final_edge = final_edge_copy.clone();
				break;
			}
		}
		if (count <= all_edge_points.size() / 10 || count < 10) {
			//edge_pnts_pos_.clear();
			edge_points.clear();
			std::cout << "find edge points from image failed" << std::endl;
			return selected;
		}

		//float2 edge1 = edge_pnts_pos_[1] - edge_pnts_pos_[0];
		//float2 edge2 = edge_pnts_pos_[2] - edge_pnts_pos_[1];

		float2 edge1 = edge_points[1] - edge_points[0];
		float2 edge2 = edge_points[2] - edge_points[1];

		//if (edge1.x*edge2.y - edge1.y*edge2.x == 0) {
			//std::cout << "edge1: " << edge1.x << " " << edge1.y << "\n";
			//std::cout << "edge2: " << edge2.x << " " << edge2.y << "\n";
			//system("pause");
		//}

		std::cout << "start inversing" << std::endl;
		//解决定点确定的多边形法向量反向的问题
		//将顶点向量中的点逆序排列
		if (edge1.x * edge2.y - edge1.y * edge2.x > 0 ^ IsConcaveEdgePoint(edge_points, 1) == 1) {
			std::vector<float2> edge_pnts_inverse;
			for (int i = edge_points.size() - 1; i >= 0; i--) {
				edge_pnts_inverse.emplace_back(edge_points[i]);
			}
			edge_points = edge_pnts_inverse;
		}
		std::cout << "edge_points.size() = " << edge_points.size() << std::endl;
		return selected;
	}

	void HWPlane::GetPolygonFromImage(cv::Mat& src, const std::map<cv::Point2i, Point_2d, Cmp> &ptop, const std::string& filename, const Eigen::Vector4d& params, bool all_steps = false)
	{
		using std::string;
		using std::vector;
		using std::cout;
		using std::endl;
		using namespace cv;

		int image_width = src.rows;
#ifdef LIULINGFEI
		imwrite(filename + ".png", src);
#endif
		string input("lines.png");
		string dilate_output("dilate.png");
		string erode_output("erode.png");
		string canny_output("canny.png");

		//Mat src = imread(input);
		//imshow("edge", src);

		//params_.dilate_half_width = ave_pixels_;
		int dilation_size = params_.dilate_half_width;
		if (dilation_size == 0) {
			dilation_size = ave_pixels_;
			initial_params_.dilate_half_width = ave_pixels_;
		}
		std::cout << "dilation_size = " << dilation_size << std::endl;
		Mat dilation_element = getStructuringElement(MORPH_RECT,
			Size(2 * dilation_size + 1, 2 * dilation_size + 1),
			Point(dilation_size, dilation_size));
		dilation_size = 1;
		Mat dilation_element3 = getStructuringElement(MORPH_RECT,
			Size(2 * dilation_size + 1, 2 * dilation_size + 1),
			Point(dilation_size, dilation_size));

		//膨胀
		Mat dst;
		dilate(src, dst, dilation_element);
		dilate(dst, dst, dilation_element);
		dilate(dst, dst, dilation_element);
		for (int i = 0; i < dst.rows; i++) {
			for (int j = 0; j < dst.cols; j++) {
				if (dst.at<uchar>(i, j) < 128) dst.at<uchar>(i, j) = 0;
				else dst.at<uchar>(i, j) = 255;
			}
		}
		//imshow("dilate", dst);
#ifdef LIULINGFEI
		imwrite(filename + dilate_output, dst);
#endif
		//腐蚀
		erode(dst, dst, dilation_element);
		erode(dst, dst, dilation_element);
		erode(dst, dst, dilation_element);
#ifdef LIULINGFEI
		imwrite(filename + erode_output, dst);
#endif

		Mat canny;
		//求边缘
		Canny(dst, canny, 0, 10, 3, true);
		//Mat dst_eroded;
		//erode(dst, dst_eroded, dilation_element3);
		//canny = dst - dst_eroded;

		//膨胀边缘
		//dilate(canny, canny, dilation_element);
		//imwrite(filename + canny_output, canny);


		//int gray = 128;
		////填充外边缘之外的部分
		//fill_color(canny, gray, Point(0, 0));
		////imshow("filled with gray", canny);

		////填充外边缘之内的部分
		//for (int i = 0; i < canny.rows; i++) {
		//	for (int j = 0; j < canny.cols; j++) {
		//		if (canny.at<uchar>(i, j) == gray) canny.at<uchar>(i, j) = 0;
		//		else canny.at<uchar>(i, j) = 255;
		//	}
		//}

		////imshow("reversed", canny);
		//erode(canny, canny, dilation_element);
		//erode(canny, canny, dilation_element);
		//imwrite(filename + "reversed.png", canny);
		//std::cout << "imwrite reversed.png" << std::endl;

		////plane_coord_pos_.clear();
		////for (int i = 0; i < canny.rows; i++) {
		////	for (int j = 0; j < canny.cols; j++) {
		////		if (canny.at<uchar>(i, j) == 255) {
		////			//从图像投影回平面
		////			Eigen::Vector2d pnt = image_to_plane(j, i, params);
		////			plane_coord_pos_.push_back(make_float3(pnt(0), pnt(1), 0));
		////		}
		////	}
		////}
		////std::cout << "从图像投影回平面成功" << std::endl;

		//if (!all_steps)
		//	return;


		////求边缘
		//Mat final_edge;

		////用原来的图像减去腐蚀后的图像，代替canny边缘检测，因为canny检测出的边缘可能不连续
		////Canny(canny, final_edge, 0, 10, 3, true);
		//Mat single_dilated;
		//erode(canny, single_dilated, dilation_element3);
		//final_edge= canny - single_dilated;

		////imshow("final_edge", final_edge);

		Mat final_edge = canny.clone();
#ifdef LIULINGFEI
		imwrite(filename+"final_edge.png", final_edge);
		std::cout << "imwrite final_edge.png" << std::endl;
#endif

		//找出所有边缘像素
		std::vector<Point2i> all_edge_points;
		for (int i = 0; i < final_edge.rows; i++) {
			for (int j = 0; j < final_edge.cols; j++) {
				if (final_edge.at<uchar>(i, j) == 255) {
					Point2i start;
					start.x = j;
					start.y = i;
					all_edge_points.push_back(start);
				}
			}
		}
		std::cout << "start found" << std::endl;
		//std::cout << start.x << ", " << start.y << std::endl;


		//开始寻找外边缘
		std::vector<Point2i> selected;
		std::vector<float2> edge_points;
		int count = 0;
		Mat final_edge_copy;
		for (int start_index = 0; start_index < all_edge_points.size(); start_index++) {
			std::cerr << "the start_index: " << start_index << std::endl;
			Mat final_edge_copy = final_edge.clone();
			//edge_pnts_pos_.clear();
			edge_points.clear();
			selected.clear();
			count = 0;
			Point2i curr = all_edge_points[start_index];
			//1cm采样一个像素点
			//int sample_rate = 0.01 / distance_per_pixel_ + 1;
			int sample_rate = params_.pixel_to_world_sampling_rate;
			if (sample_rate == 0) {
				sample_rate = ave_pixels_ / 2;
				initial_params_.pixel_to_world_sampling_rate = ave_pixels_ / 2;
			}
			std::cout << "sample_rate = " << sample_rate << std::endl;
			if (sample_rate < 1) sample_rate = 1;
			//if (sample_rate > 4) sample_rate = 4;
			std::cout << "plane_width = " << plane_width_ << std::endl;
			std::cout << "sample_rate = " << sample_rate << std::endl;
			while (true) {
				bool found_next = false;
				std::vector<Point2i> high_pri3;
				std::vector<Point2i> low_pri3;
				std::vector<Point2i> high_pri5;
				std::vector<Point2i> low_pri5;
				for (int i = -2; i < 3; i++) {
					for (int j = -2; j < 3; j++) {
						//在3x3范围内
						if (i == 0 && j == 0) continue;
						Point2i temp(curr.x + j, curr.y + i);
						if (i >= -1 && i <= 1 && j >= -1 && j <= 1) {
							if (i == 0 || j == 0) {
								high_pri3.push_back(temp);
							}
							else {
								low_pri3.push_back(temp);
							}
						}
						//不在3x3范围内
						else {
							if ((i >= -1 && i <= 1) || (j >= -1 && j <= 1)) {
								high_pri5.push_back(temp);
							}
							else {
								low_pri5.push_back(temp);
							}
						}
					}
				}
				std::vector<Point2i> pri_vec;
				pri_vec.insert(pri_vec.end(), high_pri3.begin(), high_pri3.end());
				pri_vec.insert(pri_vec.end(), low_pri3.begin(), low_pri3.end());
				pri_vec.insert(pri_vec.end(), high_pri5.begin(), high_pri5.end());
				pri_vec.insert(pri_vec.end(), low_pri5.begin(), low_pri5.end());

				for (auto & p : pri_vec) {
					int row = p.y;
					int col = p.x;
					if (final_edge_copy.at<uchar>(row, col) == 255) {
						if (count % sample_rate == 0) {
							Eigen::Vector2d pnt = image_to_plane(col, row);
							sorted_border_pnts_2d_.emplace_back(Point_2d(pnt(0), pnt(1)));
							//edge_pnts_pos_.push_back(make_float2(pnt(0), pnt(1)));
							edge_points.push_back(make_float2(pnt(0), pnt(1)));
							selected.push_back(p);
							//std::cout << "selected.push_back " << p << std::endl;
						}
						final_edge_copy.at<uchar>(row, col) = 0;
						curr = p;
						found_next = true;
						count++;
						break;
					}
				}

				//FIXME:找到最邻近的点后，扩张的方向可能与原方向相反
				if (!found_next) {
					std::cout << "not found next" << std::endl;
					int dist = 3;
					while (!found_next && dist < ave_pixels_) {
						//std::cout << "dist = " << dist << std::endl;
						int row = curr.x - dist;
						int col = curr.y - dist;

						//上面的行
						for (int i = 0; i < 2 * dist + 1; i++) {
							if (row < 0 || row >= final_edge_copy.rows ||
								col < 0 || col >= final_edge_copy.cols) continue;
							if (final_edge_copy.at<uchar>(row, col) == 255) {
								curr = Point2i(col, row);
								found_next = true;
								break;
							}
							col++;
						}
						//右边的列
						for (int i = 0; i < 2 * dist + 1; i++) {
							if (row < 0 || row >= final_edge_copy.rows ||
								col < 0 || col >= final_edge_copy.cols) continue;
							if (final_edge_copy.at<uchar>(row, col) == 255) {
								curr = Point2i(col, row);
								found_next = true;
								break;
							}
							row++;
						}
						//下面的行
						for (int i = 0; i < 2 * dist + 1; i++) {
							if (row < 0 || row >= final_edge_copy.rows ||
								col < 0 || col >= final_edge_copy.cols) continue;
							if (final_edge_copy.at<uchar>(row, col) == 255) {
								curr = Point2i(col, row);
								found_next = true;
								break;
							}
							col--;
						}
						//左边的列
						for (int i = 0; i < 2 * dist + 1; i++) {
							if (row < 0 || row >= final_edge_copy.rows ||
								col < 0 || col >= final_edge_copy.cols) continue;
							if (final_edge_copy.at<uchar>(row, col) == 255) {
								curr = Point2i(col, row);
								found_next = true;
								break;
							}
							row--;
						}
						dist++;
					}

					if (!found_next) break;
					Eigen::Vector2d pnt = image_to_plane(curr.x, curr.y);
					sorted_border_pnts_2d_.emplace_back(Point_2d(pnt(0), pnt(1)));
					//edge_pnts_pos_.push_back(make_float2(pnt(0), pnt(1)));
					edge_points.push_back(make_float2(pnt(0), pnt(1)));
					selected.push_back(curr);
					final_edge_copy.at<uchar>(curr.y, curr.x) = 0;
					count++;
				}
			}
			std::cout << "the count....... " << count << std::endl;
			if (count > all_edge_points.size() / 4) {
				final_edge = final_edge_copy.clone();
				break;
			}
		}
		if (count <= all_edge_points.size() / 4) {
			//edge_pnts_pos_.clear();
			edge_points.clear();
			std::cout << "find edge points from image failed" << std::endl;
			return;
		}

		//float2 edge1 = edge_pnts_pos_[1] - edge_pnts_pos_[0];
		//float2 edge2 = edge_pnts_pos_[2] - edge_pnts_pos_[1];

		float2 edge1 = edge_points[1] - edge_points[0];
		float2 edge2 = edge_points[2] - edge_points[1];

		//if (edge1.x*edge2.y - edge1.y*edge2.x == 0) {
			//std::cout << "edge1: " << edge1.x << " " << edge1.y << "\n";
			//std::cout << "edge2: " << edge2.x << " " << edge2.y << "\n";
			//system("pause");
		//}

		std::cout << "start inversing" << std::endl;
		//解决定点确定的多边形法向量反向的问题
		//将顶点向量中的点逆序排列
		if (edge1.x*edge2.y - edge1.y*edge2.x > 0 ^ IsConcaveEdgePoint(edge_points, 1) == 1) {
			std::vector<float2> edge_pnts_inverse;
			for (int i = edge_points.size() - 1; i >= 0; i--) {
				edge_pnts_inverse.emplace_back(edge_points[i]);
			}
			edge_points = edge_pnts_inverse;
		}

		edge_pnts_pos_ = edge_points;
		edge_pnts_pos_num_ = edge_pnts_pos_.size();
        std:cout << "selected_edge_points.size(): " << edge_pnts_pos_num_ << std::endl;
#ifdef LIULINGFEI
		imwrite(filename+"final_edge_left.png", final_edge);
#endif

		std::vector<std::vector<float2>> inner_edges;
		//开始寻找内边缘
		int index = 0;
		std::cout << "inner edges:" << std::endl;
		while (true) {
			std::stringstream ss;
			ss << index;
			std::string index_str;
			ss >> index_str;

			std::vector<float2> inner_edge;
			Mat temp = final_edge.clone();
			std::vector<Point2i> selected_pixels = GetEdgeFromImage(final_edge, inner_edge);
			if (inner_edge.empty()) break;
			inner_edges.push_back(inner_edge);
			inner_edge_pnts_pos_num_.push_back(inner_edge.size());
			std::cout << inner_edge.size() << std::endl;
#ifdef LIULINGFEI
			imwrite(filename + "inner_edge_" + index_str + ".png", temp - final_edge);
#endif
			selected.insert(selected.end(), selected_pixels.begin(), selected_pixels.end());
			index++;
		}
		inner_edge_pnts_pos_ = inner_edges;
		inner_polygon_num_ = inner_edge_pnts_pos_.size();

		//开关
		inner_edge_pnts_pos_.clear();
		inner_polygon_num_ = 0;

		//写入平面空间的边缘点和边缘
#ifdef OUTPUT_IMAGE_EDGES
		std::ofstream fh(filename + "image_edges.obj");
		for (int i = 0; i < edge_pnts_pos_.size(); i++)
		{
			float2 start_pnt = edge_pnts_pos_[i];
			fh << "v " << start_pnt.x << " " << start_pnt.y << " " << "0.0" << std::endl;
		}
		for (int ii = 0; ii < edge_pnts_pos_.size()- 1; ii++) {
			fh << "l " << ii + 1 << " " << ii + 2 << "\n";
		}
		fh << "l " << 1 << " " << edge_pnts_pos_.size() << "\n";
		fh.close();

		Mat selected_edge_points(image_width, image_width, CV_8UC3, Scalar(0, 0, 0));
		for (int i = 0; i < selected.size() - 1; i++) {
			circle(selected_edge_points, selected[i], 1, Scalar(0, 255, 0));
			line(selected_edge_points, selected[i], selected[i + 1], Scalar(255, 255, 255), 2);
		}
		circle(selected_edge_points, selected[selected.size() - 1], 1, Scalar(0, 255, 0));
		line(selected_edge_points, selected[0], selected[selected.size() - 1], Scalar(255, 255, 255), 2);
#ifdef LIULINGFEI
		imwrite(filename+"selected_edge_points.png", selected_edge_points);
#endif
		//写入三维空间的边缘点和边缘
		std::vector<float3> points_3d;
		for (int i = 0; i < edge_pnts_pos_.size(); i++) {
			Eigen::Vector4f pnt(edge_pnts_pos_[i].x, edge_pnts_pos_[i].y, 0.0, 1.0);
			Eigen::Vector4f transform_pnt = plane_to_world_ * pnt;
			points_3d.emplace_back(make_float3(transform_pnt(0), transform_pnt(1), transform_pnt(2)));
		}

		fh.open(filename + "image_edges_3d.obj");
		for (int i = 0; i < points_3d.size(); i++)
		{
			float3 start_pnt = points_3d[i];
			fh << "v " << start_pnt.x << " " << start_pnt.y << " " << start_pnt.z << std::endl;
		}
		for (int ii = 0; ii < points_3d.size() - 1; ii++) {
			fh << "l " << ii + 1 << " " << ii + 2 << "\n";
		}
		fh << "l " << 1 << " " << points_3d.size() << "\n";
		fh.close();
#endif //OUTPUT_IMAGE_EDGES
		std::cout << "GetPolygonFromImage done" << std::endl;

		//UpdateInformation();
		return;


		//Mat v_gradient(image_width, image_width, CV_32F, Scalar(0));
		//Mat h_gradient(image_width, image_width, CV_32F, Scalar(0));
		//Mat v_kernel = (Mat_<float>(3, 3) << -1, -2, -1,
		//	0, 0, 0,
		//	1, 2, 1);
		//Mat h_kernel = (Mat_<float>(3, 3) << -1, 0, 1,
		//	-2, 0, 2,
		//	-1, 0, 1);

		//filter2D(canny, v_gradient, CV_32F, v_kernel, Point(-1, -1), 0);
		//filter2D(canny, h_gradient, CV_32F, h_kernel, Point(-1, -1), 0);

		//double minVal, maxVal;
		//double divide;

		//minMaxLoc(v_gradient, &minVal, &maxVal); //find minimum and maximum intensities
		//divide = 1 / (maxVal - minVal);
		//for (int i = 0; i < image_width; i++) {
		//	for (int j = 0; j < image_width; j++) {
		//		v_gradient.at<float>(i, j) = (v_gradient.at<float>(i, j) - minVal) * 255 * divide;
		//	}
		//}
		//v_gradient.convertTo(v_gradient, CV_8U);
		//cout << minVal << ", " << maxVal << endl;

		//minMaxLoc(h_gradient, &minVal, &maxVal); //find minimum and maximum intensities
		//divide = 1 / (maxVal - minVal);
		//for (int i = 0; i < image_width; i++) {
		//	for (int j = 0; j < image_width; j++) {
		//		h_gradient.at<float>(i, j) = (h_gradient.at<float>(i, j) - minVal) * 255 * divide;
		//	}
		//}
		//h_gradient.convertTo(h_gradient, CV_8U);
		//cout << minVal << ", " << maxVal << endl;

		//Mat gradient;
		//addWeighted(v_gradient, 0.5, h_gradient, 0.5, 0, gradient);

		////v_gradient.convertTo(v_gradient, CV_8U);
		////h_gradient.convertTo(h_gradient, CV_8U);
		//gradient.convertTo(gradient, CV_8U);
		////imshow("vertical", v_gradient);
		////imshow("horizontal", h_gradient);
		////imshow("gradient", gradient);

		//imwrite("vertical.png", v_gradient);
		//imwrite("horizontal.png", h_gradient);
		//imwrite("gradient.png", gradient);

		//Mat white(image_width, image_width, CV_8UC1, Scalar(0));
		//vector<Mat> channels{ white, h_gradient, v_gradient };
		//Mat color_gradient;
		//merge(channels, color_gradient);
		////imshow("color", color_gradient);
		//imwrite("color.png", color_gradient);

		//Mat blurred_color;
		//cv::GaussianBlur(color_gradient, blurred_color, Size(3, 3), 3, 3);
		////imshow("blurred", blurred_color);
		//imwrite("blurred_color.png", blurred_color);

		//Mat final(image_width, image_width, CV_8UC3, Scalar(0, 0, 0));
		//Mat blurred_final(image_width, image_width, CV_8UC3, Scalar(0, 0, 0));
		//bool temp = false;
		//Point2i start_point{ 0, 0 };
		//for (int i = 0; i < final.rows; i++) {
		//	for (int j = 0; j < final.cols; j++) {
		//		//final.at<Vec3b>(i, j)[0] = 0;
		//		if (final_edge.at<uchar>(i, j) == 255) {
		//			//final.at<Vec3b>(i, j)[1] = v_gradient.at<uchar>(i, j);
		//			//final.at<Vec3b>(i, j)[2] = h_gradient.at<uchar>(i, j);
		//			//cout << i << ", " << j << ": " << (int)final.at<Vec3b>(i, j)[1] << ", "
		//				//<< (int)final.at<Vec3b>(i, j)[2] << endl;
		//			final.at<Vec3b>(i, j) = color_gradient.at<Vec3b>(i, j);
		//			blurred_final.at<Vec3b>(i, j) = blurred_color.at<Vec3b>(i, j);
		//			if (!temp) {
		//				start_point = Point2i{ j, i };
		//				temp = true;
		//			}
		//		}
		//	}
		//}
		////imshow("final", final);
		//imwrite("final.png", final);

		//imshow("blurred_final", blurred_final);
		//imwrite(file_name + "blurred_final.png", blurred_final);

		////waitKey();
		//cout << start_point.x << ", " << start_point.y << endl;

		//Mat image;
		//blurred_final.convertTo(image, CV_32F, 1, -128);

		//vector<Point2i> polygon;
		//Point2i pointa(0, 0), pointb(0, 0);
		//get_line(image, start_point, pointa, pointb);
		//polygon.push_back(pointa);
		//polygon.push_back(pointb);

		//curr = pointb;
		//int i = 0;
		//while (get_distance(curr, pointa) > 3) {
		//	cout << i << endl;
		//	Point2i a, b;
		//	get_line(image, curr, a, b);
		//	if (a == b) break;
		//	if (a != curr) {
		//		cout << "a != curr: " << curr << ", " << a << endl;
		//	}
		//	polygon.push_back(b);
		//	curr = b;
		//	i++;
		//}

		//for (int i = 0; i < polygon.size() - 1; i++) {
		//	if (polygon[i] == polygon[i + 1]) cout << "repeat" << endl;
		//	line(blurred_final, polygon[i], polygon[i + 1], Scalar(255, 255, 255), 1);
		//}
		////line(blurred_final, polygon.back(), polygon[0], Scalar(255, 255, 255));
		////imshow("final", blurred_final);
		//imwrite(file_name + "painted_blurred_final.png", blurred_final);

		//corner_pnts_.clear();
		//std::cout << "corner_pnts_:" << std::endl;
		//for (int i = 0; i < polygon.size(); i++) {
		//	double minDist = 1000;
		//	Point_2d point(0, 0);
		//	for (auto &p : ptop) {
		//		double currDist = get_distance(polygon[i], p.first);
		//		if (currDist < minDist) {
		//			minDist = currDist;
		//			point = p.second;
		//		}
		//	}
		//	float2 float_point;
		//	float_point.x = point.x();
		//	float_point.y = point.y();
		//	if (!corner_pnts_.empty()) {
		//		float2 previous = corner_pnts_.back();
		//		float2 first_point = corner_pnts_[0];
		//		if (abs(previous.x - float_point.x) < 1e-8 && abs(previous.y - float_point.y) < 1e-8) continue;
		//		if (abs(first_point.x - float_point.x) < 1e-8 && abs(first_point.y - float_point.y) < 1e-8) continue;
		//	}
		//	corner_pnts_.push_back(float_point);
		//	std::cout << float_point.x << ", " << float_point.y << std::endl;
		//}
		////std::ofstream fo("D:\\Huawei\\beijing_research_laboratory\\F2\\2019-12-17_14.56.10\\test_polygon\\polygon.obj");
		////for (int i = 0; i < corner_pnts_.size(); ++i)
		////{
		//	//fo << "v " << corner_pnts_[i].x << " " << corner_pnts_[i].y << " " << "0.0" << std::endl;
		////}
		////fo.close();

		//if (corner_pnts_.size() > 2) {
		//	float2 edge1 = corner_pnts_[1] - corner_pnts_[0];
		//	float2 edge2 = corner_pnts_[2] - corner_pnts_[1];
		//	if (edge1.x*edge2.y - edge1.y*edge2.x > 0 ^ IsConcaveVertex(1) == 1) {
		//		std::vector<float2> corner_pnts_inverse;
		//		for (int i = corner_pnts_.size() - 1; i >= 0; i--) {
		//			corner_pnts_inverse.emplace_back(corner_pnts_[i]);
		//		}
		//		corner_pnts_ = corner_pnts_inverse;
		//	}
		//}

		//for (int i = 0; i < corner_pnts_.size(); i++) {
		//	Eigen::Vector4f pnt(corner_pnts_[i].x, corner_pnts_[i].y, 0.0, 1.0);
		//	Eigen::Vector4f transform_pnt = plane_to_world_ * pnt;
		//	corner_pnts_3d_.emplace_back(make_float3(transform_pnt(0), transform_pnt(1), transform_pnt(2)));
		//}
		//std::cout << "border_edges_.size() = " << border_edges_.size() << std::endl;
		//std::cout << "polygon.size() = " << polygon.size() << std::endl;
		//std::cout << "corner_pnts_3d_.size() = " << corner_pnts_3d_.size() << std::endl;

		////if (corner_pnts_3d_.size() > 2) {
		////	float3 edge1 = corner_pnts_3d_[1] - corner_pnts_3d_[0];
		////	float3 edge2 = corner_pnts_3d_[2] - corner_pnts_3d_[1];
		////	Eigen::Vector3f e1(edge1.x, edge1.y, edge1.z), e2(edge2.x, edge2.y, edge2.z);
		////	Eigen::Vector3f polygon_normal = e1.cross(e2);

		////	if (polygon_normal.dot(average_pnts_normal_) <= 0) {
		////		std::vector<float3> corner_pnts_3d_inverse;
		////		for (int i = corner_pnts_3d_.size() - 1; i >= 0; i--) {
		////			corner_pnts_3d_inverse.emplace_back(corner_pnts_3d_[i]);
		////		}
		////		corner_pnts_3d_ = corner_pnts_3d_inverse;
		////	}
		////}
		//UpdateInformation();
	}

	void HWPlane::GetPolygonFromImageNew(cv::Mat& src, const std::map<cv::Point2i, Point_2d, Cmp> &ptop, const std::string& filename, const Eigen::Vector4d& params, bool all_steps = false)
	{
		using std::string;
		using std::vector;
		using std::cout;
		using std::endl;
		using namespace cv;

		int image_width = src.rows;
#ifdef LIULINGFEI
		imwrite(filename + ".png", src);
#endif
		string input("lines.png");
		string dilate_output("dilate.png");
		string erode_output("erode.png");
		string canny_output("canny.png");

		//Mat src = imread(input);
		//imshow("edge", src);

		//params_.dilate_half_width = ave_pixels_;
		int dilation_size = params_.dilate_half_width;
		if (dilation_size == 0) {
			dilation_size = ave_pixels_;
			initial_params_.dilate_half_width = ave_pixels_;
		}
		std::cout << "dilation_size = " << dilation_size << std::endl;
		Mat dilation_element = getStructuringElement(MORPH_RECT,
			Size(2 * dilation_size + 1, 2 * dilation_size + 1),
			Point(dilation_size, dilation_size));
		dilation_size = 1;
		Mat dilation_element3 = getStructuringElement(MORPH_RECT,
			Size(2 * dilation_size + 1, 2 * dilation_size + 1),
			Point(dilation_size, dilation_size));

		//膨胀
		Mat dst;
		dilate(src, dst, dilation_element);
		dilate(dst, dst, dilation_element);
		dilate(dst, dst, dilation_element);
		for (int i = 0; i < dst.rows; i++) {
			for (int j = 0; j < dst.cols; j++) {
				if (dst.at<uchar>(i, j) < 128) dst.at<uchar>(i, j) = 0;
				else dst.at<uchar>(i, j) = 255;
			}
		}
		//imshow("dilate", dst);
#ifdef LIULINGFEI
		imwrite(filename + dilate_output, dst);
#endif
		//腐蚀
		erode(dst, dst, dilation_element);
		erode(dst, dst, dilation_element);
		erode(dst, dst, dilation_element);
#ifdef LIULINGFEI
		imwrite(filename + erode_output, dst);
#endif

		Mat canny;
		//求边缘
		Canny(dst, canny, 0, 10, 3, true);
	
		Mat final_edge = canny.clone();
#ifdef LIULINGFEI
		imwrite(filename + "final_edge.png", final_edge);
		std::cout << "imwrite final_edge.png" << std::endl;
#endif

		//找出所有边缘像素
		std::vector<cv::Point2i> all_edge_points;
		for (int i = 0; i < final_edge.rows; i++) {
			for (int j = 0; j < final_edge.cols; j++) {
				if (final_edge.at<uchar>(i, j) == 255) {
					cv::Point2i start;
					start.x = j;
					start.y = i;
					all_edge_points.push_back(start);
				}
			}
		}
		std::cout << "start found" << std::endl;
		//std::cout << start.x << ", " << start.y << std::endl;

		//开始寻找外边缘
		std::vector<cv::Point2i> selected;
		std::vector<float2> edge_points;
		int count = 0;
		std::vector<std::vector<cv::Point2i> > extracted_poly_pnts;
		std::vector<bool> all_edge_points_visited_flag(all_edge_points.size(), false);
		for (int start_index = 0; start_index < all_edge_points.size(); start_index++) {
			if (all_edge_points_visited_flag[start_index])	//跳过已经搜索的顶点
				continue;
			std::vector<cv::Point2i> edge_circle_visited;	//一轮搜索成功后未采样的点云的坐标
			std::cout << "the start_idx: " << start_index << std::endl;
			Mat final_edge_copy = final_edge.clone();
			//edge_pnts_pos_.clear();
			edge_points.clear();
			selected.clear();
			count = 0;
			Point2i curr = all_edge_points[start_index];
			//1cm采样一个像素点
			//int sample_rate = 0.01 / distance_per_pixel_ + 1;
			int sample_rate = params_.pixel_to_world_sampling_rate;
			if (sample_rate == 0) {
				sample_rate = ave_pixels_ / 2;
				initial_params_.pixel_to_world_sampling_rate = ave_pixels_ / 2;
			}
			std::cout << "sample_rate = " << sample_rate << std::endl;
			if (sample_rate < 1) sample_rate = 1;
			//if (sample_rate > 4) sample_rate = 4;
			std::cout << "plane_width = " << plane_width_ << std::endl;
			std::cout << "sample_rate = " << sample_rate << std::endl;
			int dist_max = 3;	//最大的距离
			while (true) {
				bool found_next = false;
				std::vector<Point2i> high_pri3;
				std::vector<Point2i> low_pri3;
				std::vector<Point2i> high_pri5;
				std::vector<Point2i> low_pri5;
				for (int i = -2; i < 3; i++) {
					for (int j = -2; j < 3; j++) {
						//在3x3范围内
						if (i == 0 && j == 0) continue;
						Point2i temp(curr.x + j, curr.y + i);
						if (i >= -1 && i <= 1 && j >= -1 && j <= 1) {
							if (i == 0 || j == 0) {
								high_pri3.push_back(temp);
							}
							else {
								low_pri3.push_back(temp);
							}
						}
						//不在3x3范围内
						else {
							if ((i >= -1 && i <= 1) || (j >= -1 && j <= 1)) {
								high_pri5.push_back(temp);
							}
							else {
								low_pri5.push_back(temp);
							}
						}
					}
				}
				std::vector<Point2i> pri_vec;
				pri_vec.insert(pri_vec.end(), high_pri3.begin(), high_pri3.end());
				pri_vec.insert(pri_vec.end(), low_pri3.begin(), low_pri3.end());
				pri_vec.insert(pri_vec.end(), high_pri5.begin(), high_pri5.end());
				pri_vec.insert(pri_vec.end(), low_pri5.begin(), low_pri5.end());

				for (auto & p : pri_vec) {
					int row = p.y;
					int col = p.x;
					if (final_edge_copy.at<uchar>(row, col) == 255) {
						if (count % sample_rate == 0) {
							//Eigen::Vector2d pnt = image_to_plane(col, row);
							//sorted_border_pnts_2d_.emplace_back(Point_2d(pnt(0), pnt(1)));
							//edge_pnts_pos_.push_back(make_float2(pnt(0), pnt(1)));
							//edge_points.push_back(make_float2(pnt(0), pnt(1)));
							selected.push_back(p);
							//std::cout << "selected.push_back " << p << std::endl;
						}
						final_edge_copy.at<uchar>(row, col) = 0;
						curr = p;
						found_next = true;
						edge_circle_visited.emplace_back(curr);
						count++;
						break;
					}
				}

				//FIXME:找到最邻近的点后，扩张的方向可能与原方向相反
				if (!found_next) {
					std::cout << "not found next" << std::endl;
					int dist = 3;
					while (!found_next && dist < ave_pixels_) {
						//std::cout << "dist = " << dist << std::endl;
						int row = curr.x - dist;
						int col = curr.y - dist;

						//上面的行
						for (int i = 0; i < 2 * dist + 1; i++) {
							if (row < 0 || row >= final_edge_copy.rows ||
								col < 0 || col >= final_edge_copy.cols) continue;
							if (final_edge_copy.at<uchar>(row, col) == 255) {
								curr = Point2i(col, row);
								found_next = true;
								break;
							}
							col++;
						}
						//右边的列
						for (int i = 0; i < 2 * dist + 1; i++) {
							if (row < 0 || row >= final_edge_copy.rows ||
								col < 0 || col >= final_edge_copy.cols) continue;
							if (final_edge_copy.at<uchar>(row, col) == 255) {
								curr = Point2i(col, row);
								found_next = true;
								break;
							}
							row++;
						}
						//下面的行
						for (int i = 0; i < 2 * dist + 1; i++) {
							if (row < 0 || row >= final_edge_copy.rows ||
								col < 0 || col >= final_edge_copy.cols) continue;
							if (final_edge_copy.at<uchar>(row, col) == 255) {
								curr = Point2i(col, row);
								found_next = true;
								break;
							}
							col--;
						}
						//左边的列
						for (int i = 0; i < 2 * dist + 1; i++) {
							if (row < 0 || row >= final_edge_copy.rows ||
								col < 0 || col >= final_edge_copy.cols) continue;
							if (final_edge_copy.at<uchar>(row, col) == 255) {
								curr = Point2i(col, row);
								found_next = true;
								break;
							}
							row--;
						}
						dist++;
					}

					if (!found_next) break;
					//Eigen::Vector2d pnt = image_to_plane(curr.x, curr.y);
					//sorted_border_pnts_2d_.emplace_back(Point_2d(pnt(0), pnt(1)));
					////edge_pnts_pos_.push_back(make_float2(pnt(0), pnt(1)));
					//edge_points.push_back(make_float2(pnt(0), pnt(1)));
					selected.push_back(curr);
					final_edge_copy.at<uchar>(curr.y, curr.x) = 0;
					count++;
					edge_circle_visited.emplace_back(curr);
				}
			}

			//std::cout << "the count....... " << count << std::endl;
			int selected_num = selected.size();
			//std::cout << "the selected_num....... " << selected_num << std::endl;
			if (count > all_edge_points.size() / 2)
			{
				extracted_poly_pnts.emplace_back(selected);
				break;
			}
			else if (count >  max(all_edge_points.size() / 40, 3) && count < all_edge_points.size() / 2)
			{
				////首尾
				//Point2i tmp = (selected[0] - selected[selected.size() - 1]);
				//if (std::sqrtf(tmp.dot(tmp)) < ave_pixels_)
				//{
				//}
				for (int j = 0; j < all_edge_points.size(); ++j)
				{
					if (all_edge_points_visited_flag[j])
						continue;
					if (FindPoint2iInPointVec(all_edge_points[j], edge_circle_visited))
					{
						//std::cout << "find all_edge_points[j] " << j << ": " << "true..." << std::endl;
						all_edge_points_visited_flag[j] = true;
					}
				}
				extracted_poly_pnts.emplace_back(selected);
				final_edge = final_edge_copy.clone();
				//break;
			}
		}

		int poly_max_pnt_num = -1;
		int poly_max_idx = -1;
		for (int i = 0; i < extracted_poly_pnts.size(); ++i)
		{
			int ex_i_size = extracted_poly_pnts[i].size();
			//std::cout << "-----------ex_i_size: " << ex_i_size << std::endl;
			if (poly_max_pnt_num < ex_i_size)
			{
				poly_max_pnt_num = ex_i_size;
				poly_max_idx = i;
			}
		}
		std::cerr << "the poly_max_idx: " << poly_max_idx << std::endl;

		if (poly_max_idx != -1)
		{
			edge_points.clear();
			for (int i = 0; i < extracted_poly_pnts[poly_max_idx].size(); ++i)
			{
				//Point2i curr = Point2i(extracted_poly_pnts[poly_max_idx], row);
				Eigen::Vector2d pnt = image_to_plane(extracted_poly_pnts[poly_max_idx][i].x, extracted_poly_pnts[poly_max_idx][i].y);
				sorted_border_pnts_2d_.emplace_back(Point_2d(pnt(0), pnt(1)));
				edge_points.emplace_back(make_float2(pnt(0), pnt(1)));
			}
			selected = extracted_poly_pnts[poly_max_idx];
		}
		else
		{
			edge_points.clear();
			std::cout << "find edge points from image failed" << std::endl;
			return;
		}

		float2 edge1 = edge_points[1] - edge_points[0];
		float2 edge2 = edge_points[2] - edge_points[1];

		//if (edge1.x*edge2.y - edge1.y*edge2.x == 0) {
		//std::cout << "edge1: " << edge1.x << " " << edge1.y << "\n";
		//std::cout << "edge2: " << edge2.x << " " << edge2.y << "\n";
		//system("pause");
		//}

		std::cout << "start inversing" << std::endl;
		//解决定点确定的多边形法向量反向的问题
		//将顶点向量中的点逆序排列
		if (edge1.x*edge2.y - edge1.y*edge2.x > 0 ^ IsConcaveEdgePoint(edge_points, 1) == 1) {
			std::vector<float2> edge_pnts_inverse;
			for (int i = edge_points.size() - 1; i >= 0; i--) {
				edge_pnts_inverse.emplace_back(edge_points[i]);
			}
			edge_points = edge_pnts_inverse;
		}

		edge_pnts_pos_ = edge_points;
		edge_pnts_pos_num_ = edge_pnts_pos_.size();
	std:cout << "selected_edge_points.size(): " << edge_pnts_pos_num_ << std::endl;
#ifdef LIULINGFEI
		imwrite(filename + "final_edge_left.png", final_edge);
#endif

		std::cout << "inner edges:" << std::endl;
		std::vector<std::vector<float2>> inner_edges;
		//开始寻找内边缘
		if (extracted_poly_pnts.size() > 2)
		{
			std::vector<float2> inner_edge;
			for (int i = 0; i < extracted_poly_pnts.size(); ++i)
			{
				if (i == poly_max_idx)
					continue;
				for (int j = 0; j < extracted_poly_pnts[i].size(); ++j)
				{
					Eigen::Vector2d pnt = image_to_plane(extracted_poly_pnts[i][j].x, extracted_poly_pnts[i][j].y);
					inner_edge.emplace_back(make_float2(pnt(0), pnt(1)));
				}
				inner_edges.emplace_back(inner_edge);
			}
			inner_edge_pnts_pos_ = inner_edges;
			inner_polygon_num_ = inner_edge_pnts_pos_.size();
		}
		
		//开关
		inner_edge_pnts_pos_.clear();
		inner_polygon_num_ = 0;

		//写入平面空间的边缘点和边缘
#ifdef OUTPUT_IMAGE_EDGES
		std::ofstream fh(filename + "image_edges.obj");
		for (int i = 0; i < edge_pnts_pos_.size(); i++)
		{
			float2 start_pnt = edge_pnts_pos_[i];
			fh << "v " << start_pnt.x << " " << start_pnt.y << " " << "0.0" << std::endl;
		}
		for (int ii = 0; ii < edge_pnts_pos_.size() - 1; ii++) {
			fh << "l " << ii + 1 << " " << ii + 2 << "\n";
		}
		fh << "l " << 1 << " " << edge_pnts_pos_.size() << "\n";
		fh.close();

		Mat selected_edge_points(image_width, image_width, CV_8UC3, Scalar(0, 0, 0));
		for (int i = 0; i < selected.size() - 1; i++) {
			circle(selected_edge_points, selected[i], 1, Scalar(0, 255, 0));
			line(selected_edge_points, selected[i], selected[i + 1], Scalar(255, 255, 255), 2);
		}
		circle(selected_edge_points, selected[selected.size() - 1], 1, Scalar(0, 255, 0));
		line(selected_edge_points, selected[0], selected[selected.size() - 1], Scalar(255, 255, 255), 2);
#ifdef LIULINGFEI
		imwrite(filename + "selected_edge_points.png", selected_edge_points);
#endif
		//写入三维空间的边缘点和边缘
		std::vector<float3> points_3d;
		for (int i = 0; i < edge_pnts_pos_.size(); i++) {
			Eigen::Vector4f pnt(edge_pnts_pos_[i].x, edge_pnts_pos_[i].y, 0.0, 1.0);
			Eigen::Vector4f transform_pnt = plane_to_world_ * pnt;
			points_3d.emplace_back(make_float3(transform_pnt(0), transform_pnt(1), transform_pnt(2)));
		}

		fh.open(filename + "image_edges_3d.obj");
		for (int i = 0; i < points_3d.size(); i++)
		{
			float3 start_pnt = points_3d[i];
			fh << "v " << start_pnt.x << " " << start_pnt.y << " " << start_pnt.z << std::endl;
		}
		for (int ii = 0; ii < points_3d.size() - 1; ii++) {
			fh << "l " << ii + 1 << " " << ii + 2 << "\n";
		}
		fh << "l " << 1 << " " << points_3d.size() << "\n";
		fh.close();
#endif //OUTPUT_IMAGE_EDGES
		std::cout << "GetPolygonFromImage done" << std::endl;

		//UpdateInformation();
		return;
	}

	bool HWPlane::FindPoint2iInPointVec(cv::Point2i pnt, std::vector<cv::Point2i>& pnts_vec)
	{
		for (int i = 0; i < pnts_vec.size(); ++i)
		{
			if (pnts_vec[i] == pnt)
			{
				return true;
			}
		}
		return false;
	}

	void HWPlane::WriteEdges2D(std::string filename)
	{
		std::ofstream fh(filename);
		for (int i = 0; i < edge_pnts_pos_.size(); i++)
		{
			float2 start_pnt = edge_pnts_pos_[i];
			fh << "v " << start_pnt.x << " " << start_pnt.y << " " << "0.0" << std::endl;
		}
		for (int i = 0; i < edge_pnts_pos_.size() - 1; i++) {
			fh << "l " << i + 1 << " " << i + 2 << "\n";
		}
		fh << "l " << 1 << " " << edge_pnts_pos_.size() << "\n";
		fh.close();
	}

	void HWPlane::WriteEdges3D(std::string filename)
	{
		std::vector<float3> points_3d;
		std::vector<float3> points_normal;
		//out_fh << "vn " << pnts_normal_[i].x << " " << pnts_normal_[i].y << " " << pnts_normal_[i].z << std::endl;
		for (int i = 0; i < edge_pnts_pos_.size(); i++) {
			Eigen::Vector4f pnt(edge_pnts_pos_[i].x, edge_pnts_pos_[i].y, 0.0, 1.0);
			Eigen::Vector2f nml(edge_pnts_normal_[i].x, edge_pnts_normal_[i].y);
			nml.normalize();
			Eigen::Vector4f pnt_end(edge_pnts_pos_[i].x + nml.x(), edge_pnts_pos_[i].y + nml.y(), 0.0, 1.0);

			Eigen::Vector4f transform_pnt = plane_to_world_ * pnt;
			Eigen::Vector4f transform_end = plane_to_world_ * pnt_end;

			points_3d.emplace_back(make_float3(transform_pnt(0), transform_pnt(1), transform_pnt(2)));
			points_3d.emplace_back(make_float3(transform_end(0), transform_end(1), transform_end(2)));
		}

		std::ofstream fh(filename);
		for (int i = 0; i < points_3d.size(); i++)
		{
			uchar3 rgb = colors_[edge_pnts_color_index_[i/2]];
			float3 color = make_float3(rgb.x / 255.0, rgb.y / 255.0, rgb.z / 255.0);

			float3 start_pnt = points_3d[i];
			float3 nml = points_normal[i];
			fh << "v " << start_pnt.x << " " << start_pnt.y << " " << start_pnt.z << " "
						<< color.x << " " << color.y << " " << color.z << std::endl;
			//fh << "vn " << nml.x << " " << nml.y << " " << nml.z << std::endl;
		}
		for (int i = 0; i < points_3d.size() / 2 - 1; i++) {
			fh << "l " << 2*i + 1 << " " << 2*i + 2 << "\n";
			fh << "l " << 2*i + 1 << " " << 2*i + 3 << "\n";
		}
		fh << "l " << 1 << " " << points_3d.size() - 2 << "\n";
		fh << "l " << points_3d.size() - 2 << " " << points_3d.size() - 1 << "\n";
		fh.close();
	}

	void HWPlane::WriteEdges3DWithoutNormals(std::string filename)
	{
		std::vector<float3> points_3d;
		//std::vector<float3> points_normal;
		//out_fh << "vn " << pnts_normal_[i].x << " " << pnts_normal_[i].y << " " << pnts_normal_[i].z << std::endl;
		for (int i = 0; i < edge_pnts_pos_.size(); i++) {
			Eigen::Vector4f pnt(edge_pnts_pos_[i].x, edge_pnts_pos_[i].y, 0.0, 1.0);
			Eigen::Vector4f transform_pnt = plane_to_world_ * pnt;
			points_3d.emplace_back(make_float3(transform_pnt(0), transform_pnt(1), transform_pnt(2)));
		}

		std::ofstream fh(filename);
		for (int i = 0; i < points_3d.size(); i++)
		{
			uchar3 rgb = colors_[edge_pnts_color_index_[i / 2]];
			float3 color = make_float3(rgb.x / 255.0, rgb.y / 255.0, rgb.z / 255.0);
			float3 start_pnt = points_3d[i];
			fh << "v " << start_pnt.x << " " << start_pnt.y << " " << start_pnt.z << " "
				<< color.x << " " << color.y << " " << color.z << std::endl;
		}
		for (int i = 0; i < points_3d.size() - 1; i++) {
			fh << "l " <<  i + 1 << " " <<  i + 2 << "\n";
		}
		fh << "l " << 1 << " " << points_3d.size() << "\n";
		fh.close();
	}

	void HWPlane::WriteOriginEdgesPnts3DByFlag(std::string filename, const std::vector<Eigen::Vector3f>& poly_pnts,
		const std::vector<Eigen::Vector3f>& poly_pnts_normal, float len_normal,
		std::vector<Eigen::Vector3f> poly_pnts_color, std::vector<bool>& is_edge_vertex)
	{
		bool has_poly_normal = false;
		bool has_poly_color = false;
		if (poly_pnts.size() == poly_pnts_normal.size())
		{
			has_poly_normal = true;
		}
		if (poly_pnts.size() == poly_pnts_color.size())
		{
			has_poly_color = true;
		}
		std::ofstream fh(filename);
		if (fh.is_open())
		{
			for (int i = 0; i < poly_pnts.size(); ++i)
			{
				if (has_poly_color)
				{
					fh << "v " << poly_pnts[i][0]<< " " << poly_pnts[i][1] << " " << poly_pnts[i][2] << " "
						<< poly_pnts_color[i][0] << " " << poly_pnts_color[i][1] << " " << poly_pnts_color[i][2] << std::endl;
				}
				else
				{
					fh << "v " << poly_pnts[i][0] << " " << poly_pnts[i][1] << " " << poly_pnts[i][2] << std::endl;
				}
			}
			if (has_poly_normal)
			{
				for (int i = 0; i < poly_pnts_normal.size(); ++i)
				{
					Eigen::Vector3f start_pnt = poly_pnts[i];
					Eigen::Vector3f s_normalize = poly_pnts_normal[i].normalized();
					Eigen::Vector3f end_pnt = start_pnt + len_normal*s_normalize;
					fh << "v " << end_pnt[0] << " " << end_pnt[1] << " " << end_pnt[2] << " "
						<< poly_pnts_color[i][0] << " " << poly_pnts_color[i][1] << " " << poly_pnts_color[i][2] << std::endl;
				}
			}

			int poly_pnts_num = static_cast<int>(poly_pnts.size());
			if (poly_pnts_num == 1)
			{
			}
			else if (poly_pnts_num == 2)
			{
				//set line
				for (int ii = 0; ii < poly_pnts.size() - 1; ++ii)
				{
					fh << "l " << ii + 1 << " " << ii + 2 << "\n";
					fh << "l " << ii + 1 + poly_pnts_num << " " << ii + 2 + poly_pnts_num << "\n";
				}
			}
			else
			{
				//set line
				for (int ii = 0; ii < poly_pnts.size() - 1; ++ii)
				{
					fh << "l " << ii + 1 << " " << ii + 2 << "\n";
					fh << "l " << ii + 1 << " " << ii + 1 + poly_pnts_num << "\n";
				}
				fh << "l " << poly_pnts.size() << " " << 1 << "\n";
				fh << "l " << poly_pnts.size() << " " << poly_pnts.size() + poly_pnts_num << "\n";
			}
			fh.close();
		}
	}

	void HWPlane::WritePolygonPnts2dIntoPolygonPnt3dWithNormals(const std::string filename, const std::vector<Eigen::Vector3f>& poly_pnts,
		const std::vector<Eigen::Vector3f>& poly_pnts_normal, float len_normal)
	{
		bool has_poly_normal = false;
		if (poly_pnts.size() == poly_pnts_normal.size())
		{
			has_poly_normal = true;
		}
		std::ofstream fh(filename);
		if (fh.is_open())
		{
			for (int i = 0; i < poly_pnts.size(); ++i)
			{
				fh << "v " << poly_pnts[i][0] << " " << poly_pnts[i][1] << " " << poly_pnts[i][2] << std::endl;
			}
			if (has_poly_normal)
			{
				for (int i = 0; i < poly_pnts_normal.size(); ++i)
				{
					Eigen::Vector3f start_pnt = poly_pnts[i];
					Eigen::Vector3f s_normalize = poly_pnts_normal[i].normalized();
					Eigen::Vector3f end_pnt = start_pnt + len_normal*s_normalize;
					fh << "v " << end_pnt[0] << " " << end_pnt[1] << " " << end_pnt[2] << std::endl;
				}
			}
			int poly_pnts_num = static_cast<int>(poly_pnts.size());
			if (poly_pnts_num == 1)
			{
				return;
			}
			else if (poly_pnts_num == 2)
			{
				//set line
				//set line
				for (int ii = 0; ii < poly_pnts.size() - 1; ++ii)
				{
					fh << "l " << ii + 1 << " " << ii + 2 << "\n";
					fh << "l " << ii + 1 << " " << ii + 1 + poly_pnts_num << "\n";
				}
				fh << "l " << poly_pnts.size() << " " << poly_pnts.size() + poly_pnts_num << "\n";
			}
			else
			{
				//set line
				for (int ii = 0; ii < poly_pnts.size() - 1; ++ii)
				{
					fh << "l " << ii + 1 << " " << ii + 2 << "\n";
					fh << "l " << ii + 1 << " " << ii + 1 + poly_pnts_num << "\n";
				}
				fh << "l " << poly_pnts.size() << " " << 1 << "\n";
				fh << "l " << poly_pnts.size() << " " << poly_pnts.size() + poly_pnts_num << "\n";
			}
			fh.close();
		}
	}

	void HWPlane::WriteLinePnts2dIntoLinePnt3dWithNormals(const std::string filename, const std::vector<Eigen::Vector3f>& poly_pnts,
		const std::vector<Eigen::Vector3f>& poly_pnts_normal, float len_normal)
	{
		bool has_poly_normal = false;
		if (poly_pnts.size() == poly_pnts_normal.size())
		{
			has_poly_normal = true;
		}
		std::ofstream fh(filename);
		if (fh.is_open())
		{
			for (int i = 0; i < poly_pnts.size(); ++i)
			{
				fh << "v " << poly_pnts[i][0] << " " << poly_pnts[i][1] << " " << poly_pnts[i][2] << std::endl;
			}
			if (has_poly_normal)
			{
				for (int i = 0; i < poly_pnts_normal.size(); ++i)
				{
					Eigen::Vector3f start_pnt = poly_pnts[i];
					Eigen::Vector3f s_normalize = poly_pnts_normal[i].normalized();
					Eigen::Vector3f end_pnt = start_pnt + len_normal*s_normalize;
					fh << "v " << end_pnt[0] << " " << end_pnt[1] << " " << end_pnt[2] << std::endl;
				}
			}
			int poly_pnts_num = static_cast<int>(poly_pnts.size());
			if (poly_pnts_num == 1)
			{
				return;
			}
			else
			{
				//set line
				for (int ii = 0; ii < poly_pnts.size() - 1; ++ii)
				{
					fh << "l " << ii + 1 << " " << ii + 2 << "\n";
					fh << "l " << ii + 1 << " " << ii + 1 + poly_pnts_num << "\n";
				}
				fh << "l " << poly_pnts.size() << " " << poly_pnts.size() + poly_pnts_num << "\n";
			}
			fh.close();
		}
	}

	void HWPlane::WriteSelectEdges3D(std::string filename, const std::vector<int>& selected_pnt_idx)
	{
		if (corner_pnts_3d_.empty()) return;
		std::ofstream fh(filename);
		for (int i = 0; i < selected_pnt_idx.size(); i++)
		{
			float3 start_pnt = corner_pnts_3d_[selected_pnt_idx[i]];
			fh << "v " << start_pnt.x << " " << start_pnt.y << " " << start_pnt.z << std::endl;
		}
		for (int ii = 0; ii < selected_pnt_idx.size() - 1; ii++) {
			fh << "l " << ii + 1 << " " << ii + 2 << "\n";
		}
		fh << "l " << 1 << " " << selected_pnt_idx.size() << "\n";
		fh.close();
	}

	void HWPlane::WriteSelectLines3D(std::string filename, const std::vector<int>& selected_pnt_idx)
	{
		if (corner_pnts_3d_.empty()) return;
		std::ofstream fh(filename);
		for (int i = 0; i < selected_pnt_idx.size(); i++)
		{
			float3 start_pnt = corner_pnts_3d_[selected_pnt_idx[i]];
			fh << "v " << start_pnt.x << " " << start_pnt.y << " " << start_pnt.z << std::endl;
		}
		for (int ii = 0; ii < selected_pnt_idx.size() - 1; ii++) {
			fh << "l " << ii + 1 << " " << ii + 2 << "\n";
		}
		//fh << "l " << 1 << " " << selected_pnt_idx.size() << "\n";
		fh.close();
	}

	void HWPlane::WriteSelectEdges3DMoveFlag(std::string filename, const std::vector<int>& selected_pnt_idx, std::vector<bool>& movflagvec)
	{
		if (corner_pnts_3d_.empty()) return;
		std::ofstream fh(filename);
		for (int i = 0; i < selected_pnt_idx.size(); i++)
		{
			float3 start_pnt = corner_pnts_3d_[selected_pnt_idx[i]];
			float3 my_color = make_float3(1.0, 0.0, 0.0);
			if (movflagvec[i])
			{
				my_color.x = 0.0;
				my_color.y = 0.0;
				my_color.z = 1.0;
			}
			fh << "v " << start_pnt.x << " " << start_pnt.y << " " << start_pnt.z << " "
				<< my_color.x << " " << my_color.y << " " << my_color.z << std::endl;
		}
		for (int ii = 0; ii < selected_pnt_idx.size() - 1; ii++) {
			fh << "l " << ii + 1 << " " << ii + 2 << "\n";
		}
		fh << "l " << 1 << " " << selected_pnt_idx.size() << "\n";
		fh.close();
	}

	void HWPlane::WriteSelectEdges2DMoveFlag(std::string filename, std::vector<Eigen::Vector2f>& poly2dpnts, std::vector<bool>& movflagvec)
	{
		if (poly2dpnts.empty()) return;
		std::ofstream fh(filename);
		for (int i = 0; i < poly2dpnts.size(); i++)
		{
			Eigen::Vector2f start_pnt = poly2dpnts[i];
			Eigen::Vector3f startpnt3d;
			Pnt2d2Pnt3D(start_pnt, startpnt3d);
			Eigen::Vector3f my_color = Eigen::Vector3f(1.0, 0.0, 0.0);
			if (movflagvec[i])
			{
				my_color[0] = 0.0;
				my_color[1] = 0.0;
				my_color[2] = 1.0;
			}
			fh << "v " << startpnt3d[0] << " " << startpnt3d[1] << " " << startpnt3d[2] << " "
				<< my_color[0] << " " << my_color[1] << " " << my_color[2] << std::endl;
		}
		for (int ii = 0; ii < poly2dpnts.size() - 1; ii++) {
			fh << "l " << ii + 1 << " " << ii + 2 << "\n";
		}
		fh << "l " << 1 << " " << poly2dpnts.size() << "\n";
		fh.close();
	}

	void HWPlane::WriteCornerPoints3D(std::string filename)
	{
		if (all_corner_pnts_3d_.empty()) return;
		std::ofstream fh(filename);
		for (int i = 0; i < all_corner_pnts_3d_.size(); i++)
		{
			float3 start_pnt = all_corner_pnts_3d_[i];
			fh << "v " << start_pnt.x << " " << start_pnt.y << " " << start_pnt.z << std::endl;
		}
		for (int i = 0; i < contour_index_.size() - 1; i++) {
			for (int j = contour_index_[i]; j < contour_index_[i + 1] - 1; j++) {
				fh << "l " << j + 1 << " " << j + 2 << "\n";
			}
			fh << "l " << contour_index_[i + 1] << " " << contour_index_[i] + 1 << "\n";
		}
		fh.close();
		//if (corner_pnts_3d_.empty()) return;
		//std::ofstream fh(filename);
		//for (int i = 0; i < corner_pnts_3d_.size(); i++)
		//{
		//	float3 start_pnt = corner_pnts_3d_[i];
		//	fh << "v " << start_pnt.x << " " << start_pnt.y << " " << start_pnt.z << std::endl;
		//}
		//for (int ii = 0; ii < corner_pnts_3d_.size() - 1; ii++) {
		//	fh << "l " << ii + 1 << " " << ii + 2 << "\n";
		//}
		//fh << "l " << 1 << " " << corner_pnts_3d_.size() << "\n";
		//fh.close();

		for (int i = 0; i < inner_polygon_num_; i++) {
			std::stringstream ss;
			ss << i;
			std::string index;
			ss >> index;
			std::ofstream fh(filename + "inner_contour_3d_" + index + ".obj");
			for (int j = 0; j < inner_corner_pnts_3d_[i].size(); j++) {
				float3 start_pnt = inner_corner_pnts_3d_[i][j];
				fh << "v " << start_pnt.x << " " << start_pnt.y << " " << start_pnt.z << std::endl;
			}
			for (int j = 0; j < inner_corner_pnts_3d_[i].size() - 1; j++) {
				fh << "l " << j + 1 << " " << j + 2 << "\n";
			}
			fh << "l " << inner_corner_pnts_3d_[i].size() << " " << 1 << "\n";
			fh.close();
		}
		//if (corner_pnts_3d_.empty()) return;
		//std::ofstream fh(filename);
		//for (int i = 0; i < corner_pnts_3d_.size(); i++)
		//{
		//	float3 start_pnt = corner_pnts_3d_[i];
		//	fh << "v " << start_pnt.x << " " << start_pnt.y << " " << start_pnt.z << std::endl;
		//}
		//for (int ii = 0; ii < corner_pnts_3d_.size() - 1; ii++) {
		//	fh << "l " << ii + 1 << " " << ii + 2 << "\n";
		//}
		//fh << "l " << 1 << " " << corner_pnts_3d_.size() << "\n";
		//fh.close();
	}

	void HWPlane::WriteCornerPoints2D(std::string filename)
	{
		if (corner_pnts_.empty()) return;
		std::ofstream fh(filename);
		for (int i = 0; i < corner_pnts_.size(); i++)
		{
			float2 start_pnt = corner_pnts_[i];
			fh << "v " << start_pnt.x << " " << start_pnt.y << " " << 0 << std::endl;
		}
		for (int ii = 0; ii < corner_pnts_.size() - 1; ii++) {
			fh << "l " << ii + 1 << " " << ii + 2 << "\n";
		}
		fh << "l " << 1 << " " << corner_pnts_.size() << "\n";
		fh.close();
	}

	void HWPlane::ScoutCornerPoints3D()
	{
		if (corner_pnts_3d_.empty()) return;
		for (int i = 0; i < corner_pnts_3d_.size(); i++)
		{
			float3 start_pnt = corner_pnts_3d_[i];
			std::cout << "v " << start_pnt.x << " " << start_pnt.y << " " << start_pnt.z << std::endl;
		}
	}

	void HWPlane::ScoutCornerPoints2D()
	{
		if (corner_pnts_.empty()) return;
		for (int i = 0; i < corner_pnts_.size(); i++)
		{
			float2 start_pnt = corner_pnts_[i];
			std::cout << "v " << start_pnt.x << " " << start_pnt.y << std::endl;
		}
	}

	void HWPlane::ScoutPlaneVecIdxs(std::vector<int>& vecidxs)
	{
		for (int i = 0; i < vecidxs.size(); ++i)
		{
			std::cerr << vecidxs[i] << " ";
		}
		std::cerr << std::endl;
	}

	void HWPlane::AddPlanePntIdxFromAssociatedPC(int idx)
	{
		//ccGenericPointCloud* pcloud = getAssociatedCloud();
		
		if (idx >= 0)
		{
			pnts_idx_.emplace_back(idx);
		}
	}

	void HWPlane::AddPlanePnt(float x, float y, float z)
	{
		pnts_pos_.emplace_back(make_float3(x, y, z));
		pnts_pos_origin_.emplace_back(make_float3(x, y, z));
	}

	void HWPlane::AddPlanePnt(float3 p)
	{
		pnts_pos_.emplace_back(p);
		pnts_pos_origin_.emplace_back(p);
	}

	void HWPlane::SetPlanePnt(const std::vector<float3>& v)
	{
		pnts_pos_ = v;
		pnts_pos_origin_ = v;
	}

	void HWPlane::AddPlanePntNormal(float x, float y, float z)
	{
		pnts_normal_.emplace_back(make_float3(x, y, z));
		pnts_normal_origin_.emplace_back(make_float3(x, y, z));
	}

	void HWPlane::AddPlanePntNormal(float3 n)
	{
		pnts_normal_.emplace_back(n);
		pnts_normal_origin_.emplace_back(n);
	}

	void HWPlane::SetPlanePntNormal(const std::vector<float3>& v)
	{
		pnts_normal_ = v;
		pnts_normal_origin_ = v;
	}

	void HWPlane::SetPlanePntColor(const std::vector<float3>& c)
	{
		pnts_color_origin_ = c;
	}

	void HWPlane::AddPlanePntColor(uchar r, uchar g, uchar b)
	{
		pnts_color_.emplace_back(make_uchar3(r, g, b));
	}

	void HWPlane::AddPlaneOriginPntColor(float3 c)
	{
		pnts_color_origin_.emplace_back(c);
	}

	void HWPlane::AddPolygonPnt(float3& n)
	{
		corner_pnts_3d_.emplace_back(n);
	}

	const std::vector<float3>& HWPlane::GetOriginPnts()
	{
		return pnts_pos_origin_;
	}

	const std::vector<float3>& HWPlane::GetOriginPntsNormal()
	{
		return pnts_normal_origin_;
	}

	const std::vector<float3>& HWPlane::GetOriginPntsColor()
	{
		return pnts_color_origin_;
	}

	const std::vector<float3>& HWPlane::GetPlanePnts()
	{
		return pnts_pos_;
	}

	const std::vector<float3>& HWPlane::GetPlanePntsNormal()
	{
		return pnts_normal_;
	}

	const std::vector<uchar3>& HWPlane::GetPlaneColors()
	{
		return pnts_color_;
	}

	const std::vector<float3>& HWPlane::GetSampledWorldPnts()
	{
		return sample_world_coord_vertices_;
	}

	Eigen::Vector3f HWPlane::GetOuterPolygonPnt3d(int i)
	{
		Eigen::Vector3f tmppnt3d = Eigen::Vector3f(corner_pnts_3d_[i].x, corner_pnts_3d_[i].y, corner_pnts_3d_[i].z);
		return tmppnt3d;
	}

	float4 HWPlane::GetPlaneCoeff()
	{
		return coeff_;
	}

	const ccPlane* HWPlane::GetAssociatedPlane()
	{
		return associated_cc_plane_;
	}

	void HWPlane::SetAssociatedPlane(ccPlane* plane)
	{
		associated_cc_plane_ = plane;
	}

	float HWPlane::ComputeCloudResolution(const pcl::PointCloud<pcl::PointXY>::Ptr &cloud)
	{
		//
		if (cloud->empty())
			return 0.0;

		float res = 0.0;
		int n_points = 0.0;
		int nres;
		std::vector<int> indices(2);
		std::vector<float> sqr_dist(2);
		pcl::search::KdTree<pcl::PointXY> tree;
		tree.setInputCloud(cloud);
		for (int i = 0; i < cloud->size(); ++i)
		{
			if (!pcl_isfinite((*cloud)[i].x))
				continue;
			//consider the second neigbor
			nres = tree.nearestKSearch(i, 2, indices, sqr_dist);
			if (nres == 2)
			{
				res += std::sqrt(sqr_dist[1]);
				++n_points;
			}
		}
		if (n_points != 0)
			res /= n_points;
		return res;
	}

	float HWPlane::ComputePlaneCloudResolution2D(const pcl::PointCloud<pcl::PointXY>::Ptr &plane_cloud)
	{
		if (plane_cloud->empty())
			return 0.0;

		float res = 0.0;
		int n_points = 0.0;
		int nres;
		std::vector<int> indices(2);
		std::vector<float> sqr_dist(2);
		pcl::search::KdTree<pcl::PointXY> tree;
		tree.setInputCloud(plane_cloud);
		for (int i = 0; i < plane_cloud->size(); ++i)
		{
			if (!pcl_isfinite((*plane_cloud)[i].x))
				continue;
			//consider the second neigbor
			nres = tree.nearestKSearch(i, 2, indices, sqr_dist);
			if (nres == 2)
			{
				res += std::sqrt(sqr_dist[1]);
				++n_points;
			}
		}
		if (n_points != 0)
			res /= n_points;
		return res;
	}

	int HWPlane::ComputePointsNumPerSquareMeter()
	{
		return 1000;
		//std::cout << "start compute point resolution..." << std::endl;
		//pcl::PointCloud<pcl::PointXY>::Ptr p_cloud(new pcl::PointCloud<pcl::PointXY>);
		//p_cloud->resize(plane_coord_pos_.size());
		//for (int i = 0; i < plane_coord_pos_.size(); ++i)
		//{
		//	//printf("%d\n", i);
		//	p_cloud->points[i].x = plane_coord_pos_[i].x;
		//	p_cloud->points[i].y = plane_coord_pos_[i].y;
		//}
		////获取点云的密度
		//std::cout << "the computed point resolution size is：" << plane_coord_pos_.size() << std::endl;
		//int cloud_resolution = ComputeCloudResolution(p_cloud);
	}

	const std::vector<float2>& HWPlane::GetPlaneEdgePnts()
	{
		return edge_pnts_pos_;
	}

	const std::vector<float2>& HWPlane::GetPlaneEdgePntsNormals()
	{
		return edge_pnts_normal_;
	}

	const std::vector<uchar3>& HWPlane::GetPlaneEdgePntsColors()
	{
		return edge_pnts_color_;
	}

	int HWPlane::GetPlaneSemanticLabel()
	{
		return semantic_label_;
	}

	const float & HWPlane::GetPlaneWidth()
	{
		return plane_width_;
	}

	const float & HWPlane::GetPlaneHeight()
	{
		// TODO: 在此处插入 return 语句
		return plane_height_;
	}

	const std::vector<float2>& HWPlane::GetOuterPolygon2D()
	{
		return corner_pnts_;
	}

	const std::vector<float2>& HWPlane::GetPolygonUV()
	{
		return corner_pnts_uv_;
	}

	const std::vector<float3>& HWPlane::GetOuterPolygon()
	{
		return corner_pnts_3d_;
	}

	Eigen::Vector3f HWPlane::GetOuterPolygonCenterPnts3D()
	{
		int num = corner_pnts_3d_.size();
		Eigen::Vector3f center = Eigen::Vector3f(0.0, 0.0, 0.0);
		for (int i = 0; i < corner_pnts_3d_.size(); ++i)
		{
			Eigen::Vector3f pnt = Eigen::Vector3f(corner_pnts_3d_[i].x, 
				corner_pnts_3d_[i].y, corner_pnts_3d_[i].z);
			center += pnt;
		}
		if (num)
		{
			center /= num;
			return center;
		}
		return Eigen::Vector3f(std::numeric_limits<float>::max(), 
			std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
	}

	const std::vector<float3>& HWPlane::GetAllPolygonVertices()
	{
		// TODO: 在此处插入 return 语句
		return all_corner_pnts_3d_;
	}

	const std::vector<float3>& HWPlane::GetAllRefinedPolygonVertices()
	{
		return polygon_tri_refine_pnts_;
	}

	const std::vector<float2>& HWPlane::GetAllRefinedPolygonUV()
	{
		return polygon_tri_refine_uvs_;
	}

	const std::vector<float3>& HWPlane::GetAllRefinedPolygonNormals()
	{
		return polygon_tri_refine_normals_;
	}

	const std::vector<int3>& HWPlane::GetAllRefinedPolygonTris()
	{
		return polygon_tri_refine_idx_;
	}

	const std::vector<int3>& HWPlane::GetTriangles()
	{
		return triangles_idx_;
	}

	const std::vector<int>& HWPlane::GetLines()
	{
		lines_idx_.clear();
#ifdef LIULINGFEI
		std::cout << "contour_index_.size() = " << contour_index_.size() << std::endl;
#endif
		if (contour_index_.empty()) return lines_idx_;
		for (int i = 0; i < contour_index_.size() - 1; i++) {
			#ifdef LIULINGFEI
			std::cout << i << std::endl;
			#endif
			for (int j = contour_index_[i]; j < contour_index_[i + 1] - 1; j++) {
				lines_idx_.push_back(j);
				lines_idx_.push_back(j + 1);
			}
			lines_idx_.push_back(contour_index_[i + 1] - 1);
			lines_idx_.push_back(contour_index_[i]);
		}
		return lines_idx_;
	}

	const std::vector<float>& HWPlane::GetPolygonAngels()
	{
		return corner_angels_;
	}

	const std::vector<float>& HWPlane::GetMinDists() {
		return min_dists_;
	}

	const std::vector<Segment> HWPlane::GetBorderEdges()
	{
		return border_edges_;
	}

	const Eigen::Matrix4f & HWPlane::GetCameraPose()
	{
		return plane_to_world_;
	}
	
		const Eigen::Matrix4f& HWPlane::GetWorld2PlaneMatrix()
	{
		return world_to_plane_;
	}

	void HWPlane::GetBoundingBox(Eigen::Vector3f & boxMin, Eigen::Vector3f & boxMax)
	{
		boxMin = box_min_;
		boxMax = box_max_;
	}

	bool HWPlane::fuseNearbyPnts()
	{
		//去除距离非常近的相邻顶点
		//取它们之间的中点代替它们
		double min_distance = params_.min_corner_points_distance;
		if (min_distance == 0) {
			//1cm的间距太小了
			//min_distance = std::max((double)ave_alpha_edge_len_, 0.01);
			//initial_params_.min_corner_points_distance = std::max((double)ave_alpha_edge_len_, 0.01);
			min_distance = std::max((double)ave_alpha_edge_len_, plane_width_ * 0.01);
			initial_params_.min_corner_points_distance = min_distance;
		}
		std::cout << "ave_alpha_edge_len_ = " << ave_alpha_edge_len_ << ", min_distance = " << min_distance << std::endl;

		std::vector<float2> temp;
		for (int i = 0; i < corner_pnts_.size(); i++) {
			if (i == 0)
				temp.emplace_back(corner_pnts_[i]);
			else {
				float2 p = temp.back();
				if (PointPointDist(p, corner_pnts_[i]) < min_distance)
					temp.back() = (p + corner_pnts_[i]) / 2;
				else
					temp.emplace_back(corner_pnts_[i]);
			}
		}
		if (!temp.empty() && PointPointDist(temp[0], temp.back()) < min_distance) {
			temp[0] = (temp[0] + temp.back()) / 2;
			temp.pop_back();
		}

		bool result = false;

		if (temp.size() != corner_pnts_.size()) {
			result = true;
			corner_pnts_ = temp;
			ComputeCornerPnts3dFromCornerPnts2d();
			UpdateInformation();
		}
		std::cout << "after deleting near points, corner_pnts_.size() = " << corner_pnts_.size() << std::endl;

		return result;
	}

	bool HWPlane::deleteCollinearPnts()
	{
		std::vector<float2> temp2;
		//去除三点共线的情况
		//如果有三点共线，去掉中间的点
		for (int i = 0; i < corner_pnts_.size(); i++) {
			int last = (i - 1 + corner_pnts_.size()) % corner_pnts_.size();
			int next = (i + 1 + corner_pnts_.size()) % corner_pnts_.size();
			//中间点到两端点连成的直线的距离大于1cm，则判定为不共线，距离太小了
			//if (PointLineSegmentDist(temp[i], temp[last], temp[next]) > 0.01)
			//plane_width_是包围盒较短的边的长度
			if (PointLineSegmentDist(corner_pnts_[i], corner_pnts_[last], corner_pnts_[next]) > plane_width_ * 0.01)
				temp2.emplace_back(corner_pnts_[i]);
			//float2 l1 = temp[i] - temp[last];
			//float2 l2 = temp[next] - temp[i];
			//float theta = GetTheta(l1.x, l1.y, l2.x, l2.y);
			//if (theta > 10) temp2.emplace_back(temp[i]);
		}

		bool result = false;
		if (temp2.size() != corner_pnts_.size()) {
			result = true;
			corner_pnts_ = temp2;
			ComputeCornerPnts3dFromCornerPnts2d();
			UpdateInformation();
		}
		std::cout << "after deleting colinear points, corner_pnts_.size() = " << corner_pnts_.size() << std::endl;
		return result;
	}

	const std::vector<float3>& HWPlane::GetPlaneCoordPos()
	{
		return plane_coord_pos_;
	}

	bool HWPlane::HasEdgePnts()
	{
		return (edge_pnts_pos_.size() > 0);
	}

	void HWPlane::Compute2DPntsBoxPos(float2& min_corner, float2& max_corner)
	{
		float2 min_value = make_float2(FLT_MAX, FLT_MAX);
		float2 max_value = make_float2(FLT_MIN, FLT_MIN);
		for (int i = 0; i < sorted_border_pnts_2d_.size(); ++i)
		{
			if (min_value.x > sorted_border_pnts_2d_[i].x())
				min_value.x = sorted_border_pnts_2d_[i].x();
			if(min_value.y > sorted_border_pnts_2d_[i].y())
				min_value.y = sorted_border_pnts_2d_[i].y();
			if(max_value.x < sorted_border_pnts_2d_[i].x())
				max_value.x = sorted_border_pnts_2d_[i].x();
			if (max_value.y < sorted_border_pnts_2d_[i].y())
				max_value.y = sorted_border_pnts_2d_[i].y();
		}
		min_corner.x = min_value.x;
		min_corner.y = min_value.y;
		max_corner.x = max_value.x;
		max_corner.y = max_value.y;
	}

	//---------------------------borders---------------------------//
#if 1
	int HWPlane::EstimateBordersFromPcl(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, float re, float reforn)
	{
		pcl::PointCloud<pcl::Boundary> boundaries;
		pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> boundEst;
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normEst;
		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_boundary(new pcl::PointCloud<pcl::PointXYZ>);

		normEst.setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr(cloud));
		normEst.setRadiusSearch(reforn); //设置法向量估计的半径
		normEst.compute(*normals); //将法向量估计的结果保存到normals

		//输出法线的个数
		std::cout << "reforn: " << reforn << std::endl;
		std::cout << "normals: " << normals->size() << std::endl;

		boundEst.setInputCloud(cloud);
		boundEst.setInputNormals(normals);
		boundEst.setRadiusSearch(re);
		boundEst.setAngleThreshold(M_PI / 4);
		//设置搜索方式KdTree
		boundEst.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>));
		boundEst.compute(boundaries);

		//输出边界点的个数
		std::cout << "boundaries: " << boundaries.points.size() << std::endl;
		for (int i = 0; i < cloud->points.size(); ++i)
		{
			if (boundaries[i].boundary_point > 0)
			{
				cloud_boundary->push_back(cloud->points[i]);
				//
				//edge_pnts_idx_.emplace_back(i);
				/*edge_pnts_pos_.emplace_back(make_float3(cloud->points[i].x,
					cloud->points[i].y, cloud->points[i].z));*/
				//#if 1
				//				//保存normal
				//edge_pnts_normal_.emplace_back(make_float3(pnts_normal_[i].x,
				//		pnts_normal_[i].y, pnts_normal_[i].z));
				//#endif
			}
		}

		return boundaries.points.size();
	}
#endif

	bool HWPlane::DoEstimateBordersFromPcl()
	{
		//convert to PointXYZ
		float re = 0.01;
		float reforn = 0.01;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		for (int i = 0; i < pnts_pos_.size(); ++i)
		{
			pcl::PointXYZ pnt_pos(pnts_pos_[i].x, pnts_pos_[i].y, pnts_pos_[i].z);
			cloud->points.emplace_back(pnt_pos);
			//pcl::PointXYZLNormal
		}
		//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
		//pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
		//sor.setInputCloud(cloud);
		//sor.setMeanK(100);
		//sor.setStddevMulThresh(1.0);
		//std::cout << "cloud: " << cloud->points.size() << std::endl;
		//std::cout << "cloud filtered: " << cloud_filtered->points.size() << std::endl;

		EstimateBordersFromPcl(cloud, re, reforn);

		return true;
	}

	int HWPlane::EstimateBordersFromPclAngle(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::Normal>::Ptr & cloud_normal,
		int seach_k, float angle_ratio)
	{
		pcl::PointCloud<pcl::Boundary>::Ptr boundaries(new pcl::PointCloud<pcl::Boundary>);
		pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> boundEst;
		
		boundaries->resize(cloud->size());
		//test
		//for (int i = 0; i < boundaries->points.size(); ++i)
		//{
		//	int boundary_idx = static_cast<int>(boundaries->points[i].boundary_point);
		//	std::cerr << "before i: " << boundary_idx << std::endl;
		//}
		//end test
		boundEst.setInputCloud(cloud);
		boundEst.setInputNormals(cloud_normal);
		pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree_ptr(new pcl::search::KdTree<pcl::PointXYZ>);
		boundEst.setSearchMethod(kdtree_ptr);
		std::cerr << "search_k, angle_ratio: " << seach_k << ", " << angle_ratio << std::endl;
		boundEst.setKSearch(seach_k);
		boundEst.setAngleThreshold(M_PI*angle_ratio);
		boundEst.compute(*boundaries);
		//test (save the boundary points)
		std::vector<Eigen::Vector3f> pnts_edge_pnts;
		int pnts_edge_pnts_num = 0;
		for (int i = 0; i < cloud->size(); ++i)
		{
			if (boundaries->points[i].boundary_point > 0)
			{
				int boundary_idx = static_cast<int>(boundaries->points[i].boundary_point);
				std::cerr << "i: " << boundary_idx << std::endl;
				Eigen::Vector3f boundary_pnt = Eigen::Vector3f(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
				pnts_edge_pnts.emplace_back(boundary_pnt);
				++pnts_edge_pnts_num;
			}
		}
		std::string str_path = "D:/vc_project_new/huawei_data_indoor/thesis_test/polygons_lines_test/scannet_room/scene0062_01_30/30_original_pcl_edge_pnts.obj";
		SavePnts3dIntoObj(str_path, pnts_edge_pnts);
		std::cerr << "pnts_edge_pnts_num: " << pnts_edge_pnts_num << std::endl;
		//end test(save the boundary points)
		return pnts_edge_pnts_num;
	}

	int HWPlane::EstimateBordersFromPclRadius(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::Normal>::Ptr & cloud_normal,
		float seach_radius, float angle_ratio)
	{
		pcl::PointCloud<pcl::Boundary>::Ptr boundaries(new pcl::PointCloud<pcl::Boundary>);
		pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> boundEst;
		boundEst.setInputCloud(cloud);
		boundEst.setInputNormals(cloud_normal);
		boundEst.setRadiusSearch(seach_radius);
		boundEst.setAngleThreshold(M_PI*angle_ratio);
		boundEst.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>));
		boundEst.compute(*boundaries);
		//test (save the boundary points)
		std::vector<Eigen::Vector3f> pnts_edge_pnts;
		int pnts_edge_pnts_num = 0;
		for (int i = 0; i < cloud->size(); ++i)
		{
			if (boundaries->points[i].boundary_point > 0)
			{
				int boundary_idx = static_cast<int>(boundaries->points[i].boundary_point);
				std::cerr << "i: " << boundary_idx << std::endl;
				Eigen::Vector3f boundary_pnt = Eigen::Vector3f(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
				pnts_edge_pnts.emplace_back(boundary_pnt);
				++pnts_edge_pnts_num;
			}
		}
		std::string str_path = "D:/vc_project_new/huawei_data_indoor/thesis_test/polygons_lines_test/scannet_room/scene0062_01_30/30_original_pcl_radius_edge_pnts.obj";
		SavePnts3dIntoObj(str_path, pnts_edge_pnts);
		std::cerr << "pnts_edge_pnts_num: " << pnts_edge_pnts_num << std::endl;
		//end test(save the boundary points)
		return pnts_edge_pnts_num;
	}

	bool HWPlane::DoEstimateBordersFromPclAngle()
	{
		int pnt_num = static_cast<int>(pnts_pos_.size());
		if (pnt_num < 3)
		{
			std::cerr << "failed to estimate border pnts..." << std::endl;
			return false;
		}
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		for (int i = 0; i < pnts_pos_.size(); ++i)
		{
			pcl::PointXYZ pnt_pos(pnts_pos_[i].x, pnts_pos_[i].y, pnts_pos_[i].z);
			cloud->points.emplace_back(pnt_pos);
		}
		//if no normal, compute them. to do next...
		pcl::PointCloud<pcl::Normal>::Ptr cloud_normal(new pcl::PointCloud<pcl::Normal>);
		for (int i = 0; i < pnts_normal_.size(); ++i)
		{
			pcl::Normal pnt_normal(pnts_normal_[i].x, pnts_normal_[i].y, pnts_normal_[i].z);
			cloud_normal->points.emplace_back(pnt_normal);
		}
		int search_knn = 10;
		float angle_ratio = 0.6;
		int pnt_edge_num = EstimateBordersFromPclAngle(cloud, cloud_normal, search_knn, angle_ratio);
		//float d_radius = 0.01;
		//int pnt_edge_num = EstimateBordersFromPclRadius(cloud, cloud_normal, d_radius, angle_ratio);
		if (!pnt_edge_num)
			return false;
		return true;
	}

	//-----------------------------new edge point extract for point cloud------------------//
	bool HWPlane::DoEstimateBorderFrom2DPlane()
	{
		//寻找边缘点
		//float x_min = FLT_MAX;
		//int idx = -1;
		//for (int i = 0; i < plane_coord_pos_.size(); ++i)
		//{
		//	//
		//	if (plane_coord_pos_[i].x < x_min)
		//	{
		//		x_min = plane_coord_pos_[i].x;
		//		idx = i;
		//	}
		//}
		////start idx
		//float2 start_pos = make_float2(plane_coord_pos_[idx].x, plane_coord_pos_[idx].y);
		std::cout << "start compute point resolution..." << std::endl;
		pcl::PointCloud<pcl::PointXY>::Ptr p_cloud(new pcl::PointCloud<pcl::PointXY>); 
		p_cloud->resize(plane_coord_pos_.size());
		for (int i = 0; i < plane_coord_pos_.size(); ++i)
		{
			//printf("%d\n", i);
			p_cloud->points[i].x = plane_coord_pos_[i].x;
			p_cloud->points[i].y = plane_coord_pos_[i].y;
		}
		//获取点云的密度
		std::cout << "the computed point resolution size is：" << plane_coord_pos_.size() << std::endl;
		cloud_resolution_ = ComputeCloudResolution(p_cloud);
		if (std::abs(cloud_resolution_) < 1e-7)
			cloud_resolution_ = 1e-7;

		double alpha = params_.alpha;
		if (alpha == 0) {
			alpha = cloud_resolution_*0.5;
			initial_params_.alpha = alpha;
		}
		std::cout << "alpha = " << alpha << std::endl;
		std::cout << "point resolution is: " << cloud_resolution_ << std::endl;

		//寻找边缘点
		std::list<Point_2d> points_2d;
		std::list<Point_2d>::iterator iter;
		for (int i = 0; i < plane_coord_pos_.size(); ++i)
		{
			Point_2d pnt(plane_coord_pos_[i].x, plane_coord_pos_[i].y);
			points_2d.emplace_back(pnt);
		}

		std::cout << "寻找边缘点结束" << std::endl;
		//std::ofstream fh("D:\\room_sketch\\data\\huawei_data\\huawei_indoor_test\\plane_coord_pos_test.obj");
		//for (iter= points_2d.begin();iter!= points_2d.end();iter++)
		//{
		//	Point_2d tmp = *iter;
		//	fh << "v " << tmp.x() << " " << tmp.y() << " " << "0.0" << std::endl;
		//}
		//fh.close();
		//system("pause");

		Alpha_shape_2 A(points_2d.begin(), points_2d.end(),
			//FT(2 * cloud_resolution),
			FT(alpha),
			Alpha_shape_2::GENERAL);
		//
		//std::list<Point_2d> output_vertex = A.Output();

		////输出边缘顶点的数量
		//std::list<Point_2d>::iterator iter;
		//std::ofstream fh("G:\\xht\\huawei\\2019-04-28_14.54.52\\intial_edge2.obj");
		//for (std::list<Point_2d>::iterator iter = output_vertex.begin(); iter != output_vertex.end(); ++iter)
		//{
		//	Point_2d tmp = *iter;
		//	
		//	fh << "v " << tmp.x() << " " << tmp.y() << " " << "0.0" << std::endl;
		//}
		//fh.close();
		//std::cout << "the edge pnts number is: " << output_vertex.size() << std::endl;
		//std::vector<Segment> border_edges_;

		border_edges_.clear();
		alpha_edges(A, std::back_inserter(border_edges_));
		std::cout << "Alpha Shape computed" << std::endl;
		std::cout << border_edges_.size() << " alpha shape edges" << std::endl;
		int border_edges_num = static_cast<int>(border_edges_.size());
		if (border_edges_num == 0)
		{
			return false;
		}
		std::cout << "Optimal alpha 1: " << *A.find_optimal_alpha(1) << std::endl;
		std::cout << "Optimal alpha 2: " << *A.find_optimal_alpha(2) << std::endl;
		int numberofcomponents = A.number_solid_components();
		std::cout << "the components is: " << numberofcomponents << std::endl;

		alpha_edges_x_.x = 1e5;
		alpha_edges_x_.y = -1e5;
		alpha_edges_y_.x = 1e5;
		alpha_edges_y_.y = -1e5;
		double total_length = 0.0;
		double total_x = 0.0;
		double total_y = 0.0;
		for (int i = 0; i < border_edges_.size(); i++) {
			Point_2d start = border_edges_[i].start();
			Point_2d end = border_edges_[i].end();
			Eigen::Vector2d dist(end.x() - start.x(), end.y() - start.y());
			total_length += dist.norm();
			//std::cout << start.x() << ", " << start.y() << " | " << end.x() << ", " << end.y() << std::endl;
			alpha_edges_x_.x = fminf(alpha_edges_x_.x, border_edges_[i].start().x());
			alpha_edges_x_.y = fmaxf(alpha_edges_x_.y, border_edges_[i].start().x());
			alpha_edges_y_.x = fminf(alpha_edges_y_.x, border_edges_[i].start().y());
			alpha_edges_y_.y = fmaxf(alpha_edges_y_.y, border_edges_[i].start().y());

			alpha_edges_x_.x = fminf(alpha_edges_x_.x, border_edges_[i].end().x());
			alpha_edges_x_.y = fmaxf(alpha_edges_x_.y, border_edges_[i].end().x());
			alpha_edges_y_.x = fminf(alpha_edges_y_.x, border_edges_[i].end().y());
			alpha_edges_y_.y = fmaxf(alpha_edges_y_.y, border_edges_[i].end().y());
		}

		std::cout << "alpha_edges_x_.x = " << alpha_edges_x_.x << std::endl;
		std::cout << "alpha_edges_x_.y = " << alpha_edges_x_.y << std::endl;
		std::cout << "alpha_edges_y_.x = " << alpha_edges_y_.x << std::endl;
		std::cout << "alpha_edges_y_.y = " << alpha_edges_y_.y << std::endl;
		ave_alpha_edge_len_ = total_length / border_edges_.size();
		std::cout << "ave_alpha_edge_len_ = " << ave_alpha_edge_len_ << std::endl;
		////A.number_solid_components
		//int alpha_size = 0;
		//
		//Alpha_shape_2::Alpha_iterator alpha_iter = A.alpha_begin();
		//Alpha_shape_2::All_faces_iterator face_iter = A.all_faces_begin();
		//for (; alpha_iter != A.alpha_end(); ++alpha_iter)
		//{
		//	++alpha_size;
		//}
		//std::cout << "alpha_size size is: " << alpha_size << std::endl;

#ifdef OUTPUT_ALPHA_SHAPE_EDGES
		std::ofstream fh(filename_ + "_alpha_shape_edges_2d.obj");
		for (int i = 0; i < border_edges_.size(); i++)
		{
			Point_2d start_pnt = border_edges_[i].start();
			Point_2d end_pnt = border_edges_[i].end();
			fh << "v " << start_pnt.x() << " " << start_pnt.y() << " " << "0.0" << std::endl;
			fh << "v " << end_pnt.x() << " " << end_pnt.y() << " " << "0.0" << std::endl;
		}
		for (int ii = 0; ii < border_edges_.size(); ii++) {
			fh << "l " << ii * 2 + 1 << " " << ii * 2 + 2 << "\n";
		}
		fh.close();

		std::vector<float3> points_3d;
		for (int i = 0; i < border_edges_.size(); i++) {
			Eigen::Vector4f start_pnt(border_edges_[i].start().x(), border_edges_[i].start().y(), 0.0, 1.0);
			Eigen::Vector4f end_pnt(border_edges_[i].end().x(), border_edges_[i].end().y(), 0.0, 1.0);
			Eigen::Vector4f transform_start = plane_to_world_ * start_pnt;
			Eigen::Vector4f transform_end = plane_to_world_ * end_pnt;

			points_3d.emplace_back(make_float3(transform_start(0), transform_start(1), transform_start(2)));
			points_3d.emplace_back(make_float3(transform_end(0), transform_end(1), transform_end(2)));
		}

		fh.open(filename_ + "alpha_shape_edges_3d.obj");
		for (int i = 0; i < points_3d.size(); i++)
		{
			float3 start_pnt = points_3d[i];
			fh << "v " << start_pnt.x << " " << start_pnt.y << " " << start_pnt.z << std::endl;
		}
		for (int ii = 0; ii < border_edges_.size(); ii++) {
			fh << "l " << ii * 2 + 1 << " " << ii * 2 + 2 << "\n";
		}
		fh.close();
#endif // OUTPUT_ALPHA_SHAPE_EDGES
		//edge_pnts_pos_	//这是腐蚀膨胀的polygon_3d

#if 0
		//这是通过各个边来提取对应的顶点
		std::vector<Point_2d> out_points;
		//std::vector<float3> out_points_vec;
		for (int i = 0; i < segments.size(); ++i)
		{
			Point_2d tmp = segments[i].start();
			out_points.emplace_back(tmp);
			edge_pnts_pos_.emplace_back(make_float3(tmp.x(), tmp.y(), 0.0));
		}
#endif

#if 0	//test 输出点云
		std::ofstream fh1("D:\\room_sketch\\data\\huawei_data\\huawei_indoor_test\\plane_coord_pos_edge_test.obj");
		
		for (std::vector<Segment>::iterator edge_iter = border_edges_.begin(); edge_iter != border_edges_.end(); edge_iter++)
		{
			Segment tmp = *edge_iter;

			fh1 << "v " << tmp.start().x() << " " << tmp.start().y() << " " << "0.0" << std::endl;
			fh1 << "v " << tmp.end().x() << " " << tmp.end().y() << " " << "0.0" << std::endl;
		}
		for (int i = 0; i < border_edges_.size(); ++i)
		{
			fh1 << "l " << 2*i + 1 << " " << 2*i + 2 << std::endl;
		}
		
		fh1.close();
#endif

		//std::cout << "end test!!" << std::endl;
		//system("pause");

		/*std::vector<point2d_type> point_2d_pos;
		for (int i = 0; i < pnts_pos_.size(); ++i)
		{
			point2d_type pnt{ pnts_pos_[i].x, pnts_pos_[i].y};
			point_2d_pos.emplace_back(pnt);
		}
		int end_idx = point_2d_pos.size() - 1;*/
		
		return true;
	}

	bool HWPlane::DoEstimateBorderFrom2DPlaneOptiAlpha()
	{
		std::cout << "start compute point resolution..." << std::endl;
		pcl::PointCloud<pcl::PointXY>::Ptr p_cloud(new pcl::PointCloud<pcl::PointXY>);
		p_cloud->resize(plane_coord_pos_.size());
		for (int i = 0; i < plane_coord_pos_.size(); ++i)
		{
			//printf("%d\n", i);
			p_cloud->points[i].x = plane_coord_pos_[i].x;
			p_cloud->points[i].y = plane_coord_pos_[i].y;
		}
		//获取点云的密度
		std::cout << "the computed point resolution size is：" << plane_coord_pos_.size() << std::endl;
		cloud_resolution_ = ComputeCloudResolution(p_cloud);
		if (std::abs(cloud_resolution_) < 1e-7)
			cloud_resolution_ = 1e-7;
		double alpha = params_.alpha;
		if (alpha == 0) {
			alpha = cloud_resolution_*0.5;
			initial_params_.alpha = alpha;
		}
		//params_.alpha = alpha;
		std::cout << "alpha = " << alpha << std::endl;
		std::cout << "point resolution is: " << cloud_resolution_ << std::endl;
		//寻找边缘点
		std::list<Point_2d> points_2d;
		std::list<Point_2d>::iterator iter;
		for (int i = 0; i < plane_coord_pos_.size(); ++i)
		{
			Point_2d pnt(plane_coord_pos_[i].x, plane_coord_pos_[i].y);
			points_2d.emplace_back(pnt);
		}
		std::cout << "寻找边缘点结束" << std::endl;
		Alpha_shape_2 A(points_2d.begin(), points_2d.end(),
			//FT(2 * cloud_resolution),
			FT(alpha),
			Alpha_shape_2::GENERAL);
		std::cout << "Optimal alpha 1: " << *A.find_optimal_alpha(1) << std::endl;
		alpha = *A.find_optimal_alpha(1)*1.5;
		std::cout << "new alpha = " << alpha << std::endl;
		initial_params_.alpha = alpha;
		params_.alpha = alpha;	//set it 
		Alpha_shape_2 B(points_2d.begin(), points_2d.end(),
			//FT(2 * cloud_resolution),
			FT(alpha),
			Alpha_shape_2::GENERAL);
		border_edges_.clear();
		alpha_edges(B, std::back_inserter(border_edges_));
		std::cout << "Alpha Shape computed" << std::endl;
		std::cout << border_edges_.size() << " alpha shape edges" << std::endl;
		int border_edges_num = static_cast<int>(border_edges_.size());
		if (border_edges_num == 0)
		{
			return false;
		}
		int numberofcomponents = B.number_solid_components();
		std::cout << "the components is: " << numberofcomponents << std::endl;
		alpha_edges_x_.x = 1e5;
		alpha_edges_x_.y = -1e5;
		alpha_edges_y_.x = 1e5;
		alpha_edges_y_.y = -1e5;
		double total_length = 0.0;
		double total_x = 0.0;
		double total_y = 0.0;
		for (int i = 0; i < border_edges_.size(); i++) {
			Point_2d start = border_edges_[i].start();
			Point_2d end = border_edges_[i].end();
			Eigen::Vector2d dist(end.x() - start.x(), end.y() - start.y());
			total_length += dist.norm();
			//std::cout << start.x() << ", " << start.y() << " | " << end.x() << ", " << end.y() << std::endl;
			alpha_edges_x_.x = fminf(alpha_edges_x_.x, border_edges_[i].start().x());
			alpha_edges_x_.y = fmaxf(alpha_edges_x_.y, border_edges_[i].start().x());
			alpha_edges_y_.x = fminf(alpha_edges_y_.x, border_edges_[i].start().y());
			alpha_edges_y_.y = fmaxf(alpha_edges_y_.y, border_edges_[i].start().y());
			alpha_edges_x_.x = fminf(alpha_edges_x_.x, border_edges_[i].end().x());
			alpha_edges_x_.y = fmaxf(alpha_edges_x_.y, border_edges_[i].end().x());
			alpha_edges_y_.x = fminf(alpha_edges_y_.x, border_edges_[i].end().y());
			alpha_edges_y_.y = fmaxf(alpha_edges_y_.y, border_edges_[i].end().y());
		}
		std::cout << "alpha_edges_x_.x = " << alpha_edges_x_.x << std::endl;
		std::cout << "alpha_edges_x_.y = " << alpha_edges_x_.y << std::endl;
		std::cout << "alpha_edges_y_.x = " << alpha_edges_y_.x << std::endl;
		std::cout << "alpha_edges_y_.y = " << alpha_edges_y_.y << std::endl;
		ave_alpha_edge_len_ = total_length / border_edges_.size();
		std::cout << "ave_alpha_edge_len_ = " << ave_alpha_edge_len_ << std::endl;
#ifdef OUTPUT_ALPHA_SHAPE_EDGES
		std::ofstream fh(filename_ + "_alpha_shape_edges_opti_2d.obj");
		for (int i = 0; i < border_edges_.size(); i++)
		{
			Point_2d start_pnt = border_edges_[i].start();
			Point_2d end_pnt = border_edges_[i].end();
			fh << "v " << start_pnt.x() << " " << start_pnt.y() << " " << "0.0" << std::endl;
			fh << "v " << end_pnt.x() << " " << end_pnt.y() << " " << "0.0" << std::endl;
		}
		for (int ii = 0; ii < border_edges_.size(); ii++) {
			fh << "l " << ii * 2 + 1 << " " << ii * 2 + 2 << "\n";
		}
		fh.close();

		std::vector<float3> points_3d;
		for (int i = 0; i < border_edges_.size(); i++) {
			Eigen::Vector4f start_pnt(border_edges_[i].start().x(), border_edges_[i].start().y(), 0.0, 1.0);
			Eigen::Vector4f end_pnt(border_edges_[i].end().x(), border_edges_[i].end().y(), 0.0, 1.0);
			Eigen::Vector4f transform_start = plane_to_world_ * start_pnt;
			Eigen::Vector4f transform_end = plane_to_world_ * end_pnt;

			points_3d.emplace_back(make_float3(transform_start(0), transform_start(1), transform_start(2)));
			points_3d.emplace_back(make_float3(transform_end(0), transform_end(1), transform_end(2)));
		}

		fh.open(filename_ + "_alpha_shape_edges_opti_3d.obj");
		for (int i = 0; i < points_3d.size(); i++)
		{
			float3 start_pnt = points_3d[i];
			fh << "v " << start_pnt.x << " " << start_pnt.y << " " << start_pnt.z << std::endl;
		}
		for (int ii = 0; ii < border_edges_.size(); ii++) {
			fh << "l " << ii * 2 + 1 << " " << ii * 2 + 2 << "\n";
		}
		fh.close();
#endif // OUTPUT_ALPHA_SHAPE_EDGES
		//edge_pnts_pos_	//这是腐蚀膨胀的polygon_3d
	}

	void HWPlane::SortBorderEdgesFromPlaneOptiAlpha()
	{
		if (border_edges_.empty())
		{
			//std::cout << "1111111111111111111" << std::endl;
			//system("pause");
			return;
		}
		//std::vector<Segment> sorted_edges;
		std::vector<bool> traverse_vec;
		traverse_vec.resize(border_edges_.size());
		for (int i = 0; i < border_edges_.size(); ++i)
		{
			traverse_vec[i] = false;
		}

		/*std::ofstream fh("D:\\room_sketch\\data\\huawei_data\\huawei_outdoor_test\\outdoor_unsorted_edges_1.obj");
		for (int i = 0; i < border_edges_.size(); ++i)
		{
		fh << "v " << border_edges_[i].start().x() << " " << border_edges_[i].start().y() << " " << "0.0" << std::endl;
		}
		fh.close();*/

		float max_polygon_length = 0.0;

#if 0	//新的获取polygon
#else
		for (int idx = 0; idx < border_edges_.size(); ++idx) {
			if (traverse_vec[idx] == true)
				continue;
			Segment start_edge = border_edges_[idx];
			traverse_vec[idx] = true;
			std::vector<Segment> sorted_border_edges;
			sorted_border_edges.emplace_back(start_edge);
			//sorted_border_edges_2d_.emplace_back(start_edge);
			int count = 0;
			bool find_next_edge;
			do
			{
				Point_2d prev_end_pnt = start_edge.end();
				//std::cout << "prev_end_pnt: " << prev_end_pnt.x()<< " " << prev_end_pnt.y() << std::endl;

				find_next_edge = false;
				for (int i = 0; i < border_edges_.size(); ++i)
				{
					//寻找下一条边，让边按照某种顺序排列
					Point_2d start_pnt = border_edges_[i].start();
					Point_2d end_pnt = border_edges_[i].end();

					if (!traverse_vec[i] && CheckSameSegmentPnts(start_pnt, prev_end_pnt))
					{
						//std::cout << "i: " << i << std::endl;
						start_edge = border_edges_[i];
						sorted_border_edges.emplace_back(start_edge);
						traverse_vec[i] = true;
						find_next_edge = true;
						break;
					}
					else if (!traverse_vec[i] && CheckSameSegmentPnts(end_pnt, prev_end_pnt))
					{
						//std::cout << "i: " << i << std::endl;
						//构建新的segment
						Segment start_new_edge(end_pnt, start_pnt);
						start_edge = start_new_edge;
						sorted_border_edges.emplace_back(start_edge);
						traverse_vec[i] = true;
						find_next_edge = true;
						break;
					}
				}
				if (!find_next_edge)
				{
#ifdef LIULINGFEI
					std::cout << "the polygon is not closed, therefore to do next...." << std::endl;
#endif // LIULINGFEI
					if (sorted_border_edges_2d_.size() < sorted_border_edges.size())
						sorted_border_edges_2d_.swap(sorted_border_edges);
					break;
				}
				++count;
				//system("pause");
			} while (find_next_edge);
			//} while (count < border_edges_.size() && !CheckSameSegmentEdges(start_edge, border_edges_[0]));
		}
#endif

#ifdef OUTPUT_ALPHA_SHAPE_EDGES
		std::ofstream fh(filename_ + "_alpha_shape_edges_sorted_opti_2d.obj");
		for (int i = 0; i < sorted_border_edges_2d_.size(); i++)
		{
			Point_2d start_pnt = sorted_border_edges_2d_[i].start();
			Point_2d end_pnt = sorted_border_edges_2d_[i].end();
			fh << "v " << start_pnt.x() << " " << start_pnt.y() << " " << "0.0" << std::endl;
			fh << "v " << end_pnt.x() << " " << end_pnt.y() << " " << "0.0" << std::endl;
		}
		for (int ii = 0; ii < sorted_border_edges_2d_.size(); ii++) {
			fh << "l " << ii * 2 + 1 << " " << ii * 2 + 2 << "\n";
		}
		fh.close();

		std::vector<float3> points_3d;
		for (int i = 0; i < sorted_border_edges_2d_.size(); i++) {
			Eigen::Vector4f start_pnt(sorted_border_edges_2d_[i].start().x(), sorted_border_edges_2d_[i].start().y(), 0.0, 1.0);
			Eigen::Vector4f end_pnt(sorted_border_edges_2d_[i].end().x(), sorted_border_edges_2d_[i].end().y(), 0.0, 1.0);
			Eigen::Vector4f transform_start = plane_to_world_ * start_pnt;
			Eigen::Vector4f transform_end = plane_to_world_ * end_pnt;
			points_3d.emplace_back(make_float3(transform_start(0), transform_start(1), transform_start(2)));
			points_3d.emplace_back(make_float3(transform_end(0), transform_end(1), transform_end(2)));
		}
		fh.open(filename_ + "_alpha_shape_edges_sorted_opti_3d.obj");
		for (int i = 0; i < points_3d.size(); i++)
		{
			float3 start_pnt = points_3d[i];
			fh << "v " << start_pnt.x << " " << start_pnt.y << " " << start_pnt.z << std::endl;
		}
		for (int ii = 0; ii < border_edges_.size(); ii++) {
			fh << "l " << ii * 2 + 1 << " " << ii * 2 + 2 << "\n";
		}
		fh.close();
#endif // OUTPUT_ALPHA_SORTED_SHAPE_EDGES

	}

	bool HWPlane::GetPolygonPointFromSortedBorderEdges()
	{
		if (sorted_border_edges_2d_.empty())
			return false;
		edge_pnts_pos_.clear();
		for (int i = 0; i < sorted_border_edges_2d_.size(); ++i)
		{
			Point_2d start_pnt = sorted_border_edges_2d_[i].start();
			float2 tmp_pnt = make_float2(start_pnt.x(), start_pnt.y());
			edge_pnts_pos_.emplace_back(tmp_pnt);
		}
#ifdef OUTPUT_ALPHA_SHAPE_EDGES
		std::string path_str2d = filename_ + "_alpha_shape_edges_sorted_opti_edge_pnts_2d.obj";
		std::ofstream fh(path_str2d);
		for (int i = 0; i < edge_pnts_pos_.size(); i++)
		{
			float2 start_pnt = edge_pnts_pos_[i];
			fh << "v " << start_pnt.x << " " << start_pnt.y << " " << 0 << std::endl;
		}
		for (int ii = 0; ii < edge_pnts_pos_.size() - 1; ii++) {
			fh << "l " << ii + 1 << " " << ii + 2 << "\n";
		}
		fh << "l " << 1 << " " << edge_pnts_pos_.size() << "\n";
		fh.close();

		std::vector<float3> points_3d;
		for (int i = 0; i < edge_pnts_pos_.size(); i++) {
			Eigen::Vector4f start_pnt(edge_pnts_pos_[i].x, edge_pnts_pos_[i].y, 0.0, 1.0);
			Eigen::Vector4f transform_start = plane_to_world_ * start_pnt;
			points_3d.emplace_back(make_float3(transform_start(0), transform_start(1), transform_start(2)));
		}
		fh.open(filename_ + "_alpha_shape_edges_sorted_opti_edge_pnts_3d.obj");
		for (int i = 0; i < points_3d.size(); i++)
		{
			float3 start_pnt = points_3d[i];
			fh << "v " << start_pnt.x << " " << start_pnt.y << " " << start_pnt.z << std::endl;
		}
		for (int ii = 0; ii < edge_pnts_pos_.size() - 1; ii++) {
			fh << "l " << ii + 1 << " " << ii + 2 << "\n";
		}
		fh << "l " << 1 << " " << edge_pnts_pos_.size() << "\n";
		fh.close();
#endif // OUTPUT_ALPHA_SORTED_SHAPE_EDGES
		return true;
	}

	float2 HWPlane::TransformWorld2Plane(Eigen::Vector3f & world_coord)
	{
		Eigen::Vector4f pnt(world_coord(0), world_coord(1), world_coord(2), 1.0f);
		Eigen::Vector4f plane_coord = world_to_plane_*pnt;
		return make_float2(plane_coord(0), plane_coord(1));
	}

	void HWPlane::initialAfterLoad()
	{		
		Eigen::Vector3f edge1, edge2; 
		int idx;
		for (idx = 1; idx < corner_pnts_3d_.size() - 1; idx++) {
			float3 e1 = corner_pnts_3d_[idx] - corner_pnts_3d_[idx - 1];
			float3 e2 = corner_pnts_3d_[idx + 1] - corner_pnts_3d_[idx];
			edge1 = Eigen::Vector3f(e1.x, e1.y, e1.z);
			edge2 = Eigen::Vector3f(e2.x, e2.y, e2.z);
			edge1.normalize();
			edge2.normalize();
			if (abs(edge1.dot(edge2)) < 0.9)
				break;
		}
		Eigen::Vector3f normal = edge1.cross(edge2);
		//相当于一个相机坐标到世界坐标的变换矩阵，左上角3x3矩阵的每一列代表相机中的每个轴的方向
		plane_to_world_.block(0, 0, 3, 1) = edge1;
		plane_to_world_.block(0, 1, 3, 1) = edge1.cross(normal);
		plane_to_world_.block(0, 2, 3, 1) = normal;
		//相机的位置设置为顶点idx的坐标
		plane_to_world_.block(0, 3, 3, 1) = Eigen::Vector3f(corner_pnts_3d_[idx].x, corner_pnts_3d_[idx].y, corner_pnts_3d_[idx].z);
		plane_to_world_.row(3) = Eigen::RowVector4f(0, 0, 0, 1);
		world_to_plane_ = plane_to_world_.inverse();
		coeff_.x = normal.x();
		coeff_.y = normal.y();
		coeff_.z = normal.z();
		coeff_.w = -normal.dot(Eigen::Vector3f(corner_pnts_3d_[idx].x, corner_pnts_3d_[idx].y, corner_pnts_3d_[idx].z));
		boxMin_2d_ = Eigen::Vector2f(FLT_MAX, FLT_MAX), boxMax_2d_ = Eigen::Vector2f(-FLT_MAX, -FLT_MAX);
		for (int i = 0; i < corner_pnts_3d_.size(); i++) {
			Eigen::Vector4f pnt(corner_pnts_3d_[i].x, corner_pnts_3d_[i].y, corner_pnts_3d_[i].z, 1.0);
			Eigen::Vector4f transform_pnt = world_to_plane_*pnt;
			corner_pnts_.emplace_back(make_float2(transform_pnt(0), transform_pnt(1)));
		}
		if (IsConcaveVertex(idx)) {
			normal *= -1;
			plane_to_world_.block(0, 1, 3, 1) = edge1.cross(normal);
			plane_to_world_.block(0, 2, 3, 1) = normal;
			world_to_plane_ = plane_to_world_.inverse();
			coeff_.x = normal.x();
			coeff_.y = normal.y();
			coeff_.z = normal.z();
			coeff_.w = -normal.dot(Eigen::Vector3f(corner_pnts_3d_[1].x, corner_pnts_3d_[1].y, corner_pnts_3d_[1].z));
			for (int i = 0; i < corner_pnts_3d_.size(); i++) {
				Eigen::Vector4f pnt(corner_pnts_3d_[i].x, corner_pnts_3d_[i].y, corner_pnts_3d_[i].z, 1.0);
				Eigen::Vector4f transform_pnt = world_to_plane_*pnt;
				corner_pnts_[i] = make_float2(transform_pnt(0), transform_pnt(1));
			}
		}
#ifdef LIULINGFEI
		std::cout << "begin update information" << std::endl;
#endif

		pnts_normal_.push_back(make_float3(normal(0), normal(1), normal(2)));
		pnts_normal_origin_.push_back(make_float3(normal(0), normal(1), normal(2)));
		UpdateInformation();
		Eigen::Vector3f plane_normal = GetPlaneNormal();
		if (abs(plane_normal.z()) > 0.9)
			if (corner_pnts_3d_[0].z > 1.0f) semantic_label_ = 0; //Ceiling
			else semantic_label_ = 1; //Floor
		else semantic_label_ = 2; //Wall
	}

	void HWPlane::SetPolygonPnts3dFromInitialPnts(std::vector<Eigen::Vector3f>& polygon3d_pnts)
	{
		corner_pnts_3d_.clear();
		for (int i = 0; i < polygon3d_pnts.size(); ++i)
		{
			corner_pnts_3d_.emplace_back(make_float3(polygon3d_pnts[i][0], polygon3d_pnts[i][1], polygon3d_pnts[i][2]));
		}
		//寻找polygon中的一个线段作为
		Eigen::Vector3f edge1, edge2;
		int idx;
		for (idx = 1; idx < corner_pnts_3d_.size() - 1; idx++) {
			float3 e1 = corner_pnts_3d_[idx] - corner_pnts_3d_[idx - 1];
			float3 e2 = corner_pnts_3d_[idx + 1] - corner_pnts_3d_[idx];
			edge1 = Eigen::Vector3f(e1.x, e1.y, e1.z);
			edge2 = Eigen::Vector3f(e2.x, e2.y, e2.z);
			edge1.normalize();
			edge2.normalize();
			if (abs(edge1.dot(edge2)) < 0.9)
				break;
		}
		Eigen::Vector3f z_normal = edge1.cross(edge2);	//这是z轴
		Eigen::Vector3f x_aix = edge1;
		Eigen::Vector3f y_aix = z_normal.cross(x_aix);	//z叉乘x为y轴
		//相机坐标系的为z轴为负的
		//Eigen::Vector3f z_normal_vers = -z_normal;
		//上面为物体内部的坐标系，x_aix,y_aix,z_aix，转化为物体坐标(1,0,0),(0,1,0)(0,0,1)系
		//先将物体移动到原点
		Eigen::Matrix4f T_view = Eigen::Matrix4f::Identity();
		//这是以其中某个点为原点的坐标系
		T_view.block(0, 3, 3, 1) = Eigen::Vector3f(-corner_pnts_3d_[idx].x, -corner_pnts_3d_[idx].y, -corner_pnts_3d_[idx].z);
		//我们需要z轴取反来模拟相机坐标系
		//rotate x_aix->(1,0,0), y_aix->(0,1,0), z_aix->(0,0,1)
		//R_verse它表示将旋转向量x_aix<-(1,0,0), y_aix<-(0,1,0), z_aix<-(0,0,1)
		Eigen::Matrix4f R_verse = Eigen::Matrix4f::Identity();
		R_verse.block(0, 0, 3, 1) = x_aix;
		R_verse.block(0, 1, 3, 1) = y_aix;
		R_verse.block(0, 2, 3, 1) = z_normal;
		//R_verse.block(0, 3, 3, 1) = Eigen::Vector3f(0.0, 0.0, 0.0);
		//计算它的
		Eigen::Matrix4f R_view = R_verse.transpose();
		Eigen::Matrix4f RT_view = R_view*T_view;	//它是世界坐标系到平面坐标系的矩阵
		world_to_plane_ = RT_view;
		plane_to_world_ = world_to_plane_.inverse();
		coeff_.x = z_normal.x();
		coeff_.y = z_normal.y();
		coeff_.z = z_normal.z();
		coeff_.w = -z_normal.dot(Eigen::Vector3f(corner_pnts_3d_[idx].x, corner_pnts_3d_[idx].y, corner_pnts_3d_[idx].z));
		boxMin_2d_ = Eigen::Vector2f(FLT_MAX, FLT_MAX), boxMax_2d_ = Eigen::Vector2f(-FLT_MAX, -FLT_MAX);
		for (int i = 0; i < corner_pnts_3d_.size(); i++) {
			Eigen::Vector4f pnt(corner_pnts_3d_[i].x, corner_pnts_3d_[i].y, corner_pnts_3d_[i].z, 1.0);
			Eigen::Vector4f transform_pnt = world_to_plane_*pnt;
			corner_pnts_.emplace_back(make_float2(transform_pnt(0), transform_pnt(1)));
		}
		if (IsConcaveVertex(idx)) {
			z_normal = -edge1.cross(edge2);
			plane_to_world_.block(0, 1, 3, 1) = edge1.cross(z_normal);
			plane_to_world_.block(0, 2, 3, 1) = z_normal;
			world_to_plane_ = plane_to_world_.inverse();
			coeff_.x = z_normal.x();
			coeff_.y = z_normal.y();
			coeff_.z = z_normal.z();
			coeff_.w = -z_normal.dot(Eigen::Vector3f(corner_pnts_3d_[1].x, corner_pnts_3d_[1].y, corner_pnts_3d_[1].z));
			for (int i = 0; i < corner_pnts_3d_.size(); i++) {
				Eigen::Vector4f pnt(corner_pnts_3d_[i].x, corner_pnts_3d_[i].y, corner_pnts_3d_[i].z, 1.0);
				Eigen::Vector4f transform_pnt = world_to_plane_*pnt;
				corner_pnts_[i] = make_float2(transform_pnt(0), transform_pnt(1));
			}
		}
		//Eigen::Vector
		pnts_normal_.push_back(make_float3(z_normal(0), z_normal(1), z_normal(2)));
		pnts_normal_origin_.push_back(make_float3(z_normal(0), z_normal(1), z_normal(2)));
		//UpdateInformation();
		Eigen::Vector3f plane_normal = GetPlaneNormal();
		if (abs(plane_normal.z()) > 0.9)
			if (corner_pnts_3d_[0].z > 1.0f) semantic_label_ = 0; //Ceiling
			else semantic_label_ = 1; //Floor
		else semantic_label_ = 2; //Wall
	}

	//输出是(a,b)，代表直线ax+by=1
	Eigen::Vector2f HWPlane::FittingLine(std::vector<int>& neighs)
	{
		Eigen::Matrix2f A;
		A.setZero();
		Eigen::Vector2f B;
		B.setZero();
		for (int row = 0; row < 2; row++) {
			for (int col = 0; col < 2; col++) {
				for (int i = 0; i < neighs.size(); i++) {
					A(row, col) += pow(edge_pnts_pos_[neighs[i]].x, 2 - row - col)*pow(edge_pnts_pos_[neighs[i]].y, row + col);
				}
			}
		}
		for (int row = 0; row < 2; row++) {
			for (int i = 0; i < neighs.size(); i++) {
				B(row) += pow(edge_pnts_pos_[neighs[i]].x,1 - row)*pow(edge_pnts_pos_[neighs[i]].y, row);
			}
		}
		//求解
		// |sum(x*x) sum(x*y)| |a|   |sum(x)|
		// |				 | | | = |      |
		// |sum(x*y) sum(y*y)| |b|   |sum(y)|
		Eigen::Vector2f X = A.ldlt().solve(B);
		//Eigen::Vector2f X = A.inverse()*B;
		//std::cout << "coeff: " << X(0)<<"  "<<X(1) << std::endl;
		/*if (X(1) == 0) {
			for (int i = 0; i < neighs.size(); i++) {
				std::cout << neighs[i] << " ";
			}
			std::cout << "\nA: " << A << "\n";
			std::cout << "B: " << B.transpose() << "\n";
			system("pause");
		}*/
		return X;
	}

	Eigen::Vector2f HWPlane::FittingLine(std::vector<float2> edge_pnts, std::vector<int>& neighs)
	{
		Eigen::Matrix2f A;
		A.setZero();
		Eigen::Vector2f B;
		B.setZero();
		for (int row = 0; row < 2; row++) {
			for (int col = 0; col < 2; col++) {
				for (int i = 0; i < neighs.size(); i++) {
					A(row, col) += pow(edge_pnts[neighs[i]].x, 2 - row - col) * pow(edge_pnts[neighs[i]].y, row + col);
				}
			}
		}
		for (int row = 0; row < 2; row++) {
			for (int i = 0; i < neighs.size(); i++) {
				B(row) += pow(edge_pnts[neighs[i]].x, 1 - row) * pow(edge_pnts[neighs[i]].y, row);
			}
		}
		//求解
		// |sum(x*x) sum(x*y)| |a|   |sum(x)|
		// |				 | | | = |      |
		// |sum(x*y) sum(y*y)| |b|   |sum(y)|
		Eigen::Vector2f X = A.ldlt().solve(B);
		//Eigen::Vector2f X = A.inverse()*B;
		//std::cout << "coeff: " << X(0)<<"  "<<X(1) << std::endl;
		/*if (X(1) == 0) {
			for (int i = 0; i < neighs.size(); i++) {
				std::cout << neighs[i] << " ";
			}
			std::cout << "\nA: " << A << "\n";
			std::cout << "B: " << B.transpose() << "\n";
			system("pause");
		}*/
		return X;
	}

	Eigen::Vector2f HWPlane::FittingLine(float2 & p1, float2 & p2)
	{
		Eigen::Matrix2f A;
		A << p1.x, p1.y, p2.x, p2.y;
		Eigen::Vector2f B(1, 1);
		Eigen::Vector2f coeff = A.inverse()*B;
		return coeff;
	}

	Eigen::Vector2f HWPlane::FittingLineEigen(Eigen::Vector2f& p1, Eigen::Vector2f& p2)
	{
		Eigen::Matrix2f A;
		A << p1[0], p1[1], p2[0], p2[1];
		Eigen::Vector2f B(1, 1);
		Eigen::Vector2f coeff = A.inverse()*B;
		return coeff;
	}

	// Ax+By+C = 0,
	Eigen::Vector3f HWPlane::FittingLineFromPntsIdxs(const std::vector<int>& idxs)
	{
		int pnts_num = static_cast<int>(idxs.size());
		if (pnts_num < 2)
		{
			std::cerr << "invalid pnts number..." << std::endl;
			return Eigen::Vector3f(0, 0, 0);
		}
		std::vector<Eigen::Vector2f> pnts2d;
		for (int i = 0; i < idxs.size(); ++i)
		{
			int idx = idxs[i];
			Eigen::Vector2f pnt = Eigen::Vector2f(edge_pnts_pos_[idx].x, edge_pnts_pos_[idx].y);
			pnts2d.emplace_back(pnt);
		}
		float sum_x = 0.0, sum_y = 0.0, sum_xy = 0.0, sum_x2 = 0.0, sum_y2 = 0.0;
		//fitting the pnts into line
		for (int i = 0; i < pnts2d.size(); ++i)
		{
			sum_x += pnts2d[i][0];
			sum_y += pnts2d[i][1];
			sum_xy += pnts2d[i][0] * pnts2d[i][1];
			sum_x2 += pnts2d[i][0] * pnts2d[i][0];
			sum_y2 += pnts2d[i][1] * pnts2d[i][1];
		}

		float a_x = sum_x / pnts2d.size();
		float a_y = sum_y / pnts2d.size();
		float a_xy = sum_xy / pnts2d.size();
		float a_x2 = sum_x2 / pnts2d.size();
		float a_y2 = sum_y2 / pnts2d.size();
		float A = -(a_xy - a_x*a_y);
		float B;
		float Bx = a_x2 - a_x*a_x;
		float By = a_y2 - a_y*a_y;
		if (std::abs(Bx) < std::abs(By))
		{
			//Line is more Vertical
			B = By;
			std::swap(A, B);
		}
		else
		{
			//Line is more Horizotal
			B = Bx;
		}
		float C = -(A*a_x + B*a_y);
		Eigen::Vector3f func_coeff = Eigen::Vector3f(A, B, C);
		return func_coeff;
	}

	Eigen::Vector3f HWPlane::CreateLineFunctionFrom2Pnts(float2& p1, float2& p2)
	{
		Eigen::Vector3f f2d(0.0, 0.0, 0.0);
		Eigen::Vector2f l_s(p1.x, p1.y);
		Eigen::Vector2f l_e(p2.x, p2.y);
		if (std::abs((l_e - l_s).norm()) < 1e-6)
			return Eigen::Vector3f(0.0, 0.0, 0.0);
		f2d[0] = l_e[1] - l_s[1];
		f2d[1] = l_s[0] - l_e[0];
		f2d[2] = l_e[0] * l_s[1] - l_s[0] * l_e[1];
		return f2d;
	}


	Eigen::Vector3f HWPlane::GetIntersectionPoint3dWithLine(Eigen::Vector3f L_dir, Eigen::Vector3f L_pnt)
	{
		Eigen::Vector3f plane_normal = GetPlaneNormal().normalized();
		printf("plane_normal = (%f, %f, %f), L_dir = (%f, %f, %f)\n",
			plane_normal(0), plane_normal(1), plane_normal(2), L_dir(0), L_dir(1), L_dir(2));
		Eigen::Vector3f L_dir_normalized = L_dir.normalized();
		float cos_val = plane_normal.dot(L_dir_normalized);

		Eigen::Vector4f p1(L_pnt(0), L_pnt(1), L_pnt(2), 1.0f);
		//Eigen::Vector4f p2(corner_pnts_3d_[0].x, corner_pnts_3d_[0].y, corner_pnts_3d_[0].z, 1.0f);
		//std::cout << "world_to_plane_:\n" << world_to_plane_ << std::endl;
		Eigen::Vector3f plane_pnt1 = (world_to_plane_ * p1).head(3);
		//Eigen::Vector3f plane_pnt2 = (world_to_plane_ * p2).head(3);
		//printf("(world_to_plane*L_pnt).z = %f\n", plane_pnt1(2));
		//printf("(world_to_plane*corner_pnts_3d_[0]).z = %f\n", plane_pnt2(2));

		//float3 pnt = make_float3(L_pnt(0), L_pnt(1), L_pnt(2));
		//float3 seg = pnt - corner_pnts_3d_[0];
		//float seg_len = sqrt(dot(seg, seg));
		//float temp_cos_val = Eigen::Vector3f(seg.x, seg.y, seg.z).dot(plane_normal) / seg_len;
		//float dist = seg_len * temp_cos_val;
		float dist = plane_pnt1.z();
		//printf("L_pnt = (%f, %f, %f), dist = %f, cos_val = %f\n",
			//L_pnt.x(), L_pnt.y(), L_pnt.z(),
			//dist, cos_val
		//);
		Eigen::Vector3f result = L_pnt - dist / cos_val * L_dir_normalized;

		return result;
	}

	//两条直线分别为ax+by=1和cx+dy=1
	float2 HWPlane::GetIntersectionPoint(Eigen::Vector2f & l1, Eigen::Vector2f & l2)
	{
		float2 p;
		float temp = (l1(0)*l2(1) - l1(1)*l2(0));
		p.y = (l1(0) - l2(0)) / temp;
		if (l1(0) == 0) p.x = (1 - l2(1)*p.y) / l2(0);
		else p.x = (1 - l1(1)*p.y) / l1(0);
		return p;
	}

	float HWPlane::dot(float2 p1, float2 p2)
	{
		return p1.x*p2.x+p1.y*p2.y;
	}

	float HWPlane::dot(float3 p1, float3 p2)
	{
		return p1.x * p2.x + p1.y * p2.y + p1.z * p2.z;
	}

	float HWPlane::PointPointDist(float2 & p1, float2 & p2)
	{
		float2 l = p1 - p2;
		return sqrt(l.x*l.x + l.y*l.y);
	}

	float HWPlane::PointLineDist(float2 & p, Eigen::Vector2f & coeff)
	{
		float d = ((coeff(0)*p.x + coeff(1)*p.y - 1) / sqrt(coeff(0)*coeff(0) + coeff(1)*coeff(1)));
		return d;
	}

	float HWPlane::PointLineSegmentDist(float2 & p, float2 & p1, float2 & p2)
	{
		float2 l1 = p1 - p, l2 = p2 - p, l = p2 - p1;
		if (dot(l1, l) >= 0)
			return sqrt(l1.x*l1.x + l1.y*l1.y);
		else if (dot(l2, l)<=0)
			return sqrt(l2.x*l2.x + l2.y*l2.y);
		else {
			float theta = GetTheta(-l1.x, -l1.y, l.x, l.y);
			float d = sqrt(l1.x*l1.x + l1.y*l1.y)*sin(theta/180*MATH_PI);
			return d;
			//return abs(l1.x*l.y - l1.y*l.x) / sqrt(l1.x*l1.x + l1.y*l1.y);
		}
	}

	float2 HWPlane::ProjectToLine(float2 & p, Eigen::Vector2f & coeff)
	{
		float2 p1;
		if (coeff(0) == 0) {
			p1.x = p.x;
			p1.y = 1 / coeff(1);
		}
		else if (coeff(1) == 0) {
			p1.y = p.y;
			p1.x = 1 / coeff(0);
		}
		else {
			p1.x = (coeff(0) + pow(coeff(1), 2)*p.x - coeff(0)*coeff(1)*p.y) / (pow(coeff(0), 2) + pow(coeff(1), 2));
			p1.y = (coeff(1) + pow(coeff(0), 2)*p.y - coeff(0)*coeff(1)*p.x) / (pow(coeff(0), 2) + pow(coeff(1), 2));
			//p1.y = (1 - coeff(0)*p.x) / coeff(1);
		}
		return p1;
	}

	void HWPlane::SetSemanticLabel(int label)
	{
		semantic_label_ = label;
	}

	void HWPlane::SetDiameter(float diameter_max)
	{
		diameter_max_ = diameter_max;
		std::cout << "set diameter: " << diameter_max_ << std::endl;
	}

	void HWPlane::SetCornerPt(float2 pnt, int idx)
	{
		corner_pnts_last_state_ = corner_pnts_;
		corner_angels_last_state_ = corner_angels_;
		min_dists_last_state_ = min_dists_;
		corner_pnts_3d_last_state_ = corner_pnts_3d_;
		triangles_idx_last_state_ = triangles_idx_;
		corner_pnts_[idx] = pnt;
		Eigen::Vector4f transform_pnt = plane_to_world_*Eigen::Vector4f(pnt.x, pnt.y, 0, 1);
		corner_pnts_3d_[idx] = make_float3(transform_pnt(0), transform_pnt(1), transform_pnt(2));
		UpdateInformation();
	}

	void HWPlane::SetCornerPt3d(Eigen::Vector3f& pnt3d, int idx)
	{
		if (corner_pnts_3d_.size() <= idx || idx < 0)
			return;
		corner_pnts_last_state_ = corner_pnts_;
		corner_angels_last_state_ = corner_angels_;
		min_dists_last_state_ = min_dists_;
		corner_pnts_3d_last_state_ = corner_pnts_3d_;
		triangles_idx_last_state_ = triangles_idx_;
		float3 tmppnt3d = make_float3(pnt3d[0], pnt3d[1], pnt3d[2]);
		corner_pnts_3d_[idx] = tmppnt3d;
		Eigen::Vector2f tmp_pnt2d;
		Pnt3d2Pnt2D(pnt3d, tmp_pnt2d);
		float2 tmppnt2d = make_float2(tmp_pnt2d[0], tmp_pnt2d[1]);
		corner_pnts_[idx] = tmppnt2d;
		UpdateInformation();
	}

	void HWPlane::SetCornerPts(std::vector<float3> & corner_pnts_3d)
	{
		corner_pnts_last_state_ = corner_pnts_;
		corner_angels_last_state_ = corner_angels_;
		min_dists_last_state_ = min_dists_;
		corner_pnts_3d_last_state_ = corner_pnts_3d_;
		triangles_idx_last_state_ = triangles_idx_;
		corner_pnts_3d_ = corner_pnts_3d;
		std::cout << "set corner_pnts_3d_" << std::endl;
		for (int i = 0; i < corner_pnts_3d_.size(); i++) {
			std::cout << i << ": " << corner_pnts_3d_[i].x << ", " << corner_pnts_3d_[i].y << ", " << corner_pnts_3d_[i].z << std::endl;
		}
		if (corner_pnts_3d_last_state_.size() == 0&& pnts_pos_.size()!=0) {
			float3 edge1 = corner_pnts_3d_[1] - corner_pnts_3d_[0];
			float3 edge2 = corner_pnts_3d_[2] - corner_pnts_3d_[1];
			Eigen::Vector3f e1(edge1.x, edge1.y, edge1.z), e2(edge2.x, edge2.y, edge2.z);
			Eigen::Vector3f polygon_normal = e1.cross(e2);
			if (polygon_normal.dot(average_pnts_normal_) <= 0) {
				std::vector<float3> corner_pnts_3d_inverse;
				for (int i = corner_pnts_3d_.size() - 1; i >= 0; i--) {
					corner_pnts_3d_inverse.emplace_back(corner_pnts_3d_[i]);
				}
				corner_pnts_3d_ = corner_pnts_3d_inverse;
			}
		}
		std::vector<float2> corner_pnts;
		//GenerateWorldCoordToPlaneCoordMatrix();
		for (int i = 0; i < corner_pnts_3d_.size(); i++) {
			Eigen::Vector4f pnt(corner_pnts_3d_[i].x, corner_pnts_3d_[i].y, corner_pnts_3d_[i].z, 1.0);
			Eigen::Vector4f transform_pnt = world_to_plane_*pnt;
			corner_pnts.emplace_back(make_float2(transform_pnt(0), transform_pnt(1)));			
			Eigen::Vector4f transform_pnt2 = plane_to_world_*Eigen::Vector4f(transform_pnt(0), transform_pnt(1), 0, 1);
			corner_pnts_3d_[i] = make_float3(transform_pnt2(0), transform_pnt2(1), transform_pnt2(2)); 
		}
		corner_pnts_ = corner_pnts;
		UpdateInformation();
	}

	void HWPlane::SetPlaneCornerPts(const std::vector<float3>& corner_pnts_3d)
	{
		corner_pnts_last_state_ = corner_pnts_;
		corner_angels_last_state_ = corner_angels_;
		min_dists_last_state_ = min_dists_;
		corner_pnts_3d_last_state_ = corner_pnts_3d_;
		triangles_idx_last_state_ = triangles_idx_;
		corner_pnts_3d_ = corner_pnts_3d;
		if (corner_pnts_3d_last_state_.size() == 0 && pnts_pos_.size() != 0) {
			float3 edge1 = corner_pnts_3d_[1] - corner_pnts_3d_[0];
			float3 edge2 = corner_pnts_3d_[2] - corner_pnts_3d_[1];
			Eigen::Vector3f e1(edge1.x, edge1.y, edge1.z), e2(edge2.x, edge2.y, edge2.z);
			Eigen::Vector3f polygon_normal = e1.cross(e2);
			if (polygon_normal.dot(average_pnts_normal_) <= 0) {
				std::vector<float3> corner_pnts_3d_inverse;
				for (int i = corner_pnts_3d_.size() - 1; i >= 0; i--) {
					corner_pnts_3d_inverse.emplace_back(corner_pnts_3d_[i]);
				}
				corner_pnts_3d_ = corner_pnts_3d_inverse;
			}
		}
		std::vector<float2> corner_pnts;
		GenerateWorldCoordToPlaneCoordMatrixByCornerPnts3d();
		for (int i = 0; i < corner_pnts_3d_.size(); i++) {
			Eigen::Vector4f pnt(corner_pnts_3d_[i].x, corner_pnts_3d_[i].y, corner_pnts_3d_[i].z, 1.0);
			Eigen::Vector4f transform_pnt = world_to_plane_ * pnt;
			corner_pnts.emplace_back(make_float2(transform_pnt(0), transform_pnt(1)));
			Eigen::Vector4f transform_pnt2 = plane_to_world_ * Eigen::Vector4f(transform_pnt(0), transform_pnt(1), 0, 1);
			corner_pnts_3d_[i] = make_float3(transform_pnt2(0), transform_pnt2(1), transform_pnt2(2));
		}
		corner_pnts_ = corner_pnts;
		UpdateInformation();
	}

	void HWPlane::SetMaxPolyDistThreshold(float r)
	{
		poly_dist_threshold_ = r;
	}

	void HWPlane::UpdateCornerPts(int idx1, float2 & p1, int idx2, float2 & p2, int idx, std::vector<int>& nomove_idxs, Eigen::Vector3f & L_dir, Eigen::Vector3f & L_point)
	{
		corner_pnts_last_state_= corner_pnts_;
		corner_angels_last_state_= corner_angels_;
		min_dists_last_state_= min_dists_;
		corner_pnts_3d_last_state_= corner_pnts_3d_;
		triangles_idx_last_state_=triangles_idx_;
		float2 trans1 = p1 - corner_pnts_[idx1];
		float2 trans2 = p2 - corner_pnts_[idx2];
		if (dot(trans1, trans2) < 0) {
			printf("Two points are in the different sides of the intersection line\n");
			return;
		}
		if (idx1 > idx2) {
			int idx_temp = idx2;
			float2 p_temp = p2;
			idx2 = idx1;
			p2 = p1;
			idx1 = idx_temp;
			p1 = p_temp;
		}
		std::vector<float2> corner_temp;
		if (corner_pnts_.size() == 3) {
			corner_temp.emplace_back(corner_pnts_[idx1]);
			corner_temp.emplace_back(corner_pnts_[idx2]);
			corner_temp.emplace_back(p2);
			corner_temp.emplace_back(p1);
		}
		else {
			bool flag = idx > idx1&&idx < idx2 || (idx2 - idx1) < 1;
			//std::cout << "flag: " << flag << "\n";
			if (idx == idx1 || idx == idx2) {
				for (int i = 0; i < corner_pnts_.size(); i++) {
					if (i == idx1) {
						corner_temp.emplace_back(p1);
					}
					else if (i == idx2) {
						corner_temp.emplace_back(p2);
					}
					else
						corner_temp.emplace_back(corner_pnts_[i]);
				}
			}
			else {
				for (int i = 0; i < corner_pnts_.size(); i++) {
					if (flag&&i > idx1&&i < idx2) {
						for (int j = 0; j < nomove_idxs.size(); j++) {
							if (i == nomove_idxs[j]) {
								if (nomove_idxs[(j + 1) % nomove_idxs.size()] == (nomove_idxs[j] + 1) % corner_pnts_.size()) {
									float2 proj = ProjToLine3D(i, L_dir, L_point);
									corner_temp.emplace_back(proj);
								}
								corner_temp.emplace_back(corner_pnts_[i]);
								if (nomove_idxs[(j - 1 + nomove_idxs.size()) % nomove_idxs.size()] == (nomove_idxs[j] - 1) % corner_pnts_.size()) {
									float2 proj = ProjToLine3D(i, L_dir, L_point);
									corner_temp.emplace_back(proj);
								}
								break;
							}
						}
					}
					//continue;
					else if (!flag && (i<idx1 || i>idx2)) {
						for (int j = 0; j < nomove_idxs.size(); j++) {
							if (i == nomove_idxs[j]) {
								if (nomove_idxs[(j + 1) % nomove_idxs.size()] == (nomove_idxs[j] + 1) % corner_pnts_.size()) {
									float2 proj = ProjToLine3D(i, L_dir, L_point);
									corner_temp.emplace_back(proj);
								}
								corner_temp.emplace_back(corner_pnts_[i]);
								if (nomove_idxs[(j - 1 + nomove_idxs.size()) % nomove_idxs.size()] == (nomove_idxs[j] - 1) % corner_pnts_.size()) {
									float2 proj = ProjToLine3D(i, L_dir, L_point);
									corner_temp.emplace_back(proj);
								}
								break;
							}
						}
					}
					else {
						if (i == idx1) {
							/*if (flag&&!IsConcaveVertex(i)&& idx2!=idx1&&corner_pnts_.size()==3)
								corner_temp.emplace_back(corner_pnts_[i]);*/
							corner_temp.emplace_back(p1);
							/*if(!flag && !IsConcaveVertex(i) && idx2 != idx1&&corner_pnts_.size() == 3)
								corner_temp.emplace_back(corner_pnts_[i]);*/
						}
						else if (i == idx2) {
							/*if (!flag && !IsConcaveVertex(i) && idx2 != idx1&&corner_pnts_.size()==3)
								corner_temp.emplace_back(corner_pnts_[i]);*/
							corner_temp.emplace_back(p2);
							/*if (flag && !IsConcaveVertex(i) && idx2 != idx1&&corner_pnts_.size()==3)
								corner_temp.emplace_back(corner_pnts_[i]);*/
						}
						else
							corner_temp.emplace_back(corner_pnts_[i]);
					}
				}
			}
		}
		corner_pnts_ = corner_temp;
		std::vector<float3> corner_3d_temp;
		for (int i = 0; i < corner_pnts_.size(); i++) {
			Eigen::Vector4f transform_pnt = plane_to_world_*Eigen::Vector4f(corner_pnts_[i].x, corner_pnts_[i].y, 0, 1);
			corner_3d_temp.emplace_back(make_float3(transform_pnt(0), transform_pnt(1), transform_pnt(2)));
		}
		corner_pnts_3d_ = corner_3d_temp;

		UpdateInformation();
	}

	void HWPlane::UpdateCornerPtsFromNewPolygon(std::vector<Eigen::Vector2f>& polygon_new)
	{
		//
		corner_pnts_last_state_ = corner_pnts_;
		corner_angels_last_state_ = corner_angels_;
		min_dists_last_state_ = min_dists_;
		corner_pnts_3d_last_state_ = corner_pnts_3d_;
		triangles_idx_last_state_ = triangles_idx_;
		std::vector<float2> corner_temp;
		if (polygon_new.size() <= 3) {
			//不做任何处理
			std::cout << "no enough pnts..." << std::endl;
		}
		else
		{
			for (int i = 0; i < polygon_new.size(); ++i)
			{
				float2 tmpnt = make_float2(polygon_new[i][0], polygon_new[i][1]);
				corner_temp.emplace_back(tmpnt);
			}
			corner_pnts_ = corner_temp;
			std::vector<float3> corner_3d_temp;
			for (int i = 0; i < corner_pnts_.size(); i++) {
				Eigen::Vector4f transform_pnt = plane_to_world_*Eigen::Vector4f(corner_pnts_[i].x, corner_pnts_[i].y, 0, 1);
				corner_3d_temp.emplace_back(make_float3(transform_pnt(0), transform_pnt(1), transform_pnt(2)));
			}
			corner_pnts_3d_ = corner_3d_temp;
			UpdateInformation();
		}
	}

	void HWPlane::SetRefineTriMaxLen(float& tri_len)
	{
		plane_r_tri_len_ = tri_len;
	}

	void HWPlane::SeetRefineTriMinDegree(float& tri_degree)
	{
		plane_r_tri_degree_ = tri_degree;
	}

	void HWPlane::SaveCurrentStateData()
	{
		corner_pnts_last_state_ = corner_pnts_;
		corner_angels_last_state_ = corner_angels_;
		min_dists_last_state_ = min_dists_;
		corner_pnts_3d_last_state_ = corner_pnts_3d_;
		triangles_idx_last_state_ = triangles_idx_;
	}

	void HWPlane::BackToLastState()
	{
		if (corner_pnts_last_state_.size() != 0) {
			std::cout << "corner_pnts_: " << corner_pnts_.size() << "\n";
			corner_pnts_ = corner_pnts_last_state_;
			std::cout << "corner_pnts_back: " << corner_pnts_.size() << "\n";
			corner_angels_ = corner_angels_last_state_;
			min_dists_ = min_dists_last_state_;
			corner_pnts_3d_ = corner_pnts_3d_last_state_;
			triangles_idx_ = triangles_idx_last_state_;
		}
	}

	//相机看向plane所在坐标系的原点，距离为相机离平面的最近的位置
	void HWPlane::SetCameraPose(Eigen::Matrix4f & camera_pose)
	{
		plane_to_world_ = camera_pose;
		world_to_plane_ = camera_pose.inverse();
		Eigen::Vector3f coeff = camera_pose.block(0, 2, 3, 1);
		Eigen::Vector3f p = camera_pose.topRightCorner(3, 1);
		float d = -coeff.dot(p);
		coeff_.x = coeff[0];
		coeff_.y = coeff[1];
		coeff_.z = coeff[2];
		coeff_.w = d;
	}

	void HWPlane::GetSmoothRegions()
	{
		/*if (diameter_max_ > plane_width_ / 2&& plane_width_<0.4f)
			diameter_max_ = plane_width_ / 2;*/
		//std::cout << "edge_pnts_pos_num_: " << edge_pnts_pos_num_ << std::endl;
		edge_pnts_smooth_regions_.resize(edge_pnts_pos_num_);
#pragma omp parallel for
		for (int i = 0; i < edge_pnts_pos_num_; i++) {
			std::vector<int> in_region(edge_pnts_pos_num_, 0);
			edge_pnts_smooth_regions_[i].emplace_back(i);
			in_region[i] = 1;
			float region_diameter = 0;
			float2& p = edge_pnts_pos_[i];
			int last_idx = (i + edge_pnts_pos_num_ - 1) % edge_pnts_pos_num_;
			int next_idx = (i + edge_pnts_pos_num_ + 1) % edge_pnts_pos_num_;
			Eigen::Vector2f coeff;
			while (1){
				if (PointPointDist(p, edge_pnts_pos_[last_idx]) <= PointPointDist(p, edge_pnts_pos_[next_idx])) {
					edge_pnts_smooth_regions_[i].emplace_back(last_idx);
					in_region[last_idx] = 1;
					last_idx = (last_idx + edge_pnts_pos_num_ - 1) % edge_pnts_pos_num_;
				}
				else {
					edge_pnts_smooth_regions_[i].emplace_back(next_idx);
					in_region[next_idx] = 1;
					next_idx = (next_idx + edge_pnts_pos_num_ + 1) % edge_pnts_pos_num_;
				}
				coeff = FittingLine(edge_pnts_smooth_regions_[i]);
				if (abs(coeff(0)) > 1E-6 && abs(coeff(1)) > 1E-6)
					break;
			}
			/*for (int j = -1; j <= 1; j++) {
				if (j == 0)
					continue;
				int idx = (i + edge_pnts_pos_num_ + j) % edge_pnts_pos_num_;
				edge_pnts_smooth_regions_[i].emplace_back(idx);
				in_region[idx] = 1;
			}*/
			/*Eigen::Matrix2f A;
			A << edge_pnts_pos_[edge_pnts_smooth_regions_[i][0]].x, edge_pnts_pos_[edge_pnts_smooth_regions_[i][0]].y,
				edge_pnts_pos_[edge_pnts_smooth_regions_[i][1]].x, edge_pnts_pos_[edge_pnts_smooth_regions_[i][1]].y;
			Eigen::Vector2f B;
			B << 1, 1;
			Eigen::Vector2f coeff = A.inverse()*B;*/
			while (region_diameter < diameter_max_&&edge_pnts_smooth_regions_[i].size() < edge_pnts_pos_num_) {
				float dist_min = FLT_MAX;
				int idx_min;
				///std::cout << "coeff:" << coeff << "\n";
				for (int j = 0; j < edge_pnts_pos_num_; j++) {
					if (in_region[j])
						continue;
					float dist = PointLineDist(edge_pnts_pos_[j], coeff);
					if (abs(dist) < abs(dist_min)) {
						dist_min = dist;
						idx_min = j;
					}
				}
				edge_pnts_smooth_regions_[i].emplace_back(idx_min);
				in_region[idx_min] = 1;
				coeff = FittingLine(edge_pnts_smooth_regions_[i]);
				float max = -FLT_MAX, min = FLT_MAX;
				for (int k = 0; k < edge_pnts_smooth_regions_[i].size(); k++) {
					int idx = edge_pnts_smooth_regions_[i][k];
					float d = PointLineDist(edge_pnts_pos_[idx], coeff);
					//std::cout << "d: " << d << "\n";
					if (d > max)
						max = d;
					if (d < min)
						min = d;
				}
				region_diameter = max - min;
			}
			edge_pnts_smooth_regions_[i].pop_back();	
		}
		
	}

	void HWPlane::NewSmoothRegions()
	{
		/*if (diameter_max_ > plane_width_ / 2&& plane_width_<0.4f)
			diameter_max_ = plane_width_ / 2;*/
			//std::cout << "edge_pnts_pos_num_: " << edge_pnts_pos_num_ << std::endl;
		edge_pnts_smooth_regions_.clear();
		edge_pnts_smooth_regions_.resize(edge_pnts_pos_num_);
		std::cout << "smooth regions:" << std::endl;
#pragma omp parallel for
		for (int i = 0; i < edge_pnts_pos_num_; i++) {
			std::vector<int> in_region(edge_pnts_pos_num_, 0);
			edge_pnts_smooth_regions_[i].emplace_back(i);
			in_region[i] = 1;
			float region_diameter = 0;
			float2& p = edge_pnts_pos_[i];
			int last_idx = (i + edge_pnts_pos_num_ - 1) % edge_pnts_pos_num_;
			int next_idx = (i + edge_pnts_pos_num_ + 1) % edge_pnts_pos_num_;
			Eigen::Vector2f coeff;
			while (1) {
				if (PointPointDist(p, edge_pnts_pos_[last_idx]) <= PointPointDist(p, edge_pnts_pos_[next_idx])) {
					edge_pnts_smooth_regions_[i].emplace_back(last_idx);
					in_region[last_idx] = 1;
					last_idx = (last_idx + edge_pnts_pos_num_ - 1) % edge_pnts_pos_num_;
				}
				else {
					edge_pnts_smooth_regions_[i].emplace_back(next_idx);
					in_region[next_idx] = 1;
					next_idx = (next_idx + edge_pnts_pos_num_ + 1) % edge_pnts_pos_num_;
				}
				coeff = FittingLine(edge_pnts_smooth_regions_[i]);
				if (abs(coeff(0)) < 1E-6 && abs(coeff(1)) < 1E-6) continue;

				float max = -FLT_MAX, min = FLT_MAX;
				for (int k = 0; k < edge_pnts_smooth_regions_[i].size(); k++) {
					int idx = edge_pnts_smooth_regions_[i][k];
					float d = PointLineDist(edge_pnts_pos_[idx], coeff);
					if (d > max)
						max = d;
					if (d < min)
						min = d;
				}
				region_diameter = max - min;
				if (region_diameter > diameter_max_ ||
					edge_pnts_smooth_regions_[i].size() == edge_pnts_pos_num_) {
					//edge_pnts_smooth_regions_[i].pop_back();
					break;
				}
			}
		}
		//for (int i = 0; i < edge_pnts_pos_num_; i++) {
			//std::cout << i << ": ";
			//for (int j = 0; j < edge_pnts_smooth_regions_[i].size(); j++) {
				//std::cout << edge_pnts_smooth_regions_[i][j] << ", ";
			//}
			//std::cout << std::endl;
		//}

		//内边缘
		inner_edge_pnts_smooth_regions_.resize(inner_polygon_num_);
		for (int xxx = 0; xxx < inner_edge_pnts_pos_.size(); xxx++) {
			//inner_edge_pnts_smooth_regions_.clear();
			int pnts_pos_num = inner_edge_pnts_pos_num_[xxx];
			std::vector<std::vector<int>>& smooth_regions = inner_edge_pnts_smooth_regions_[xxx];
			std::vector<float2>& pnts_pos = inner_edge_pnts_pos_[xxx];
			smooth_regions.resize(pnts_pos_num);
			std::cout << "smooth regions:" << std::endl;
#pragma omp parallel for
			for (int i = 0; i < pnts_pos_num; i++) {
				std::vector<int> in_region(pnts_pos_num, 0);
				smooth_regions[i].emplace_back(i);
				in_region[i] = 1;
				float region_diameter = 0;
				float2& p = pnts_pos[i];
				int last_idx = (i + pnts_pos_num - 1) % pnts_pos_num;
				int next_idx = (i + pnts_pos_num + 1) % pnts_pos_num;
				Eigen::Vector2f coeff;
				while (1) {
					if (PointPointDist(p, pnts_pos[last_idx]) <= PointPointDist(p, pnts_pos[next_idx])) {
						smooth_regions[i].emplace_back(last_idx);
						in_region[last_idx] = 1;
						last_idx = (last_idx + pnts_pos_num - 1) % pnts_pos_num;
					}
					else {
						smooth_regions[i].emplace_back(next_idx);
						in_region[next_idx] = 1;
						next_idx = (next_idx + pnts_pos_num + 1) % pnts_pos_num;
					}
					coeff = FittingLine(smooth_regions[i]);
					if (abs(coeff(0)) < 1E-6 && abs(coeff(1)) < 1E-6) continue;

					float max = -FLT_MAX, min = FLT_MAX;
					for (int k = 0; k < smooth_regions[i].size(); k++) {
						int idx = smooth_regions[i][k];
						float d = PointLineDist(pnts_pos[idx], coeff);
						if (d > max)
							max = d;
						if (d < min)
							min = d;
					}
					region_diameter = max - min;
					if (region_diameter > diameter_max_ ||
						smooth_regions[i].size() == pnts_pos_num) {
						//edge_pnts_smooth_regions_[i].pop_back();
						break;
					}
				}
			}
		}
	}

	void HWPlane::GetNeighbors()
	{
		std::cout << "start getneighbors" << std::endl;
		edge_pnts_neighs_.clear();
		edge_pnts_neighs_.resize(edge_pnts_pos_num_);
#pragma omp parallel for 
		for (int i = 0; i < edge_pnts_pos_num_; i++) {
			//printf("i:  %d\n", i);
			std::sort(edge_pnts_smooth_regions_[i].begin(), edge_pnts_smooth_regions_[i].end());
			for (int j = 0; j < edge_pnts_smooth_regions_[i].size(); j++) {
				int idx = edge_pnts_smooth_regions_[i][j];
				//std::cout << "smooth_regions:  " << idx << std::endl;
				std::vector<int>::iterator result = find(edge_pnts_smooth_regions_[idx].begin(), edge_pnts_smooth_regions_[idx].end(), i);
				if (result != edge_pnts_smooth_regions_[idx].end()) {
					edge_pnts_neighs_[i].emplace_back(idx);
				}
			}
		}

		//写入颜色信息
		edge_pnts_color_index_.resize(edge_pnts_pos_num_);
		for (int i = 0; i < edge_pnts_pos_num_; i++) {
			edge_pnts_color_index_[i] = -1;
		}

		int color_index = 0;
		for (int i = 0; i < edge_pnts_pos_num_; i++) {
			if (edge_pnts_color_index_[i] == -1) {
				//edge_pnts_color_index_[i] = color_index;
				for (int j = 0; j < edge_pnts_smooth_regions_[i].size(); j++) {
					int idx = edge_pnts_smooth_regions_[i][j];
					edge_pnts_color_index_[idx] = color_index;
				}
				color_index = (color_index + 1) % colors_.size();
			}
		}

		for (int i = 0; i < edge_pnts_pos_num_; i++) {
			if (edge_pnts_color_index_[i] == -1) {
				std::cout << "error: edge_pnts " << i << "has no color" << std::endl;
			}
		}

		//内边缘
		inner_edge_pnts_neighs_.resize(inner_polygon_num_);
		for (int xxx = 0; xxx < inner_polygon_num_; xxx++) {
			std::vector<std::vector<int>> &pnts_neighs = inner_edge_pnts_neighs_[xxx];
			std::vector<std::vector<int>> &smooth_regions = inner_edge_pnts_smooth_regions_[xxx];
			int pnts_pos_num = inner_edge_pnts_pos_num_[xxx];
			pnts_neighs.clear();
			pnts_neighs.resize(pnts_pos_num);
#pragma omp parallel for 
			for (int i = 0; i < pnts_pos_num; i++) {
				//printf("i:  %d\n", i);
				std::sort(smooth_regions[i].begin(), smooth_regions[i].end());
				for (int j = 0; j < smooth_regions[i].size(); j++) {
					int idx = smooth_regions[i][j];
					//std::cout << "smooth_regions:  " << idx << std::endl;
					std::vector<int>::iterator result = find(smooth_regions[idx].begin(), smooth_regions[idx].end(), i);
					if (result != smooth_regions[idx].end()) {
						pnts_neighs[i].emplace_back(idx);
					}
				}
			}
		}
	}

	void HWPlane::GetInitialNormals()
	{
		edge_pnts_normal_.clear();
		edge_pnts_normal_.resize(edge_pnts_pos_num_);
#pragma omp parallel for
		for (int i = 0; i < edge_pnts_pos_num_; i++) {
			Eigen::MatrixX2f neighbor(edge_pnts_neighs_[i].size(), 2);

			//std::sort(edge_pnts_neighs_[i].begin(), edge_pnts_neighs_[i].end());
			//float2 first_neighbor = edge_pnts_pos_[edge_pnts_neighs_[i][0]];
			//float2 last_neighbor = edge_pnts_pos_[edge_pnts_neighs_[i][1]];
			//float2 direction = last_neighbor - first_neighbor;
			//std::cout << "direction: index: " << i << "first: " << edge_pnts_neighs_[i][0] << " last: " << edge_pnts_neighs_[i].back() << std::endl;
			for (int j = 0; j < edge_pnts_neighs_[i].size(); j++) {
				neighbor(j, 0) = edge_pnts_pos_[edge_pnts_neighs_[i][j]].x;
				neighbor(j, 1) = edge_pnts_pos_[edge_pnts_neighs_[i][j]].y;
			}
			Eigen::RowVector2f var_mean = neighbor.colwise().mean();
			Eigen::MatrixX2f centered = neighbor.rowwise() - var_mean;
			Eigen::Matrix2f covariance = (centered.adjoint()*centered) / (neighbor.rows() - 1);
			Eigen::EigenSolver<Eigen::Matrix2f> es(covariance);
			Eigen::Matrix2f eig_values = es.pseudoEigenvalueMatrix();
			Eigen::Matrix2f eig_vectors = es.pseudoEigenvectors();
			int min_value_idx = eig_values(0, 0) <= eig_values(1, 1) ? 0 : 1;
			float2 normal;
			normal.x = eig_vectors(0, min_value_idx);
			normal.y = eig_vectors(1, min_value_idx);
			//if (normal.x * direction.y - normal.y * direction.x < 0) normal = -normal;
			edge_pnts_normal_[i] = normal;
			//std::cout << i << " :  " << normal.x << "  " << normal.y << std::endl;
			//system("pause");
		}

		for (int i = 0; i < edge_pnts_pos_num_; i++) {
			float2 prev_point1 = edge_pnts_pos_[(i - 1 + edge_pnts_pos_num_) % edge_pnts_pos_num_];
			float2 prev_point2 = edge_pnts_pos_[(i - 2 + edge_pnts_pos_num_) % edge_pnts_pos_num_];
			float2 prev_point3 = edge_pnts_pos_[(i - 3 + edge_pnts_pos_num_) % edge_pnts_pos_num_];
			float2 start_point = edge_pnts_pos_[i];
			float2 end_point1 = edge_pnts_pos_[(i + 1) % edge_pnts_pos_num_];
			float2 end_point2 = edge_pnts_pos_[(i + 2) % edge_pnts_pos_num_];
			float2 end_point3 = edge_pnts_pos_[(i + 3) % edge_pnts_pos_num_];
			float2 prev_direction1 = start_point - prev_point1;
			float2 prev_direction2 = start_point - prev_point2;
			float2 prev_direction3 = start_point - prev_point3;
			float2 direction1 = end_point1 - start_point;
			float2 direction2 = end_point2 - start_point;
			float2 direction3 = end_point3 - start_point;
			float2 big_direction1 = end_point1 - prev_point1;
			float2 big_direction2 = end_point2 - prev_point2;
			float2 big_direction3 = end_point3 - prev_point3;

			float2 curr_normal = edge_pnts_normal_[i];

			int count = 0;
			if (big_direction1.x * curr_normal.y - big_direction1.y * curr_normal.x > 0) count++;
			if (big_direction2.x * curr_normal.y - big_direction2.y * curr_normal.x > 0) count++;
			if (big_direction3.x * curr_normal.y - big_direction3.y * curr_normal.x > 0) count++;

			//if (prev_direction1.x * curr_normal.y - prev_direction1.y * curr_normal.x > 0) count++;
			//if (prev_direction2.x * curr_normal.y - prev_direction2.y * curr_normal.x > 0) count++;
			//if (prev_direction3.x * curr_normal.y - prev_direction3.y * curr_normal.x > 0) count++;

			//if (direction1.x * curr_normal.y - direction1.y * curr_normal.x > 0) count++;
			//if (direction2.x * curr_normal.y - direction2.y * curr_normal.x > 0) count++;
			//if (direction3.x * curr_normal.y - direction3.y * curr_normal.x > 0) count++;

			if (count >= 2) {
				edge_pnts_normal_[i] = -edge_pnts_normal_[i];
			}
		}
		std::cout << "edge_pnts_normal_.size = " << edge_pnts_normal_.size() << std::endl;
		//std::cout << "edge_pnts_normal_:" << std::endl;
		//for (int i = 0; i < edge_pnts_normal_.size(); i++) {
		//	std::cout << i << ": " << edge_pnts_normal_[i].x << ", " << edge_pnts_normal_[i].y << std::endl;
		//}


		//内循环
		inner_edge_pnts_normal_.resize(inner_polygon_num_);

		for (int xxx = 0; xxx < inner_polygon_num_; xxx++) {
			int pnts_pos_num = inner_edge_pnts_pos_num_[xxx];
			std::vector<float2> &pnts_normal = inner_edge_pnts_normal_[xxx];
			std::vector<float2> &pnts_pos = inner_edge_pnts_pos_[xxx];
			std::vector<std::vector<int>>& pnts_neighs = inner_edge_pnts_neighs_[xxx];
			pnts_normal.clear();
			pnts_normal.resize(pnts_pos_num);
#pragma omp parallel for
			for (int i = 0; i < pnts_pos_num; i++) {
				Eigen::MatrixX2f neighbor(pnts_neighs[i].size(), 2);

				//std::sort(edge_pnts_neighs_[i].begin(), edge_pnts_neighs_[i].end());
				//float2 first_neighbor = edge_pnts_pos_[edge_pnts_neighs_[i][0]];
				//float2 last_neighbor = edge_pnts_pos_[edge_pnts_neighs_[i][1]];
				//float2 direction = last_neighbor - first_neighbor;
				//std::cout << "direction: index: " << i << "first: " << edge_pnts_neighs_[i][0] << " last: " << edge_pnts_neighs_[i].back() << std::endl;
				for (int j = 0; j < pnts_neighs[i].size(); j++) {
					neighbor(j, 0) = pnts_pos[pnts_neighs[i][j]].x;
					neighbor(j, 1) = pnts_pos[pnts_neighs[i][j]].y;
				}
				Eigen::RowVector2f var_mean = neighbor.colwise().mean();
				Eigen::MatrixX2f centered = neighbor.rowwise() - var_mean;
				Eigen::Matrix2f covariance = (centered.adjoint() * centered) / (neighbor.rows() - 1);
				Eigen::EigenSolver<Eigen::Matrix2f> es(covariance);
				Eigen::Matrix2f eig_values = es.pseudoEigenvalueMatrix();
				Eigen::Matrix2f eig_vectors = es.pseudoEigenvectors();
				int min_value_idx = eig_values(0, 0) <= eig_values(1, 1) ? 0 : 1;
				float2 normal;
				normal.x = eig_vectors(0, min_value_idx);
				normal.y = eig_vectors(1, min_value_idx);
				//if (normal.x * direction.y - normal.y * direction.x < 0) normal = -normal;
				pnts_normal[i] = normal;
				//std::cout << i << " :  " << normal.x << "  " << normal.y << std::endl;
				//system("pause");
			}

			for (int i = 0; i < pnts_pos_num; i++) {
				float2 prev_point1 = pnts_pos[(i - 1 + pnts_pos_num) % pnts_pos_num];
				float2 prev_point2 = pnts_pos[(i - 2 + pnts_pos_num) % pnts_pos_num];
				float2 prev_point3 = pnts_pos[(i - 3 + pnts_pos_num) % pnts_pos_num];
				float2 start_point = pnts_pos[i];
				float2 end_point1 = pnts_pos[(i + 1) % pnts_pos_num];
				float2 end_point2 = pnts_pos[(i + 2) % pnts_pos_num];
				float2 end_point3 = pnts_pos[(i + 3) % pnts_pos_num];
				float2 prev_direction1 = start_point - prev_point1;
				float2 prev_direction2 = start_point - prev_point2;
				float2 prev_direction3 = start_point - prev_point3;
				float2 direction1 = end_point1 - start_point;
				float2 direction2 = end_point2 - start_point;
				float2 direction3 = end_point3 - start_point;
				float2 big_direction1 = end_point1 - prev_point1;
				float2 big_direction2 = end_point2 - prev_point2;
				float2 big_direction3 = end_point3 - prev_point3;

				float2 curr_normal = pnts_normal[i];

				int count = 0;
				if (big_direction1.x * curr_normal.y - big_direction1.y * curr_normal.x > 0) count++;
				if (big_direction2.x * curr_normal.y - big_direction2.y * curr_normal.x > 0) count++;
				if (big_direction3.x * curr_normal.y - big_direction3.y * curr_normal.x > 0) count++;

				//if (prev_direction1.x * curr_normal.y - prev_direction1.y * curr_normal.x > 0) count++;
				//if (prev_direction2.x * curr_normal.y - prev_direction2.y * curr_normal.x > 0) count++;
				//if (prev_direction3.x * curr_normal.y - prev_direction3.y * curr_normal.x > 0) count++;

				//if (direction1.x * curr_normal.y - direction1.y * curr_normal.x > 0) count++;
				//if (direction2.x * curr_normal.y - direction2.y * curr_normal.x > 0) count++;
				//if (direction3.x * curr_normal.y - direction3.y * curr_normal.x > 0) count++;

				if (count >= 2) {
					pnts_normal[i] = -pnts_normal[i];
				}
			}
		}
	}	

	//void HWPlane::OptimizeByHLBFGS(int N, double *init_x, int num_iter, int M, int T, void(*evalfunc)(int, double*, double*,
	//	double*, double*))
	//{
	//	double parameter[20];
	//	int info[20];
	//	//initialize
	//	INIT_HLBFGS(parameter, info);
	//	info[4] = num_iter;
	//	info[6] = T;
	//	info[7] = 0;
	//	info[10] = 0;
	//	info[11] = 1;
	//	HLBFGS(N, M, init_x,this, evalfunc, 0, HLBFGS_UPDATE_Hessian, 0, parameter, info);
	//}

	double HWPlane::GetAngle(double x1, double y1, double x2, double y2)
	{
		double dot = (x1*x2 + y1 * y2) / sqrt(x1*x1 + y1 * y1) / sqrt(x2*x2 + y2 * y2);
		if (dot > 1)
			dot = 1;
		else if (dot < -1)
			dot = -1;
		double theta = 180.0f * acos(dot) / MATH_PI;
		return theta;
	}

	double HWPlane::GetTheta(double x1, double y1, double x2, double y2)
	{
		double dot = (x1*x2 + y1*y2)/sqrt(x1*x1+y1*y1)/sqrt(x2*x2+y2*y2);
		if (dot > 1)
			dot = 1;
		else if (dot < -1)
			dot = -1;
		double theta = 180.0f * acos(dot) / MATH_PI;
		if (theta > 90.0f)
			return 180 - theta;
		return theta;
	}

	float HWPlane::GetTheta(float3 e1, float3 e2)
	{
		float dot = (e1.x*e2.x + e1.y*e2.y + e1.z*e2.z) / sqrt(e1.x*e1.x + e1.y*e1.y + e1.z*e1.z) / sqrt(e2.x*e2.x + e2.y*e2.y + e2.z*e2.z);
		if (dot > 1)
			dot = 1;
		else if (dot < -1)
			dot = -1;
		double theta = 180.0f * acos(dot) / MATH_PI;
		if (theta > 90.0f)
			return 180 - theta;
		return theta;
	}

	double HWPlane::GetWeight(double x1, double y1, double x2, double y2)
	{
		double theta = GetAngle(x1, y1, x2, y2);
		//double theta = GetTheta(x1, y1, x2, y2);
		double weight = exp(-pow(theta / 20.0f, 2));
		return weight;
	}

	void HWPlane::Newiteration(int iter, int call_iter, double *x, double* f, double *g, double* gnorm)
	{
		//std::cout << iter << ": " << call_iter << " " << *f << " " << *gnorm << std::endl;
		/*for (int i = 0; i < 4; i++)
			printf("p%d:%0.6f  %0.6f\n", i + 1, x[2 * i], x[2 * i + 1]);*/
	}

	void HWPlane::EvalfuncForNormalEstimation(int N, double * x, double * prev_x, double * f, double * g)
	{
		*f = 0;
		for (int i = 0; i < N; i++) g[i] = 0;
		for (int i = 0; i < N; i += 2) {
			double& Np_x = x[i];
			double& Np_y = x[i + 1];
			//std::cout << " Np:  " << Np_x << "  " << Np_y << std::endl;
			for (int j = 0; j < edge_pnts_neighs_[i / 2].size(); j++) {
				int idx = edge_pnts_neighs_[i / 2][j];
				if (idx == i / 2) continue;
				double& Nq_x = x[idx * 2];
				double& Nq_y = x[idx * 2 + 1];
				//std::cout << " Nq:  " << Nq_x << "  " << Nq_y << std::endl;
				//double theta = 180*acos(Np_x*Nq_x + Np_y*Nq_y)/MATH_PI;
				double weight = GetWeight(Np_x, Np_y, Nq_x, Nq_y);
				//if(j > idx)
				if (idx > i / 2) {
					*f += weight*(pow(Np_x - Nq_x, 2) + pow(Np_y - Nq_y, 2));
					double pqx = 2 * weight * (Np_x - Nq_x);
					double pqy = 2 * weight * (Np_y - Nq_y);
					//g[i] += 2 * weight*(Np_x - Nq_x);
					//g[i + 1] += 2 * weight*(Np_y - Nq_y);
					g[i] += pqx;
					g[i + 1] += pqy;
					g[idx * 2] -= pqx;
					g[idx * 2 + 1] -= pqy;
				}
			}
			double Np_x0 = edge_pnts_normal_[i / 2].x;
			double Np_y0 = edge_pnts_normal_[i / 2].y;
			//*f += 0.01*(pow(Np_x - Np_x0, 2) + pow(Np_y - Np_y0, 2));
			//g[i] += 0.02*(Np_x - Np_x0);
			//g[i + 1] += 0.02*(Np_y - Np_y0);
			*f += 0.1*(pow(Np_x - Np_x0, 2) + pow(Np_y - Np_y0, 2));
			g[i] += 0.2*(Np_x - Np_x0);
			g[i + 1] += 0.2*(Np_y - Np_y0);
		}
		//std::cout << "normal f: " << *f << std::endl;
	}

	void HWPlane::DoNormalEstimation()
	{
		std::vector<double> normals;

		for (int i = 0; i < edge_pnts_pos_num_; i++) {
			normals.emplace_back(edge_pnts_normal_[i].x);
			normals.emplace_back(edge_pnts_normal_[i].y);
		}
		int M = 10;
		int T = 0;
		
		double parameter[20];
		int info[20];
		//initialize
		INIT_HLBFGS(parameter, info);
		info[4] = 1000;
		info[6] = T;
		info[7] = 0;
		info[10] = 0;
		info[11] = 1;
		HLBFGS(normals.size(), M, &normals[0], this, &HWPlane::EvalfuncForNormalEstimation, 0, HLBFGS_UPDATE_Hessian, &HWPlane::Newiteration, parameter, info);
		//OptimizeByHLBFGS(normals.size(), &normals[0], 1000, M, T, HWPlane::EvalfuncForNormalEstimation);
		/*std::ofstream fh("G:\\xht\\huawei\\2019-04-28_14.54.52\\intial_normal.obj");
		for (int i = 0; i < edge_pnts_normal_.size(); ++i)
		{
			for (int j = 0; j < 100; j++) {
				float2 p = edge_pnts_pos_[i] + 0.001*j*edge_pnts_normal_[i];
				fh << "v " << p.x << " " <<p.y << " " << "0.0" << std::endl;
			}			
		}
		fh.close();*/
		for (int i = 0; i < edge_pnts_pos_num_; i++) {
			double norm = sqrt(normals[2 * i] * normals[2 * i] + normals[2 * i + 1] * normals[2 * i + 1]);
			edge_pnts_normal_[i].x = normals[2 * i] / norm;
			edge_pnts_normal_[i].y = normals[2 * i + 1] / norm;
		}
		/*std::ofstream fo("G:\\xht\\huawei\\2019-04-28_14.54.52\\opt_normal.obj");
		for (int i = 0; i < edge_pnts_normal_.size(); ++i)
		{
			for (int j = 0; j < 100; j++) {
				float2 p = edge_pnts_pos_[i] + 0.001*j*edge_pnts_normal_[i];
				fo << "v " << p.x << " " << p.y << " " << "0.0" << std::endl;
			}
		}
		fo.close();*/
		/*edge_pnts_pos_[0] += 0.05*edge_pnts_normal_[0];
		edge_pnts_pos_[1] -= 0.05*edge_pnts_normal_[1];
		edge_pnts_pos_[2] += 0.05*edge_pnts_normal_[2];
		edge_pnts_pos_[3] += 0.05*edge_pnts_normal_[3];
		edge_pnts_pos_[4] -= 0.1*edge_pnts_normal_[4];*/
		/*for (int i = 0; i < edge_pnts_pos_num_; i++) {
			edge_pnts_color_.emplace_back(make_uchar3(0, 0, 0));
		}*/
		//double theta_max = 10.0f;
		//for (int i = 0; i < edge_pnts_pos_num_; i++) {
		//	int last_idx = (i + edge_pnts_pos_num_ - 1) % edge_pnts_pos_num_;
		//	int next_idx = (i + edge_pnts_pos_num_ + 1) % edge_pnts_pos_num_;
		//	float2 n = edge_pnts_normal_[i], n1 = edge_pnts_normal_[last_idx], n2 = edge_pnts_normal_[next_idx];
		//	if (i==1) {
		//		edge_pnts_color_[i] = make_uchar3(255, 0, 0);
		//		//std::cout << "i: " << i << "\n";
		//		for (int j = 1; j < edge_pnts_neighs_[i].size(); j++) {
		//			int idx = edge_pnts_neighs_[i][j];
		//			//printf("neighor: %d\n", idx);
		//			edge_pnts_color_[idx] = make_uchar3(0, 255, 0);
		//		}	
		//		//edge_pnts_color_[293] = make_uchar3(0, 0, 255);
		//		//system("pause");
		//		break;
		//	}
		//}
		
		////内边缘
		//for (int xxx = 0; xxx < inner_polygon_num_; xxx++) {
		//	normals.clear();
		//	int pnts_pos_num = inner_edge_pnts_pos_num_[xxx];
		//	auto &pnts_normal = inner_edge_pnts_normal_[xxx];
		//	for (int i = 0; i < pnts_pos_num; i++) {
		//		normals.emplace_back(pnts_normal[i].x);
		//		normals.emplace_back(pnts_normal[i].y);
		//	}
		//	int M = 10;
		//	int T = 0;

		//	double parameter[20];
		//	int info[20];
		//	//initialize
		//	INIT_HLBFGS(parameter, info);
		//	info[4] = 1000;
		//	info[6] = T;
		//	info[7] = 0;
		//	info[10] = 0;
		//	info[11] = 1;
		//	HLBFGS(normals.size(), M, &normals[0], this, &HWPlane::EvalfuncForNormalEstimation, 0, HLBFGS_UPDATE_Hessian, &HWPlane::Newiteration, parameter, info);
		//	//OptimizeByHLBFGS(normals.size(), &normals[0], 1000, M, T, HWPlane::EvalfuncForNormalEstimation);
		//	/*std::ofstream fh("G:\\xht\\huawei\\2019-04-28_14.54.52\\intial_normal.obj");
		//	for (int i = 0; i < edge_pnts_normal_.size(); ++i)
		//	{
		//		for (int j = 0; j < 100; j++) {
		//			float2 p = edge_pnts_pos_[i] + 0.001*j*edge_pnts_normal_[i];
		//			fh << "v " << p.x << " " <<p.y << " " << "0.0" << std::endl;
		//		}
		//	}
		//	fh.close();*/
		//	for (int i = 0; i < pnts_pos_num; i++) {
		//		double norm = sqrt(normals[2 * i] * normals[2 * i] + normals[2 * i + 1] * normals[2 * i + 1]);
		//		pnts_normal[i].x = normals[2 * i] / norm;
		//		pnts_normal[i].y = normals[2 * i + 1] / norm;
		//	}
		//}
	}

	void HWPlane::EvalfuncForPolygonSmoothing(int N, double * x, double * prev_x, double * f, double * g)
	{
		*f = 0;
		for (int i = 0; i < N; i++) g[i] = 0;
#ifdef USE_OPENMP
#pragma omp parallel for schedule(dynamic)
#endif
		double mu = params_.polygon_smoothing_mu;
		if (mu == 0) {
			mu = 1.0;
			initial_params_.polygon_smoothing_mu = 1.0;
		}
		//std::cout << "mu = " << mu << std::endl;
		for (int i = 0; i < N; i++) {
			float2& p = edge_pnts_pos_[i];
			float2& n_p = edge_pnts_normal_[i];
			float2 p_ = p + x[i] * n_p;
			//g[i] = 0;
			for (int j = 0; j < edge_pnts_neighs_[i].size(); j++) {
				int idx = edge_pnts_neighs_[i][j];
				if (idx == i) continue;
				float2& q = edge_pnts_pos_[idx];
				float2& n_q = edge_pnts_normal_[idx];
				float2 q_ = q + x[idx] * n_q;
				//float2 pq = p - q;
				double weight = GetWeight(n_p.x, n_p.y, n_q.x, n_q.y);
				float2 pq_ = p_ - q_;
				//float norm = pow(pq_.x*pq_.x + pq_.y*pq_.y, 0.5);
				//if (norm < 1e-8)
				//	continue;
				//float2 pq_normlize = pq_ / norm;

				/**f += weight*pow(dot(pq_normlize,n_q), 2);
				g[i] += 2 * weight*((dot(pq_, n_q)*dot(n_q, n_p) + dot(pq_, n_p)*dot(n_p, n_p))*pow(norm,2)-
					dot(pq_,n_p)*(pow(dot(pq_,n_p),2)+ pow(dot(pq_, n_q), 2)))/pow(norm,4);*/


				//*f += weight*pow(dot(pq_normlize, n_p), 2);
				//g[i] += 2 * weight*((dot(pq_normlize, n_q)*dot(n_q, n_p) + dot(pq_normlize, n_p)*dot(n_p, n_p)));// / pow(norm, 2);

				*f += weight*pow(dot(pq_, n_q), 2);
				g[i] += 2 * weight*(dot(pq_, n_q)*dot(n_p, n_q));// / pow(norm, 2);
				g[idx] -= 2 * weight*(dot(pq_, n_q)*dot(n_q, n_q));
			}
			//system("pause");

			*f += mu*x[i] * x[i];
			g[i] += 2*mu*x[i];
		}
		//std::cout << "f: " << *f << std::endl;
	}

	void HWPlane::DoPolygonSmoothing(std::string filename)
	{
		std::vector<double> tp(edge_pnts_pos_num_, 0.0);
		int M = 7;
		int T = 0;
		
		double parameter[20];
		int info[20];
		//initialize
		INIT_HLBFGS(parameter, info);
		info[4] = 1000;
		info[6] = T;
		info[7] = 0;
		info[10] = 0;
		info[11] = 1;
		HLBFGS(tp.size(), M, &tp[0], this, &HWPlane::EvalfuncForPolygonSmoothing, 0, HLBFGS_UPDATE_Hessian, &HWPlane::Newiteration, parameter, info);
		//OptimizeByHLBFGS(tp.size(), &tp[0], 1000, M, T);
		/*for (int i = 0; i < 5; i++) {
			std::cout << "tp: " << tp[i] << std::endl;
		}*/
		for (int i = 0; i < tp.size(); i++) {
			edge_pnts_pos_[i] += tp[i] * edge_pnts_normal_[i];
		}
		
		////内边缘
		//for (int xxx = 0; xxx < inner_polygon_num_; xxx++) {
		//	int pnts_pos_num = inner_edge_pnts_pos_num_[xxx];
		//	auto &pnts_pos = inner_edge_pnts_pos_[xxx];
		//	auto &pnts_normal = inner_edge_pnts_normal_[xxx];

		//	std::vector<double> tp(pnts_pos_num, 0.0);
		//	int M = 7;
		//	int T = 0;

		//	double parameter[20];
		//	int info[20];
		//	//initialize
		//	INIT_HLBFGS(parameter, info);
		//	info[4] = 1000;
		//	info[6] = T;
		//	info[7] = 0;
		//	info[10] = 0;
		//	info[11] = 1;
		//	HLBFGS(tp.size(), M, &tp[0], this, &HWPlane::EvalfuncForPolygonSmoothing, 0, HLBFGS_UPDATE_Hessian, &HWPlane::Newiteration, parameter, info);
		//	//OptimizeByHLBFGS(tp.size(), &tp[0], 1000, M, T);
		//	/*for (int i = 0; i < 5; i++) {
		//		std::cout << "tp: " << tp[i] << std::endl;
		//	}*/
		//	for (int i = 0; i < tp.size(); i++) {
		//		pnts_pos[i] += tp[i] * pnts_normal[i];
		//	}
		//}

#ifdef OUTPUT_EDGES_SMOOTHED
		std::ofstream fh(filename + "edges_smoothed.obj");
		for (int i = 0; i < edge_pnts_pos_.size(); ++i)
		{
			fh << "v " << edge_pnts_pos_[i].x << " " << edge_pnts_pos_[i].y << " " << 0 << std::endl;
		}
		for (int i = 0; i < edge_pnts_pos_.size() - 1; i++) {
			fh << "l " << i + 1 << " " << i + 2 << "\n";
		}
		fh << "l " << 1 << " " << edge_pnts_pos_.size() << "\n";
		fh.close();


		//写入三维空间的边缘点和边缘
		std::vector<float3> points_3d;
		for (int i = 0; i < edge_pnts_pos_.size(); i++) {
			Eigen::Vector4f pnt(edge_pnts_pos_[i].x, edge_pnts_pos_[i].y, 0.0, 1.0);
			Eigen::Vector4f transform_pnt = plane_to_world_ * pnt;
			points_3d.emplace_back(make_float3(transform_pnt(0), transform_pnt(1), transform_pnt(2)));
		}

		fh.open(filename + "edges_smoothed_3d.obj");
		for (int i = 0; i < points_3d.size(); i++)
		{
			float3 start_pnt = points_3d[i];
			fh << "v " << start_pnt.x << " " << start_pnt.y << " " << start_pnt.z << std::endl;
		}
		for (int ii = 0; ii < points_3d.size() - 1; ii++) {
			fh << "l " << ii + 1 << " " << ii + 2 << "\n";
		}
		fh << "l " << 1 << " " << points_3d.size() << "\n";
		fh.close();
#endif // OUTPUT_EDGES_SMOOTHED
	}
	
	//struct ExponentialResidual {
	//	ExponentialResidual(double x, double y) : x_(x), y_(y) {}

	//	template <typename T>
	//	bool operator()(const T* const m, const T* const c, T* residual) const {
	//		residual[0] = y_ - exp(m[0] * x_ + c[0]);
	//		return true;
	//	}

	//private:
	//	const double x_;
	//	const double y_;
	//};
	//using ceres::AutoDiffCostFunction;
	//using ceres::CostFunction;
	//using ceres::Problem;
	//using ceres::Solve;
	//using ceres::Solver;

	//struct InitFunctor {
	//	InitFunctor(float mu): _mu(mu) {}
	//	template <typename T>
	//	bool operator() (const T* const t, T* residual) const {
	//		residual[0] = (T)_mu * t[0] * t[0];
	//		return true;
	//	}
	//private:
	//	float _mu;
	//};

	//struct SmoothFunctor {
	//	 SmoothFunctor(float2 p, float2 np, float2 q, float2 nq, double weight)
	//	: _p(p), _np(np), _q(q), _nq(nq), _weight(weight) {}

	//	template <typename T>
	//	bool operator() (const T* const tp, const T* const tq, T* residual) const {
	//		//float2 pp = _p + tp[0] * _np;
	//		//float2 qq = _q + tq[0] * _nq;
	//		//float2 pq = pp - qq;
	//		const T px = (T)_p.x + tp[0] * (T)_np.x;
	//		const T py = (T)_p.y + tp[0] * (T)_np.y;
	//		const T qx = (T)_q.x + tq[0] * (T)_nq.x;
	//		const T qy = (T)_q.y + tq[0] * (T)_nq.y;
	//		const T pqx = px - qx;
	//		const T pqy = py - qy;
	//		const T qpx = qx - px;
	//		const T qpy = qy - py;
	//		residual[0] = (T)_weight * (pow(pqx*(T)_nq.x + pqy*(T)_nq.y, 2) +
	//									pow(qpx*(T)_np.x + qpy*(T)_np.y, 2));
	//		return true;
	//	}
	//private:
	//	const float2 _p;
	//	const float2 _np;
	//	const float2 _q;
	//	const float2 _nq;
	//	const double _weight;
	//};

	//void HWPlane::NewPolygonSmoothing(std::string filename)
	//{
	//	//google::InitGoogleLogging(nullptr);
	//	std::cout << "start polygon smoothing" << std::endl;

	//	Problem problem;
	//	std::vector<double> tp(edge_pnts_pos_num_, 0.0);
	//	double *data = tp.data();

	//	std::cout << "start adding residual" << std::endl;
	//	double mu = params_.polygon_smoothing_mu;
	//	if (mu == 0) {
	//		mu = 1.0;
	//		initial_params_.polygon_smoothing_mu = 1.0;
	//	}
	//	std::cout << "mu = " << mu << std::endl;
	//	for (int i = 0; i < edge_pnts_pos_num_; i++) {
	//		float2& p = edge_pnts_pos_[i];
	//		float2& np = edge_pnts_normal_[i];
	//		for (int j = 0; j < edge_pnts_neighs_[i].size(); j++) {
	//			int idx = edge_pnts_neighs_[i][j];
	//			if (idx == i) continue;
	//			float2& q = edge_pnts_pos_[idx];
	//			float2& nq = edge_pnts_normal_[idx];
	//			double weight = GetWeight(np.x, np.y, nq.x, nq.y);
	//			problem.AddResidualBlock(
	//				new AutoDiffCostFunction<SmoothFunctor, 1, 1, 1>(
	//					new SmoothFunctor(p, np, q, nq, weight)),
	//				nullptr,
	//				data + i,
	//				data + idx
	//			);
	//		}
	//		problem.AddResidualBlock(
	//			new AutoDiffCostFunction<InitFunctor, 1, 1>(
	//				new InitFunctor(mu)),
	//			nullptr,
	//			data + i
	//		);
	//	}

	//	Solver::Options options;
	//	options.max_num_iterations = 25;
	//	//options.linear_solver_type = ceres::DENSE_QR;
	//	options.minimizer_progress_to_stdout = true;

	//	Solver::Summary summary;
	//	Solve(options, &problem, &summary);
	//	std::cout << summary.BriefReport() << "\n";


	//	for (int i = 0; i < tp.size(); i++) {
	//		edge_pnts_pos_[i] += tp[i] * edge_pnts_normal_[i];
	//	}

	//	std::ofstream fh(filename + "edges_smoothed.obj");
	//	for (int i = 0; i < edge_pnts_pos_.size(); ++i)
	//	{
	//		fh << "v " << edge_pnts_pos_[i].x << " " << edge_pnts_pos_[i].y << " " << 0 << std::endl;
	//	}
	//	for (int i = 0; i < edge_pnts_pos_.size() - 1; i++) {
	//		fh << "l " << i + 1 << " " << i + 2 << "\n";
	//	}
	//	fh << "l " << 1 << " " << edge_pnts_pos_.size() << "\n";
	//	fh.close();


	//	//写入三维空间的边缘点和边缘
	//	std::vector<float3> points_3d;
	//	for (int i = 0; i < edge_pnts_pos_.size(); i++) {
	//		Eigen::Vector4f pnt(edge_pnts_pos_[i].x, edge_pnts_pos_[i].y, 0.0, 1.0);
	//		Eigen::Vector4f transform_pnt = plane_to_world_ * pnt;
	//		points_3d.emplace_back(make_float3(transform_pnt(0), transform_pnt(1), transform_pnt(2)));
	//	}

	//	fh.open(filename + "edges_smoothed_3d.obj");
	//	for (int i = 0; i < points_3d.size(); i++)
	//	{
	//		float3 start_pnt = points_3d[i];
	//		fh << "v " << start_pnt.x << " " << start_pnt.y << " " << start_pnt.z << std::endl;
	//	}
	//	for (int ii = 0; ii < points_3d.size() - 1; ii++) {
	//		fh << "l " << ii + 1 << " " << ii + 2 << "\n";
	//	}
	//	fh << "l " << 1 << " " << points_3d.size() << "\n";
	//	fh.close();
	//}

	void HWPlane::DoPolygonExtraction()
	{
		double theta_max = 10.0f;
		double adjacent_distance = 0.05;
		int edge_idx = 0;
		std::vector<int> edge_idxs(edge_pnts_pos_num_, -1);

		for (int i = 0; i < edge_pnts_pos_num_; i++) {
			int last_idx = (i + edge_pnts_pos_num_ - 1) % edge_pnts_pos_num_;
			int last_group = last_idx;
			//找到上一组的最后一个点
			while (edge_idxs[last_group] == -1 && edge_idx != 0) {
				last_group--;
			}
			int next_idx = (i + edge_pnts_pos_num_ + 1) % edge_pnts_pos_num_;
			//int last_last_idx = (i + edge_pnts_pos_num_ - 2) % edge_pnts_pos_num_;
			//int next_next_idx = (i + edge_pnts_pos_num_ + 2) % edge_pnts_pos_num_;
			float2 n = edge_pnts_normal_[i], n1 = edge_pnts_normal_[last_idx], n2 = edge_pnts_normal_[next_idx];
			float2 l = edge_pnts_pos_[i] - edge_pnts_pos_[last_idx];
			//n3 = edge_pnts_normal_[last_last_idx], n4 = edge_pnts_normal_[next_next_idx];
			float2 n0 = edge_pnts_normal_[last_group], e0 = edge_pnts_pos_[i] - edge_pnts_pos_[last_group];
			//如果上一个点还没确定，当前组不是第0组，
			//当前点的法向量和上一组的法向量接近，当前点的法向量和下一个点的法向量接近，当前点的法向量和（上一组的最后一个点到当前点的方向）接近垂直
			//则将当前点加入上一组当中
			if (edge_idxs[last_idx] == -1 && edge_idx != 0 && GetTheta(n.x, n.y, n0.x, n0.y) < theta_max
				&&GetTheta(n.x, n.y, n2.x, n2.y) < theta_max&&GetTheta(n.x, n.y, e0.x, e0.y)>75.0f) {
				//&&GetTheta(n.x, n.y, n0.x, n0.y) < theta_max&&GetTheta(n.x, n.y, e0.x, e0.y)>75.0f) {
				edge_idxs[i] = edge_idxs[last_group];
			}
			//否则，如果当前点的法向量和上一个点的法向量接近，和下一个点的法向量也接近
			else if (GetTheta(n.x, n.y, n1.x, n1.y) < theta_max && GetTheta(n.x, n.y, n2.x, n2.y) < theta_max) {
			//&&GetTheta(n.x, n.y, n3.x, n3.y) < theta_max&&GetTheta(n.x, n.y, n4.x, n4.y) < theta_max) {
				float2 edge_next = edge_pnts_pos_[i] - edge_pnts_pos_[next_idx];
				float2 edge_last = edge_pnts_pos_[i] - edge_pnts_pos_[last_idx];
				//如果下一个点已经分好组了，并且
				//当前点和下一个点的位置很接近，或者当前点的法向量和（下一个点到当前点的方向）接近垂直
				//则将当前点分到下一个点的组，并且把之前所有分到当前组的点都分到下一个点的组
				if (edge_idxs[next_idx] != -1 && (edge_next.x*edge_next.x + edge_next.y*edge_next.y<adjacent_distance || GetTheta(n.x, n.y, edge_next.x, edge_next.y)>75.0f)) {
					int idx = i;
					edge_idxs[idx] = edge_idxs[next_idx];
					while (edge_idxs[--idx] == edge_idx) {
						edge_idxs[idx] = edge_idxs[next_idx];
					}
					if (edge_idxs[last_idx] != -1 && edge_idx != 1)
						edge_idx--;
				}
				//否则，如果上一个点已经分好组，并且
				//当前点和上一个点的位置很接近，或者当前点的法向量和（上一个点到当前点的方向）接近垂直
				//则将当前点分到上一个点所在的组
				else if (edge_idxs[last_idx] != -1 && (edge_last.x*edge_last.x + edge_last.y*edge_last.y<adjacent_distance || GetTheta(n.x, n.y, edge_last.x, edge_last.y)>75.0f))
					edge_idxs[i] = edge_idxs[last_idx];
				//否则，将当前点划分到新的一组
				else
					edge_idxs[i] = ++edge_idx;
			}
		}

		for (int i = 0; i < edge_pnts_pos_num_; i++) {
			//如果i不等于0，第i和第i-1个点不在同一组，第i-1个点已经分好组
			//则新打印一行
			if (i != 0 && edge_idxs[i] != edge_idxs[i - 1] && edge_idxs[i - 1] == -1)
				printf("\ni: %d   :", i);
			printf("%d ", edge_idxs[i]);
		}
		printf("\n");

		//记录每组点的第一个点和最后一个点的序号
		std::vector<float2> first_lasts;
		first_lasts.resize(edge_idx);
		int mid_idx;
		bool has_thin_edge = false;
		//对于每一组点
		for (int idx = 0; idx < edge_idx; idx++) {
			std::vector<int> edges_pnts;
			int flag = 1;
			for (int i = 0; i < edge_pnts_pos_num_; i++) {
				//std::cout << "idx  i: " << idx << " " << i << "\n";
				int last_idx = (i + edge_pnts_pos_num_ - 1) % edge_pnts_pos_num_;
				if (edge_idxs[i] == (idx + 1)) {
					if (flag && edge_idxs[i] == 1 && edge_idxs[last_idx] != 1 && edges_pnts.size() != 0) {
						while (edge_idxs[last_idx] == -1)
							last_idx = (last_idx + edge_pnts_pos_num_ - 1) % edge_pnts_pos_num_;
						if (edge_idxs[i] < edge_idxs[last_idx]) {
							first_lasts[idx].x = i;
							mid_idx = edges_pnts[0];
							//std::cout << "edges_pnts.size(): " << edges_pnts.size() << "\n";
							first_lasts[idx].y = edges_pnts.back();
							//std::cout << "idx irst_lasts[idx]: " << idx << "  " << first_lasts[idx].x << "  " << first_lasts[idx].y << "\n";
							flag = 0;
						}
					}
					edges_pnts.emplace_back(i);
				}
			}
			if (flag) {
				//std::cout << "edges_pnts.size(): " << edges_pnts.size() << "\n";
				first_lasts[idx].x = edges_pnts[0];
				first_lasts[idx].y = edges_pnts.back();
				mid_idx = edges_pnts[edges_pnts.size() / 2];
			}
			if (edges_pnts.size() >= 5) {
				Eigen::Vector2f coeff = FittingLine(edges_pnts);
				float2 start, mid, end;
				start = edge_pnts_pos_[first_lasts[idx].x];
				mid = edge_pnts_pos_[mid_idx];
				end = edge_pnts_pos_[first_lasts[idx].y];

				float2 start_proj, mid_proj, end_proj;
				start_proj = ProjectToLine(start, coeff);
				mid_proj = ProjectToLine(mid, coeff);
				end_proj = ProjectToLine(end, coeff);
				float dist = PointPointDist(start_proj, end_proj);
				float dist1 = PointPointDist(start_proj, mid);
				float dist2 = PointPointDist(mid, end_proj);
				//这里没看懂
				//why?
				if (dist < max(dist1, dist2)) {
					//std::cout << "idx: " << idx << "\n";
					/*std::cout << "dist: " << dist << "  " << dist1 << "  " << dist2 << "\n";
					std::cout << "start: " << start.x << "  " << start.y << "\n";
					std::cout << "end: " << end.x << "  " << end.y << "\n";
					std::cout << "start_next: " << start_next.x << "  " << start_next.y << "\n";
					std::cout << "end_pre: " << end.x << "  " << end.y << "\n";*/
					has_thin_edge = true;
				}
				edges_.emplace_back(coeff);
				edge_idx_.emplace_back(idx);
			}

		}

		std::cout << "edges_.size:" << edges_.size() << "\n";

		/*if (edges_.size() == 1) {
			for (int i = 0; i < tp_.size(); i++) {
				edge_pnts_pos_[i] -= tp_[i] * edge_pnts_normal_[i];
			}
			diameter_max_ /= 2.0f;
			GetSmoothRegions();
			GetNeighbors();
			GetInitialNormals();
			DoNormalEstimation();
			DoPolygonSmoothing();
			DoPolygonExtraction();
			return;
		}*/
		if (edges_.size() <= 1)
			return; 
		if (edges_.size() > 1 && has_thin_edge) {
			//system("pause");
			printf("has_thin_edge!!!!!!!!!!!!!!!!!!!!\n");
			//return;
		}
		if (edges_.size() == 2) {
			int this_first = first_lasts[edge_idx_[0]].x, this_last = first_lasts[edge_idx_[0]].y;
			int next_first = first_lasts[edge_idx_[1]].x, next_last = first_lasts[edge_idx_[1]].y;
			float2 p1 = ProjectToLine(edge_pnts_pos_[this_first], edges_[0]);
			float2 p2 = ProjectToLine(edge_pnts_pos_[this_last], edges_[0]);
			float2 p3 = ProjectToLine(edge_pnts_pos_[next_first], edges_[1]);
			float2 p4 = ProjectToLine(edge_pnts_pos_[next_last], edges_[1]);
			corner_pnts_.emplace_back(p1);
			corner_pnts_.emplace_back(p2);
			corner_pnts_.emplace_back(p3);
			corner_pnts_.emplace_back(p4);
			for (int i = 0; i < corner_pnts_.size(); i++) {
				if (IsSelfIntersection(i, corner_pnts_[i])) {
					corner_pnts_[1] = p3;
					corner_pnts_[2] = p2;
					break;
				}
			}
			//for (int i = 0; i < corner_pnts_.size(); i++) {
			//	if (IsSelfIntersection(i, corner_pnts_[i])) {
			//		printf("2222\n");
			//		/*corner_pnts_[0] = p2;
			//		corner_pnts_[1] = p1;*/
			//		break;
			//	}
			//}
		}
		else {
			for (int i = 0; i < edges_.size(); i++) {
				int next_idx = (i + edges_.size() + 1) % edges_.size();
				//当前边和下一条边之间的夹角
				float theta = GetTheta(edges_[i](0), edges_[i](1), edges_[next_idx](0), edges_[next_idx](1));
				//当前边的最后一点，和下一条边的第一个点
				int this_last = first_lasts[edge_idx_[i]].y, next_first = first_lasts[edge_idx_[next_idx]].x;
				float2 p = GetIntersectionPoint(edges_[i], edges_[next_idx]);
				float2 p1 = ProjectToLine(edge_pnts_pos_[this_last], edges_[i]);
				float2 p2 = ProjectToLine(edge_pnts_pos_[next_first], edges_[next_idx]);
				float value = 1;
				//bool self_intersection = false;
				if (corner_pnts_.size() > 0) {
					float2 p_last = corner_pnts_.back();
					float2 edge = p - p_last;
					value = dot(edge, p1 - p_last);

					/*if (corner_pnts_.size() > 1) {
						Eigen::Vector2f edge_last = FittingLine(p, p_last);
						for (int j = 0; j < corner_pnts_.size() - 2; j++) {
							float2 p3 = corner_pnts_[j];
							float2 p4 = corner_pnts_[j + 1];
							Eigen::Vector2f edge_pre = FittingLine(p3, p4);
							float2 p_i = GetIntersectionPoint(edge_last, edge_pre);
							if (dot(p_i - p, p_i - p_last) < 0 && dot(p_i - p3, p_i - p4) < 0) {
								self_intersection = true;
								std::cout << "i j:" << i << " " << j << "\n";
								break;
							}
						}
					}*/
				}
				/*if (self_intersection)
					continue;*/
				//如果当前边和下一条边之间的夹角>20度，且p和p1在当前最后一个顶点的同一侧，则将p作为新的顶点加入到顶点向量中
				if (theta > 20.0f&&value > 0) {//|| PointPointDist(p, edge_pnts_pos_[this_last]) < 0.1 || PointPointDist(p, edge_pnts_pos_[next_first]) < 0.1) {
					corner_pnts_.emplace_back(p);
					std::cout << "p: " << p.x << " " << p.y << "\n";
				}
				//否则，将p1和p2作为两个顶点加入到顶点向量中
				else {
					/*std::cout << "this_last: " << this_last << "\n";
					std::cout << "next_first: " << next_first << "\n";
					std::cout << "p1: " << p1.x << " " << p1.y << "\n";
					std::cout << "p2: " << p2.x << " " << p2.y << "\n";*/
					corner_pnts_.emplace_back(p1);
					corner_pnts_.emplace_back(p2);
				}
			}
		}


		//liulingfei
		//double theta_max = 20;
		//corner_pnts_.clear();
		//for (int i = 1; i < edge_pnts_pos_num_; i++) {
		//	int last_idx = (i + edge_pnts_pos_num_ - 1) % edge_pnts_pos_num_;
		//	int next_idx = (i + edge_pnts_pos_num_ + 1) % edge_pnts_pos_num_;
		//	float2 n = edge_pnts_normal_[i];
		//	float2 n1 = edge_pnts_normal_[last_idx];
		//	float2 n2 = edge_pnts_normal_[next_idx];
		//	if (GetTheta(n.x, n.y, n1.x, n1.y) > theta_max) {
		//		//GetTheta(n.x, n.y, n2.x, n2.y) > theta_max) {
		//		corner_pnts_.push_back(edge_pnts_pos_[i]);
		//	}
		//}
		//std::cout << "corner_pnts_.size() = " << corner_pnts_.size() << std::endl;

		//去除距离非常近的相邻顶点
		//取它们之间的中点代替它们
		double min_distance = 0.01;
		std::vector<float2> temp, temp2;
		for (int i = 0; i < corner_pnts_.size(); i++) {
			if (i == 0)
				temp.emplace_back(corner_pnts_[i]);
			else {
				float2 p = temp.back();
				if (PointPointDist(p, corner_pnts_[i]) < min_distance)
					temp.back() = (p + corner_pnts_[i]) / 2;
				else
					temp.emplace_back(corner_pnts_[i]);
			}
		}
		if (PointPointDist(temp[0], temp.back()) < min_distance) {
			temp[0] = (temp[0] + temp.back()) / 2;
			temp.pop_back();
		}
		std::cout << "after deleting near points, corner_pnts_.size() = " << corner_pnts_.size() << std::endl;

		//std::cout << "temp: " << temp.size() << "\n";
		//去除三点共线的情况
		//如果有三点共线，去掉中间的点
		for (int i = 0; i < temp.size(); i++) {
			int last = (i - 1 + temp.size()) % temp.size();
			int next = (i + 1 + temp.size()) % temp.size();
			//if (PointLineSegmentDist(temp[i], temp[last], temp[next]) > 0.01)
			if (PointLineSegmentDist(temp[i], temp[last], temp[next]) > 0.01)
				temp2.emplace_back(temp[i]);
			//float2 l1 = temp[i] - temp[last];
			//float2 l2 = temp[next] - temp[i];
			//float theta = GetTheta(l1.x, l1.y, l2.x, l2.y);
			//if (theta > 10) temp2.emplace_back(temp[i]);
		}

		//如果最终剩下的顶点少于3个，则直接返回
		if(temp2.size()>2)
			corner_pnts_ = temp2;
		else {
			corner_pnts_.clear();
			return;
		}

		std::cout << "after deleting colinear points, corner_pnts_.size() = " << corner_pnts_.size() << std::endl;

		//解决多边形的边自相交的问题
		bool self_intersection = true;
		while (self_intersection) {
			self_intersection = false;
			float delete_edge_length = FLT_MAX;
			int delete_edge_idx;
			//先解决（与顶点相邻的两条边）与（多边形的另外两条边）相交的情况
			//直接删除这个顶点
			for (int i = 0; i < corner_pnts_.size(); i++) {
				if (IsSelfIntersection(i, corner_pnts_[i]) == 2) {
					std::vector<float2> temp(corner_pnts_.begin() + i + 1, corner_pnts_.end());
					corner_pnts_.resize(i);
					corner_pnts_.insert(corner_pnts_.begin(), temp.begin(), temp.end());
					self_intersection = true;
					break;
				}
			}
			if (self_intersection)
				continue;
			//再解决相交一次的情况
			//直接删除这个顶点
			for (int i = 0; i < corner_pnts_.size(); i++) {
				if (IsSelfIntersection(i, corner_pnts_[i]) == 1) {
					std::vector<float2> temp(corner_pnts_.begin() + i + 1, corner_pnts_.end());
					corner_pnts_.resize(i);
					corner_pnts_.insert(corner_pnts_.begin(), temp.begin(), temp.end());
					self_intersection = true;
					break;
				}
			}
		}

		std::cout << "after deleting intersecting points, corner_pnts_.size() = " << corner_pnts_.size() << std::endl;

		float2 edge1 = corner_pnts_[1] - corner_pnts_[0];
		float2 edge2 = corner_pnts_[2] - corner_pnts_[1];
		if (edge1.x*edge2.y - edge1.y*edge2.x == 0) {
			std::cout << "edge1: " << edge1.x << " " << edge1.y << "\n";
			std::cout << "edge2: " << edge2.x << " " << edge2.y << "\n";
			system("pause");
		}

		//解决定点确定的多边形法向量反向的问题
		//将顶点向量中的点逆序排列
		if (edge1.x*edge2.y - edge1.y*edge2.x > 0 ^ IsConcaveVertex(1)==1) {
			std::vector<float2> corner_pnts_inverse;
			for (int i = corner_pnts_.size() - 1; i >= 0; i--) {
				corner_pnts_inverse.emplace_back(corner_pnts_[i]);
			}
			corner_pnts_ = corner_pnts_inverse;
		}
		for (int i = 0; i < corner_pnts_.size(); i++) {
			Eigen::Vector4f pnt(corner_pnts_[i].x, corner_pnts_[i].y, 0.0, 1.0);
			Eigen::Vector4f transform_pnt = plane_to_world_*pnt;
			corner_pnts_3d_.emplace_back(make_float3(transform_pnt(0), transform_pnt(1), transform_pnt(2)));
		}
		/*float3 edge1 = corner_pnts_3d_[1] - corner_pnts_3d_[0];
		float3 edge2 = corner_pnts_3d_[2] - corner_pnts_3d_[1];
		Eigen::Vector3f e1(edge1.x, edge1.y, edge1.z), e2(edge2.x, edge2.y, edge2.z);
		Eigen::Vector3f polygon_normal = e1.cross(e2);
		std::cout << "polygon_normal:" << polygon_normal.transpose() << "\n";
		if (polygon_normal.dot(average_pnts_normal_) <= 0 ^ IsConcaveVertex(1) == 1) {
			std::vector<float3> corner_pnts_3d_inverse;
			for (int i = corner_pnts_3d_.size() - 1; i >= 0; i--) {
				corner_pnts_3d_inverse.emplace_back(corner_pnts_3d_[i]);
			}
			corner_pnts_3d_ = corner_pnts_3d_inverse;
			std::vector<float2> corner_pnts_inverse;
			for (int i = corner_pnts_.size() - 1; i >= 0; i--) {
				corner_pnts_inverse.emplace_back(corner_pnts_[i]);
			}
			corner_pnts_ = corner_pnts_inverse;
		}*/

		UpdateInformation();
	}

	void HWPlane::NewPolygonExtraction()
	{
		double theta_max = params_.max_theta;
		if (theta_max == 0) {
			theta_max = 10;
			initial_params_.max_theta = 10;
		}
		std::cout << "theta_max = " << theta_max << std::endl;
		corner_pnts_.clear();
		corner_pnts_3d_.clear();
		std::vector<bool> is_edge_points(edge_pnts_pos_num_, false);
		std::vector<std::vector<int>> edge_idx;

		for (int i = 0; i < edge_pnts_pos_num_; i++) {
			int last_idx = (i + edge_pnts_pos_num_ - 1) % edge_pnts_pos_num_;
			int next_idx = (i + edge_pnts_pos_num_ + 1) % edge_pnts_pos_num_;
			float2 n = edge_pnts_normal_[i];
			float2 n1 = edge_pnts_normal_[last_idx];
			float2 n2 = edge_pnts_normal_[next_idx];
			if (GetTheta(n.x, n.y, n1.x, n1.y) < theta_max &&
				GetTheta(n.x, n.y, n2.x, n2.y) < theta_max) {
				is_edge_points[i] = true;
			}
		}
		std::cout << "a" << std::endl;

		//int count = 0;
		//for (int i = 0; i < edge_pnts_pos_num_; i++) if (!is_edge_points[i]) count++;
		//if (count < 3) return;

		int first_edge_point_idx = 0;
		//找到第一个edge point
		while (first_edge_point_idx < edge_pnts_pos_num_ && is_edge_points[first_edge_point_idx]) first_edge_point_idx++;
		while (first_edge_point_idx < edge_pnts_pos_num_ && !is_edge_points[first_edge_point_idx]) first_edge_point_idx++;
		if (first_edge_point_idx == edge_pnts_pos_num_) return;
		//for (int i = 0; i < is_edge_points.size(); i++) std::cout << i << ": " << is_edge_points[i] << std::endl;

		bool has_thin_edge = false;
		edges_.clear();
		int curr_idx = first_edge_point_idx;
		while (true) {
			std::vector<int> curr_edge;
			while (is_edge_points[curr_idx]) {
				curr_edge.push_back(curr_idx);
				int old_idx = curr_idx;
				curr_idx = (curr_idx + 1) % edge_pnts_pos_num_;

				if (curr_edge.size() < 2)
				{
					continue;
				}
				Eigen::Vector2f coeff = FittingLine(curr_edge);
				if (abs(coeff(0)) < 1E-6 && abs(coeff(1)) < 1E-6)
				{
					continue;
				}
				float max = -FLT_MAX, min = FLT_MAX;
				for (int k = 0; k < curr_edge.size(); k++) {
					int idx = curr_edge[k];
					float d = PointLineDist(edge_pnts_pos_[idx], coeff);
					if (d > max)
						max = d;
					if (d < min)
						min = d;
				}
				double region_diameter = max - min;
				if (region_diameter > diameter_max_) {
					//edge_pnts_smooth_regions_[i].pop_back();
					curr_edge.pop_back();
					curr_idx = old_idx;
					break;
				}
			}
			//如果这条线段的点数大于5，则拟合这条线段
			if (curr_edge.size() > 5) {
				Eigen::Vector2f coeff = FittingLine(curr_edge);

				float2 start, end, mid;
				start= edge_pnts_pos_[curr_edge[0]];
				mid= edge_pnts_pos_[curr_edge[curr_edge.size() / 2]];
				end= edge_pnts_pos_[curr_edge.back()];

				float2 start_proj, mid_proj, end_proj;
				start_proj = ProjectToLine(start, coeff);
				mid_proj = ProjectToLine(mid, coeff);
				end_proj = ProjectToLine(end, coeff);
				float dist = PointPointDist(start_proj, end_proj);
				float dist1 = PointPointDist(start_proj, mid);
				float dist2 = PointPointDist(mid, end_proj);
				//这里没看懂
				//why?
				if (dist < max(dist1, dist2)) {
					has_thin_edge = true;
				}
				edges_.emplace_back(coeff);
				edge_idx.push_back(curr_edge);
			}

			while (!is_edge_points[curr_idx]) {
				curr_idx = (curr_idx + 1) % edge_pnts_pos_num_;
			}

			//如果又回到起点，说明全部遍历了一遍
			if (curr_idx == first_edge_point_idx) break;
			//else {
				//std::cout << "first_edge_point_idx: " << first_edge_point_idx << ", curr_idx: " << curr_idx << std::endl;
			//}
		}
		std::cout << "b" << std::endl;

		int edge_num = edges_.size();

		if (edges_.size() <= 1)
			return;
		if (edges_.size() > 1 && has_thin_edge) {
			printf("has_thin_edge!!!!!!!!!!!!!!!!!!!!\n");
		}
		if (edges_.size() == 2) {
			//int this_first = first_lasts[edge_idx_[0]].x, this_last = first_lasts[edge_idx_[0]].y;
			//int next_first = first_lasts[edge_idx_[1]].x, next_last = first_lasts[edge_idx_[1]].y;
			int this_first = edge_idx[0][0];
			int this_last = edge_idx[0].back();
			int next_first = edge_idx[1][0];
			int next_last = edge_idx[1].back();
			float2 p1 = ProjectToLine(edge_pnts_pos_[this_first], edges_[0]);
			float2 p2 = ProjectToLine(edge_pnts_pos_[this_last], edges_[0]);
			float2 p3 = ProjectToLine(edge_pnts_pos_[next_first], edges_[1]);
			float2 p4 = ProjectToLine(edge_pnts_pos_[next_last], edges_[1]);
			corner_pnts_.emplace_back(p1);
			corner_pnts_.emplace_back(p2);
			corner_pnts_.emplace_back(p3);
			corner_pnts_.emplace_back(p4);
			for (int i = 0; i < corner_pnts_.size(); i++) {
				if (IsSelfIntersection(i, corner_pnts_[i])) {
					corner_pnts_[1] = p3;
					corner_pnts_[2] = p2;
					break;
				}
			}
		}
		else {
			for (int i = 0; i < edges_.size(); i++) {
				int next_idx = (i + edges_.size() + 1) % edges_.size();
				//当前边和下一条边之间的夹角
				float theta = GetTheta(edges_[i](0), edges_[i](1), edges_[next_idx](0), edges_[next_idx](1));
				//当前边的最后一点，和下一条边的第一个点
				//int this_last = first_lasts[edge_idx_[i]].y, next_first = first_lasts[edge_idx_[next_idx]].x;
				int this_last = edge_idx[i].back();
				int next_first = edge_idx[next_idx][0];
				float2 p = GetIntersectionPoint(edges_[i], edges_[next_idx]);
				float2 p1 = ProjectToLine(edge_pnts_pos_[this_last], edges_[i]);
				float2 p2 = ProjectToLine(edge_pnts_pos_[next_first], edges_[next_idx]);
				float value = 1;
				//bool self_intersection = false;
				if (corner_pnts_.size() > 0) {
					float2 p_last = corner_pnts_.back();
					float2 edge = p - p_last;
					value = dot(edge, p1 - p_last);
				}
				//如果当前边和下一条边之间的夹角大于theta_max度，且p和p1在当前最后一个顶点的同一侧，则将p作为新的顶点加入到顶点向量中
				if (!isinf(p.x) && !isinf(p.y) && theta > theta_max && value > 0) {//|| PointPointDist(p, edge_pnts_pos_[this_last]) < 0.1 || PointPointDist(p, edge_pnts_pos_[next_first]) < 0.1) {
					corner_pnts_.emplace_back(p);
					std::cout << "p: " << p.x << " " << p.y << "\n";
				}
				//否则，将p1和p2作为两个顶点加入到顶点向量中
				else {
					/*std::cout << "this_last: " << this_last << "\n";
					std::cout << "next_first: " << next_first << "\n";
					std::cout << "p1: " << p1.x << " " << p1.y << "\n";
					std::cout << "p2: " << p2.x << " " << p2.y << "\n";*/
					corner_pnts_.emplace_back(p1);
					corner_pnts_.emplace_back(p2);
				}
			}
		}
		std::cout << "before deleting near points, corner_pnts_.size() = " << corner_pnts_.size() << std::endl;

		////去除距离非常近的相邻顶点
		////取它们之间的中点代替它们
		//double min_distance = params_.min_corner_points_distance;
		//if (min_distance == 0) {
		//	//1cm的间距太小了
		//	//min_distance = std::max((double)ave_alpha_edge_len_, 0.01);
		//	//initial_params_.min_corner_points_distance = std::max((double)ave_alpha_edge_len_, 0.01);
		//	min_distance = std::max((double)ave_alpha_edge_len_, plane_width_ * 0.01);
		//	initial_params_.min_corner_points_distance = std::max((double)ave_alpha_edge_len_, plane_width_ * 0.1);
		//}
		//std::cout << "min_distance = " << min_distance << std::endl;
		//std::vector<float2> temp, temp2;
		//for (int i = 0; i < corner_pnts_.size(); i++) {
		//	if (i == 0)
		//		temp.emplace_back(corner_pnts_[i]);
		//	else {
		//		float2 p = temp.back();
		//		if (PointPointDist(p, corner_pnts_[i]) < min_distance)
		//			temp.back() = (p + corner_pnts_[i]) / 2;
		//		else
		//			temp.emplace_back(corner_pnts_[i]);
		//	}
		//}
		//if (PointPointDist(temp[0], temp.back()) < min_distance) {
		//	temp[0] = (temp[0] + temp.back()) / 2;
		//	temp.pop_back();
		//}
		//std::cout << "after deleting near points, corner_pnts_.size() = " << temp.size() << std::endl;

		//fuseNearbyPnts();
		if (corner_pnts_.size() < 3) {
			corner_pnts_.clear();
			corner_pnts_3d_.clear();
			return;
		}

		////std::cout << "temp: " << temp.size() << "\n";
		////去除三点共线的情况
		////如果有三点共线，去掉中间的点
		//for (int i = 0; i < temp.size(); i++) {
		//	int last = (i - 1 + temp.size()) % temp.size();
		//	int next = (i + 1 + temp.size()) % temp.size();
		//	//中间点到两端点连成的直线的距离大于1cm，则判定为不共线，距离太小了
		//	//if (PointLineSegmentDist(temp[i], temp[last], temp[next]) > 0.01)
		//	//plane_width_是包围盒较短的边的长度
		//	if (PointLineSegmentDist(temp[i], temp[last], temp[next]) > plane_width_ * 0.01) 
		//		temp2.emplace_back(temp[i]);
		//	//float2 l1 = temp[i] - temp[last];
		//	//float2 l2 = temp[next] - temp[i];
		//	//float theta = GetTheta(l1.x, l1.y, l2.x, l2.y);
		//	//if (theta > 10) temp2.emplace_back(temp[i]);
		//}

		////如果最终剩下的顶点少于3个，则直接返回
		//if (temp2.size() > 2)
		//	corner_pnts_ = temp2;
		//else {
		//	corner_pnts_.clear();
		//	return;
		//}
		deleteCollinearPnts();
		if (corner_pnts_.size() < 3) {
			corner_pnts_.clear();
			corner_pnts_3d_.clear();
			return;
		}

		//解决多边形的边自相交的问题
		bool self_intersection = true;
		while (self_intersection) {
			self_intersection = false;
			float delete_edge_length = FLT_MAX;
			int delete_edge_idx;
			//先解决（与顶点相邻的两条边）与（多边形的另外两条边）相交的情况
			//直接删除这个顶点
			for (int i = 0; i < corner_pnts_.size(); i++) {
				if (IsSelfIntersection(i, corner_pnts_[i]) == 2) {
					std::vector<float2> temp(corner_pnts_.begin() + i + 1, corner_pnts_.end());
					corner_pnts_.resize(i);
					corner_pnts_.insert(corner_pnts_.begin(), temp.begin(), temp.end());
					self_intersection = true;
					break;
				}
			}
			if (self_intersection)
				continue;
			//再解决相交一次的情况
			//直接删除这个顶点
			for (int i = 0; i < corner_pnts_.size(); i++) {
				if (IsSelfIntersection(i, corner_pnts_[i]) == 1) {
					std::vector<float2> temp(corner_pnts_.begin() + i + 1, corner_pnts_.end());
					corner_pnts_.resize(i);
					corner_pnts_.insert(corner_pnts_.begin(), temp.begin(), temp.end());
					self_intersection = true;
					break;
				}
			}
		}

		std::cout << "after deleting intersecting points, corner_pnts_.size() = " << corner_pnts_.size() << std::endl;

		//float2 edge1 = corner_pnts_[1] - corner_pnts_[0];
		//float2 edge2 = corner_pnts_[2] - corner_pnts_[1];
		//if (edge1.x*edge2.y - edge1.y*edge2.x == 0) {
		//	std::cout << "edge1: " << edge1.x << " " << edge1.y << "\n";
		//	std::cout << "edge2: " << edge2.x << " " << edge2.y << "\n";
		//}

		//解决定点确定的多边形法向量反向的问题
		//将顶点向量中的点逆序排列
		//if (edge1.x*edge2.y - edge1.y*edge2.x > 0 ^ IsConcaveVertex(1) == 1) {
		//	std::vector<float2> corner_pnts_inverse;
		//	for (int i = corner_pnts_.size() - 1; i >= 0; i--) {
		//		corner_pnts_inverse.emplace_back(corner_pnts_[i]);
		//	}
		//	corner_pnts_ = corner_pnts_inverse;
		//}

		ComputeCornerPnts3dFromCornerPnts2d();
		//UpdateInformation();
	}

	void HWPlane::NewPolygonExtractionZDG()
	{
		double theta_max = params_.max_theta;
		if (theta_max == 0) {
			theta_max = 10;
			initial_params_.max_theta = 10;
		}
		std::cout << "theta_max = " << theta_max << std::endl;
		
		std::vector<bool> is_edge_points(edge_pnts_pos_num_, false);
		std::vector<std::vector<int>> edge_idx;

		for (int i = 0; i < edge_pnts_pos_num_; i++) {
			int last_idx = (i + edge_pnts_pos_num_ - 1) % edge_pnts_pos_num_;
			int next_idx = (i + edge_pnts_pos_num_ + 1) % edge_pnts_pos_num_;
			float2 n = edge_pnts_normal_[i];
			float2 n1 = edge_pnts_normal_[last_idx];
			float2 n2 = edge_pnts_normal_[next_idx];
			if (GetTheta(n.x, n.y, n1.x, n1.y) < theta_max &&
				GetTheta(n.x, n.y, n2.x, n2.y) < theta_max) {
				is_edge_points[i] = true;
			}
		}
		std::cout << "a" << std::endl;

#if 0//HW_DEBUG
		//test
		std::vector<Eigen::Vector3f> all_edges_pnts;
		std::vector<Eigen::Vector3f> all_edges_pnts_normals;
		//is_edge_points
		std::vector<Eigen::Vector3f> all_edges_pnts_color;
		float len_normal = 0.01;
		//assign color to pnt by is_edge_points;
		for (int i = 0; i < edge_pnts_pos_.size(); ++i)
		{
			Eigen::Vector4f tmp_pnt = Eigen::Vector4f(edge_pnts_pos_[i].x, edge_pnts_pos_[i].y, 0.0, 1.0);
			Eigen::Vector2f nml(edge_pnts_normal_[i].x, edge_pnts_normal_[i].y);
			nml.normalize();
			Eigen::Vector4f pnt_end(edge_pnts_pos_[i].x + nml.x(), edge_pnts_pos_[i].y + nml.y(), 0.0, 1.0);
			Eigen::Vector4f transform_pnt = plane_to_world_ * tmp_pnt;
			Eigen::Vector4f transform_end = plane_to_world_ * pnt_end;
			Eigen::Vector3f transform_pnt_3d = Eigen::Vector3f(transform_pnt[0], transform_pnt[1], transform_pnt[2]);
			Eigen::Vector3f transform_end_pnt_3d = Eigen::Vector3f(transform_end[0], transform_end[1], transform_end[2]);
			Eigen::Vector3f transform_normal = transform_end_pnt_3d - transform_pnt_3d;
			transform_normal.normalize();
			all_edges_pnts.emplace_back(transform_pnt_3d);
			all_edges_pnts_normals.emplace_back(transform_normal);
			if (is_edge_points[i])
			{
				Eigen::Vector3f color_pnt = Eigen::Vector3f(1.0, 0.0, 0.0);
				all_edges_pnts_color.emplace_back(color_pnt);
			}
			else
			{
				Eigen::Vector3f color_pnt = Eigen::Vector3f(0.0, 1.0, 0.0);
				all_edges_pnts_color.emplace_back(color_pnt);
			}
		}
		std::string out_pnts_path = filename_ + "_is_edges_pnts_normal_3d.obj";
		WriteOriginEdgesPnts3DByFlag(out_pnts_path, all_edges_pnts,
			all_edges_pnts_normals, len_normal, all_edges_pnts_color, is_edge_points);
		//end test
#endif

		//int count = 0;
		//for (int i = 0; i < edge_pnts_pos_num_; i++) if (!is_edge_points[i]) count++;
		//if (count < 3) return;

		//copy std::vector<HWOptiPnt2D>
		CopyExractionEdgePntsPosToInitialEdgePnts();

		int first_edge_point_idx = 0;
		//找到第一个edge point
		while (first_edge_point_idx < edge_pnts_pos_num_ && is_edge_points[first_edge_point_idx]) first_edge_point_idx++;
		while (first_edge_point_idx < edge_pnts_pos_num_ && !is_edge_points[first_edge_point_idx]) first_edge_point_idx++;
		if (first_edge_point_idx == edge_pnts_pos_num_) return;
		//for (int i = 0; i < is_edge_points.size(); i++) std::cout << i << ": " << is_edge_points[i] << std::endl;

		std::vector<HWLineToPntsIdxs> initial_pnts_idxs_values;
		bool has_thin_edge = false;
		edges_.clear();
		int curr_idx = first_edge_point_idx;
		while (true) {
			HWLineToPntsIdxs cur_fun_edge_idxs;
			std::vector<int> curr_edge_idxs;
			while (is_edge_points[curr_idx]) {
				curr_edge_idxs.push_back(curr_idx);
				int old_idx = curr_idx;
				curr_idx = (curr_idx + 1) % edge_pnts_pos_num_;

				if (curr_edge_idxs.size() < 2)
				{
					continue;
				}
				//Eigen::Vector2f coeff = FittingLine(curr_edge_idxs);
				Eigen::Vector3f coeff_fun = FittingLineFromPntsIdxs(curr_edge_idxs);
				if (abs(coeff_fun(0)) < KMIN_FLOAT_THRESHOLD_REFINED 
					&& abs(coeff_fun(1)) < KMIN_FLOAT_THRESHOLD_REFINED)
				{
					continue;
				}
				float max = -FLT_MAX, min = FLT_MAX;
				for (int k = 0; k < curr_edge_idxs.size(); k++) {
					int idx = curr_edge_idxs[k];
					//float d = PointLineDist(edge_pnts_pos_[idx], coeff);
					Eigen::Vector2f tmp_edge_pnt = Eigen::Vector2f(edge_pnts_pos_[idx].x, 
						edge_pnts_pos_[idx].y);
					float d = ComputePntToLineFunction2D(coeff_fun, tmp_edge_pnt);
					if (d > max)
						max = d;
					if (d < min)
						min = d;
				}
				double region_diameter = max - min;
				if (region_diameter > diameter_max_) {
					//edge_pnts_smooth_regions_[i].pop_back();
					curr_edge_idxs.pop_back();
					curr_idx = old_idx;
					break;
				}
			}
			//如果这条线段的点数大于5，则拟合这条线段
			if (curr_edge_idxs.size() > edges_pnts_fitting_num_threshold_) {

				//Eigen::Vector2f coeff = FittingLine(curr_edge_idxs);
				Eigen::Vector3f coeff_fun = FittingLineFromPntsIdxs(curr_edge_idxs);
				
				/*
				float2 start, end, mid;
				start = edge_pnts_pos_[curr_edge_idxs[0]];
				mid = edge_pnts_pos_[curr_edge_idxs[curr_edge_idxs.size() / 2]];
				end = edge_pnts_pos_[curr_edge_idxs.back()];
				float2 start_proj, mid_proj, end_proj;
				start_proj = ProjectToLine(start, coeff);
				mid_proj = ProjectToLine(mid, coeff);
				end_proj = ProjectToLine(end, coeff);
				float dist = PointPointDist(start_proj, end_proj);
				float dist1 = PointPointDist(start_proj, mid);
				float dist2 = PointPointDist(mid, end_proj);
				//这里没看懂
				//why?
				if (dist < max(dist1, dist2)) {
					has_thin_edge = true;
				}
				*/

				Eigen::Vector2f start, end, mid;
				start[0] = edge_pnts_pos_[curr_edge_idxs[0]].x;
				start[1] = edge_pnts_pos_[curr_edge_idxs[0]].y;
				mid[0] = edge_pnts_pos_[curr_edge_idxs[curr_edge_idxs.size() / 2]].x;
				mid[1] = edge_pnts_pos_[curr_edge_idxs[curr_edge_idxs.size() / 2]].y;
				end[0] = edge_pnts_pos_[curr_edge_idxs.back()].x;
				end[1] = edge_pnts_pos_[curr_edge_idxs.back()].y;
				Eigen::Vector2f start_proj, mid_proj, end_proj;
				ComputePntProjToLineFunction2D(coeff_fun, start, start_proj);
				ComputePntProjToLineFunction2D(coeff_fun, mid, mid_proj);
				ComputePntProjToLineFunction2D(coeff_fun, end, end_proj);
				float dist = (start_proj - end_proj).norm();
				float dist1 = (start_proj - mid).norm();
				float dist2 = (mid - end_proj).norm();
				if (dist < max(dist1, dist2)) 
				{
					has_thin_edge = true;
				}
				cur_fun_edge_idxs.line_fun_ = coeff_fun;
				cur_fun_edge_idxs.associated_pnts_idxs_ = curr_edge_idxs;
				//edges_.emplace_back(coeff_fun);
				//edge_idx.push_back(curr_edge_idxs);

				initial_pnts_idxs_values.emplace_back(cur_fun_edge_idxs);
			}
			while (!is_edge_points[curr_idx]) {
				curr_idx = (curr_idx + 1) % edge_pnts_pos_num_;
			}

			//如果又回到起点，说明全部遍历了一遍
			if (curr_idx == first_edge_point_idx) break;
			//else {
			//std::cout << "first_edge_point_idx: " << first_edge_point_idx << ", curr_idx: " << curr_idx << std::endl;
			//}
		}
		std::cout << "b" << std::endl;

		int edge_num = initial_pnts_idxs_values.size();

		if (initial_pnts_idxs_values.size() <= 1)
			return;
		if (initial_pnts_idxs_values.size() > 1 && has_thin_edge) {
			printf("has_thin_edge!!!!!!!!!!!!!!!!!!!!\n");
		}
		initial_funs_coeffs_ = initial_pnts_idxs_values;
#if HW_DEBUG
		//test by zdg
		if (true)
		{
			std::vector<Eigen::Vector2f> poly_lines;
			std::vector<Eigen::Vector3f> tmp_lines_funs;
			for (int j = 0; j < initial_funs_coeffs_.size(); ++j)
			{
				Eigen::Vector3f tmp_cur_f_coeff = initial_funs_coeffs_[j].line_fun_;
				tmp_lines_funs.emplace_back(tmp_cur_f_coeff);
			}
			PolygonExtractionFromSortedLinesFunctions(tmp_lines_funs, poly_lines);
			std::cerr << "optimize the polygon lines number: " << poly_lines.size() << std::endl;
			std::vector<Eigen::Vector2f> initial_corner_pnt2d;
			//copy to corner pnts
			for (int i = 0; i < poly_lines.size(); ++i)
			{
				Eigen::Vector2f tmp_pnt;
				tmp_pnt[0] = poly_lines[i][0];
				tmp_pnt[1] = poly_lines[i][1];
				initial_corner_pnt2d.emplace_back(tmp_pnt);
			}
			std::string prefix_dir = SavePolygonLineDebugDir;
			SetPlaneOutPutDir(prefix_dir);
			std::string initial_corner_pnt3d_path = prefix_dir + "initial_polygon3d.obj";
			SavePnts2dIntoPnts3DOBJ(initial_corner_pnt3d_path, initial_corner_pnt2d);
		}
		//end test by zdg
		//return;
#endif
		OptimizeNewPolygonExtraction(initial_edge_pnts_pos_, is_edge_points, initial_funs_coeffs_, funs_coeffs_optimized_);

		corner_pnts_.clear();
		corner_pnts_3d_.clear();

		if (initial_pnts_idxs_values.size() == 2) {
			//int this_first = first_lasts[edge_idx_[0]].x, this_last = first_lasts[edge_idx_[0]].y;
			//int next_first = first_lasts[edge_idx_[1]].x, next_last = first_lasts[edge_idx_[1]].y;
			int this_first = initial_pnts_idxs_values[0].associated_pnts_idxs_[0];
			int this_last = initial_pnts_idxs_values[0].associated_pnts_idxs_.back();
			int next_first = initial_pnts_idxs_values[1].associated_pnts_idxs_[0];
			int next_last = initial_pnts_idxs_values[1].associated_pnts_idxs_.back();			
			Eigen::Vector3f edge0_fun = initial_pnts_idxs_values[0].line_fun_;
			Eigen::Vector3f edge1_fun = initial_pnts_idxs_values[1].line_fun_;
			Eigen::Vector2f first_this_pnt = Eigen::Vector2f(edge_pnts_pos_[this_first].x, 
				edge_pnts_pos_[this_first].y);
			Eigen::Vector2f last_this_pnt = Eigen::Vector2f(edge_pnts_pos_[this_last].x,
				edge_pnts_pos_[this_last].y);
			Eigen::Vector2f first_next_pnt = Eigen::Vector2f(edge_pnts_pos_[next_first].x,
				edge_pnts_pos_[next_first].y);
			Eigen::Vector2f last_next_pnt = Eigen::Vector2f(edge_pnts_pos_[next_last].x,
				edge_pnts_pos_[next_last].y);
			Eigen::Vector2f p1_proj, p2_proj, p3_proj, p4_proj;
			ComputePntProjToLineFunction2D(edge0_fun, first_this_pnt, p1_proj);
			ComputePntProjToLineFunction2D(edge0_fun, last_this_pnt, p2_proj);
			ComputePntProjToLineFunction2D(edge1_fun, first_next_pnt, p3_proj);
			ComputePntProjToLineFunction2D(edge1_fun, last_next_pnt, p4_proj);

			/*float2 p1 = ProjectToLine(edge_pnts_pos_[this_first], edges_[0]);
			float2 p2 = ProjectToLine(edge_pnts_pos_[this_last], edges_[0]);
			float2 p3 = ProjectToLine(edge_pnts_pos_[next_first], edges_[1]);
			float2 p4 = ProjectToLine(edge_pnts_pos_[next_last], edges_[1]);*/
			float2 p1 = make_float2(p1_proj[0], p1_proj[1]);
			float2 p2 = make_float2(p2_proj[0], p2_proj[1]);
			float2 p3 = make_float2(p3_proj[0], p3_proj[1]);
			float2 p4 = make_float2(p4_proj[0], p4_proj[1]);

			corner_pnts_.emplace_back(p1);
			corner_pnts_.emplace_back(p2);
			corner_pnts_.emplace_back(p3);
			corner_pnts_.emplace_back(p4);
			for (int i = 0; i < corner_pnts_.size(); i++) {
				if (IsSelfIntersection(i, corner_pnts_[i])) {
					corner_pnts_[1] = p3;
					corner_pnts_[2] = p2;
					break;
				}
			}
		}
		else {

			std::vector<Eigen::Vector2f> poly_lines;
			std::vector<Eigen::Vector3f> tmp_lines_funs;
			for (int j = 0; j < funs_coeffs_optimized_.size(); ++j)
			{
				Eigen::Vector3f tmp_cur_f_coeff = funs_coeffs_optimized_[j].line_fun_;
				tmp_lines_funs.emplace_back(tmp_cur_f_coeff);
			}
			PolygonExtractionFromSortedLinesFunctions(tmp_lines_funs, poly_lines);
			std::cerr << "optimize the polygon lines number: " << poly_lines.size() << std::endl;
			//copy to corner pnts
			for (int i = 0; i < poly_lines.size(); ++i)
			{
				float2 tmp_pnt;
				tmp_pnt.x = poly_lines[i][0];
				tmp_pnt.y = poly_lines[i][1];
				corner_pnts_.emplace_back(tmp_pnt);
			}

#if 0
			for (int i = 0; i < edges_.size(); i++) {
				int next_idx = (i + edges_.size() + 1) % edges_.size();
				//当前边和下一条边之间的夹角
				float theta = GetTheta(edges_[i](0), edges_[i](1), edges_[next_idx](0), edges_[next_idx](1));
				//当前边的最后一点，和下一条边的第一个点
				//int this_last = first_lasts[edge_idx_[i]].y, next_first = first_lasts[edge_idx_[next_idx]].x;
				int this_last = edge_idx[i].back();
				int next_first = edge_idx[next_idx][0];
				float2 p = GetIntersectionPoint(edges_[i], edges_[next_idx]);
				float2 p1 = ProjectToLine(edge_pnts_pos_[this_last], edges_[i]);
				float2 p2 = ProjectToLine(edge_pnts_pos_[next_first], edges_[next_idx]);
				float value = 1;
				//bool self_intersection = false;
				if (corner_pnts_.size() > 0) {
					float2 p_last = corner_pnts_.back();
					float2 edge = p - p_last;
					value = dot(edge, p1 - p_last);
				}
				//如果当前边和下一条边之间的夹角大于theta_max度，且p和p1在当前最后一个顶点的同一侧，则将p作为新的顶点加入到顶点向量中
				if (!isinf(p.x) && !isinf(p.y) && theta > theta_max && value > 0) {//|| PointPointDist(p, edge_pnts_pos_[this_last]) < 0.1 || PointPointDist(p, edge_pnts_pos_[next_first]) < 0.1) {
					corner_pnts_.emplace_back(p);
					std::cout << "p: " << p.x << " " << p.y << "\n";
				}
				//否则，将p1和p2作为两个顶点加入到顶点向量中
				else {
					std::cout << "this_last: " << this_last << "\n";
					std::cout << "next_first: " << next_first << "\n";
					std::cout << "p1: " << p1.x << " " << p1.y << "\n";
					std::cout << "p2: " << p2.x << " " << p2.y << "\n";
					corner_pnts_.emplace_back(p1);
					corner_pnts_.emplace_back(p2);
				}
			}
#endif
		}
		std::cout << "before deleting near points, corner_pnts_.size() = " << corner_pnts_.size() << std::endl;

		////去除距离非常近的相邻顶点
		////取它们之间的中点代替它们
		//double min_distance = params_.min_corner_points_distance;
		//if (min_distance == 0) {
		//	//1cm的间距太小了
		//	//min_distance = std::max((double)ave_alpha_edge_len_, 0.01);
		//	//initial_params_.min_corner_points_distance = std::max((double)ave_alpha_edge_len_, 0.01);
		//	min_distance = std::max((double)ave_alpha_edge_len_, plane_width_ * 0.01);
		//	initial_params_.min_corner_points_distance = std::max((double)ave_alpha_edge_len_, plane_width_ * 0.1);
		//}
		//std::cout << "min_distance = " << min_distance << std::endl;
		//std::vector<float2> temp, temp2;
		//for (int i = 0; i < corner_pnts_.size(); i++) {
		//	if (i == 0)
		//		temp.emplace_back(corner_pnts_[i]);
		//	else {
		//		float2 p = temp.back();
		//		if (PointPointDist(p, corner_pnts_[i]) < min_distance)
		//			temp.back() = (p + corner_pnts_[i]) / 2;
		//		else
		//			temp.emplace_back(corner_pnts_[i]);
		//	}
		//}
		//if (PointPointDist(temp[0], temp.back()) < min_distance) {
		//	temp[0] = (temp[0] + temp.back()) / 2;
		//	temp.pop_back();
		//}
		//std::cout << "after deleting near points, corner_pnts_.size() = " << temp.size() << std::endl;

		fuseNearbyPnts();
		if (corner_pnts_.size() < 3) {
			corner_pnts_.clear();
			corner_pnts_3d_.clear();
			return;
		}

		////std::cout << "temp: " << temp.size() << "\n";
		////去除三点共线的情况
		////如果有三点共线，去掉中间的点
		//for (int i = 0; i < temp.size(); i++) {
		//	int last = (i - 1 + temp.size()) % temp.size();
		//	int next = (i + 1 + temp.size()) % temp.size();
		//	//中间点到两端点连成的直线的距离大于1cm，则判定为不共线，距离太小了
		//	//if (PointLineSegmentDist(temp[i], temp[last], temp[next]) > 0.01)
		//	//plane_width_是包围盒较短的边的长度
		//	if (PointLineSegmentDist(temp[i], temp[last], temp[next]) > plane_width_ * 0.01) 
		//		temp2.emplace_back(temp[i]);
		//	//float2 l1 = temp[i] - temp[last];
		//	//float2 l2 = temp[next] - temp[i];
		//	//float theta = GetTheta(l1.x, l1.y, l2.x, l2.y);
		//	//if (theta > 10) temp2.emplace_back(temp[i]);
		//}

		////如果最终剩下的顶点少于3个，则直接返回
		//if (temp2.size() > 2)
		//	corner_pnts_ = temp2;
		//else {
		//	corner_pnts_.clear();
		//	return;
		//}
		deleteCollinearPnts();
		if (corner_pnts_.size() < 3) {
			corner_pnts_.clear();
			corner_pnts_3d_.clear();
			return;
		}

		//解决多边形的边自相交的问题
		bool self_intersection = true;
		while (self_intersection) {
			self_intersection = false;
			float delete_edge_length = FLT_MAX;
			int delete_edge_idx;
			//先解决（与顶点相邻的两条边）与（多边形的另外两条边）相交的情况
			//直接删除这个顶点
			for (int i = 0; i < corner_pnts_.size(); i++) {
				if (IsSelfIntersection(i, corner_pnts_[i]) == 2) {
					std::vector<float2> temp(corner_pnts_.begin() + i + 1, corner_pnts_.end());
					corner_pnts_.resize(i);
					corner_pnts_.insert(corner_pnts_.begin(), temp.begin(), temp.end());
					self_intersection = true;
					break;
				}
			}
			if (self_intersection)
				continue;
			//再解决相交一次的情况
			//直接删除这个顶点
			for (int i = 0; i < corner_pnts_.size(); i++) {
				if (IsSelfIntersection(i, corner_pnts_[i]) == 1) {
					std::vector<float2> temp(corner_pnts_.begin() + i + 1, corner_pnts_.end());
					corner_pnts_.resize(i);
					corner_pnts_.insert(corner_pnts_.begin(), temp.begin(), temp.end());
					self_intersection = true;
					break;
				}
			}
		}

		std::cout << "after deleting intersecting points, corner_pnts_.size() = " << corner_pnts_.size() << std::endl;

		//float2 edge1 = corner_pnts_[1] - corner_pnts_[0];
		//float2 edge2 = corner_pnts_[2] - corner_pnts_[1];
		//if (edge1.x*edge2.y - edge1.y*edge2.x == 0) {
		//	std::cout << "edge1: " << edge1.x << " " << edge1.y << "\n";
		//	std::cout << "edge2: " << edge2.x << " " << edge2.y << "\n";
		//}

		//解决定点确定的多边形法向量反向的问题
		//将顶点向量中的点逆序排列
		//if (edge1.x*edge2.y - edge1.y*edge2.x > 0 ^ IsConcaveVertex(1) == 1) {
		//	std::vector<float2> corner_pnts_inverse;
		//	for (int i = corner_pnts_.size() - 1; i >= 0; i--) {
		//		corner_pnts_inverse.emplace_back(corner_pnts_[i]);
		//	}
		//	corner_pnts_ = corner_pnts_inverse;
		//}

		ComputeCornerPnts3dFromCornerPnts2d();
		//UpdateInformation();
	}

	void HWPlane::NewPolygonInitialExtractionZDG()
	{
		double theta_max = params_.max_theta;
		if (theta_max == 0) {
			theta_max = 10;
			initial_params_.max_theta = 10;
		}
		std::cout << "theta_max = " << theta_max << std::endl;

		std::vector<bool> is_edge_points(edge_pnts_pos_num_, false);
		std::vector<std::vector<int>> edge_idx;
		for (int i = 0; i < edge_pnts_pos_num_; i++) {
			int last_idx = (i + edge_pnts_pos_num_ - 1) % edge_pnts_pos_num_;
			int next_idx = (i + edge_pnts_pos_num_ + 1) % edge_pnts_pos_num_;
			float2 n = edge_pnts_normal_[i];
			float2 n1 = edge_pnts_normal_[last_idx];
			float2 n2 = edge_pnts_normal_[next_idx];
			if (GetTheta(n.x, n.y, n1.x, n1.y) < theta_max &&
				GetTheta(n.x, n.y, n2.x, n2.y) < theta_max) {
				is_edge_points[i] = true;
			}
		}
		std::cout << "Edge points computed..." << std::endl;
		int first_edge_point_idx = 0;
		//找到第一个edge point
		while (first_edge_point_idx < edge_pnts_pos_num_ && is_edge_points[first_edge_point_idx]) first_edge_point_idx++;
		while (first_edge_point_idx < edge_pnts_pos_num_ && !is_edge_points[first_edge_point_idx]) first_edge_point_idx++;
		if (first_edge_point_idx == edge_pnts_pos_num_) return;
		//is_edge_points
	}

	void HWPlane::NewPolygonExtractionWithoutSmoothZDG()
	{
		double theta_max = params_.max_theta;
		if (theta_max == 0) {
			theta_max = 10;
			initial_params_.max_theta = 10;
		}
		std::cout << "theta_max = " << theta_max << std::endl;
		std::vector<bool> is_edge_points(edge_pnts_pos_num_, false);
		std::vector<std::vector<int>> edge_idx;
		for (int i = 0; i < edge_pnts_pos_num_; i++) {
			int last_idx = (i + edge_pnts_pos_num_ - 1) % edge_pnts_pos_num_;
			int next_idx = (i + edge_pnts_pos_num_ + 1) % edge_pnts_pos_num_;
			float2 n = edge_pnts_normal_[i];
			float2 n1 = edge_pnts_normal_[last_idx];
			float2 n2 = edge_pnts_normal_[next_idx];
			if (GetTheta(n.x, n.y, n1.x, n1.y) < theta_max &&
				GetTheta(n.x, n.y, n2.x, n2.y) < theta_max) {
				is_edge_points[i] = true;
			}
		}
		std::cout << "aaaaaaaaaaa" << std::endl;
#if HW_DEBUG 
		//test
		std::vector<Eigen::Vector3f> all_edges_pnts;
		std::vector<Eigen::Vector3f> all_edges_pnts_normals;
		//is_edge_points
		std::vector<Eigen::Vector3f> all_edges_pnts_color;
		float len_normal = 0.01;
		//assign color to pnt by is_edge_points;
		for (int i = 0; i < edge_pnts_pos_.size(); ++i)
		{
			Eigen::Vector4f tmp_pnt = Eigen::Vector4f(edge_pnts_pos_[i].x, edge_pnts_pos_[i].y, 0.0, 1.0);
			Eigen::Vector2f nml(edge_pnts_normal_[i].x, edge_pnts_normal_[i].y);
			nml.normalize();
			Eigen::Vector4f pnt_end(edge_pnts_pos_[i].x + nml.x(), edge_pnts_pos_[i].y + nml.y(), 0.0, 1.0);
			Eigen::Vector4f transform_pnt = plane_to_world_ * tmp_pnt;
			Eigen::Vector4f transform_end = plane_to_world_ * pnt_end;
			Eigen::Vector3f transform_pnt_3d = Eigen::Vector3f(transform_pnt[0], transform_pnt[1], transform_pnt[2]);
			Eigen::Vector3f transform_end_pnt_3d = Eigen::Vector3f(transform_end[0], transform_end[1], transform_end[2]);
			Eigen::Vector3f transform_normal = transform_end_pnt_3d - transform_pnt_3d;
			transform_normal.normalize();
			all_edges_pnts.emplace_back(transform_pnt_3d);
			all_edges_pnts_normals.emplace_back(transform_normal);
			if (is_edge_points[i])
			{
				Eigen::Vector3f color_pnt = Eigen::Vector3f(1.0, 0.0, 0.0);
				all_edges_pnts_color.emplace_back(color_pnt);
			}
			else
			{
				Eigen::Vector3f color_pnt = Eigen::Vector3f(0.0, 1.0, 0.0);
				all_edges_pnts_color.emplace_back(color_pnt);
			}
		}
		std::string out_pnts_path = filename_ + "_initial_is_edges_pnts_normal_3d.obj";
		WriteOriginEdgesPnts3DByFlag(out_pnts_path, all_edges_pnts,
			all_edges_pnts_normals, len_normal, all_edges_pnts_color, is_edge_points);
		//end test
		return;
#endif
		CopyExractionEdgePntsPosToInitialEdgePnts();	//check the function
		int first_edge_point_idx = 0;
		//找到第一个edge point
		while (first_edge_point_idx < edge_pnts_pos_num_ && is_edge_points[first_edge_point_idx]) first_edge_point_idx++;
		while (first_edge_point_idx < edge_pnts_pos_num_ && !is_edge_points[first_edge_point_idx]) first_edge_point_idx++;
		if (first_edge_point_idx == edge_pnts_pos_num_) return;
		//for (int i = 0; i < is_edge_points.size(); i++) std::cout << i << ": " << is_edge_points[i] << std::endl;

		std::vector<HWLineToPntsIdxs> initial_pnts_idxs_values;
		bool has_thin_edge = false;
		edges_.clear();
		int curr_idx = first_edge_point_idx;
		while (true) {
			HWLineToPntsIdxs cur_fun_edge_idxs;
			std::vector<int> curr_edge_idxs;
			while (is_edge_points[curr_idx]) {
				curr_edge_idxs.push_back(curr_idx);
				int old_idx = curr_idx;
				curr_idx = (curr_idx + 1) % edge_pnts_pos_num_;

				if (curr_edge_idxs.size() < 2)
				{
					continue;
				}
				//Eigen::Vector2f coeff = FittingLine(curr_edge_idxs);
				Eigen::Vector3f coeff_fun = FittingLineFromPntsIdxs(curr_edge_idxs);
				if (abs(coeff_fun(0)) < KMIN_FLOAT_THRESHOLD_REFINED
					&& abs(coeff_fun(1)) < KMIN_FLOAT_THRESHOLD_REFINED)
				{
					continue;
				}
				float max = -FLT_MAX, min = FLT_MAX;
				for (int k = 0; k < curr_edge_idxs.size(); k++) {
					int idx = curr_edge_idxs[k];
					//float d = PointLineDist(edge_pnts_pos_[idx], coeff);
					Eigen::Vector2f tmp_edge_pnt = Eigen::Vector2f(edge_pnts_pos_[idx].x,
						edge_pnts_pos_[idx].y);
					float d = ComputePntToLineFunction2D(coeff_fun, tmp_edge_pnt);
					if (d > max)
						max = d;
					if (d < min)
						min = d;
				}
				double region_diameter = max - min;
				if (region_diameter > diameter_max_) {
					//edge_pnts_smooth_regions_[i].pop_back();
					curr_edge_idxs.pop_back();
					curr_idx = old_idx;
					break;
				}
			}
			//如果这条线段的点数大于5，则拟合这条线段
			if (curr_edge_idxs.size() > edges_pnts_fitting_num_threshold_) {

				//Eigen::Vector2f coeff = FittingLine(curr_edge_idxs);
				Eigen::Vector3f coeff_fun = FittingLineFromPntsIdxs(curr_edge_idxs);

				/*
				float2 start, end, mid;
				start = edge_pnts_pos_[curr_edge_idxs[0]];
				mid = edge_pnts_pos_[curr_edge_idxs[curr_edge_idxs.size() / 2]];
				end = edge_pnts_pos_[curr_edge_idxs.back()];
				float2 start_proj, mid_proj, end_proj;
				start_proj = ProjectToLine(start, coeff);
				mid_proj = ProjectToLine(mid, coeff);
				end_proj = ProjectToLine(end, coeff);
				float dist = PointPointDist(start_proj, end_proj);
				float dist1 = PointPointDist(start_proj, mid);
				float dist2 = PointPointDist(mid, end_proj);
				//这里没看懂
				//why?
				if (dist < max(dist1, dist2)) {
				has_thin_edge = true;
				}
				*/

				Eigen::Vector2f start, end, mid;
				start[0] = edge_pnts_pos_[curr_edge_idxs[0]].x;
				start[1] = edge_pnts_pos_[curr_edge_idxs[0]].y;
				mid[0] = edge_pnts_pos_[curr_edge_idxs[curr_edge_idxs.size() / 2]].x;
				mid[1] = edge_pnts_pos_[curr_edge_idxs[curr_edge_idxs.size() / 2]].y;
				end[0] = edge_pnts_pos_[curr_edge_idxs.back()].x;
				end[1] = edge_pnts_pos_[curr_edge_idxs.back()].y;
				Eigen::Vector2f start_proj, mid_proj, end_proj;
				ComputePntProjToLineFunction2D(coeff_fun, start, start_proj);
				ComputePntProjToLineFunction2D(coeff_fun, mid, mid_proj);
				ComputePntProjToLineFunction2D(coeff_fun, end, end_proj);
				float dist = (start_proj - end_proj).norm();
				float dist1 = (start_proj - mid).norm();
				float dist2 = (mid - end_proj).norm();
				if (dist < max(dist1, dist2))
				{
					has_thin_edge = true;
				}
				cur_fun_edge_idxs.line_fun_ = coeff_fun;
				cur_fun_edge_idxs.associated_pnts_idxs_ = curr_edge_idxs;
				//edges_.emplace_back(coeff_fun);
				//edge_idx.push_back(curr_edge_idxs);

				initial_pnts_idxs_values.emplace_back(cur_fun_edge_idxs);
			}
			while (!is_edge_points[curr_idx]) {
				curr_idx = (curr_idx + 1) % edge_pnts_pos_num_;
			}

			//如果又回到起点，说明全部遍历了一遍
			if (curr_idx == first_edge_point_idx) break;
			//else {
			//std::cout << "first_edge_point_idx: " << first_edge_point_idx << ", curr_idx: " << curr_idx << std::endl;
			//}
		}
		std::cout << "b" << std::endl;

		int edge_num = initial_pnts_idxs_values.size();

		if (initial_pnts_idxs_values.size() <= 1)
			return;
		if (initial_pnts_idxs_values.size() > 1 && has_thin_edge) {
			printf("has_thin_edge!!!!!!!!!!!!!!!!!!!!\n");
		}
		initial_funs_coeffs_ = initial_pnts_idxs_values;

	}

	void HWPlane::CopyExractionEdgePntsPosToInitialEdgePnts()
	{
		initial_edge_pnts_pos_.clear();
		for (int i = 0; i < edge_pnts_pos_.size(); ++i)
		{
			Eigen::Vector2f tmp_pnt 
				= Eigen::Vector2f(edge_pnts_pos_[i].x, edge_pnts_pos_[i].y);
			HWOptiPnt2D tmp_pnt2d;
			tmp_pnt2d.pos_ = tmp_pnt;
			initial_edge_pnts_pos_.emplace_back(tmp_pnt2d);
		}
		if (edge_pnts_pos_.size() != edge_pnts_normal_.size())
			return;
		for (int i = 0; i < edge_pnts_normal_.size(); ++i)
		{
			initial_edge_pnts_pos_[i].normal_
				= Eigen::Vector2f(edge_pnts_normal_[i].x, edge_pnts_normal_[i].y);
		}
	}

	void HWPlane::OptimizeNewPolygonExtraction(const std::vector<HWOptiPnt2D>& pnts, std::vector<bool> is_edges_pnts, 
		const std::vector<HWLineToPntsIdxs>& fun_coeffs, std::vector<HWLineToPntsIdxs>& fun_coeffs_opti)
	{
		//std::cerr << "to do next..." << std::endl;
		std::vector<Eigen::Vector2f> initial_polygon_pnts;
		initial_funs_coeffs_ = fun_coeffs;
		float initial_polygon_area;
		int initial_polygon_lines_num;
		int pnts_assigned_lines_num;

#if ZDG_DEBUG
		std::string prefix_dir = SavePolygonLineDebugDir;
		SetPlaneOutPutDir(prefix_dir);
#endif
		std::vector<PolygonNeighborEnergy> cur_neighbors_energys_nodes;
		//construct neighbors from initial nodes
		for (int i = 0; i < fun_coeffs.size(); ++i)
		{
			int ii = (i + 1) % fun_coeffs.size();
			PolygonNeighborEnergy tmp_node;
			tmp_node.lidx = i;
			tmp_node.ridx = ii;
			float eu_lv = EuPolygonPntsToPolygonLineDistance(pnts, fun_coeffs[tmp_node.lidx]);
			float en_lv = EnPolygonPntsNormalToPolygonLineNormal(pnts, fun_coeffs[tmp_node.lidx]);
			float eu_rv = EuPolygonPntsToPolygonLineDistance(pnts, fun_coeffs[tmp_node.ridx]);
			float en_rv = EnPolygonPntsNormalToPolygonLineNormal(pnts, fun_coeffs[tmp_node.ridx]);
			float eun_v = polygon_extract_energy_weights_.wu_lambda_*(eu_lv + eu_rv)
				+ polygon_extract_energy_weights_.wn_lambda_*(en_lv + en_rv);
			//compute the neighbor lines remained pnts
			std::vector<int> remained_neighbor_pnts_idxs;
			ComputeNeighborLinesFunsRemainedPntsIdxs(pnts, fun_coeffs, tmp_node.lidx, tmp_node.ridx, remained_neighbor_pnts_idxs);
			
			//test
			//plane_obj_name_
			std::string tmp_pnts_path_str = plane_output_dir_ + "plane_pnts_" + std::to_string(i) + "_" + std::to_string(ii) + ".obj";
			std::string tmp_edges_path_str = plane_output_dir_ + "plane_edges_" + std::to_string(i) + "_" + std::to_string(ii) + ".obj";
			std::vector<Eigen::Vector2f> tmp_plane_pnts;
			std::vector<Eigen::Vector2f> tmp_plane_edges;
			for (int j = 0; j < remained_neighbor_pnts_idxs.size(); ++j)
			{
				int test_idx = remained_neighbor_pnts_idxs[j];
				Eigen::Vector2f test_pnt = pnts[test_idx].pos_;
				tmp_plane_pnts.emplace_back(test_pnt);
			}
			for (int j = 0; j < fun_coeffs[tmp_node.lidx].associated_pnts_idxs_.size() - 1; ++j)
			{
				int test_idx0 = fun_coeffs[tmp_node.lidx].associated_pnts_idxs_[j];
				Eigen::Vector2f test_pnt0 = pnts[test_idx0].pos_;
				int test_idx1 = fun_coeffs[tmp_node.lidx].associated_pnts_idxs_[j + 1];
				Eigen::Vector2f test_pnt1 = pnts[test_idx1].pos_;
				tmp_plane_edges.emplace_back(test_pnt0);
				tmp_plane_edges.emplace_back(test_pnt1);
			}
			for (int j = 0; j < fun_coeffs[tmp_node.ridx].associated_pnts_idxs_.size() - 1; ++j)
			{
				int test_idx0 = fun_coeffs[tmp_node.ridx].associated_pnts_idxs_[j];
				Eigen::Vector2f test_pnt0 = pnts[test_idx0].pos_;
				int test_idx1 = fun_coeffs[tmp_node.ridx].associated_pnts_idxs_[j + 1];
				Eigen::Vector2f test_pnt1 = pnts[test_idx1].pos_;
				tmp_plane_edges.emplace_back(test_pnt0);
				tmp_plane_edges.emplace_back(test_pnt1);
			}
			SavePnts2dIntoPnts3DOBJ(tmp_pnts_path_str, tmp_plane_pnts);
			SaveLinesPnts2dIntoLinesPnts3DOBJ(tmp_edges_path_str, tmp_plane_edges);
			//end test

			float ec_neighbor_v = polygon_extract_energy_weights_.wc_lambda_ * (remained_neighbor_pnts_idxs.size() / pnts.size());
			//float ec_neighbor_v = polygon_extract_energy_weights_.wc_lambda_ * (remained_neighbor_pnts_idxs.size());
			tmp_node.energy_value = eun_v + ec_neighbor_v;
			cur_neighbors_energys_nodes.emplace_back(tmp_node);
		}
		std::sort(cur_neighbors_energys_nodes.begin(), cur_neighbors_energys_nodes.end(), ComparePolygonNeighborEnergyValueDecrease);

		//test
		std::cerr << "-----------------------0---------------------------- " << std::endl;
		for (int i = 0; i < cur_neighbors_energys_nodes.size(); ++i)
		{
			std::cerr << i << "->" << "(" << cur_neighbors_energys_nodes[i].lidx <<
				" " << cur_neighbors_energys_nodes[i].ridx << ")" << std::endl;
		}
		std::cerr << "-----------------------0----------------------------- " << std::endl << std::endl;
		//end test

		//return;

		std::vector<int> initial_remained_pnts_idxs;
		//get remained points idxs
		std::vector<bool> all_pnts_idxs_flag;
		all_pnts_idxs_flag.resize(pnts.size(), false);
		for (int i = 0; i < fun_coeffs.size(); ++i)
		{
			for (int j = 0; j < fun_coeffs[i].associated_pnts_idxs_.size(); ++j)
			{
				int idx = fun_coeffs[i].associated_pnts_idxs_[j];
				all_pnts_idxs_flag[idx] = true;
			}
		}
		for (int i = 0; i < all_pnts_idxs_flag.size(); ++i)
		{
			if (!all_pnts_idxs_flag[i])
			{
				initial_remained_pnts_idxs.emplace_back(i);
			}
		}

		//get initial_funs_coeffs_
		int cur_idx = 0;
		bool stop_process = false;
		int cur_iterators_number = 0;
		std::cerr << "initial energy value start..." << std::endl;
		////test
		//std::vector<Eigen::Vector2f> initial_poly_lines;
		//std::vector<Eigen::Vector3f> initial_tmp_lines_funs;
		//for (int j = 0; j < fun_coeffs.size(); ++j)
		//{
		//	Eigen::Vector3f tmp_initial_line = fun_coeffs[j].line_fun_;
		//	initial_tmp_lines_funs.emplace_back(tmp_initial_line);
		//}
		//PolygonExtractionFromSortedLinesFunctions(initial_tmp_lines_funs, initial_poly_lines);
		////save the poly_lines
		//std::string initial_test_path = plane_output_dir_ +  "initial_poly_line.obj";
		//std::cerr << "initial_test_path: " << initial_test_path << std::endl;
		//SavePolygonLines2dIntoPolygonLines3DOBJ(initial_test_path, initial_poly_lines);
		////end test
		std::cerr << "the pnts number: " << pnts.size() << std::endl;
		std::cerr << "the initial_remained_pnts_idxs number: " << initial_remained_pnts_idxs.size() << std::endl;
		float pre_all_energy_value = ComputeAllEnergyValueFromPolygonExtractedPnts(pnts, fun_coeffs, initial_remained_pnts_idxs);
		float cur_all_energy_value = pre_all_energy_value;
		std::cerr << "initial energy value end..." << std::endl;
		std::vector<HWLineToPntsIdxs> pre_fun_coeffs = fun_coeffs;
		std::vector<int> pre_remained_pnts_idxs = initial_remained_pnts_idxs;
		std::vector<HWLineToPntsIdxs> cur_fun_coeffs = fun_coeffs;
		std::vector<int> cur_remained_pnts_idxs = initial_remained_pnts_idxs;
		std::vector<HWLineToPntsIdxs> final_fun_coeffs = fun_coeffs;
		std::vector<int> final_remained_fun_coeffs = initial_remained_pnts_idxs;

		//polygon_extract_opti_iterators_num_ = 5;
		ComputeSortedAllNodeNeighborEnergysFromHWLineToPntsIdxs(pnts, cur_fun_coeffs, cur_neighbors_energys_nodes);
		
		//test
		std::cerr << "------------------+1------------------------- " << std::endl;
		for (int i = 0; i < cur_neighbors_energys_nodes.size(); ++i)
		{
			std::cerr << i << "->" << "(" << cur_neighbors_energys_nodes[i].lidx <<
				" " << cur_neighbors_energys_nodes[i].ridx << ")" << std::endl;
		}
		std::cerr << "-------------------1------------------------- " << std::endl;
		//end test

		//insert energy function....
		while (cur_idx < cur_neighbors_energys_nodes.size()
			&& !stop_process)
		{
			pre_fun_coeffs = cur_fun_coeffs;
			pre_remained_pnts_idxs = cur_remained_pnts_idxs;
			int l_idx = cur_neighbors_energys_nodes[cur_idx].lidx;
			int r_idx = cur_neighbors_energys_nodes[cur_idx].ridx;

			/*std::cerr << "cur_neighbors_energys_nodes " << "[" << cur_idx << "].lidx: " <<
				l_idx << std::endl;
			std::cerr << "cur_neighbors_energys_nodes " << "[" << cur_idx << "].ridx: " <<
				r_idx << std::endl;*/

			//operation
			float energy_insert_energy_v = PolygonLineInsertLineIntoTwoPolygonLine(pnts, 
				pre_fun_coeffs, pre_remained_pnts_idxs, l_idx, r_idx, cur_fun_coeffs, cur_remained_pnts_idxs);

			//test
			std::vector<Eigen::Vector2f> poly_lines;
			std::vector<Eigen::Vector3f> tmp_lines_funs;
			for (int j = 0; j < cur_fun_coeffs.size(); ++j)
			{
				Eigen::Vector3f tmp_cur_f_coeff = cur_fun_coeffs[j].line_fun_;
				tmp_lines_funs.emplace_back(tmp_cur_f_coeff);
			}
			PolygonExtractionFromSortedLinesFunctions(tmp_lines_funs, poly_lines);
			//save the poly_lines
			std::string test_path = plane_output_dir_ + std::to_string(cur_iterators_number) + "_insert_poly_line_temp.obj";
			SavePolygonLines2dIntoPolygonLines3DOBJ(test_path, poly_lines);
			//end test

			//update cur_neighbors_energys_nodes
			if (energy_insert_energy_v < cur_all_energy_value)
			{
				cur_all_energy_value = energy_insert_energy_v;
				ComputeSortedAllNodeNeighborEnergysFromHWLineToPntsIdxs(pnts, cur_fun_coeffs, cur_neighbors_energys_nodes);
				final_fun_coeffs = cur_fun_coeffs;
				final_remained_fun_coeffs = cur_remained_pnts_idxs;
			}
			else
			{
				stop_process = true;
			}

			std::cerr << "cur_iterators_number: " << cur_iterators_number << std::endl;
			if (cur_iterators_number > polygon_extract_opti_iterators_num_)
			{
				stop_process = true;
			}
			++cur_iterators_number;
			//++cur_idx;
		}

		////test 
		//std::string initial_pnts_normal_test_path = plane_output_dir_ + "initial_pnts_normals.obj";
		//SaveHWOptiPnts2dIntoPolygonLineObj(initial_pnts_normal_test_path, pnts);
		////end test

		//split the lines, important
		cur_idx = 0;
		stop_process = false;
		cur_iterators_number = 0;
		cur_fun_coeffs = final_fun_coeffs;
		cur_remained_pnts_idxs = final_remained_fun_coeffs;
		polygon_extract_energy_weights_.wu_lambda_ *= 100;
		polygon_extract_energy_weights_.wn_lambda_ *= 5;
		polygon_extract_energy_weights_.ws_lambda_ *= 0.2;
		cur_all_energy_value = ComputeAllEnergyValueFromPolygonExtractedPnts(pnts, cur_fun_coeffs, cur_remained_pnts_idxs);
		std::cerr << "start to split polygon line into two lines..." << std::endl;
		//
		ComputeSortedAllNodeNeighborEnergysFromHWLineToPntsIdxs(pnts, cur_fun_coeffs, cur_neighbors_energys_nodes);
		
		//test
		std::cerr << "------------------+2------------------------- " << std::endl;
		for (int i = 0; i < cur_neighbors_energys_nodes.size(); ++i)
		{
			std::cerr << i << "->" << "(" << cur_neighbors_energys_nodes[i].lidx <<
				" " << cur_neighbors_energys_nodes[i].ridx << ")" << std::endl;
		}
		std::cerr << "-------------------2------------------------- " << std::endl << std::endl;
		//end test

#if 0
		//test
		//construct neighbors from  nodes
		for (int i = 0; i < cur_fun_coeffs.size(); ++i)
		{
			int ii = (i + 1) % cur_fun_coeffs.size();
			PolygonNeighborEnergy tmp_node;
			tmp_node.lidx = i;
			tmp_node.ridx = ii;
			std::cerr << "cur tmp_node.lidx, tmp_node.ridx: " << tmp_node.lidx << ", " << tmp_node.ridx << std::endl;
			ComputeNeighborPairsEuncEnergyValue(pnts, cur_fun_coeffs, tmp_node);

			std::vector<int> remained_neighbor_pnts_idxs;
			ComputeNeighborLinesFunsRemainedPntsIdxs(pnts, cur_fun_coeffs, tmp_node.lidx, tmp_node.ridx, remained_neighbor_pnts_idxs);

			//test
			//plane_obj_name_
			std::string tmp_pnts_path_str = plane_output_dir_ + std::to_string(cur_idx) + "_intermediate_plane_pnts_" + std::to_string(i) + "_" + std::to_string(ii) + ".obj";
			std::string tmp_edges_path_str = plane_output_dir_ + std::to_string(cur_idx) + "_intermediate_plane_edges_" + std::to_string(i) + "_" + std::to_string(ii) + ".obj";
			std::vector<Eigen::Vector2f> tmp_plane_pnts;
			std::vector<Eigen::Vector2f> tmp_plane_edges;
			for (int j = 0; j < remained_neighbor_pnts_idxs.size(); ++j)
			{
				int test_idx = remained_neighbor_pnts_idxs[j];
				Eigen::Vector2f test_pnt = pnts[test_idx].pos_;
				tmp_plane_pnts.emplace_back(test_pnt);
			}
			std::cerr << "cur_fun_coeffs[tmp_node.lidx].associated_pnts_idxs_ num: " <<
				cur_fun_coeffs[tmp_node.lidx].associated_pnts_idxs_.size() << std::endl;
			std::cerr << "cur_fun_coeffs[tmp_node.ridx].associated_pnts_idxs_ num: " <<
				cur_fun_coeffs[tmp_node.ridx].associated_pnts_idxs_.size() << std::endl;
			for (int j = 0; j < cur_fun_coeffs[tmp_node.lidx].associated_pnts_idxs_.size() - 1; ++j)
			{
				int test_idx0 = cur_fun_coeffs[tmp_node.lidx].associated_pnts_idxs_[j];
				Eigen::Vector2f test_pnt0 = pnts[test_idx0].pos_;
				int test_idx1 = cur_fun_coeffs[tmp_node.lidx].associated_pnts_idxs_[j + 1];
				Eigen::Vector2f test_pnt1 = pnts[test_idx1].pos_;
				tmp_plane_edges.emplace_back(test_pnt0);
				tmp_plane_edges.emplace_back(test_pnt1);
			}
			for (int j = 0; j < cur_fun_coeffs[tmp_node.ridx].associated_pnts_idxs_.size() - 1; ++j)
			{
				int test_idx0 = cur_fun_coeffs[tmp_node.ridx].associated_pnts_idxs_[j];
				Eigen::Vector2f test_pnt0 = pnts[test_idx0].pos_;
				int test_idx1 = cur_fun_coeffs[tmp_node.ridx].associated_pnts_idxs_[j + 1];
				Eigen::Vector2f test_pnt1 = pnts[test_idx1].pos_;
				tmp_plane_edges.emplace_back(test_pnt0);
				tmp_plane_edges.emplace_back(test_pnt1);
			}
			SavePnts2dIntoPnts3DOBJ(tmp_pnts_path_str, tmp_plane_pnts);
			SaveLinesPnts2dIntoLinesPnts3DOBJ(tmp_edges_path_str, tmp_plane_edges);
			//end test
		}
		//end test
#endif

		while (cur_idx < cur_neighbors_energys_nodes.size()
			&& !stop_process)
		{
			pre_fun_coeffs = cur_fun_coeffs;
			pre_remained_pnts_idxs = cur_remained_pnts_idxs;
			int l_idx = cur_neighbors_energys_nodes[cur_idx].lidx;
			int r_idx = cur_neighbors_energys_nodes[cur_idx].ridx;

			/*std::cerr << "split cur_neighbors_energys_nodes " << "[" << cur_idx << "].lidx: " <<
				l_idx << std::endl;
			std::cerr << "split cur_neighbors_energys_nodes " << "[" << cur_idx << "].ridx: " <<
				r_idx << std::endl;*/

			std::vector<HWLineToPntsIdxs> cur_left_fun_coeffs, cur_right_fun_coeffs;
			std::vector<int> cur_remained_left_fun_coeffs, cur_remained_right_fun_coeffs;
			float energy_min_value = KMAX_FLOAT_LIMIT_VALUE;
			//operation
			float energy_split_left_energy_v = PolygonLineSplitIntoTwoPolygonLines(pnts,
				pre_fun_coeffs, pre_remained_pnts_idxs, l_idx, cur_left_fun_coeffs, cur_remained_left_fun_coeffs);
			float energy_split_right_energy_v = PolygonLineSplitIntoTwoPolygonLines(pnts,
				pre_fun_coeffs, pre_remained_pnts_idxs, r_idx, cur_right_fun_coeffs, cur_remained_right_fun_coeffs);
			//other operation...
			float energy_split_energy_v;
			if (energy_split_left_energy_v < energy_split_right_energy_v)
			{
				std::cerr << "left........." << l_idx << std::endl;
				cur_fun_coeffs = cur_left_fun_coeffs;
				energy_split_energy_v = energy_split_left_energy_v;
				cur_remained_pnts_idxs = cur_remained_left_fun_coeffs;
			}
			else
			{
				std::cerr << "right........." << r_idx << std::endl;
				cur_fun_coeffs = cur_right_fun_coeffs;
				energy_split_energy_v = energy_split_right_energy_v;
				cur_remained_pnts_idxs = cur_remained_right_fun_coeffs;
			}

			//test
			std::vector<Eigen::Vector2f> poly_lines;
			std::vector<Eigen::Vector3f> tmp_lines_funs;
			for (int j = 0; j < cur_fun_coeffs.size(); ++j)
			{
				Eigen::Vector3f tmp_cur_f_coeff = cur_fun_coeffs[j].line_fun_;
				tmp_lines_funs.emplace_back(tmp_cur_f_coeff);
			}
			PolygonExtractionFromSortedLinesFunctions(tmp_lines_funs, poly_lines);
			//save the poly_lines
			std::string test_path = plane_output_dir_ + std::to_string(cur_iterators_number) + "_split_poly_line_temp.obj";
			std::cerr << "test_path: " << test_path << std::endl;
			SavePolygonLines2dIntoPolygonLines3DOBJ(test_path, poly_lines);
			//end test

			//update cur_neighbors_energys_nodes
			if (energy_split_energy_v < cur_all_energy_value)
			{
				cur_all_energy_value = energy_split_energy_v;
				ComputeSortedAllNodeNeighborEnergysFromHWLineToPntsIdxs(pnts, cur_fun_coeffs, cur_neighbors_energys_nodes);
				final_fun_coeffs = cur_fun_coeffs;
				final_remained_fun_coeffs = cur_remained_pnts_idxs;
			}
			else
			{
				stop_process = true;
			}

			//test
			for (int i = 0; i < cur_neighbors_energys_nodes.size(); ++i)
			{
				std::cerr << i << "->" << "(" << cur_neighbors_energys_nodes[i].lidx <<
					" " << cur_neighbors_energys_nodes[i].ridx << ")" << std::endl;
			}
			//end test

			std::cerr << "split cur_iterators_number: " << cur_iterators_number << std::endl;
			if (cur_iterators_number > polygon_extract_opti_iterators_num_)
			{
				stop_process = true;
			}
			++cur_iterators_number;
			//++cur_idx;
		}
		std::cerr << "end split polygon line into two lines..." << std::endl << std::endl;

		//test
		std::cerr << "------------------+3------------------------- " << std::endl;
		for (int i = 0; i < cur_neighbors_energys_nodes.size(); ++i)
		{  
			std::cerr << i << "->" << "(" << cur_neighbors_energys_nodes[i].lidx <<
				" " << cur_neighbors_energys_nodes[i].ridx << ")" << std::endl;
		}
		std::cerr << "-------------------3------------------------- " << std::endl << std::endl;
		//end test

#if 0
		cur_idx = 0;
		while (cur_idx < cur_neighbors_energys_nodes.size()
			&& !stop_process)
		{
			pre_fun_coeffs = cur_fun_coeffs;
			pre_remained_pnts_idxs = cur_remained_pnts_idxs;
			int l_idx = cur_neighbors_energys_nodes[cur_idx].lidx;
			int r_idx = cur_neighbors_energys_nodes[cur_idx].ridx;

			std::cerr << "cur_neighbors_energys_nodes " << "[" << cur_idx << "].lidx: " <<
				l_idx << std::endl;
			std::cerr << "cur_neighbors_energys_nodes " << "[" << cur_idx << "].ridx: " <<
				r_idx << std::endl;

			float energy_min_value = KMAX_FLOAT_LIMIT_VALUE;
			//operation
			float energy_exchange_energy_v = PolygonLinesExchangePolygonLinePnts(pnts,
				pre_fun_coeffs, pre_remained_pnts_idxs, l_idx, r_idx, cur_fun_coeffs, cur_remained_pnts_idxs);
			//std::cerr << "to do next..." << std::endl;
			//other operation...

			//test
			std::vector<Eigen::Vector2f> poly_lines;
			std::vector<Eigen::Vector3f> tmp_lines_funs;
			for (int j = 0; j < cur_fun_coeffs.size(); ++j)
			{
				Eigen::Vector3f tmp_cur_f_coeff = cur_fun_coeffs[j].line_fun_;
				tmp_lines_funs.emplace_back(tmp_cur_f_coeff);
			}
			PolygonExtractionFromSortedLinesFunctions(tmp_lines_funs, poly_lines);
			//save the poly_lines
			std::string test_path = plane_output_dir_ + std::to_string(cur_iterators_number) + "_poly_line_temp.obj";
			SavePolygonLines2dIntoPolygonLines3DOBJ(test_path, poly_lines);
			//end test

			//update cur_neighbors_energys_nodes
			if (energy_exchange_energy_v < cur_all_energy_value)
			{
				cur_all_energy_value = energy_exchange_energy_v;
				ComputeSortedAllNodeNeighborEnergysFromHWLineToPntsIdxs(pnts, cur_fun_coeffs, cur_neighbors_energys_nodes);
			}
			else
			{
				stop_process = true;
			}

			std::cerr << "cur_iterators_number: " << cur_iterators_number << std::endl;
			if (cur_iterators_number > polygon_extract_opti_iterators_num_)
			{
				stop_process = true;
			}
			++cur_iterators_number;
			//++cur_idx;
		}
#endif
		
		//merge the polygon lines
		cur_idx = 0;
		stop_process = false;
		cur_iterators_number = 0;
		cur_fun_coeffs = final_fun_coeffs;
		cur_remained_pnts_idxs = final_remained_fun_coeffs;
		polygon_extract_energy_weights_.wu_lambda_ *= 1;
		polygon_extract_energy_weights_.wn_lambda_ *= 1;
		polygon_extract_energy_weights_.ws_lambda_ *= 1;
		cur_all_energy_value = ComputeAllEnergyValueFromPolygonExtractedPnts(pnts, cur_fun_coeffs, cur_remained_pnts_idxs);
		std::cerr << "start to merge two neighbor polygons line into signle line..." << std::endl;
		ComputeAllNodeNeighborLinesAngleEnergysFromHWLineToPntsIdxsIncrease(pnts, cur_fun_coeffs, cur_neighbors_energys_nodes);
		while (cur_idx < cur_neighbors_energys_nodes.size()
			&& !stop_process)
		{
			pre_fun_coeffs = cur_fun_coeffs;
			pre_remained_pnts_idxs = cur_remained_pnts_idxs;
			int l_idx = cur_neighbors_energys_nodes[cur_idx].lidx;
			int r_idx = cur_neighbors_energys_nodes[cur_idx].ridx;
			std::cerr << "merge cur_neighbors_energys_nodes " << "[" << cur_idx << "].lidx: " <<
				l_idx << std::endl;
			std::cerr << "merge cur_neighbors_energys_nodes " << "[" << cur_idx << "].ridx: " <<
				r_idx << std::endl;
			
			////check
			//if (cur_neighbors_energys_nodes[cur_idx].energy_value > polygon_neighbor_lines_angle_threhold_)
			//{
			//	std::cerr << "none polygon lines merge..." << std::endl;
			//	break;
			//}
			float energy_merge_energy_v = PolygonLineMergeTwoPolygonIntoPolygonLine(pnts,
				pre_fun_coeffs, pre_remained_pnts_idxs, l_idx, r_idx, cur_fun_coeffs, cur_remained_pnts_idxs);
			
			////test
			//std::vector<Eigen::Vector2f> poly_lines;
			//std::vector<Eigen::Vector3f> tmp_lines_funs;
			//for (int j = 0; j < cur_fun_coeffs.size(); ++j)
			//{
			//	Eigen::Vector3f tmp_cur_f_coeff = cur_fun_coeffs[j].line_fun_;
			//	tmp_lines_funs.emplace_back(tmp_cur_f_coeff);
			//}
			//PolygonExtractionFromSortedLinesFunctions(tmp_lines_funs, poly_lines);
			////save the poly_lines
			//std::string test_path = plane_output_dir_ + std::to_string(cur_iterators_number) + "_merge_poly_line_temp.obj";
			//SavePolygonLines2dIntoPolygonLines3DOBJ(test_path, poly_lines);
			////end test

			//update cur_neighbors_energys_nodes
			if (cur_neighbors_energys_nodes[cur_idx].energy_value < polygon_neighbor_lines_angle_threhold_ &&
				energy_merge_energy_v < cur_all_energy_value)
			{
				cur_all_energy_value = energy_merge_energy_v;
				ComputeAllNodeNeighborLinesAngleEnergysFromHWLineToPntsIdxsIncrease(pnts, cur_fun_coeffs, cur_neighbors_energys_nodes);
				final_fun_coeffs = cur_fun_coeffs;
				final_remained_fun_coeffs = cur_remained_pnts_idxs;
			}
			else
			{
				stop_process = true;
			}

			std::cerr << "merge cur_iterators_number: " << cur_iterators_number;
			if (cur_iterators_number > polygon_extract_opti_iterators_num_)
			{
				std::cerr << "->failed..." << std::endl;
				stop_process = true;
			}
			++cur_iterators_number;

		}
		std::cerr << "end merge two neighbor polygons line into signle line..." << std::endl << std::endl;
		//end merge operation

		//exchange the polygon lines
		std::cerr << "start to exchange two neighbor polygons lines pnts ..." << std::endl;
		cur_idx = 0;
		stop_process = false;
		cur_iterators_number = 0;
		cur_fun_coeffs = final_fun_coeffs;
		cur_remained_pnts_idxs = final_remained_fun_coeffs;
		polygon_extract_energy_weights_.wu_lambda_ *= 1;
		polygon_extract_energy_weights_.wn_lambda_ *= 1;
		polygon_extract_energy_weights_.ws_lambda_ *= 1;
		cur_all_energy_value = ComputeAllEnergyValueFromPolygonExtractedPnts(pnts, cur_fun_coeffs, cur_remained_pnts_idxs);
		ComputeSortedAllNodeNeighborEnergysFromHWLineToPntsIdxs(pnts, cur_fun_coeffs, cur_neighbors_energys_nodes);
		while (cur_idx < cur_neighbors_energys_nodes.size()
			&& !stop_process)
		{
			pre_fun_coeffs = cur_fun_coeffs;
			pre_remained_pnts_idxs = cur_remained_pnts_idxs;
			int l_idx = cur_neighbors_energys_nodes[cur_idx].lidx;
			int r_idx = cur_neighbors_energys_nodes[cur_idx].ridx;
			/*std::cerr << "exchange cur_neighbors_energys_nodes " << "[" << cur_idx << "].lidx: " <<
				l_idx << std::endl;
			std::cerr << "exchange cur_neighbors_energys_nodes " << "[" << cur_idx << "].ridx: " <<
				r_idx << std::endl;*/
			float energy_exchange_energy_v = PolygonLinesExchangePolygonLinePnts(pnts,
				pre_fun_coeffs, pre_remained_pnts_idxs, l_idx, r_idx, cur_fun_coeffs, cur_remained_pnts_idxs);
			
			////test
			//std::vector<Eigen::Vector2f> poly_lines;
			//std::vector<Eigen::Vector3f> tmp_lines_funs;
			//for (int j = 0; j < cur_fun_coeffs.size(); ++j)
			//{
			//	Eigen::Vector3f tmp_cur_f_coeff = cur_fun_coeffs[j].line_fun_;
			//	tmp_lines_funs.emplace_back(tmp_cur_f_coeff);
			//}
			//PolygonExtractionFromSortedLinesFunctions(tmp_lines_funs, poly_lines);
			////save the poly_lines
			//std::string test_path = plane_output_dir_ + std::to_string(cur_iterators_number) + "_exchange_poly_line_temp.obj";
			//SavePolygonLines2dIntoPolygonLines3DOBJ(test_path, poly_lines);
			////end test

			//update exchange cur_neighbors_energys_nodes
			if (energy_exchange_energy_v < cur_all_energy_value)
			{
				cur_all_energy_value = energy_exchange_energy_v;
				ComputeSortedAllNodeNeighborEnergysFromHWLineToPntsIdxs(pnts, cur_fun_coeffs, cur_neighbors_energys_nodes);
				final_fun_coeffs = cur_fun_coeffs;
				final_remained_fun_coeffs = cur_remained_pnts_idxs;
			}
			else
			{
				stop_process = true;
			}

			std::cerr << "exchange operation cur_iterators_number: " << cur_iterators_number << std::endl;
			if (cur_iterators_number > polygon_extract_opti_iterators_num_)
			{
				stop_process = true;
			}
			++cur_iterators_number;

		}
		std::cerr << "end exchange two neighbor polygons lines pnts ..." << std::endl << std::endl;
		//end exchange the polygon lines

		//copy data to HW system
		fun_coeffs_opti = final_fun_coeffs;

#if 0
		//test
		//construct neighbors from initial nodes
		for (int i = 0; i < fun_coeffs_opti.size(); ++i)
		{
			int ii = (i + 1) % fun_coeffs_opti.size();
			PolygonNeighborEnergy tmp_node;
			tmp_node.lidx = i;
			tmp_node.ridx = ii;
			std::cerr << "cur tmp_node.lidx, tmp_node.ridx: " << tmp_node.lidx << ", " << tmp_node.ridx << std::endl;
			ComputeNeighborPairsEuncEnergyValue(pnts, fun_coeffs_opti, tmp_node);

			//float eu_lv = EuPolygonPntsToPolygonLineDistance(pnts, cur_fun_coeffs[tmp_node.lidx]);
			//float en_lv = EnPolygonPntsNormalToPolygonLineNormal(pnts, cur_fun_coeffs[tmp_node.lidx]);
			//float eu_rv = EuPolygonPntsToPolygonLineDistance(pnts, cur_fun_coeffs[tmp_node.ridx]);
			//float en_rv = EnPolygonPntsNormalToPolygonLineNormal(pnts, cur_fun_coeffs[tmp_node.ridx]);
			//float eun_v = polygon_extract_energy_weights_.wu_lambda_*(eu_lv + eu_rv)
			//	+ polygon_extract_energy_weights_.wn_lambda_*(en_lv + en_rv);
			//compute the neighbor lines remained pnts
			std::vector<int> remained_neighbor_pnts_idxs;
			ComputeNeighborLinesFunsRemainedPntsIdxs(pnts, fun_coeffs_opti, tmp_node.lidx, tmp_node.ridx, remained_neighbor_pnts_idxs);

			//test
			//plane_obj_name_
			std::string tmp_pnts_path_str = plane_output_dir_ + std::to_string(cur_idx) + "_final_plane_pnts_" + std::to_string(i) + "_" + std::to_string(ii) + ".obj";
			std::string tmp_edges_path_str = plane_output_dir_ + std::to_string(cur_idx) + "_final_plane_edges_" + std::to_string(i) + "_" + std::to_string(ii) + ".obj";
			std::vector<Eigen::Vector2f> tmp_plane_pnts;
			std::vector<Eigen::Vector2f> tmp_plane_edges;
			for (int j = 0; j < remained_neighbor_pnts_idxs.size(); ++j)
			{
				int test_idx = remained_neighbor_pnts_idxs[j];
				Eigen::Vector2f test_pnt = pnts[test_idx].pos_;
				tmp_plane_pnts.emplace_back(test_pnt);
			}
			std::cerr << "fun_coeffs_opti[tmp_node.lidx].associated_pnts_idxs_ num: " <<
				fun_coeffs_opti[tmp_node.lidx].associated_pnts_idxs_.size() << std::endl;
			std::cerr << "fun_coeffs_opti[tmp_node.ridx].associated_pnts_idxs_ num: " <<
				fun_coeffs_opti[tmp_node.ridx].associated_pnts_idxs_.size() << std::endl;
			for (int j = 0; j < fun_coeffs_opti[tmp_node.lidx].associated_pnts_idxs_.size() - 1; ++j)
			{
				int test_idx0 = fun_coeffs_opti[tmp_node.lidx].associated_pnts_idxs_[j];
				Eigen::Vector2f test_pnt0 = pnts[test_idx0].pos_;
				int test_idx1 = fun_coeffs_opti[tmp_node.lidx].associated_pnts_idxs_[j + 1];
				Eigen::Vector2f test_pnt1 = pnts[test_idx1].pos_;
				tmp_plane_edges.emplace_back(test_pnt0);
				tmp_plane_edges.emplace_back(test_pnt1);
			}
			for (int j = 0; j < fun_coeffs_opti[tmp_node.ridx].associated_pnts_idxs_.size() - 1; ++j)
			{
				int test_idx0 = fun_coeffs_opti[tmp_node.ridx].associated_pnts_idxs_[j];
				Eigen::Vector2f test_pnt0 = pnts[test_idx0].pos_;
				int test_idx1 = fun_coeffs_opti[tmp_node.ridx].associated_pnts_idxs_[j + 1];
				Eigen::Vector2f test_pnt1 = pnts[test_idx1].pos_;
				tmp_plane_edges.emplace_back(test_pnt0);
				tmp_plane_edges.emplace_back(test_pnt1);
			}
			SavePnts2dIntoPnts3DOBJ(tmp_pnts_path_str, tmp_plane_pnts);
			SaveLinesPnts2dIntoLinesPnts3DOBJ(tmp_edges_path_str, tmp_plane_edges);
			//end test
		}
		//end test
#endif
		std::cerr << "end optimize new polygon optimization..." << std::endl;
		return;
	}

	bool HWPlane::PolygonExtractionFromSortedLinesFunctions(const std::vector<Eigen::Vector3f>& lines_funcs, std::vector<Eigen::Vector2f>& poly_pnts)
	{
		std::cerr << "start to extract polygon from sorted lines function..." << std::endl;
		int lines_num = static_cast<int> (lines_funcs.size());
		if (lines_num < 3)
		{
			std::cerr << "invalid lines number..." << std::endl;
			return false;
		}
		std::vector<Eigen::Vector2f> pnts_new;
		for (int i = 0; i < lines_funcs.size(); ++i)
		{
			int ii = (i + 1) % lines_funcs.size();
			Eigen::Vector3f line0_fun = lines_funcs[i];
			Eigen::Vector3f line1_fun = lines_funcs[ii];
			//
			Eigen::Vector2f pnt_in;
			bool inter_flag = ComputeLine2LineIntersection(line0_fun, line1_fun, pnt_in);
			/*std::cerr << "line0_fun: " << line0_fun.transpose() << std::endl;
			std::cerr << "line1_fun: " << line1_fun.transpose() << std::endl;
			std::cerr << "pnt_in: " << pnt_in.transpose() << std::endl;*/
			if (inter_flag)
			{
				pnts_new.emplace_back(pnt_in);
			}
		}
		poly_pnts = pnts_new;
		std::cerr << "end extract polygon from sorted lines function..." << std::endl;
		return true;
	}

	void HWPlane::ComputeSortedAllNodeNeighborEnergysFromHWLineToPntsIdxs(const std::vector<HWOptiPnt2D>& pnts, const std::vector<HWLineToPntsIdxs>& fun_coeffs,
		std::vector<PolygonNeighborEnergy>& all_neighbor_energys)
	{
		std::vector<PolygonNeighborEnergy> cur_neighbors_energys_nodes;
		//construct neighbors from initial nodes
		for (int i = 0; i < fun_coeffs.size(); ++i)
		{
			int ii = (i + 1) % fun_coeffs.size();
			PolygonNeighborEnergy tmp_node;
			tmp_node.lidx = i;
			tmp_node.ridx = ii;
			float eu_lv = EuPolygonPntsToPolygonLineDistance(pnts, fun_coeffs[tmp_node.lidx]);
			float en_lv = EnPolygonPntsNormalToPolygonLineNormal(pnts, fun_coeffs[tmp_node.lidx]);
			float eu_rv = EuPolygonPntsToPolygonLineDistance(pnts, fun_coeffs[tmp_node.ridx]);
			float en_rv = EnPolygonPntsNormalToPolygonLineNormal(pnts, fun_coeffs[tmp_node.ridx]);
			float eun_v = polygon_extract_energy_weights_.wu_lambda_*(eu_lv + eu_rv)
				+ polygon_extract_energy_weights_.wn_lambda_*(en_lv + en_rv);
			//compute the neighbor lines remained pnts
			std::vector<int> remained_neighbor_pnts_idxs;
			ComputeNeighborLinesFunsRemainedPntsIdxs(pnts, fun_coeffs, tmp_node.lidx, tmp_node.ridx, remained_neighbor_pnts_idxs);

			////test
			////plane_obj_name_
			//std::string tmp_pnts_path_str = plane_output_dir_ + "plane_pnts_" + std::to_string(i) + "_" + std::to_string(ii) + ".obj";
			//std::string tmp_edges_path_str = plane_output_dir_ + "plane_edges_" + std::to_string(i) + "_" + std::to_string(ii) + ".obj";
			//std::vector<Eigen::Vector2f> tmp_plane_pnts;
			//std::vector<Eigen::Vector2f> tmp_plane_edges;
			//for (int j = 0; j < remained_neighbor_pnts_idxs.size(); ++j)
			//{
			//	int test_idx = remained_neighbor_pnts_idxs[j];
			//	Eigen::Vector2f test_pnt = pnts[test_idx].pos_;
			//	tmp_plane_pnts.emplace_back(test_pnt);
			//}
			//for (int j = 0; j < fun_coeffs[tmp_node.lidx].associated_pnts_idxs_.size() - 1; ++j)
			//{
			//	int test_idx0 = fun_coeffs[tmp_node.lidx].associated_pnts_idxs_[j];
			//	Eigen::Vector2f test_pnt0 = pnts[test_idx0].pos_;
			//	int test_idx1 = fun_coeffs[tmp_node.lidx].associated_pnts_idxs_[j + 1];
			//	Eigen::Vector2f test_pnt1 = pnts[test_idx1].pos_;
			//	tmp_plane_edges.emplace_back(test_pnt0);
			//	tmp_plane_edges.emplace_back(test_pnt1);
			//}
			//for (int j = 0; j < fun_coeffs[tmp_node.ridx].associated_pnts_idxs_.size() - 1; ++j)
			//{
			//	int test_idx0 = fun_coeffs[tmp_node.ridx].associated_pnts_idxs_[j];
			//	Eigen::Vector2f test_pnt0 = pnts[test_idx0].pos_;
			//	int test_idx1 = fun_coeffs[tmp_node.ridx].associated_pnts_idxs_[j + 1];
			//	Eigen::Vector2f test_pnt1 = pnts[test_idx1].pos_;
			//	tmp_plane_edges.emplace_back(test_pnt0);
			//	tmp_plane_edges.emplace_back(test_pnt1);
			//}
			//SavePnts2dIntoPnts3DOBJ(tmp_pnts_path_str, tmp_plane_pnts);
			//SaveLinesPnts2dIntoLinesPnts3DOBJ(tmp_edges_path_str, tmp_plane_edges);
			////end test

			float remained_neighbor_pnt_idx = static_cast<float> (remained_neighbor_pnts_idxs.size());
			float ec_neighbor_v = polygon_extract_energy_weights_.wc_lambda_ * (remained_neighbor_pnt_idx / pnts.size());
			tmp_node.energy_value = eun_v + ec_neighbor_v;
			/*std::cerr << "tmp_node.lidx, tmp_node.ridx->value: " << tmp_node.lidx << "," << tmp_node.ridx << "->" << tmp_node.energy_value << std::endl;
			std::cerr << "tmp_node.lidx, tmp_node.ridx->eu_lv, en_lv, eu_rv, en_rv: " << tmp_node.lidx << ", " << tmp_node.ridx << "->" 
				<< eu_lv << ", " << en_lv << ", " << eu_rv << ", " << en_rv << std::endl;
			std::cerr << "ec_neighbor_v: " << ec_neighbor_v << std::endl;
			std::cerr << "tmp_node.energy_value: " << tmp_node.energy_value << std::endl;*/
			cur_neighbors_energys_nodes.emplace_back(tmp_node);
		}
		std::sort(cur_neighbors_energys_nodes.begin(), cur_neighbors_energys_nodes.end(), ComparePolygonNeighborEnergyValueDecrease);

		////test
		//for (int i = 0; i < cur_neighbors_energys_nodes.size(); ++i)
		//{
		//	std::cerr << i << "->" << "(" << cur_neighbors_energys_nodes[i].lidx <<
		//		" " << cur_neighbors_energys_nodes[i].ridx << ")" << std::endl;
		//}
		////end test

		all_neighbor_energys = cur_neighbors_energys_nodes;
	}

	void HWPlane::ComputeAllNodeNeighborLinesAngleEnergysFromHWLineToPntsIdxsIncrease(const std::vector<HWOptiPnt2D>& pnts, const std::vector<HWLineToPntsIdxs>& fun_coeffs,
		std::vector<PolygonNeighborEnergy>& all_neighbor_energys)
	{
		std::vector<PolygonNeighborEnergy> cur_neighbors_energys_nodes;
		//construct neighbors from initial nodes
		for (int i = 0; i < fun_coeffs.size(); ++i)
		{
			int ii = (i + 1) % fun_coeffs.size();
			PolygonNeighborEnergy tmp_node;
			tmp_node.lidx = i;
			tmp_node.ridx = ii;
			
			Eigen::Vector2f lfun_normal_coeff, rfun_normal_coeff;
			ComputeLineFunctionVerticeNormal(fun_coeffs[tmp_node.lidx].line_fun_, lfun_normal_coeff);
			ComputeLineFunctionVerticeNormal(fun_coeffs[tmp_node.ridx].line_fun_, rfun_normal_coeff);
			Eigen::Vector2f lpnt_normal, rpnt_normal;
			ComputeSingleOptiPntLinePntsAssociatedPntsNormal(pnts, fun_coeffs, tmp_node.lidx, lpnt_normal);
			ComputeSingleOptiPntLinePntsAssociatedPntsNormal(pnts, fun_coeffs, tmp_node.ridx, rpnt_normal);
			if (lpnt_normal.dot(lfun_normal_coeff) < 0)
			{
				lfun_normal_coeff *= -1;
			}
			if (rpnt_normal.dot(rfun_normal_coeff) < 0)
			{
				rfun_normal_coeff *= -1;
			}
			float tmp_angle = ComputeAngleFromTwoLinesVector2D(lfun_normal_coeff, rfun_normal_coeff);
			//std::cerr << "line_vertical: " << line_vertical.transpose() << std::endl;
			//std::cerr << "tmp_pnt_normal: " << tmp_pnt_normal.transpose() << std::endl;
			//std::cerr << "tmp_angle: " << tmp_angle << std::endl;
			/*if (tmp_angle > ANGLE_VERTICAL)
			{
				tmp_angle = ANGLE_HALF_CIRCLE - tmp_angle;
			}*/

			tmp_node.energy_value = tmp_angle;
			/*std::cerr << "tmp_node.lidx, tmp_node.ridx->value: " << tmp_node.lidx << "," << tmp_node.ridx << "->" << tmp_node.energy_value << std::endl;
			std::cerr << "tmp_node.lidx, tmp_node.ridx->eu_lv, en_lv, eu_rv, en_rv: " << tmp_node.lidx << ", " << tmp_node.ridx << "->"
			<< eu_lv << ", " << en_lv << ", " << eu_rv << ", " << en_rv << std::endl;
			std::cerr << "ec_neighbor_v: " << ec_neighbor_v << std::endl;
			std::cerr << "tmp_node.energy_value: " << tmp_node.energy_value << std::endl;*/
			cur_neighbors_energys_nodes.emplace_back(tmp_node);
		}
		std::sort(cur_neighbors_energys_nodes.begin(), cur_neighbors_energys_nodes.end(), ComparePolygonNeighborEnergyValueIncrease);

		////test
		//for (int i = 0; i < cur_neighbors_energys_nodes.size(); ++i)
		//{
		//	std::cerr << i << "->" << "(" << cur_neighbors_energys_nodes[i].lidx <<
		//		" " << cur_neighbors_energys_nodes[i].ridx << ")" << std::endl;
		//}
		////end test

		all_neighbor_energys = cur_neighbors_energys_nodes;
	}

	void HWPlane::ComputeSingleOptiPntLinePntsAssociatedPntsNormal(const std::vector<HWOptiPnt2D>& pnts, const std::vector<HWLineToPntsIdxs>& fun_coeffs, int pidx, Eigen::Vector2f& pnt_normal)
	{
		HWLineToPntsIdxs line_pnts_idxs = fun_coeffs[pidx];
		if (line_pnts_idxs.associated_pnts_idxs_.empty())
		{
			pnt_normal = Eigen::Vector2f(0.0, 0.0);
			return;
		}
		Eigen::Vector2f sum_normal = Eigen::Vector2f(0.0, 0.0);
		for (int i = 0; i < line_pnts_idxs.associated_pnts_idxs_.size(); ++i)
		{
			int idx = line_pnts_idxs.associated_pnts_idxs_[i];
			Eigen::Vector2f tmp_pnt_normal = pnts[idx].normal_;
			sum_normal += tmp_pnt_normal;
		}
		int sum_num = static_cast<int>(line_pnts_idxs.associated_pnts_idxs_.size());
		pnt_normal = sum_normal / sum_num;
	}

	void HWPlane::ComputeEnergyNodeMeFunFromItsIdxs(const std::vector<HWOptiPnt2D>& pnts, HWLineToPntsIdxs& fun_coeff)
	{
		if (fun_coeff.associated_pnts_idxs_.empty())
		{
			return;
		}
		std::vector<Eigen::Vector2f> pnts_pos;
		for (int i = 0; i < fun_coeff.associated_pnts_idxs_.size(); ++i)
		{
			int idx = fun_coeff.associated_pnts_idxs_[i];
			pnts_pos.emplace_back(pnts[idx].pos_);
		}
		FittingLine2DFromPnts2D(pnts_pos, fun_coeff.line_fun_);
	}

	float HWPlane::EuPolygonPntsToPolygonLinesDistances(const std::vector<HWOptiPnt2D>& pnts_in,
		const std::vector<HWLineToPntsIdxs>& fun_coeffs)
	{
		int icount = 0;
		float sum_value = 0.0;
		for (int i = 0; i < fun_coeffs.size(); ++i)
		{
			Eigen::Vector3f tmp_line = fun_coeffs[i].line_fun_;
			for (int j = 0; j < fun_coeffs[i].associated_pnts_idxs_.size(); ++j)
			{
				int idx = fun_coeffs[i].associated_pnts_idxs_[j];
				Eigen::Vector2f tmp_pnt = pnts_in[idx].pos_;
				float dist_to_line = ComputePntToLineFunction2D(tmp_line, tmp_pnt);
				sum_value += dist_to_line;
				++icount;
			}
		}
		if (icount < 1)
		{
			return KMAX_FLOAT_LIMIT_VALUE;
		}
		else
		{
			float vv = sum_value / icount;
			return vv;
		}
	}

	float HWPlane::EuPolygonPntsToPolygonLineDistance(const std::vector<HWOptiPnt2D>& pnts_in,
		const HWLineToPntsIdxs& fun_coeff)
	{
		int icount = 0;
		float sum_value = 0.0;
		for (int i = 0; i < fun_coeff.associated_pnts_idxs_.size(); ++i)
		{
			int idx = fun_coeff.associated_pnts_idxs_[i];
			Eigen::Vector2f tmp_pnt = pnts_in[idx].pos_;
			float dist_to_line = ComputePntToLineFunction2D(fun_coeff.line_fun_, tmp_pnt);
			sum_value += dist_to_line;
			++icount;
		}
		if (icount < 1)
		{
			return KMAX_FLOAT_LIMIT_VALUE;
		}
		else
		{
			float vv = sum_value / icount;
			return vv;
		}
	}

	float HWPlane::EnPolygonPntsNormalToPolygonLinesNormal(const std::vector<HWOptiPnt2D>& pnts_in,
		const std::vector<HWLineToPntsIdxs>& fun_coeffs)
	{
		float sum_angle = 0.0;
		int icount = 0;
		for (int i = 0; i < fun_coeffs.size(); ++i)
		{
			Eigen::Vector2f line_vertical;
			bool line_flag = ComputeLineFunctionVerticeNormal(fun_coeffs[i].line_fun_, line_vertical);
			if (!line_flag)
				continue;
			for (int j = 0; j < fun_coeffs[i].associated_pnts_idxs_.size(); ++j)
			{
				int idx = fun_coeffs[i].associated_pnts_idxs_[j];
				HWOptiPnt2D tmp_pnt = pnts_in[idx];
				Eigen::Vector2f tmp_pnt_normal = tmp_pnt.normal_;
				float tmp_angle = ComputeAngleFromTwoLinesVector2D(line_vertical, tmp_pnt_normal);
				if (tmp_angle > ANGLE_VERTICAL)
				{
					tmp_angle = ANGLE_HALF_CIRCLE - tmp_angle;
				}
				sum_angle += tmp_angle;
				icount++;
			}
		}
		if (icount < 1)
		{
			return KMAX_FLOAT_LIMIT_VALUE;
		}
		else
		{
			float vv = sum_angle / icount;
			return vv;
		}
	}

	float HWPlane::EnPolygonPntsNormalToPolygonLineNormal(const std::vector<HWOptiPnt2D>& pnts_in,
		const HWLineToPntsIdxs& fun_coeff)
	{
		float sum_angle = 0.0;
		int icount = 0;
		Eigen::Vector2f line_vertical;
		bool line_flag = ComputeLineFunctionVerticeNormal(fun_coeff.line_fun_, line_vertical);
		if (!line_flag)
			return KMAX_FLOAT_LIMIT_VALUE;
		for (int i = 0; i < fun_coeff.associated_pnts_idxs_.size(); ++i)
		{
			int idx = fun_coeff.associated_pnts_idxs_[i];
			Eigen::Vector2f tmp_pnt_normal = pnts_in[idx].normal_;
			float tmp_angle = ComputeAngleFromTwoLinesVector2D(line_vertical, tmp_pnt_normal);
			//std::cerr << "line_vertical: " << line_vertical.transpose() << std::endl;
			//std::cerr << "tmp_pnt_normal: " << tmp_pnt_normal.transpose() << std::endl;
			//std::cerr << "tmp_angle: " << tmp_angle << std::endl;
			if (tmp_angle >= ANGLE_VERTICAL)
			{
				tmp_angle = ANGLE_HALF_CIRCLE - tmp_angle;
			}
			//std::cerr << "tmp_angle: " << tmp_angle << std::endl;
			sum_angle += tmp_angle;
			icount++;
		}
		if (icount < 1)
		{
			return KMAX_FLOAT_LIMIT_VALUE;
		}
		else
		{
			float vv = sum_angle / icount;
			//std::cerr << "vv: " << vv << std::endl;
			return vv;
		}
	}

	float HWPlane::EaPolygonArea(const std::vector<HWOptiPnt2D>& pnts_in, const std::vector<HWLineToPntsIdxs>& fun_coeffs)
	{
		std::vector<Eigen::Vector3f> lines_funs;
		for (int i = 0; i < fun_coeffs.size(); ++i)
		{
			lines_funs.emplace_back(fun_coeffs[i].line_fun_);
		}
		std::vector<Eigen::Vector2f> poly_lines;
		PolygonExtractionFromSortedLinesFunctions(lines_funs, poly_lines);
		float poly_area = ComputePolygon2DAreaFromPolygonPnts2D(poly_lines);
		return poly_area;
	}

	float HWPlane::EsPolygonLinesNum(const std::vector<HWLineToPntsIdxs>& fun_coeffs, const std::vector<HWLineToPntsIdxs>& initial_fun_coeffs)
	{
		float fun_coeffs_num = static_cast<float> (fun_coeffs.size());
		float initial_fun_coeffs_num = static_cast<float>(initial_fun_coeffs.size());
		float es = fun_coeffs_num / initial_fun_coeffs_num;
		return es;
	}

	float HWPlane::EcPolygonPntsInPolygonNum(const std::vector<HWOptiPnt2D>& pnts_in, const std::vector<int>& remained_pnts_idxs)
	{
		float all_pnts_num = static_cast<float> (pnts_in.size());
		if (all_pnts_num < KMIN_FLOAT_THRESHOLD_REFINED)
		{
			return KMAX_FLOAT_LIMIT_VALUE;
		}
		float pnts_remained_num = static_cast<float> (remained_pnts_idxs.size());
		float ec = pnts_remained_num / all_pnts_num;
		return ec;
	}

	float HWPlane::ComputeAllEnergyValueFromPolygonExtractedPnts(const std::vector<HWOptiPnt2D>& pnts_in, 
		const std::vector<HWLineToPntsIdxs>& fun_coeffs, const std::vector<int>& remained_pnts_idxs)
	{
		float all_eu_v = EuPolygonPntsToPolygonLinesDistances(pnts_in, fun_coeffs);
		float all_en_v = EnPolygonPntsNormalToPolygonLinesNormal(pnts_in, fun_coeffs);
		float all_ea_v = EaPolygonArea(pnts_in, fun_coeffs);
		float all_es_v = EsPolygonLinesNum(fun_coeffs, initial_funs_coeffs_);
		float all_ec_v = EcPolygonPntsInPolygonNum(pnts_in, remained_pnts_idxs);

		std::cerr << "all_eu_v: " << all_eu_v << std::endl;
		std::cerr << "all_en_v: " << all_en_v << std::endl;
		std::cerr << "all_ea_v: " << all_ea_v << std::endl;
		std::cerr << "all_es_v: " << all_es_v << std::endl;
		std::cerr << "all_ec_v: " << all_ec_v << std::endl;

		float all_value = polygon_extract_energy_weights_.wu_lambda_ * all_eu_v + 
			polygon_extract_energy_weights_.wn_lambda_ * all_en_v + 
			polygon_extract_energy_weights_.wa_lambda_ * all_ea_v + 
			polygon_extract_energy_weights_.ws_lambda_ * all_es_v + 
			polygon_extract_energy_weights_.wc_lambda_ * all_ec_v;

		std::cerr << "all_value: " << all_value << std::endl;
		return all_value;
	}

	float HWPlane::ComputeNeighborPairsEuncEnergyValue(const std::vector<HWOptiPnt2D>& pnts_in, const std::vector<HWLineToPntsIdxs>& fun_coeffs, PolygonNeighborEnergy& lines_pairs_energy)
	{
		float eu_lv = EuPolygonPntsToPolygonLineDistance(pnts_in, fun_coeffs[lines_pairs_energy.lidx]);
		float en_lv = EnPolygonPntsNormalToPolygonLineNormal(pnts_in, fun_coeffs[lines_pairs_energy.lidx]);
		float eu_rv = EuPolygonPntsToPolygonLineDistance(pnts_in, fun_coeffs[lines_pairs_energy.ridx]);
		float en_rv = EnPolygonPntsNormalToPolygonLineNormal(pnts_in, fun_coeffs[lines_pairs_energy.ridx]);
		float eun_v = polygon_extract_energy_weights_.wu_lambda_*(eu_lv + eu_rv)
			+ polygon_extract_energy_weights_.wn_lambda_*(en_lv + en_rv);
		//compute the neighbor lines remained pnts
		std::vector<int> remained_neighbor_pnts_idxs;
		ComputeNeighborLinesFunsRemainedPntsIdxs(pnts_in, fun_coeffs, lines_pairs_energy.lidx, lines_pairs_energy.ridx, remained_neighbor_pnts_idxs);
		float ec_neighbor_v = polygon_extract_energy_weights_.wc_lambda_ * (remained_neighbor_pnts_idxs.size() / pnts_in.size());
		lines_pairs_energy.energy_value = eun_v + ec_neighbor_v;
		return lines_pairs_energy.energy_value;
	}

	void HWPlane::InsertHWLinesPairEnergyIntoHWLinesPairsVecByOrder(std::vector<PolygonNeighborEnergy>& neighbors_vec, const PolygonNeighborEnergy& lines_pairs)
	{
		std::vector<PolygonNeighborEnergy>::iterator it = neighbors_vec.begin();
		for (; it != neighbors_vec.end(); ++it)
		{
			if (it->energy_value < lines_pairs.energy_value)
			{
				break;
			}
		}
		neighbors_vec.insert(it, lines_pairs);
	}

	void HWPlane::DeleteHWLinesPairEnergyFromHWLinesPairs(std::vector<PolygonNeighborEnergy>& neighbors_vec, int idx)
	{
		//std::cerr << "to do next..." << std::endl;
		neighbors_vec.erase(neighbors_vec.begin() + idx);
	}

	bool HWPlane::CirclePntsIdxsHasZeroFlag(const std::vector<int>& circle_idxs)
	{
		for (int i = 0; i < circle_idxs.size(); ++i)
		{
			if (circle_idxs[i] == 0)
			{
				return true;
			}
		}
		return false;
	}

	void HWPlane::ComputeAllLinesFunsRemainedPntsIdxsFromPnts(const std::vector<HWOptiPnt2D>& pnts_in, const std::vector<HWLineToPntsIdxs>& fun_coeffs,
		std::vector<int>& all_remained_pnts_idxs)
	{
		std::vector<int> cur_remained_pnts_idxs;
		//get remained points idxs
		std::vector<bool> all_pnts_idxs_flag;
		all_pnts_idxs_flag.resize(pnts_in.size(), false);
		for (int i = 0; i < fun_coeffs.size(); ++i)
		{
			for (int j = 0; j < fun_coeffs[i].associated_pnts_idxs_.size(); ++j)
			{
				int idx = fun_coeffs[i].associated_pnts_idxs_[j];
				all_pnts_idxs_flag[idx] = true;
			}
		}
		for (int i = 0; i < all_pnts_idxs_flag.size(); ++i)
		{
			if (!all_pnts_idxs_flag[i])
			{
				cur_remained_pnts_idxs.emplace_back(i);
			}
		}
		all_remained_pnts_idxs = cur_remained_pnts_idxs;
	}

	void HWPlane::ComputeNeighborLinesFunsRemainedPntsIdxs(const std::vector<HWOptiPnt2D>& pnts_in, const std::vector<HWLineToPntsIdxs>& fun_coeffs,
		int lidx, int ridx, std::vector<int>& neighbor_remained_pnts_idxs)
	{
		std::vector<int> lpnts_idxs = fun_coeffs[lidx].associated_pnts_idxs_;
		std::vector<int> rpnts_idxs = fun_coeffs[ridx].associated_pnts_idxs_;
		std::vector<int> neighbor_remain_pnts_idxs;
		int pnts_num = static_cast<int>(pnts_in.size());
		int lpnts_end_idx = lpnts_idxs.back();
		int rpnts_front_idx = rpnts_idxs.front();
		int cur_pnts_idx = lpnts_end_idx + 1;
		while ((cur_pnts_idx % pnts_num) < rpnts_front_idx)
		{
			int cur_pnts_idx_circle = cur_pnts_idx % pnts_num;
			neighbor_remain_pnts_idxs.emplace_back(cur_pnts_idx_circle);
			cur_pnts_idx++;
			if (cur_pnts_idx >= pnts_num)
			{
				cur_pnts_idx = 0;
			}
		}
		neighbor_remained_pnts_idxs = neighbor_remain_pnts_idxs;
	}

	//important...
	float HWPlane::PolygonLinesExchangePolygonLinePnts(const std::vector<HWOptiPnt2D>& pnts_in, const std::vector<HWLineToPntsIdxs>& fun_coeffs, const std::vector<int>& remained_pnts_idxs,
		int l_idx, int r_idx, std::vector<HWLineToPntsIdxs>& fun_coeffs_opti, std::vector<int>& remained_pnts_idxs_opti)
	{
		HWLineToPntsIdxs lfun_coeff = fun_coeffs[l_idx];
		HWLineToPntsIdxs rfun_coeff = fun_coeffs[r_idx];
		std::vector<int> lpnts_idxs = lfun_coeff.associated_pnts_idxs_;
		std::vector<int> rpnts_idxs = rfun_coeff.associated_pnts_idxs_;
		//check it has remained idx
		std::vector<int> lr_pnts_remained_idxs;
		int pnts_num = static_cast<int>(pnts_in.size());
		/*int lpnts_front_idx = lpnts_idxs.front();
		int lpnts_end_idx = lpnts_idxs.back();
		int rpnts_front_idx = rpnts_idxs.front();
		int rpnts_end_idx = rpnts_idxs.back();*/
		ComputeNeighborLinesFunsRemainedPntsIdxs(pnts_in, fun_coeffs, l_idx, r_idx, lr_pnts_remained_idxs);
		/*if (lpnts_end_idx != rpnts_front_idx)
		{
			for (int i = lpnts_end_idx + 1; (i% pnts_num) != rpnts_front_idx; ++i)
			{
				int idx = (i% pnts_num);
				lr_pnts_remained_idxs.emplace_back(idx);
			}
		}*/
		std::vector<std::pair<int, int> > lr_pnts_idxs_to_labels;
		int initial_lpnts_idxs_end = static_cast<int>(lpnts_idxs.size());
		//get the data from lpnts_idxs, lr_pnts_remained_idxs, rpnts_idxs
		for (int i = 0; i < lpnts_idxs.size(); ++i)
		{
			int idx = lpnts_idxs[i];
			std::pair<int, int> tmp_pnt = std::make_pair(idx, -1);
			lr_pnts_idxs_to_labels.emplace_back(tmp_pnt);
		}
		int initial_lr_pnts_remained_num = static_cast<int>(lr_pnts_remained_idxs.size());
		for (int i = 0; i < lr_pnts_remained_idxs.size(); ++i)
		{
			int idx = lr_pnts_remained_idxs[i];
			std::pair<int, int> tmp_pnt = std::make_pair(idx, 0);
			lr_pnts_idxs_to_labels.emplace_back(tmp_pnt);
		}
		int initial_rpnts_num = static_cast<int>(rpnts_idxs.size());
		for (int i = 0; i < rpnts_idxs.size(); ++i)
		{
			int idx = rpnts_idxs[i];
			std::pair<int, int> tmp_pnt = std::make_pair(idx, 1);
			lr_pnts_idxs_to_labels.emplace_back(tmp_pnt);
		}
		Eigen::Vector3f left_fun_coeff = lfun_coeff.line_fun_;
		Eigen::Vector3f right_fun_coeff = rfun_coeff.line_fun_;
		//
		Eigen::Vector2f left_fun_vertical,right_fun_vertical;
		ComputeLineFunctionVerticeNormal(left_fun_coeff, left_fun_vertical);
		ComputeLineFunctionVerticeNormal(right_fun_coeff, right_fun_vertical);
		//Eigen::Vector2f left_fun_dir = Eigen::Vector2f(left_fun_coeff[0], left_fun_coeff[1]);
		//Eigen::Vector2f right_fun_dir = Eigen::Vector2f(right_fun_coeff[0], right_fun_coeff[1]);
		//std::cerr << "left_fun_dir: " << left_fun_dir.transpose() << std::endl;
		//std::cerr << "right_fun_dir: " << right_fun_dir.transpose() << std::endl;
		
		////test 
		//if (l_idx == 2 && r_idx == 3)
		//{
		//	//std::string tmp_pnts_path_str = plane_output_dir_ + "plane_pnts_" + std::to_string(i) + "_" + std::to_string(ii) + ".obj";
		//	std::string test_left_line_normal_path = plane_output_dir_ + "plane_pnts_left_line_normal_" + std::to_string(l_idx) + "_" + std::to_string(r_idx) + ".obj";
		//	std::string test_right_line_normal_path = plane_output_dir_ + "plane_pnts_right_line_normal_" + std::to_string(l_idx) + "_" + std::to_string(r_idx) + ".obj";
		//	std::string test_left_vertices_path = plane_output_dir_ + "plane_pnts_left_vertices_normal_" + std::to_string(l_idx) + "_" + std::to_string(r_idx) + ".obj";
		//	std::string test_right_vertices_path = plane_output_dir_ + "plane_pnts_right_vertices_normal_" + std::to_string(l_idx) + "_" + std::to_string(r_idx) + ".obj";
		//	std::vector<Eigen::Vector2f> test_left_line_normal_pos;
		//	std::vector<Eigen::Vector2f> test_right_line_normal_pos;
		//	std::vector<Eigen::Vector2f> test_left_pnt2d_pos;
		//	std::vector<Eigen::Vector2f> test_left_pnt2d_normal;
		//	std::vector<Eigen::Vector2f> test_right_pnt2d_pos;
		//	std::vector<Eigen::Vector2f> test_right_pnt2d_normal;	
		//	std::vector<int> l_pnts_label_nagtive1_idxs;
		//	std::vector<int> r_pnts_label1_idxs;
		//	for (int i = 0; i < lr_pnts_idxs_to_labels.size(); ++i)
		//	{
		//		if (lr_pnts_idxs_to_labels[i].second == -1)
		//		{
		//			l_pnts_label_nagtive1_idxs.emplace_back(lr_pnts_idxs_to_labels[i].first);
		//		}
		//		if (lr_pnts_idxs_to_labels[i].second == 1)
		//		{
		//			r_pnts_label1_idxs.emplace_back(lr_pnts_idxs_to_labels[i].first);
		//		}
		//	}
		//	Eigen::Vector2f center_left_pos = Eigen::Vector2f(0, 0);
		//	for (int i = 0; i < l_pnts_label_nagtive1_idxs.size(); ++i)
		//	{
		//		int idx = l_pnts_label_nagtive1_idxs[i];
		//		Eigen::Vector2f test_l_pnt_pos = pnts_in[idx].pos_;
		//		Eigen::Vector2f test_l_pnt_normal = pnts_in[idx].normal_;
		//		test_left_pnt2d_pos.emplace_back(test_l_pnt_pos);
		//		test_left_pnt2d_normal.emplace_back(test_l_pnt_normal);
		//		center_left_pos += test_l_pnt_pos;
		//	}
		//	center_left_pos = center_left_pos / l_pnts_label_nagtive1_idxs.size();
		//	Eigen::Vector2f center_left_end_pos = center_left_pos + left_fun_vertical;
		//	test_left_line_normal_pos.emplace_back(center_left_pos);
		//	test_left_line_normal_pos.emplace_back(center_left_end_pos);
		//	Eigen::Vector2f center_right_pos = Eigen::Vector2f(0, 0);
		//	for (int i = 0; i < r_pnts_label1_idxs.size(); ++i)
		//	{
		//		int idx = r_pnts_label1_idxs[i];
		//		Eigen::Vector2f test_r_pnt_pos = pnts_in[idx].pos_;
		//		Eigen::Vector2f test_r_pnt_normal = pnts_in[idx].normal_;
		//		test_right_pnt2d_pos.emplace_back(test_r_pnt_pos);
		//		test_right_pnt2d_normal.emplace_back(test_r_pnt_normal);
		//		center_right_pos += test_r_pnt_pos;
		//	}
		//	center_right_pos = center_right_pos / r_pnts_label1_idxs.size();
		//	Eigen::Vector2f center_right_end_pos = center_right_pos + right_fun_vertical;
		//	test_right_line_normal_pos.emplace_back(center_right_pos);
		//	test_right_line_normal_pos.emplace_back(center_right_end_pos);
		//	SaveLinesPnts2dIntoLinesPnt3DOBJWithNormal(test_left_vertices_path, 
		//		test_left_pnt2d_pos, test_left_pnt2d_normal);
		//	SaveLinesPnts2dIntoLinesPnt3DOBJWithNormal(test_right_vertices_path,
		//		test_right_pnt2d_pos, test_right_pnt2d_normal);
		//	SaveLinesPnts2dIntoLinesPnts3DOBJ(test_left_line_normal_path, test_left_line_normal_pos);
		//	SaveLinesPnts2dIntoLinesPnts3DOBJ(test_right_line_normal_path, test_right_line_normal_pos);
		//}
		////end test

		if (!lr_pnts_remained_idxs.empty())
		{
			std::cerr << "has pnts remained idxs...." << std::endl;
			for (int i = initial_lpnts_idxs_end; i < initial_lpnts_idxs_end + initial_lr_pnts_remained_num; ++i)
			{
				int cur_idx = lr_pnts_idxs_to_labels[i].first;
				Eigen::Vector2f cur_pnt_normal = pnts_in[cur_idx].normal_;
				Eigen::Vector2f cur_pnt_pos = pnts_in[cur_idx].pos_;
				float dist_to_left = ComputePntToLineFunction2D(left_fun_coeff, cur_pnt_pos);
				float dist_to_right = ComputePntToLineFunction2D(right_fun_coeff, cur_pnt_pos);
				float angle_to_left = ComputeAngleFromTwoLinesVector2D(cur_pnt_normal, left_fun_vertical);
				float angle_to_right = ComputeAngleFromTwoLinesVector2D(cur_pnt_normal, right_fun_vertical);
				
				if (angle_to_left > ANGLE_VERTICAL)
				{
					angle_to_left = ANGLE_HALF_CIRCLE - angle_to_left;
				}
				if (angle_to_right > ANGLE_VERTICAL)
				{
					angle_to_right = ANGLE_HALF_CIRCLE - angle_to_right;
				}

				/*if (std::abs(angle_to_left) > std::abs(angle_to_right))
				{
					lr_pnts_idxs_to_labels[i].second = 1;
				}
				else
				{
					lr_pnts_idxs_to_labels[i].second = -1;
				}*/
				std::cerr << "to do next..." << std::endl;
			}
			/*bool lr_flag = false;
			int split_cur_idx_new = -1;
			for (int i = initial_lpnts_idxs_end; i < initial_lpnts_idxs_end + initial_lr_pnts_remained_num; ++i)
			{
				if (lr_pnts_idxs_to_labels[i].second == 1)
				{
					lr_flag = true;
					split_cur_idx_new = i;
					break;
				}
			}
			for (int i = split_cur_idx_new; i < initial_lpnts_idxs_end + initial_lr_pnts_remained_num; ++i)
			{
				lr_pnts_idxs_to_labels[i].second = 1;
			}*/

		}
		else
		{
			int cur_split_pnt_idx = initial_lpnts_idxs_end;
			//std::cerr << "initial cur_split_pnt_idx: " << cur_split_pnt_idx << std::endl;
			//std::cerr << "left_fun_vertical: " << left_fun_vertical.transpose() << std::endl;
			//std::cerr << "left_fun_vertical norm: " << left_fun_vertical.norm() << std::endl;
			//std::cerr << "right_fun_vertical: " << right_fun_vertical.transpose() << std::endl;
			//std::cerr << "right_fun_vertical norm: " << right_fun_vertical.norm() << std::endl;

			std::cerr << "-----------------" << std::endl;
			for (int i = 0; i < lr_pnts_idxs_to_labels.size(); ++i)
			{
				std::cerr << lr_pnts_idxs_to_labels[i].first << " " << lr_pnts_idxs_to_labels[i].second << ", ";
			}
			std::cerr << std::endl;
			std::cerr << "-----------------" << std::endl;

			bool stop_process = false;
			bool left_growth = false;
			bool right_growth = false;
			do
			{
				int cur_idx = lr_pnts_idxs_to_labels[cur_split_pnt_idx].first;
				Eigen::Vector2f cur_pnt_normal = pnts_in[cur_idx].normal_;
				Eigen::Vector2f cur_pnt_pos = pnts_in[cur_idx].pos_;
				//angle
				std::cerr << "cur_pnt_normal: " << cur_pnt_normal.transpose() << std::endl;
				std::cerr << "cur_pnt_normal norm: " << cur_pnt_normal.norm() << std::endl;
				float angle_to_left = ComputeAngleFromTwoLinesVector2D(cur_pnt_normal, left_fun_vertical);
				float angle_to_right = ComputeAngleFromTwoLinesVector2D(cur_pnt_normal, right_fun_vertical);
				if (angle_to_left > ANGLE_VERTICAL)
				{
					angle_to_left = ANGLE_HALF_CIRCLE - angle_to_left;
				}
				if (angle_to_right > ANGLE_VERTICAL)
				{
					angle_to_right = ANGLE_HALF_CIRCLE - angle_to_right;
				}
				std::cerr << "angle_to_left, angle_to_right: " << angle_to_left << ", " << angle_to_right << std::endl;
				if (std::abs(angle_to_left) > std::abs(angle_to_right))
				{
					right_growth = true;
					std::cerr << "right grow" << std::endl;
					lr_pnts_idxs_to_labels[cur_split_pnt_idx].second = 1;
					--cur_split_pnt_idx;
				}
				else
				{
					left_growth = true;
					std::cerr << "left grow" << std::endl;
					lr_pnts_idxs_to_labels[cur_split_pnt_idx].second = -1;
					++cur_split_pnt_idx;
				}
				if (left_growth && right_growth)
				{
					stop_process = true;
				}
				if (cur_split_pnt_idx <= 0 || cur_split_pnt_idx >= lr_pnts_idxs_to_labels.size())
				{
					stop_process = true;
				}

			} while (!stop_process);
			std::cerr << "cur_split_pnt_idx: " << cur_split_pnt_idx << std::endl;

			//
			int split_cur_idx = -1;
			for (int i = 0; i < lr_pnts_idxs_to_labels.size(); ++i)
			{
				if (lr_pnts_idxs_to_labels[i].second == 1)
				{
					split_cur_idx = i;
					break;
				}
			}
			if (split_cur_idx != -1)
			{
				for (int i = split_cur_idx; i < lr_pnts_idxs_to_labels.size(); ++i)
				{
					lr_pnts_idxs_to_labels[i].second = 1;
				}
			}
		}
		
		HWLineToPntsIdxs tmp_left_pntidx_new;
		HWLineToPntsIdxs tmp_right_pntidx_new;
		//cur_lpnts_end_idx
		for (int i = 0; i < lr_pnts_idxs_to_labels.size(); ++i)
		{
			if (lr_pnts_idxs_to_labels[i].second == -1)
			{
				tmp_left_pntidx_new.associated_pnts_idxs_.emplace_back(lr_pnts_idxs_to_labels[i].first);
			}
			if (lr_pnts_idxs_to_labels[i].second == 1)
			{
				tmp_right_pntidx_new.associated_pnts_idxs_.emplace_back(lr_pnts_idxs_to_labels[i].first);
			}
		}

		/*std::cerr << "-----------------" << std::endl;
		for (int i = 0; i < lr_pnts_idxs_to_labels.size(); ++i)
		{
			std::cerr << lr_pnts_idxs_to_labels[i].first << " " << lr_pnts_idxs_to_labels[i].second << ", ";
		}
		std::cerr << std::endl;
		std::cerr << "-----------------" << std::endl;*/

		ComputeEnergyNodeMeFunFromItsIdxs(pnts_in, tmp_left_pntidx_new);
		ComputeEnergyNodeMeFunFromItsIdxs(pnts_in, tmp_right_pntidx_new);

		if (tmp_left_pntidx_new.associated_pnts_idxs_.empty())
		{
			fun_coeffs_opti.clear();
			if (r_idx == 0)
			{
				fun_coeffs_opti.emplace_back(tmp_right_pntidx_new);
				for (int i = 1; i < l_idx; ++i)
				{
					fun_coeffs_opti.emplace_back(fun_coeffs[i]);
				}
			}
			else
			{
				for (int i = 0; i < l_idx; ++i)
				{
					fun_coeffs_opti.emplace_back(fun_coeffs[i]);
				}
				fun_coeffs_opti.emplace_back(tmp_right_pntidx_new);
				for (int i = r_idx + 1; i < fun_coeffs.size(); ++i)
				{
					fun_coeffs_opti.emplace_back(fun_coeffs[i]);
				}
			}
		}
		else if (tmp_right_pntidx_new.associated_pnts_idxs_.empty())
		{
			fun_coeffs_opti.clear();
			if (r_idx == 0)
			{
				for (int i = 1; i < l_idx; ++i)
				{
					fun_coeffs_opti.emplace_back(fun_coeffs[i]);
				}
				fun_coeffs_opti.emplace_back(tmp_left_pntidx_new);
			}
			else
			{
				for (int i = 0; i < l_idx; ++i)
				{
					fun_coeffs_opti.emplace_back(fun_coeffs[i]);
				}
				fun_coeffs_opti.emplace_back(tmp_left_pntidx_new);
				for (int i = r_idx + 1; i < fun_coeffs.size(); ++i)
				{
					fun_coeffs_opti.emplace_back(fun_coeffs[i]);
				}
			}
		}
		else
		{
			//tmp_left_pntidx_new.line_fun_ = 
			fun_coeffs_opti = fun_coeffs;
			fun_coeffs_opti[l_idx] = tmp_left_pntidx_new;
			fun_coeffs_opti[r_idx] = tmp_right_pntidx_new;
		}
		remained_pnts_idxs_opti.clear();
		//get the remained line idxs
		ComputeAllLinesFunsRemainedPntsIdxsFromPnts(pnts_in, fun_coeffs_opti, remained_pnts_idxs_opti);
		//compute the energy...
		float all_energy_value = ComputeAllEnergyValueFromPolygonExtractedPnts(pnts_in, fun_coeffs_opti, remained_pnts_idxs_opti);
		return all_energy_value;
	}

	float HWPlane::PolygonLineSplitIntoTwoPolygonLines(const std::vector<HWOptiPnt2D>& pnts_in, const std::vector<HWLineToPntsIdxs>& fun_coeffs, const std::vector<int>& remained_pnts_idxs,
		int p_idx, std::vector<HWLineToPntsIdxs>& fun_coeffs_opti, std::vector<int>& remained_pnts_idxs_opti)
	{
		HWLineToPntsIdxs pfun_coeff = fun_coeffs[p_idx];
		std::vector<int> ppnts_idxs = pfun_coeff.associated_pnts_idxs_;
		//check it has remained idx
		int pnts_num = static_cast<int>(pnts_in.size());
		//cur split idx
		int p_pnts_idxs_num = static_cast<int>(ppnts_idxs.size());
		if (p_pnts_idxs_num > 3)
		{
			HWLineToPntsIdxs tmp_left_pntidx_new;
			HWLineToPntsIdxs tmp_right_pntidx_new;
			//exchange idxs
			for (int i = 0; i <= p_pnts_idxs_num / 2; ++i)
			{
				int idx = ppnts_idxs[i];
				tmp_left_pntidx_new.associated_pnts_idxs_.emplace_back(idx);
			}
			for (int i = p_pnts_idxs_num / 2 + 1; i < p_pnts_idxs_num; ++i)
			{
				int idx = ppnts_idxs[i];
				tmp_right_pntidx_new.associated_pnts_idxs_.emplace_back(idx);
			}
			ComputeEnergyNodeMeFunFromItsIdxs(pnts_in, tmp_left_pntidx_new);
			ComputeEnergyNodeMeFunFromItsIdxs(pnts_in, tmp_right_pntidx_new);

			std::vector<HWLineToPntsIdxs> fun_coeffs_opti_mid;
			for (int i = 0; i < p_idx; ++i)
			{
				fun_coeffs_opti_mid.emplace_back(fun_coeffs[i]);
			}
			fun_coeffs_opti_mid.emplace_back(tmp_left_pntidx_new);
			fun_coeffs_opti_mid.emplace_back(tmp_right_pntidx_new);
			for (int i = p_idx + 1; i < fun_coeffs.size(); ++i)
			{
				fun_coeffs_opti_mid.emplace_back(fun_coeffs[i]);
			}
			std::vector<int> remained_pnts_idxs_mid;
			ComputeAllLinesFunsRemainedPntsIdxsFromPnts(pnts_in, fun_coeffs_opti_mid, remained_pnts_idxs_mid);

			int lidx = p_idx;
			int ridx = p_idx + 1;
			float all_energy_function_value1 = PolygonLinesExchangePolygonLinePnts(pnts_in, 
				fun_coeffs_opti_mid, remained_pnts_idxs_mid, lidx, ridx, fun_coeffs_opti, remained_pnts_idxs_opti);
			/*float all_energy_function_value = ComputeAllEnergyValueFromPolygonExtractedPnts(pnts_in, fun_coeffs_opti_mid, remained_pnts_idxs_mid);
			fun_coeffs_opti = fun_coeffs_opti_mid;
			remained_pnts_idxs_opti = remained_pnts_idxs_mid;*/
			return all_energy_function_value1;
		}
		else
		{
			fun_coeffs_opti = fun_coeffs;
			return KMAX_FLOAT_LIMIT_VALUE;
		}
	}

	float HWPlane::PolygonLineMergeTwoPolygonIntoPolygonLine(const std::vector<HWOptiPnt2D>& pnts_in, const std::vector<HWLineToPntsIdxs>& fun_coeffs, const std::vector<int>& remained_pnts_idxs,
		int l_idx, int r_idx, std::vector<HWLineToPntsIdxs>& fun_coeffs_opti, std::vector<int>& remained_pnts_idxs_opti)
	{
		std::vector<int> neighbor_pnts_remained_idxs;
		ComputeNeighborLinesFunsRemainedPntsIdxs(pnts_in, fun_coeffs, l_idx, r_idx, neighbor_pnts_remained_idxs);
		HWLineToPntsIdxs tmp_pntidx_new;
		std::vector<int> merge_pnts_idxs;
		HWLineToPntsIdxs lfun_coeff = fun_coeffs[l_idx];
		HWLineToPntsIdxs rfun_coeff = fun_coeffs[r_idx];
		for (int i = 0; i < lfun_coeff.associated_pnts_idxs_.size(); ++i)
		{
			merge_pnts_idxs.emplace_back(lfun_coeff.associated_pnts_idxs_[i]);
		}
		for (int i = 0; i < neighbor_pnts_remained_idxs.size(); ++i)
		{
			merge_pnts_idxs.emplace_back(neighbor_pnts_remained_idxs[i]);
		}
		for (int i = 0; i < rfun_coeff.associated_pnts_idxs_.size(); ++i)
		{
			merge_pnts_idxs.emplace_back(rfun_coeff.associated_pnts_idxs_[i]);
		}
		Eigen::Vector3f merge_fun_coeff;
		std::vector<Eigen::Vector2f> merge_pnts_pos;
		for (int i = 0; i < merge_pnts_idxs.size(); ++i)
		{
			int idx = merge_pnts_idxs[i];
			Eigen::Vector2f tmp_pnt = pnts_in[idx].pos_;
			merge_pnts_pos.emplace_back(tmp_pnt);
		}
		FittingLine2DFromPnts2D(merge_pnts_pos, merge_fun_coeff);
		tmp_pntidx_new.line_fun_ = merge_fun_coeff;
		tmp_pntidx_new.associated_pnts_idxs_ = merge_pnts_idxs;
		fun_coeffs_opti.clear();
		for (int i = 0; i < l_idx; ++i)
		{
			fun_coeffs_opti.emplace_back(fun_coeffs[i]);
		}
		//add the a line between the l_idx, r_idx
		fun_coeffs_opti.emplace_back(tmp_pntidx_new);
		if (l_idx < r_idx)
		{
			for (int i = r_idx + 1; i < fun_coeffs.size(); ++i)
			{
				fun_coeffs_opti.emplace_back(fun_coeffs[i]);
			}
		}
		remained_pnts_idxs_opti.clear();
		//get the remained line idxs
		ComputeAllLinesFunsRemainedPntsIdxsFromPnts(pnts_in, fun_coeffs_opti, remained_pnts_idxs_opti);
		//compute the energy...
		float all_energy_value = ComputeAllEnergyValueFromPolygonExtractedPnts(pnts_in, fun_coeffs_opti, remained_pnts_idxs_opti);
		return all_energy_value;
	}

	float HWPlane::PolygonLineInsertLineIntoTwoPolygonLine(const std::vector<HWOptiPnt2D>& pnts_in, const std::vector<HWLineToPntsIdxs>& fun_coeffs, 
		const std::vector<int>& remained_pnts_idxs, int l_idx, int r_idx, 
		std::vector<HWLineToPntsIdxs>& fun_coeffs_opti, std::vector<int>& remained_pnts_idxs_opti)
	{
		std::vector<int> neighbor_pnts_lr_remained_idxs;
		ComputeNeighborLinesFunsRemainedPntsIdxs(pnts_in, fun_coeffs, l_idx, r_idx, neighbor_pnts_lr_remained_idxs);
		int remained_idxs_num = static_cast<int>(neighbor_pnts_lr_remained_idxs.size());
		if (remained_idxs_num < 1)
		{
			return KMAX_FLOAT_LIMIT_VALUE;
		}
		if (remained_idxs_num == 1)
		{
			std::cerr << "to do next..." << std::endl;
			return KMAX_FLOAT_LIMIT_VALUE;
		}
		std::vector<Eigen::Vector2f> remained_idxs_pnts;
		for (int i = 0; i < neighbor_pnts_lr_remained_idxs.size(); ++i)
		{
			int idx = neighbor_pnts_lr_remained_idxs[i];
			Eigen::Vector2f tmp_pos = pnts_in[idx].pos_;
			remained_idxs_pnts.emplace_back(tmp_pos);
		}
		Eigen::Vector3f fun_coeff;
		FittingLine2DFromPnts2D(remained_idxs_pnts, fun_coeff);
		/*std::cerr << "PolygonLineInsertLineIntoTwoPolygonLine remained_idxs_pnts number: " << remained_idxs_pnts.size() << std::endl;
		std::string path_test = filename_ + "_insert_remained_pnts.obj";
		SavePnts2dIntoPnts3DOBJ(path_test, remained_idxs_pnts);*/
		HWLineToPntsIdxs tmp_pntidx_new;
		tmp_pntidx_new.line_fun_ = fun_coeff;
		tmp_pntidx_new.associated_pnts_idxs_ = neighbor_pnts_lr_remained_idxs;
		fun_coeffs_opti.clear();
		for (int i = 0; i <= l_idx; ++i)
		{
			fun_coeffs_opti.emplace_back(fun_coeffs[i]);
		}
		//add the a line between the l_idx, r_idx
		fun_coeffs_opti.emplace_back(tmp_pntidx_new);
		if (l_idx < r_idx)
		{
			for (int i = r_idx; i < fun_coeffs.size(); ++i)
			{
				fun_coeffs_opti.emplace_back(fun_coeffs[i]);
			}
		}
		remained_pnts_idxs_opti.clear();
		//get the remained line idxs
		ComputeAllLinesFunsRemainedPntsIdxsFromPnts(pnts_in, fun_coeffs_opti, remained_pnts_idxs_opti);

		//compute the energy...
		float all_energy_value = ComputeAllEnergyValueFromPolygonExtractedPnts(pnts_in, fun_coeffs_opti, remained_pnts_idxs_opti);
		return all_energy_value;
	}

	bool HWPlane::ComparePolygonNeighborEnergyValueDecrease(const PolygonNeighborEnergy& a, const PolygonNeighborEnergy& b)
	{
		return a.energy_value > b.energy_value;
	}

	bool HWPlane::ComparePolygonNeighborEnergyValueIncrease(const PolygonNeighborEnergy& a, const PolygonNeighborEnergy& b)
	{
		return a.energy_value < b.energy_value;
	}

	void HWPlane::InnerPolygonExtraction()
	{
		if (inner_polygon_num_ == 0) return;
		inner_corner_pnts_.resize(inner_polygon_num_);
		inner_corner_pnts_3d_.resize(inner_polygon_num_);
		inner_edges_.resize(inner_polygon_num_);
		std::cout << "inner polygon extraction" << std::endl;
		for (int i = 0; i < inner_polygon_num_; i++) {
			std::cout << i << std::endl;
			SingleInnerPolygonExtraction(i);
		}
		std::vector<std::vector<float2>> inner_corner_pnts;
		std::vector<std::vector<float3>> inner_corner_pnts_3d;
		std::vector<std::vector<Eigen::Vector2f>> inner_edges;
		for (int i = 0; i < inner_polygon_num_; i++) {
			if (!inner_corner_pnts_[i].empty()) {
				inner_corner_pnts.push_back(inner_corner_pnts_[i]);
				inner_corner_pnts_3d.push_back(inner_corner_pnts_3d_[i]);
				inner_edges.push_back(inner_edges_[i]);
			}
		}
		inner_corner_pnts_ = inner_corner_pnts;
		inner_corner_pnts_3d_ = inner_corner_pnts_3d_;
		inner_edges_ = inner_edges;
		inner_polygon_num_ = inner_edges_.size();

	}

	void HWPlane::SingleInnerPolygonExtraction(int index)
	{
		double theta_max = params_.max_theta;
		if (theta_max == 0) {
			theta_max = 10;
			initial_params_.max_theta = 10;
		}
		std::cout << "theta_max = " << theta_max << std::endl;
		//corner_pnts.clear();
		//corner_pnts_3d.clear();
		auto& corner_pnts = inner_corner_pnts_[index];
		auto& corner_pnts_3d = inner_corner_pnts_3d_[index];
		corner_pnts.clear();
		corner_pnts_3d.clear();

		int pnts_pos_num = inner_edge_pnts_pos_num_[index];
		auto& pnts_normal = inner_edge_pnts_normal_[index];
		auto& pnts_pos = inner_edge_pnts_pos_[index];
		auto& edges = inner_edges_[index];

		std::vector<bool> is_edge_points(pnts_pos_num, false);
		std::vector<std::vector<int>> edge_idx;

		for (int i = 0; i < pnts_pos_num; i++) {
			int last_idx = (i + pnts_pos_num - 1) % pnts_pos_num;
			int next_idx = (i + pnts_pos_num + 1) % pnts_pos_num;
			float2 n = pnts_normal[i];
			float2 n1 = pnts_normal[last_idx];
			float2 n2 = pnts_normal[next_idx];
			if (GetTheta(n.x, n.y, n1.x, n1.y) < theta_max &&
				GetTheta(n.x, n.y, n2.x, n2.y) < theta_max) {
				is_edge_points[i] = true;
			}
		}
		std::cout << "a" << std::endl;

		int first_edge_point_idx = 0;
		//找到第一个edge point
		while (is_edge_points[first_edge_point_idx]) first_edge_point_idx++;
		while (!is_edge_points[first_edge_point_idx]) first_edge_point_idx++;

		bool has_thin_edge = false;
		edges.clear();
		int curr_idx = first_edge_point_idx;
		while (true) {
			std::vector<int> curr_edge;
			while (is_edge_points[curr_idx]) {
				curr_edge.push_back(curr_idx);
				int old_idx = curr_idx;
				curr_idx = (curr_idx + 1) % pnts_pos_num;

				if (curr_edge.size() < 2) continue;
				Eigen::Vector2f coeff = FittingLine(pnts_pos, curr_edge);
				if (abs(coeff(0)) < 1E-6 && abs(coeff(1)) < 1E-6) continue;

				float max = -FLT_MAX, min = FLT_MAX;
				for (int k = 0; k < curr_edge.size(); k++) {
					int idx = curr_edge[k];
					float d = PointLineDist(pnts_pos[idx], coeff);
					if (d > max)
						max = d;
					if (d < min)
						min = d;
				}
				double region_diameter = max - min;
				if (region_diameter > diameter_max_) {
					//edge_pnts_smooth_regions_[i].pop_back();
					curr_edge.pop_back();
					curr_idx = old_idx;
					break;
				}
			}
			//如果这条线段的点数大于5，则拟合这条线段
			if (curr_edge.size() > 5) {
				Eigen::Vector2f coeff = FittingLine(pnts_pos, curr_edge);

				float2 start, end, mid;
				start = pnts_pos[curr_edge[0]];
				mid = pnts_pos[curr_edge[curr_edge.size() / 2]];
				end = pnts_pos[curr_edge.back()];

				float2 start_proj, mid_proj, end_proj;
				start_proj = ProjectToLine(start, coeff);
				mid_proj = ProjectToLine(mid, coeff);
				end_proj = ProjectToLine(end, coeff);
				float dist = PointPointDist(start_proj, end_proj);
				float dist1 = PointPointDist(start_proj, mid);
				float dist2 = PointPointDist(mid, end_proj);
				//这里没看懂
				//why?
				if (dist < max(dist1, dist2)) {
					has_thin_edge = true;
				}
				edges.emplace_back(coeff);
				edge_idx.push_back(curr_edge);
			}

			while (!is_edge_points[curr_idx]) {
				curr_idx = (curr_idx + 1) % pnts_pos_num;
			}

			//如果又回到起点，说明全部遍历了一遍
			if (curr_idx == first_edge_point_idx) break;
		}
		std::cout << "b" << std::endl;

		int edge_num = edges.size();

		if (edges.size() <= 1)
			return;
		if (edges.size() > 1 && has_thin_edge) {
			printf("has_thin_edge!!!!!!!!!!!!!!!!!!!!\n");
		}
		if (edges.size() == 2) {
			//int this_first = first_lasts[edge_idx_[0]].x, this_last = first_lasts[edge_idx_[0]].y;
			//int next_first = first_lasts[edge_idx_[1]].x, next_last = first_lasts[edge_idx_[1]].y;
			int this_first = edge_idx[0][0];
			int this_last = edge_idx[0].back();
			int next_first = edge_idx[1][0];
			int next_last = edge_idx[1].back();
			float2 p1 = ProjectToLine(pnts_pos[this_first], edges[0]);
			float2 p2 = ProjectToLine(pnts_pos[this_last], edges[0]);
			float2 p3 = ProjectToLine(pnts_pos[next_first], edges[1]);
			float2 p4 = ProjectToLine(pnts_pos[next_last], edges[1]);
			corner_pnts.emplace_back(p1);
			corner_pnts.emplace_back(p2);
			corner_pnts.emplace_back(p3);
			corner_pnts.emplace_back(p4);
			for (int i = 0; i < corner_pnts.size(); i++) {
				if (IsSelfIntersection(corner_pnts, i, corner_pnts[i])) {
					corner_pnts[1] = p3;
					corner_pnts[2] = p2;
					break;
				}
			}
		}
		else {
			for (int i = 0; i < edges.size(); i++) {
				int next_idx = (i + edges.size() + 1) % edges.size();
				//当前边和下一条边之间的夹角
				float theta = GetTheta(edges[i](0), edges[i](1), edges[next_idx](0), edges[next_idx](1));
				//当前边的最后一点，和下一条边的第一个点
				//int this_last = first_lasts[edge_idx_[i]].y, next_first = first_lasts[edge_idx_[next_idx]].x;
				int this_last = edge_idx[i].back();
				int next_first = edge_idx[next_idx][0];
				float2 p = GetIntersectionPoint(edges[i], edges[next_idx]);
				//float2 p1 = pnts_pos[this_last];
				//float2 p2 = pnts_pos[next_first];
				float2 p1 = ProjectToLine(pnts_pos[this_last], edges[i]);
				float2 p2 = ProjectToLine(pnts_pos[next_first], edges[next_idx]);
				float value = 1;
				//bool self_intersection = false;
				if (corner_pnts.size() > 0) {
					float2 p_last = corner_pnts.back();
					float2 edge = p - p_last;
					value = dot(edge, p1 - p_last);
				}
				//如果当前边和下一条边之间的夹角大于theta_max度，且p和p1在当前最后一个顶点的同一侧，则将p作为新的顶点加入到顶点向量中
				if (!isinf(p.x) && !isinf(p.y) && theta > theta_max && value > 0) {//|| PointPointDist(p, pnts_pos[this_last]) < 0.1 || PointPointDist(p, pnts_pos[next_first]) < 0.1) {
					corner_pnts.emplace_back(p);
					std::cout << "p: " << p.x << " " << p.y << "\n";
				}
				//否则，将p1和p2作为两个顶点加入到顶点向量中
				else {
					/*std::cout << "this_last: " << this_last << "\n";
					std::cout << "next_first: " << next_first << "\n";
					std::cout << "p1: " << p1.x << " " << p1.y << "\n";
					std::cout << "p2: " << p2.x << " " << p2.y << "\n";*/
					corner_pnts.emplace_back(p1);
					corner_pnts.emplace_back(p2);
				}
			}
		}
		std::cout << "before deleting near points, corner_pnts.size() = " << corner_pnts.size() << std::endl;
		std::stringstream ss;
		ss << index;
		std::string str_index;
		ss >> str_index;
		std::ofstream fh(filename_ + "inner_contour" + str_index + ".obj");
		for (int j = 0; j < corner_pnts.size(); j++) {
			float2 start_pnt = corner_pnts[j];
			fh << "v " << start_pnt.x << " " << start_pnt.y << " " << 1.0 << std::endl;
		}
		for (int j = 0; j < corner_pnts.size() - 1; j++) {
			fh << "l " << j + 1 << " " << j + 2 << "\n";
		}
		fh << "l " << corner_pnts.size() << " " << 1 << "\n";
		fh.close();

		//去除距离非常近的相邻顶点
		//取它们之间的中点代替它们
		double min_distance = params_.min_corner_points_distance;
		if (min_distance == 0) {
			min_distance = ave_alpha_edge_len_;
			initial_params_.min_corner_points_distance = ave_alpha_edge_len_;
		}
		std::cout << "min_distance = " << min_distance << std::endl;
		std::vector<float2> temp, temp2;
		for (int i = 0; i < corner_pnts.size(); i++) {
			if (i == 0)
				temp.emplace_back(corner_pnts[i]);
			else {
				float2 p = temp.back();
				if (PointPointDist(p, corner_pnts[i]) < min_distance)
					temp.back() = (p + corner_pnts[i]) / 2;
				else
					temp.emplace_back(corner_pnts[i]);
			}
		}
		if (PointPointDist(temp[0], temp.back()) < min_distance) {
			temp[0] = (temp[0] + temp.back()) / 2;
			temp.pop_back();
		}
		std::cout << "after deleting near points, corner_pnts.size() = " << corner_pnts.size() << std::endl;

		//std::cout << "temp: " << temp.size() << "\n";
		//去除三点共线的情况
		//如果有三点共线，去掉中间的点
		for (int i = 0; i < temp.size(); i++) {
			int last = (i - 1 + temp.size()) % temp.size();
			int next = (i + 1 + temp.size()) % temp.size();
			//if (PointLineSegmentDist(temp[i], temp[last], temp[next]) > 0.01)
			if (PointLineSegmentDist(temp[i], temp[last], temp[next]) > 0.01)
				temp2.emplace_back(temp[i]);
			//float2 l1 = temp[i] - temp[last];
			//float2 l2 = temp[next] - temp[i];
			//float theta = GetTheta(l1.x, l1.y, l2.x, l2.y);
			//if (theta > 10) temp2.emplace_back(temp[i]);
		}

		//如果最终剩下的顶点少于3个，则直接返回
		if (temp2.size() > 2)
			corner_pnts = temp2;
		else {
			corner_pnts.clear();
			return;
		}

		std::cout << "after deleting colinear points, corner_pnts.size() = " << corner_pnts.size() << std::endl;

		//解决多边形的边自相交的问题
		bool self_intersection = true;
		while (self_intersection) {
			self_intersection = false;
			float delete_edge_length = FLT_MAX;
			int delete_edge_idx;
			//先解决（与顶点相邻的两条边）与（多边形的另外两条边）相交的情况
			//直接删除这个顶点
			for (int i = 0; i < corner_pnts.size(); i++) {
				if (IsSelfIntersection(corner_pnts, i, corner_pnts[i]) == 2) {
					std::vector<float2> temp(corner_pnts.begin() + i + 1, corner_pnts.end());
					corner_pnts.resize(i);
					corner_pnts.insert(corner_pnts.begin(), temp.begin(), temp.end());
					self_intersection = true;
					break;
				}
			}
			if (self_intersection)
				continue;
			//再解决相交一次的情况
			//直接删除这个顶点
			for (int i = 0; i < corner_pnts.size(); i++) {
				if (IsSelfIntersection(corner_pnts, i, corner_pnts[i]) == 1) {
					std::vector<float2> temp(corner_pnts.begin() + i + 1, corner_pnts.end());
					corner_pnts.resize(i);
					corner_pnts.insert(corner_pnts.begin(), temp.begin(), temp.end());
					self_intersection = true;
					break;
				}
			}
		}

		std::cout << "after deleting intersecting points, corner_pnts.size() = " << corner_pnts.size() << std::endl;

		float2 edge1 = corner_pnts[1] - corner_pnts[0];
		float2 edge2 = corner_pnts[2] - corner_pnts[1];
		if (edge1.x * edge2.y - edge1.y * edge2.x == 0) {
			std::cout << "edge1: " << edge1.x << " " << edge1.y << "\n";
			std::cout << "edge2: " << edge2.x << " " << edge2.y << "\n";
			system("pause");
		}

		//解决定点确定的多边形法向量反向的问题
		//将顶点向量中的点逆序排列
		//if (edge1.x*edge2.y - edge1.y*edge2.x > 0 ^ IsConcaveVertex(1) == 1) {
		//	std::vector<float2> corner_pnts_inverse;
		//	for (int i = corner_pnts.size() - 1; i >= 0; i--) {
		//		corner_pnts_inverse.emplace_back(corner_pnts[i]);
		//	}
		//	corner_pnts = corner_pnts_inverse;
		//}

		std::cout << "corner_pnts_3d.emplace_back" << std::endl;
		for (int i = 0; i < corner_pnts.size(); i++) {
			Eigen::Vector4f pnt(corner_pnts[i].x, corner_pnts[i].y, 0.0, 1.0);
			Eigen::Vector4f transform_pnt = plane_to_world_ * pnt;
			corner_pnts_3d.emplace_back(make_float3(transform_pnt(0), transform_pnt(1), transform_pnt(2)));
		}

	}

	void HWPlane::CalcMinDists()
	{
		//std::cout << "begin CalcMinDists" << std::endl;
		min_dists_.clear();
		int num = corner_pnts_.size();
		for (int i = 0; i < num; i++) {
			float2 p = corner_pnts_[i];
			float min_dist = FLT_MAX;
			for (int j = i + 1; j < i + num - 1; j++) {
				int idx = j%num;
				int idx_next = (j + 1) % num;
				float2 p1 = corner_pnts_[idx], p2 = corner_pnts_[idx_next];
				float dist = PointLineSegmentDist(p, p1, p2);
				if (dist < min_dist)
					min_dist = dist;
			}
			//std::cout << "min_dist: " << min_dist << "\n";
			min_dists_.emplace_back(min_dist);
		}
		//std::cout << "end CalcMinDists" << std::endl;
	}

	bool HWPlane::DoPointMoving(int idx, Eigen::Vector3f& dest, float max_dist)
	{
		Eigen::Vector4f projection_point = world_to_plane_ * Eigen::Vector4f(dest(0), dest(1), dest(2), 1.0f);
		float2 proj = make_float2(projection_point(0), projection_point(1));
		float2 original = corner_pnts_[idx];

		if (std::isnan(proj.x) || std::isnan(proj.y)) return false;

		int idx_last = (idx - corner_pnts_.size() - 1) % corner_pnts_.size();
		int idx_next = (idx + 1) % corner_pnts_.size();

		float2 p_last = corner_pnts_[idx_last];
		float2 p_next = corner_pnts_[idx_next];
		Eigen::Vector2f edge1 = FittingLine(p_last, proj);
		Eigen::Vector2f edge2 = FittingLine(proj, p_next);

		for (int i = 0; i < corner_pnts_.size(); i++) {
			int start_idx = i;
			int end_idx = (i + 1) % corner_pnts_.size();

			if (start_idx == idx || end_idx == idx) continue;

			float2 start_pnt = corner_pnts_[start_idx];
			float2 end_pnt = corner_pnts_[end_idx];
			Eigen::Vector2f curr_edge = FittingLine(start_pnt, end_pnt);

			float2 a = start_pnt - proj;
			float2 b = end_pnt - proj;

			float dist = abs(PointLineDist(proj, curr_edge));

			if (dist < 0.001 && dot(a, b) <= 0) return false;
		}

		float2 src2dst = proj - original;
		float dist = sqrt(dot(src2dst, src2dst));
		if (dist > max_dist) {
			printf("move point failed due to max_dist\n");
			return false;
		}

		corner_pnts_[idx] = proj;
		if (IsSelfIntersection(idx, corner_pnts_[idx]) > 0) {
			corner_pnts_[idx] = original;
			printf("move point failed due to self intersection\n");
			return false;
		}
		corner_pnts_[idx] = original;

		Eigen::Vector4f transform_pnt = plane_to_world_ * Eigen::Vector4f(proj.x, proj.y, 0, 1);
		if (std::isnan(transform_pnt(0)) || std::isnan(transform_pnt(1)) || std::isnan(transform_pnt(2))) {
			printf("move point failed due to nan\n");
			return false;
		}
		corner_pnts_[idx] = proj;
		corner_pnts_3d_[idx] = make_float3(transform_pnt(0), transform_pnt(1), transform_pnt(2));
		UpdateInformation();
		return true;
	}

	bool HWPlane::DoPointMoving(int idx, Eigen::Vector3f & L_dir, Eigen::Vector3f & L_point, float max_dist)
	{
		corner_pnts_last_state_ = corner_pnts_;
		corner_angels_last_state_ = corner_angels_;
		min_dists_last_state_ = min_dists_;
		corner_pnts_3d_last_state_ = corner_pnts_3d_;
		triangles_idx_last_state_ = triangles_idx_;

		float2 original = corner_pnts_[idx];

		float2 proj = ProjToLine3D(idx, L_dir, L_point);
		#ifdef LIULINGFEI
		std::cout << "proj: " << proj.x << " " << proj.y << "\n";
		#endif
		if (std::isnan(proj.x) || std::isnan(proj.y)) return false;

		int idx_last = (idx - corner_pnts_.size() - 1) % corner_pnts_.size();
		int idx_next = (idx + 1) % corner_pnts_.size();

		float2 p_last = corner_pnts_[idx_last];
		float2 p_next = corner_pnts_[idx_next];
		Eigen::Vector2f edge1 = FittingLine(p_last, proj);
		Eigen::Vector2f edge2 = FittingLine(proj, p_next);

		for (int i = 0; i < corner_pnts_.size(); i++) {
			int start_idx = i;
			int end_idx = (i + 1) % corner_pnts_.size();

			if (start_idx == idx || end_idx == idx) continue;

			float2 start_pnt = corner_pnts_[start_idx];
			float2 end_pnt = corner_pnts_[end_idx];
			Eigen::Vector2f curr_edge = FittingLine(start_pnt, end_pnt);

			float2 a = start_pnt - proj;
			float2 b = end_pnt - proj;

			float dist = abs(PointLineDist(proj, curr_edge));

			if (dist < 0.001 && dot(a, b) <= 0) return false;
		}

		float2 src2dst = proj - original;
		float dist = sqrt(dot(src2dst, src2dst));
		if (dist > max_dist) {
			printf("failed due to dist > max_dist\n");
			return false;
		}

		corner_pnts_[idx] = proj;
		if (IsSelfIntersection(idx, corner_pnts_[idx]) > 0) {
			corner_pnts_[idx] = original;
			printf("failed due to self intersection\n");
			return false;
		}
		corner_pnts_[idx] = original;

		Eigen::Vector4f transform_pnt = plane_to_world_*Eigen::Vector4f(proj.x, proj.y, 0, 1);
		if (std::isnan(transform_pnt(0)) || std::isnan(transform_pnt(1)) || std::isnan(transform_pnt(2))) return false;
		corner_pnts_[idx] = proj;
		corner_pnts_3d_[idx] = make_float3(transform_pnt(0), transform_pnt(1), transform_pnt(2));
		UpdateInformation();
		return true;
	}

	void HWPlane::DoPointMoving(int idx1, int idx2, Eigen::Vector3f & L_dir, Eigen::Vector3f & L_point)
	{
		corner_pnts_last_state_ = corner_pnts_;
		corner_angels_last_state_ = corner_angels_;
		min_dists_last_state_ = min_dists_;
		corner_pnts_3d_last_state_ = corner_pnts_3d_;
		triangles_idx_last_state_ = triangles_idx_;
		float2 proj = ProjToLine3D(idx1, L_dir, L_point);
		corner_pnts_[idx1] = proj;
		Eigen::Vector4f transform_pnt = plane_to_world_*Eigen::Vector4f(proj.x, proj.y, 0, 1);
		corner_pnts_3d_[idx1] = make_float3(transform_pnt(0), transform_pnt(1), transform_pnt(2));
		proj = ProjToLine3D(idx2, L_dir, L_point);
		corner_pnts_[idx2] = proj;
		transform_pnt = plane_to_world_*Eigen::Vector4f(proj.x, proj.y, 0, 1);
		corner_pnts_3d_[idx2] = make_float3(transform_pnt(0), transform_pnt(1), transform_pnt(2));
		UpdateInformation();
	}

	void HWPlane::DoPointMovingDirect(int idx, Eigen::Vector3f& dest, float max_dist)
	{
		float3 destFloat = make_float3(dest(0), dest(1), dest(2));
		float3 src2dest = destFloat - corner_pnts_3d_[idx];
		float dist3d = sqrt(dot(src2dest, src2dest));

		if (dist3d > max_dist) {
			printf("move point failed due to max_dist\n");
			return;
		}

		HWPlane temp = *this;
		temp.corner_pnts_3d_[idx] = destFloat;
		temp.UpdateInformation();

		if (temp.IsSelfIntersection(idx, temp.corner_pnts_[idx]) > 0) {
			printf("move point failed due to self intersection\n");
		}
		else {
			*this = temp;
		}
	}

	bool HWPlane::DoMovePnt2dToNewPnt2d(int p_idx, Eigen::Vector2f& ray_pnt)
	{
		if (p_idx < 0 || p_idx >= corner_pnts_.size())
			return false;
		Eigen::Vector4f transform_pnt = plane_to_world_*Eigen::Vector4f(ray_pnt[0], ray_pnt[1], 0, 1);
		corner_pnts_[p_idx] = make_float2(ray_pnt[0], ray_pnt[1]);
		corner_pnts_3d_[p_idx] = make_float3(transform_pnt(0), transform_pnt(1), transform_pnt(2));
		UpdateInformation();
		return true;
	}

	void HWPlane::DoPointCreating(int idx, Eigen::Vector3f & L_dir, Eigen::Vector3f & L_point)
	{
		corner_pnts_last_state_ = corner_pnts_;
		corner_angels_last_state_ = corner_angels_;
		min_dists_last_state_ = min_dists_;
		corner_pnts_3d_last_state_ = corner_pnts_3d_;
		triangles_idx_last_state_ = triangles_idx_;
		float2 proj = ProjToLine3D(idx, L_dir, L_point);
		Eigen::Vector4f transform_pnt = plane_to_world_*Eigen::Vector4f(proj.x, proj.y, 0, 1);	
		float3 p = corner_pnts_3d_[idx], p_pre = corner_pnts_3d_[(idx + corner_pnts_3d_.size() - 1) % corner_pnts_3d_.size()];
		Eigen::Vector3f e1(transform_pnt(0) - p_pre.x, transform_pnt(1) - p_pre.y, transform_pnt(2) - p_pre.z);
		Eigen::Vector3f e2(p.x - transform_pnt(0), p.y - transform_pnt(1), p.z - transform_pnt(1));
		Eigen::Vector3f n = e1.cross(e2);
		bool is_before = false;
		if (n(0)*coeff_.x + n(1)*coeff_.y + n(2)*coeff_.z > 0)
			is_before = true;
		std::vector<float2> corner_pnts_temp;
		std::vector<float3> corner_pnts_3d_temp;
		for (int i = 0; i < corner_pnts_.size(); i++) {
			if (is_before&&i == idx) {
				corner_pnts_temp.emplace_back(proj);
				corner_pnts_3d_temp.emplace_back(make_float3(transform_pnt(0), transform_pnt(1), transform_pnt(2)));
			}
			corner_pnts_temp.emplace_back(corner_pnts_[i]);
			corner_pnts_3d_temp.emplace_back(corner_pnts_3d_[i]);
			if (!is_before&&i == idx) {
				corner_pnts_temp.emplace_back(proj);
				corner_pnts_3d_temp.emplace_back(make_float3(transform_pnt(0), transform_pnt(1), transform_pnt(2)));
			}
			
		}
		corner_pnts_ = corner_pnts_temp;
		corner_pnts_3d_ = corner_pnts_3d_temp;
		UpdateInformation();
	}

	void HWPlane::DoPointCreateOnLineSeg(int ls_idx, int le_idx, Eigen::Vector2f& pnt2d)
	{
		//如果添加的顶点和线段上的一个端点很近，不再添加
		if (ls_idx < 0 || ls_idx >= corner_pnts_.size()
			|| le_idx < 0 || le_idx >= corner_pnts_.size())
			return;

		float2 ls = corner_pnts_[ls_idx];
		float2 le = corner_pnts_[le_idx];

		if (PointPointDist(ls, le) < 1e-4)
			return;

		//保留之前的polygon
		corner_pnts_last_state_ = corner_pnts_;
		corner_angels_last_state_ = corner_angels_;
		min_dists_last_state_ = min_dists_;
		corner_pnts_3d_last_state_ = corner_pnts_3d_;
		triangles_idx_last_state_ = triangles_idx_;

		//计算世界坐标系上的点
		Eigen::Vector4f transform_pnt = plane_to_world_*Eigen::Vector4f(pnt2d[0], pnt2d[1], 0, 1);
		std::vector<float2> corner_pnts_temp;
		std::vector<float3> corner_pnts_3d_temp;

		if (ls_idx > le_idx)
		{
			std::cout << "ls_idx > le_idx " << std::endl;
			for (int i = 0; i < corner_pnts_.size(); ++i)
			{
				corner_pnts_temp.emplace_back(corner_pnts_[i]);
				corner_pnts_3d_temp.emplace_back(corner_pnts_3d_[i]);
			}
			corner_pnts_temp.emplace_back(make_float2(pnt2d[0], pnt2d[1]));
			corner_pnts_3d_temp.emplace_back(make_float3(transform_pnt[0], transform_pnt[1], transform_pnt[2]));
		}
		else
		{
			std::cout << "ls_idx < le_idx " << std::endl;
			for (int i = 0; i <= ls_idx; ++i)
			{
				corner_pnts_temp.emplace_back(corner_pnts_[i]);
				corner_pnts_3d_temp.emplace_back(corner_pnts_3d_[i]);
			}
			corner_pnts_temp.emplace_back(make_float2(pnt2d[0], pnt2d[1]));
			corner_pnts_3d_temp.emplace_back(make_float3(transform_pnt[0], transform_pnt[1], transform_pnt[2]));
			for (int i = le_idx; i < corner_pnts_.size(); ++i)
			{
				corner_pnts_temp.emplace_back(corner_pnts_[i]);
				corner_pnts_3d_temp.emplace_back(corner_pnts_3d_[i]);
			}
		}
		corner_pnts_ = corner_pnts_temp;
		corner_pnts_3d_ = corner_pnts_3d_temp;
		UpdateInformation();
	}

	bool HWPlane::PlanePntProjPolygonLine2D(Eigen::Vector2f& pnt2d, float& threshold_dist, Eigen::Vector2i& line_pnt_idx, Eigen::Vector2f& proj_pnt)
	{
		if (corner_pnts_.size() < 2)
			return false;
		float mindist2ls = FLT_MAX;
		Eigen::Vector2f minprojpnt;
		Eigen::Vector2i minpntidx = Eigen::Vector2i(-1, -1);
		for (int i = 0, j = corner_pnts_.size() - 1; i < corner_pnts_.size(); j = i++)
		{
			Eigen::Vector2f ls = Eigen::Vector2f(corner_pnts_[j].x, corner_pnts_[j].y);
			Eigen::Vector2f le = Eigen::Vector2f(corner_pnts_[i].x, corner_pnts_[i].y);

			//顶点到这线段距离
			Eigen::Vector2f tmp_projpnt;
			if (Pnt2dProjLineSeg2D(pnt2d, ls, le, tmp_projpnt) == 0)
			{
				if ((tmp_projpnt - pnt2d).norm() < mindist2ls)
				{
					mindist2ls = (tmp_projpnt - pnt2d).norm();
					minpntidx = Eigen::Vector2i(j, i);
					minprojpnt = tmp_projpnt;
				}
			}
		}

		if (mindist2ls < threshold_dist)
		{
			line_pnt_idx = minpntidx;
			proj_pnt = minprojpnt;
			return true;
		}
		return false;
	}

	void HWPlane::DoPolygonNormalsAlignPntsNormals()
	{
		std::cerr << "start to align the plane normal..." << std::endl;
		std::cerr << "to do next..." << std::endl;
		std::cerr << "end align the plane normal..." << std::endl;
	}

	void HWPlane::DoPolygonInversing()
	{
		std::cout << "before inverse plane normal: " << plane_normal_.transpose() << std::endl;
		corner_pnts_last_state_ = corner_pnts_;
		corner_angels_last_state_ = corner_angels_;
		min_dists_last_state_ = min_dists_;
		corner_pnts_3d_last_state_ = corner_pnts_3d_;
		triangles_idx_last_state_ = triangles_idx_;
		std::vector<float2> corner_pnts_temp;
		std::vector<float3> corner_pnts_3d_temp;
		for (int i = corner_pnts_.size() - 1; i >= 0; i--) {
			//corner_pnts_temp.emplace_back(corner_pnts_[i]);
			corner_pnts_3d_temp.emplace_back(corner_pnts_3d_[i]);		
			}		
			for (int i = 0; i < pnts_normal_origin_.size(); ++i) {			
						pnts_normal_origin_[i] *= -1;
						pnts_normal_[i] *= -1;
			}
		//corner_pnts_ = corner_pnts_temp;
		corner_pnts_3d_ = corner_pnts_3d_temp;
		plane_to_world_.col(1) *= -1;
		plane_to_world_.col(2) *= -1;
		world_to_plane_ = plane_to_world_.inverse();
		coeff_ *= -1;
		for (int i = 0; i < corner_pnts_3d_.size(); i++) {
			Eigen::Vector4f pnt(corner_pnts_3d_[i].x, corner_pnts_3d_[i].y, corner_pnts_3d_[i].z, 1.0);
			Eigen::Vector4f transform_pnt = world_to_plane_*pnt;
			corner_pnts_[i] = make_float2(transform_pnt(0), transform_pnt(1));
		}
		UpdateInformation();
		std::cout << "after inverse plane normal: " << plane_normal_.transpose() << std::endl;
	}

	void HWPlane::DoPolygonDeleting()
	{
		corner_pnts_last_state_ = corner_pnts_;
		corner_angels_last_state_ = corner_angels_;
		min_dists_last_state_ = min_dists_;
		corner_pnts_3d_last_state_ = corner_pnts_3d_;
		triangles_idx_last_state_ = triangles_idx_;
		corner_pnts_.clear();
		corner_pnts_3d_.clear();
		min_dists_.clear();
		corner_angels_.clear();
		triangles_idx_.clear();
	}

	void HWPlane::DoPointDeleting(int idx)
	{
		corner_pnts_last_state_ = corner_pnts_;
		corner_angels_last_state_ = corner_angels_;
		min_dists_last_state_ = min_dists_;
		corner_pnts_3d_last_state_ = corner_pnts_3d_;
		triangles_idx_last_state_ = triangles_idx_;
		corner_pnts_.erase(corner_pnts_.begin()+idx);
		corner_pnts_3d_.erase(corner_pnts_3d_.begin() + idx);
		UpdateInformation();
	}

	void HWPlane::DoPointDeleting(int start, int end)
	{
		corner_pnts_last_state_ = corner_pnts_;
		corner_angels_last_state_ = corner_angels_;
		min_dists_last_state_ = min_dists_;
		corner_pnts_3d_last_state_ = corner_pnts_3d_;
		triangles_idx_last_state_ = triangles_idx_;
		std::vector<float2> corner_pnts_temp;
		std::vector<float3> corner_pnts_3d_temp;
		for (int i = 0; i < corner_pnts_.size(); i++) {
			if ((i <= start && i >= end&&start > end) || ((start < end) && (i >= end&&i < corner_pnts_.size() || (i <= start&&i >= 0)))) {
				corner_pnts_temp.emplace_back(corner_pnts_[i]);
				corner_pnts_3d_temp.emplace_back(corner_pnts_3d_[i]);
			}
		}
		corner_pnts_ = corner_pnts_temp;
		corner_pnts_3d_ = corner_pnts_3d_temp;
		UpdateInformation();
	}

	void HWPlane::DoPointAdding(float2 pnt, int idx, bool is_before)
	{
		corner_pnts_last_state_ = corner_pnts_;
		corner_angels_last_state_ = corner_angels_;
		min_dists_last_state_ = min_dists_;
		corner_pnts_3d_last_state_ = corner_pnts_3d_;
		triangles_idx_last_state_ = triangles_idx_;
		std::vector<float2> corner_pnts_temp;
		std::vector<float3> corner_pnts_3d_temp;
		for (int i = 0; i < corner_pnts_.size(); i++) {
			if (i == idx&&is_before) {
				corner_pnts_temp.emplace_back(pnt);
				Eigen::Vector4f transform_pnt = plane_to_world_*Eigen::Vector4f(pnt.x, pnt.y, 0, 1);
				corner_pnts_3d_temp.emplace_back(make_float3(transform_pnt(0), transform_pnt(1), transform_pnt(2)));
			}	
			corner_pnts_temp.emplace_back(corner_pnts_[i]);
			corner_pnts_3d_temp.emplace_back(corner_pnts_3d_[i]);
			if (i == idx&&!is_before) {
				corner_pnts_temp.emplace_back(pnt);
				Eigen::Vector4f transform_pnt = plane_to_world_*Eigen::Vector4f(pnt.x, pnt.y, 0, 1);
				corner_pnts_3d_temp.emplace_back(make_float3(transform_pnt(0), transform_pnt(1), transform_pnt(2)));
			}
		}
		corner_pnts_ = corner_pnts_temp;
		corner_pnts_3d_ = corner_pnts_3d_temp;
		UpdateInformation();
	}

	void HWPlane::GetPolygonFromPnts()
	{
		for (int i = 0; i < plane_coord_pos_.size(); i++) {
			corner_pnts_.emplace_back(make_float2(plane_coord_pos_[i].x, plane_coord_pos_[i].y));
			corner_pnts_3d_.emplace_back(pnts_pos_[i]);
		}
		UpdateInformation();
	}

	int HWPlane::IsInPolygon(float2 & p)
	{
		int i, j, c = 0;
		for (i = 0, j = corner_pnts_.size() - 1; i < corner_pnts_.size(); j = i++) {
			if (((corner_pnts_[i].y > p.y) != (corner_pnts_[j].y > p.y)) &&
				(p.x < (corner_pnts_[i].x - corner_pnts_[j].x)*(p.y - corner_pnts_[i].y) / (corner_pnts_[i].y - corner_pnts_[j].y) + corner_pnts_[i].x))
				c = !c;
		}
		return c;
	}

	int HWPlane::IsInSelectedPolygonNew(std::vector<Eigen::Vector2f>& polypnts2d, Eigen::Vector2f& p)
	{
		int i, j, c = 0;
		for (i = 0, j = polypnts2d.size() - 1; i < polypnts2d.size(); j = i++) {
			if (((polypnts2d[i][1] > p[1]) != (polypnts2d[j][1] > p[1])) &&
				(p[0] < (polypnts2d[i][0] - polypnts2d[j][0])*(p[1] - polypnts2d[i][1]) / (polypnts2d[i][1] - polypnts2d[j][1]) + polypnts2d[i][0]))
				c = !c;
		}
		return c;
	}

	int HWPlane::IsInPolygon(float3 & p)
	{
		float2 p_2d = ProjToPlane3D(p);
		if (IsInPolygon(p_2d))
			return 1; 
		else return 0;
	}

	int HWPlane::IsPointInPolygon3D(Eigen::Vector3f& pnt)
	{
		Eigen::Vector2f pnt2d;
		Pnt3d2Pnt2D(pnt, pnt2d);
		//只处理corner_pnts_, inner_corner_pnts_这个镂空的结构后续处理
		float2 p_2d = make_float2(pnt2d[0], pnt2d[1]);
		if (IsInPolygon(p_2d))
			return 1;
		else return 0;
	}

	int HWPlane::IsConcaveVertex(int idx)
	{
		std::vector<float2> corner_pnts;
		for (int i = 0; i < corner_pnts_.size(); i++) {
			if (i == idx)
				continue;
			corner_pnts.emplace_back(corner_pnts_[i]);
		}
		float2 p = corner_pnts_[idx];
		int i, j, c = 0;
		for (i = 0, j = corner_pnts.size() - 1; i < corner_pnts.size(); j = i++) {
			if (((corner_pnts[i].y > p.y) != (corner_pnts[j].y > p.y)) &&
				(p.x < (corner_pnts[i].x - corner_pnts[j].x)*(p.y - corner_pnts[i].y) / (corner_pnts[i].y - corner_pnts[j].y) + corner_pnts[i].x))
				c = !c;
		}
		return c;
	}

	int HWPlane::IsConcaveVertex(const std::vector<float2> &corner_points, int idx)
	{
		std::vector<float2> corner_pnts;
		for (int i = 0; i < corner_points.size(); i++) {
			if (i == idx)
				continue;
			corner_pnts.emplace_back(corner_points[i]);
		}
		float2 p = corner_points[idx];
		int i, j, c = 0;
		for (i = 0, j = corner_pnts.size() - 1; i < corner_pnts.size(); j = i++) {
			if (((corner_pnts[i].y > p.y) != (corner_pnts[j].y > p.y)) &&
				(p.x < (corner_pnts[i].x - corner_pnts[j].x) * (p.y - corner_pnts[i].y) / (corner_pnts[i].y - corner_pnts[j].y) + corner_pnts[i].x))
				c = !c;
		}
		return c;
	}


	int HWPlane::IsConcaveVertexNew(const std::vector<Eigen::Vector2f> &corner_points, int idx)
	{
		std::vector<Eigen::Vector2f> corner_pnts;
		for (int i = 0; i < corner_points.size(); i++) {
			if (i == idx)
				continue;
			corner_pnts.emplace_back(corner_points[i]);
		}
		Eigen::Vector2f p = corner_points[idx];
		int i, j, c = 0;
		for (i = 0, j = corner_pnts.size() - 1; i < corner_pnts.size(); j = i++) {
			if (((corner_pnts[i][1] > p[1]) != (corner_pnts[j][1] > p[1])) &&
				(p[0] < (corner_pnts[i][0] - corner_pnts[j][0]) * (p[1] - corner_pnts[i][1]) / (corner_pnts[i][1] - corner_pnts[j][1]) + corner_pnts[i][0]))
				c = !c;
		}
		return c;
	}

	float HWPlane::Pnt2PolygonMinDist2D(Eigen::Vector2f p)
	{
		float min_dist = FLT_MAX;
		for (std::size_t i = 0, j = corner_pnts_.size() - 1; i < corner_pnts_.size(); j = i++)
		{
			Eigen::Vector2f s = Eigen::Vector2f(corner_pnts_[j].x, corner_pnts_[j].y);
			Eigen::Vector2f e = Eigen::Vector2f(corner_pnts_[i].x, corner_pnts_[i].y);
			float dist = PntDist2LineSegment2D(p, s, e);
			if (dist < min_dist)
			{
				min_dist = dist;
			}
		}
		return min_dist;
	}

	int HWPlane::IsPInPolygonVec(Eigen::Vector2f& p, std::vector<Eigen::Vector2f>& in_polygon)
	{
		int i, j, c = 0;
		for (i = 0, j = in_polygon.size() - 1; i < in_polygon.size(); j = i++) {
			if (((in_polygon[i][1] > p[1]) != (in_polygon[j][1] > p[1])) &&
				(p[0] < (in_polygon[i][0] - in_polygon[j][0])*(p[1] - in_polygon[i][1]) / (in_polygon[i][1] - in_polygon[j][1]) + in_polygon[i][0]))
				c = !c;
		}
		return c;
	}

	void HWPlane::GetPolygonConcaveVertex2DIdx(std::vector<int>& p_idxs)
	{
		//
		for(int i = 0; i < corner_pnts_.size(); ++i)
		{
			if (IsConcaveVertex(i))
				p_idxs.emplace_back(i);
		}
	}

	void HWPlane::GetPolygonConcaveVertex2DPnts(std::vector<Eigen::Vector2f> p_pnts)
	{
		//处理这些
		for (int i = 0; i < corner_pnts_.size(); ++i)
		{
			if (IsConcaveVertex(i))
			{
				Eigen::Vector2f pnt(corner_pnts_[i].x, corner_pnts_[i].y);
				p_pnts.emplace_back(pnt);
			}
		}
	}

	int HWPlane::IsConcaveEdgePoint(int idx)
	{
		std::vector<float2> edge_points;
		for (int i = 0; i < edge_pnts_pos_.size(); i++) {
			if (i == idx)
				continue;
			edge_points.emplace_back(edge_pnts_pos_[i]);
		}
		float2 p = edge_pnts_pos_[idx];
		int i, j, c = 0;
		for (i = 0, j = edge_points.size() - 1; i < edge_points.size(); j = i++) {
			if (((edge_points[i].y > p.y) != (edge_points[j].y > p.y)) &&
				(p.x < (edge_points[i].x - edge_points[j].x)*(p.y - edge_points[i].y) / (edge_points[i].y - edge_points[j].y) + edge_points[i].x))
				c = !c;
		}
		return c;
	}

	int HWPlane::IsConcaveEdgePoint(const std::vector<float2>& points, int idx)
	{
		std::vector<float2> edge_points;
		for (int i = 0; i < points.size(); i++) {
			if (i == idx)
				continue;
			edge_points.emplace_back(points[i]);
		}
		float2 p = points[idx];
		int i, j, c = 0;
		for (i = 0, j = edge_points.size() - 1; i < edge_points.size(); j = i++) {
			if (((edge_points[i].y > p.y) != (edge_points[j].y > p.y)) &&
				(p.x < (edge_points[i].x - edge_points[j].x) * (p.y - edge_points[i].y) / (edge_points[i].y - edge_points[j].y) + edge_points[i].x))
				c = !c;
		}
		return c;
	}

	int HWPlane::PointMatchingJudge(float3 & point, float rp, float& dist)
	{
		dist = abs(GetDistToPlane(point));
		if (dist > rp)
			return false;
		float3 point_proj = GetProj3DPnt(point);
		float2 point_proj2D = TransformWorld2Plane(Eigen::Vector3f(point_proj.x, point_proj.y, point_proj.z));
		int c = IsInPolygon(point_proj2D);
		return c;
	}

	float HWPlane::GetEdgesAngel(float3 & v1, float3 & v2, int edge_idx1, int edge_idx2)
	{
		float2 v3 = corner_pnts_[edge_idx1]; 
		float2 v4 = corner_pnts_[edge_idx2];
		float2 v1_proj = ProjToPlane3D(v1);
		float2 v2_proj = ProjToPlane3D(v2);
		float2 e1 = v1_proj - v2_proj;
		float2 e2 = v3 - v4;
		return GetTheta(e1.x,e1.y, e2.x,e2.y);
	}

	float HWPlane::GetDistToPolygon(Eigen::Vector3f & p)
	{
		/*float3 pnt = make_float3(p(0), p(1), p(2));
		float2 p_2d = ProjToPlane3D(pnt);
		if (IsInPolygon(p_2d))
			return 0;*/

		float min_dist = FLT_MAX;
		for (int i = 0; i < corner_pnts_3d_.size(); i++) {
			float dist;
			int next_idx = (i + 1) % corner_pnts_3d_.size();
			Eigen::Vector3f p1(corner_pnts_3d_[i].x, corner_pnts_3d_[i].y, corner_pnts_3d_[i].z);
			Eigen::Vector3f p2(corner_pnts_3d_[next_idx].x, corner_pnts_3d_[next_idx].y, corner_pnts_3d_[next_idx].z);
			/*std::cout << "p: " << p.transpose() << "\n";
			std::cout << "p1: " << p1.transpose() << "\n";
			std::cout << "p2: " << p2.transpose() << "\n";*/
			Eigen::Vector3f l1 = p1 - p, l2 = p2 - p, l = p2 - p1;
			if (l1.dot(l) >= 0)
				dist = l1.norm();
			else if (l2.dot(l) <= 0)
				dist = l2.norm();
			else {
				float theta = acos(-l1.dot(l) / l1.norm() / l.norm());
				dist = l1.norm()*sin(theta);
			}
			//std::cout << "dist: " << dist << "\n";
			if (dist < min_dist)
				min_dist = dist;
		}
		return min_dist;
		//float2 l1 = p1 - p, l2 = p2 - p, l = p2 - p1;
		//if (dot(l1, l) >= 0)
		//	return sqrt(l1.x*l1.x + l1.y*l1.y);
		//else if (dot(l2, l) <= 0)
		//	return sqrt(l2.x*l2.x + l2.y*l2.y);
		//else {
		//	float theta = GetTheta(-l1.x, -l1.y, l.x, l.y);
		//	float d = sqrt(l1.x*l1.x + l1.y*l1.y)*sin(theta / 180 * MATH_PI);
		//	return d;
		//	//return abs(l1.x*l.y - l1.y*l.x) / sqrt(l1.x*l1.x + l1.y*l1.y);
		//}
		//return 0.0f;
	}

	void HWPlane::DoTriangulation()
	{
		printf("begin DoTriangulation\n");
		std::vector<Point2> points;
		for (int i = 0; i < all_corner_pnts_.size(); i++) {
			points.push_back(Point2(all_corner_pnts_[i].x, all_corner_pnts_[i].y));
		}
		Polygon_2 polygon;
		for (int i = 0; i < corner_pnts_.size(); i++) {
			polygon.push_back(Point2(corner_pnts_[i].x, corner_pnts_[i].y));
			//points.push_back(Point2(corner_pnts_[i].x, corner_pnts_[i].y));
		}
		std::vector<Polygon_2> inner_polygons;
		for (int i = 0; i < inner_polygon_num_; i++) {
			Polygon_2 curr;
			auto& corner_pnts = inner_corner_pnts_[i];
			for (int j = 0; j < corner_pnts.size(); j++) {
				curr.push_back(Point2(corner_pnts[j].x, corner_pnts[j].y));
				//points.push_back(Point2(corner_pnts[j].x, corner_pnts[j].y));
			}
			inner_polygons.push_back(curr);
		}
		CDT cdt_polygon;
		cdt_polygon.insert_constraint(polygon.vertices_begin(), polygon.vertices_end(), true);
		for (int i = 0; i < inner_polygon_num_; i++) {
			cdt_polygon.insert_constraint(inner_polygons[i].vertices_begin(), inner_polygons[i].vertices_end(), true);
		}
		//Mark facets that are inside the domain bounded by the polygon
		mark_domains1(cdt_polygon);
		triangles_idx_.clear();
		std::vector<Point2> triangulated;
		int tri_num = 0;
		Eigen::Vector3f plane_normal = GetPlaneNormal();
		//std::cout << "plane_normal: " << plane_normal.transpose() << std::endl;
		for (Face_handle f : cdt_polygon.finite_face_handles())
		{
			std::vector<Point2> tri_vertices;
			if (f->info().in_domain())
			{
				++tri_num;
				int tri[3] = { -1,-1,-1 };
				for (int i = 0; i < 3; ++i)
				{
					Point2 v;
					//v=f->vertex(f->ccw(i))->point();
					//std::cout << "the f->cw(i): " << f->cw(i) << std::endl;
					v = f->vertex(i)->point();
					triangulated.push_back(v);
					for (int j = 0; j < points.size(); j++) {
						//if (v.x() == points[j].x()&&v.y() == points[j].y()) {
						//if (abs(v.x() - points[j].x()) < 0.001 &&
							//abs(v.y() - points[j].y()) < 0.001) {
						if (v == points[j]) {
							tri[i] = j;
							break;
						}
					}

					//for (int j = 0; j < corner_pnts_.size(); j++) {
					//	if (v.x() == corner_pnts_[j].x&&v.y() == corner_pnts_[j].y) {
					//		tri[i] = j;
					//		break;
					//	}
					//}
				}
#ifdef LIULINGFEI
				std::cout << tri[0] << ", " << tri[1] << ", " << tri[2] << std::endl;
#endif
				if (tri[0] == -1 || tri[1] == -1 || tri[2] == -1) continue;

				float3 p0 = all_corner_pnts_3d_[tri[0]];
				float3 p1 = all_corner_pnts_3d_[tri[1]];
				float3 p2 = all_corner_pnts_3d_[tri[2]];
				Eigen::Vector3f tri_normal = GetNormalFrom3Points(p0, p1, p2);	//感觉这个有问题，和方向相反，可能需要改一下
				//std::cout << "the plane_normal " << tri_num<< ": " << plane_normal.transpose() << std::endl;
				//std::cout << "the tri_normal " << tri_num <<": " << tri_normal.transpose() << std::endl;
				if (plane_normal.dot(tri_normal) > 0) {
					triangles_idx_.emplace_back(make_int3(tri[2], tri[1], tri[0]));
				}
				else
				{
					triangles_idx_.emplace_back(make_int3(tri[0], tri[1], tri[2]));	//其中triangles_idx_有问题
				}
				
			}
		}
		printf("end DoTriangulation\n");
		/*std::cout << "the filename_: " << filename_ << std::endl;
		std::ofstream fh("D:\\vc_project_new\\test_data\\lake1\\7\\triangulation_test3.obj");
		for (int i = 0; i < triangulated.size(); i++)
		{
			Point2 start_pnt = triangulated[i];
			Eigen::Vector4f transform_pnt = plane_to_world_ * Eigen::Vector4f(start_pnt.x(), start_pnt.y(), 0, 1);
			fh << "v " << transform_pnt.x() << " " << transform_pnt.y() << " " << transform_pnt.z() << std::endl;
		}
		for (int i = 0; i < triangulated.size(); i += 3) {
			fh << "f " << i + 1 << " " << i + 2 <<" " << i+3 << std::endl;
		}
		fh.close();
		std::vector<Eigen::Vector3f> triglated_pnts3d;
		std::ofstream fhd("D:\\vc_project_new\\test_data\\lake1\\7\\triangulation_test4.obj");
		for (int i = 0; i < triangulated.size(); i++)
		{
			Point2 start_pnt = triangulated[i];
			Eigen::Vector4f transform_pnt = plane_to_world_ * Eigen::Vector4f(start_pnt.x(), start_pnt.y(), 0, 1);
			triglated_pnts3d.emplace_back(Eigen::Vector3f(transform_pnt[0], transform_pnt[1], transform_pnt[2]));
			fhd << "v " << transform_pnt.x() << " " << transform_pnt.y() << " " << transform_pnt.z() << std::endl;
		}
		int trinum_test = 0;
		for (int i = 0; i < triglated_pnts3d.size(); i += 3) 
		{
			float3 p0 = make_float3(triglated_pnts3d[i][0], triglated_pnts3d[i][1], triglated_pnts3d[i][2]);
			float3 p1 = make_float3(triglated_pnts3d[i+1][0], triglated_pnts3d[i+1][1], triglated_pnts3d[i+1][2]);
			float3 p2 = make_float3(triglated_pnts3d[i+2][0], triglated_pnts3d[i+2][1], triglated_pnts3d[i+2][2]);
			Eigen::Vector3f tri_normal = GetNormalFrom3Points(p0, p1, p2);
			std::cout << "the plane_normal0 " << trinum_test << ": " << plane_normal.transpose() << std::endl;
			std::cout << "the tri_normal0 " << trinum_test << ": " << tri_normal.transpose() << std::endl;
			if (plane_normal.dot(tri_normal) < 0) {
				fhd << "f " << i + 3 << " " << i + 2 << " " << i + 1 << std::endl;
			}
			else
			{
				fhd << "f " << i + 1 << " " << i + 2 << " " << i + 3 << std::endl;
			}
			trinum_test++;
		}
		fhd.close();*/

#ifdef LIULINGFEI
		std::cout << "the filename_: " << filename_ << std::endl;
		std::ofstream fh("D:\\vc_project_new\\test_data\\lake1\\7\\triangulation_test3.obj");
		for (int i = 0; i < triangulated.size(); i++)
		{
			Point2 start_pnt = triangulated[i];
			Eigen::Vector4f transform_pnt = plane_to_world_ * Eigen::Vector4f(start_pnt.x(), start_pnt.y(), 0, 1);
			fh << "v " << transform_pnt.x() << " " << transform_pnt.y() << " " << transform_pnt.z() << std::endl;
		}
		for (int i = 0; i < triangulated.size(); i += 3) {
			fh << "l " << i + 1 << " " << i + 2 << std::endl;
			fh << "l " << i + 2 << " " << i + 3 << std::endl;
			fh << "l " << i + 3 << " " << i + 1 << std::endl;
		}
		fh.close();
#endif
	}

	void HWPlane::DoTriangulationRefined()	//最新版本的三角化函数
	{
		printf("begin DoTriangulation refine\n");
		std::vector<C1Point> points;
		for (int i = 0; i < all_corner_pnts_.size(); i++) {
			points.push_back(C1Point(all_corner_pnts_[i].x, all_corner_pnts_[i].y));
		}
		Polygon_2 polygon;
		for (int i = 0; i < corner_pnts_.size(); i++) {
			polygon.push_back(C1Point(corner_pnts_[i].x, corner_pnts_[i].y));
			//points.push_back(Point2(corner_pnts_[i].x, corner_pnts_[i].y));
		}
		std::vector<Polygon_2> inner_polygons;
		for (int i = 0; i < inner_polygon_num_; i++) {
			Polygon_2 curr;
			auto& corner_pnts = inner_corner_pnts_[i];
			for (int j = 0; j < corner_pnts.size(); j++) {
				curr.push_back(Point2(corner_pnts[j].x, corner_pnts[j].y));
				//points.push_back(Point2(corner_pnts[j].x, corner_pnts[j].y));
			}
			inner_polygons.push_back(curr);
		}
		//先只处理外部点云
		CDT1 cdt_polygon;
		std::vector<Vertex_handle> polyvhd;
		for (int i = 0; i < polygon.size(); ++i)
		{
			Vertex_handle tmpvhd = cdt_polygon.insert(polygon[i]);
			polyvhd.emplace_back(tmpvhd);
		}
		for (int i = 0; i < polyvhd.size(); ++i)
		{
			int nextidx = (i + 1) % polyvhd.size();
			Eigen::Vector2f p_len = Eigen::Vector2f(polyvhd[i]->point().x() - polyvhd[nextidx]->point().x(),
				polyvhd[i]->point().y() - polyvhd[nextidx]->point().y());
			if (p_len.norm() < 1e-2)
				continue;
			cdt_polygon.insert_constraint(polyvhd[i], polyvhd[nextidx]);
		}
		std::cout << "Number of vertices: " << cdt_polygon.number_of_vertices() << std::endl;
		std::list<Point2> list_of_seeds;
		std::cout << "Meshing the domain..." << std::endl;
		float radian = plane_r_tri_degree_*M_PI / 180;
		//radian = 20;
		//plane_r_tri_len_ = 0.03;	//test
		std::cout << "the radian: " << radian << std::endl;
		std::cout << "the plane_r_tri_len_: " << plane_r_tri_len_ << std::endl;
		
		CGAL::refine_Delaunay_mesh_2(cdt_polygon, list_of_seeds.begin(), list_of_seeds.end(),
			Criteria(radian, plane_r_tri_len_));

		std::cout << "Number of vertices: " << cdt_polygon.number_of_vertices() << std::endl;
		std::cout << "Number of finite faces: " << cdt_polygon.number_of_faces() << std::endl;

		/*cdt_polygon.insert_constraint(polygon.vertices_begin(), polygon.vertices_end(), true);
		for (int i = 0; i < inner_polygon_num_; i++) {
			cdt_polygon.insert_constraint(inner_polygons[i].vertices_begin(), inner_polygons[i].vertices_end(), true);
		}*/
		//triangles_idx_.clear();
		std::vector<Point2> triangulated;
		Eigen::Vector3f plane_normal = GetPlaneNormal();
		for (CDT1::Finite_faces_iterator f = cdt_polygon.finite_faces_begin(); 
			f != cdt_polygon.finite_faces_end(); ++f)
		{
			std::vector<Point2> tri_vertices;
			if (f->is_in_domain())
			{
				int tri[3] = { -1,-1,-1 };
				for (int i = 0; i < 3; ++i)
				{
					Point2 v;
					//Vertex_handle tmphdx = f->vertex(i)->incident_vertices();
					//v=f->vertex(f->ccw(i))->point();
					v = f->vertex(i)->point();
					triangulated.push_back(v);
					for (int j = 0; j < points.size(); j++) {
						//if (v.x() == points[j].x()&&v.y() == points[j].y()) {
						//if (abs(v.x() - points[j].x()) < 0.001 &&
						//abs(v.y() - points[j].y()) < 0.001) {
						if (v == points[j]) {
							tri[i] = j;
							break;
						}
					}

					//for (int j = 0; j < corner_pnts_.size(); j++) {
					//	if (v.x() == corner_pnts_[j].x&&v.y() == corner_pnts_[j].y) {
					//		tri[i] = j;
					//		break;
					//	}
					//}
				}
#ifdef LIULINGFEI
				std::cout << tri[0] << ", " << tri[1] << ", " << tri[2] << std::endl;
#endif
				if (tri[0] == -1 || tri[1] == -1 || tri[2] == -1) continue;

				float3 p0 = all_corner_pnts_3d_[tri[0]];
				float3 p1 = all_corner_pnts_3d_[tri[1]];
				float3 p2 = all_corner_pnts_3d_[tri[2]];
				Eigen::Vector3f tri_normal = GetNormalFrom3Points(p0, p1, p2);
				if (plane_normal.dot(tri_normal) < 0) {
					std::swap(tri[0], tri[2]);
				}
				//triangles_idx_.emplace_back(make_int3(tri[0], tri[1], tri[2]));
			}
		}

		polygon_tri_refine_pnts_.clear();
		polygon_tri_refine_idx_.clear();
		std::map<Vertex_handle, int> v2tris;
		int num_vertices = 0;
		for (CDT1::Finite_vertices_iterator vh = cdt_polygon.finite_vertices_begin();
			vh != cdt_polygon.finite_vertices_end(); ++vh)
		{
			//Point2 tmppnt = vh->point();
			float2 tmp_pnt = make_float2(vh->point().x(), vh->point().y());
			Eigen::Vector4f transform_pnt = plane_to_world_ * Eigen::Vector4f(tmp_pnt.x, tmp_pnt.y, 0, 1);
			float3 tmp_pnt3d = make_float3(transform_pnt[0], transform_pnt[1], transform_pnt[2]);
			polygon_tri_refine_pnts_.emplace_back(tmp_pnt3d);
			v2tris[vh] = num_vertices++;
		}
		for (CDT1::Finite_faces_iterator f = cdt_polygon.finite_faces_begin();
			f != cdt_polygon.finite_faces_end(); ++f)
		{
			if (f->is_in_domain())
			{
				//int3 tmpfidx = make_int3(-1, -1, -1);
				Eigen::Vector3i tmpf_idx(-1, -1, -1);
				for (int i = 0; i < 3; ++i)
				{
					Vertex_handle tmphdx = f->vertex(i);
					tmpf_idx[i] = v2tris[tmphdx];
				}
				if (tmpf_idx[0] != -1 || tmpf_idx[1] != -1 || tmpf_idx[2] != -1)
				{
					float3 p0 = polygon_tri_refine_pnts_[tmpf_idx[0]];
					float3 p1 = polygon_tri_refine_pnts_[tmpf_idx[1]];
					float3 p2 = polygon_tri_refine_pnts_[tmpf_idx[2]];
					Eigen::Vector3f tri_normal = GetNormalFrom3Points(p0, p1, p2);
					if (plane_normal.dot(tri_normal) < 0) {
						std::swap(tmpf_idx[0], tmpf_idx[2]);
					}
					polygon_tri_refine_idx_.emplace_back(make_int3(tmpf_idx[0], tmpf_idx[1], tmpf_idx[2]));
				}
			}
		}

#ifdef LIULINGFEI
		std::ofstream fh("D:\\vc_project_new\\test_data\\lake1\\2\\triangulation_test3.obj");
		for (int i = 0; i < polygon_tri_refine_pnts_.size(); ++i)
		{
			fh << "v " << polygon_tri_refine_pnts_[i].x << " " << polygon_tri_refine_pnts_[i].y << " " 
				<< polygon_tri_refine_pnts_[i].z << std::endl;
		}
		for (int i = 0; i < polygon_tri_refine_idx_.size(); ++i)
		{
			fh << "f " << polygon_tri_refine_idx_[i].x+1 << " " << polygon_tri_refine_idx_[i].y+1 << " "
				<< polygon_tri_refine_idx_[i].z+1 << std::endl;
		}
		/*for (int i = 0; i < triangulated.size(); i++)
		{
			Point2 start_pnt = triangulated[i];
			Eigen::Vector4f transform_pnt = plane_to_world_ * Eigen::Vector4f(start_pnt.x(), start_pnt.y(), 0, 1);
			fh << "v " << transform_pnt.x() << " " << transform_pnt.y() << " " << transform_pnt.z() << std::endl;
		}
		for (int i = 0; i < triangulated.size(); i += 3) {
			fh << "l " << i + 1 << " " << i + 2 << std::endl;
			fh << "l " << i + 2 << " " << i + 3 << std::endl;
			fh << "l " << i + 3 << " " << i + 1 << std::endl;
		}*/
		fh.close();
#endif
	}

	void HWPlane::ComputeTriNormalRefined()
	{
		polygon_tri_refine_normals_.clear();
		Eigen::Vector3f plane_normal = GetPlaneNormal();
		for (int i = 0; i < polygon_tri_refine_pnts_.size(); ++i)
		{
			polygon_tri_refine_normals_.emplace_back(make_float3(plane_normal[0], plane_normal[1], plane_normal[2]));
		}
	}

	void HWPlane::ComputeTriPntsUVRefined()
	{
		//std::cout << "UpdateInformation" << std::endl;
		boxMin_2d_ = Eigen::Vector2f(FLT_MAX, FLT_MAX), boxMax_2d_ = Eigen::Vector2f(-FLT_MAX, -FLT_MAX);
		for (int i = 0; i < polygon_tri_refine_pnts_.size(); i++) {
			if (polygon_tri_refine_pnts_[i].x < boxMin_2d_(0))
				boxMin_2d_(0) = polygon_tri_refine_pnts_[i].x;
			if (polygon_tri_refine_pnts_[i].y < boxMin_2d_(1))
				boxMin_2d_(1) = polygon_tri_refine_pnts_[i].y;
			if (polygon_tri_refine_pnts_[i].x > boxMax_2d_(0))
				boxMax_2d_(0) = polygon_tri_refine_pnts_[i].x;
			if (polygon_tri_refine_pnts_[i].y > boxMax_2d_(1))
				boxMax_2d_(1) = polygon_tri_refine_pnts_[i].y;
		}
		plane_width_ = min(boxMax_2d_(0) - boxMin_2d_(0), boxMax_2d_(1) - boxMin_2d_(1));
		plane_height_ = max(boxMax_2d_(0) - boxMin_2d_(0), boxMax_2d_(1) - boxMin_2d_(1));
		float length_x = boxMax_2d_(0) - boxMin_2d_(0), length_y = boxMax_2d_(1) - boxMin_2d_(1);
		polygon_tri_refine_uvs_.clear();
		for (int i = 0; i < polygon_tri_refine_pnts_.size(); i++) {
			float2 uv;
			uv.x = (polygon_tri_refine_pnts_[i].x - boxMin_2d_(0)) / length_x;
			uv.y = (polygon_tri_refine_pnts_[i].y - boxMin_2d_(1)) / length_y;
			polygon_tri_refine_uvs_.emplace_back(uv);
		}
	}

	void HWPlane::UpdateInformation()
	{
		//std::ofstream fh(filename_ + "all_corner_pnts_before.obj");
		//std::cout << "contour_index_.size() = " << contour_index_.size() << std::endl;
		//18446744073709551615 on windows
		//std::cout << "contour_index_.size() - 1 = " << contour_index_.size() - 1 << std::endl;
		//std::cout << "all_corner_pnts_3d_.size() = " << all_corner_pnts_3d_.size() << std::endl;
		//for (int i = 0; i < all_corner_pnts_3d_.size(); i++)
		//{
		//	float2 start_pnt = all_corner_pnts_[i];
		//	fh << "v " << start_pnt.x << " " << start_pnt.y << " " << 1.0 << std::endl;
		//}
		//std::cout << "contour_index_.size() = " << contour_index_.size() << std::endl;
		//for (int i = 0; i < contour_index_.size() - 1; i++) {
		//	std::cout << "xxxxxxxxxxxxxx " << i << std::endl;
		//	for (int j = contour_index_[i]; j < contour_index_[i + 1] - 1; j++) {
		//		std::cout << "xxxxxxxxxjxxxx " << j << std::endl;
		//		fh << "l " << j + 1 << " " << j + 2 << "\n";
		//	}
		//	fh << "l " << contour_index_[i + 1] << " " << contour_index_[i] + 1 << "\n";
		//}
		//std::cout << "before close" << std::endl;
		//fh.close();
		//std::cout << "after close" << std::endl;

		//if (corner_pnts_3d_.empty()) return;
		std::cout << "begin UpdateInformation" << std::endl;
		std::cout << "begin generateworldcoordtoplanecoordmatrix" << std::endl;
		GenerateWorldCoordToPlaneCoordMatrixByCornerPnts3d();
		std::cout << "end generateworldcoordtoplanecoordmatrix" << std::endl;
		corner_pnts_.clear();
		std::cerr << "the corner_pnts_3d_ num: " << corner_pnts_3d_.size() << std::endl;
		for (int i = 0; i < corner_pnts_3d_.size(); i++) {
			Eigen::Vector4f pnt(corner_pnts_3d_[i].x, corner_pnts_3d_[i].y, corner_pnts_3d_[i].z, 1.0);
			Eigen::Vector4f transform_pnt = world_to_plane_ * pnt;
			corner_pnts_.emplace_back(make_float2(transform_pnt(0), transform_pnt(1)));
		}
		for (int i = 0; i < inner_polygon_num_; i++) {
			inner_corner_pnts_[i].clear();
			for (int j = 0; j < inner_corner_pnts_3d_[i].size(); j++) {
				Eigen::Vector4f pnt(inner_corner_pnts_3d_[i][j].x, inner_corner_pnts_3d_[i][j].y, inner_corner_pnts_3d_[i][j].z, 1.0);
				Eigen::Vector4f transform_pnt = world_to_plane_ * pnt;
				inner_corner_pnts_[i].emplace_back(make_float2(transform_pnt(0), transform_pnt(1)));
			}
		}
		std::cout << "UpdateInformation running..." << std::endl;
		boxMin_2d_ = Eigen::Vector2f(FLT_MAX, FLT_MAX), boxMax_2d_ = Eigen::Vector2f(-FLT_MAX, -FLT_MAX);
		for (int i = 0; i < corner_pnts_.size(); i++) {
			if (corner_pnts_[i].x < boxMin_2d_(0))
				boxMin_2d_(0) = corner_pnts_[i].x;
			if (corner_pnts_[i].y < boxMin_2d_(1))
				boxMin_2d_(1) = corner_pnts_[i].y;
			if (corner_pnts_[i].x > boxMax_2d_(0))
				boxMax_2d_(0) = corner_pnts_[i].x;
			if (corner_pnts_[i].y > boxMax_2d_(1))
				boxMax_2d_(1) = corner_pnts_[i].y;
		}
		plane_width_ = min(boxMax_2d_(0) - boxMin_2d_(0), boxMax_2d_(1) - boxMin_2d_(1));
		plane_height_ = max(boxMax_2d_(0) - boxMin_2d_(0), boxMax_2d_(1) - boxMin_2d_(1));
		float length_x = boxMax_2d_(0) - boxMin_2d_(0), length_y = boxMax_2d_(1) - boxMin_2d_(1);
		corner_pnts_uv_.clear();
		for (int i = 0; i < corner_pnts_.size(); i++) {
			float2 uv;
			uv.x = (corner_pnts_[i].x - boxMin_2d_(0)) / length_x;
			uv.y = (corner_pnts_[i].y - boxMin_2d_(1)) / length_y;
			corner_pnts_uv_.emplace_back(uv);
		}


#ifdef LIULINGFEI
		std::cout << "begin contour_index" << std::endl;
#endif
		all_corner_pnts_.clear();
		all_corner_pnts_3d_.clear();
		all_corner_pnts_.insert(all_corner_pnts_.end(), corner_pnts_.begin(), corner_pnts_.end());
		all_corner_pnts_3d_.insert(all_corner_pnts_3d_.end(), corner_pnts_3d_.begin(), corner_pnts_3d_.end());
		contour_index_.clear();
		contour_index_.resize(inner_polygon_num_ + 2);
		contour_index_[0] = 0;
		contour_index_[1] = corner_pnts_3d_.size();
		for (int i = 0; i < inner_polygon_num_; i++) {
			all_corner_pnts_.insert(all_corner_pnts_.end(), inner_corner_pnts_[i].begin(), inner_corner_pnts_[i].end());
			all_corner_pnts_3d_.insert(all_corner_pnts_3d_.end(), inner_corner_pnts_3d_[i].begin(), inner_corner_pnts_3d_[i].end());
			contour_index_[i + 2] = contour_index_[i + 1] + inner_corner_pnts_[i].size();
		}

#ifdef LIULINGFEI
		std::cout << "corner_pnts:" << std::endl;
		for (int i = 0; i < corner_pnts_.size(); i++) {
			std::cout << corner_pnts_[i].x << " " << corner_pnts_[i].y << std::endl;
		}
		std::cout << std::endl;
		for (int i = 0; i < inner_polygon_num_; i++) {
			for (int j = 0; j < inner_corner_pnts_[i].size(); j++) {
				std::cout << inner_corner_pnts_[i][j].x << " " << inner_corner_pnts_[i][j].y << std::endl;
			}
			std::cout << std::endl;
		}
#endif
		/*std::ofstream fh;
		fh.open(filename_ + "all_corner_pnts_after.obj");
		for (int i = 0; i < all_corner_pnts_3d_.size(); i++)
		{
			float2 start_pnt = all_corner_pnts_[i];
			fh << "v " << start_pnt.x << " " << start_pnt.y << " " << 1.0 << std::endl;
		}
		for (int i = 0; i < contour_index_.size() - 1; i++) {
			for (int j = contour_index_[i]; j < contour_index_[i + 1] - 1; j++) {
				fh << "l " << j + 1 << " " << j + 2 << "\n";
			}
			fh << "l " << contour_index_[i + 1] << " " << contour_index_[i] + 1 << "\n";
		}
		fh.close();*/

		//DoTriangulationRefined();
		DoTriangulation();
		
		ComputeArea();
		corner_angels_.clear();
		for (int j = 0; j < corner_pnts_.size(); j++) {
			int next = (j + 1) % corner_pnts_.size();
			int pre = (j - 1) % corner_pnts_.size();
			Eigen::Vector2f e1(corner_pnts_[next].x - corner_pnts_[j].x, corner_pnts_[next].y - corner_pnts_[j].y);
			Eigen::Vector2f e2(corner_pnts_[pre].x - corner_pnts_[j].x, corner_pnts_[pre].y - corner_pnts_[j].y);
			float dot = e1.dot(e2);
			float norm1 = e1.norm(), norm2 = e2.norm();
			float theta = acos(dot / norm1 / norm2) / MATH_PI * 180;
			corner_angels_.emplace_back(theta);
		}
		CalcMinDists();
		std::cout << "end UpdateInformation" << std::endl;
	}

	void HWPlane::DoPolygonSampling(std::vector<float3>& sample_vertices, int density)
	{
		Polygon_2 polygon;
		for (int i = 0; i < corner_pnts_.size(); i++) {
			polygon.push_back(Point2(corner_pnts_[i].x, corner_pnts_[i].y));
		}
		cdt_polygon_.insert_constraint(polygon.vertices_begin(), polygon.vertices_end(), true);
		mark_domains1(cdt_polygon_);
		int point_density = density;
		//int point_density = 100;  // 设置点密度 ori:10000
        //if (outdoorScene) {
            //point_density = 100;    // 室外100点/平米
        //}
		for (Face_handle f : cdt_polygon_.finite_face_handles())
		{
			std::vector<Point2> tri_vertices;
			if (f->info().in_domain())
			{
				std::vector<Point2> tri;
				for (int i = 0; i < 3; ++i)
				{
					tri.emplace_back(f->vertex(f->ccw(i))->point());
				}
				SamplePntsFromTri(tri_vertices, tri, point_density);
			}

			for (int i = 0; i < tri_vertices.size(); ++i)
			{
				Eigen::Vector4f pnt(tri_vertices[i][0], tri_vertices[i][1], 0.0, 1.0);
				Eigen::Vector4f transform_pnt = plane_to_world_*pnt;
				sample_vertices.emplace_back(make_float3(transform_pnt(0), transform_pnt(1), transform_pnt(2)));
			}
		}
	}

	float HWPlane::RayToPolygonDist(Eigen::Vector3f & ray_o, Eigen::Vector3f & ray_direct, int& nearest_pnt_idx)
	{
		Eigen::Vector3f polygon_n = plane_to_world_.block(0, 2, 3, 1);
		Eigen::Vector3f polygon_p = plane_to_world_.block(0, 3, 3, 1);
		float t = polygon_n.dot(polygon_p - ray_o) / (polygon_n.dot(ray_direct));
		if (t < 0)
			return -1.0f;
		Eigen::Vector3f p = ray_o + t*ray_direct;
		float2 p_proj = TransformWorld2Plane(p);
		int c = IsInPolygon(p_proj);
		if (c) {
			float dist = (p - ray_o).norm();
			float dist_min = FLT_MAX;
			for (int i = 0; i < corner_pnts_.size(); i++) {
				float pp_dist = PointPointDist(p_proj, corner_pnts_[i]);
				if (pp_dist < dist_min) {
					dist_min = pp_dist;
					nearest_pnt_idx = i;
				}
			}
			return dist;
		}
		else 
			return -1.0f;
	}

	bool HWPlane::RayToPolyPlanePntPos(Eigen::Vector3f& ray_o, Eigen::Vector3f& ray_direct, Eigen::Vector2f& proj_p)
	{
		//
		Eigen::Vector3f polygon_n = plane_to_world_.block(0, 2, 3, 1);
		Eigen::Vector3f polygon_p = plane_to_world_.block(0, 3, 3, 1);
		float t = polygon_n.dot(polygon_p - ray_o) / (polygon_n.dot(ray_direct));
		if (t < 0)
		{
			std::cout << "the t: " << t << std::endl;
			return false;
		}
		Eigen::Vector3f p = ray_o + t*ray_direct;
		float2 p_proj = TransformWorld2Plane(p);
		proj_p = Eigen::Vector2f(p_proj.x, p_proj.y);
		std::cout << "the proj p is: " << proj_p[0] << " " << proj_p[1] << std::endl;
		return true;
	}
	bool HWPlane::RayToPolyIntersectionPnt(Eigen::Vector3f& ray_o, Eigen::Vector3f& ray_direct, Eigen::Vector3f& pnt)
	{
		Eigen::Vector3f polygon_n = plane_to_world_.block(0, 2, 3, 1);
		Eigen::Vector3f polygon_p = plane_to_world_.block(0, 3, 3, 1);
		float t = polygon_n.dot(polygon_p - ray_o) / (polygon_n.dot(ray_direct));
		if (t < 0)
		{
			std::cout << "the t: " << t << std::endl;
			return false;
		}
		pnt = ray_o + t * ray_direct;
		return true;
	}

	bool HWPlane::RayToPolyPlaneIntersectionPntConst(const Eigen::Vector3f& ray_o, const Eigen::Vector3f& ray_direct, Eigen::Vector3f& pnt)
	{
		Eigen::Vector3f polygon_n = plane_to_world_.block(0, 2, 3, 1);
		Eigen::Vector3f polygon_p = plane_to_world_.block(0, 3, 3, 1);
		float t = polygon_n.dot(polygon_p - ray_o) / (polygon_n.dot(ray_direct));
		if (t < 0)
		{
			std::cout << "the t: " << t << std::endl;
			return false;
		}
		pnt = ray_o + t * ray_direct;
		return true;
	}

	bool HWPlane::RayLinePnt2PolygonPntThreshold(const Eigen::Vector3f& oc, const Eigen::Vector3f& odir, Eigen::Vector3f& pnt3d)
	{
		//to do next filter it(cam dir and plane dir is weight that is considered), now ignore its
		//double d = -plane_.center_.dot(plane_.nornmal_);
		Eigen::Vector3f polygon_n = plane_to_world_.block(0, 2, 3, 1);
		Eigen::Vector3f polygon_p = plane_to_world_.block(0, 3, 3, 1);
		//double d = -polygon_n.dot(polygon_p);
		//Eigen::Vector4f fn = Eigen::Vector4f(polygon_n[0], polygon_n[1], polygon_n[2], d);
		bool f = LineIntersectPlanePntN(oc, odir, polygon_p, polygon_n, pnt3d);
		if (f)
		{
			//check it in polygon
			//to 2d coord
			//Eigen::Vector2d pnt2d = Pnt3dToPnt2d(pnt3d);
			bool polyin = IsPointInPolygon3D(pnt3d);
			if (polyin)
			{
				return true;
			}
			else
			{
				//check it to all polygon line segment
				Eigen::Vector2f pnt2d;
				Pnt3d2Pnt2D(pnt3d, pnt2d);
				float mindist = Pnt2PolygonMinDist2D(pnt2d);
				if (mindist < r_poly_dist_max_)
				{
					return true;
				}
				return false;
			}
		}
		else
		{
			return false;
		}
	}

	bool HWPlane::IsDegeneration(int idx, Eigen::Vector3f& L_dir, Eigen::Vector3f& L_point)
	{
		idx = idx%corner_pnts_.size();
		float2 p = corner_pnts_[idx];
		float2 p_proj = ProjToLine3D(idx, L_dir, L_point);
		float2 p_pre = corner_pnts_[(idx + corner_pnts_.size() - 1) % corner_pnts_.size()];
		float2 p_next = corner_pnts_[(idx + 1) % corner_pnts_.size()];
		Eigen::Vector2f coeff = FittingLine(p_pre,p_next);
		//std::cout << p.x*coeff(0) + p.y*coeff(1) - 1 << "  " << p_proj.x*coeff(0) + p_proj.y*coeff(1) - 1 << "\n";
		int c = IsInPolygon(p_proj);
		//如果投影前后，点idx在其相邻两个顶点之间的直线的两边，并且投影后的点不再原多边形中，并且点idx不是凹点
		//或者投影后发生自相交的情况
		//则多边形退化
		if (((p.x*coeff(0) + p.y*coeff(1) - 1)*(p_proj.x*coeff(0) + p_proj.y*coeff(1) - 1) <= 0 && c==0 && (!IsConcaveVertex(idx)))
			|| IsSelfIntersection(idx,p_proj))
			return true;
		return false;
	}

	bool HWPlane::IsDegenerationIgnoreConvexPnts(int idx, Eigen::Vector3f& L_dir, Eigen::Vector3f& L_point)
	{
		//处理这些顶点
		idx = idx%corner_pnts_.size();
		float2 p = corner_pnts_[idx];
		float2 p_proj = ProjToLine3D(idx, L_dir, L_point);
		float2 p_pre = corner_pnts_[(idx + corner_pnts_.size() - 1) % corner_pnts_.size()];
		float2 p_next = corner_pnts_[(idx + 1) % corner_pnts_.size()];
		Eigen::Vector2f coeff = FittingLine(p_pre, p_next);

		//Eigen::Vector3f coeffd = CreateLineFunctionFrom2Pnts(p_pre, p_next);
		//std::cout << p.x*coeff(0) + p.y*coeff(1) - 1 << "  " << p_proj.x*coeff(0) + p_proj.y*coeff(1) - 1 << "\n";
		int c = IsInPolygon(p_proj);
		//如果投影前后，点idx在其相邻两个顶点之间的直线的两边，并且投影后的点不再原多边形中，并且点idx不是凹点
		//或者投影后发生自相交的情况
		//则多边形退化
		if (((p.x*coeff(0) + p.y*coeff(1) - 1)*(p_proj.x*coeff(0) + p_proj.y*coeff(1) - 1) <= 0 && c == 0 && (!IsConcaveVertex(idx)))
			|| IsSelfIntersection(idx, p_proj)) //&& (!IsConcaveVertex(idx))
			return true;
		return false;
	}

	bool HWPlane::FindLineSegIdxInLSVec(std::vector<Eigen::Vector2i>& vec, Eigen::Vector2i& l)
	{
		//
		for (int i = 0; i < vec.size(); ++i)
		{
			if (vec[i] == l)
				return true;
		}
		return false;
	}

	float HWPlane::GetPolyon3DLength(int i, int j)
	{
		if (i >= corner_pnts_3d_.size() || i < 0
			|| j >= corner_pnts_3d_.size() || j < 0)
		{
			std::cout << "GetPolyon3DLength: out of range" << std::endl;
			return -1.0;
		}
		Eigen::Vector3f s_pnt = Eigen::Vector3f(corner_pnts_3d_[i].x, corner_pnts_3d_[i].y, corner_pnts_3d_[i].z);
		Eigen::Vector3f e_pnt = Eigen::Vector3f(corner_pnts_3d_[j].x, corner_pnts_3d_[j].y, corner_pnts_3d_[j].z);
		float dist = (e_pnt - s_pnt).norm();
		return dist;
	}

	bool HWPlane::FindValueInVec(std::vector<int>& vec, int v)
	{
		for (int i = 0; i < vec.size(); ++i)
		{
			if (vec[i] == v)
				return true;
		}
		return false;
	}

	void HWPlane::BuildLSIdxFromPolygonIdxsVec(std::vector<int>& polygon_idxs, std::vector<Eigen::Vector2i>& linesegidx)
	{
		if (polygon_idxs.size() < 2)
			return;
		for (int i = 0; i < polygon_idxs.size() - 1; ++i)
		{
			int preidx = i % polygon_idxs.size();
			int postidx = (i + 1) % polygon_idxs.size();
			Eigen::Vector2i tmp = Eigen::Vector2i(polygon_idxs[preidx], polygon_idxs[postidx]);
			linesegidx.emplace_back(tmp);
		}
	}

	void HWPlane::ComputeTwoPointsIdxCorrespondingLine(Eigen::Vector3f& L_dir, Eigen::Vector3f& L_point, Eigen::Vector2i& pnts_idxs)
	{
		//
		Eigen::Vector4f temp = world_to_plane_.topLeftCorner(3, 3)*L_dir;
		float2 L_dir_2D = make_float2(temp(0), temp(1));
		temp = world_to_plane_*Eigen::Vector4f(L_point(0), L_point(1), L_point(2), 1.0f);
		float2 L_point_2D = make_float2(temp(0), temp(1));

		/*std::cout << "the lpnt2d pnt: " << L_point_2D.x << " " << L_point_2D.y <<
			",  ldir2d dir: " << L_dir_2D.x << " " << L_dir_2D.y << std::endl;
		std::cout << "the L_dir_2D dot value: " << dot(L_dir_2D, L_dir_2D) << std::endl;*/

		float min_k = std::numeric_limits<float>::max();
		float max_k = std::numeric_limits<float>::lowest();	//这个bug很隐蔽
		//std::cout << "the min_k: " << min_k << std::endl;
		//std::cout << "the max_k: " << max_k << std::endl;

		int min_k_idx = -1;
		int max_k_idx = -1;
		//对这些数据进行处理, 寻找在线段的两个端点
		for (int i = 0; i < corner_pnts_.size(); ++i)
		{
			float2 p2L_point = corner_pnts_[i] - L_point_2D;
			float k = dot(p2L_point, L_dir_2D) / dot(L_dir_2D, L_dir_2D);
			//std::cout << "ComputeTwoPointsIdxCorrespondingLine k: " << k << std::endl;

			if (k > max_k)
			{
				max_k_idx = i;
				max_k = k;
			}
			if (k < min_k)
			{
				min_k_idx = i;
				min_k = k;
			}
			//float2 p_proj = L_point_2D + dot(p2L_point, L_dir_2D) / dot(L_dir_2D, L_dir_2D)*L_dir_2D;
		}

		//std::cout << "min k idx: " << min_k_idx << " ; max k idx: " << max_k_idx << std::endl;

		pnts_idxs[0] = min_k_idx;
		pnts_idxs[1] = max_k_idx;
	}

	void HWPlane::ComputeLeftRightPointsIdxLine2D(Eigen::Vector2f& L_dir2d, Eigen::Vector2f& L_point2d, Eigen::Vector2i& pnts_idxs)
	{
		//
		float min_k = std::numeric_limits<float>::max();
		float max_k = std::numeric_limits<float>::lowest();
		int min_k_idx = -1;
		int max_k_idx = -1;
		//对这些数据进行处理, 寻找在线段的两个端点
		for (int i = 0; i < corner_pnts_.size(); ++i)
		{
			//std::cout << "ComputeTwoPointsIdxCorrespondingLine k: " << k << std::endl;

			Eigen::Vector2f pnt2d = Eigen::Vector2f(corner_pnts_[i].x, corner_pnts_[i].y);
			Eigen::Vector2f p2L_point = pnt2d - L_point2d;
			float k = p2L_point.dot(L_dir2d) / (L_dir2d.norm()*L_dir2d.norm());

			if (k > max_k)
			{
				max_k_idx = i;
				max_k = k;
			}
			if (k < min_k)
			{
				min_k_idx = i;
				min_k = k;
			}
			//float2 p_proj = L_point_2D + dot(p2L_point, L_dir_2D) / dot(L_dir_2D, L_dir_2D)*L_dir_2D;
		}

		//std::cout << "min k idx: " << min_k_idx << " ; max k idx: " << max_k_idx << std::endl;

		pnts_idxs[0] = min_k_idx;
		pnts_idxs[1] = max_k_idx;
	}

	bool HWPlane::ComputeLeftRightPointsPosLine2D(Eigen::Vector2f& L_dir2d, Eigen::Vector2f& L_point2d,
		Eigen::Vector2i& pnts_idxs, Eigen::Vector2f& lpnt, Eigen::Vector2f& rpnt)
	{
		//
		float min_k = std::numeric_limits<float>::max();
		float max_k = std::numeric_limits<float>::lowest();
		int min_k_idx = -1;
		Eigen::Vector2f min_pnt;
		int max_k_idx = -1;
		Eigen::Vector2f max_pnt;
		//对这些数据进行处理, 寻找在线段的两个端点
		for (int i = 0; i < corner_pnts_.size(); ++i)
		{
			Eigen::Vector2f pnt2d = Eigen::Vector2f(corner_pnts_[i].x, corner_pnts_[i].y);
			Eigen::Vector2f p2L_point = pnt2d - L_point2d;
			float k = p2L_point.dot(L_dir2d) / (L_dir2d.norm()*L_dir2d.norm());
			Eigen::Vector2f p_proj = L_point2d +  k * L_dir2d;
			if (k > max_k)
			{
				max_k_idx = i;
				max_k = k;
				max_pnt = p_proj;
			}
			if (k < min_k)
			{
				min_k_idx = i;
				min_k = k;
				min_pnt = p_proj;
			}
		}
		//std::cout << "min k idx: " << min_k_idx << " ; max k idx: " << max_k_idx << std::endl;
		if (min_k_idx == -1 || max_k_idx == -1)
			return false;
		pnts_idxs[0] = min_k_idx;
		pnts_idxs[1] = max_k_idx;
		lpnt = min_pnt;
		rpnt = max_pnt;
		return true;
	}

	void HWPlane::ComputeLRPointsWithFlagIdxLine2D(Eigen::Vector2f& L_dir2d, Eigen::Vector2f& L_point2d,
		std::vector<bool>& valided_vec, Eigen::Vector2i& pnts_idxs)
	{
		//
		float min_k = std::numeric_limits<float>::max();
		float max_k = std::numeric_limits<float>::lowest();
		int min_k_idx = -1;
		int max_k_idx = -1;
		//对这些数据进行处理, 寻找在线段的两个端点
		for (int i = 0; i < corner_pnts_.size(); ++i)
		{
			if (!valided_vec[i])
				continue;
			//std::cout << "ComputeTwoPointsIdxCorrespondingLine k: " << k << std::endl;
			Eigen::Vector2f pnt2d = Eigen::Vector2f(corner_pnts_[i].x, corner_pnts_[i].y);
			Eigen::Vector2f p2L_point = pnt2d - L_point2d;
			float k = p2L_point.dot(L_dir2d) / (L_dir2d.norm()*L_dir2d.norm());
			if (k > max_k)
			{
				max_k_idx = i;
				max_k = k;
			}
			if (k < min_k)
			{
				min_k_idx = i;
				min_k = k;
			}
			//float2 p_proj = L_point_2D + dot(p2L_point, L_dir_2D) / dot(L_dir_2D, L_dir_2D)*L_dir_2D;
		}
		pnts_idxs[0] = min_k_idx;
		pnts_idxs[1] = max_k_idx;
	}

	float HWPlane::ComputeProjPRelativeDist2LPntDir2d(Eigen::Vector2f& pnt2d, Eigen::Vector2f& L_dir2d, Eigen::Vector2f& L_point2d)
	{
		if (L_dir2d.norm() < 1e-8)
			return std::numeric_limits<float>::max();
		Eigen::Vector2f p2L_point = pnt2d - L_point2d;
		float k = p2L_point.dot(L_dir2d) / (L_dir2d.norm()*L_dir2d.norm());
		return k;
	}

	void HWPlane::ComputeLRPntsIdxLine2DFromSelectPoly(Eigen::Vector2f& L_dir2d, Eigen::Vector2f& L_point2d,
		std::vector<int>& split_idxs, Eigen::Vector2i& pnts_idxs)
	{
		//
		float min_k = std::numeric_limits<float>::max();
		float max_k = std::numeric_limits<float>::lowest();
		int min_k_idx = -1;
		int max_k_idx = -1;

		//对这些数据进行处理, 寻找在线段的两个端点
		for (int i = 0; i < split_idxs.size(); ++i)
		{
			Eigen::Vector2f pnt2d = Eigen::Vector2f(corner_pnts_[split_idxs[i]].x, corner_pnts_[split_idxs[i]].y);
			Eigen::Vector2f p2L_point = pnt2d - L_point2d;
			float k = p2L_point.dot(L_dir2d) / (L_dir2d.norm()*L_dir2d.norm());
			//std::cout << "ComputeTwoPointsIdxCorrespondingLine k: " << k << std::endl;

			if (k > max_k)
			{
				max_k_idx = split_idxs[i];
				max_k = k;
			}
			if (k < min_k)
			{
				min_k_idx = split_idxs[i];
				min_k = k;
			}
			//float2 p_proj = L_point_2D + dot(p2L_point, L_dir_2D) / dot(L_dir_2D, L_dir_2D)*L_dir_2D;
		}

		//std::cout << "min k idx: " << min_k_idx << " ; max k idx: " << max_k_idx << std::endl;
		pnts_idxs[0] = min_k_idx;
		pnts_idxs[1] = max_k_idx;
	}

	void HWPlane::ComputePolygonMoveIdxs2Line(Eigen::Vector3f& L_dir, Eigen::Vector3f& L_point, std::vector<int>& no_move_idxs)
	{
#if 1
		if (corner_pnts_.size() < 2)
			return;
		Eigen::Vector2f lpnt2d, ldir2d;
		ComputeWorldLf2PlaneLf(L_dir, L_point, ldir2d, lpnt2d);
		std::cout << "the lpnt2d pnt: " << lpnt2d[0] << " " << lpnt2d[1] <<
			",  ldir2d dir: " << ldir2d[0] << " " << ldir2d[1] << std::endl;
		Eigen::Vector2i pnts_lr_idx(-1, -1);
		ComputeLeftRightPointsIdxLine2D(ldir2d, lpnt2d, pnts_lr_idx);

		/*std::string path1 = "D:\\vc_project_new\\huawei_data_indoor\\room\\test_room_cam4\\lr_split_scene.obj";
		std::ofstream fh(path1);
		fh << "v " << corner_pnts_3d_[pnts_lr_idx[0]].x << " " << corner_pnts_3d_[pnts_lr_idx[0]].y << " "
			<< corner_pnts_3d_[pnts_lr_idx[0]].z << std::endl;
		fh << "v " << corner_pnts_3d_[pnts_lr_idx[1]].x << " " << corner_pnts_3d_[pnts_lr_idx[1]].y << " "
			<< corner_pnts_3d_[pnts_lr_idx[1]].z << std::endl;
		fh << "l " << 1 << " " << 2 << std::endl;
		fh.close();*/

		if (pnts_lr_idx[0] != -1 && pnts_lr_idx[1] != -1)
		{
			//pnts_lr_idx 将polygon切分为polygon1_new_purn, polygon2_new_purn;
			std::vector<int> polygon1_new_purn, polygon2_new_purn;

			//在polygon搜索的时候，优先访问到idx_in_splitidx1这个部分
			if (pnts_lr_idx[0] < pnts_lr_idx[1])
			{
				//形成新的circle polygon1
				for (int i = pnts_lr_idx[0]; i <= pnts_lr_idx[1]; ++i)
				{
					polygon1_new_purn.emplace_back(i);
				}
				//形成新的circle polygon2
				for (int i = pnts_lr_idx[1]; i <= corner_pnts_.size() + pnts_lr_idx[0]; ++i)
				{
					//
					int idx_curr = i%corner_pnts_.size();
					polygon2_new_purn.emplace_back(idx_curr);
				}
			}
			else
			{
				//形成新的circle polygon1
				for (int i = pnts_lr_idx[1]; i <= pnts_lr_idx[0]; ++i)
				{
					polygon1_new_purn.emplace_back(i);
				}
				//形成新的circle polygon2
				for (int i = pnts_lr_idx[0]; i <= corner_pnts_.size() + pnts_lr_idx[1]; ++i)
				{
					//
					int idx_curr = i%corner_pnts_.size();
					polygon2_new_purn.emplace_back(idx_curr);
				}
			}

			//获取平面坐标系下的垂直与split polygon的方程:为起始顶点为m_lpnt, 方向为：m_ldir
			Eigen::Vector2f m_lpnt, m_ldir;
			std::cout << "the source polygon pnts lr idx: " << pnts_lr_idx[0] << " " << pnts_lr_idx[1] << std::endl;
			ComputeVerticalLineFromTwoPnts2D(pnts_lr_idx, lpnt2d, ldir2d, m_lpnt, m_ldir);

			//计算直线m_lpnt, m_ldir到polygon2_new_purn， polygon1_new_purn的交线，以及它的线段
			float l2poly1max = std::numeric_limits<float>::lowest();
			Eigen::Vector2i polygon1_faridx(-1, -1);
			for (int j = 0, i = j + 1; j < polygon1_new_purn.size() - 1; ++j, i = j + 1)
			{
				//j,i为一个线段
				Eigen::Vector2f ls = Eigen::Vector2f(corner_pnts_[polygon1_new_purn[j]].x, corner_pnts_[polygon1_new_purn[j]].y);
				Eigen::Vector2f le = Eigen::Vector2f(corner_pnts_[polygon1_new_purn[i]].x, corner_pnts_[polygon1_new_purn[i]].y);
				//计算线段和直线是否有交点
				Eigen::Vector2f tmp_cross_pnt;
				if (ComputeLineSeg2DCrossLine2DPntsNew(ls, le, m_lpnt, m_ldir, tmp_cross_pnt))
				{
					//获取这个交点，并且计算它和lpnt2d, ldir2d距离
					float dist2line = Pnt2DDistToLine2D(tmp_cross_pnt, lpnt2d, ldir2d);
					std::cout << "the split 1 cross pnt: " << tmp_cross_pnt[0] << " " << tmp_cross_pnt[1] << std::endl;
					std::cout << "the split 1 dist 2 line: " << dist2line << std::endl;

					if (l2poly1max < dist2line)
					{
						polygon1_faridx = Eigen::Vector2i(j, i);
						l2poly1max = dist2line;
					}
				}
			}

			float l2poly2max = std::numeric_limits<float>::lowest();
			Eigen::Vector2i polygon2_faridx(-1, -1);
			for (int j = 0, i = j + 1; j < polygon2_new_purn.size() - 1; ++j, i = j + 1)
			{
				//j,i为一个线段
				Eigen::Vector2f ls = Eigen::Vector2f(corner_pnts_[polygon2_new_purn[j]].x, corner_pnts_[polygon2_new_purn[j]].y);
				Eigen::Vector2f le = Eigen::Vector2f(corner_pnts_[polygon2_new_purn[i]].x, corner_pnts_[polygon2_new_purn[i]].y);
				//计算线段和直线是否有交点
				Eigen::Vector2f tmp_cross_pnt;
				if (ComputeLineSeg2DCrossLine2DPntsNew(ls, le, m_lpnt, m_ldir, tmp_cross_pnt))
				{
					//获取这个交点，并且计算它和lpnt2d, ldir2d距离
					float dist2line = Pnt2DDistToLine2D(tmp_cross_pnt, lpnt2d, ldir2d);
					std::cout << "the split 2 cross pnt: " << tmp_cross_pnt[0] << " " << tmp_cross_pnt[1] << std::endl;
					std::cout << "the split 2 dist 2 line: " << dist2line << std::endl;

					if (l2poly2max < dist2line)
					{
						polygon2_faridx = Eigen::Vector2i(j, i);
						l2poly2max = dist2line;
					}
				}
			}

			if (l2poly1max > 1e-6 && l2poly2max > 1e-6)
			{
				if (l2poly1max > l2poly2max)
				{
					no_move_idxs = polygon1_new_purn;
				}
				else
				{
					no_move_idxs = polygon2_new_purn;
				}
			}
			else if (l2poly1max < 1e-6 && l2poly2max > 1e-6)
			{
				no_move_idxs = polygon2_new_purn;
			}
			else if (l2poly1max > 1e-6 && l2poly2max < 1e-6)
			{
				no_move_idxs = polygon1_new_purn;
			}
			else
			{
				std::cout << "the wrong polygon, to do next..." << std::endl;
				std::vector<int> my_new_polygon_idxs;
				for (int i = 0; i < corner_pnts_.size(); ++i)
				{
					my_new_polygon_idxs.emplace_back(i);
				}
				no_move_idxs = my_new_polygon_idxs;
			}

			//no_move_idxs = polygon1_new;
			std::cout << "the no move idx: ";
			for (int i = 0; i < no_move_idxs.size(); ++i)
			{
				std::cout << no_move_idxs[i] << " ";
			}
			std::cout << std::endl;
		}

#else
		//
		Eigen::Vector2i pnts_lr_idx;
		//计算两个端点
		ComputeTwoPointsIdxCorrespondingLine(L_dir, L_point, pnts_lr_idx);
		
		/*std::cout << "the L_point3d pnt: " << L_point[0] << " " << L_point[1] << " " << L_point[2] << 
		",  L_dir3d dir: " << L_dir[0] << " " << L_dir[1] << " " << L_dir[2] << std::endl;*/

		Eigen::Vector2f lpnt2d, ldir2d;
		ComputeWorldLf2PlaneLf(L_dir, L_point, ldir2d, lpnt2d);
		std::cout << "the lpnt2d pnt: " << lpnt2d[0] << " " << lpnt2d[1] <<
			",  ldir2d dir: " << ldir2d[0] << " " << ldir2d[1] << std::endl;

		//获取平面坐标系下的垂直与polygon的方程:为起始顶点为m_lpnt, 方向为：m_ldir
		Eigen::Vector2f m_lpnt, m_ldir;
		std::cout << "the pnts lr idx: " << pnts_lr_idx[0] << " " << pnts_lr_idx[1] << std::endl;
		ComputeVerticalLineFromTwoPnts2D(pnts_lr_idx, lpnt2d, ldir2d, m_lpnt, m_ldir);
		
		std::cout << "the lpnt2d pnt: " << lpnt2d[0] << " " << lpnt2d[1] <<
			",  ldir2d dir: " << ldir2d[0] << " " << ldir2d[1] << std::endl;
		std::cout << "the mean pnt: " << m_lpnt[0] << " " << m_lpnt[1] << 
			",  mean dir: " << m_ldir[0] << " " << m_ldir[1] << std::endl;

		//获取直线和polygon的线段的交点，保存这些线段的顶点索引,求得这些顶点和最初的直线的距离
		//获取最近和最远的顶点的距离, far_near_idxs[0],最近的边；far_near_idxs[1],最远的边
		std::vector<Eigen::Vector2i> far_near_idxs;
		far_near_idxs.resize(2);
		float dist_min = std::numeric_limits<float>::max();
		float dist_max = std::numeric_limits<float>::min();
		for (int i = 0, j = corner_pnts_.size() - 1; i < corner_pnts_.size(); j = i++)
		{
			//j,i为一个线段
			Eigen::Vector2f ls = Eigen::Vector2f(corner_pnts_[j].x, corner_pnts_[j].y);
			Eigen::Vector2f le = Eigen::Vector2f(corner_pnts_[i].x, corner_pnts_[i].y);
			//计算线段和直线是否有交点
			Eigen::Vector2f tmp_cross_pnt;
			if (ComputeLineSeg2DCrossLine2DPnts(ls, le, m_lpnt, m_ldir, tmp_cross_pnt))
			{
				//获取这个交点，并且计算它和lpnt2d, ldir2d距离
				float dist2line = Pnt2DDistToLine2D(tmp_cross_pnt, lpnt2d, ldir2d);
				std::cout << "the cross pnt: " << tmp_cross_pnt[0] << " " << tmp_cross_pnt[1] << std::endl;
				std::cout << "the dist 2 line: " << dist2line << std::endl;

				if (dist_min > dist2line)
				{
					far_near_idxs[0] = Eigen::Vector2i(j, i);
					dist_min = dist2line;
				}
				if (dist_max < dist2line)
				{
					far_near_idxs[1] = Eigen::Vector2i(j, i);
					dist_max = dist2line;
				}
			}
		}

		std::cout << "the near idx: " << far_near_idxs[0][0] << " " << far_near_idxs[0][1] << std::endl;
		
		//std::cout << "the far idx: " << far_near_idxs[1][0] << " " << far_near_idxs[1][1] << std::endl;
		//std::string path1 = "D:\\vc_project_new\\huawei_data_indoor\\room\\test_room_cam\\line_outdoor_scene.obj";
		//std::ofstream fh(path1);
		//fh << "v " << corner_pnts_3d_[far_near_idxs[0][0]].x << " " << corner_pnts_3d_[far_near_idxs[0][0]].y << " "
		//	<< corner_pnts_3d_[far_near_idxs[0][0]].z << std::endl;
		//fh << "v " << corner_pnts_3d_[far_near_idxs[0][1]].x << " " << corner_pnts_3d_[far_near_idxs[0][1]].y << " "
		//	<< corner_pnts_3d_[far_near_idxs[0][1]].z << std::endl;
		//fh << "v " << corner_pnts_3d_[far_near_idxs[1][0]].x << " " << corner_pnts_3d_[far_near_idxs[1][0]].y << " "
		//	<< corner_pnts_3d_[far_near_idxs[1][0]].z << std::endl;
		//fh << "v " << corner_pnts_3d_[far_near_idxs[1][1]].x << " " << corner_pnts_3d_[far_near_idxs[1][1]].y << " "
		//	<< corner_pnts_3d_[far_near_idxs[1][1]].z << std::endl;
		////这是最左和最右
		//fh << "v " << corner_pnts_3d_[pnts_lr_idx[0]].x << " " << corner_pnts_3d_[pnts_lr_idx[0]].y << " "
		//	<< corner_pnts_3d_[pnts_lr_idx[0]].z << std::endl;
		//fh << "v " << corner_pnts_3d_[pnts_lr_idx[1]].x << " " << corner_pnts_3d_[pnts_lr_idx[1]].y << " "
		//	<< corner_pnts_3d_[pnts_lr_idx[1]].z << std::endl;
		//fh << "l " << 1 << " " << 2 << std::endl;
		//fh << "l " << 3 << " " << 4 << std::endl;
		//fh.close();

		//处理这些顶点
		std::vector<int> polygon_idxs1, polygon_idxs2; //存放polygon的idx
		bool p2_flag = false;
		for (int i = pnts_lr_idx[0]; i < corner_pnts_.size() + pnts_lr_idx[0]; ++i)
		{
			//恢复这个顶点
			int idx = i % corner_pnts_.size();
			if (idx == pnts_lr_idx[1])
			{
				p2_flag = true;
			}
			if (p2_flag)
			{
				polygon_idxs2.emplace_back(idx);
			}
			else
			{
				polygon_idxs1.emplace_back(idx);
			}
		}
		
		/*std::cout << "the polygon1 idx begin: " << std::endl;
		for (int i = 0; i < polygon_idxs1.size(); ++i)
		{
			std::cout << polygon_idxs1[i] << " ";
		}
		std::cout << std::endl;
		std::cout << "the polygon2 idx begin: " << std::endl;
		for (int i = 0; i < polygon_idxs2.size(); ++i)
		{
			std::cout << polygon_idxs2[i] << " ";
		}
		std::cout << std::endl;*/

		//构成polygon的线段, 添加
		std::vector<int> polygon1_idxs, polygon2_idxs;
		for (int i = 0; i < polygon_idxs1.size(); ++i)
		{
			polygon1_idxs.emplace_back(polygon_idxs1[i]);
		}
		polygon1_idxs.emplace_back(pnts_lr_idx[1]);
		for (int i = 0; i < polygon_idxs2.size(); ++i)
		{
			polygon2_idxs.emplace_back(polygon_idxs2[i]);
		}
		polygon2_idxs.emplace_back(pnts_lr_idx[0]);

		std::cout << "the polygon1_idxs idx begin: " << std::endl;
		for (int i = 0; i < polygon1_idxs.size(); ++i)
		{
			std::cout << polygon1_idxs[i] << " ";
		}
		std::cout << std::endl;
		std::cout << "the polygon2_idxs idx begin: " << std::endl;
		for (int i = 0; i < polygon2_idxs.size(); ++i)
		{
			std::cout << polygon2_idxs[i] << " ";
		}
		std::cout << std::endl;

		std::vector<Eigen::Vector2i> linevec1, linevec2;
		BuildLSIdxFromPolygonIdxsVec(polygon1_idxs, linevec1);
		BuildLSIdxFromPolygonIdxsVec(polygon2_idxs, linevec2);
		
		/*for (int i = 0; i < linevec1.size(); ++i)
		{
			std::cout << "polygon1 line idx: " << linevec1[i][0] << " " << linevec1[i][1] << std::endl;
		}
		for (int i = 0; i < linevec2.size(); ++i)
		{
			std::cout << "polygon2 line idx: " << linevec2[i][0] << " " << linevec2[i][1] << std::endl;
		}*/

		//对平面坐标系的polygon进行处理
		//离得远的
		//std::vector<Eigen::Vector2f> polygon_new;//需要构建新的polygon
		if (FindLineSegIdxInLSVec(linevec1, far_near_idxs[1]))
		{
			std::cout << "the polygon1: started: " << std::endl;
			no_move_idxs = polygon1_idxs;
		}
		if (FindLineSegIdxInLSVec(linevec2, far_near_idxs[1]))
		{
			//
			std::cout << "the polygon2: started: " << std::endl;
			no_move_idxs = polygon2_idxs;
		}
#endif
	}

	bool HWPlane::ComputePolygonMoveIdxsFromline2d(Eigen::Vector2f& ldir2d, Eigen::Vector2f& lpnt2d, std::vector<int>& move_idxs)
	{
		Eigen::Vector2i pnts_lr_idx(-1, -1);
		ComputeLeftRightPointsIdxLine2D(ldir2d, lpnt2d, pnts_lr_idx);

		if (pnts_lr_idx[0] != -1 && pnts_lr_idx[1] != -1)
		{
			//pnts_lr_idx 将polygon切分为polygon1_new_purn, polygon2_new_purn;
			std::vector<int> polygon1_new_purn, polygon2_new_purn;

			//在polygon搜索的时候，优先访问到idx_in_splitidx1这个部分
			if (pnts_lr_idx[0] < pnts_lr_idx[1])
			{
				//形成新的circle polygon1
				for (int i = pnts_lr_idx[0]; i <= pnts_lr_idx[1]; ++i)
				{
					polygon1_new_purn.emplace_back(i);
				}
				//形成新的circle polygon2
				for (int i = pnts_lr_idx[1]; i <= corner_pnts_.size() + pnts_lr_idx[0]; ++i)
				{
					//
					int idx_curr = i%corner_pnts_.size();
					polygon2_new_purn.emplace_back(idx_curr);
				}
			}
			else
			{
				//形成新的circle polygon1
				for (int i = pnts_lr_idx[1]; i <= pnts_lr_idx[0]; ++i)
				{
					polygon1_new_purn.emplace_back(i);
				}
				//形成新的circle polygon2
				for (int i = pnts_lr_idx[0]; i <= corner_pnts_.size() + pnts_lr_idx[1]; ++i)
				{
					//
					int idx_curr = i%corner_pnts_.size();
					polygon2_new_purn.emplace_back(idx_curr);
				}
			}

			//获取平面坐标系下的垂直与split polygon的方程:为起始顶点为m_lpnt, 方向为：m_ldir
			Eigen::Vector2f m_lpnt, m_ldir;
			std::cout << "the source polygon pnts lr idx: " << pnts_lr_idx[0] << " " << pnts_lr_idx[1] << std::endl;
			ComputeVerticalLineFromTwoPnts2D(pnts_lr_idx, lpnt2d, ldir2d, m_lpnt, m_ldir);

			//计算直线m_lpnt, m_ldir到polygon2_new_purn， polygon1_new_purn的交线，以及它的线段
			float l2poly1max = std::numeric_limits<float>::lowest();
			Eigen::Vector2i polygon1_faridx(-1, -1);
			for (int j = 0, i = j + 1; j < polygon1_new_purn.size() - 1; ++j, i = j + 1)
			{
				//j,i为一个线段
				Eigen::Vector2f ls = Eigen::Vector2f(corner_pnts_[polygon1_new_purn[j]].x, corner_pnts_[polygon1_new_purn[j]].y);
				Eigen::Vector2f le = Eigen::Vector2f(corner_pnts_[polygon1_new_purn[i]].x, corner_pnts_[polygon1_new_purn[i]].y);
				//计算线段和直线是否有交点
				Eigen::Vector2f tmp_cross_pnt;
				if (ComputeLineSeg2DCrossLine2DPntsNew(ls, le, m_lpnt, m_ldir, tmp_cross_pnt))
				{
					//获取这个交点，并且计算它和lpnt2d, ldir2d距离
					float dist2line = Pnt2DDistToLine2D(tmp_cross_pnt, lpnt2d, ldir2d);
					//std::cout << "the split 1 cross pnt: " << tmp_cross_pnt[0] << " " << tmp_cross_pnt[1] << std::endl;
					//std::cout << "the split 1 dist 2 line: " << dist2line << std::endl;

					if (l2poly1max < dist2line)
					{
						polygon1_faridx = Eigen::Vector2i(j, i);
						l2poly1max = dist2line;
					}
				}
			}

			float l2poly2max = std::numeric_limits<float>::lowest();
			Eigen::Vector2i polygon2_faridx(-1, -1);
			for (int j = 0, i = j + 1; j < polygon2_new_purn.size() - 1; ++j, i = j + 1)
			{
				//j,i为一个线段
				Eigen::Vector2f ls = Eigen::Vector2f(corner_pnts_[polygon2_new_purn[j]].x, corner_pnts_[polygon2_new_purn[j]].y);
				Eigen::Vector2f le = Eigen::Vector2f(corner_pnts_[polygon2_new_purn[i]].x, corner_pnts_[polygon2_new_purn[i]].y);
				//计算线段和直线是否有交点
				Eigen::Vector2f tmp_cross_pnt;
				if (ComputeLineSeg2DCrossLine2DPntsNew(ls, le, m_lpnt, m_ldir, tmp_cross_pnt))
				{
					//获取这个交点，并且计算它和lpnt2d, ldir2d距离
					float dist2line = Pnt2DDistToLine2D(tmp_cross_pnt, lpnt2d, ldir2d);
					//std::cout << "the split 2 cross pnt: " << tmp_cross_pnt[0] << " " << tmp_cross_pnt[1] << std::endl;
					//std::cout << "the split 2 dist 2 line: " << dist2line << std::endl;

					if (l2poly2max < dist2line)
					{
						polygon2_faridx = Eigen::Vector2i(j, i);
						l2poly2max = dist2line;
					}
				}
			}
			std::cerr << "l2poly1max, l2poly2max: " << l2poly1max << " " << l2poly2max << std::endl;
			if (l2poly1max > 1e-6 && l2poly2max > 1e-6)
			{
				if (l2poly1max > l2poly2max)
				{
					move_idxs = polygon2_new_purn;
				}
				else
				{
					move_idxs = polygon1_new_purn;
				}
			}
			else if (l2poly1max < 1e-6 && l2poly2max > 1e-6)
			{
				move_idxs = polygon1_new_purn;
				return true;
			}
			else if (l2poly1max > 1e-6 && l2poly2max < 1e-6)
			{
				move_idxs = polygon2_new_purn;
				return true;
			}
			else
			{
				std::cout << "the wrong polygon, to do next..." << std::endl;
				std::vector<int> my_new_polygon_idxs;
				for (int i = 0; i < corner_pnts_.size(); ++i)
				{
					my_new_polygon_idxs.emplace_back(i);
				}
				move_idxs = my_new_polygon_idxs;
				return false;
			}

			//no_move_idxs = polygon1_new;
			std::cout << "the move idx: ";
			for (int i = 0; i < move_idxs.size(); ++i)
			{
				std::cout << move_idxs[i] << " ";
			}
			std::cout << std::endl;
		}
		return false;
	}

	void HWPlane::ComputeSelectedPolygonMoveIdxs2Line(Eigen::Vector3f& L_dir, Eigen::Vector3f& L_point,
		std::vector<int>& split_idxs, std::vector<int>& no_move_idxs, Eigen::Vector2i& my_lr_idx)
	{
#if 1
		if (split_idxs.size() < 2)
			return;

		Eigen::Vector2f lpnt2d, ldir2d;
		ComputeWorldLf2PlaneLf(L_dir, L_point, ldir2d, lpnt2d);
		std::cout << "the lpnt2d pnt: " << lpnt2d[0] << " " << lpnt2d[1] <<
			",  ldir2d dir: " << ldir2d[0] << " " << ldir2d[1] << std::endl;

		Eigen::Vector2i pnts_lr_idx(-1, -1);
		ComputeLRPntsIdxLine2DFromSelectPoly(ldir2d, lpnt2d, split_idxs, pnts_lr_idx);

		/*std::string path1 = "D:\\vc_project_new\\huawei_data_indoor\\room\\test_room_cam1\\lr_split_scene.obj";
		std::ofstream fh(path1);
		fh << "v " << corner_pnts_3d_[pnts_lr_idx[0]].x << " " << corner_pnts_3d_[pnts_lr_idx[0]].y << " "
			<< corner_pnts_3d_[pnts_lr_idx[0]].z << std::endl;
		fh << "v " << corner_pnts_3d_[pnts_lr_idx[1]].x << " " << corner_pnts_3d_[pnts_lr_idx[1]].y << " "
			<< corner_pnts_3d_[pnts_lr_idx[1]].z << std::endl;
		fh << "l " << 1 << " " << 2 << std::endl;
		fh.close();*/

		//只要找到split polygon最左和最右的端点idx
		if (pnts_lr_idx[0] != -1 && pnts_lr_idx[1] != -1)
		{
			//将polygon切分成两个顶点集合
			int idx_in_splitidx1 = -1;
			int idx_in_splitidx2 = -1;
			for (int i = 0; i < split_idxs.size(); ++i)
			{
				//处理这个polygon
				if (split_idxs[i] == pnts_lr_idx[0])
				{
					idx_in_splitidx1 = i;
				}
				if (split_idxs[i] == pnts_lr_idx[1])
				{
					idx_in_splitidx2 = i;
				}
			}
			if (idx_in_splitidx1 == -1 || idx_in_splitidx2 == -1)
				return;

			if (idx_in_splitidx1 == idx_in_splitidx2)
				return;

			std::vector<int> polygon1_new, polygon2_new;
			std::vector<int> polygon1_new_purn, polygon2_new_purn;
			//在polygon搜索的时候，优先访问到idx_in_splitidx1这个部分
			if (idx_in_splitidx1 < idx_in_splitidx2)
			{
				//形成新的circle polygon1
				if (idx_in_splitidx1 != 0)
					polygon1_new.emplace_back(split_idxs[0]);
				for (int i = idx_in_splitidx1; i <= idx_in_splitidx2; ++i)
				{
					polygon1_new.emplace_back(split_idxs[i]);
					polygon1_new_purn.emplace_back(split_idxs[i]);
				}
				//
				if (idx_in_splitidx2 != split_idxs.size() - 1)
					polygon1_new.emplace_back(split_idxs[split_idxs.size() - 1]);

				//被pnts_lr_idx两个端点切分的另一个polygon2
				/*polygon2_new.emplace_back(split_idxs[idx_in_splitidx2]);
				if (idx_in_splitidx2 != split_idxs.size() - 1)
					polygon2_new.emplace_back(split_idxs[idx_in_splitidx2]);*/
				for (int i = idx_in_splitidx2; i <= split_idxs.size() + idx_in_splitidx1; ++i)
				{
					//
					int idx_curr = i%split_idxs.size();
					polygon2_new.emplace_back(split_idxs[idx_curr]);
					polygon2_new_purn.emplace_back(split_idxs[idx_curr]);
				}
				/*if (idx_in_splitidx1 != 0)
					polygon2_new.emplace_back(split_idxs[0]);*/
			}
			else
			{
				//
				//形成新的circle polygon
				if (idx_in_splitidx2 != 0)
					polygon1_new.emplace_back(split_idxs[0]);
				for (int i = idx_in_splitidx2; i <= idx_in_splitidx1; ++i)
				{
					polygon1_new.emplace_back(split_idxs[i]);
					polygon1_new_purn.emplace_back(split_idxs[i]);
				}
				//
				if (idx_in_splitidx1 != split_idxs.size() - 1)
					polygon1_new.emplace_back(split_idxs[split_idxs.size() - 1]);

				//被pnts_lr_idx两个端点切分的另一个polygon2
				/*if (idx_in_splitidx1 != split_idxs.size() - 1)
					polygon2_new.emplace_back(split_idxs[split_idxs.size() - 1]);*/
				for (int i = idx_in_splitidx1; i <= split_idxs.size() + idx_in_splitidx2; ++i)
				{
					//
					int idx_curr = i%split_idxs.size();
					polygon2_new.emplace_back(split_idxs[idx_curr]);
					polygon2_new_purn.emplace_back(split_idxs[idx_curr]);
				}
				/*if (idx_in_splitidx2 != 0)
					polygon2_new.emplace_back(split_idxs[0]);*/
			}

			//通过获取两个polygon的
			//获取平面坐标系下的垂直与split polygon的方程:为起始顶点为m_lpnt, 方向为：m_ldir
			Eigen::Vector2f m_lpnt, m_ldir;
			std::cout << "the split pnts lr idx: " << pnts_lr_idx[0] << " " << pnts_lr_idx[1] << std::endl;
			ComputeVerticalLineFromTwoPnts2D(pnts_lr_idx, lpnt2d, ldir2d, m_lpnt, m_ldir);

			//计算直线m_lpnt, m_ldir到polygon2_new_purn， polygon1_new_purn的交线，以及它的线段
			float l2poly1max = std::numeric_limits<float>::lowest();
			Eigen::Vector2i polygon1_faridx(-1, -1);
			for (int j = 0, i = j + 1; j < polygon1_new_purn.size() - 1; ++j, i = j+1)
			{
				//j,i为一个线段
				Eigen::Vector2f ls = Eigen::Vector2f(corner_pnts_[polygon1_new_purn[j]].x, corner_pnts_[polygon1_new_purn[j]].y);
				Eigen::Vector2f le = Eigen::Vector2f(corner_pnts_[polygon1_new_purn[i]].x, corner_pnts_[polygon1_new_purn[i]].y);
				//计算线段和直线是否有交点
				Eigen::Vector2f tmp_cross_pnt;
				if (ComputeLineSeg2DCrossLine2DPntsNew(ls, le, m_lpnt, m_ldir, tmp_cross_pnt))
				{
					//获取这个交点，并且计算它和lpnt2d, ldir2d距离
					float dist2line = Pnt2DDistToLine2D(tmp_cross_pnt, lpnt2d, ldir2d);
					std::cout << "the split 1 cross pnt: " << tmp_cross_pnt[0] << " " << tmp_cross_pnt[1] << std::endl;
					std::cout << "the split 1 dist 2 line: " << dist2line << std::endl;

					if (l2poly1max < dist2line)
					{
						polygon1_faridx = Eigen::Vector2i(j, i);
						l2poly1max = dist2line;
					}
				}
			}

			float l2poly2max = std::numeric_limits<float>::lowest();
			Eigen::Vector2i polygon2_faridx(-1, -1);
			for (int j = 0, i = j + 1; j < polygon2_new_purn.size() - 1; ++j, i = j + 1)
			{
				//j,i为一个线段
				Eigen::Vector2f ls = Eigen::Vector2f(corner_pnts_[polygon2_new_purn[j]].x, corner_pnts_[polygon2_new_purn[j]].y);
				Eigen::Vector2f le = Eigen::Vector2f(corner_pnts_[polygon2_new_purn[i]].x, corner_pnts_[polygon2_new_purn[i]].y);
				//计算线段和直线是否有交点
				Eigen::Vector2f tmp_cross_pnt;
				if (ComputeLineSeg2DCrossLine2DPntsNew(ls, le, m_lpnt, m_ldir, tmp_cross_pnt))
				{
					//获取这个交点，并且计算它和lpnt2d, ldir2d距离
					float dist2line = Pnt2DDistToLine2D(tmp_cross_pnt, lpnt2d, ldir2d);
					std::cout << "the split 2 cross pnt: " << tmp_cross_pnt[0] << " " << tmp_cross_pnt[1] << std::endl;
					std::cout << "the split 2 dist 2 line: " << dist2line << std::endl;

					if (l2poly2max < dist2line)
					{
						polygon2_faridx = Eigen::Vector2i(j, i);
						l2poly2max = dist2line;
					}
				}
			}

			if (l2poly1max > 1e-6 && l2poly2max > 1e-6)
			{
				if (l2poly1max > l2poly2max)
				{
					no_move_idxs = polygon1_new_purn;
				}
				else
				{
					no_move_idxs = polygon2_new_purn;
				}
			}
			else if (l2poly1max < 1e-6 && l2poly2max > 1e-6)
			{
				no_move_idxs = polygon2_new_purn;
			}
			else if (l2poly1max > 1e-6 && l2poly2max < 1e-6)
			{
				no_move_idxs = polygon1_new_purn;
			}
			else
			{
				std::cout << "the wrong polygon, to do next..." << std::endl;
				no_move_idxs = split_idxs;
			}

			//no_move_idxs = polygon1_new;
			std::cout << "the no move idx: ";
			for (int i = 0; i < no_move_idxs.size(); ++i)
			{
				std::cout << no_move_idxs[i] << " ";
			}
			std::cout << std::endl;

			if (no_move_idxs.size() >= 2)
			{
				if (no_move_idxs[0] == pnts_lr_idx[0] &&
					no_move_idxs[no_move_idxs.size() - 1] == pnts_lr_idx[1])
				{
					my_lr_idx = pnts_lr_idx;
				}
				else if (no_move_idxs[0] == pnts_lr_idx[1] &&
					no_move_idxs[no_move_idxs.size() - 1] == pnts_lr_idx[0])
				{
					my_lr_idx[0] = pnts_lr_idx[1];
					my_lr_idx[1] = pnts_lr_idx[0];
				}
				else
				{
					std::cout << "the no split idx...." << std::endl;
					return;
				}
			}
			//my_lr_idx = pnts_lr_idx;
		}
#endif
	}

	void HWPlane::ComputeSelectedPolygonNoMoveIdxs2LineSeam(Eigen::Vector2f& l_dir2d, Eigen::Vector2f& l_pnt2d, std::vector<int>& selected_idxs, 
		float dist_threshold, std::vector<bool>& dists2l_flag, Eigen::Vector2i& my_lr_idx)
	{
		dists2l_flag.resize(selected_idxs.size());
		for (int i = 0; i < dists2l_flag.size(); ++i)
		{
			dists2l_flag[i] = false;
		}
		int nom_in_mcount = 0;
		std::vector<float> dist2ls;
		for (int i = 0; i < selected_idxs.size(); ++i)
		{
			Eigen::Vector2f scltpnt = Eigen::Vector2f(corner_pnts_[selected_idxs[i]].x, corner_pnts_[selected_idxs[i]].y);
			float dist2l = Pnt2DDistToLine2D(scltpnt, l_pnt2d, l_dir2d);
			std::cout << "ComputeSelectedPolygonNoMoveIdxs2LineSeam Dist: " << dist2l << ", dist_threshold: " << dist_threshold << std::endl;
			if (dist2l < dist_threshold)
			{
				dists2l_flag[i] = true;
			}
			dist2ls.emplace_back(dist2l);
		}
		int icount_i = 0;
		for (int i = 0; i < dist2ls.size(); ++i)
		{
			if (i == 0)
			{
				if (dists2l_flag[i])
				{
					my_lr_idx[0] = i;
					++icount_i;
				}
			}
			else
			{
				int prei = i - 1;
				if (!dists2l_flag[prei] && dists2l_flag[i] && !icount_i)
				{
					my_lr_idx[0] = i;
					++icount_i;
				}
				else if(dists2l_flag[prei] && !dists2l_flag[i] && icount_i)
				{
					my_lr_idx[1] = prei;
					++icount_i;
				}
			}
		}
	}

	void HWPlane::ComputeSelectedPolygonNoMoveIdxs2LineSegmentSeam(Eigen::Vector2f& ls, Eigen::Vector2f& le, std::vector<int>& selected_idxs, float dist_threshold,
		std::vector<bool>& dists2l_flag, Eigen::Vector2i& my_lr_idx)
	{
		dists2l_flag.resize(selected_idxs.size());
		for (int i = 0; i < dists2l_flag.size(); ++i)
		{
			dists2l_flag[i] = false;
		}
		int nom_in_mcount = 0;
		std::vector<float> dist2ls;
		for (int i = 0; i < selected_idxs.size(); ++i)
		{
			Eigen::Vector2f scltpnt = Eigen::Vector2f(corner_pnts_[selected_idxs[i]].x, corner_pnts_[selected_idxs[i]].y);
			float dist2l = Pnt2DToLineSegment2D(scltpnt, ls, le);
			std::cout << "ComputeSelectedPolygonNoMoveIdxs2LineSegmentSeam Dist: " << dist2l << std::endl;
			if (dist2l < dist_threshold)
			{
				dists2l_flag[i] = true;
			}
			dist2ls.emplace_back(dist2l);
		}
		int icount_i = 0;
		for (int i = 0; i < dist2ls.size(); ++i)
		{
			if (i == 0)
			{
				if (dists2l_flag[i])
				{
					my_lr_idx[0] = i;
					++icount_i;
				}
			}
			else
			{
				int prei = i - 1;
				if (!dists2l_flag[prei] && dists2l_flag[i] && !icount_i)
				{
					my_lr_idx[0] = i;
					++icount_i;
				}
				else if (dists2l_flag[prei] && !dists2l_flag[i] && icount_i)
				{
					my_lr_idx[1] = prei;
					++icount_i;
				}
			}
		}
	}

	void HWPlane::ComputePolygonMoveLRIdxs2LineSeam(Eigen::Vector2f& l_dir2d, Eigen::Vector2f& l_pnt2d, float dist_threshold, Eigen::Vector2i& my_lr_idx)
	{
		my_lr_idx = Eigen::Vector2i(-1, -1);
		std::vector<bool> move_flag;
		move_flag.resize(corner_pnts_.size());
		for (int i = 0; i < move_flag.size(); ++i)
		{
			move_flag[i] = false;
		}
		//获取这些顶点的首尾顶点
		int fst_idx = -1;
		float fstdist = std::numeric_limits<float>::lowest();
		std::vector<float> dist2ls;
		for (int i = 0; i < corner_pnts_.size(); ++i)
		{
			Eigen::Vector2f scltpnt = Eigen::Vector2f(corner_pnts_[i].x, corner_pnts_[i].y);
			float dist2l = Pnt2DDistToLine2D(scltpnt, l_pnt2d, l_dir2d);
			std::cout << "ComputePolygonMoveLRIdxs2LineSeam Dist: " << dist2l << std::endl;
			if (dist2l < dist_threshold)
			{
				move_flag[i] = true;
			}
			if (dist2l > fstdist)
			{
				fstdist = dist2l;
				fst_idx = i;
			}
			dist2ls.emplace_back(dist2l);
		}
		ComputeLRPointsWithFlagIdxLine2D(l_dir2d, l_pnt2d, move_flag, my_lr_idx);
	}

	void HWPlane::ComputeAllPolyNewPntsFromNoMoveIdxs2DSeam(Eigen::Vector2f& l_dir2d, Eigen::Vector2f& l_pnt2d, 
		std::vector<bool>& dists2l_flag, std::vector<Eigen::Vector2f>& polygonnewpnts)
	{
		//新的polygon进行自我相交的判断
		std::vector<Eigen::Vector2f> current_polygon;
		for (int i = 0; i < corner_pnts_.size(); ++i)
		{
			Eigen::Vector2f tmp_pnt2d = Eigen::Vector2f(corner_pnts_[i].x, corner_pnts_[i].y);
			current_polygon.emplace_back(tmp_pnt2d);
		}
		std::vector<Eigen::Vector2f> mynewpnts;
		for (int i = 0; i < dists2l_flag.size(); ++i)
		{
			Eigen::Vector2f tmp_pnt2d = Eigen::Vector2f(corner_pnts_[i].x, corner_pnts_[i].y);
			if (dists2l_flag[i])
			{
				//投影
				Eigen::Vector2f projpnt;
				Pnt2dProjLine2D(tmp_pnt2d, l_pnt2d, l_dir2d, projpnt);
				//current_polygon[i] = projpnt;
				//check polygon 是否相交
				if (!IsSelectedAreaIncreasing(current_polygon, i, projpnt))
					//|| IsSelfIntersectionInPolygon(current_polygon, i, projpnt))
				{
					//恢复为原来的polygon
					std::cout << "!IsSelectedAreaIncreasing: " << i << std::endl;
					//current_polygon[i] = tmp_pnt2d;
					mynewpnts.emplace_back(tmp_pnt2d);
				}
				else
				{
					//更新原来的polygon
					current_polygon[i] = projpnt;
					mynewpnts.emplace_back(projpnt);
				}
			}
			else
			{
				mynewpnts.emplace_back(tmp_pnt2d);
			}
		}
		polygonnewpnts = mynewpnts;
	}

	//后续再处理这些函数，明天做
	void HWPlane::ComputeSelectPolyNewPntsFromNoMoveIdxs2DSeam(Eigen::Vector2f& l_dir2d, Eigen::Vector2f& l_pnt2d,
		std::vector<int>& poly_idxs, std::vector<bool>& dists2l_flag, std::vector<Eigen::Vector2f>& polygonnewpnts)
	{
		//新的polygon进行自我相交的判断
		std::vector<int> polysplit1, polysplit2;
		bool split_flag = false;
		bool poly2_flag = false;
		split_flag = SplitSelectedPolyFromInterLNew(l_dir2d, l_pnt2d, poly_idxs, polysplit1, polysplit2);
		if (!polysplit2.empty())
		{
			//表示出现了
			std::cout << "polysplit2 existing... " << std::endl;
			poly2_flag = true;
		}
		else
		{
			std::cout << "polysplit2 empty... " << std::endl;
			poly2_flag = false;
		}
		std::vector<Eigen::Vector2f> current_polygon;
		for (int i = 0; i < poly_idxs.size(); ++i)
		{
			Eigen::Vector2f tmp_pnt2d = Eigen::Vector2f(corner_pnts_[poly_idxs[i]].x, corner_pnts_[poly_idxs[i]].y);
			current_polygon.emplace_back(tmp_pnt2d);
		}
		int icount_move = 0;
		for (int i = 0; i < dists2l_flag.size(); ++i)
		{
			if(dists2l_flag[i])
				++icount_move;
		}
		std::vector<Eigen::Vector2f> mynewpnts;
		for (int i = 0; i < dists2l_flag.size(); ++i)
		{
			Eigen::Vector2f tmp_pnt2d = Eigen::Vector2f(corner_pnts_[poly_idxs[i]].x, corner_pnts_[poly_idxs[i]].y);
			if (dists2l_flag[i])
			{
				//投影
				Eigen::Vector2f projpnt;
				Pnt2dProjLine2D(tmp_pnt2d, l_pnt2d, l_dir2d, projpnt);
				//current_polygon[i] = projpnt;
				//check polygon 是否相交
				if (!IsSelectedAreaIncreasing(current_polygon, i, projpnt) 
					&& icount_move != 1)	
					//|| IsSelfIntersectionInPolygon(current_polygon, poly_idxs[i], projpnt))
				{
					if (poly2_flag) //如果poly2_flag为真，表示它是外点，这种可以面积缩小
					{
						//std::cout << "the i: " << i << " " << std::endl;
						if (std::find(polysplit2.begin(), polysplit2.end(), i) != polysplit2.end())
						{
							//更新原来的polygon
							current_polygon[i] = projpnt;
							std::cout <<"the " << i << ": poly2_flag->change one pnt to proj pnt..." << std::endl;
							mynewpnts.emplace_back(projpnt);
						}
						else
						{
							//恢复为原来的polygon
							std::cout << "!poly2_flag->IsSelectedAreaIncreasing poly_idxs[i]: " << poly_idxs[i] << std::endl;
							//current_polygon[i] = tmp_pnt2d;
							mynewpnts.emplace_back(tmp_pnt2d);
						}
					}
					else
					{
						//恢复为原来的polygon
						std::cout << "!IsSelectedAreaIncreasing poly_idxs[i]: " << poly_idxs[i] << std::endl;
						//current_polygon[i] = tmp_pnt2d;
						mynewpnts.emplace_back(tmp_pnt2d);
					}
				}
				else
				{
					//更新原来的polygon
					current_polygon[i] = projpnt;
					std::cout << "the " << i << ": poly2_flag->change one pnt to proj pnt..." << std::endl;
					mynewpnts.emplace_back(projpnt);
				}
			}
			else
			{
				mynewpnts.emplace_back(tmp_pnt2d);
			}
		}
		polygonnewpnts = mynewpnts;
	}

	void HWPlane::ComputeTwoPolygonConnectIdx(std::vector<int>& source_idxs, std::vector<int>& target_idxs,
		Eigen::Vector2i& connect_idxs)
	{
		if (source_idxs.empty())
			return;

		std::vector<int> polygon_c;
		//处理这两个polygon连接处的idx
		for (int i = 0; i < source_idxs.size(); ++i)
		{
			int idx_corner = source_idxs[i];
			int idx_corner_next = idx_corner + 1;
			int idx_corner_pre = idx_corner - 1;
			if (idx_corner_next == corner_pnts_.size())
				idx_corner_next = 0;
			if (idx_corner == 0)
				idx_corner_pre = corner_pnts_.size() - 1;

			//如果在自己的vector里面找不到idx_corner_next表示它是边界点，在另一个polygon中
			if (FindValueInVec(target_idxs, idx_corner_next) && 
				!FindValueInVec(source_idxs, idx_corner_next))
			{
				std::cout << "the tmp next connect idx: " << idx_corner << std::endl;
				polygon_c.emplace_back(idx_corner);
			}

			if (FindValueInVec(target_idxs, idx_corner_pre) &&
				!FindValueInVec(source_idxs, idx_corner_pre))
			{
				std::cout << "the tmp pre connect idx: " << idx_corner << std::endl;
				polygon_c.emplace_back(idx_corner);
			}
		}
		if (polygon_c.size() == 2)
		{
			if (polygon_c[0] > polygon_c[1])
			{
				connect_idxs[0] = polygon_c[1];
				connect_idxs[1] = polygon_c[0];
			}
			else
			{
				connect_idxs[0] = polygon_c[0];
				connect_idxs[1] = polygon_c[1];
			}
		}
		else if (polygon_c.size() > 2)
		{
			connect_idxs[0] = polygon_c[0];
			connect_idxs[1] = polygon_c[polygon_c.size() - 1];
		}
		else 
		{
			std::cout << "no connect polygon idx...." << std::endl;
		}
	}
	/*
	目的：当前的polygon为sor polygon;目标polygon为j的polygon
	*/
	void HWPlane::GetExpandTgtIntersectionLine(int j, Eigen::Vector3f& L_dir, Eigen::Vector3f& L_point)
	{
		if (j == splited_label_)
		{
			std::cerr << "same plane..." << std::endl;
			return;
		}

		float4 coeff1 = coeff_;
		float4 coeff2 = model_planes_[j]->GetPlaneCoeff();
		Eigen::Vector3f n1(coeff1.x, coeff1.y, coeff1.z);
		Eigen::Vector3f n2(coeff2.x, coeff2.y, coeff2.z);
		L_dir = n1.cross(n2);
		//下面是对轴的选着，用于计算初始的顶点
		int flag[3] = { -1,-1,-1 };
		int flag_min_idx = -1;
		float flag_min = std::numeric_limits<float>::max();
		for (int k = 0; k < 3; k++)
		{
			float sum_tmp = std::abs(n1(k)) + std::abs(n2(k));
			if (sum_tmp < flag_min)
			{
				flag_min = sum_tmp;
				flag_min_idx = k;
			}
		}
		if (flag_min_idx == 0)
		{
			flag[0] = 1;
		}
		else if (flag_min_idx == 1)
		{
			flag[1] = 1;
		}
		else if (flag_min_idx == 2)
		{
			flag[2] = 1;
		}
		Eigen::Matrix2f A;
		Eigen::Vector3f pos = plane_to_world_.topRightCorner(3, 1);
		//std::cout << "cam pos: " << pos.transpose() << std::endl;
		//std::cout << "n1 pos: " << n1.transpose() << std::endl;
		//std::cout << "n2 pos: " << n2.transpose() << std::endl;
		if (flag[0] == 1) {
			A << n1[1], n1[2], n2[1], n2[2];
			//std::cout << "A-0: \n" << A << std::endl;
			Eigen::Vector2f B(-coeff1.w - pos[0] * n1[0], -coeff2.w - pos[0] * n2[0]);
			Eigen::Vector2f point = A.inverse()*B;
			L_point = Eigen::Vector3f(pos[0], point(0), point(1));
		}
		else if (flag[1] == 1) {
			A << n1[0], n1[2], n2[0], n2[2];
			//std::cout << "A-1: \n" << A << std::endl;
			Eigen::Vector2f B(-coeff1.w - pos[1] * n1[1], -coeff2.w - pos[1] * n2[1]);
			Eigen::Vector2f point = A.inverse()*B;
			L_point = Eigen::Vector3f(point(0), pos[1], point(1));
		}
		else {
			A << n1[0], n1[1], n2[0], n2[1];
			//std::cout << "A-2: \n" << A << std::endl;
			Eigen::Vector2f B(-coeff1.w - pos[2] * n1[2], -coeff2.w - pos[2] * n2[2]);
			Eigen::Vector2f point = A.inverse()*B;
			L_point = Eigen::Vector3f(point(0), point(1), pos[2]);
		}
	}

	void HWPlane::GetExpandTgtCoeffIntersectionLine(float4& coeff, Eigen::Vector3f& L_dir, Eigen::Vector3f& L_point)
	{

	}

	bool HWPlane::SplitPolygonFromIntersectionLineNew(Eigen::Vector3f& L_dir, Eigen::Vector3f& L_point,
		std::vector<int>& split_idxs1, std::vector<int>& split_idxs2)
	{
		//
		Eigen::Vector2f lpnt2d, ldir2d;
		ComputeWorldLf2PlaneLf(L_dir, L_point, ldir2d, lpnt2d);
		std::cout << "the lpnt2d pnt: " << lpnt2d[0] << " " << lpnt2d[1] <<
			",  ldir2d dir: " << ldir2d[0] << " " << ldir2d[1] << std::endl;
		//获取垂直于直线的ldir2d, lpnt2d的直线vertical_pnt, vertical_dir
		Eigen::Vector2f vertical_dir = Eigen::Vector2f(-ldir2d[1], ldir2d[0]);
		Eigen::Vector2f vertical_pnt = lpnt2d;
		Eigen::Vector2i left_right_idxs = Eigen::Vector2i(-1, -1);

		for (int i = 0; i < corner_pnts_.size(); ++i)
		{
			Eigen::Vector2f pnt2d = Eigen::Vector2f(corner_pnts_[i].x, corner_pnts_[i].y);
			Eigen::Vector2f p2L_point = pnt2d - vertical_pnt;
			float k = p2L_point.dot(vertical_dir) / (vertical_dir.norm()*vertical_dir.norm());

			if (k < 1e-8)
				split_idxs1.emplace_back(i);
			else
				split_idxs2.emplace_back(i);
		}

		if (split_idxs1.size() <= 0 || split_idxs2.size() <= 0)
			return false;

		//test
		std::cout << "the new split_idxs1 pnts test idx begin: " << std::endl;
		for (int i = 0; i < split_idxs1.size(); ++i)
		{
			std::cout << split_idxs1[i] << " ";
		}
		std::cout << std::endl;
		std::cout << "the new split_idxs2 pnts test idx begin: " << std::endl;
		for (int i = 0; i < split_idxs2.size(); ++i)
		{
			std::cout << split_idxs2[i] << " ";
		}
		std::cout << std::endl;
		//test

		return true;
	}

	bool HWPlane::SplitSelectedPolyFromInterLNew(Eigen::Vector2f& ldir2d, Eigen::Vector2f& lpnt2d, std::vector<int>& poly_idx,
		std::vector<int>& split_idxs1, std::vector<int>& split_idxs2)
	{
		Eigen::Vector2f vertical_dir = Eigen::Vector2f(-ldir2d[1], ldir2d[0]);
		Eigen::Vector2f vertical_pnt = lpnt2d;
		//Eigen::Vector2i left_right_idxs = Eigen::Vector2i(-1, -1);
		std::vector<int> polysplit1, polysplit2;
		for (int i = 0; i < poly_idx.size(); ++i)
		{
			Eigen::Vector2f pnt2d = Eigen::Vector2f(corner_pnts_[poly_idx[i]].x, corner_pnts_[poly_idx[i]].y);
			Eigen::Vector2f p2L_point = pnt2d - vertical_pnt;
			float k = p2L_point.dot(vertical_dir) / (vertical_dir.norm()*vertical_dir.norm());
			if (k < 1e-5)
				polysplit1.emplace_back(i);
			else
				polysplit2.emplace_back(i);
		}
		if (polysplit1.empty() || polysplit2.empty())
		{
			if (!polysplit1.empty())
			{
				std::cout << "polysplit1 existing" << std::endl;
				split_idxs1 = polysplit1;
			}
			if (!polysplit2.empty())
			{
				std::cout << "polysplit2 existing" << std::endl;
				split_idxs1 = polysplit2;
			}	
			return false;
		}
		else
		{
			float split1max = std::numeric_limits<float>::lowest();
			float split2max = std::numeric_limits<float>::lowest();
			for (int i = 0; i < polysplit1.size(); ++i)
			{
				Eigen::Vector2f pnt2d = Eigen::Vector2f(corner_pnts_[poly_idx[polysplit1[i]]].x, corner_pnts_[poly_idx[polysplit1[i]]].y);
				float dist2l = Pnt2DDistToLine2D(pnt2d, lpnt2d, ldir2d);
				if (dist2l > split1max)
				{
					split1max = dist2l;
				}
			}
			for (int i = 0; i < polysplit2.size(); ++i)
			{
				Eigen::Vector2f pnt2d = Eigen::Vector2f(corner_pnts_[poly_idx[polysplit2[i]]].x, corner_pnts_[poly_idx[polysplit2[i]]].y);
				float dist2l = Pnt2DDistToLine2D(pnt2d, lpnt2d, ldir2d);
				if (dist2l > split2max)
				{
					split2max = dist2l;
				}
			}
			if (split1max > split2max)
			{
				split_idxs1 = polysplit1;
				split_idxs2 = polysplit2;
			}
			else
			{
				split_idxs1 = polysplit2;
				split_idxs2 = polysplit1;
			}
			return true;
		}
	}

	bool HWPlane::SplitPolygonIntersectionFromLine(Eigen::Vector3f& L_dir, Eigen::Vector3f& L_point,
		std::vector<int>& split_idxs1, std::vector<int>& split_idxs2)
	{
		//
		Eigen::Vector2f lpnt2d, ldir2d;
		ComputeWorldLf2PlaneLf(L_dir, L_point, ldir2d, lpnt2d);
		std::cout << "the lpnt2d pnt: " << lpnt2d[0] << " " << lpnt2d[1] <<
			",  ldir2d dir: " << ldir2d[0] << " " << ldir2d[1] << std::endl;

		std::vector<Eigen::Vector2i> left_right_idxs;
		left_right_idxs.resize(2);
		left_right_idxs[0] = Eigen::Vector2i(-1, -1);
		left_right_idxs[1] = Eigen::Vector2i(-1, -1);

		float min_k = std::numeric_limits<float>::max();
		float max_k = std::numeric_limits<float>::lowest();
		for (int i = 0, j = corner_pnts_.size() - 1; i < corner_pnts_.size(); j = i++)
		{
			//j,i为一个线段
			Eigen::Vector2f ls = Eigen::Vector2f(corner_pnts_[j].x, corner_pnts_[j].y);
			Eigen::Vector2f le = Eigen::Vector2f(corner_pnts_[i].x, corner_pnts_[i].y);
			//计算线段和直线是否有交点
			Eigen::Vector2f tmp_cross_pnt;
			if (ComputeLineSeg2DCrossLine2DPntsNew(ls, le, lpnt2d, ldir2d, tmp_cross_pnt))
			{
				//如果有交点，就获取交点到
				Eigen::Vector2f p2lpnt = tmp_cross_pnt - lpnt2d;
				float k = p2lpnt.dot(ldir2d) / (ldir2d.norm() * ldir2d.norm());

				if (min_k > k)
				{
					left_right_idxs[0] = Eigen::Vector2i(j, i);
					min_k = k;
				}
				if (max_k < k)
				{
					left_right_idxs[1] = Eigen::Vector2i(j, i);
					max_k = k;
				}
			}
		}

		std::cout << "the left: " << left_right_idxs[0][0] << " " << left_right_idxs[0][1]
			<< " " << left_right_idxs[1][0] << " " << left_right_idxs[1][1] << std::endl;

		if (left_right_idxs[0][0] == -1 || left_right_idxs[0][1] == -1
			|| left_right_idxs[1][0] == -1 || left_right_idxs[1][1] == -1)
			return false;

		//处理这些顶点
		std::vector<int> polygon_idxs1, polygon_idxs2; //存放polygon的idx
		bool p2_flag = false;

		int splitfirsti = -1;
		int splitfirstype = -1;
		for (int i = 0; i < corner_pnts_.size(); ++i)
		{
			if (i == left_right_idxs[0][0])
			{
				splitfirstype = 1;
				break;
			}
			if (i == left_right_idxs[0][1])
			{
				splitfirstype = 2;
				break;
			}
			if (i == left_right_idxs[1][0])
			{
				splitfirstype = 3;
				break;
			}
			if (i == left_right_idxs[1][1])
			{
				splitfirstype = 4;
				break;
			}
		}

		if (splitfirstype == 1)
		{
			//polygon1
			for (int i = left_right_idxs[0][1]; i <= left_right_idxs[1][0]; ++i)
			{
				polygon_idxs1.emplace_back(i);
			}

			//polygon2
			for (int i = left_right_idxs[1][1]; i <= left_right_idxs[0][0] + corner_pnts_.size(); ++i)
			{
				int idx = i % corner_pnts_.size();
				polygon_idxs2.emplace_back(idx);
			}
		}
		else if (splitfirstype == 2)
		{
			if (left_right_idxs[0][0] == corner_pnts_.size() - 1)
			{
				//polygon1
				for (int i = left_right_idxs[0][1]; i <= left_right_idxs[1][0]; ++i)
				{
					polygon_idxs1.emplace_back(i);
				}
				//polygon2
				for (int i = left_right_idxs[1][1]; i <= left_right_idxs[0][0]; ++i)
				{
					int idx = i % corner_pnts_.size();
					polygon_idxs2.emplace_back(idx);
				}
			}
		}
		else if (splitfirstype == 3)
		{
			//polygon1
			for (int i = left_right_idxs[1][1]; i <= left_right_idxs[0][0]; ++i)
			{
				polygon_idxs1.emplace_back(i);
			}
			//polygon2
			for (int i = left_right_idxs[0][1]; i <= left_right_idxs[1][0] + corner_pnts_.size(); ++i)
			{
				int idx = i % corner_pnts_.size();
				polygon_idxs2.emplace_back(idx);
			}
		}
		else if (splitfirstype == 4)
		{
			if (left_right_idxs[1][0] == corner_pnts_.size() - 1)
			{
				//polygon1
				for (int i = left_right_idxs[1][1]; i <= left_right_idxs[0][0]; ++i)
				{
					polygon_idxs1.emplace_back(i);
				}
				//polygon2
				for (int i = left_right_idxs[0][1]; i <= left_right_idxs[1][0]; ++i)
				{
					int idx = i % corner_pnts_.size();
					polygon_idxs2.emplace_back(idx);
				}
			}
		}
		else
		{
			std::cout << "no split..." << std::endl;
			return false;
		}

#if 0
		int icount_end = 0;
		if (left_right_idxs[0][1] == corner_pnts_.size() - 1)
		{
			icount_end = 0;
		}
		else
		{
			icount_end = left_right_idxs[0][0];
		}

		for (int i = left_right_idxs[0][1]; i <= corner_pnts_.size() + left_right_idxs[0][0]; ++i)
		{
			//恢复这个顶点
			int idx = i % corner_pnts_.size();
			if (idx == left_right_idxs[1][1])
			{
				p2_flag = true;
			}
			if (p2_flag)
			{
				polygon_idxs2.emplace_back(idx);
			}
			else
			{
				polygon_idxs1.emplace_back(idx);
			}
		}
#endif

		if (polygon_idxs2.empty() || polygon_idxs1.empty())
			return false;

		std::cout << "the new polygon1 idx begin: " << std::endl;
		for (int i = 0; i < polygon_idxs1.size(); ++i)
		{
			std::cout << polygon_idxs1[i] << " ";
		}
		std::cout << std::endl;
		std::cout << "the new polygon2 idx begin: " << std::endl;
		for (int i = 0; i < polygon_idxs2.size(); ++i)
		{
			std::cout << polygon_idxs2[i] << " ";
		}
		std::cout << std::endl;

		//
		split_idxs1 = polygon_idxs1;
		split_idxs2 = polygon_idxs2;
		return true;
	}

	bool HWPlane::SplitSelectPolyFromSplitLine(Eigen::Vector2f& ldir2d, Eigen::Vector2f& lpnt2d, std::vector<int>& selected_poly,
		std::vector<int>& splitidx1, std::vector<int>& splitidx2)
	{
		if (selected_poly.size() < 2)
			return false;
		for (int i = 0, j = i + 1; i < selected_poly.size()-1; ++i, j = i+1)
		{
			//获取两个端点
			//i,j为一个线段
			Eigen::Vector2f ls = Eigen::Vector2f(corner_pnts_[selected_poly[i]].x, corner_pnts_[selected_poly[i]].y);
			Eigen::Vector2f le = Eigen::Vector2f(corner_pnts_[selected_poly[j]].x, corner_pnts_[selected_poly[j]].y);
			//计算线段和直线是否有交点
			Eigen::Vector2f tmp_cross_pnt;
			if (ComputeLineSeg2DCrossLine2DPntsNew(ls, le, lpnt2d, ldir2d, tmp_cross_pnt))
			{
				//获取顶点

			}
		}
		return true;
	}

	bool HWPlane::ComputeNearestLineCrossPnts(Eigen::Vector2f& ldir2d, Eigen::Vector2f& lpnt2d,
		Eigen::Vector2i& pnts_idxs, Eigen::Vector2f& cross_pnt)
	{
		if (corner_pnts_.size() < 2)
			return false;
		Eigen::Vector2i left_right_idxs = Eigen::Vector2i(-1, -1);
		//left_right_idxs[1] = Eigen::Vector2i(-1, -1);

		float min_k = std::numeric_limits<float>::max();
		float max_k = std::numeric_limits<float>::lowest();
		Eigen::Vector2f pnt_min = Eigen::Vector2f(0.0, 0.0);
		for (int i = 0, j = corner_pnts_.size() - 1; i < corner_pnts_.size(); j = i++)
		{
			//j,i为一个线段
			Eigen::Vector2f ls = Eigen::Vector2f(corner_pnts_[j].x, corner_pnts_[j].y);
			Eigen::Vector2f le = Eigen::Vector2f(corner_pnts_[i].x, corner_pnts_[i].y);
			//计算线段和直线是否有交点
			Eigen::Vector2f tmp_cross_pnt;
			if (ComputeLineSeg2DCrossLine2DPntsNew(ls, le, lpnt2d, ldir2d, tmp_cross_pnt))
			{
				//如果有交点，就获取交点到
				Eigen::Vector2f p2lpnt = tmp_cross_pnt - lpnt2d;
				//float k = p2lpnt.dot(ldir2d) / (ldir2d.norm() * ldir2d.norm());
				if (p2lpnt.norm() < min_k)
				{
					left_right_idxs = Eigen::Vector2i(j, i);
					min_k = p2lpnt.norm();
					pnt_min = tmp_cross_pnt;
				}
			}
		}
		std::cout << "the left: " << left_right_idxs[0] << " " << left_right_idxs[1] << std::endl;
		if (left_right_idxs[0] == -1 || left_right_idxs[1] == -1)
			return false;
		pnts_idxs = left_right_idxs;
		cross_pnt = pnt_min;
		return true;
	}

	bool HWPlane::ComputeCrossLSIdxFromRect(std::vector<Eigen::Vector2f>& rectpnts, std::vector<int>& polygonidxs,
		Eigen::Vector2i& start_l, Eigen::Vector2i& end_l)
	{
		//处理这个rect
		int polgonidxs_num = polygonidxs.size();
		if (polgonidxs_num < 2)
			return false;
		Eigen::Vector2i tmp_sl = Eigen::Vector2i(-1, -1);
		Eigen::Vector2i tmp_el = Eigen::Vector2i(-1, -1);
		int icount = 0;
		for (int i = 0, j = 1; i < polygonidxs.size() - 1; ++i,++j)
		{
			Eigen::Vector2f ls = Eigen::Vector2f(corner_pnts_[polygonidxs[i]].x, corner_pnts_[polygonidxs[i]].y);
			Eigen::Vector2f le = Eigen::Vector2f(corner_pnts_[polygonidxs[j]].x, corner_pnts_[polygonidxs[j]].y);
			//std::cerr << "ComputeCrossLSIdxFromRect: i, j: " << i << ", " << j << ": "
				//<< ls[0] << " " << ls[1] << ", " << le[0] << " " << le[1] << std::endl;
			Eigen::Vector2f cross_pnt0, cross_pnt1;
			if (ComputeCrossPntFromRect(rectpnts, ls, le, cross_pnt0, cross_pnt1) == 1)
			{
				++icount;
				std::cerr << "icount: " << icount << std::endl;
				if (icount == 1)
				{
					start_l = Eigen::Vector2i(i, j);
				}
				end_l = Eigen::Vector2i(i, j);
			}
			else if (ComputeCrossPntFromRect(rectpnts, ls, le, cross_pnt0, cross_pnt1) == 2)
			{
				++icount;
				std::cerr << "icount: " << icount << std::endl;
				if (icount == 1)
				{
					start_l = Eigen::Vector2i(i, j);
				}
				end_l = Eigen::Vector2i(i, j);
			}
		}
		std::cerr << "end icount: " << icount << std::endl;
		if (start_l[0] != -1 && start_l[1] != -1 && end_l[0] != -1 && end_l[1] != -1)
			return true;
		return false;
	}

	int HWPlane::ComputeCrossPntFromRect(std::vector<Eigen::Vector2f>& rectpnts,
		Eigen::Vector2f& ls, Eigen::Vector2f& le, Eigen::Vector2f& crosspnt0, Eigen::Vector2f& crosspnt1)
	{
		int icount = 0;
		for (int i = 0, j = rectpnts.size() - 1; i < rectpnts.size(); j = i++)
		{
			Eigen::Vector2f cross_pnt;
			Eigen::Vector2f ts = rectpnts[j];
			Eigen::Vector2f te = rectpnts[i];
			//std::cerr << "ComputeCrossPntFromRect..." << std::endl;
			//std::cerr << ts[0] << " " << ts[1] << std::endl;
			//std::cerr << te[0] << " " << te[1] << std::endl;
			if (ComputeTwoLineSegsCrossPnt(ls, le, ts, te, cross_pnt))
			{
				++icount;
				if (icount == 1)
				{
					crosspnt0 = cross_pnt;
				}
				if (icount == 2)
				{
					crosspnt1 = cross_pnt;
				}
			}
		}
		return icount;
	}

	bool HWPlane::ComputeSENearestLineCrossPnts(Eigen::Vector2f& ldir2d, Eigen::Vector2f& lpnt2d,
		Eigen::Vector2i& ls_pnts_idx, Eigen::Vector2f& ls_cross_pnt,
		Eigen::Vector2i& le_pnts_idx, Eigen::Vector2f& le_cross_pnt)
	{
		if (corner_pnts_.size() < 2)
			return false;
		Eigen::Vector2i ls_left_right_idxs = Eigen::Vector2i(-1, -1);
		Eigen::Vector2i le_left_right_idxs = Eigen::Vector2i(-1, -1);

		float ls_min_k = std::numeric_limits<float>::max();
		float le_min_k = std::numeric_limits<float>::max();
		Eigen::Vector2f ls_pnt_min = Eigen::Vector2f(0.0, 0.0);
		Eigen::Vector2f le_pnt_min = Eigen::Vector2f(0.0, 0.0);
		for (int i = 0, j = corner_pnts_.size() - 1; i < corner_pnts_.size(); j = i++)
		{
			//j,i为一个线段
			Eigen::Vector2f ls = Eigen::Vector2f(corner_pnts_[j].x, corner_pnts_[j].y);
			Eigen::Vector2f le = Eigen::Vector2f(corner_pnts_[i].x, corner_pnts_[i].y);
			//计算线段和直线是否有交点
			Eigen::Vector2f tmp_cross_pnt;
			if (ComputeLineSeg2DCrossLine2DPntsNew(ls, le, lpnt2d, ldir2d, tmp_cross_pnt))
			{
				//如果有交点，就获取交点到
				std::cerr << "tmp_cross_pnt: " << tmp_cross_pnt[0] << " " << tmp_cross_pnt[1] << std::endl;
				Eigen::Vector2f p2lpnt = tmp_cross_pnt - lpnt2d;
				//float k = p2lpnt.dot(ldir2d) / (ldir2d.norm() * ldir2d.norm());
				if (p2lpnt.norm() < ls_min_k)
				{
					ls_left_right_idxs = Eigen::Vector2i(j, i);
					ls_min_k = p2lpnt.norm();
					ls_pnt_min = tmp_cross_pnt;
				}
				if (p2lpnt.norm() < le_min_k)
				{
					le_left_right_idxs = Eigen::Vector2i(j, i);
					le_min_k = p2lpnt.norm();
					le_pnt_min = tmp_cross_pnt;
				}
			}
		}
		std::cout << "the ls min: " << ls_left_right_idxs[0] << " " << ls_left_right_idxs[1] << std::endl
			<< "the le min: " << le_left_right_idxs[0] << " " << le_left_right_idxs[1] << std::endl;
		if (ls_left_right_idxs[0] == -1 || ls_left_right_idxs[1] == -1
			|| le_left_right_idxs[0] == -1 || le_left_right_idxs[1] == -1)
			return false;
		if (ls_left_right_idxs == le_left_right_idxs)
		{
			if (ls_min_k > le_min_k)
			{
				//对ls_min_k进行恢复
				ls_left_right_idxs = Eigen::Vector2i(-1, -1);
				ls_min_k = std::numeric_limits<float>::max();
				Eigen::Vector2f ls_pnt_min = Eigen::Vector2f(0.0, 0.0);
				for (int i = 0, j = corner_pnts_.size() - 1; i < corner_pnts_.size(); j = i++)
				{
					//j,i为一个线段
					if (ls_left_right_idxs[0] == j && ls_left_right_idxs[1] == i)
						continue;
					Eigen::Vector2f ls = Eigen::Vector2f(corner_pnts_[j].x, corner_pnts_[j].y);
					Eigen::Vector2f le = Eigen::Vector2f(corner_pnts_[i].x, corner_pnts_[i].y);
					//计算线段和直线是否有交点
					Eigen::Vector2f tmp_cross_pnt;
					if (ComputeLineSeg2DCrossLine2DPntsNew(ls, le, lpnt2d, ldir2d, tmp_cross_pnt))
					{
						//如果有交点，就获取交点到
						Eigen::Vector2f p2lpnt = tmp_cross_pnt - lpnt2d;
						//float k = p2lpnt.dot(ldir2d) / (ldir2d.norm() * ldir2d.norm());
						if (p2lpnt.norm() < ls_min_k)
						{
							ls_left_right_idxs = Eigen::Vector2i(j, i);
							ls_min_k = p2lpnt.norm();
							ls_pnt_min = tmp_cross_pnt;
						}
					}
				}
				if (ls_left_right_idxs[0] == -1 || ls_left_right_idxs[1] == -1)
					return false;
			}
			else
			{
				//对ls_min_k进行恢复
				le_left_right_idxs = Eigen::Vector2i(-1, -1);
				le_min_k = std::numeric_limits<float>::max();
				Eigen::Vector2f le_pnt_min = Eigen::Vector2f(0.0, 0.0);
				for (int i = 0, j = corner_pnts_.size() - 1; i < corner_pnts_.size(); j = i++)
				{
					//j,i为一个线段
					if (le_left_right_idxs[0] == j && le_left_right_idxs[1] == i)
						continue;
					Eigen::Vector2f ls = Eigen::Vector2f(corner_pnts_[j].x, corner_pnts_[j].y);
					Eigen::Vector2f le = Eigen::Vector2f(corner_pnts_[i].x, corner_pnts_[i].y);
					//计算线段和直线是否有交点
					Eigen::Vector2f tmp_cross_pnt;
					if (ComputeLineSeg2DCrossLine2DPntsNew(ls, le, lpnt2d, ldir2d, tmp_cross_pnt))
					{
						//如果有交点，就获取交点到
						Eigen::Vector2f p2lpnt = tmp_cross_pnt - lpnt2d;
						//float k = p2lpnt.dot(ldir2d) / (ldir2d.norm() * ldir2d.norm());
						if (p2lpnt.norm() < le_min_k)
						{
							le_left_right_idxs = Eigen::Vector2i(j, i);
							le_min_k = p2lpnt.norm();
							le_pnt_min = tmp_cross_pnt;
						}
					}
				}
				if (ls_left_right_idxs[0] == -1 || ls_left_right_idxs[1] == -1)
					return false;
			}
		}
		ls_pnts_idx = ls_left_right_idxs;
		ls_cross_pnt = ls_pnt_min;
		le_pnts_idx = le_left_right_idxs;
		le_cross_pnt = le_pnt_min;
		return true;
	}

	bool HWPlane::CheckLineSplitSrcPolygon(Eigen::Vector2f& ldir2d, Eigen::Vector2f& lpnt2d)
	{
		for (int i = 0, j = corner_pnts_.size() - 1; i < corner_pnts_.size(); j = i++)
		{
			//j,i为一个线段
			Eigen::Vector2f ls = Eigen::Vector2f(corner_pnts_[j].x, corner_pnts_[j].y);
			Eigen::Vector2f le = Eigen::Vector2f(corner_pnts_[i].x, corner_pnts_[i].y);
			//计算线段和直线是否有交点
			Eigen::Vector2f tmp_cross_pnt;
			if (ComputeLineSeg2DCrossLine2DPntsNew(ls, le, lpnt2d, ldir2d, tmp_cross_pnt))
				return true;
		}
		return false;
	}

	bool HWPlane::ComputeLine2NearestHalfPolyCrossPnt(Eigen::Vector2f& ldir2d, Eigen::Vector2f& lpnt2d,
		std::vector<Eigen::Vector2f>& hf_poly_pnts, Eigen::Vector2f& cross_pnts)
	{
		Eigen::Vector2i lr_idx = Eigen::Vector2i(-1, -1);
		float min_k = std::numeric_limits<float>::max();
		Eigen::Vector2f pnt_min;
		for (int i = 1, j = 0; i < hf_poly_pnts.size(); j = i++)
		{
			//j,i为一个线段
			Eigen::Vector2f ls = hf_poly_pnts[j];
			Eigen::Vector2f le = hf_poly_pnts[i];	
			//计算线段和直线是否有交点
			Eigen::Vector2f tmp_cross_pnt;
			if (ComputeLineSeg2DCrossLine2DPntsNew(ls, le, lpnt2d, ldir2d, tmp_cross_pnt))
			{
				//如果有交点，就获取交点到
				Eigen::Vector2f p2lpnt = tmp_cross_pnt - lpnt2d;
				//float k = p2lpnt.dot(ldir2d) / (ldir2d.norm() * ldir2d.norm());
				if (p2lpnt.norm() < min_k)
				{
					lr_idx = Eigen::Vector2i(j, i);
					min_k = p2lpnt.norm();
					pnt_min = tmp_cross_pnt;
				}
			}
		}
		if (lr_idx[0] == -1 && lr_idx[1] == -1)
			return false;
		cross_pnts = pnt_min;
		return true;
	}

	/*bool HWPlane::CheckLineSegSplitSrcPolygon(Eigen::Vector2f& ls, Eigen::Vector2f& le)
	{
		Eigen::Vector2f ldir2d, lpnt2d;
		ComputeLdirFunctionFromEndPnts2D(ls, le, ldir2d, lpnt2d);
		if (ldir2d.norm() < 1e-4)
		{
			return false;
		}
		return CheckLineSplitSrcPolygon(ldir2d, lpnt2d);
	}*/

	void HWPlane::CheckSrcPolygonCanonicalType()
	{
		//这个用于对src polygon的类型判断
		//将结构转化为图像，在图像上处理
		std::cerr << "the file name_ is: " << filename_ << std::endl;
		std::string idx_str = std::to_string(splited_label_);
		std::string filename = filename_ +"_" + idx_str + "_img.png";
		std::vector<Eigen::Vector2f> cornerpnts;
		for (int i = 0; i < corner_pnts_.size(); ++i)
		{
			cornerpnts.emplace_back(Eigen::Vector2f(corner_pnts_[i].x, corner_pnts_[i].y));
		}
		cv::Mat src_image;
		DrawImageFromCornerPnts2d(cornerpnts, src_image, filename);
#if 1
		if (!related_poly_corners_widths_2d_.empty())
		{
			for (int i = 0; i < related_poly_corners_widths_2d_.size(); ++i)
			{
				Eigen::Vector2f ls = related_poly_corners_widths_2d_[i].first;
				Eigen::Vector2f le = related_poly_corners_widths_2d_[i].second;
				std::vector<Eigen::Vector2f> vec;
				std::cerr << i << " related: " << ls[0] << " " << ls[1] << ", " << le[0] << " " << le[1] << std::endl;
				vec.emplace_back(ls);
				vec.emplace_back(le);
				DrawImageFromLineSegPnts2d(vec, src_image, filename);
			}
		}
		else
		{
			std::cerr << "wrong pca ls and le..." << std::endl;
		}
#endif

#if ZDG_DEBUG
		if (filename.find(".png") != std::string::npos)
			cv::imwrite(filename, src_image);
#endif
	}

	void HWPlane::DrawSrcPolygon2DImgCoord(cv::Mat& img)
	{
		//将结构转化为图像，在图像上处理
		std::cerr << "the file name_ is: " << filename_ << std::endl;
		std::string idx_str = std::to_string(splited_label_);
		std::string filename = filename_ + "_" + idx_str + "_img.png";
		std::vector<Eigen::Vector2f> cornerpnts;
		for (int i = 0; i < corner_pnts_.size(); ++i)
		{
			cornerpnts.emplace_back(Eigen::Vector2f(corner_pnts_[i].x, corner_pnts_[i].y));
		}
		DrawImageFromCornerPnts2d(cornerpnts, img, filename);
	}

	void HWPlane::DrawTgtPolygonLine2DImgCoord(cv::Mat& img)
	{
		//将结构转化为图像，在图像上处理
		std::cerr << "the file name_ is: " << filename_ << std::endl;
		std::string idx_str = std::to_string(splited_label_);
		std::string filename = filename_ + "_" + idx_str + "_img.png";

		if (!related_poly_corners_widths_2d_.empty())
		{
			for (int i = 0; i < related_poly_corners_widths_2d_.size(); ++i)
			{
				Eigen::Vector2f ls = related_poly_corners_widths_2d_[i].first;
				Eigen::Vector2f le = related_poly_corners_widths_2d_[i].second;
				std::vector<Eigen::Vector2f> vec;
				std::cerr << i << " related: " << ls[0] << " " << ls[1] << ", " << le[0] << " " << le[1] << std::endl;
				vec.emplace_back(ls);
				vec.emplace_back(le);
				DrawImageFromLineSegPnts2d(vec, img, filename);
			}
		}
		else
		{
			std::cerr << "wrong pca ls and le..." << std::endl;
		}
	}

	void HWPlane::ComputeSrcAndTgtParamsOnSrcCoord()
	{
		if (splited_label_ == -1)
		{
			return;
		}
		else
		{
			float polyarea;
			Eigen::Vector2f ls, le;
			ComputePolyParamsOnSrcCoord(splited_label_, polyarea, ls, le);
			corners_area_2d_ = polyarea;
			corners_width_2d_ = std::make_pair(ls, le);
		}
		//将所有的关于polygon的几何信息都计算好，用于后续判断sor polygon的类型
		for (int i = 0; i < related_poly_idxs_.size(); ++i)
		{
			float polyarea;
			Eigen::Vector2f ls, le;
			ComputePolyParamsOnSrcCoord(related_poly_idxs_[i], polyarea, ls, le);
			std::pair<Eigen::Vector2f, Eigen::Vector2f> tmp = std::make_pair(ls, le);
			related_poly_corners_widths_2d_.emplace_back(tmp);
		}
	}

	void HWPlane::ComputePolyParamsOnSrcCoord(int idx, float& poly_area,
		Eigen::Vector2f& ls, Eigen::Vector2f& le)
	{
		//if(idx)
		if (idx >= model_planes_.size() || idx < 0)
			return;
		model_planes_[idx]->ComputePolygon2DAreaFromCornerPnts3d();
		poly_area = model_planes_[idx]->GetPolygonCorner2DArea();
		Eigen::Vector3f ldir3d, lpnt3d;
		GetExpandTgtIntersectionLine(idx, ldir3d, lpnt3d);
		Eigen::Vector2i tgt_pnts_lr_idx(-1, -1);
		model_planes_[idx]->ComputeTwoPointsIdxCorrespondingLine(ldir3d, lpnt3d, tgt_pnts_lr_idx);
		if (tgt_pnts_lr_idx[0] == -1 || tgt_pnts_lr_idx[1] == -1)
			return;
		float2 tprjlpnt2d = model_planes_[idx]->ProjToLine3D(tgt_pnts_lr_idx[0], ldir3d, lpnt3d);
		float2 tprjrpnt2d = model_planes_[idx]->ProjToLine3D(tgt_pnts_lr_idx[1], ldir3d, lpnt3d);
		Eigen::Vector3f tprjlpnt3d, tprjrpnt3d;
		model_planes_[idx]->Pnt2d2Pnt3D(Eigen::Vector2f(tprjlpnt2d.x, tprjlpnt2d.y), tprjlpnt3d);
		model_planes_[idx]->Pnt2d2Pnt3D(Eigen::Vector2f(tprjrpnt2d.x, tprjrpnt2d.y), tprjrpnt3d);
		Eigen::Vector2f sproj_lpnt2d, sproj_rpnt2d;
		Pnt3d2Pnt2D(tprjlpnt3d, sproj_lpnt2d);
		Pnt3d2Pnt2D(tprjrpnt3d, sproj_rpnt2d);	//这是tgt 线段的左右两个端点
		ls = sproj_lpnt2d;
		le = sproj_rpnt2d;
	}

#if 0
	bool HWPlane::CheckLineSplitSrcPolyNode(SplitNode* srcnode, Eigen::Vector2f& ldir2d, Eigen::Vector2f& lpnt2d)
	{
		if (srcnode)
		{
			if (srcnode->t == kLOffPoly
				|| srcnode->t == kPolyOffL)
			{
				return false;
			}
			else if (srcnode->t == kLIM)
			{
				//这个需要重新写
				for (int i = 0, j = srcnode->s_pnts_idxs_.size() - 1; i < srcnode->s_pnts_idxs_.size(); j = i++)
				{
					//j,i为一个线段
					Eigen::Vector2f ls = Eigen::Vector2f(corner_pnts_[srcnode->s_pnts_idxs_[j]].x, corner_pnts_[srcnode->s_pnts_idxs_[j]].y);
					Eigen::Vector2f le = Eigen::Vector2f(corner_pnts_[srcnode->s_pnts_idxs_[i]].x, corner_pnts_[srcnode->s_pnts_idxs_[i]].y);
					//计算线段和直线是否有交点
					Eigen::Vector2f tmp_cross_pnt;
					if (ComputeLineSeg2DCrossLine2DPntsNew(ls, le, lpnt2d, ldir2d, tmp_cross_pnt))
						return true;
				}
			}
			else if (srcnode->t == kMIL)
			{

			}
			else
			{

			}
		}
		return false;
	}


	bool HWPlane::LineSplitSrcPoly(Eigen::Vector2f& ldir2d, Eigen::Vector2f& lpnt2d,
		SplitNode* lnode, SplitNode* rnode)
	{
		if (CheckLineSplitSrcPolygon(ldir2d, lpnt2d))
		{
			//表示它穿过polygon
			if (related_poly_idxs_.size() == 1)
			{
				SplitNode* mnode = new SplitNode();
				//判断类型，需要处理一下判断它的类型
				//它表示切开了这个polygon,需要判断它的类型
				if (polygon_cannoical_ == kPolyConvex)
				{
					//只有两个顶点
				}
				else if (polygon_cannoical_ == kPolyNoConvex)
				{
				}
				else
				{
					return false;
				}

				if (split_poly_nodes_[0]->t == kLIM)
				{
					//
				}
				else if (split_poly_nodes_[0]->t == kMIL)
				{
				}
				else
				{
				}
			}
			else
			{

			}
		}
		else
		{
			//表示它未穿过polygon
			if (related_poly_idxs_.size() == 1)
			{
				SplitNode* mnode = new SplitNode();
				//判断类型，需要处理一下判断它的类型
				//它表示切开了这个polygon,需要判断它的类型,是凸形状还是凹形状
				//判断

				if (polygon_cannoical_ == kPolyConvex)
				{
					//只有两个顶点
				}
				else if (polygon_cannoical_ == kPolyNoConvex)
				{
				}
				else
				{
					return false;
				}

				if (split_poly_nodes_[0]->t == kLOffPoly)
				{
					//没有剪切src poly
					//split_poly_nodes_[0]->s_pnts_idxs_
				}
				else if (split_poly_nodes_[0]->t == kPolyOffL)
				{

				}
				else
				{
					std::cerr << "LineSplitSrcPoly: Error in split..." << std::endl;
					return false;
				}
			}
			else
			{
			}
		}
		return true;
	}


	bool HWPlane::LineSegSplitSrcPoly(int lidx, Eigen::Vector2f& ls, Eigen::Vector2f& le,
		SplitNode* lnode, SplitNode* rnode)
	{
		if (lidx < 0 || lidx >= model_planes_.size())
			return false;
		//获取当前t polygon的面积,一般对于大面积，它才具有门的那种形状非convex polygon
		float sarea = corners_area_2d_;
		float tarea = model_planes_[lidx]->GetPolygonCorner2DArea();
		//计算t的线段的长度tlen，其中 tlen是一个非常重要的标志
		float tlen = (le - ls).norm();
		//转化为直线表达式
		Eigen::Vector2f ldir2d, lpnt2d;
		ComputeLdirFunctionFromEndPnts2D(ls, le, ldir2d, lpnt2d);
		//计算原多边形的相对的宽度
		Eigen::Vector2i slr_idx = Eigen::Vector2i(-1, -1);
		ComputeLeftRightPointsIdxLine2D(ldir2d, lpnt2d, slr_idx);
		if (slr_idx[0] == -1 || slr_idx[1] == -1)
			return false;
		Eigen::Vector2f slpnt2d = Eigen::Vector2f(corner_pnts_[slr_idx[0]].x, corner_pnts_[slr_idx[0]].y);
		Eigen::Vector2f srpnt2d = Eigen::Vector2f(corner_pnts_[slr_idx[1]].x, corner_pnts_[slr_idx[1]].y);
		//需要各种信息在一起判断当前的polygon是否为convex还是正常的非convex
		float slen = (srpnt2d - slpnt2d).norm();

		//判断面积和线段，表示两者关系
		return true;
	}
#endif

	void HWPlane::SortCornerPnts()
	{
		int mincor_idx = ComputeApproxMinCorIdx();
		if (mincor_idx == -1)
			return;
		std::vector<int> restarted_poly_idxs;
		RestartPolyIdxsFromSelectedIdx(mincor_idx, restarted_poly_idxs);
		//形成新的polygon，然后更新原始的polygon？
		//test
		//std::cerr << "before sort poly pnt..." << std::endl;
		//print_polygon_pnts_2d_float2(corner_pnts_);
		//end test
		std::vector<Eigen::Vector2f> poly_pnts_new;
		for (int i = 0; i < restarted_poly_idxs.size(); ++i)
		{
			Eigen::Vector2f tmp_pnt = Eigen::Vector2f(corner_pnts_[restarted_poly_idxs[i]].x, 
				corner_pnts_[restarted_poly_idxs[i]].y);
			poly_pnts_new.emplace_back(tmp_pnt);
		}
		UpdateCornerPtsFromNewPolygon(poly_pnts_new);
		resorted_sor_corner_flag_ = true;
		//test
		//std::cerr << "end sort poly pnt..." << std::endl;
		//print_polygon_pnts_2d_float2(corner_pnts_);
		//end test
	}

	void HWPlane::ComputeCornerPnts3dBoxPos(Eigen::Vector3f& mincor, Eigen::Vector3f& maxcor)
	{
		Eigen::Vector3f min_value = Eigen::Vector3f(FLT_MAX, FLT_MAX, FLT_MAX);
		Eigen::Vector3f max_value = Eigen::Vector3f(FLT_MIN, FLT_MIN, FLT_MIN);
		for (int i = 0; i < corner_pnts_3d_.size(); ++i)
		{
			if (min_value[0] > corner_pnts_3d_[i].x)
				min_value[0] = corner_pnts_3d_[i].x;
			if (min_value[1] > corner_pnts_3d_[i].y)
				min_value[1] = corner_pnts_3d_[i].y;
			if (min_value[2] > corner_pnts_3d_[i].z)
				min_value[2] = corner_pnts_3d_[i].z;
			if (max_value[0] < corner_pnts_3d_[i].x)
				max_value[0] = corner_pnts_3d_[i].x;
			if (max_value[1] < corner_pnts_3d_[i].y)
				max_value[1] = corner_pnts_3d_[i].y;
			if (max_value[2] < corner_pnts_3d_[i].z)
				max_value[2] = corner_pnts_3d_[i].z;
		}
		mincor = min_value;
		maxcor = max_value;
	}

	void HWPlane::ComputeCornerPnts2dBoxPos(Eigen::Vector2f& mincor, Eigen::Vector2f& maxcor)
	{
		//处理平面polygon的bounding box
		Eigen::Vector2f min_value = Eigen::Vector2f(FLT_MAX, FLT_MAX);
		Eigen::Vector2f max_value = Eigen::Vector2f(FLT_MIN, FLT_MIN);
		for (int i = 0; i < corner_pnts_.size(); ++i)
		{
			if (min_value[0] > corner_pnts_[i].x)
				min_value[0] = corner_pnts_[i].x;
			if (min_value[1] > corner_pnts_[i].y)
				min_value[1] = corner_pnts_[i].y;
			if (max_value[0] < corner_pnts_[i].x)
				max_value[0] = corner_pnts_[i].x;
			if (max_value[1] < corner_pnts_[i].y)
				max_value[1] = corner_pnts_[i].y;
		}
		mincor = min_value;
		maxcor = max_value;
	}

	int HWPlane::ComputeApproxMinCorIdx()
	{
		if (corner_pnts_.empty())
			return -1;
		//计算2维polygon的box
		Eigen::Vector2f mincor = Eigen::Vector2f(FLT_MIN, FLT_MIN);
		Eigen::Vector2f maxcor = Eigen::Vector2f(FLT_MAX, FLT_MAX);
		ComputeCornerPnts2dBoxPos(mincor, maxcor);
		//寻找最左和最下的顶点
		int idx = -1;
		float dist_min = FLT_MAX;
		for (int i = 0; i < corner_pnts_.size(); ++i)
		{
			Eigen::Vector2f tmp_pnt = Eigen::Vector2f(corner_pnts_[i].x, corner_pnts_[i].y);
			if ((tmp_pnt - mincor).norm() < dist_min)
			{
				idx = i;
				dist_min = (tmp_pnt - mincor).norm();
			}
		}
		return idx;
	}

	bool HWPlane::LineSplitSrcNode(int idx, Eigen::Vector2f& ldir2d, Eigen::Vector2f& lpnt2d,
		SplitNode* node, SplitNode* lnode, SplitNode* rnode)
	{
		if (node->t == kLIM)
		{
		}
		else if (node->t == kMIL)
		{
		}
		if (node->t == kLOffPoly)
		{
		}
		else if (node->t == kPolyOffL)
		{
		}
		else if (node->t == kLOffMOffL)
		{
		}
		else if (node->t == kLIMOffL)
		{
		}
		else if (node->t == kLIMDoor)
		{
		}
		else if (node->t == kMILDoor)
		{
		}
		else if (node->t == kLOffMOffLDoor)
		{
		}
		else if (node->t == kLIMOffLDoor)
		{
		}
		else
		{
			std::cerr << "LineSplitSrcPoly: Error in split..." << std::endl;
			return false;
		}
		return true;
	}

	bool HWPlane::ComputeWholePolyInsectLIdxsFromLine(Eigen::Vector3f& L_dir, Eigen::Vector3f& L_point,
		Eigen::Vector2i& line1pidx, Eigen::Vector2i& line2pidx, Eigen::Vector2f& l1_pnt, Eigen::Vector2f& l2_pnt)
	{
		//
		Eigen::Vector2f lpnt2d, ldir2d;
		ComputeWorldLf2PlaneLf(L_dir, L_point, ldir2d, lpnt2d);
		std::cout << "the lpnt2d pnt: " << lpnt2d[0] << " " << lpnt2d[1] <<
			",  ldir2d dir: " << ldir2d[0] << " " << ldir2d[1] << std::endl;

		std::vector<Eigen::Vector2i> left_right_idxs;
		left_right_idxs.resize(2);
		left_right_idxs[0] = Eigen::Vector2i(-1, -1);
		left_right_idxs[1] = Eigen::Vector2i(-1, -1);

		Eigen::Vector2f cross_pnt1, cross_pnt2;

		float min_k = std::numeric_limits<float>::max();
		float max_k = std::numeric_limits<float>::lowest();
		for (int i = 0, j = corner_pnts_.size() - 1; i < corner_pnts_.size(); j = i++)
		{
			//j,i为一个线段
			Eigen::Vector2f ls = Eigen::Vector2f(corner_pnts_[j].x, corner_pnts_[j].y);
			Eigen::Vector2f le = Eigen::Vector2f(corner_pnts_[i].x, corner_pnts_[i].y);
			//计算线段和直线是否有交点
			Eigen::Vector2f tmp_cross_pnt;
			if (ComputeLineSeg2DCrossLine2DPntsNew(ls, le, lpnt2d, ldir2d, tmp_cross_pnt))
			{
				//如果有交点，就获取交点到
				Eigen::Vector2f p2lpnt = tmp_cross_pnt - lpnt2d;
				float k = p2lpnt.dot(ldir2d) / (ldir2d.norm() * ldir2d.norm());

				if (min_k > k)
				{
					left_right_idxs[0] = Eigen::Vector2i(j, i);
					min_k = k;
					cross_pnt1 = tmp_cross_pnt;
				}
				if (max_k < k)
				{
					left_right_idxs[1] = Eigen::Vector2i(j, i);
					max_k = k;
					cross_pnt2 = tmp_cross_pnt;
				}
			}
		}

		std::cout << "the whole polygon left and right: " << left_right_idxs[0][0] << " " << left_right_idxs[0][1]
			<< " " << left_right_idxs[1][0] << " " << left_right_idxs[1][1] << std::endl;

		if (left_right_idxs[0][0] == -1 || left_right_idxs[0][1] == -1
			|| left_right_idxs[1][0] == -1 || left_right_idxs[1][1] == -1)
			return false;

		line1pidx = left_right_idxs[0];
		line2pidx = left_right_idxs[1];
		l1_pnt = cross_pnt1;
		l2_pnt = cross_pnt2;

		return true;
	}

	bool HWPlane::AddSplitPntsIntoSplitPoly(std::vector<int>& select_polygon, Eigen::Vector2i& line1pidx, Eigen::Vector2i& line2pidx,
		Eigen::Vector2f& l1_pnt, Eigen::Vector2f& l2_pnt, std::vector<Eigen::Vector2f>& poly_addedpnts)
	{
		//计算polygon
		int pre_poly1_idx = -1, next_poly1_idx = -1;
		int spoly1_idx = select_polygon[0];
		int epoly1_idx = select_polygon[select_polygon.size() - 1];
		std::cout << "the spoly1_idx, the epoly1_idx: " << spoly1_idx << " " << epoly1_idx << std::endl;

		if (spoly1_idx == line1pidx[0]
			|| spoly1_idx == line1pidx[1])
		{
			if (epoly1_idx == line2pidx[0]
				|| epoly1_idx == line2pidx[1])
			{
				poly_addedpnts.emplace_back(l1_pnt);
				for (int i = 0; i < select_polygon.size(); ++i)
				{
					float2 poly_pnt = corner_pnts_[select_polygon[i]];
					Eigen::Vector2f polypnt = Eigen::Vector2f(poly_pnt.x, poly_pnt.y);
					poly_addedpnts.emplace_back(polypnt);
				}
				poly_addedpnts.emplace_back(l2_pnt);
			}
		}
		else if (spoly1_idx == line2pidx[0]
			|| spoly1_idx == line2pidx[1])
		{
			if (epoly1_idx == line1pidx[0]
				|| epoly1_idx == line1pidx[1])
			{
				//
				poly_addedpnts.emplace_back(l2_pnt);
				for (int i = 0; i < select_polygon.size(); ++i)
				{
					float2 poly_pnt = corner_pnts_[select_polygon[i]];
					Eigen::Vector2f polypnt = Eigen::Vector2f(poly_pnt.x, poly_pnt.y);
					poly_addedpnts.emplace_back(polypnt);
				}
				poly_addedpnts.emplace_back(l1_pnt);
			}
		}
		else
		{
			std::cout << "AddSplitPntsIntoSplitPoly: no polygon idx...." << std::endl;
			return false;
		}
		return true;
	}

	bool HWPlane::ComputeSplitPolyInsectLDistToLine(Eigen::Vector3f& L_dir, Eigen::Vector3f& L_point, Eigen::Vector2f& m_pnt, Eigen::Vector2f& m_dir,
		std::vector<Eigen::Vector2f>& polypnts2d, Eigen::Vector2f& dist_near_far, Eigen::Vector2i& near_far_idx)
	{
		//
		Eigen::Vector2f lpnt2d, ldir2d;
		ComputeWorldLf2PlaneLf(L_dir, L_point, ldir2d, lpnt2d);
		std::cout << "ComputeSplitPolyInsectLDistToLine   the lpnt2d pnt: " << lpnt2d[0] << " " << lpnt2d[1] <<
			",  ldir2d dir: " << ldir2d[0] << " " << ldir2d[1] << std::endl;
		if (polypnts2d.size() < 2)
		{
			return false;
		}

		Eigen::Vector2f cross_pnt1, cross_pnt2;
		float min_k = std::numeric_limits<float>::max();
		float max_k = std::numeric_limits<float>::lowest();
		Eigen::Vector2i idx_near_far = Eigen::Vector2i(-1, -1);

		for (int i = 0; i < polypnts2d.size() - 1; ++i)
		{
			int j = i + 1;
			//j,i为一个线段
			Eigen::Vector2f ls = Eigen::Vector2f(polypnts2d[i][0], polypnts2d[i][1]);
			Eigen::Vector2f le = Eigen::Vector2f(polypnts2d[j][0], polypnts2d[j][1]);
			//计算线段和直线是否有交点
			Eigen::Vector2f tmp_cross_pnt;
			if (ComputeLineSeg2DCrossLine2DPntsNew(ls, le, m_pnt, m_dir, tmp_cross_pnt))
			{
				//如果有交点，就获取交点到
				float k = Pnt2DDistToLine2D(tmp_cross_pnt, lpnt2d, ldir2d);
				std::cout << "k dist: " << k << std::endl;
				if (min_k > k)
				{
					min_k = k;
					cross_pnt1 = tmp_cross_pnt;
					idx_near_far[0] = i;
				}
				if (max_k < k)
				{
					max_k = k;
					cross_pnt2 = tmp_cross_pnt;
					idx_near_far[1] = j;
				}
			}
		}
		dist_near_far[0] = min_k;
		dist_near_far[1] = max_k;
		near_far_idx = idx_near_far;
		return true;
	}

	bool HWPlane::ComputeSplitPolygonToIntersectionLine(Eigen::Vector3f& L_dir, Eigen::Vector3f& L_point,
		std::vector<int>& split_idxs, float& polygon_dist)
	{
		//
		if (split_idxs.empty())
		{
			polygon_dist = std::numeric_limits<float>::max();
			return false;
		}
		
		float sum_dist = 0;
		for (int i = 0; i < split_idxs.size(); ++i)
		{
			int idx = split_idxs[i];
			Eigen::Vector3f pnt = Eigen::Vector3f(corner_pnts_3d_[idx].x,
				corner_pnts_3d_[idx].y, corner_pnts_3d_[idx].z);
			float dist_tmp3d = Pnt3DDistToLine3D(L_point, L_dir, pnt);

			//std::cout << "dist_tmp3d: " << dist_tmp3d << std::endl;
			sum_dist += dist_tmp3d;
		}
		polygon_dist = sum_dist / split_idxs.size();
		return true;
	}

	bool HWPlane::ComputePolygonRangeDistToIntersectionLineNew(Eigen::Vector3f& L_dir, Eigen::Vector3f& L_point,
		std::vector<int>& split_idxs, float& polygon_dist)
	{
		//
		if (split_idxs.empty())
		{
			polygon_dist = std::numeric_limits<float>::max();
			return false;
		}

		float dist_far = std::numeric_limits<float>::lowest();
		float dist_near = std::numeric_limits<float>::max();

		for (int i = 0; i < split_idxs.size(); ++i)
		{
			int idx = split_idxs[i];
			Eigen::Vector3f pnt = Eigen::Vector3f(corner_pnts_3d_[idx].x,
				corner_pnts_3d_[idx].y, corner_pnts_3d_[idx].z);
			float dist_tmp3d = Pnt3DDistToLine3D(L_point, L_dir, pnt);

			if (dist_tmp3d < dist_near)
			{
				dist_near = dist_tmp3d;
			}
			if (dist_tmp3d > dist_far)
			{
				dist_far = dist_tmp3d;
			}
		}
		polygon_dist = dist_far - dist_near;
		return true;
	}

	bool HWPlane::ComputePolygonRangeDistToIntersectionLine(Eigen::Vector3f& L_dir, Eigen::Vector3f& L_point,
		std::vector<int>& split_idxs, float& polygon_dist)
	{
		if (split_idxs.empty())
		{
			polygon_dist = std::numeric_limits<float>::max();
			return false;
		}

		float dist_far = std::numeric_limits<float>::lowest();
		//float dist_near = std::numeric_limits<float>::max();

		for (int i = 0; i < split_idxs.size(); ++i)
		{
			int idx = split_idxs[i];
			Eigen::Vector3f pnt = Eigen::Vector3f(corner_pnts_3d_[idx].x,
				corner_pnts_3d_[idx].y, corner_pnts_3d_[idx].z);
			float dist_tmp3d = Pnt3DDistToLine3D(L_point, L_dir, pnt);

			if (dist_tmp3d > dist_far)
			{
				dist_far = dist_tmp3d;
			}
		}
		polygon_dist = dist_far;
		return true;
	}

	void HWPlane::ComputePolygonFarestDistToIntersectionLine(Eigen::Vector2f& l_dir2d, Eigen::Vector2f& l_point2d,
		int& farest_idx, float& max_dist)
	{
		float dist_far = std::numeric_limits<float>::lowest();
		for (int i = 0; i < corner_pnts_.size(); ++i)
		{
			Eigen::Vector2f tmppnt2d = Eigen::Vector2f(corner_pnts_[i].x, corner_pnts_[i].y);
			float dist2l = Pnt2DDistToLine2D(tmppnt2d, l_point2d, l_dir2d);
			if (dist2l > dist_far)
			{
				farest_idx = i;
				dist_far = dist2l;
			}
		}
		max_dist = dist_far;
	}

	void HWPlane::ComputePolygonFarestDistToIntersectionLineSegment(Eigen::Vector2f& ls, Eigen::Vector2f& le,
		int& farest_idx, float& max_dist)
	{
		float dist_far = std::numeric_limits<float>::lowest();
		for (int i = 0; i < corner_pnts_.size(); ++i)
		{
			Eigen::Vector2f tmppnt2d = Eigen::Vector2f(corner_pnts_[i].x, corner_pnts_[i].y);
			float dist2l = Pnt2DToLineSegment2D(tmppnt2d, ls, le);
			if (dist2l > dist_far)
			{
				farest_idx = i;
				dist_far = dist2l;
			}
		}
		max_dist = dist_far;
	}

	void HWPlane::RestartPolyIdxsFromSelectedIdx(int& farestidx, std::vector<int>& repolyidx)
	{
		if (farestidx >= corner_pnts_.size() && farestidx < 0)
			return;
		for (int i = farestidx; i < corner_pnts_.size(); ++i)
		{
			repolyidx.emplace_back(i);
		}
		for (int i = 0; i < farestidx; ++i)
		{
			repolyidx.emplace_back(i);
		}
	}

	void HWPlane::ComputeIdxFromSplitPolygonIntersection(Eigen::Vector3f& L_dir, Eigen::Vector3f& L_point,
		std::vector<int>& split_idxs1, std::vector<int>& split_idxs2, std::vector<int>& moved_idxs)
	{
		if (split_idxs1.size() < 2)
		{
			moved_idxs = split_idxs1;
			return;
		}
		
		if (split_idxs2.size() < 2)
		{
			moved_idxs = split_idxs2;
			return;
		}

		//获取两个平面交叉的直线，并且转化为平面坐标系下
		Eigen::Vector2f lpnt2d, ldir2d;
		ComputeWorldLf2PlaneLf(L_dir, L_point, ldir2d, lpnt2d);
		std::cout << "the lpnt2d pnt: " << lpnt2d[0] << " " << lpnt2d[1] <<
			",  ldir2d dir: " << ldir2d[0] << " " << ldir2d[1] << std::endl;
		//对polygon分开的两个线段集合，进行处理，获取离的近的线段polygon，让它们进行移动
		int flag = -1;
		float sum_dist1 = 0;
		float sum_dist2 = 0;
		//处理split_idxs1的线段集合
		for (int i = 0, j = split_idxs1.size() - 1; i < split_idxs1.size(); j = i++)
		{
			//
			int pre_idx = split_idxs1[j];
			int post_idx = split_idxs1[i];
			Eigen::Vector2f pre_pnt = Eigen::Vector2f(corner_pnts_[pre_idx].x, corner_pnts_[pre_idx].y);
			Eigen::Vector2f post_pnt = Eigen::Vector2f(corner_pnts_[post_idx].x, corner_pnts_[post_idx].y);
			float dist2linepre = Pnt2DDistToLine2D(pre_pnt, lpnt2d, ldir2d);
			float dist2linepos = Pnt2DDistToLine2D(post_pnt, lpnt2d, ldir2d);

			sum_dist1 = sum_dist1 + (dist2linepre + dist2linepos) / 2;
		}

		float dist1_average = sum_dist1 / split_idxs1.size();

		//处理split_idxs1的线段集合
		for (int i = 0, j = split_idxs2.size() - 1; i < split_idxs2.size(); j = i++)
		{
			//
			int pre_idx = split_idxs2[j];
			int post_idx = split_idxs2[i];
			Eigen::Vector2f pre_pnt = Eigen::Vector2f(corner_pnts_[pre_idx].x, corner_pnts_[pre_idx].y);
			Eigen::Vector2f post_pnt = Eigen::Vector2f(corner_pnts_[post_idx].x, corner_pnts_[post_idx].y);
			float dist2linepre = Pnt2DDistToLine2D(pre_pnt, lpnt2d, ldir2d);
			float dist2linepos = Pnt2DDistToLine2D(post_pnt, lpnt2d, ldir2d);

			sum_dist2 = sum_dist2 + (dist2linepre + dist2linepos) / 2;
		}
		float dist2_average = sum_dist2 / split_idxs2.size();

		if (dist2_average > dist1_average)
			moved_idxs = split_idxs1;
		else
			moved_idxs = split_idxs2;
	}

	//void HWPlane::ComputeNewPolygonFromSplitPolygonIntersection(Eigen::Vector3f& L_dir, Eigen::Vector3f& L_point,
	//	std::vector<int>& moved_idxs, std::vector<Eigen::Vector2f>& polygon_new)
	//{
	//	//获取两个平面交叉的直线，并且转化为平面坐标系下
	//	Eigen::Vector2f lpnt2d, ldir2d;
	//	ComputeWorldLf2PlaneLf(L_dir, L_point, ldir2d, lpnt2d);
	//	std::cout << "the lpnt2d pnt: " << lpnt2d[0] << " " << lpnt2d[1] <<
	//		",  ldir2d dir: " << ldir2d[0] << " " << ldir2d[1] << std::endl;

	//	//将顶点投影到直线上,明天再写,已经有区分的代码，但是可能需要再改进这一步
	//}

	void HWPlane::ComputeNewPolygonPntsWithNomoveIdx(Eigen::Vector3f& L_dir, Eigen::Vector3f& L_point,
		std::vector<int>& no_move_idxs, float dist_thresh, std::vector<Eigen::Vector2f>& polygon_new)
	{
		Eigen::Vector2f lpnt2d, ldir2d;
		ComputeWorldLf2PlaneLf(L_dir, L_point, ldir2d, lpnt2d);

		/*std::cout << "the lpnt2d pnt: " << lpnt2d[0] << " " << lpnt2d[1] <<
		",  ldir2d dir: " << ldir2d[0] << " " << ldir2d[1] << std::endl;*/
		//计算顶点到平面的投影,两个顶点
		if (no_move_idxs.size() >= 2)
		{
			int pidx1 = no_move_idxs[0];
			int pidx2 = no_move_idxs[no_move_idxs.size() - 1];

			Eigen::Vector2f p1 = Eigen::Vector2f(corner_pnts_[pidx1].x, corner_pnts_[pidx1].y);
			Eigen::Vector2f p2 = Eigen::Vector2f(corner_pnts_[pidx2].x, corner_pnts_[pidx2].y);

			//std::cout << "the pnt left right idx: " << pidx1 << " " << pidx2 << std::endl;
			////
			//Eigen::Vector2f proj_p1, proj_p2;
			////std::cout << "the p1: " << p1[0] << " " << p1[1] << std::endl;
			//Pnt2dProjLine2D(p1, lpnt2d, ldir2d, proj_p1);
			////std::cout << "the proj p1: " << proj_p1[0] << " " << proj_p1[1] << std::endl;
			////std::cout << "the p2: " << p2[0] << " " << p2[1] << std::endl;
			//Pnt2dProjLine2D(p2, lpnt2d, ldir2d, proj_p2);
			////std::cout << "the proj p2: " << proj_p2[0] << " " << proj_p2[1] << std::endl;

			float2 projp1, projp2;
			projp1 = ProjToLine3D(pidx1, L_dir, L_point);
			projp2 = ProjToLine3D(pidx2, L_dir, L_point);

			Eigen::Vector2f proj_p1 = Eigen::Vector2f(projp1.x, projp1.y);
			Eigen::Vector2f proj_p2 = Eigen::Vector2f(projp2.x, projp2.y);

			std::vector<Eigen::Vector2f> tmp_pnts2d;
			std::pair<int, int> p2idx2tmpp2idx;
			std::pair<int, int> p1idx2tmpp1idx;
			//处理这些顶点
			for (int i = 0; i < no_move_idxs.size(); ++i)
			{
				Eigen::Vector2f tmppnt =
					Eigen::Vector2f(corner_pnts_[no_move_idxs[i]].x, corner_pnts_[no_move_idxs[i]].y);
				
				if (no_move_idxs[i] == pidx1)
				{
					p1idx2tmpp1idx.first = pidx1;
					p1idx2tmpp1idx.second = i;
				}
				if (no_move_idxs[i] == pidx2)
				{
					p2idx2tmpp2idx.first = pidx2;
					p2idx2tmpp2idx.second = i;
				}

				tmp_pnts2d.emplace_back(tmppnt);
			}

			//如果两个顶点很近的话，就不在生成新的点
			if ((p2 - proj_p2).norm() > dist_thresh)
				tmp_pnts2d.emplace_back(proj_p2);
			else
				tmp_pnts2d[p2idx2tmpp2idx.second] = proj_p2;

			if ((p1 - proj_p1).norm() > dist_thresh)
				tmp_pnts2d.emplace_back(proj_p1);
			else
				tmp_pnts2d[p1idx2tmpp1idx.second] = proj_p1;

			polygon_new = tmp_pnts2d;
		}
	}

	//需要lr_idx[0] = no_move_idxs[0],并且lr_idx[1] = no_move_idxs[no_move_idxs.size() -1]
	int HWPlane::ComputeNewPolyPntsWithNomoveIdxSplitPoly(Eigen::Vector3f& L_dir, Eigen::Vector3f& L_point, std::vector<int>& split_polygon, std::vector<int>& no_move_idxs,
		Eigen::Vector2i& lr_idx, Eigen::Vector2i& connect_idx, float dist_thresh, std::vector<Eigen::Vector2f>& polygon_new)
	{
#if 1
		int expand_type_idx = -1;

		if (no_move_idxs.size() < 2)
			return expand_type_idx;

		//
		int end_nomveidx = no_move_idxs[no_move_idxs.size() - 1];
		int end_nomveidxi = -1;
		for (int i = 0; i < split_polygon.size(); ++i)
		{
			if (end_nomveidx == split_polygon[i])
			{
				end_nomveidxi = i;
				break;
			}
		}
		int select_connectidxi = -1;
		for (int i = end_nomveidxi; i < split_polygon.size() + end_nomveidxi; ++i)
		{
			//
			int tmp_idxi = i % split_polygon.size();
			if (split_polygon[tmp_idxi] == connect_idx[0])
			{
				//std::cout << "the tmp_idxi: " << tmp_idxi << std::endl;
				//std::cout << "split_polygon[tmp_idxi]: " << split_polygon[tmp_idxi] << std::endl;
				select_connectidxi = 0;
				break;
			}
			if (split_polygon[tmp_idxi] == connect_idx[1])
			{
				//std::cout << "the tmp_idxi: " << tmp_idxi << std::endl;
				//std::cout << "split_polygon[tmp_idxi]: " << split_polygon[tmp_idxi] << std::endl;
				select_connectidxi = 1;
				break;
			}
		}

		if (no_move_idxs.size() >= 2)
		{
			if (lr_idx[0] == lr_idx[1])
				return expand_type_idx;

			//获取最初的两个polygon的索引,其中pidx1 < pidx2
			int pidx1 = connect_idx[0];
			int pidx2 = connect_idx[1];
			std::cout << "the start end idx: " << pidx1 << " " << pidx2 << std::endl;
			/*std::string path1 = "D:\\vc_project_new\\huawei_data_indoor\\room\\test_room_cam2\\start_end_scene1.obj";
			std::ofstream fh(path1);
			fh << "v " << corner_pnts_3d_[pidx1].x << " " << corner_pnts_3d_[pidx1].y << " "
				<< corner_pnts_3d_[pidx1].z << " " << 255 << " " << 0 << " " << 0 << std::endl;
			fh << "v " << corner_pnts_3d_[pidx2].x << " " << corner_pnts_3d_[pidx2].y << " "
				<< corner_pnts_3d_[pidx2].z << " " << 0 << " " << 0 << " " << 255 << std::endl;
			fh << "l " << 1 << " " << 2 << std::endl;
			fh.close();*/

			Eigen::Vector2f p1 = Eigen::Vector2f(corner_pnts_[pidx1].x, corner_pnts_[pidx1].y);
			Eigen::Vector2f p2 = Eigen::Vector2f(corner_pnts_[pidx2].x, corner_pnts_[pidx2].y);
			float2 projp1, projp2;
			projp1 = ProjToLine3D(pidx1, L_dir, L_point);
			projp2 = ProjToLine3D(pidx2, L_dir, L_point);
			Eigen::Vector2f proj_p1 = Eigen::Vector2f(projp1.x, projp1.y);
			Eigen::Vector2f proj_p2 = Eigen::Vector2f(projp2.x, projp2.y);

			std::vector<Eigen::Vector2f> tmp_pnts2d;
			//被最左右两端切割的是从小到大lr_idx[0] < lr_idx[1]
			if (select_connectidxi == 1)
			{
				if (lr_idx[0] == pidx1 && lr_idx[1] == pidx2)
				{
					std::cout << "111111111" << std::endl;
					for (int i = 0; i < no_move_idxs.size(); ++i)
					{
						if (i == 0)
						{
							//添加一个初始的顶点
							if ((p1 - proj_p1).norm() > dist_thresh)
							{
								tmp_pnts2d.emplace_back(proj_p1);
								tmp_pnts2d.emplace_back(p1);
							}
							else
							{
								tmp_pnts2d.emplace_back(proj_p1);
							}
						}
						else if (i == (no_move_idxs.size() - 1))
						{
							//添加一个初始的顶点
							if ((p2 - proj_p2).norm() > dist_thresh)
							{
								tmp_pnts2d.emplace_back(p2);
								tmp_pnts2d.emplace_back(proj_p2);
							}
							else
							{
								tmp_pnts2d.emplace_back(proj_p2);
							}
						}
						else
						{
							Eigen::Vector2f tmppnt =
								Eigen::Vector2f(corner_pnts_[no_move_idxs[i]].x, corner_pnts_[no_move_idxs[i]].y);
							tmp_pnts2d.emplace_back(tmppnt);
						}	
					}
					expand_type_idx = 1;
				}
				else if (lr_idx[0] == pidx1 && lr_idx[1] != pidx2)
				{
					std::cout << "22222222" << std::endl;
					for (int i = 0; i < no_move_idxs.size(); ++i)
					{
						if (i == 0)
						{
							//添加一个投影的顶点
							if ((p1 - proj_p1).norm() > dist_thresh)
							{
								tmp_pnts2d.emplace_back(proj_p1);
								tmp_pnts2d.emplace_back(p1);
							}
							else
							{
								tmp_pnts2d.emplace_back(proj_p1);
							}
						}
						else if (i == (no_move_idxs.size() - 1))
						{
							//添加一个投影顶点
							Eigen::Vector2f p2pre = Eigen::Vector2f(corner_pnts_[no_move_idxs[i]].x, corner_pnts_[no_move_idxs[i]].y);

							float2 projp2pre;
							projp2pre = ProjToLine3D(no_move_idxs[i], L_dir, L_point);
							Eigen::Vector2f proj_p2pre = Eigen::Vector2f(projp2pre.x, projp2pre.y);

							if ((proj_p2pre - p2pre).norm() > dist_thresh)
							{
								//顺序是啥样的？
								tmp_pnts2d.emplace_back(p2pre);
								tmp_pnts2d.emplace_back(proj_p2pre);
							}
							else
							{
								//直接移动顶点（因为它们离的很近）
								tmp_pnts2d.emplace_back(proj_p2pre);
							}

							/*if ((p2 - proj_p2).norm() > dist_thresh)
							{
								tmp_pnts2d.emplace_back(proj_p2);
								tmp_pnts2d.emplace_back(p2);
							}
							else
							{
								tmp_pnts2d.emplace_back(proj_p2);
							}*/
						}
						else
						{
							Eigen::Vector2f tmppnt =
								Eigen::Vector2f(corner_pnts_[no_move_idxs[i]].x, corner_pnts_[no_move_idxs[i]].y);
							tmp_pnts2d.emplace_back(tmppnt);
						}
					}
					//移动连接的边缘顶点
					tmp_pnts2d.emplace_back(proj_p2);
					expand_type_idx = 2;
				}
				else if (lr_idx[0] != pidx1 && lr_idx[1] == pidx2)
				{
					std::cout << "lr_idx[0] != pidx1 && lr_idx[1] == pidx2 type" << std::endl;
					//移动连接的边缘顶点
					tmp_pnts2d.emplace_back(proj_p1);
					for (int i = 0; i < no_move_idxs.size(); ++i)
					{
						if (i == 0)
						{
							//添加一个投影顶点
							Eigen::Vector2f p1next = Eigen::Vector2f(corner_pnts_[no_move_idxs[i]].x, corner_pnts_[no_move_idxs[i]].y);

							float2 projp1next;
							projp1next = ProjToLine3D(no_move_idxs[i], L_dir, L_point);
							Eigen::Vector2f proj_p1next = Eigen::Vector2f(projp1next.x, projp1next.y);

							if ((proj_p1next - p1next).norm() > dist_thresh)
							{
								//顺序是啥样的？
								tmp_pnts2d.emplace_back(proj_p1next);
								tmp_pnts2d.emplace_back(p1next);
							}
							else
							{
								//直接移动顶点（因为它们离的很近）
								tmp_pnts2d.emplace_back(proj_p1next);
							}
						}
						else if (i == (no_move_idxs.size() - 1))
						{
							//添加一个投影的顶点
							if ((p2 - proj_p2).norm() > dist_thresh)
							{
								tmp_pnts2d.emplace_back(p2);
								tmp_pnts2d.emplace_back(proj_p2);
							}
							else
							{
								tmp_pnts2d.emplace_back(proj_p2);
							}
						}
						else
						{
							Eigen::Vector2f tmppnt =
								Eigen::Vector2f(corner_pnts_[no_move_idxs[i]].x, corner_pnts_[no_move_idxs[i]].y);
							tmp_pnts2d.emplace_back(tmppnt);
						}
					}
					expand_type_idx = 3;
				}
				else if (lr_idx[0] != pidx1 && lr_idx[1] != pidx2)
				{
					std::cout << "lr_idx[0] != pidx1 && lr_idx[1] != pidx2 type" << std::endl;
					//移动连接另一个polygon的边缘顶点
					tmp_pnts2d.emplace_back(proj_p1);
					for (int i = 0; i < no_move_idxs.size(); ++i)
					{
						if (i == 0)
						{
							//添加一个投影顶点
							Eigen::Vector2f p1next = Eigen::Vector2f(corner_pnts_[no_move_idxs[i]].x, corner_pnts_[no_move_idxs[i]].y);

							float2 projp1next;
							projp1next = ProjToLine3D(no_move_idxs[i], L_dir, L_point);
							Eigen::Vector2f proj_p1next = Eigen::Vector2f(projp1next.x, projp1next.y);

							if ((proj_p1next - p1next).norm() > dist_thresh)
							{
								//顺序是啥样的？
								tmp_pnts2d.emplace_back(proj_p1next);
								tmp_pnts2d.emplace_back(p1next);
							}
							else
							{
								//直接移动顶点（因为它们离的很近）
								tmp_pnts2d.emplace_back(proj_p1next);
							}
						}
						else if (i == (no_move_idxs.size() - 1))
						{
							//添加一个投影顶点
							Eigen::Vector2f p2pre = Eigen::Vector2f(corner_pnts_[no_move_idxs[i]].x, corner_pnts_[no_move_idxs[i]].y);

							float2 projp2pre;
							projp2pre = ProjToLine3D(no_move_idxs[i], L_dir, L_point);
							Eigen::Vector2f proj_p2pre = Eigen::Vector2f(projp2pre.x, projp2pre.y);

							if ((proj_p2pre - p2pre).norm() > dist_thresh)
							{
								//顺序是啥样的？
								tmp_pnts2d.emplace_back(p2pre);
								tmp_pnts2d.emplace_back(proj_p2pre);
							}
							else
							{
								//直接移动顶点（因为它们离的很近）
								tmp_pnts2d.emplace_back(proj_p2pre);
							}
						}
						else
						{
							Eigen::Vector2f tmppnt =
								Eigen::Vector2f(corner_pnts_[no_move_idxs[i]].x, corner_pnts_[no_move_idxs[i]].y);
							tmp_pnts2d.emplace_back(tmppnt);
						}
					}
					//移动连接另一个polygon的边缘顶点
					tmp_pnts2d.emplace_back(proj_p2);
					expand_type_idx = 4;					
				}
				else
				{
					std::cout << "other process..." << std::endl;
				}
			}
			//这个是从大到小，这个途中会遇到no_move_idx的部分
			else if(select_connectidxi == 0)
			{
				if (lr_idx[0] == pidx2 && lr_idx[1] == pidx1)
				{
					//
					std::cout << "5555555" << std::endl;
					for (int i = 0; i < no_move_idxs.size(); ++i)
					{
						if (i == 0)
						{
							//添加一个初始的顶点
							if ((p2 - proj_p2).norm() > dist_thresh)
							{
								tmp_pnts2d.emplace_back(proj_p2);
								tmp_pnts2d.emplace_back(p2);
							}
							else
							{
								tmp_pnts2d.emplace_back(proj_p2);
							}
						}
						else if (i == (no_move_idxs.size() - 1))
						{
							//添加一个初始的顶点
							if ((p1 - proj_p1).norm() > dist_thresh)
							{
								tmp_pnts2d.emplace_back(p1);
								tmp_pnts2d.emplace_back(proj_p1);
							}
							else
							{
								tmp_pnts2d.emplace_back(proj_p1);
							}
						}
						else
						{
							Eigen::Vector2f tmppnt =
								Eigen::Vector2f(corner_pnts_[no_move_idxs[i]].x, corner_pnts_[no_move_idxs[i]].y);
							tmp_pnts2d.emplace_back(tmppnt);
						}
					}
					expand_type_idx = 5;
				}
				else if (lr_idx[0] == pidx2 && lr_idx[1] != pidx1)
				{
					std::cout << "666666666" << std::endl;
					for (int i = 0; i < no_move_idxs.size(); ++i)
					{
						//
						if (i == 0)
						{
							//添加一个投影的顶点
							if ((p2 - proj_p2).norm() > dist_thresh)
							{
								tmp_pnts2d.emplace_back(proj_p2);
								tmp_pnts2d.emplace_back(p2);
							}
							else
							{
								tmp_pnts2d.emplace_back(proj_p2);
							}
						}
						else if (i == (no_move_idxs.size() - 1))
						{
							//添加一个投影顶点
							Eigen::Vector2f p1pre = Eigen::Vector2f(corner_pnts_[no_move_idxs[i]].x, corner_pnts_[no_move_idxs[i]].y);

							float2 projp1pre;
							projp1pre = ProjToLine3D(no_move_idxs[i], L_dir, L_point);
							Eigen::Vector2f proj_p1pre = Eigen::Vector2f(projp1pre.x, projp1pre.y);

							if ((proj_p1pre - p1pre).norm() > dist_thresh)
							{
								//顺序是啥样的？
								tmp_pnts2d.emplace_back(p1pre);
								tmp_pnts2d.emplace_back(proj_p1pre);
							}
							else
							{
								//直接移动顶点（因为它们离的很近）
								tmp_pnts2d.emplace_back(proj_p1pre);
							}
						}
						else
						{
							Eigen::Vector2f tmppnt =
								Eigen::Vector2f(corner_pnts_[no_move_idxs[i]].x, corner_pnts_[no_move_idxs[i]].y);
							tmp_pnts2d.emplace_back(tmppnt);
						}
					}
					//移动连接的边缘顶点
					tmp_pnts2d.emplace_back(proj_p1);
					expand_type_idx = 6;
				}
				else if (lr_idx[0] != pidx2 && lr_idx[1] == pidx1)
				{
					std::cout << "7777777" << std::endl;
					//移动连接的边缘顶点
					tmp_pnts2d.emplace_back(proj_p2);
					for (int i = 0; i < no_move_idxs.size(); ++i)
					{
						if (i == 0)
						{
							//添加一个投影顶点
							Eigen::Vector2f p2next = Eigen::Vector2f(corner_pnts_[no_move_idxs[i]].x, corner_pnts_[no_move_idxs[i]].y);
							float2 projp2next;
							projp2next = ProjToLine3D(no_move_idxs[i], L_dir, L_point);
							Eigen::Vector2f proj_p2next = Eigen::Vector2f(projp2next.x, projp2next.y);

							if ((proj_p2next - p2next).norm() > dist_thresh)
							{
								//顺序是啥样的？
								tmp_pnts2d.emplace_back(proj_p2next);
								tmp_pnts2d.emplace_back(p2next);
							}
							else
							{
								//直接移动顶点（因为它们离的很近）
								tmp_pnts2d.emplace_back(proj_p2next);
							}
						}
						else if (i == (no_move_idxs.size() - 1))
						{
							if ((p1 - proj_p1).norm() > dist_thresh)
							{
								tmp_pnts2d.emplace_back(p1);
								tmp_pnts2d.emplace_back(proj_p1);
							}
							else
							{
								tmp_pnts2d.emplace_back(proj_p1);
							}
						}
						else
						{
							Eigen::Vector2f tmppnt =
								Eigen::Vector2f(corner_pnts_[no_move_idxs[i]].x, corner_pnts_[no_move_idxs[i]].y);
							tmp_pnts2d.emplace_back(tmppnt);
						}
					}
					expand_type_idx = 7;
				}
				else if (lr_idx[0] != pidx2 && lr_idx[1] != pidx1)
				{
					std::cout << "888888888" << std::endl;
					//移动连接的边缘顶点
					tmp_pnts2d.emplace_back(proj_p2);
					for (int i = 0; i < no_move_idxs.size(); ++i)
					{
						if (i == 0)
						{
							//添加一个投影顶点
							Eigen::Vector2f p2next = Eigen::Vector2f(corner_pnts_[no_move_idxs[i]].x, corner_pnts_[no_move_idxs[i]].y);
							float2 projp2next;
							projp2next = ProjToLine3D(no_move_idxs[i], L_dir, L_point);
							Eigen::Vector2f proj_p2next = Eigen::Vector2f(projp2next.x, projp2next.y);

							if ((proj_p2next - p2next).norm() > dist_thresh)
							{
								//顺序是啥样的？
								tmp_pnts2d.emplace_back(proj_p2next);
								tmp_pnts2d.emplace_back(p2next);
							}
							else
							{
								//直接移动顶点（因为它们离的很近）
								tmp_pnts2d.emplace_back(proj_p2next);
							}
						}
						else if (i == (no_move_idxs.size() - 1))
						{
							//添加一个投影顶点
							Eigen::Vector2f p1pre = Eigen::Vector2f(corner_pnts_[no_move_idxs[i]].x, corner_pnts_[no_move_idxs[i]].y);
							float2 projp1pre;
							projp1pre = ProjToLine3D(no_move_idxs[i], L_dir, L_point);
							Eigen::Vector2f proj_p1pre = Eigen::Vector2f(projp1pre.x, projp1pre.y);

							if ((proj_p1pre - p1pre).norm() > dist_thresh)
							{
								//顺序是啥样的？
								tmp_pnts2d.emplace_back(p1pre);
								tmp_pnts2d.emplace_back(proj_p1pre);
							}
							else
							{
								//直接移动顶点（因为它们离的很近）
								tmp_pnts2d.emplace_back(proj_p1pre);
							}
						}
						else
						{
							Eigen::Vector2f tmppnt =
								Eigen::Vector2f(corner_pnts_[no_move_idxs[i]].x, corner_pnts_[no_move_idxs[i]].y);
							tmp_pnts2d.emplace_back(tmppnt);
						}
					}
					//移动连接的边缘顶点
					tmp_pnts2d.emplace_back(proj_p1);
					expand_type_idx = 8;
				}
				else
				{
					std::cout << "to do next time......" << std::endl;
				}
			}
			else
			{
				std::cout << "no split existing......" << std::endl;
			}

			polygon_new = tmp_pnts2d;
		}

		return select_connectidxi;
#endif

#if 0
		//计算顶点到平面的投影,两个顶点
		if (no_move_idxs.size() >= 2)
		{
			int pidx1 = connect_idx[0];
			int pidx2 = connect_idx[1];

			std::string path1 = "D:\\vc_project_new\\huawei_data_indoor\\room\\test_room_cam1\\start_end_scene.obj";
			std::ofstream fh(path1);
			fh << "v " << corner_pnts_3d_[pidx1].x << " " << corner_pnts_3d_[pidx1].y << " "
				<< corner_pnts_3d_[pidx1].z << " " << 255 << " " << 0 << " " << 0 << std::endl;
			fh << "v " << corner_pnts_3d_[pidx2].x << " " << corner_pnts_3d_[pidx2].y << " "
				<< corner_pnts_3d_[pidx2].z << " " << 0 << " " << 0 << " " << 255 << std::endl;
			fh << "l " << 1 << " " << 2 << std::endl;
			fh.close();

			Eigen::Vector2f p1 = Eigen::Vector2f(corner_pnts_[pidx1].x, corner_pnts_[pidx1].y);
			Eigen::Vector2f p2 = Eigen::Vector2f(corner_pnts_[pidx2].x, corner_pnts_[pidx2].y);

			//std::cout << "the pnt left right idx: " << pidx1 << " " << pidx2 << std::endl;
			////
			//Eigen::Vector2f proj_p1, proj_p2;
			////std::cout << "the p1: " << p1[0] << " " << p1[1] << std::endl;
			//Pnt2dProjLine2D(p1, lpnt2d, ldir2d, proj_p1);
			////std::cout << "the proj p1: " << proj_p1[0] << " " << proj_p1[1] << std::endl;
			////std::cout << "the p2: " << p2[0] << " " << p2[1] << std::endl;
			//Pnt2dProjLine2D(p2, lpnt2d, ldir2d, proj_p2);
			////std::cout << "the proj p2: " << proj_p2[0] << " " << proj_p2[1] << std::endl;

			float2 projp1, projp2;
			projp1 = ProjToLine3D(pidx1, L_dir, L_point);
			projp2 = ProjToLine3D(pidx2, L_dir, L_point);

			Eigen::Vector2f proj_p1 = Eigen::Vector2f(projp1.x, projp1.y);
			Eigen::Vector2f proj_p2 = Eigen::Vector2f(projp2.x, projp2.y);
			std::vector<Eigen::Vector2f> tmp_pnts2d;

			//对其它的顶点进行处理
			/*int pidx2i = no_move_idxs.size() - 1;
			int pidx1i = 0;
			int pidx2pre = no_move_idxs[no_move_idxs.size() - 2];
			int pidx2prei = no_move_idxs.size() - 2;
			int pidx1next = no_move_idxs[1];
			int pidx1nexti = 1;*/

			for (int i = 0; i < no_move_idxs.size(); ++i)
			{

			}

			int pidx2i = no_move_idxs.size() - 1;
			int pidx1i = 0;
			int pidx2pre = no_move_idxs[no_move_idxs.size() - 2];
			int pidx2prei = no_move_idxs.size() - 2;
			int pidx1next = no_move_idxs[1];
			int pidx1nexti = 1;

			if (lr_idx[0] == lr_idx[1])
				return;

			/*if (lr_idx[0] == pidx1)
			{
				if (lr_idx[1] != pidx1)
				{
				}
				else if (lr_idx[1] == pidx1)
				{
				}
			}
			else if(lr_idx[1] == pidx1)
			{
			}
			else 
			{
			}*/

			if ((lr_idx[0] == pidx1 && lr_idx[1] == pidx2) || 
				(lr_idx[0] == pidx2 && lr_idx[1] == pidx1))
			{	
				std::cout << "1111111" << std::endl;
				std::pair<int, int> p2idx2tmpp2idx;
				std::pair<int, int> p1idx2tmpp1idx;
				//处理这些顶点
				for (int i = 0; i < no_move_idxs.size(); ++i)
				{
					Eigen::Vector2f tmppnt =
						Eigen::Vector2f(corner_pnts_[no_move_idxs[i]].x, corner_pnts_[no_move_idxs[i]].y);

					if (no_move_idxs[i] == pidx1)
					{
						p1idx2tmpp1idx.first = pidx1;
						p1idx2tmpp1idx.second = i;
					}
					if (no_move_idxs[i] == pidx2)
					{
						p2idx2tmpp2idx.first = pidx2;
						p2idx2tmpp2idx.second = i;
					}

					tmp_pnts2d.emplace_back(tmppnt);
				}

				//如果两个顶点很近的话，就不在生成新的点
				if ((p2 - proj_p2).norm() > dist_thresh)
					tmp_pnts2d.emplace_back(proj_p2);
				else
					tmp_pnts2d[p2idx2tmpp2idx.second] = proj_p2;

				if ((p1 - proj_p1).norm() > dist_thresh)
					tmp_pnts2d.emplace_back(proj_p1);
				else
					tmp_pnts2d[p1idx2tmpp1idx.second] = proj_p1;

				//polygon_new = tmp_pnts2d;
			}
			else if (lr_idx[0] != pidx1 && lr_idx[1] == pidx2)
			{
				std::cout << "2222222" << std::endl;

				//处理这些顶点，
				for (int i = 0; i < no_move_idxs.size(); ++i)
				{
					if (i == 0)
					{
						//移动顶点
						//p1 改变为 proj_p1
						tmp_pnts2d.emplace_back(proj_p1);
					}
					else if (i == pidx1nexti)
					{
						//添加一个投影顶点
						Eigen::Vector2f p1next = Eigen::Vector2f(corner_pnts_[pidx1next].x, corner_pnts_[pidx1next].y);

						float2 projp1next;
						projp1next = ProjToLine3D(pidx1next, L_dir, L_point);
						Eigen::Vector2f proj_p1next = Eigen::Vector2f(projp1next.x, projp1next.y);

						if ((proj_p1next - p1next).norm() > dist_thresh)
						{
							//顺序是啥样的？
							tmp_pnts2d.emplace_back(proj_p1next);
							tmp_pnts2d.emplace_back(p1next);
						}
						else
						{
							//直接移动顶点（因为它们离的很近）
							tmp_pnts2d.emplace_back(proj_p1next);
						}
					}
					else if (i == pidx2i)
					{
						//添加一个投影顶点
						if ((p2 - proj_p2).norm() > dist_thresh)
						{
							//顺序是啥样的？
							tmp_pnts2d.emplace_back(p2);
							tmp_pnts2d.emplace_back(proj_p2);
						}
						else
						{
							//直接移动顶点（因为它们离的很近）
							tmp_pnts2d.emplace_back(proj_p2);
						}
					}
					else
					{
						Eigen::Vector2f tmppnt =
							Eigen::Vector2f(corner_pnts_[no_move_idxs[i]].x, corner_pnts_[no_move_idxs[i]].y);
						tmp_pnts2d.emplace_back(tmppnt);
					}
				}

			}
			else if (lr_idx[0] == pidx1 && lr_idx[1] != pidx2)
			{
				std::cout << "33333333" << std::endl;
				//处理这些顶点，
				for (int i = 0; i < no_move_idxs.size(); ++i)
				{
					if (i == pidx1i)
					{
						//添加一个投影顶点
						if ((p1 - proj_p1).norm() > dist_thresh)
						{
							//顺序是啥样的？
							tmp_pnts2d.emplace_back(proj_p1);
							tmp_pnts2d.emplace_back(p1);
						}
						else
						{
							//直接移动顶点（因为它们离的很近）
							tmp_pnts2d.emplace_back(proj_p1);
						}
					}
					else if (i == pidx2prei)
					{
						//添加一个投影顶点
						Eigen::Vector2f p2pre = Eigen::Vector2f(corner_pnts_[pidx2pre].x, corner_pnts_[pidx2pre].y);

						float2 projp2pre;
						projp2pre = ProjToLine3D(pidx2pre, L_dir, L_point);
						Eigen::Vector2f proj_p2pre = Eigen::Vector2f(projp2pre.x, projp2pre.y);

						if ((proj_p2pre - p2pre).norm() > dist_thresh)
						{
							//顺序是啥样的？
							tmp_pnts2d.emplace_back(p2pre);
							tmp_pnts2d.emplace_back(proj_p2pre);
						}
						else
						{
							//直接移动顶点（因为它们离的很近）
							tmp_pnts2d.emplace_back(proj_p2pre);
						}
					}
					else if (i == pidx2i)
					{
						//移动顶点
						tmp_pnts2d.emplace_back(proj_p2);
					}
					else
					{
						Eigen::Vector2f tmppnt =
							Eigen::Vector2f(corner_pnts_[no_move_idxs[i]].x, corner_pnts_[no_move_idxs[i]].y);
						tmp_pnts2d.emplace_back(tmppnt);
					}
				}
			}
			else if (lr_idx[0] == pidx2 && lr_idx[1] != pidx1)
			{
				//
				std::cout << "4444444444" << std::endl;
				std::cout << "the lr_idx: " << lr_idx[0] << " " << lr_idx[1] << std::endl;
				std::cout << "the pidx1, pidx2: " << pidx1 << " " << pidx2 << std::endl;

				//处理这些顶点，
				for (int i = 0; i < no_move_idxs.size(); ++i)
				{
					if (i == pidx1i)
					{
						//直接移动顶点到投影点上
						tmp_pnts2d.emplace_back(proj_p1);
					}
					else if (i == pidx1nexti)
					{
						//添加一个投影点
						std::cout << "the next pidx1 idx: " << pidx1next << std::endl;

						Eigen::Vector2f p1next = Eigen::Vector2f(corner_pnts_[pidx1next].x, corner_pnts_[pidx1next].y);

						float2 projp1next;
						projp1next = ProjToLine3D(pidx1next, L_dir, L_point);
						Eigen::Vector2f proj_p1next = Eigen::Vector2f(projp1next.x, projp1next.y);

						if ((proj_p1next - p1next).norm() > dist_thresh)
						{
							//顺序是啥样的？
							tmp_pnts2d.emplace_back(proj_p1next);
							tmp_pnts2d.emplace_back(p1next);
						}
						else
						{
							//直接移动顶点（因为它们离的很近）
							tmp_pnts2d.emplace_back(proj_p1next);
						}
					}
					else if (i == pidx2i)
					{
						//添加一个投影顶点
						if ((p2 - proj_p2).norm() > dist_thresh)
						{
							//顺序是啥样的？
							tmp_pnts2d.emplace_back(p2);
							tmp_pnts2d.emplace_back(proj_p2);
						}
						else
						{
							//直接移动顶点（因为它们离的很近）
							tmp_pnts2d.emplace_back(proj_p2);
						}
					}
					else
					{
						Eigen::Vector2f tmppnt =
							Eigen::Vector2f(corner_pnts_[no_move_idxs[i]].x, corner_pnts_[no_move_idxs[i]].y);
						tmp_pnts2d.emplace_back(tmppnt);
					}
				}
			}
			else if (lr_idx[0] != pidx1 && lr_idx[1] != pidx2 
				&& lr_idx[0] != pidx2 && lr_idx[1] != pidx1)
			{
				std::cout << "77777777" << std::endl;
				std::cout << "the lr_idx: " << lr_idx[0] << " " << lr_idx[1] << std::endl;
				std::cout << "the pidx1, pidx2: " << pidx1 << " " << pidx2 << std::endl;
				//处理这些顶点，
				for (int i = 0; i < no_move_idxs.size(); ++i)
				{
					if (i == pidx1i)
					{
						//直接移动顶点到投影点上
						tmp_pnts2d.emplace_back(proj_p1);
					}
					else if (i == pidx1nexti)
					{
						//添加一个投影点
						Eigen::Vector2f p1next = Eigen::Vector2f(corner_pnts_[pidx1next].x, corner_pnts_[pidx1next].y);

						float2 projp1next;
						projp1next = ProjToLine3D(pidx1next, L_dir, L_point);
						Eigen::Vector2f proj_p1next = Eigen::Vector2f(projp1next.x, projp1next.y);

						if ((proj_p1next - p1next).norm() > dist_thresh)
						{
							//顺序是啥样的？
							tmp_pnts2d.emplace_back(proj_p1next);
							tmp_pnts2d.emplace_back(p1next);
						}
						else
						{
							//直接移动顶点（因为它们离的很近）
							tmp_pnts2d.emplace_back(proj_p1next);
						}
					}
					else if (i == pidx2prei)
					{
						//添加一个投影顶点
						Eigen::Vector2f p2pre = Eigen::Vector2f(corner_pnts_[pidx2pre].x, corner_pnts_[pidx2pre].y);

						float2 projp2pre;
						projp2pre = ProjToLine3D(pidx2pre, L_dir, L_point);
						Eigen::Vector2f proj_p2pre = Eigen::Vector2f(projp2pre.x, projp2pre.y);

						if ((proj_p2pre - p2pre).norm() > dist_thresh)
						{
							//顺序是啥样的？
							tmp_pnts2d.emplace_back(p2pre);
							tmp_pnts2d.emplace_back(proj_p2pre);
						}
						else
						{
							//直接移动顶点（因为它们离的很近）
							tmp_pnts2d.emplace_back(proj_p2pre);
						}
					}
					else if (i == pidx2i)
					{
						//直接移动顶点（因为它们离的很近）
						tmp_pnts2d.emplace_back(proj_p2);
					}
					else
					{
						Eigen::Vector2f tmppnt =
							Eigen::Vector2f(corner_pnts_[no_move_idxs[i]].x, corner_pnts_[no_move_idxs[i]].y);
						tmp_pnts2d.emplace_back(tmppnt);
					}
				}
			}
			else
			{
				std::cout << "to do next..." << std::endl;
			}

			polygon_new = tmp_pnts2d;
		}
#endif
	}

	void HWPlane::ComputeWorldLf2PlaneLf(Eigen::Vector3f& L_dir, Eigen::Vector3f& L_point,
		Eigen::Vector2f& l_dir2d, Eigen::Vector2f& l_point2d)
	{
#if 1
		Eigen::Vector4f temp = world_to_plane_.topLeftCorner(3, 3)*L_dir;
		float2 L_dir_2D = make_float2(temp(0), temp(1));
		l_dir2d = Eigen::Vector2f(L_dir_2D.x, L_dir_2D.y);

		temp = world_to_plane_*Eigen::Vector4f(L_point(0), L_point(1), L_point(2), 1.0f);
		float2 L_point_2D = make_float2(temp(0), temp(1));
		l_point2d = Eigen::Vector2f(L_point_2D.x, L_point_2D.y);

#else
		//需要查看一下新的方式
		Eigen::Vector4f plane_ls4 = world_to_plane_*Eigen::Vector4f(L_point(0), L_point(1), L_point(2), 1.0f);
		Eigen::Vector3f le3 = L_point + L_dir;
		Eigen::Vector4f plane_le4 = world_to_plane_*Eigen::Vector4f(le3(0), le3(1), le3(2), 1.0f);

		l_point2d = Eigen::Vector2f(plane_ls4[0], plane_ls4[1]);
		Eigen::Vector4f plane_dir4 = plane_le4 - plane_ls4;
		l_dir2d = Eigen::Vector2f(plane_dir4[0], plane_dir4[1]);
#endif
	}

	void HWPlane::ComputeVerticalLineFromTwoPnts2D(Eigen::Vector2i lr_idx, Eigen::Vector2f& l_point2d, Eigen::Vector2f& l_dir2d,
		Eigen::Vector2f& mean_pnt, Eigen::Vector2f& mean_dir)
	{
		Eigen::Vector2f l_pnt = Eigen::Vector2f(corner_pnts_[lr_idx[0]].x, corner_pnts_[lr_idx[0]].y);
		Eigen::Vector2f r_pnt = Eigen::Vector2f(corner_pnts_[lr_idx[1]].x, corner_pnts_[lr_idx[1]].y);
		
		//顶点投到直线上
		Eigen::Vector2f prj_l;
		Pnt2dProjLine2D(l_pnt, l_point2d, l_dir2d,  prj_l);
		Eigen::Vector2f prj_r;
		Pnt2dProjLine2D(r_pnt, l_point2d, l_dir2d,  prj_r);
		//获取中间顶点
		mean_pnt = (prj_l + prj_r) / 2;
		//因为方向垂直
		mean_dir = Eigen::Vector2f(-l_dir2d[1], l_dir2d[0]);
	}

	void HWPlane::ComputeVerticalLFunctionFromTwoPnts2D(Eigen::Vector2f& ls, Eigen::Vector2f& le,
		Eigen::Vector2f& mean_pnt, Eigen::Vector2f& mean_dir)
	{
		if ((le - ls).norm() < 1e-4)
		{
			return;
		}
		Eigen::Vector2f ldir = le - ls;
		ldir.norm();
		mean_dir[0] = -ldir[1];
		mean_dir[1] = ldir[0];

		mean_pnt = (ls + le) / 2;
	}

	void HWPlane::ComputeLdirFunctionFromEndPnts2D(Eigen::Vector2f& ls2d, Eigen::Vector2f& le2d,
		Eigen::Vector2f& ldir2d, Eigen::Vector2f& lpnt2d)
	{
		if ((le2d - ls2d).norm() < KMIN_FLOAT_THRESHOLD_REFINED)
		{
			ldir2d = Eigen::Vector2f(0.0, 0.0);
			lpnt2d = Eigen::Vector2f(0.0, 0.0);
			return;
		}
		ldir2d = le2d - ls2d;
		lpnt2d = ls2d;
	}

	bool HWPlane::ComputeLineSeg2DCrossLine2DPnts(Eigen::Vector2f& ls, Eigen::Vector2f& le,
		Eigen::Vector2f& mean_pnt, Eigen::Vector2f& mean_dir, Eigen::Vector2f& cross_pnt)
	{
		//
		if (std::abs((ls - le).norm()) < 1e-4)
			return false;
		
		//直线的线段设置的端点为
		Eigen::Vector2f pnt_s = mean_pnt;
		Eigen::Vector2f pnt_e = mean_pnt + mean_dir;
		
		//计算两个线段上的直线的交点
		Eigen::Vector2f tmp_cross_pnt = ComputePlaneTwoLinesCrossPnt(ls, le, pnt_s, pnt_e);
		//std::cout << "the tmp_cross_pnt: " << tmp_cross_pnt[0] << " " << tmp_cross_pnt[1] << std::endl;

		Eigen::Vector2f l_sc = tmp_cross_pnt - ls;
		Eigen::Vector2f l_ec = le - ls;
		float kabs = l_sc.norm() / l_ec.norm();
		if (l_sc.dot(l_ec) > 1e-6 && kabs > 1e-6 && kabs < 1.0f)
		{
			cross_pnt = tmp_cross_pnt;
			return true;
		}
		return false;
	}

	Eigen::Vector2f HWPlane::ComputePlaneTwoLinesCrossPnt(Eigen::Vector2f& ss, Eigen::Vector2f& se,
		Eigen::Vector2f& ts, Eigen::Vector2f& te)
	{
		//
		Eigen::Vector3f fs;
		Eigen::Vector3f fe;
		ComputePlaneFunctionFromTwoPnts(ss, se, fs);
		ComputePlaneFunctionFromTwoPnts(ts, te, fe);
		
		//std::cout << "the function s: " << fs[0] << " " << fs[1] << " " << fs[2] << std::endl;
		//std::cout << "the function e: " << fe[0] << " " << fe[1] << " " << fe[2] << std::endl;

		//两个线段，计算它们的交点
		Eigen::Matrix2f A;
		Eigen::Vector2f B;
		A(0, 0) = fs[0];
		A(0, 1) = fs[1];
		A(1, 0) = fe[0];
		A(1, 1) = fe[1];
		B[0] = -fs[2];
		B[1] = -fe[2];
		Eigen::Vector2f cross_p = A.inverse()*B;
		return cross_p;
	}

	bool HWPlane::ComputeLineSeg2DCrossLine2DPntsNew(Eigen::Vector2f& ls, Eigen::Vector2f& le,
		Eigen::Vector2f& t_pnt, Eigen::Vector2f& t_dir, Eigen::Vector2f& cross_pnt)
	{
		if (std::abs((ls - le).norm()) < 1e-4)
			return false;
		Eigen::Vector2f tmp_cross_pnt;
		//计算交点
		Eigen::Vector2f lsdir = le - ls;
		tmp_cross_pnt = ComputePlaneTwoLinesCrossPntNew(ls, lsdir, t_pnt, t_dir);

		Eigen::Vector2f l_sc = tmp_cross_pnt - ls;
		Eigen::Vector2f l_ec = le - ls;
		float kabs = l_sc.norm() / l_ec.norm();
		if (l_sc.dot(l_ec) > 1e-8 && kabs > -1e-8 && kabs < 1.0f)
		{
			cross_pnt = tmp_cross_pnt;
			return true;
		}
		//cross_pnt = tmp_cross_pnt;
		return false;
	}

	bool HWPlane::ComputeTwoLineSegsCrossPnt(Eigen::Vector2f& ls, Eigen::Vector2f& le,
		Eigen::Vector2f& ts, Eigen::Vector2f& te, Eigen::Vector2f& cross_pnt)
	{
		//
		Eigen::Vector2f s_dir = le - ls;
		Eigen::Vector2f t_dir = te - ts;
		if (s_dir.norm() < 1e-6 || t_dir.norm() < 1e-6)
			return false;
		s_dir.normalize();
		t_dir.normalize();
		//先判断它们是否平行
		if (std::abs(s_dir[0] - t_dir[0]) < 1e-4
			&& std::abs(s_dir[1] - t_dir[1]) < 1e-4)
		{
			std::cerr << "s_dir: " << s_dir[0] << " " << s_dir[1] <<" ";
			std::cerr << "t_dir: " << t_dir[0] << " " << t_dir[1] << " " << std::endl;
			std::cerr << "two lines parallel..." << std::endl;
			return false;
		}
		Eigen::Vector2f tmp_cross_pnt = ComputePlaneTwoLinesCrossPntNew(ls, s_dir, ts, t_dir);
		Eigen::Vector2f l_sc = tmp_cross_pnt - ls;
		Eigen::Vector2f l_ec = le - ls;
		float kabs = l_sc.norm() / l_ec.norm();
		Eigen::Vector2f t_sc = tmp_cross_pnt - ts;
		Eigen::Vector2f t_ec = te - ts;
		float kabt = t_sc.norm() / t_ec.norm();
		if (l_sc.dot(l_ec) > 1e-6 && kabs > -1e-6 && kabs < 1.0f
			&& t_sc.dot(t_ec) > 1e-6 && kabt > -1e-6 && kabt < 1.0f)
		{
			cross_pnt = tmp_cross_pnt;
			return true;
		}
		return false;
	}

	bool HWPlane::ComputeTwoLinesCrosspnt(Eigen::Vector2f& ls, Eigen::Vector2f& le,
		Eigen::Vector2f& ts, Eigen::Vector2f& te, Eigen::Vector2f& cross_pnt)
	{
		//
		Eigen::Vector2f s_dir = le - ls;
		Eigen::Vector2f t_dir = te - ts;
		if (s_dir.norm() < 1e-6 || t_dir.norm() < 1e-6)
		{
			std::cerr << "lower dir..." << std::endl;
			return false;
		}
		s_dir.normalize();
		t_dir.normalize();
		//先判断它们是否平行
		if (std::abs(s_dir[0] - t_dir[0]) < 1e-4
			&& std::abs(s_dir[1] - t_dir[1]) < 1e-4)
		{
			std::cerr << "ComputeTwoLinesCrosspnt: two lines parallel..." << std::endl;
			return false;
		}

		Eigen::Vector2f tmp_cross_pnt = ComputePlaneTwoLinesCrossPntNew(ls, s_dir, ts, t_dir);
		cross_pnt = tmp_cross_pnt;
		return true;
	}

	//sp+k2*sdir=tp+k1*tdir
	Eigen::Vector2f HWPlane::ComputePlaneTwoLinesCrossPntNew(Eigen::Vector2f& sp, Eigen::Vector2f& sdir,
		Eigen::Vector2f& tp, Eigen::Vector2f& tdir)
	{
		Eigen::Matrix2f A;
		A << tdir[0], -sdir[0], tdir[1], -sdir[1];
		Eigen::Vector2f B;
		B[0] = sp[0] - tp[0];
		B[1] = sp[1] - tp[1];
		Eigen::Vector2f kvalue = A.fullPivHouseholderQr().solve(B);
		//std::cout << "the kvalue: " << kvalue[0] << " " << kvalue[1] << std::endl;
		Eigen::Vector2f crosspnt = sp + kvalue[1] * sdir;
		return crosspnt;
	}

	//f:Ax+By+C = 0->A = y2-y1; B=x1-x2; C=x2*y1-x1*y2
	bool HWPlane::ComputePlaneFunctionFromTwoPnts(Eigen::Vector2f& l_s, Eigen::Vector2f& l_e, Eigen::Vector3f& f2d)
	{
		//
		if (std::abs((l_e - l_s).norm()) < KMIN_FLOAT_THRESHOLD_REFINED)
			return false;
		f2d[0] = l_e[1] - l_s[1];
		f2d[1] = l_s[0] - l_e[0];
		f2d[2] = l_e[0] * l_s[1] - l_s[0] * l_e[1];
		return true;
	}

	void HWPlane::ComputeTwoPointsIdxFarestCorrepondingLine(Eigen::Vector3f& L_dir, Eigen::Vector3f& L_point, Eigen::Vector2i& pnts_idxs)
	{
		float min_k = std::numeric_limits<float>::max();
		float max_k = std::numeric_limits<float>::min();
		int min_k_idx = -1;
		int max_k_idx = -1;

		//计算离线较远的两个端点
		for (int i = 0; i < corner_pnts_3d_.size(); ++i)
		{
			Eigen::Vector3f pnt3d = Eigen::Vector3f(corner_pnts_3d_[i].x, 
				corner_pnts_3d_[i].y, corner_pnts_3d_[i].z);
			float k_dist = Pnt3DDistToLine3D(pnt3d, L_dir, L_point);
			if (k_dist > max_k)
			{
				max_k_idx = i;
				max_k = k_dist;
			}
			if (k_dist < min_k)
			{
				min_k_idx = i;
				min_k = k_dist;
			}
		}

		pnts_idxs[0] = min_k_idx;
		pnts_idxs[1] = max_k_idx;
	}

	int HWPlane::IsSelfIntersection(int idx, float2 & p_curr)
	{
		int intersection_cnt = 0;
		int idx_last = (idx + corner_pnts_.size() - 1) % corner_pnts_.size();
		int idx_next = (idx + 1) % corner_pnts_.size();
		float2 p_last = corner_pnts_[idx_last];
		float2 p_next = corner_pnts_[idx_next];
		float2 to_last = p_last - p_curr;
		float2 to_next = p_next - p_curr;
		if (abs(dot(to_last, to_last)) < 0.001 ||
			abs(dot(to_next, to_next)) < 0.001
			) {
			intersection_cnt++;
		}

		Eigen::Vector2f edge_last = FittingLine(p_curr, p_last);
		Eigen::Vector2f edge_next = FittingLine(p_curr, p_next);
		for (int i = idx + 2; i < idx - 1 + corner_pnts_.size(); i++) {
			float2 p1 = corner_pnts_[i%corner_pnts_.size()];
			float2 p2 = corner_pnts_[(i + 1) % corner_pnts_.size()];
			Eigen::Vector2f edge = FittingLine(p1, p2);
			float2 p = GetIntersectionPoint(edge_next, edge);
			if (dot(p - p_curr, p - p_next) < 0 && dot(p - p1, p - p2) < 0) {
				intersection_cnt++;
				break;
			}
		}
		for (int i = idx + 1; i < idx + corner_pnts_.size() - 2; i++) {
			float2 p1 = corner_pnts_[i%corner_pnts_.size()];
			float2 p2 = corner_pnts_[(i + 1) % corner_pnts_.size()];
			Eigen::Vector2f edge = FittingLine(p1, p2);
			float2 p = GetIntersectionPoint(edge_last, edge);
			if (dot(p - p_curr, p - p_last) < 0 && dot(p - p1, p - p2) < 0) {
				intersection_cnt++;
				break;
			}
		}
		return intersection_cnt;
	}

	int HWPlane::IsSelfIntersection(std::vector<float2>& corner_pnts, int idx, float2& p_curr)
	{
		int intersection_cnt = 0;
		int idx_last = (idx + corner_pnts.size() - 1) % corner_pnts.size();
		int idx_next = (idx + 1) % corner_pnts.size();
		float2 p_last = corner_pnts[idx_last];
		float2 p_next = corner_pnts[idx_next];
		Eigen::Vector2f edge_last = FittingLine(p_curr, p_last);
		Eigen::Vector2f edge_next = FittingLine(p_curr, p_next);
		for (int i = idx + 2; i < idx - 1 + corner_pnts.size(); i++) {
			float2 p1 = corner_pnts[i % corner_pnts.size()];
			float2 p2 = corner_pnts[(i + 1) % corner_pnts.size()];
			Eigen::Vector2f edge = FittingLine(p1, p2);
			float2 p = GetIntersectionPoint(edge_next, edge);
			if (dot(p - p_curr, p - p_next) < 0 && dot(p - p1, p - p2) < 0) {
				intersection_cnt++;
				break;
			}
		}
		for (int i = idx + 1; i < idx + corner_pnts.size() - 2; i++) {
			float2 p1 = corner_pnts[i % corner_pnts.size()];
			float2 p2 = corner_pnts[(i + 1) % corner_pnts.size()];
			Eigen::Vector2f edge = FittingLine(p1, p2);
			float2 p = GetIntersectionPoint(edge_last, edge);
			if (dot(p - p_curr, p - p_last) < 0 && dot(p - p1, p - p2) < 0) {
				intersection_cnt++;
				break;
			}
		}
		return intersection_cnt;
	}

	int HWPlane::IsSelfIntersections(std::vector<float2>& corner_pnts)
	{
		int intersection_self_cnt = 0;
		if (corner_pnts.size() <= 3)
		{
			return intersection_self_cnt;
		}
		else
		{
			for (int i = 0; i < corner_pnts.size(); ++i)
			{
				float2 cur_pnt = corner_pnts[i];
				int self_cnt = IsSelfIntersection(corner_pnts, i, cur_pnt);
				intersection_self_cnt += self_cnt;
			}
		}
		return intersection_self_cnt;
	}

	int HWPlane::IsSelfIntersectionInPolygon(std::vector<Eigen::Vector2f>& in_polygon, int idx, Eigen::Vector2f& p_curr)
	{
		int intersection_cnt = 0;
		int idx_last = (idx + in_polygon.size() - 1) % in_polygon.size();
		int idx_next = (idx + 1) % in_polygon.size();
		Eigen::Vector2f p_last = in_polygon[idx_last];
		Eigen::Vector2f p_next = in_polygon[idx_next];

		Eigen::Vector2f edge_last = FittingLineEigen(p_curr, p_last);
		Eigen::Vector2f edge_next = FittingLineEigen(p_curr, p_next);

		for (int i = idx + 2; i < idx - 1 + in_polygon.size(); i++) {
			Eigen::Vector2f p1 = in_polygon[i%in_polygon.size()];
			Eigen::Vector2f p2 = in_polygon[(i + 1) % in_polygon.size()];
			Eigen::Vector2f edge = FittingLineEigen(p1, p2);
			float2 p = GetIntersectionPoint(edge_next, edge);
			Eigen::Vector2f pe = Eigen::Vector2f(p.x, p.y);

			if ((pe-p_curr).dot(pe-p_next) < 0 && (pe - p1).dot(pe - p2) < 0) {
				intersection_cnt++;
				break;
			}
		}

		for (int i = idx + 1; i < idx + in_polygon.size() - 2; i++) {
			Eigen::Vector2f p1 = in_polygon[i%in_polygon.size()];
			Eigen::Vector2f p2 = in_polygon[(i + 1) % in_polygon.size()];
			Eigen::Vector2f edge = FittingLineEigen(p1, p2);
			float2 p = GetIntersectionPoint(edge_last, edge);
			Eigen::Vector2f pe = Eigen::Vector2f(p.x, p.y);

			if ( (pe - p_curr).dot(pe - p_last) < 0 && (pe - p1).dot(pe - p2) < 0) {
				intersection_cnt++;
				break;
			}
		}
		return intersection_cnt;
	}

	int HWPlane::IsSelfIntersectionInPolygonNew(std::vector<Eigen::Vector2f>& in_polygon, int idx, Eigen::Vector2f& p_curr)
	{
		int intersection_cnt = 0;
		int idx_last = (idx + in_polygon.size() - 1) % in_polygon.size();
		int idx_next = (idx + 1) % in_polygon.size();

		//判断这两个线段和其它另外两个线段是否有交点
		Eigen::Vector2f last_ls = in_polygon[idx_last];
		Eigen::Vector2f last_le = in_polygon[idx];
		Eigen::Vector2f next_ls = in_polygon[idx];
		Eigen::Vector2f next_le = in_polygon[idx_next];
		Eigen::Vector2f last_dir = last_le - last_ls;
		Eigen::Vector2f next_dir = next_le - next_ls;
		for (int i = idx + 2; i < idx - 1 + in_polygon.size(); i++)
		{
			Eigen::Vector2f p1 = in_polygon[i%in_polygon.size()];
			Eigen::Vector2f p2 = in_polygon[(i + 1) % in_polygon.size()];
			Eigen::Vector2f pdir = p2 - p1;
			Eigen::Vector2f tmp_crosspnt0;
			Eigen::Vector2f tmp_crosspnt1;
			if (ComputeLineSeg2DCrossLine2DPntsNew(next_ls, next_le, p1, pdir, tmp_crosspnt0)
				&& ComputeLineSeg2DCrossLine2DPntsNew(p1, p2, next_ls, next_dir, tmp_crosspnt1))
			{
				intersection_cnt++;
				break;
			}
		}
		for (int i = idx + 1; i < idx + in_polygon.size() - 2; i++) 
		{
			Eigen::Vector2f p1 = in_polygon[i%in_polygon.size()];
			Eigen::Vector2f p2 = in_polygon[(i + 1) % in_polygon.size()];
			Eigen::Vector2f pdir = p2 - p1;
			Eigen::Vector2f tmp_crosspnt0;
			Eigen::Vector2f tmp_crosspnt1;
			if (ComputeLineSeg2DCrossLine2DPntsNew(last_ls, last_le, p1, pdir, tmp_crosspnt0)
				&& ComputeLineSeg2DCrossLine2DPntsNew(p1, p2, last_ls, last_dir, tmp_crosspnt1))
			{
				intersection_cnt++;
				break;
			}
		}
		return intersection_cnt;
	}

	int HWPlane::IsSelfIntersectionsEigenNew(std::vector<Eigen::Vector2f>& corner_pnts)
	{
		int intersection_self_cnt = 0;
		if (corner_pnts.size() <= 3)
		{
			return intersection_self_cnt;
		}
		else
		{
			for (int i = 0; i < corner_pnts.size(); ++i)
			{
				Eigen::Vector2f cur_pnt = corner_pnts[i];
				int self_cnt = IsSelfIntersectionInPolygonNew(corner_pnts, i, cur_pnt);
				intersection_self_cnt += self_cnt;
			}
		}
		return intersection_self_cnt;
	}

	//
	float HWPlane::Pnt3DDistToLine3D(Eigen::Vector3f& pnt, Eigen::Vector3f& L_dir, Eigen::Vector3f& L_point)
	{
		Eigen::Vector3f dist2lp = L_point - pnt;
		float dist = (dist2lp.cross(L_dir)).norm() / L_dir.norm();
		return dist;
		/*if (L_dir.norm() <= 1e-6)
			return std::numeric_limits<float>::max();
		return std::abs(L_dir.dot(dist2lp) / L_dir.norm());*/
	}

	float HWPlane::Pnt2DDistToLine2D(Eigen::Vector2f& pnt2d, Eigen::Vector2f& L_point2d, Eigen::Vector2f& L_dir2d)
	{
		if (L_dir2d.norm() < 1e-6)
		{
			return (pnt2d - L_point2d).norm();
			//return std::numeric_limits<float>::max();
		}
		//计算投影点
		Eigen::Vector2f proj_pnt;
		Pnt2dProjLine2D(pnt2d, L_point2d, L_dir2d, proj_pnt);
		//点到直线的距离
		float dist = (pnt2d - proj_pnt).norm();
		return dist;
	}

	float HWPlane::Pnt2DDistToLine2DNew1(Eigen::Vector2f& pnt2d, Eigen::Vector2f& lpnt2d, Eigen::Vector2f& ldir2d)
	{
		//需要对ldir2d进行处理
		if (ldir2d.norm() < 1e-6)
		{
			return (pnt2d - lpnt2d).norm();
		}
		Eigen::Vector2f ldir2dnew = ldir2d.normalized();
		Eigen::Vector2f ls = lpnt2d;
		Eigen::Vector2f le = lpnt2d + ldir2dnew;
		float a = std::abs((pnt2d[0] - ls[0])*(le[1] - ls[1]) + (pnt2d[1] - ls[1])*(ls[0] - le[0]));
		float dist = a / (le - ls).norm();
		return dist;
	}

	float HWPlane::Pnt2DDistToLine2DLSLE(Eigen::Vector2f& pnt2d, Eigen::Vector2f& ls, Eigen::Vector2f& le)
	{
		if ((le - ls).norm() < 1e-6)
		{
			return (pnt2d - ls).norm();
		}
		float a = std::abs((pnt2d[0] - ls[0])*(le[1] - ls[1]) + (pnt2d[1] - ls[1])*(ls[0] - le[0]));
		float dist = a / (le - ls).norm();
		return dist;
	}

	float HWPlane::Pnt2DToLineSegment2D(Eigen::Vector2f& pnt2d, Eigen::Vector2f& ls, Eigen::Vector2f& le)
	{
		if ((le - ls).norm() < 1e-6)
		{
			return (pnt2d - ls).norm();
		}
		float d = Pnt2DDistToLine2DLSLE(pnt2d, ls, le);
		float s2e = (le - ls).norm();
		float s2p = (pnt2d - ls).norm();
		float e2p = (pnt2d - le).norm();
		float max2p = std::max(s2p, e2p);
		if (std::sqrtf(max2p*max2p - d*d) > s2e)
		{
			return std::min(s2p, e2p);
		}
		else
		{
			return d;
		}
	}

	float HWPlane::Pnt2DDistToLine2DNew(Eigen::Vector2f& pnt2d, Eigen::Vector2f& ldir2d, Eigen::Vector2f& lpnt2d)
	{
		//需要对ldir2d进行处理
		if (ldir2d.norm() < 1e-6)
		{
			return (pnt2d - lpnt2d).norm();
		}
		Eigen::Vector2f ldir2dnew = ldir2d.normalized();
		Eigen::Vector2f ls = lpnt2d;
		Eigen::Vector2f le = lpnt2d + ldir2dnew;
		float a = (ls - pnt2d).norm();
		float b = (pnt2d - le).norm();
		float c = (le - ls).norm();
		float p = (a + b + c) / 2;
		float area = std::sqrtf(std::abs(p*(p - a)*(p - b)*(p - c)));
		float dist = (2 * area) / c;
		return dist;
	}

	float2 HWPlane::ProjToLine3D(int idx, Eigen::Vector3f & L_dir, Eigen::Vector3f & L_point)
	{	
		float2 p = corner_pnts_[idx];
		Eigen::Vector4f temp = world_to_plane_.topLeftCorner(3, 3)*L_dir;
		//只算相机坐标系下的x坐标和y坐标，不算z坐标
		float2 L_dir_2D = make_float2(temp(0), temp(1));
		temp = world_to_plane_*Eigen::Vector4f(L_point(0), L_point(1), L_point(2), 1.0f);
		float2 L_point_2D = make_float2(temp(0), temp(1));
		float2 p2L_point = p - L_point_2D;
		float2 p_proj = L_point_2D + dot(p2L_point, L_dir_2D) / dot(L_dir_2D, L_dir_2D)*L_dir_2D;
		return p_proj;
	}

	float2 HWPlane::ProjToPlane3D(int idx, float4 & plane_coeff)
	{
		float3 pnt = corner_pnts_3d_[idx];
		float dist = (plane_coeff.x * pnt.x + plane_coeff.y * pnt.y + plane_coeff.z * pnt.z + plane_coeff.w) /
			std::sqrtf(plane_coeff.x*plane_coeff.x + plane_coeff.y*plane_coeff.y + plane_coeff.z*plane_coeff.z);

		/*std::cout << "the pnt is: " << pnt.x << " " << pnt.y << " " << pnt.z << std::endl;
		std::cout << "the plane function: " << plane_coeff.x << " " << plane_coeff.y << " " << plane_coeff.z <<" " << d_ << std::endl;
		std::cout << "dist is: " << dist << std::endl;*/

		float k = -1 * dist / (std::sqrtf(plane_coeff.x*plane_coeff.x + plane_coeff.y*plane_coeff.y + plane_coeff.z*plane_coeff.z));
		//std::cout << "k: " << k << std::endl;
		Eigen::Vector4f p_proj;
		p_proj[0] = pnt.x + k * plane_coeff.x;
		p_proj[1] = pnt.y + k * plane_coeff.y;
		p_proj[2] = pnt.z + k * plane_coeff.z;
		p_proj[3] = 1;
		Eigen::Vector4f p_proj_2d = world_to_plane_*p_proj;
		return make_float2(p_proj_2d[0], p_proj_2d[1]);
	}

	float2 HWPlane::ProjToPlane3D(float3 & pnt)
	{
		float dist = (coeff_.x * pnt.x + coeff_.y * pnt.y + coeff_.z * pnt.z + coeff_.w) /
			std::sqrtf(coeff_.x*coeff_.x + coeff_.y*coeff_.y + coeff_.z*coeff_.z);

		/*std::cout << "the pnt is: " << pnt.x << " " << pnt.y << " " << pnt.z << std::endl;
		std::cout << "the plane function: " << coeff_.x << " " << coeff_.y << " " << coeff_.z <<" " << d_ << std::endl;
		std::cout << "dist is: " << dist << std::endl;*/

		float k = -1 * dist / (std::sqrtf(coeff_.x*coeff_.x + coeff_.y*coeff_.y + coeff_.z*coeff_.z));
		//std::cout << "k: " << k << std::endl;
		Eigen::Vector4f p_proj;
		p_proj[0] = pnt.x + k * coeff_.x;
		p_proj[1] = pnt.y + k * coeff_.y;
		p_proj[2] = pnt.z + k * coeff_.z;
		p_proj[3] = 1;
		Eigen::Vector4f p_proj_2d = world_to_plane_*p_proj;
		return make_float2(p_proj_2d[0], p_proj_2d[1]);
	}

	float3 HWPlane::ProjToPlane(float3 & pnt)
	{
		float dist = (coeff_.x * pnt.x + coeff_.y * pnt.y + coeff_.z * pnt.z + coeff_.w) /
			std::sqrtf(coeff_.x*coeff_.x + coeff_.y*coeff_.y + coeff_.z*coeff_.z);

		/*std::cout << "the pnt is: " << pnt.x << " " << pnt.y << " " << pnt.z << std::endl;
		std::cout << "the plane function: " << coeff_.x << " " << coeff_.y << " " << coeff_.z <<" " << d_ << std::endl;
		std::cout << "dist is: " << dist << std::endl;*/

		float k = -1 * dist / (std::sqrtf(coeff_.x*coeff_.x + coeff_.y*coeff_.y + coeff_.z*coeff_.z));
		//std::cout << "k: " << k << std::endl;
		float3 result;
		result.x = pnt.x + k * coeff_.x;
		result.y = pnt.y + k * coeff_.y;
		result.z = pnt.z + k * coeff_.z;
		return result;
	}

	Eigen::Vector3f HWPlane::Pnt3dProjToPlane3D(const Eigen::Vector3f& pnt)
	{
		float3 tmp_pos;
		tmp_pos.x = pnt[0];
		tmp_pos.y = pnt[1];
		tmp_pos.z = pnt[2];
		float3 tmp_proj_pos = ProjToPlane(tmp_pos);
		Eigen::Vector3f proj_pos 
			= Eigen::Vector3f(tmp_proj_pos.x, tmp_proj_pos.y, tmp_proj_pos.z);
		return proj_pos;
	}

	void HWPlane::Pnt2dProjLine2D(Eigen::Vector2f& pnt2d, Eigen::Vector2f& ls, Eigen::Vector2f& ldir,
		Eigen::Vector2f& proj_pnt2d)
	{
		Eigen::Vector2f le = ls + ldir;
		Eigen::Vector2f lse = le - ls;	//P2-P1
		Eigen::Vector2f lsp = pnt2d - ls;	//P3-P1
		if (std::abs(lse.norm()) < 1e-6)
		{
			std::cout << "project to pnt because of line(pnt)" << std::endl;
			proj_pnt2d = ls;
		}
		else
		{
			float k = lsp.dot(lse) / (lse.norm()*lse.norm());
			proj_pnt2d = ls + k * lse;
		}
	}

	int HWPlane::Pnt2dProjLineSeg2D(Eigen::Vector2f& pnt2d, Eigen::Vector2f& ls, Eigen::Vector2f& le,
		Eigen::Vector2f& proj_pnt2d)
	{
		Eigen::Vector2f lse = le - ls;	//P2-P1
		Eigen::Vector2f lsp = pnt2d - ls;	//P3-P1

		if (std::abs(lse.norm()) < 1e-4)
		{
			std::cout << "project to pnt because of line(pnt)" << std::endl;
			proj_pnt2d = ls;
			return -1;
		}
		else
		{
			float k = lsp.dot(lse) / (lse.norm()*lse.norm());
			std::cout << "Pnt2dProjLineSeg2D k: " << k << std::endl;
			proj_pnt2d = ls + k * lse;
			if (k < 1.0 && k > 0.0)
			{
				return 0;
			}
			else if (k > 1.0)
			{
				return 1;
			}
			else
			{
				return -1;
			}
			
		}
	}

	void HWPlane::Pnt2d2Pnt3D(Eigen::Vector2f& pnt2d, Eigen::Vector3f& pnt3d)
	{
		//
		Eigen::Vector4f tmp = plane_to_world_*Eigen::Vector4f(pnt2d[0], pnt2d[1], 0, 1);
		pnt3d[0] = tmp[0];
		pnt3d[1] = tmp[1];
		pnt3d[2] = tmp[2];
	}

	void HWPlane::Pnt3d2Pnt2D(Eigen::Vector3f& pnt3d, Eigen::Vector2f& pnt2d)
	{
		Eigen::Vector4f tmp = plane_to_world_.inverse()*Eigen::Vector4f(pnt3d(0), pnt3d(1), pnt3d(2), 1.0f);
		pnt2d[0] = tmp[0];
		pnt2d[1] = tmp[1];
	}

	bool HWPlane::IsAreaIncreasing(int idx, Eigen::Vector3f & L_dir, Eigen::Vector3f & L_point)
	{
		float2 p_proj = ProjToLine3D(idx, L_dir, L_point);
		Eigen::Vector4f p_proj_3d = plane_to_world_*Eigen::Vector4f(p_proj.x, p_proj.y, 0, 1);
		std::cout << "p_proj_3d: " << p_proj_3d.transpose() << "\n";
		float2 p = corner_pnts_[idx];
		Eigen::Vector4f p_3d = plane_to_world_*Eigen::Vector4f(p.x, p.y, 0, 1);
		std::cout << "p_3d: " << p_3d.transpose() << "\n";
		float2 p_last = corner_pnts_[(idx + corner_pnts_.size() - 1) % corner_pnts_.size()];
		float2 p_next = corner_pnts_[(idx + 1) % corner_pnts_.size()];

		Eigen::Vector2f p0 = Eigen::Vector2f(p.x, p.y);
		Eigen::Vector2f p1 = Eigen::Vector2f(p_next.x, p_next.y);
		Eigen::Vector2f p2 = Eigen::Vector2f(p_last.x, p_last.y);
		Eigen::Vector2f e1 = p0 - p1;
		Eigen::Vector2f e2 = p2 - p1;
		Eigen::Vector2f e3 = e1.dot(e2) / e2.norm() / e2.norm()*e2;
		Eigen::Vector2f p_vertical_line = e3 - e1;
		float2 move_dir = p_proj - p;

		Eigen::Vector2f dir = Eigen::Vector2f(move_dir.x, move_dir.y);
		std::cout << "p_vertical_line: " << p_vertical_line.transpose() << "\n";
		std::cout << "dir: " << dir.transpose() << "\n";
		std::cout << "dot: " << (dir.dot(p_vertical_line)<=0) << "\n";
		std::cout << "IsConcaveVertex(idx)==1: " << (IsConcaveVertex(idx) == 0) << "\n";
		if (dir.dot(p_vertical_line) <= 0^(IsConcaveVertex(idx)==1))
			return true;
		return false;
	}

	bool HWPlane::IsSelectedAreaIncreasing(std::vector<Eigen::Vector2f>& polygonpnts, int idx, Eigen::Vector2f& projpnt)
	{
		Eigen::Vector2f p = polygonpnts[idx];
		Eigen::Vector2f p_last = polygonpnts[(idx + polygonpnts.size() - 1) % polygonpnts.size()];
		Eigen::Vector2f p_next = polygonpnts[(idx + 1) % polygonpnts.size()];

		Eigen::Vector2f p0 = p;
		Eigen::Vector2f p1 = p_next;
		Eigen::Vector2f p2 = p_last;
		Eigen::Vector2f e1 = p0 - p1;
		Eigen::Vector2f e2 = p2 - p1;
		Eigen::Vector2f e3 = e1.dot(e2) / e2.norm() / e2.norm()*e2;
		Eigen::Vector2f p_vertical_line = e3 - e1;
		Eigen::Vector2f move_dir = projpnt - p;

		//Eigen::Vector2f dir = Eigen::Vector2f(move_dir.x, move_dir.y);
		std::cout << "p_vertical_line: " << p_vertical_line.transpose() << "\n";
		std::cout << "move_dir: " << move_dir.transpose() << "\n";
		std::cout << "dot: " << (move_dir.dot(p_vertical_line) <= 0) << "\n";
		std::cout << "IsConcaveVertex(idx)==1: " << (IsConcaveVertexNew(polygonpnts, idx) == 0) << "\n";
		if (move_dir.dot(p_vertical_line) <= 0 ^ (IsConcaveVertexNew(polygonpnts, idx) == 1))
			return true;
		return false;
	}

	float HWPlane::ComputePolygon2DAreaFromPnts3d()
	{
		//3d点云转化为平面坐标系下的点云
		std::vector<Eigen::Vector2f> corner_pnts2d;

		for (int i = 0; i < corner_pnts_3d_.size(); ++i)
		{
			Eigen::Vector4f tmp_pnt3d = Eigen::Vector4f(corner_pnts_3d_[i].x, corner_pnts_3d_[i].y,
				corner_pnts_3d_[i].z, 1.0);
			Eigen::Vector4f tmp_pnt = world_to_plane_*tmp_pnt3d;
			Eigen::Vector2f tmp_pnt2d = Eigen::Vector2f(tmp_pnt[0], tmp_pnt[1]);
			corner_pnts2d.emplace_back(tmp_pnt2d);
		}
		//计算polygon 2d的面积大小
		float area_tmp = 0.0;
		int icount = corner_pnts2d.size();
		for (int i = 0; i < corner_pnts2d.size(); ++i)
		{
			//
			area_tmp += (corner_pnts2d[i][0] * corner_pnts2d[(i + 1) % icount][1]
				- corner_pnts2d[(i + 1) % icount][0] * corner_pnts2d[i][1]);
		}
		return std::abs(0.5*area_tmp);
	}

	void HWPlane::ComputePolygon2DAreaFromCornerPnts3d()
	{
		//3d点云转化为平面坐标系下的点云
		std::vector<Eigen::Vector2f> corner_pnts2d;

		for (int i = 0; i < corner_pnts_3d_.size(); ++i)
		{
			Eigen::Vector4f tmp_pnt3d = Eigen::Vector4f(corner_pnts_3d_[i].x, corner_pnts_3d_[i].y,
				corner_pnts_3d_[i].z, 1.0);
			Eigen::Vector4f tmp_pnt = world_to_plane_*tmp_pnt3d;
			Eigen::Vector2f tmp_pnt2d = Eigen::Vector2f(tmp_pnt[0], tmp_pnt[1]);
			corner_pnts2d.emplace_back(tmp_pnt2d);
		}
		//计算polygon 2d的面积大小
		float area_tmp = 0.0;
		int icount = corner_pnts2d.size();
		for (int i = 0; i < corner_pnts2d.size(); ++i)
		{
			//
			area_tmp += (corner_pnts2d[i][0] * corner_pnts2d[(i + 1) % icount][1]
				- corner_pnts2d[(i + 1) % icount][0] * corner_pnts2d[i][1]);
		}
		corners_area_2d_ = 0.5 * area_tmp;
	}

	float HWPlane::GetPolygonCorner2DArea()
	{
		return corners_area_2d_;
	}

	bool HWPlane::GenerateWorldCoordToPlaneCoordMatrix()
	{
		std::cout << "start generate 2d plane!" << std::endl;

		//
		if (true)
		{
			//Eigen::Matrix3f covariance_matrix;
			//if (!corner_pnts_3d_.empty()) covariance_matrix = ComputeCovarianceMatrix(corner_pnts_3d_);
			//else covariance_matrix = ComputeCovarianceMatrix(pnts_pos_origin_);
			Eigen::Matrix3f covariance_matrix = ComputeCovarianceMatrix(pnts_pos_origin_);
			Eigen::Matrix3f eigen_values = Eigen::Matrix3f::Zero(3, 3);
			Eigen::Matrix3f eigen_vector = Eigen::Matrix3f::Zero(3, 3);
			ComputeMatrixEigens(covariance_matrix, eigen_values, eigen_vector);

			//
			//std::cout << "the eigen matrix value is: \n" << eigen_values << std::endl;
			//std::cout << "the eigen matrix vector is: \n" << eigen_vector << std::endl;

			int min_eigen_idx = -1;
			int max_eigen_idx = -1;
			float min_value = FLT_MAX;
			float max_value = FLT_MIN;

			for (int i = 0; i < 3; ++i)
			{
				if (min_value > eigen_values(i, i))
				{
					min_value = eigen_values(i, i);
					min_eigen_idx = i;
				}

				if (max_value < eigen_values(i, i))
				{
					max_value = eigen_values(i, i);
					max_eigen_idx = i;
				}
			}

			//std::cout << "the max eigen idx is: " << max_eigen_idx << std::endl;
			//std::cout << "the min eigen idx is: " << min_eigen_idx << std::endl;
			average_pnts_normal_ = ComputeAverageVertexNormal();
			Eigen::Vector3f PCA_normal = eigen_vector.col(min_eigen_idx);
			Eigen::Matrix3f r_matrix;
			r_matrix.col(0) = eigen_vector.col(max_eigen_idx);
			//std::cout << "PCA_normal: " << PCA_normal.transpose() << "\n";
			//std::cout << "average_pnts_normal_: " << average_pnts_normal_.transpose() << "\n";
			if (PCA_normal.dot(average_pnts_normal_) > 0)
			{
				r_matrix.col(2) = PCA_normal;
			}
			else
			{
				r_matrix.col(2) = -PCA_normal;
			}
			plane_normal_ = r_matrix.col(2);

			//ax + by + cz + d = 0
			Eigen::Vector3f coeff;
			//float3 coeff = make_float3(0.0, 0.0, 0.0);
			coeff = r_matrix.col(2);
			
			float3 average_pos = ComputeAverageVertexPos();
			//std::cout << "average_pos = " << average_pos.x << ", " << average_pos.y << ", " << average_pos.z << std::endl;
			float d = -(coeff.x() * average_pos.x + coeff.y() * average_pos.y + coeff.z() * average_pos.z);

			coeff_.x = coeff[0];
			coeff_.y = coeff[1];
			coeff_.z = coeff[2];
			coeff_.w = d;
			//d_ = d;

			//获取矩阵
			

			/*int mean_eigen_idx = -1;
			for (int j = 0; j < 3; j++)
			{
				if (j != min_eigen_idx || j != min_eigen_idx)
				{
					mean_eigen_idx = j;
				}
			}
			if (mean_eigen_idx != -1)
			{
				r_matrix.col(1) = eigen_vector.col(mean_eigen_idx);
			}*/

			//计算另一个向量
			r_matrix.col(1) = r_matrix.col(0).cross(r_matrix.col(2));

			Eigen::Vector3f t_matrix(average_pos.x, average_pos.y, average_pos.z);

			plane_to_world_.topLeftCorner(3, 3) = r_matrix;
			
			plane_to_world_(0, 3) = t_matrix(0);
			plane_to_world_(1, 3) = t_matrix(1);
			plane_to_world_(2, 3) = t_matrix(2);
			//world_to_plane_.topRightCorner(3, 1) = t_matrix;
			plane_to_world_(3, 3) = 1;
			plane_to_world_.row(3) = Eigen::RowVector4f(0, 0, 0, 1);
			//system("pause");

			//Eigen::Matrix4f world_to_plane_inverse = world_to_plane_.inverse();
			world_to_plane_ = plane_to_world_.inverse();

			//std::cout << "plane_to_world =" << std::endl;
			//std::cout << plane_to_world_ << std::endl;
			//std::cout << "world_to_plane =" << std::endl;
			//std::cout << world_to_plane_ << std::endl;

			/*std::cout << "plane_to_world_:\n" << plane_to_world_ << "\n";
			std::cout << "world_to_plane_:\n " << world_to_plane_ << "\n";*/
			//world_to_plane_inverse(0, 3) = t_matrix(0);
			//world_to_plane_inverse(1, 3) = t_matrix(1);
			//world_to_plane_inverse(2, 3) = t_matrix(2);
			//world_to_plane_inverse(3, 3) = 1;

#if PAINT_CAM_POS
			//画相机位置
			//xyz_step
			std::ofstream out_points("G:\\xht\\huawei\\2019-04-28_14.54.52\\cam.obj");
			float xyz_step = 0.01;
			Eigen::Vector3f camera_position = plane_to_world_.topRightCorner(3, 1);
			Eigen::Matrix3f camera_rotation = plane_to_world_.topLeftCorner(3, 3);
			std::cout << "the rotation matrix is: " << camera_rotation << std::endl;
			std::cout << "the position matrix is: " << camera_position << std::endl;

			for (int i = 0; i < 100; i++)
			{
				float3 tempVersX;
				tempVersX.x = camera_position(0) + i*xyz_step*camera_rotation(0, 0);
				tempVersX.y = camera_position(1) + i*xyz_step*camera_rotation(0, 1);
				tempVersX.z = camera_position(2) + i*xyz_step*camera_rotation(0, 2);

				//输出x朝向的顶点
				out_points << "v" << " " << tempVersX.x << " " << tempVersX.y << " " << tempVersX.z << " " << 255 << " " << 0 << " " << 0 << std::endl;

				float3 tempVersY;
				tempVersY.x = camera_position(0) + i*xyz_step * 1.5 *camera_rotation(1, 0);
				tempVersY.y = camera_position(1) + i*xyz_step * 1.5 *camera_rotation(1, 1);
				tempVersY.z = camera_position(2) + i*xyz_step * 1.5 *camera_rotation(1, 2);
				out_points << "v" << " " << tempVersY.x << " " << tempVersY.y << " " << tempVersY.z << " " << 0 << " " << 255 << " " << 0 << std::endl;

				float3 tempVersZ;
				tempVersZ.x = camera_position(0) + i*xyz_step * 2 * camera_rotation(2, 0);
				tempVersZ.y = camera_position(1) + i*xyz_step * 2 * camera_rotation(2, 1);
				tempVersZ.z = camera_position(2) + i*xyz_step * 2 * camera_rotation(2, 2);

				out_points << "v" << " " << tempVersZ.x << " " << tempVersZ.y << " " << tempVersZ.z << " " << 0 << " " << 0 << " " << 255 << std::endl;
			}
			std::cout << "end save cam pose" << std::endl;
#endif

#if 0
			Eigen::Vector3f max_vector;
			//max_vector = eigen_vector.col(min_eigen_idx);
			max_vector.x() = eigen_vector(0, min_eigen_idx);
			max_vector.y() = eigen_vector(1, min_eigen_idx);
			max_vector.z() = eigen_vector(2, min_eigen_idx);

			//得到平面方程,定义投到平面上的坐标系，X为最大的点云的方向
			//Eigen::Vector3f max
			std::vector<float3> proj_vertices;
			for (int i = 0; i < pnts_pos_.size(); ++i)
			{
				//
				proj_vertices.emplace_back(GetAPtProjectedToPlane(pnts_pos_[i]));
			}

			//从原来的坐标系转化到平面方向的坐标系
#endif
			return true;
		}
		return false;
	}

	bool HWPlane::GenerateWorldCoordToPlaneCoordMatrixByCornerPnts3d()
	{
		Eigen::Matrix3f covariance_matrix = ComputeCovarianceMatrix(corner_pnts_3d_);
		Eigen::Matrix3f eigen_values = Eigen::Matrix3f::Zero(3, 3);
		Eigen::Matrix3f eigen_vector = Eigen::Matrix3f::Zero(3, 3);
		ComputeMatrixEigens(covariance_matrix, eigen_values, eigen_vector);

		//
		//std::cout << "the eigen matrix value is: \n" << eigen_values << std::endl;
		//std::cout << "the eigen matrix vector is: \n" << eigen_vector << std::endl;

		int min_eigen_idx = -1;
		int max_eigen_idx = -1;
		float min_value = FLT_MAX;
		float max_value = FLT_MIN;

		for (int i = 0; i < 3; ++i)
		{
			if (min_value > eigen_values(i, i))
			{
				min_value = eigen_values(i, i);
				min_eigen_idx = i;
			}

			if (max_value < eigen_values(i, i))
			{
				max_value = eigen_values(i, i);
				max_eigen_idx = i;
			}
		}

		//std::cout << "the max eigen idx is: " << max_eigen_idx << std::endl;
		//std::cout << "the min eigen idx is: " << min_eigen_idx << std::endl;
		average_pnts_normal_ = ComputeAverageVertexNormal();
		Eigen::Vector3f PCA_normal = eigen_vector.col(min_eigen_idx);
		Eigen::Matrix3f r_matrix;
		r_matrix.col(0) = eigen_vector.col(max_eigen_idx);
		//std::cout << "PCA_normal: " << PCA_normal.transpose() << "\n";
		//std::cout << "average_pnts_normal_: " << average_pnts_normal_.transpose() << "\n";
		if(PCA_normal.dot(average_pnts_normal_) > 0)
		{
			r_matrix.col(2) = PCA_normal;
		}
		else
		{
			r_matrix.col(2) = -PCA_normal;
		}
		//if (average_pnts_normal_.norm() > 1e-6)
		//{
		//	if (PCA_normal.dot(average_pnts_normal_) > 0)
		//	{
		//		r_matrix.col(2) = PCA_normal;
		//	}
		//	else
		//	{
		//		r_matrix.col(2) = -PCA_normal;
		//	}
		//}
		//else
		//{
		//	//		//float3 e1 = corner_pnts_3d_[1] - corner_pnts_3d_[0];
		//	//		//float3 e2 = corner_pnts_3d_[2] - corner_pnts_3d_[1];
		//	//		//Eigen::Vector3d e1_(e1.x, e1.y, e1.z), e2_(e2.x, e2.y, e2.z);
		//	//		//Eigen::Vector3d normal = e1_.cross(e2_);
		//	//		//normal.normalize();
		//	//		//return normal;
		//	//		//std::cout << "begin getplanenormal" << std::endl;
		//	//		Eigen::Vector3d normal(0, 0, 0);
		//	//		if (corner_pnts_3d_.size() < 3) return normal;
		//	//		//int first_id = 0;
		//	//		//float3 first_point = corner_pnts_3d_[first_id];
		//	//		//float3 second_point = make_float3(0, 0, 0);
		//	//		//float3 third_point = make_float3(0, 0, 0);
		//	//		//float3 first_second = make_float3(0, 0, 0);
		//	//		//float3 second_third = make_float3(0, 0, 0);
		//	//
		//	//		//int second_id = 1;
		//	//		////先确定前两个点，不能离得太近
		//	//		//for (; second_id < corner_pnts_3d_.size(); second_id++) {
		//	//		//	second_point = corner_pnts_3d_[second_id];
		//	//		//	first_second = first_point - second_point;
		//	//		//	float dist = norm(first_second);
		//	//		//	if (dist > 0.01) break;
		//	//		//}
		//	//		//if (second_id == corner_pnts_3d_.size()) {
		//	//		//	std::cerr << "error, all points are close to first point!" << std::endl;
		//	//		//	return Eigen::Vector3d(0, 0, 0);
		//	//		//}
		//	//		////确定第三个点，不能与前两个点共线
		//	//		//int third_id = second_id + 1;
		//	//		//for (; third_id < corner_pnts_3d_.size(); third_id++) {
		//	//		//	third_point = corner_pnts_3d_[third_id];
		//	//		//	second_third = second_id - third_point;
		//	//		//	if (norm(cross(first_second, second_third)) > 0.01) break;
		//	//		//}
		//	//		//if (third_id == corner_pnts_3d_.size()) {
		//	//		//	std::cerr << "error, cannot find a third point to compute normal" << std::endl;
		//	//		//	return Eigen::Vector3d(0, 0, 0);
		//	//		//}
		//	//		int first_id = 0;
		//	//		int third_id = corner_pnts_3d_.size() - 1;
		//	//		int second_id = third_id / 2;
		//	//
		//	//		if (!corner_pnts_3d_.empty()) normal = GetNormalFrom3Points(corner_pnts_3d_[first_id], corner_pnts_3d_[second_id], corner_pnts_3d_[third_id]);
		//	//		if (!corner_pnts_.empty()) {
		//	//			if (IsConcaveEdgePoint(corner_pnts_, 1) != 1) normal *= -1;
		//	//		}
		//	//#ifdef LIULINGFEI
		//	//		else {
		//	//			std::cout << "normal may be inverse" << std::endl;
		//	//		}
		//	//#endif // LIULINGFEI
		//	//		//std::cout << "end getplanenormal" << std::endl;
		//	//		return normal;
		//	//polygon idx计算polygon 法向量
		//	if (corner_pnts_3d_.size() > 3)
		//	{
		//		float max_length = std::numeric_limits<float>::lowest();
		//		int idx0 = 0, idx1 = 1, idx2 = 2;
		//		for (int i = 0; i < corner_pnts_3d_.size() - 2; ++i)
		//		{
		//			Eigen::Vector3f a = Eigen::Vector3f(corner_pnts_3d_[i].x, corner_pnts_3d_[i].y, corner_pnts_3d_[i].z);
		//			Eigen::Vector3f b = Eigen::Vector3f(corner_pnts_3d_[i+1].x, corner_pnts_3d_[i+1].y, corner_pnts_3d_[i+1].z);
		//			Eigen::Vector3f c = Eigen::Vector3f(corner_pnts_3d_[i+2].x, corner_pnts_3d_[i+2].y, corner_pnts_3d_[i+2].z);
		//			float lsum = (c - b).norm() + (b - a).norm();
		//			if (max_length < lsum && 
		//				(c - b).norm() > lsum / 10 && 
		//				(b - a).norm() > lsum / 10)
		//			{
		//				idx0 = i;
		//				idx1 = i + 1;
		//				idx2 = i + 2;
		//				max_length = lsum;
		//			}
		//		}
		//		float3 p0 = corner_pnts_3d_[idx0];
		//		float3 p1 = corner_pnts_3d_[idx1];
		//		float3 p2 = corner_pnts_3d_[idx2];
		//		Eigen::Vector3f tri_normal = GetNormalFrom3Points(p0, p1, p2);
		//		if (tri_normal.norm() > 1e-4)
		//		{
		//			if (PCA_normal.dot(plane_normal_) > 0)
		//			{
		//				r_matrix.col(2) = PCA_normal;
		//			}
		//			else
		//			{
		//				r_matrix.col(2) = -PCA_normal;
		//			}
		//		}
		//		else
		//		{
		//			r_matrix.col(2) = PCA_normal;
		//		}
		//	}
		//	else
		//	{
		//		r_matrix.col(2) = PCA_normal;
		//	}
		//}
		
		plane_normal_ = r_matrix.col(2);
		
		//ax + by + cz + d = 0
		Eigen::Vector3f coeff;
		//float3 coeff = make_float3(0.0, 0.0, 0.0);
		coeff = r_matrix.col(2);

		float3 average_pos = ComputeAverageCornerPnts3dPos();
		//std::cout << "average_pos = " << average_pos.x << ", " << average_pos.y << ", " << average_pos.z << std::endl;
		float d = -(coeff.x() * average_pos.x + coeff.y() * average_pos.y + coeff.z() * average_pos.z);

		coeff_.x = coeff[0];
		coeff_.y = coeff[1];
		coeff_.z = coeff[2];
		coeff_.w = d;
		
		/*int mean_eigen_idx = -1;
		for (int j = 0; j < 3; j++)
		{
			if (j != min_eigen_idx || j != min_eigen_idx)
			{
				mean_eigen_idx = j;
			}
		}
		if (mean_eigen_idx != -1)
		{
			r_matrix.col(1) = eigen_vector.col(mean_eigen_idx);
		}*/

		//计算另一个向量
		r_matrix.col(1) = r_matrix.col(0).cross(r_matrix.col(2));

		Eigen::Vector3f t_matrix(average_pos.x, average_pos.y, average_pos.z);

		plane_to_world_.topLeftCorner(3, 3) = r_matrix;

		plane_to_world_(0, 3) = t_matrix(0);
		plane_to_world_(1, 3) = t_matrix(1);
		plane_to_world_(2, 3) = t_matrix(2);
		//world_to_plane_.topRightCorner(3, 1) = t_matrix;
		plane_to_world_(3, 3) = 1;
		plane_to_world_.row(3) = Eigen::RowVector4f(0, 0, 0, 1);
		//system("pause");

		//Eigen::Matrix4f world_to_plane_inverse = world_to_plane_.inverse();
		world_to_plane_ = plane_to_world_.inverse();

		//std::cout << "plane_to_world =" << std::endl;
		//std::cout << plane_to_world_ << std::endl;
		//std::cout << "world_to_plane =" << std::endl;
		//std::cout << world_to_plane_ << std::endl;

		return true;
	}

	void HWPlane::extractPolygon()
	{
		std::cout << "origin: " << GetOriginPnts().size() << std::endl;
		std::cout << "plane pnts: " <<GetPlanePnts().size() << std::endl;
		if (GetOriginPnts().empty()) return;

		clock_t start_time, end_time;

		start_time = clock();
		GenerateWorldCoordToPlaneCoordMatrix();
		end_time = clock();
		print_time(start_time, end_time, "GenerateWorldCoordToPlane");

		//如果法向量朝上，语义标签就设置为地面，否则设置成墙壁
		std::cout << "semantic label" << std::endl;
		if (abs(plane_normal_.z()) > 0.9) {
			if (pnts_pos_origin_[0].z > 1.0f) semantic_label_ = 0; //Ceiling
			else semantic_label_ = 1; //Floor
		}
		else semantic_label_ = 2; //Wall
		std::cout << "semantic label: " << semantic_label_ << std::endl;

		start_time = clock();;
		ProjectTo3DPlane();
		end_time = clock();
		print_time(start_time, end_time, "ProjectTo3DPlane");

		//planes_vec_[i]->SavePlaneCoordPntsIntoOBJ("D:\\vc_project_new\\huawei_data_indoor\\room\\test_room\\outdoor_top.obj");
		start_time = clock();
		Generate2DPlane();
		end_time = clock();
		print_time(start_time, end_time, "Generate2DPlane");
		start_time = clock();

		//planes_vec_[i]->SetDiameter(planes_vec_[i]->GetPlaneWidth() / 10);
		//std::cout << "set diameter to " << planes_vec_[i]->GetPlaneWidth() / 10 << std::endl;
		//Liulingfei
		//if (!planes_iswide[i])
			//continue;

		start_time = clock();
		DoEstimateBorderFrom2DPlane();
		end_time = clock();
		print_time(start_time, end_time, "DoEstimateBorderFrom2DPlane");
		start_time = clock();
		DoEstimateBorderFrom2DPlaneOptiAlpha();
		end_time = clock();
		print_time(start_time, end_time, "DoEstimateBorderFrom2DPlaneOptiAlpha");
		start_time = clock();
		SortBorderEdgesFromPlaneOptiAlpha();
		end_time = clock();
		print_time(start_time, end_time, "SortBorderEdgesFromPlaneOptiAlpha");
		//planes_vec_[i]->SetDiameter(planes_vec_[i]->GetCloudResolution() * 10);
		//std::cout << "set diameter: " <<  planes_vec_[i]->GetCloudResolution() * 10 << std::endl;

		start_time = clock();;
		LiulingfeiPolygonExtraction(filename_, true);
		end_time = clock();
		print_time(start_time, end_time, "LiuliulingfeiPolygonExtraction");
		std::cout << "LiulingfeiPolygonExtraction done" << std::endl;
		std::cout << "inner_polygon_num_ = " << inner_polygon_num_ << std::endl;
		for (int i = 0; i < inner_polygon_num_; i++) std::cout << inner_edge_pnts_pos_[i].size() << ", ";
		std::cout << std::endl;
		if (edge_pnts_pos_.empty()) return;

		//planes_vec_[i]->SortBorderEdges();
		//planes_vec_[i]->GenerateInitial2DSortedPnts();
		//planes_vec_[i]->GetSmoothRegions();

		double diameter = params_.smooth_region_diameter;
		if (std::abs(diameter) > HW::KMIN_DOUBLE_THRESHOLD) {
			SetDiameter(diameter);
		}
		else {
			//SetDiameter(0.05);
			//initial_params_.smooth_region_diameter = 0.05;
			SetDiameter(0.02);
			initial_params_.smooth_region_diameter = 0.02;
		}
		std::cout << "diameter_max_ = " << diameter_max_ << std::endl;

		start_time = clock();
		NewSmoothRegions();
		end_time = clock();
		print_time(start_time, end_time, "GetSmoothRegions");
		std::cout << "GetSmoothRegions done" << std::endl;
		std::cout << "inner_edge_pnts_smooth_regions_.size() = " << inner_edge_pnts_smooth_regions_.size() << std::endl;

		start_time = clock();
		GetNeighbors();
		end_time = clock();
		print_time(start_time, end_time, "GetNeighbors");
		std::cout << "GetNeighbors done" << std::endl;
		std::cout << "inner_edge_pnts_smooth_regions_.size() = " << inner_edge_pnts_smooth_regions_.size() << std::endl;

		start_time = clock();;
		GetInitialNormals();
		end_time = clock();
		print_time(start_time, end_time, "GetInitialNormals");
		std::cout << "GetInitialNormals done" << std::endl;
		std::cout << "inner_edge_pnts_normal_.size() = " << inner_edge_pnts_normal_.size() << std::endl;

		for (int i = 0; i < inner_polygon_num_; i++) {
			std::vector<float3> points_3d;
			std::vector<float3> points_normal;
			//out_fh << "vn " << pnts_normal_[i].x << " " << pnts_normal_[i].y << " " << pnts_normal_[i].z << std::endl;
			auto& pnts_pos = inner_edge_pnts_pos_[i];
			auto& pnts_normal = inner_edge_pnts_normal_[i];
			for (int i = 0; i < pnts_pos.size(); i++) {
				Eigen::Vector4f pnt(pnts_pos[i].x, pnts_pos[i].y, 0.0, 1.0);
				Eigen::Vector2f nml(pnts_normal[i].x, pnts_normal[i].y);
				nml.normalize();
				Eigen::Vector4f pnt_end(pnts_pos[i].x + nml.x(), pnts_pos[i].y + nml.y(), 0.0, 1.0);

				Eigen::Vector4f transform_pnt = plane_to_world_ * pnt;
				Eigen::Vector4f transform_end = plane_to_world_ * pnt_end;

				points_3d.emplace_back(make_float3(transform_pnt(0), transform_pnt(1), transform_pnt(2)));
				points_3d.emplace_back(make_float3(transform_end(0), transform_end(1), transform_end(2)));
			}
			
			std::stringstream ss;
			ss << i;
			std::string index;
			ss >> index;
			std::ofstream fh(filename_ + "inner_normal" + index + ".obj");
			for (int i = 0; i < points_3d.size(); i++)
			{
				//uchar3 rgb = colors_[edge_pnts_color_index_[i / 2]];
				//float3 color = make_float3(rgb.x / 255.0, rgb.y / 255.0, rgb.z / 255.0);

				float3 start_pnt = points_3d[i];
				float3 nml = points_normal[i];
				fh << "v " << start_pnt.x << " " << start_pnt.y << " " << start_pnt.z << std::endl;
					//<< color.x << " " << color.y << " " << color.z << std::endl;
				//fh << "vn " << nml.x << " " << nml.y << " " << nml.z << std::endl;
			}
			for (int i = 0; i < points_3d.size() / 2 - 1; i++) {
				fh << "l " << 2 * i + 1 << " " << 2 * i + 2 << "\n";
				fh << "l " << 2 * i + 1 << " " << 2 * i + 3 << "\n";
			}
			fh << "l " << 1 << " " << points_3d.size() - 2 << "\n";
			fh << "l " << points_3d.size() - 2 << " " << points_3d.size() - 1 << "\n";
			fh.close();
		}

		start_time = clock();
		DoNormalEstimation();
		end_time = clock();

#ifdef OUTPUT_IMAGE_EDGES_NORMAL
		WriteEdges3D(filename_ + "_image_edges_normal_3d.obj");
		WriteEdges3DWithoutNormals(filename_ + "_image_edges_3d.obj");
#endif // OUTPUT_IMAGE_EDGES_NORMAL

		//std::cout << "after normal smoothing:" << std::endl;
		//for (int i = 0; i < edge_pnts_normal_.size(); i++) {
		//	std::cout << i << ": " << edge_pnts_normal_[i].x << ", " << edge_pnts_normal_[i].y << std::endl;
		//}

		print_time(start_time, end_time, "DoNormalEstimation");
		std::cout << "DoNormalEstimation done" << std::endl;


		start_time = clock();
		DoPolygonSmoothing(filename_);
		//NewPolygonSmoothing(filename_);
		end_time = clock();
		print_time(start_time, end_time, "DoPolygonSmoothing");
		std::cout << "DoPolygonSmoothing done" << std::endl;
#ifdef OUTPUT_IMAGE_EDGES_NORMAL_SMOOTHED
		WriteEdges3D(filename_ + "_image_edges_normal_smoothed_3d.obj");
		WriteEdges3DWithoutNormals(filename_ + "_image_edges_smoothed_3d.obj");
#endif // OUTPUT_IMAGE_EDGES_NORMAL_SMOOTHED

		start_time = clock();
		//planes_vec_[i]->DoPolygonExtraction();
		NewPolygonExtraction();	//initial 
		//NewPolygonExtractionZDG();
		if (corner_pnts_3d_.empty()) return;
		InnerPolygonExtraction();
		std::cout << "inner_corner_pnts_.size() = " << inner_corner_pnts_.size() << std::endl;
		end_time = clock();
		print_time(start_time, end_time, "DoPolygonExtraction");
		std::cout << "DoPolygonExtraction done" << std::endl;

		all_corner_pnts_.clear();
		all_corner_pnts_3d_.clear();
		all_corner_pnts_.insert(all_corner_pnts_.end(), corner_pnts_.begin(), corner_pnts_.end());
		all_corner_pnts_3d_.insert(all_corner_pnts_3d_.end(), corner_pnts_3d_.begin(), corner_pnts_3d_.end());
		contour_index_.resize(inner_polygon_num_ + 2);
		contour_index_[0] = 0;
		contour_index_[1] = corner_pnts_3d_.size();
		for (int i = 0; i < inner_polygon_num_; i++) {
			all_corner_pnts_.insert(all_corner_pnts_.end(), inner_corner_pnts_[i].begin(), inner_corner_pnts_[i].end());
			all_corner_pnts_3d_.insert(all_corner_pnts_3d_.end(), inner_corner_pnts_3d_[i].begin(), inner_corner_pnts_3d_[i].end());
			contour_index_[i + 2] = contour_index_[i + 1] + inner_corner_pnts_[i].size();
		}
		//std::cout << "corner_pnts:" << std::endl;
		//for (int i = 0; i < corner_pnts_.size(); i++) {
		//	std::cout << corner_pnts_[i].x << ", " << all_corner_pnts_[i].x << ", " << corner_pnts_[i].y << ", " << all_corner_pnts_[i].y << std::endl;
		//}
		//for (int i = 0; i < inner_polygon_num_; i++) {
		//	for (int j = 0; j < inner_corner_pnts_[i].size(); j++) {
		//		std::cout << inner_corner_pnts_[i][j].x << ", " << all_corner_pnts_[contour_index_[i+1] + j].x << ", " <<
		//			inner_corner_pnts_[i][j].y << ", " << all_corner_pnts_[contour_index_[i+1] + j].y << std::endl;
		//	}
		//}
		std::cout << "begin update information" << std::endl;
		std::cerr << "outer contour " << corner_pnts_3d_.size() << std::endl;

		UpdateInformation();

#ifdef LIULINGFEI
		std::cout << "outer contour size: " << corner_pnts_3d_.size() << std::endl;
		std::cout << "inner contours size: ";
		for (int i = 0; i < inner_corner_pnts_3d_.size(); i++) std::cout << inner_corner_pnts_3d_[i].size() << ", ";
		std::cout << std::endl;
		std::cout << "contour index: ";
		for (int i = 0; i < contour_index_.size(); i++) std::cout << contour_index_[i] << ", ";
		std::cout << std::endl;
		std::cout << "all_corner_pnts_.size() = " << all_corner_pnts_.size() << std::endl;
#endif

#ifdef OUTPUT_POLYGON
		WriteCornerPoints3D(filename_ + "polygon.obj");
#endif // OUTPUT_POLYGON

#ifdef LIULINGFEI
		std::cout << "WriteCornerPoints3D done" << std::endl;
#endif // LIULINGFEI


		std::cout << "begin calc min dists" << std::endl;
		CalcMinDists();
		std::cout << "end calc min dists" << std::endl;
	}

	void HWPlane::extractPolygonAlpha()
	{
		std::cout << "origin: " << GetOriginPnts().size() << std::endl;
		std::cout << "plane pnts: " << GetPlanePnts().size() << std::endl;
		if (GetOriginPnts().empty()) return;

		clock_t start_time, end_time;
		start_time = clock();
		GenerateWorldCoordToPlaneCoordMatrix();
		end_time = clock();
		print_time(start_time, end_time, "GenerateWorldCoordToPlane");

		//如果法向量朝上，语义标签就设置为地面，否则设置成墙壁
		std::cout << "semantic label" << std::endl;
		if (abs(plane_normal_.z()) > 0.9) {
			if (pnts_pos_origin_[0].z > 1.0f) semantic_label_ = 0; //Ceiling
			else semantic_label_ = 1; //Floor
		}
		else semantic_label_ = 2; //Wall
		std::cout << "semantic label: " << semantic_label_ << std::endl;

		start_time = clock();
		ProjectTo3DPlane();
		end_time = clock();
		print_time(start_time, end_time, "ProjectTo3DPlane");

		//planes_vec_[i]->SavePlaneCoordPntsIntoOBJ("D:\\vc_project_new\\huawei_data_indoor\\room\\test_room\\outdoor_top.obj");
		start_time = clock();
		Generate2DPlane();
		end_time = clock();
		print_time(start_time, end_time, "Generate2DPlane");
		start_time = clock();

		start_time = clock();
		DoEstimateBorderFrom2DPlane();
		end_time = clock();
		print_time(start_time, end_time, "DoEstimateBorderFrom2DPlane");
		start_time = clock();
		DoEstimateBorderFrom2DPlaneOptiAlpha();
		end_time = clock();
		print_time(start_time, end_time, "DoEstimateBorderFrom2DPlaneOptiAlpha");
		start_time = clock();
		SortBorderEdgesFromPlaneOptiAlpha();
		end_time = clock();
		print_time(start_time, end_time, "SortBorderEdgesFromPlaneOptiAlpha");

		edge_pnts_pos_.clear();
		GetPolygonPointFromSortedBorderEdges();
		//edge_pnts_pos_ = edge_points;
		edge_pnts_pos_num_ = edge_pnts_pos_.size();

		/*std::cerr << "---------params_-----------.alpha: " << params_.alpha << std::endl;
		double diameter = params_.alpha*2.0;
		SetDiameter(diameter);*/
		double diameter = params_.smooth_region_diameter;
		if (std::abs(diameter) > HW::KMIN_DOUBLE_THRESHOLD) {
			SetDiameter(diameter);
		}
		else {
			//SetDiameter(0.05);
			//initial_params_.smooth_region_diameter = 0.05;
			SetDiameter(0.02);
			initial_params_.smooth_region_diameter = 0.02;
		}
		SetDiameter(ave_alpha_edge_len_);
		std::cout << "diameter_max_ = " << diameter_max_ << std::endl;

		start_time = clock();
		NewSmoothRegions();
		end_time = clock();
		print_time(start_time, end_time, "GetSmoothRegions");
		std::cout << "GetSmoothRegions done" << std::endl;
		std::cout << "inner_edge_pnts_smooth_regions_.size() = " << inner_edge_pnts_smooth_regions_.size() << std::endl;

		start_time = clock();
		GetNeighbors();
		end_time = clock();
		print_time(start_time, end_time, "GetNeighbors");
		std::cout << "GetNeighbors done" << std::endl;
		std::cout << "inner_edge_pnts_smooth_regions_.size() = " << inner_edge_pnts_smooth_regions_.size() << std::endl;

		start_time = clock();;
		GetInitialNormals();
		end_time = clock();
		print_time(start_time, end_time, "GetInitialNormals");
		std::cout << "GetInitialNormals done" << std::endl;
		std::cout << "inner_edge_pnts_normal_.size() = " << inner_edge_pnts_normal_.size() << std::endl;

		for (int i = 0; i < inner_polygon_num_; i++) {
			std::vector<float3> points_3d;
			std::vector<float3> points_normal;
			//out_fh << "vn " << pnts_normal_[i].x << " " << pnts_normal_[i].y << " " << pnts_normal_[i].z << std::endl;
			auto& pnts_pos = inner_edge_pnts_pos_[i];
			auto& pnts_normal = inner_edge_pnts_normal_[i];
			for (int i = 0; i < pnts_pos.size(); i++) {
				Eigen::Vector4f pnt(pnts_pos[i].x, pnts_pos[i].y, 0.0, 1.0);
				Eigen::Vector2f nml(pnts_normal[i].x, pnts_normal[i].y);
				nml.normalize();
				Eigen::Vector4f pnt_end(pnts_pos[i].x + nml.x(), pnts_pos[i].y + nml.y(), 0.0, 1.0);

				Eigen::Vector4f transform_pnt = plane_to_world_ * pnt;
				Eigen::Vector4f transform_end = plane_to_world_ * pnt_end;

				points_3d.emplace_back(make_float3(transform_pnt(0), transform_pnt(1), transform_pnt(2)));
				points_3d.emplace_back(make_float3(transform_end(0), transform_end(1), transform_end(2)));
			}

			std::stringstream ss;
			ss << i;
			std::string index;
			ss >> index;
			std::ofstream fh(filename_ + "inner_normal_new" + index + ".obj");
			for (int i = 0; i < points_3d.size(); i++)
			{
				//uchar3 rgb = colors_[edge_pnts_color_index_[i / 2]];
				//float3 color = make_float3(rgb.x / 255.0, rgb.y / 255.0, rgb.z / 255.0);

				float3 start_pnt = points_3d[i];
				float3 nml = points_normal[i];
				fh << "v " << start_pnt.x << " " << start_pnt.y << " " << start_pnt.z << std::endl;
				//<< color.x << " " << color.y << " " << color.z << std::endl;
				//fh << "vn " << nml.x << " " << nml.y << " " << nml.z << std::endl;
			}
			for (int i = 0; i < points_3d.size() / 2 - 1; i++) {
				fh << "l " << 2 * i + 1 << " " << 2 * i + 2 << "\n";
				fh << "l " << 2 * i + 1 << " " << 2 * i + 3 << "\n";
			}
			fh << "l " << 1 << " " << points_3d.size() - 2 << "\n";
			fh << "l " << points_3d.size() - 2 << " " << points_3d.size() - 1 << "\n";
			fh.close();
		}

		start_time = clock();
		DoNormalEstimation();
		end_time = clock();
		print_time(start_time, end_time, "DoNormalEstimation");
		std::cout << "DoNormalEstimation done" << std::endl;

#ifdef OUTPUT_IMAGE_EDGES_NORMAL
		WriteEdges3D(filename_ + "_image_edges_normal_alpha_3d.obj");
		WriteEdges3DWithoutNormals(filename_ + "_image_edges_alpha_3d.obj");
#endif // OUTPUT_IMAGE_EDGES_NORMAL

		start_time = clock();
		DoPolygonSmoothing(filename_);
		//NewPolygonSmoothing(filename_);
		end_time = clock();
		print_time(start_time, end_time, "DoPolygonSmoothing");
		std::cout << "DoPolygonSmoothing done" << std::endl;
#ifdef OUTPUT_IMAGE_EDGES_NORMAL_SMOOTHED
		WriteEdges3D(filename_ + "_image_edges_normal_alpha_smoothed_3d.obj");
		WriteEdges3DWithoutNormals(filename_ + "_image_edges_alpha_smoothed_3d.obj");
#endif // OUTPUT_IMAGE_EDGES_NORMAL_SMOOTHED

		start_time = clock();
		//planes_vec_[i]->DoPolygonExtraction();
		NewPolygonExtraction();	//initial 
		//NewPolygonExtractionZDG();
		if (corner_pnts_3d_.empty()) return;
		InnerPolygonExtraction();
		std::cout << "inner_corner_pnts_.size() = " << inner_corner_pnts_.size() << std::endl;
		end_time = clock();
		print_time(start_time, end_time, "DoPolygonExtraction");
		std::cout << "DoPolygonExtraction done" << std::endl;

		all_corner_pnts_.clear();
		all_corner_pnts_3d_.clear();
		all_corner_pnts_.insert(all_corner_pnts_.end(), corner_pnts_.begin(), corner_pnts_.end());
		all_corner_pnts_3d_.insert(all_corner_pnts_3d_.end(), corner_pnts_3d_.begin(), corner_pnts_3d_.end());
		contour_index_.resize(inner_polygon_num_ + 2);
		contour_index_[0] = 0;
		contour_index_[1] = corner_pnts_3d_.size();
		for (int i = 0; i < inner_polygon_num_; i++) {
			all_corner_pnts_.insert(all_corner_pnts_.end(), inner_corner_pnts_[i].begin(), inner_corner_pnts_[i].end());
			all_corner_pnts_3d_.insert(all_corner_pnts_3d_.end(), inner_corner_pnts_3d_[i].begin(), inner_corner_pnts_3d_[i].end());
			contour_index_[i + 2] = contour_index_[i + 1] + inner_corner_pnts_[i].size();
		}
		std::cout << "begin update information" << std::endl;
		std::cerr << "outer contour " << corner_pnts_3d_.size() << std::endl;

		UpdateInformation();

#ifdef OUTPUT_POLYGON
		WriteCornerPoints3D(filename_ + "polygon_alpha.obj");
#endif // OUTPUT_POLYGON

		std::cout << "begin calc min dists" << std::endl;
		CalcMinDists();
		std::cout << "end calc min dists" << std::endl;
	}

	void HWPlane::extractPolygonOpti()
	{
		std::cout << "origin: " << GetOriginPnts().size() << std::endl;
		std::cout << "plane pnts: " << GetPlanePnts().size() << std::endl;
		if (GetOriginPnts().empty()) return;

		clock_t start_time, end_time;

		start_time = clock();
		GenerateWorldCoordToPlaneCoordMatrix();
		end_time = clock();
		print_time(start_time, end_time, "GenerateWorldCoordToPlane");

		//如果法向量朝上，语义标签就设置为地面，否则设置成墙壁
		std::cout << "semantic label" << std::endl;
		if (abs(plane_normal_.z()) > 0.9) {
			if (pnts_pos_origin_[0].z > 1.0f) semantic_label_ = 0; //Ceiling
			else semantic_label_ = 1; //Floor
		}
		else semantic_label_ = 2; //Wall
		std::cout << "semantic label: " << semantic_label_ << std::endl;

		start_time = clock();
		ProjectTo3DPlane();
		end_time = clock();
		print_time(start_time, end_time, "ProjectTo3DPlane");

		//planes_vec_[i]->SavePlaneCoordPntsIntoOBJ("D:\\vc_project_new\\huawei_data_indoor\\room\\test_room\\outdoor_top.obj");
		start_time = clock();
		Generate2DPlane();
		end_time = clock();
		print_time(start_time, end_time, "Generate2DPlane");
		start_time = clock();

		//planes_vec_[i]->SetDiameter(planes_vec_[i]->GetPlaneWidth() / 10);
		//std::cout << "set diameter to " << planes_vec_[i]->GetPlaneWidth() / 10 << std::endl;
		//Liulingfei
		//if (!planes_iswide[i])
		//continue;

		start_time = clock();;
		DoEstimateBorderFrom2DPlane();
		end_time = clock();
		print_time(start_time, end_time, "DoEstimateBorderFrom2DPlane");

		//planes_vec_[i]->SetDiameter(planes_vec_[i]->GetCloudResolution() * 10);
		//std::cout << "set diameter: " <<  planes_vec_[i]->GetCloudResolution() * 10 << std::endl;

		start_time = clock();;
		LiulingfeiPolygonExtraction(filename_, true);
		end_time = clock();
		print_time(start_time, end_time, "LiuliulingfeiPolygonExtraction");
		std::cout << "LiulingfeiPolygonExtraction done" << std::endl;
		std::cout << "inner_polygon_num_ = " << inner_polygon_num_ << std::endl;
		for (int i = 0; i < inner_polygon_num_; i++) std::cout << inner_edge_pnts_pos_[i].size() << ", ";
		std::cout << std::endl;
		if (edge_pnts_pos_.empty()) return;

		//planes_vec_[i]->SortBorderEdges();
		//planes_vec_[i]->GenerateInitial2DSortedPnts();
		//planes_vec_[i]->GetSmoothRegions();

		double diameter = params_.smooth_region_diameter;
		if (std::abs(diameter) > HW::KMIN_DOUBLE_THRESHOLD) {
			SetDiameter(diameter);
		}
		else {
			//SetDiameter(0.05);
			//initial_params_.smooth_region_diameter = 0.05;
			SetDiameter(0.01);
			initial_params_.smooth_region_diameter = 0.01;
		}
		std::cout << "diameter_max_ = " << diameter_max_ << std::endl;

		start_time = clock();;
		NewSmoothRegions();
		end_time = clock();
		print_time(start_time, end_time, "GetSmoothRegions");
		std::cout << "GetSmoothRegions done" << std::endl;
		std::cout << "inner_edge_pnts_smooth_regions_.size() = " << inner_edge_pnts_smooth_regions_.size() << std::endl;

		start_time = clock();;
		GetNeighbors();
		end_time = clock();
		print_time(start_time, end_time, "GetNeighbors");
		std::cout << "GetNeighbors done" << std::endl;
		std::cout << "inner_edge_pnts_smooth_regions_.size() = " << inner_edge_pnts_smooth_regions_.size() << std::endl;

		start_time = clock();;
		GetInitialNormals();
		end_time = clock();
		print_time(start_time, end_time, "GetInitialNormals");
		std::cout << "GetInitialNormals done" << std::endl;
		std::cout << "inner_edge_pnts_normal_.size() = " << inner_edge_pnts_normal_.size() << std::endl;

		for (int i = 0; i < inner_polygon_num_; i++) {
			std::vector<float3> points_3d;
			std::vector<float3> points_normal;
			//out_fh << "vn " << pnts_normal_[i].x << " " << pnts_normal_[i].y << " " << pnts_normal_[i].z << std::endl;
			auto& pnts_pos = inner_edge_pnts_pos_[i];
			auto& pnts_normal = inner_edge_pnts_normal_[i];
			for (int i = 0; i < pnts_pos.size(); i++) {
				Eigen::Vector4f pnt(pnts_pos[i].x, pnts_pos[i].y, 0.0, 1.0);
				Eigen::Vector2f nml(pnts_normal[i].x, pnts_normal[i].y);
				nml.normalize();
				Eigen::Vector4f pnt_end(pnts_pos[i].x + nml.x(), pnts_pos[i].y + nml.y(), 0.0, 1.0);

				Eigen::Vector4f transform_pnt = plane_to_world_ * pnt;
				Eigen::Vector4f transform_end = plane_to_world_ * pnt_end;

				points_3d.emplace_back(make_float3(transform_pnt(0), transform_pnt(1), transform_pnt(2)));
				points_3d.emplace_back(make_float3(transform_end(0), transform_end(1), transform_end(2)));
			}

			std::stringstream ss;
			ss << i;
			std::string index;
			ss >> index;
			std::ofstream fh(filename_ + "inner_normal" + index + ".obj");
			for (int i = 0; i < points_3d.size(); i++)
			{
				//uchar3 rgb = colors_[edge_pnts_color_index_[i / 2]];
				//float3 color = make_float3(rgb.x / 255.0, rgb.y / 255.0, rgb.z / 255.0);

				float3 start_pnt = points_3d[i];
				float3 nml = points_normal[i];
				fh << "v " << start_pnt.x << " " << start_pnt.y << " " << start_pnt.z << std::endl;
				//<< color.x << " " << color.y << " " << color.z << std::endl;
				//fh << "vn " << nml.x << " " << nml.y << " " << nml.z << std::endl;
			}
			for (int i = 0; i < points_3d.size() / 2 - 1; i++) {
				fh << "l " << 2 * i + 1 << " " << 2 * i + 2 << "\n";
				fh << "l " << 2 * i + 1 << " " << 2 * i + 3 << "\n";
			}
			fh << "l " << 1 << " " << points_3d.size() - 2 << "\n";
			fh << "l " << points_3d.size() - 2 << " " << points_3d.size() - 1 << "\n";
			fh.close();
		}

		start_time = clock();
		DoNormalEstimation();
		end_time = clock();

#ifdef OUTPUT_IMAGE_EDGES_NORMAL
		WriteEdges3D(filename_ + "_image_edges_normal_3d.obj");
		WriteEdges3DWithoutNormals(filename_ + "_image_edges_3d.obj");
#endif // OUTPUT_IMAGE_EDGES_NORMAL

		//std::cout << "after normal smoothing:" << std::endl;
		//for (int i = 0; i < edge_pnts_normal_.size(); i++) {
		//	std::cout << i << ": " << edge_pnts_normal_[i].x << ", " << edge_pnts_normal_[i].y << std::endl;
		//}

		print_time(start_time, end_time, "DoNormalEstimation");
		std::cout << "DoNormalEstimation done" << std::endl;


		start_time = clock();
		DoPolygonSmoothing(filename_);
		//NewPolygonSmoothing(filename_);
		end_time = clock();
		print_time(start_time, end_time, "DoPolygonSmoothing");
		std::cout << "DoPolygonSmoothing done" << std::endl;
#ifdef OUTPUT_IMAGE_EDGES_NORMAL_SMOOTHED
		WriteEdges3D(filename_ + "_image_edges_normal_smoothed_3d.obj");
		WriteEdges3DWithoutNormals(filename_ + "_image_edges_smoothed_3d.obj");
#endif // OUTPUT_IMAGE_EDGES_NORMAL_SMOOTHED

		start_time = clock();
		//planes_vec_[i]->DoPolygonExtraction();
		//NewPolygonExtraction();	//initial 
		NewPolygonExtractionZDG();	//optimization(our algorithm)
		if (corner_pnts_3d_.empty()) return;
		InnerPolygonExtraction();
		std::cout << "inner_corner_pnts_.size() = " << inner_corner_pnts_.size() << std::endl;
		end_time = clock();
		print_time(start_time, end_time, "DoPolygonExtraction");
		std::cout << "DoPolygonExtraction done" << std::endl;

		all_corner_pnts_.clear();
		all_corner_pnts_3d_.clear();
		all_corner_pnts_.insert(all_corner_pnts_.end(), corner_pnts_.begin(), corner_pnts_.end());
		all_corner_pnts_3d_.insert(all_corner_pnts_3d_.end(), corner_pnts_3d_.begin(), corner_pnts_3d_.end());
		contour_index_.resize(inner_polygon_num_ + 2);
		contour_index_[0] = 0;
		contour_index_[1] = corner_pnts_3d_.size();
		for (int i = 0; i < inner_polygon_num_; i++) {
			all_corner_pnts_.insert(all_corner_pnts_.end(), inner_corner_pnts_[i].begin(), inner_corner_pnts_[i].end());
			all_corner_pnts_3d_.insert(all_corner_pnts_3d_.end(), inner_corner_pnts_3d_[i].begin(), inner_corner_pnts_3d_[i].end());
			contour_index_[i + 2] = contour_index_[i + 1] + inner_corner_pnts_[i].size();
		}
		//std::cout << "corner_pnts:" << std::endl;
		//for (int i = 0; i < corner_pnts_.size(); i++) {
		//	std::cout << corner_pnts_[i].x << ", " << all_corner_pnts_[i].x << ", " << corner_pnts_[i].y << ", " << all_corner_pnts_[i].y << std::endl;
		//}
		//for (int i = 0; i < inner_polygon_num_; i++) {
		//	for (int j = 0; j < inner_corner_pnts_[i].size(); j++) {
		//		std::cout << inner_corner_pnts_[i][j].x << ", " << all_corner_pnts_[contour_index_[i+1] + j].x << ", " <<
		//			inner_corner_pnts_[i][j].y << ", " << all_corner_pnts_[contour_index_[i+1] + j].y << std::endl;
		//	}
		//}
		std::cout << "begin update information" << std::endl;
		std::cerr << "outer contour " << corner_pnts_3d_.size() << std::endl;

		UpdateInformation();

#ifdef LIULINGFEI
		std::cout << "outer contour size: " << corner_pnts_3d_.size() << std::endl;
		std::cout << "inner contours size: ";
		for (int i = 0; i < inner_corner_pnts_3d_.size(); i++) std::cout << inner_corner_pnts_3d_[i].size() << ", ";
		std::cout << std::endl;
		std::cout << "contour index: ";
		for (int i = 0; i < contour_index_.size(); i++) std::cout << contour_index_[i] << ", ";
		std::cout << std::endl;
		std::cout << "all_corner_pnts_.size() = " << all_corner_pnts_.size() << std::endl;
#endif

#ifdef OUTPUT_POLYGON
		WriteCornerPoints3D(filename_ + "polygon.obj");
#endif // OUTPUT_POLYGON


		std::cout << "begin calc min dists" << std::endl;
		CalcMinDists();
		std::cout << "end calc min dists" << std::endl;

		std::cerr << "to do next..." << std::endl;
	}

	void HWPlane::extractPolygonNoSmoothOpti()
	{
		std::cout << "origin: " << GetOriginPnts().size() << std::endl;
		std::cout << "plane pnts: " << GetPlanePnts().size() << std::endl;
		if (GetOriginPnts().empty()) return;
		clock_t start_time, end_time;
		start_time = clock();
		GenerateWorldCoordToPlaneCoordMatrix();
		end_time = clock();
		print_time(start_time, end_time, "GenerateWorldCoordToPlane");
		start_time = clock();
		ProjectTo3DPlane();
		end_time = clock();
		print_time(start_time, end_time, "ProjectTo3DPlane");
		//planes_vec_[i]->SavePlaneCoordPntsIntoOBJ("D:\\vc_project_new\\huawei_data_indoor\\room\\test_room\\outdoor_top.obj");
		start_time = clock();
		Generate2DPlane();
		end_time = clock();
		print_time(start_time, end_time, "Generate2DPlane");
		start_time = clock();

		start_time = clock();;
		LiulingfeiPolygonExtraction(filename_, true);
		end_time = clock();
		print_time(start_time, end_time, "LiuliulingfeiPolygonExtraction");
		std::cout << "LiulingfeiPolygonExtraction done" << std::endl;
		std::cout << "inner_polygon_num_ = " << inner_polygon_num_ << std::endl;
		for (int i = 0; i < inner_polygon_num_; i++) std::cout << inner_edge_pnts_pos_[i].size() << ", ";
		std::cout << std::endl;
		if (edge_pnts_pos_.empty()) return;

		double diameter = params_.smooth_region_diameter;
		if (std::abs(diameter) > HW::KMIN_DOUBLE_THRESHOLD) {
			SetDiameter(diameter);
		}
		else {
			//SetDiameter(0.05);
			//initial_params_.smooth_region_diameter = 0.05;
			SetDiameter(0.01);
			initial_params_.smooth_region_diameter = 0.01;
		}
		std::cout << "diameter_max_ = " << diameter_max_ << std::endl;
		start_time = clock();;
		NewSmoothRegions();
		end_time = clock();
		print_time(start_time, end_time, "GetSmoothRegions");
		std::cout << "GetSmoothRegions done" << std::endl;
		std::cout << "inner_edge_pnts_smooth_regions_.size() = " << inner_edge_pnts_smooth_regions_.size() << std::endl;
		start_time = clock();;
		GetNeighbors();
		end_time = clock();
		print_time(start_time, end_time, "GetNeighbors");
		std::cout << "GetNeighbors done" << std::endl;
		std::cout << "inner_edge_pnts_smooth_regions_.size() = " << inner_edge_pnts_smooth_regions_.size() << std::endl;
		start_time = clock();;
		GetInitialNormals();
		end_time = clock();
		print_time(start_time, end_time, "GetInitialNormals");
		std::cout << "GetInitialNormals done" << std::endl;
		std::cout << "inner_edge_pnts_normal_.size() = " << inner_edge_pnts_normal_.size() << std::endl;
		start_time = clock();
		DoNormalEstimation();
		end_time = clock();

#ifdef OUTPUT_IMAGE_EDGES_NORMAL
		WriteEdges3D(filename_ + "_image_edges_normal_3d.obj");
		WriteEdges3DWithoutNormals(filename_ + "_image_edges_3d.obj");
#endif // OUTPUT_IMAGE_EDGES_NORMAL
		//delete smoothing process
		start_time = clock();
		//planes_vec_[i]->DoPolygonExtraction();
		//NewPolygonExtraction();	//initial 
		//NewPolygonExtractionZDG();	//optimization(our algorithm)
		NewPolygonExtractionWithoutSmoothZDG();	//optimization without smooth plane points
		return;
		if (corner_pnts_3d_.empty()) return;
		InnerPolygonExtraction();
		std::cout << "inner_corner_pnts_.size() = " << inner_corner_pnts_.size() << std::endl;
		end_time = clock();
		print_time(start_time, end_time, "DoPolygonExtraction");
		std::cout << "DoPolygonExtraction done" << std::endl;

		all_corner_pnts_.clear();
		all_corner_pnts_3d_.clear();
		all_corner_pnts_.insert(all_corner_pnts_.end(), corner_pnts_.begin(), corner_pnts_.end());
		all_corner_pnts_3d_.insert(all_corner_pnts_3d_.end(), corner_pnts_3d_.begin(), corner_pnts_3d_.end());
		contour_index_.resize(inner_polygon_num_ + 2);
		contour_index_[0] = 0;
		contour_index_[1] = corner_pnts_3d_.size();
		for (int i = 0; i < inner_polygon_num_; i++) {
			all_corner_pnts_.insert(all_corner_pnts_.end(), inner_corner_pnts_[i].begin(), inner_corner_pnts_[i].end());
			all_corner_pnts_3d_.insert(all_corner_pnts_3d_.end(), inner_corner_pnts_3d_[i].begin(), inner_corner_pnts_3d_[i].end());
			contour_index_[i + 2] = contour_index_[i + 1] + inner_corner_pnts_[i].size();
		}
		//std::cout << "corner_pnts:" << std::endl;
		//for (int i = 0; i < corner_pnts_.size(); i++) {
		//	std::cout << corner_pnts_[i].x << ", " << all_corner_pnts_[i].x << ", " << corner_pnts_[i].y << ", " << all_corner_pnts_[i].y << std::endl;
		//}
		//for (int i = 0; i < inner_polygon_num_; i++) {
		//	for (int j = 0; j < inner_corner_pnts_[i].size(); j++) {
		//		std::cout << inner_corner_pnts_[i][j].x << ", " << all_corner_pnts_[contour_index_[i+1] + j].x << ", " <<
		//			inner_corner_pnts_[i][j].y << ", " << all_corner_pnts_[contour_index_[i+1] + j].y << std::endl;
		//	}
		//}
		std::cout << "begin update information" << std::endl;
		std::cerr << "outer contour " << corner_pnts_3d_.size() << std::endl;

		UpdateInformation();

#ifdef OUTPUT_POLYGON
		WriteCornerPoints3D(filename_ + "polygon_nosmooth.obj");
#endif // OUTPUT_POLYGON


		std::cout << "begin calc min dists" << std::endl;
		CalcMinDists();
		std::cout << "end calc min dists" << std::endl;
	}

	void HWPlane::SetFilename(std::string filename)
	{
		filename_ = filename;
	}

	float3 HWPlane::ComputeAverageCornerPnts3dPos()
	{
		float3 pos = make_float3(0, 0, 0);
		for (int i = 0; i < corner_pnts_3d_.size(); i++) {
			pos += corner_pnts_3d_[i];
		}
		pos /= corner_pnts_3d_.size();
		return pos;
	}

	Eigen::Vector3f HWPlane::GetPlaneNormal()
	{
//		//float3 e1 = corner_pnts_3d_[1] - corner_pnts_3d_[0];
//		//float3 e2 = corner_pnts_3d_[2] - corner_pnts_3d_[1];
//		//Eigen::Vector3d e1_(e1.x, e1.y, e1.z), e2_(e2.x, e2.y, e2.z);
//		//Eigen::Vector3d normal = e1_.cross(e2_);
//		//normal.normalize();
//		//return normal;
//		//std::cout << "begin getplanenormal" << std::endl;
//		Eigen::Vector3d normal(0, 0, 0);
//		if (corner_pnts_3d_.size() < 3) return normal;
//		//int first_id = 0;
//		//float3 first_point = corner_pnts_3d_[first_id];
//		//float3 second_point = make_float3(0, 0, 0);
//		//float3 third_point = make_float3(0, 0, 0);
//		//float3 first_second = make_float3(0, 0, 0);
//		//float3 second_third = make_float3(0, 0, 0);
//
//		//int second_id = 1;
//		////先确定前两个点，不能离得太近
//		//for (; second_id < corner_pnts_3d_.size(); second_id++) {
//		//	second_point = corner_pnts_3d_[second_id];
//		//	first_second = first_point - second_point;
//		//	float dist = norm(first_second);
//		//	if (dist > 0.01) break;
//		//}
//		//if (second_id == corner_pnts_3d_.size()) {
//		//	std::cerr << "error, all points are close to first point!" << std::endl;
//		//	return Eigen::Vector3d(0, 0, 0);
//		//}
//
//		////确定第三个点，不能与前两个点共线
//		//int third_id = second_id + 1;
//		//for (; third_id < corner_pnts_3d_.size(); third_id++) {
//		//	third_point = corner_pnts_3d_[third_id];
//		//	second_third = second_id - third_point;
//		//	if (norm(cross(first_second, second_third)) > 0.01) break;
//		//}
//		//if (third_id == corner_pnts_3d_.size()) {
//		//	std::cerr << "error, cannot find a third point to compute normal" << std::endl;
//		//	return Eigen::Vector3d(0, 0, 0);
//		//}
//		int first_id = 0;
//		int third_id = corner_pnts_3d_.size() - 1;
//		int second_id = third_id / 2;
//
//		if (!corner_pnts_3d_.empty()) normal = GetNormalFrom3Points(corner_pnts_3d_[first_id], corner_pnts_3d_[second_id], corner_pnts_3d_[third_id]);
//		if (!corner_pnts_.empty()) {
//			if (IsConcaveEdgePoint(corner_pnts_, 1) != 1) normal *= -1;
//		}
//#ifdef LIULINGFEI
//		else {
//			std::cout << "normal may be inverse" << std::endl;
//		}
//#endif // LIULINGFEI
//		//std::cout << "end getplanenormal" << std::endl;
//		return normal;

		//Eigen::Vector3f z = plane_to_world_.block(0, 2, 3, 1).normalized();
		//return Eigen::Vector3d(z(0), z(1), z(2));
		return plane_normal_.normalized();
	}

	Eigen::Vector3f HWPlane::GetNormalFrom3Points(float3 a, float3 b, float3 c)
	{
#ifdef LIULINGFEI
		printf("GetNormalFrom3Points: (%f, %f, %f), (%f, %f, %f), (%f, %f, %f), ",
			a.x, a.y, a.z,
			b.x, b.y, b.z,
			c.x, c.y, c.z
		);
#endif
		float3 e1 = b - a;
		float3 e2 = c - b;
		Eigen::Vector3f e1_(e1.x, e1.y, e1.z), e2_(e2.x, e2.y, e2.z);
		Eigen::Vector3f normal = e1_.cross(e2_);
		if (normal.norm() < 0.0001) {
#ifdef LIULINGFEI
			printf("result = %f, %f, %f\n", 0, 0, 0);
#endif
			return Eigen::Vector3f(0, 0, 0);
		}
		else {
			Eigen::Vector3f temp = normal.normalized();
#ifdef LIULINGFEI
			printf("result = %f, %f, %f\n", temp(0), temp(1), temp(2));
#endif
			return normal.normalized();
		}
	}

	Eigen::Vector3f HWPlane::GetNormalFrom3PntsEigen(Eigen::Vector3f& a, Eigen::Vector3f& b, Eigen::Vector3f& c)
	{
		Eigen::Vector3f e1 = b - a;
		Eigen::Vector3f e2 = c - b;
		Eigen::Vector3f normal = e1.cross(e2);
		if (normal.norm() < 1e-6)
		{
			return Eigen::Vector3f(0, 0, 0);
		}
		else
		{
			return normal.normalized();
		}
	}

	//映射到平面的直线为：x = x1 + k*A_; y = y1+k*B_; z = z1 + k*C_ 求1/k
	float3 HWPlane::GetProj3DPnt(const float3& pnt)
	{
		float dist = GetDistToPlane(pnt);

		/*std::cout << "the pnt is: " << pnt.x << " " << pnt.y << " " << pnt.z << std::endl;
		std::cout << "the plane function: " << coeff_.x << " " << coeff_.y << " " << coeff_.z <<" " << d_ << std::endl;
		std::cout << "dist is: " << dist << std::endl;*/

		float k = -1 * dist / (std::sqrtf(coeff_.x*coeff_.x + coeff_.y*coeff_.y + coeff_.z*coeff_.z));
		//std::cout << "k: " << k << std::endl;

		float x = pnt.x + k * coeff_.x;
		float y = pnt.y + k * coeff_.y;
		float z = pnt.z + k * coeff_.z;
		
		//std::cout << "the proj pnt is: " << x << " " << y << " " << z << std::endl;
		//system("pause");

		return make_float3(x, y, z);
	}

	float2 HWPlane::Proj2Plane(float3 & pnt)
	{
		Eigen::Vector4f p(pnt.x, pnt.y, pnt.z, 1.0f);
		Eigen::Vector4f plane_pnt = world_to_plane_*p;
		return make_float2(plane_pnt(0), plane_pnt(1));
	}

	void HWPlane::ProjectTo3DPlane()
	{
		//先要判断顶点是否和平面离的非常近
		for (int i = 0; i < pnts_pos_origin_.size(); ++i)
		{
			//float dist = GetDistToPlane(pnts_pos_[i]);
			/*if (std::abs(dist) < 0.02)
				continue;*/
			float3 proj_pnt = GetProj3DPnt(pnts_pos_origin_[i]);
			pnts_pos_[i].x = proj_pnt.x;
			pnts_pos_[i].y = proj_pnt.y;
			pnts_pos_[i].z = proj_pnt.z;
		}
	}

	//plane operation
	void HWPlane::Generate2DPlane()
	{
		/*for (int i = 0; i < pnts_pos_.size(); ++i)
		{
			Eigen::Vector4f pnt(pnts_pos_[i].x, pnts_pos_[i].y, pnts_pos_[i].z, 1.0f);
			Eigen::Vector4f plane_pnt = world_to_plane_*pnt;

			plane_coord_pos_.emplace_back(make_float3(plane_pnt(0), plane_pnt(1), plane_pnt(2)));
		}*/

		plane_coord_pos_.clear();
		boxMin_2d_ = Eigen::Vector2f(FLT_MAX, FLT_MAX), boxMax_2d_ = Eigen::Vector2f(-FLT_MAX, -FLT_MAX);
		box_min_= Eigen::Vector3f(FLT_MAX, FLT_MAX, FLT_MAX), box_max_= Eigen::Vector3f(-FLT_MAX, -FLT_MAX, -FLT_MAX);
		for (int i = 0; i < pnts_pos_.size(); ++i)
		{
			Eigen::Vector4f pnt(pnts_pos_[i].x, pnts_pos_[i].y, pnts_pos_[i].z, 1.0f);
			if (pnt(0) < box_min_(0))
				box_min_(0) = pnt(0);
			if (pnt(1) < box_min_(1))
				box_min_(1) = pnt(1);
			if (pnt(2) < box_min_(2))
				box_min_(2) = pnt(2);
			if (pnt(0) > box_max_(0))
				box_max_(0) = pnt(0);
			if (pnt(1) > box_max_(1))
				box_max_(1) = pnt(1);
			if (pnt(2) > box_max_(2))
				box_max_(2) = pnt(2);
			Eigen::Vector4f plane_pnt = world_to_plane_*pnt;
			//std::cout << "before world_to_plane: " << pnt << std::endl;
			//std::cout << "after world_to_plane: " << plane_pnt << std::endl;
			if (plane_pnt(0) < boxMin_2d_(0))
				boxMin_2d_(0) = plane_pnt(0);
			if (plane_pnt(1) < boxMin_2d_(1))
				boxMin_2d_(1) = plane_pnt(1);
			if (plane_pnt(0) > boxMax_2d_(0))
				boxMax_2d_(0) = plane_pnt(0);
			if (plane_pnt(1) > boxMax_2d_(1))
				boxMax_2d_(1) = plane_pnt(1);
			plane_coord_pos_.emplace_back(make_float3(plane_pnt(0), plane_pnt(1), plane_pnt(2)));
		}
		plane_width_ = min(boxMax_2d_(0) - boxMin_2d_(0), boxMax_2d_(1) - boxMin_2d_(1));
		plane_height_ = max(boxMax_2d_(0) - boxMin_2d_(0), boxMax_2d_(1) - boxMin_2d_(1));

#ifdef LIULINGFEI
		std::cout << "plane_width_ = " << plane_width_ << std::endl;
		std::cout << "plane_height_ = " << plane_height_ << std::endl;
#endif // LIULINGFEI

		//std::cout << "plane_coord_pos_:" << std::endl;
		//for (int i = 0; i < plane_coord_pos_.size(); i++) {
			//std::cout << i << ": " << plane_coord_pos_[i].x << ", " << plane_coord_pos_[i].y << ", " << plane_coord_pos_[i].z << std::endl;
		//}

		plane_coord_pnts_normal_.clear();
		for (int i = 0; i < pnts_normal_.size(); ++i)
		{
			Eigen::Vector4f pnt_normal(pnts_normal_[i].x, pnts_normal_[i].y, pnts_normal_[i].z, 1.0f);
			Eigen::Vector4f plane_pnt_normal = world_to_plane_ * pnt_normal;

			plane_coord_pnts_normal_.emplace_back(make_float3(plane_pnt_normal(0), plane_pnt_normal(1), plane_pnt_normal(2)));
		}
		std::cout << "end compute the plane coordnate position and normal oritation" << std::endl;
		//system("pause");
	}

	void HWPlane::PointCloudSORFilter()
	{
		CCLib::PointCloudTpl<CCLib::GenericIndexedCloudPersist>* cloud = new CCLib::PointCloudTpl<CCLib::GenericIndexedCloudPersist>();
		int sor_point_num = HW::HWParams::getInstance().sor_params_.point_num_;
		float sor_multiplier = HW::HWParams::getInstance().sor_params_.devialtion_multiplier_;
		//设置参数
		for (int i = 0; i < pnts_pos_.size(); i++) {
			cloud->addPoint(CCVector3(pnts_pos_[i].x, pnts_pos_[i].y, pnts_pos_[i].z));
		}

		CCLib::ReferenceCloud* selection = CCLib::CloudSamplingTools::sorFilter(cloud, sor_point_num, sor_multiplier);

		//生成一个sor对象
		HW::HWPointCloud* sor_output = new HW::HWPointCloud();
		uchar3 color_tmp;
		//vertex_pos_vec.clear();
		for (int i = 0; i < selection->size(); ++i) {
			const CCVector3* p = cloud->getPointPersistentPtr(selection->getPointGlobalIndex(i));
			float3 point;
			point.x = p->x;
			point.y = p->y;
			point.z = p->z;
			sor_output->AddPoint(point);
			if (!this->pnts_normal_.empty())
			{
				point.x = this->pnts_normal_[selection->getPointGlobalIndex(i)].x;
				point.y = this->pnts_normal_[selection->getPointGlobalIndex(i)].y;
				point.z = this->pnts_normal_[selection->getPointGlobalIndex(i)].z;
				sor_output->AddNormal(point);
			}
			if (!this->pnts_color_.empty())
			{
				color_tmp.x = this->pnts_color_[selection->getPointGlobalIndex(i)].x;
				color_tmp.y = this->pnts_color_[selection->getPointGlobalIndex(i)].y;
				color_tmp.z = this->pnts_color_[selection->getPointGlobalIndex(i)].z;
				sor_output->AddColor(color_tmp);
			}
		}

		filtered_plane_pc = sor_output;
	}


	void HWPlane::ComputeCornerPnts3dFromCornerPnts2d()
	{
		corner_pnts_3d_.clear();
		for (int i = 0; i < corner_pnts_.size(); i++) {
			Eigen::Vector4f pnt(corner_pnts_[i].x, corner_pnts_[i].y, 0.0, 1.0);
			Eigen::Vector4f transform_pnt = plane_to_world_ * pnt;
			corner_pnts_3d_.emplace_back(make_float3(transform_pnt(0), transform_pnt(1), transform_pnt(2)));
		}
	}

	void HWPlane::ComputeArea()
	{
		area_ = 0;
		for (int i = 0; i < triangles_idx_.size(); i++) {
			int ia = triangles_idx_[i].x;
			int ib = triangles_idx_[i].y;
			int ic = triangles_idx_[i].z;

			float2 a = all_corner_pnts_[ia];
			float2 b = all_corner_pnts_[ib];
			float2 c = all_corner_pnts_[ic];

			float area = ComputeTriangleAreaFrom3Points2d(a, b, c);
			area_ += area;
		}
		std::cout << "Compute area, area_ = " << area_ << std::endl;
	}

	float HWPlane::ComputeTriangleAreaFrom3Points2d(float2 a, float2 b, float2 c)
	{
		return std::abs(0.5 * ((b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y)));
	}

	void HWPlane::UpdateRefinedSortedBorderPnts2D()
	{
		if (!sorted_border_pnts_2d_.empty())
		{
			//
			if(!sorted_border_pnts_weight_2d_.empty())
				sorted_border_pnts_weight_2d_.clear();
			ComputeSortedVerticesListWeights();
		}
	}

	bool HWPlane::MapSortedEdgePnt2PntPosIdx()
	{
		if (!plane_coord_pos_.empty())
		{
			for (int i = 0; i < sorted_border_pnts_2d_.size(); ++i)
			{
				sbp_to_pnts_pos.emplace_back(-1);
			}

			for (int i = 0; i < sorted_border_pnts_2d_.size(); ++i)
			{
				for (int j = 0; j < plane_coord_pos_.size(); ++j)
				{
					Point_2d tmp_pnt(plane_coord_pos_[j].x, plane_coord_pos_[j].y);
					if (CheckSameSegmentPnts(sorted_border_pnts_2d_[i], tmp_pnt))
					{
						sbp_to_pnts_pos[i] = j;
						break;
					}
				}
			}
		}

		//
		if (sbp_to_pnts_pos.empty())
			return false;
		for (int i = 0; i < sbp_to_pnts_pos.size(); ++i)
		{
			if (sbp_to_pnts_pos[i] == -1)
				return false;
		}

		return true;
	}

	//
	void HWPlane::SavePlanePointIntoOBJ(const std::string path)
	{
		std::ofstream out_fh(path);
		if (pnts_pos_.size() == 0)
		{
			printf("none vertices!\n");
			return;
		}
		for (int i = 0; i < pnts_pos_.size(); ++i)
		{
			out_fh << "v " << pnts_pos_[i].x << " " << pnts_pos_[i].y << " " << pnts_pos_[i].z << std::endl;
		}
		for (int i = 0; i < pnts_normal_.size(); ++i)
		{
			out_fh << "vn " << pnts_normal_[i].x << " " << pnts_normal_[i].y << " " << pnts_normal_[i].z << std::endl;
		}
		out_fh.close();
	}
	
	void HWPlane::SavePnts2dIntoPnts3DOBJ(const std::string& path,
		const std::vector<Eigen::Vector2f>& pnts2d)
	{
		
		if (pnts2d.size() == 0)
		{
			printf("none vertices!\n");
			return;
		}
		std::ofstream out_fh(path);
		std::vector<Eigen::Vector3f> pnts3d;
		for (int i = 0; i < pnts2d.size(); ++i)
		{
			Eigen::Vector4f tmp_pnt = Eigen::Vector4f(pnts2d[i][0], pnts2d[i][1], 0.0, 1.0);
			Eigen::Vector4f transform_pnt = plane_to_world_ * tmp_pnt;
			Eigen::Vector3f transform_pnt3d = Eigen::Vector3f(transform_pnt[0], transform_pnt[1], transform_pnt[2]);
			pnts3d.emplace_back(transform_pnt3d);
		}

		for (int i = 0; i < pnts3d.size(); ++i)
		{
			out_fh << "v " << pnts3d[i][0] << " " << pnts3d[i][1] << " " << pnts3d[i][2] << std::endl;
		}
		out_fh.close();
	}

	void HWPlane::SaveLinesPnts2dIntoLinesPnts3DOBJ(const std::string& path,
		const std::vector<Eigen::Vector2f>& pnts2d)
	{
		if (pnts2d.size() < 2)
		{
			printf("none vertices!\n");
			return;
		}
		std::ofstream out_fh(path);
		std::vector<Eigen::Vector3f> pnts3d;
		for (int i = 0; i < pnts2d.size(); ++i)
		{
			Eigen::Vector4f tmp_pnt = Eigen::Vector4f(pnts2d[i][0], pnts2d[i][1], 0.0, 1.0);
			Eigen::Vector4f transform_pnt = plane_to_world_ * tmp_pnt;
			Eigen::Vector3f transform_pnt3d = Eigen::Vector3f(transform_pnt[0], transform_pnt[1], transform_pnt[2]);
			pnts3d.emplace_back(transform_pnt3d);
		}
		for (int i = 0; i < pnts3d.size(); ++i)
		{
			out_fh << "v " << pnts3d[i][0] << " " << pnts3d[i][1] << " " << pnts3d[i][2] << std::endl;
		}
		for (int i = 0; i < pnts3d.size() / 2; ++i)
		{
			out_fh << "l " << 2 * i + 1 << " " << 2 * i + 2 << std::endl;
		}
		out_fh.close();
	}

	void HWPlane::SavePolygonLines2dIntoPolygonLines3DOBJ(const std::string& path,
		const std::vector<Eigen::Vector2f>& pnts2d)
	{
		if (pnts2d.size() < 3)
		{
			printf("none vertices!\n");
			return;
		}
		std::ofstream out_fh(path);
		std::vector<Eigen::Vector3f> pnts3d;
		for (int i = 0; i < pnts2d.size(); ++i)
		{
			Eigen::Vector4f tmp_pnt = Eigen::Vector4f(pnts2d[i][0], pnts2d[i][1], 0.0, 1.0);
			Eigen::Vector4f transform_pnt = plane_to_world_ * tmp_pnt;
			Eigen::Vector3f transform_pnt3d = Eigen::Vector3f(transform_pnt[0], transform_pnt[1], transform_pnt[2]);
			pnts3d.emplace_back(transform_pnt3d);
		}
		for (int i = 0; i < pnts3d.size(); ++i)
		{
			out_fh << "v " << pnts3d[i][0] << " " << pnts3d[i][1] << " " << pnts3d[i][2] << std::endl;
		}
		for (int i = 0; i < pnts3d.size(); ++i)
		{
			int ii = (i + 1) % pnts3d.size();
			out_fh << "l " << i + 1 << " " << ii + 1 << std::endl;
		}
	}

	void HWPlane::SaveHWOptiPnts2dIntoPolygonLineObj(const std::string& path, const std::vector<HWOptiPnt2D>& pnts)
	{
		std::vector<Eigen::Vector3f> all_edges_pnts;
		std::vector<Eigen::Vector3f> all_edges_pnts_normals;
		float len_normal = 0.01;
		for (int i = 0; i < pnts.size(); ++i)
		{
			Eigen::Vector2f tmp_pnt2d = pnts[i].pos_;
			Eigen::Vector4f tmp_pnt = Eigen::Vector4f(tmp_pnt2d[0], tmp_pnt2d[1], 0.0, 1.0);
			Eigen::Vector2f nml = pnts[i].normal_;
			nml.normalize();
			Eigen::Vector4f pnt_end(tmp_pnt2d[0] + nml.x(), tmp_pnt2d[1] + nml.y(), 0.0, 1.0);
			Eigen::Vector4f transform_pnt = plane_to_world_ * tmp_pnt;
			Eigen::Vector4f transform_end = plane_to_world_ * pnt_end;
			Eigen::Vector3f transform_pnt_3d = Eigen::Vector3f(transform_pnt[0], transform_pnt[1], transform_pnt[2]);
			Eigen::Vector3f transform_end_pnt_3d = Eigen::Vector3f(transform_end[0], transform_end[1], transform_end[2]);
			Eigen::Vector3f transform_normal = transform_end_pnt_3d - transform_pnt_3d;
			transform_normal.normalize();
			all_edges_pnts.emplace_back(transform_pnt_3d);
			all_edges_pnts_normals.emplace_back(transform_normal);
		}
		WritePolygonPnts2dIntoPolygonPnt3dWithNormals(path, all_edges_pnts,
			all_edges_pnts_normals, len_normal);
	}

	void HWPlane::SaveLinesPnts2dIntoLinesPnt3DOBJWithNormal(const std::string& path, const std::vector<Eigen::Vector2f>& pnts_pos,
		const std::vector<Eigen::Vector2f>& pnts_normals)
	{
		std::vector<Eigen::Vector3f> all_edges_pnts;
		std::vector<Eigen::Vector3f> all_edges_pnts_normals;
		float len_normal = 0.01;
		for (int i = 0; i < pnts_pos.size(); ++i)
		{
			Eigen::Vector2f tmp_pnt2d = pnts_pos[i];
			Eigen::Vector4f tmp_pnt = Eigen::Vector4f(tmp_pnt2d[0], tmp_pnt2d[1], 0.0, 1.0);
			Eigen::Vector2f nml = pnts_normals[i];
			nml.normalize();
			Eigen::Vector4f pnt_end(tmp_pnt2d[0] + nml.x(), tmp_pnt2d[1] + nml.y(), 0.0, 1.0);
			Eigen::Vector4f transform_pnt = plane_to_world_ * tmp_pnt;
			Eigen::Vector4f transform_end = plane_to_world_ * pnt_end;
			Eigen::Vector3f transform_pnt_3d = Eigen::Vector3f(transform_pnt[0], transform_pnt[1], transform_pnt[2]);
			Eigen::Vector3f transform_end_pnt_3d = Eigen::Vector3f(transform_end[0], transform_end[1], transform_end[2]);
			Eigen::Vector3f transform_normal = transform_end_pnt_3d - transform_pnt_3d;
			transform_normal.normalize();
			all_edges_pnts.emplace_back(transform_pnt_3d);
			all_edges_pnts_normals.emplace_back(transform_normal);
		}
		WriteLinePnts2dIntoLinePnt3dWithNormals(path, all_edges_pnts, all_edges_pnts_normals, len_normal);
	}

	void HWPlane::SavePnts3dIntoObj(const std::string& path, const std::vector<Eigen::Vector3f>& pnts3d)
	{
		std::ofstream fh(path);
		if (fh.is_open())
		{
			for (int i = 0; i < pnts3d.size(); ++i)
			{
				fh << "v " << pnts3d[i][0] << " " << pnts3d[i][1] << " " << pnts3d[i][2] << std::endl;
			}
		}
		fh.close();
	}

	void HWPlane::SortBorderEdges()
	{
		if (border_edges_.empty())
		{
			//std::cout << "1111111111111111111" << std::endl;
			//system("pause");
			return;
		}
		//std::vector<Segment> sorted_edges;
		std::vector<bool> traverse_vec;
		traverse_vec.resize(border_edges_.size());
		for (int i = 0; i < border_edges_.size(); ++i)
		{
			traverse_vec[i] = false;
		}

		/*std::ofstream fh("D:\\room_sketch\\data\\huawei_data\\huawei_outdoor_test\\outdoor_unsorted_edges_1.obj");
		for (int i = 0; i < border_edges_.size(); ++i)
		{
			fh << "v " << border_edges_[i].start().x() << " " << border_edges_[i].start().y() << " " << "0.0" << std::endl;
		}
		fh.close();*/
		
		float max_polygon_length = 0.0;

#if 0	//新的获取polygon
#else
		for (int idx = 0; idx < border_edges_.size(); ++idx) {
			if (traverse_vec[idx] == true)
				continue;
			Segment start_edge = border_edges_[idx];	
			traverse_vec[idx] = true;
			std::vector<Segment> sorted_border_edges;
			sorted_border_edges.emplace_back(start_edge);
			//sorted_border_edges_2d_.emplace_back(start_edge);
			int count = 0;
			bool find_next_edge;
			do
			{
				Point_2d prev_end_pnt = start_edge.end();
				//std::cout << "prev_end_pnt: " << prev_end_pnt.x()<< " " << prev_end_pnt.y() << std::endl;

				find_next_edge = false;
				for (int i = 0; i < border_edges_.size(); ++i)
				{
					//寻找下一条边，让边按照某种顺序排列
					Point_2d start_pnt = border_edges_[i].start();
					Point_2d end_pnt = border_edges_[i].end();

					if (!traverse_vec[i] && CheckSameSegmentPnts(start_pnt, prev_end_pnt))
					{
						//std::cout << "i: " << i << std::endl;
						start_edge = border_edges_[i];
						sorted_border_edges.emplace_back(start_edge);
						traverse_vec[i] = true;
						find_next_edge = true;
						break;
					}
					else if (!traverse_vec[i] && CheckSameSegmentPnts(end_pnt, prev_end_pnt))
					{
						//std::cout << "i: " << i << std::endl;
						//构建新的segment
						Segment start_new_edge(end_pnt, start_pnt);
						start_edge = start_new_edge;
						sorted_border_edges.emplace_back(start_edge);
						traverse_vec[i] = true;
						find_next_edge = true;
						break;
					}
				}
				if (!find_next_edge)
				{
#ifdef LIULINGFEI
					std::cout << "the polygon is not closed, therefore to do next...." << std::endl;
#endif // LIULINGFEI
					if (sorted_border_edges_2d_.size() < sorted_border_edges.size())
						sorted_border_edges_2d_.swap(sorted_border_edges);
					break;
				}
				++count;
				//system("pause");
			} while (find_next_edge);
			//} while (count < border_edges_.size() && !CheckSameSegmentEdges(start_edge, border_edges_[0]));
		}
#endif
	}

	bool HWPlane::GenerateInitial2DSortedPnts()
	{
		std::cout << "the edge pnts number is: " << sorted_border_edges_2d_.size() << std::endl;
		if (sorted_border_edges_2d_.empty())
		{
			return false;
		}
		for (int i = 0; i < sorted_border_edges_2d_.size(); ++i)
		{
			Point_2d start_pnt = sorted_border_edges_2d_[i].start();
			sorted_border_pnts_2d_.emplace_back(start_pnt);
			edge_pnts_pos_.emplace_back(make_float2(start_pnt.x(), start_pnt.y()));		
		}
		edge_pnts_pos_num_ = edge_pnts_pos_.size();
		
		////test 保存这些顶点
		//std::ofstream fh("D:\\room_sketch\\data\\huawei_data\\huawei_outdoor_test\\outdoor_sorted_edges_1.obj");
		//for (int i = 0; i < sorted_border_pnts_2d_.size(); ++i)
		//{
		//	fh << "v " << sorted_border_pnts_2d_[i].x() << " " << sorted_border_pnts_2d_[i].y() <<" "<< "0.0" << std::endl;
		//}
		///*fh << "f ";
		//for (int i = 0; i < sorted_border_pnts_2d_.size(); ++i)
		//{
		//	fh << i + 1 << " ";
		//}*/
		//fh.close();
		//std::cout << "end save the sorted edge pnts!!" << std::endl;
		//system("pause");
		//end test
		return true;
	}

	void HWPlane::ComputeSortedVerticesListWeights()
	{
		if (sorted_border_pnts_2d_.size() < 3)
		{
#ifdef LIULINGFEI
			std::cout << "no close edge existed!!!" << std::endl;
#endif // LIULINGFEI
			return;
		}

		int pre_idx = 0;
		//int next_idx = 
		for (int i = 0; i < sorted_border_pnts_2d_.size(); ++i)
		{
			int pre_idx = i - 1;
			if (pre_idx < 0)
				pre_idx = sorted_border_pnts_2d_.size() - 1;
			int next_idx = i + 1;
			if (next_idx >= sorted_border_pnts_2d_.size())
				next_idx = 0;

			float2 pre_pnt = make_float2(sorted_border_pnts_2d_[pre_idx].x(), sorted_border_pnts_2d_[pre_idx].y());
			float2 pnt = make_float2(sorted_border_pnts_2d_[i].x(), sorted_border_pnts_2d_[i].y());
			float2 next_pnt = make_float2(sorted_border_pnts_2d_[next_idx].x(), sorted_border_pnts_2d_[next_idx].y());

			float3 f_coeff = Compute2DPlaneLineFunction(pre_pnt, next_pnt);
			float pnt_edge_dist = ComputeDistTo2DLine(pnt, f_coeff);
			sorted_border_pnts_weight_2d_.emplace_back(pnt_edge_dist);
		}

		//test
		for (int i = 0; i < sorted_border_pnts_weight_2d_.size(); ++i)
		{
			std::cout << "the list weight " << i << ": " << sorted_border_pnts_weight_2d_[i] << std::endl;
		}
		//end test
	}

	float HWPlane::ComputeSortedPntsAvarageWeight(const std::vector<float>& pnts_weight)
	{
		float sum = 0; 
		for (int i = 0; i < sorted_border_pnts_weight_2d_.size(); ++i)
		{
			sum += sorted_border_pnts_weight_2d_[i];
		}
		return (sum / sorted_border_pnts_weight_2d_.size());
	}

	void HWPlane::Generate2DPloygons()
	{
		UpdateRefinedSortedBorderPnts2D();

		int min_pnts_num = 30;

		int iter_num = 10;
		for (int count = 0; count < iter_num; ++count)
		{

			std::vector<bool> to_delete_flag;
			to_delete_flag.resize(sorted_border_pnts_weight_2d_.size());
			for (int i = 0; i < to_delete_flag.size(); ++i)
			{
				to_delete_flag[i] = false;
			}

			//averge
			float average_dist = ComputeSortedPntsAvarageWeight(sorted_border_pnts_weight_2d_);

			for (int i = 0; i < sorted_border_pnts_weight_2d_.size(); ++i)
			{
				if (sorted_border_pnts_weight_2d_[i] < average_dist)
				{
					to_delete_flag[i] = true;
					if (i != 0 && to_delete_flag[i - 1])
						to_delete_flag[i] = false;
					if((i == (sorted_border_pnts_weight_2d_.size() - 1)) && to_delete_flag[0])
						to_delete_flag[i] = false;
				}
			}

			std::cout << "the sorted_border_pnts_2d_ size is: " << sorted_border_pnts_2d_.size() << std::endl;

			//输出这些被删除的边
			std::vector<Point_2d> remain_vertices;
			std::vector<float2> vertices;
			/*std::list<Segment>::iterator remain_iter = remain_edges.begin();
			for (; remain_iter != remain_edges.end(); ++remain_iter)
			{
				float2 tmp_vertex = make_float2(remain_iter->start().x(), remain_iter->start().y());
				vertices.emplace_back(tmp_vertex);
			}*/

			for (int i = 0; i < sorted_border_pnts_2d_.size(); ++i)
			{
				if (!to_delete_flag[i])
				{
					remain_vertices.emplace_back(sorted_border_pnts_2d_[i]);
					vertices.emplace_back(make_float2(sorted_border_pnts_2d_[i].x(), sorted_border_pnts_2d_[i].y()));
				}
			}
#if 1
			//test 保存这些顶点
			//std::string path_name = "D:\\vc_project_xht\\back_up\\4\\remain_edge_refine" + std::to_string(count) + ".obj";
			//std::string path_name = "D:\\vc_project_xht\\back_up\\test_for_plane\\remain_edge_refine" + std::to_string(count) + ".obj";
			//std::ofstream fh(path_name);
			//for (int i = 0; i < vertices.size(); ++i)
			//{
			//	fh << "v " << vertices[i].x << " " << vertices[i].y << " "<< "0.0" << std::endl;
			//}
			//fh.close();
#ifdef LIULINGFEI
			std::cout << "end save the refine edge pnts!!" << std::endl;
#endif // LIULINGFEI
			//end test
#endif			
			//更新原来的顶点
			sorted_border_pnts_2d_.clear();
			for (int i = 0; i < remain_vertices.size(); ++i)
			{
				sorted_border_pnts_2d_.emplace_back(remain_vertices[i]);
			}
			//system("pause");
			UpdateRefinedSortedBorderPnts2D();

			//当顶点少多一定数量，可一直接跳出去
			if (min_pnts_num > sorted_border_pnts_2d_.size())
				break;
		}

#if 0
		pcl::PointCloud<pcl::PointXY>::Ptr cloud(new pcl::PointCloud<pcl::PointXY>);
		for (int i = 0; i < edge_pnts_pos_.size(); ++i)
		{
			pcl::PointXY tmp_point;
			tmp_point.x = edge_pnts_pos_[i].x;
			tmp_point.y = edge_pnts_pos_[i].y;
			cloud->points.emplace_back(tmp_point);
		}
		pcl::octree::OctreePointCloudSearch<pcl::PointXY> octree(0.01);
		octree.setInputCloud(cloud);
		octree.addPointsFromInputCloud(); //构建octree
		std::vector<int> point_idx_vec;	//存邻近像素的索引
		//
#endif

	}

	void HWPlane::GenerateSampleWorldCoordPnts()
	{
		//先转化为平面坐标
		std::vector<Point2> plane_coord_pnts;
		for (int i = 0; i < sample_polygon_vertices_.size(); ++i)
		{
			Point2 pnt = sample_polygon_vertices_[i];
			cv::Point2f img_pnt(pnt.x(), pnt.y());
			Point2 plane_pnt = ImageCoordPnt2PlaneCoordPnt(img_pnt);
			plane_coord_pnts.emplace_back(plane_pnt);
		}

		//生成和src一样坐标系的顶点
		for (int i = 0; i < plane_coord_pnts.size(); ++i)
		{
			Eigen::Vector4f pnt(plane_coord_pnts[i][0], plane_coord_pnts[i][1], 0.0, 1.0);
			Eigen::Vector4f transform_pnt = plane_to_world_*pnt;
			sample_world_coord_vertices_.push_back(make_float3(transform_pnt[0], transform_pnt[1], transform_pnt[2]));
		}
	}

	void HWPlane::MergeSampleAndSrcPnts()
	{
		for (int i = 0; i < pnts_pos_.size(); ++i)
		{
			merged_pnts_pos_.emplace_back(pnts_pos_[i]);
		}

		if (HasNormals())
		{
			for (int i = 0; i < pnts_pos_.size(); ++i)
			{
				merged_pnts_normal_.emplace_back(pnts_normal_[i]);
			}
		}
		
		if (HasColors())
		{
			for (int i = 0; i < pnts_pos_.size(); ++i)
			{
				merged_pnts_normal_.emplace_back(pnts_normal_[i]);
			}

		}

		std::cout << "the average normal is: " << average_pnts_normal_.x() << " " << average_pnts_normal_.y() << " " << average_pnts_normal_.z() << std::endl;

		ccColor::Rgb col(255, 255, 255);
		uchar3 tmp_color = make_uchar3(col.r, col.g, col.b);
		for (int i = 0; i < sample_world_coord_vertices_.size(); ++i)
		{
			merged_pnts_pos_.emplace_back(sample_world_coord_vertices_[i]);
			
			//将平面方程的法向量加入进去
			if(HasNormals())
				merged_pnts_normal_.emplace_back(make_float3(average_pnts_normal_.x(), average_pnts_normal_.y(), average_pnts_normal_.z()));

			if(HasColors())
				merged_pnts_color_.emplace_back(tmp_color);
		}

		std::cout << "the merge pnts is: " << pnts_pos_.size() << std::endl;
		std::cout << "the merged pnts normal is: " << pnts_normal_.size() << std::endl;
		std::cout << "the merged pnt color is: " << pnts_color_.size() << std::endl;
	}

	//判断是否是同一个边
	bool HWPlane::CheckSameSegmentEdges(Segment& a, Segment& b)
	{
		Point_2d a_start = a.start();
		Point_2d a_end = a.end();
		Point_2d b_start = b.start();
		Point_2d b_end = b.end();

		if (CheckSameSegmentPnts(a_start, b_start) && CheckSameSegmentPnts(a_end, b_end))
			return true;
		else if (CheckSameSegmentPnts(a_start, b_end) && CheckSameSegmentPnts(a_end, b_start))
			return true;
		else
			return false;
	}

	//判断是否是同一个顶点
	bool HWPlane::CheckSameSegmentPnts(Point_2d& a, Point_2d& b)
	{
		bool flag = false;
		if (std::abs(a.x() - b.x()) < 1e-6 && std::abs(a.y() - b.y()) < 1e-6)
		{
			flag = true;
		}
		return flag;
	}

	bool HWPlane::CheckThresholdTwoPnts(double threshold, Point_2d& a, Point_2d& b)
	{
		bool flag = false;
		if (std::sqrtf((a.x() - b.x()) * (a.x() - b.x()) + (a.x() - b.x()) * (a.x() - b.x())) < threshold)
		{
			flag = true;
		}
		return flag;
	}

	//---------------------------------------end edge-----------------//
	//ax + by = c
	float3 HWPlane::Compute2DPlaneLineFunction(const float2& a, const float2& b)
	{
		if (std::abs(a.x - b.x) < std::abs(a.y - b.y))
		{
			float k = (a.x - b.x) / (a.y - b.y);
			float b = a.x - k*a.y;
			return (make_float3(1.0, -k, b));
		}
		else if(std::abs(a.x - b.x) >= std::abs(a.y - b.y))
		{
			float k = (a.y - b.y) / (a.x - b.x);
			float b = a.y - k*a.x;
			return (make_float3(-k, 1.0, b));
		}
		return (make_float3(0.0, 0.0, 0.0));
	}

	float HWPlane::ComputeVerticesDistIn2DPlane(const float2& a, const float2& b)
	{
		return(std::sqrtf((a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y)));
	}

	float HWPlane::ComputeDistTo2DLine(const float2& a, const float3& f)
	{
		if (std::sqrtf(f.x*f.x + f.y*f.y) < 1e-8)
			return 10000.0;
		float dist = std::abs(f.x*a.x + f.y*a.y - f.z) / std::sqrtf(f.x*f.x + f.y*f.y);
		return dist;
	}

	float HWPlane::Compute2DVerticesCrossValue(const float2& a, const float2& b)
	{
		return std::abs(a.x*b.x + a.y*b.y) / std::sqrtf(a.x * a.x + a.y * a.y + b.x * b.x + b.y*b.y);
	}

#if HW_DEBUG
	void HWPlane::SavePlaneCoordPntsIntoOBJ(const std::string path)
	{
		std::ofstream out_fh(path);
		if (pnts_pos_.size() == 0)
		{
			printf("none vertices!\n");
			return;
		}
		for (int i = 0; i < pnts_pos_.size(); ++i)
		{
			out_fh << "v " << pnts_pos_[i].x << " " << pnts_pos_[i].y << " " << pnts_pos_[i].z << std::endl;
		}
		for (int i = 0; i < pnts_normal_.size(); ++i)
		{
			out_fh << "vn " << pnts_normal_[i].x << " " << pnts_normal_[i].y
				<< " " << pnts_normal_[i].z << std::endl;
		}
		out_fh.close();
		std::cout << "end save the plane coord pnts!!" << std::endl;
	}

	void HWPlane::SavePlaneInitialEdgePntsIntoOBJ(const std::string& path)
	{
		std::ofstream out_fh(path);
		if (sbp_to_pnts_pos.size() == 0)
		{
			printf("none vertices!\n");
			return;
		}
		for (int i = 0; i < sbp_to_pnts_pos.size(); ++i)
		{
			if (sbp_to_pnts_pos[i] != -1)
			{
				out_fh << "v " << plane_coord_pos_[sbp_to_pnts_pos[i]].x << " " << plane_coord_pos_[sbp_to_pnts_pos[i]].y
					<< " " << plane_coord_pos_[sbp_to_pnts_pos[i]].z << std::endl;

				out_fh << "vn " << plane_coord_pnts_normal_[sbp_to_pnts_pos[i]].x << " " << plane_coord_pnts_normal_[sbp_to_pnts_pos[i]].y
					<< " " << plane_coord_pnts_normal_[sbp_to_pnts_pos[i]].z << std::endl;
			}
		}
		out_fh.close();
	}

	void HWPlane::SavePlaneRefinedEdgePntsIntoOBJ(const std::string& path)
	{
		std::ofstream out_fh(path);
		if (sbp_to_pnts_pos.size() == 0)
		{
#ifdef LIULINGFEI
			printf("none vertices!\n");
#endif
			return;
		}
		for (int i = 0; i < sbp_to_pnts_pos.size(); ++i)
		{
			if (sbp_to_pnts_pos[i] != -1)
			{
				out_fh << "v " << plane_coord_pos_[sbp_to_pnts_pos[i]].x << " " << plane_coord_pos_[sbp_to_pnts_pos[i]].y
					<< " " << plane_coord_pos_[sbp_to_pnts_pos[i]].z << std::endl;

				out_fh << "vn " << plane_coord_pnts_normal_[sbp_to_pnts_pos[i]].x << " " << plane_coord_pnts_normal_[sbp_to_pnts_pos[i]].y
					<< " " << plane_coord_pnts_normal_[sbp_to_pnts_pos[i]].z << std::endl;
			}
		}
		out_fh.close();
	}
#endif

	//--------------------------least fit plane--------------------//
	float HWPlane::GetDistToPlane(float3 pt)
	{
		return ((coeff_.x * pt.x + coeff_.y * pt.y + coeff_.z * pt.z + coeff_.w) /
			std::sqrtf(coeff_.x*coeff_.x + coeff_.y*coeff_.y + coeff_.z*coeff_.z));
	}

	float3 HWPlane::GetAPtProjectedToPlane(float3 pt)
	{
		float k = -1 * GetDistToPlane(pt);
		float x = pt.x + k * coeff_.x;
		float y = pt.y + k * coeff_.y;
		float z = pt.z + k * coeff_.z;
		return (make_float3(x, y, z));
	}

	float3 HWPlane::ComputeAverageVertexPos()
	{
		float3 sum = make_float3(0.0, 0.0, 0.0);
		float3 averge_pos = make_float3(0.0, 0.0, 0.0);
		int count = static_cast<int>(pnts_pos_origin_.size());
		for (int i = 0; i < pnts_pos_origin_.size(); ++i)
		{		
			sum.x += pnts_pos_origin_[i].x;
			sum.y += pnts_pos_origin_[i].y;
			sum.z += pnts_pos_origin_[i].z;
		}
		averge_pos.x = sum.x / count;
		averge_pos.y = sum.y / count;
		averge_pos.z = sum.z / count;
		return averge_pos;
	}

	Eigen::Vector3f HWPlane::ComputeAverageVertexNormal()
	{
		float3 sum = make_float3(0.0, 0.0, 0.0);
		Eigen::Vector3f averge_normal;
		int count = static_cast<int>(pnts_normal_.size());
		for (int i = 0; i < pnts_normal_.size(); ++i)
		{
			sum.x += pnts_normal_[i].x;
			sum.y += pnts_normal_[i].y;
			sum.z += pnts_normal_[i].z;
		}
		if (count == 0)
		{
			return Eigen::Vector3f(0.0, 0.0, 0.0);
		}
		averge_normal.x() = sum.x / count;
		averge_normal.y() = sum.y / count;
		averge_normal.z() = sum.z / count;
		averge_normal.normalize();
        //printf("averge_normal = (%f, %f, %f)\n", averge_normal[0], averge_normal[1], averge_normal[2]);
		return averge_normal;
	}

	void HWPlane::SetSplitFlag(int split_flag)
	{
		splited_label_ = split_flag;
	}

	void HWPlane::SetSplitStructure(std::vector<HW::HWPlane*> model_planes,
		std::vector<int>& related_idxs)
	{
		model_planes_ = model_planes;
		related_poly_idxs_ = related_idxs;
	}

	//void HWPlane::BuildSplitBasedStructrue()
	//{
	//	//
	//	if (splited_label_ == -1
	//		|| related_poly_idxs_.empty()
	//		|| related_poly_idxs_.empty())
	//	{
	//		std::cout << "BuildSplitBasedStructrue: error " << std::endl;
	//		return;
	//	}
	//	HWPlane* src_plane = model_planes_[splited_label_];
	//	std::vector<HWPlane*> tgt_planes;
	//	for (int i = 0; i < related_poly_idxs_.size(); ++i)
	//	{
	//		HWPlane* tmp_plane = model_planes_[related_poly_idxs_[i]];
	//		tgt_planes.emplace_back(tmp_plane);
	//	}
	//	//就在这里切开
	//	if (related_poly_idxs_.size() == 0)
	//	{
	//		return;
	//	}
	//	Eigen::Vector3f ldir3d0, lpnt3d0;
	//	GetExpandTgtIntersectionLine(related_poly_idxs_[0], ldir3d0, lpnt3d0);
	//	Eigen::Vector2i tgt_pnts_lr_idx;	//相对于tgt的宽度的idx
	//	//计算两个端点
	//	model_planes_[related_poly_idxs_[0]]->ComputeTwoPointsIdxCorrespondingLine(ldir3d0, lpnt3d0, tgt_pnts_lr_idx);
	//	float2 tprjlpnt2d = model_planes_[related_poly_idxs_[0]]->ProjToLine3D(tgt_pnts_lr_idx[0], ldir3d0, lpnt3d0);
	//	float2 tprjrpnt2d = model_planes_[related_poly_idxs_[0]]->ProjToLine3D(tgt_pnts_lr_idx[1], ldir3d0, lpnt3d0);
	//	Eigen::Vector3f tprjlpnt3d, tprjrpnt3d;
	//	model_planes_[related_poly_idxs_[0]]->Pnt2d2Pnt3D(Eigen::Vector2f(tprjlpnt2d.x, tprjlpnt2d.y), tprjlpnt3d);
	//	model_planes_[related_poly_idxs_[0]]->Pnt2d2Pnt3D(Eigen::Vector2f(tprjrpnt2d.x, tprjrpnt2d.y), tprjrpnt3d);
	//	Eigen::Vector2f sproj_lpnt2d, sproj_rpnt2d;
	//	Pnt3d2Pnt2D(tprjlpnt3d, sproj_lpnt2d);
	//	Pnt3d2Pnt2D(tprjrpnt3d, sproj_rpnt2d);	//这是tgt 线段的左右两个端点
	//	//计算它们的穿插这个polygon的中线
	//	Eigen::Vector2f sm_pnt0, sm_dir0;
	//	ComputeVerticalLFunctionFromTwoPnts2D(sproj_lpnt2d, sproj_rpnt2d, sm_pnt0, sm_dir0);
	//	/*Eigen::Vector2f wlpnt0, wldir0;
	//	ComputeWorldLf2PlaneLf(ldir3d0, lpnt3d0, wldir0, wlpnt0);
	//	sm_dir0[0] = -wldir0[1];
	//	sm_dir0[1] = wldir0[0];*/
	//	std::cout << "the sm_pnt0, sm_dir0: " << sm_pnt0[0] << " " << sm_pnt0[1] << ", "
	//		<< sm_dir0[0] << " " << sm_dir0[1] << std::endl;
	//	//sm_pnt, sm_dir，这是中线，它可以 进行排序，其中sm_pnt，为0.
	//	if (std::abs(sm_dir0[0]) < 1e-6 && std::abs(sm_dir0[1]) < 1e-6)
	//		return;
	//	if (related_poly_idxs_.size() == 1)
	//	{
	//		//只有一个切线，直接表示几种切开方式
	//		//判断是否和直线相切
	//		Eigen::Vector2f ldir2d, lpnt2d;
	//		ComputeWorldLf2PlaneLf(ldir3d0, lpnt3d0, ldir2d, lpnt2d);
	//		//使用ldir2d, lpnt2d对原始的polygon进行剪切
	//	}
	//	else if (related_poly_idxs_.size() == 2)
	//	{
	//		
	//		Eigen::Vector3f ldir3d1, lpnt3d1;
	//		GetExpandTgtIntersectionLine(related_poly_idxs_[1], ldir3d1, lpnt3d1);
	//		
	//		std::vector<float> dist2smpnt_vec;
	//		dist2smpnt_vec.emplace_back(0.0);
	//	}
	//	else
	//	{
	//		for (int i = 0; i < related_poly_idxs_.size(); ++i)
	//		{
	//			Eigen::Vector3f ldir3d, lpnt3d;
	//		}
	//	}
	//}

	bool HWPlane::BuildAroundPolygonPnts2d(std::vector<std::vector<int> >& polygons_adjs)
	{
		if (splited_label_ == -1 || !related_param_flag)
		{
			std::cerr << "wrong BuildAroundPolygonPnts2d..." << std::endl;
			return false;
		}
		if (!resorted_sor_corner_flag_)
		{
			SortCornerPnts();
		}
		int related_lines_num = related_poly_idxs_.size();
		std::vector<int> resorted_related_poly_idxs;	//保存已经排序的polygon
		/*std::string filename;
		std::vector<Eigen::Vector2f> cornerpnts;
		for (int i = 0; i < corner_pnts_.size(); ++i)
		{
			cornerpnts.emplace_back(Eigen::Vector2f(corner_pnts_[i].x, corner_pnts_[i].y));
		}
		cv::Mat src_image;
		DrawImageFromCornerPnts2d(cornerpnts, src_image, filename);*/

		//设置polygon方向
		SetAdjLinesDirect();
		//设置polygon的方向
		if (!SetRelatedIdxAroundSrcPoly())
			return false;
		//测试上述的sorted 的 polygon是否正确
		
		//这个polygon方向很重要
		std::vector<int> all_sorted_poly_idxs_tmp;
		for (int i = 0; i < sorted_around_related_poly_idxs_.size(); ++i)
		{
			all_sorted_poly_idxs_tmp.emplace_back(related_poly_idxs_[sorted_around_related_poly_idxs_[i]]);
		}
		std::cerr << "the all_sorted_poly_idxs_tmp: " << std::endl;
		ScoutPlaneVecIdxs(all_sorted_poly_idxs_tmp);

		//正确，处理下一步，通过三个polygon之间的关系，完成拼接
		std::vector<std::pair<bool, Eigen::Vector2f> > adj_cross_pnts; //保存三个polygon之间的关系
		adj_cross_pnts.resize(all_sorted_poly_idxs_tmp.size());
		bool closed_poly_flag = true;
		for (int i = 0, j = 1; i < sorted_around_related_poly_idxs_.size(); ++i, ++j)
		{
			//
			i = i % sorted_around_related_poly_idxs_.size();
			j = j % sorted_around_related_poly_idxs_.size();
			int idxi = all_sorted_poly_idxs_tmp[i];
			int idxj = all_sorted_poly_idxs_tmp[j];
			std::cerr << "idxi, idxj: " << idxi << ", " << idxj << std::endl;
			if (CheckTwoPolysAdj(idxi, idxj, polygons_adjs))
			{
				//三个平面sor,j,i合成一个顶点
				//替补的方案,在sor的平面上两个直线i,j相交为一个直线
				Eigen::Vector2f crosspnt;
				std::cerr << "sorted_around_related_poly_idxs_ i , j: " << sorted_around_related_poly_idxs_[i] << ", " << sorted_around_related_poly_idxs_[j] << std::endl;
				Eigen::Vector2f ls = related_poly_corners_widths_2d_[sorted_around_related_poly_idxs_[i]].first;
				Eigen::Vector2f le = related_poly_corners_widths_2d_[sorted_around_related_poly_idxs_[i]].second;
				Eigen::Vector2f ts = related_poly_corners_widths_2d_[sorted_around_related_poly_idxs_[j]].first;
				Eigen::Vector2f te = related_poly_corners_widths_2d_[sorted_around_related_poly_idxs_[j]].second;
				//std::cerr << "ls, le: " << ls[0] << " " << ls[1] << ", " << le[0] << " " << le[1] << std::endl;
				//std::cerr << "ts, te: " << ts[0] << " " << ts[1] << ", " << te[0] << " " << te[1] << std::endl;
				if(ComputeTwoLinesCrosspnt(ls, le, ts, te, crosspnt))
				{
					std::cerr << "idxi, idxj: ls le cross pnt: " << crosspnt[0] << ", " << crosspnt[1] << std::endl;
					std::pair<bool, Eigen::Vector2f> tmp_cross = std::make_pair(true, crosspnt);
					adj_cross_pnts[i] = tmp_cross;
				}
				else
				{
					std::pair<bool, Eigen::Vector2f> tmp_cross = std::make_pair(false, Eigen::Vector2f(0, 0));
					adj_cross_pnts[i] = tmp_cross;
					closed_poly_flag = false;
				}
			}
			else
			{
				std::pair<bool, Eigen::Vector2f> tmp_cross = std::make_pair(false, Eigen::Vector2f(0, 0));
				adj_cross_pnts[i] = tmp_cross;
				closed_poly_flag = false;
			}
		}
		//通过adj_cross_pnts构建新的polygon
		//非常重要。
		if (closed_poly_flag)
		{
			//直接通过polygon替换的方法，构成新的polygon
			std::cerr << "start to closed polygon expand..." << std::endl;
			//std::cerr << "around_polygon_pnts_new_ num: " << around_polygon_pnts_new_.size() << std::endl;
			std::cerr << "the plane to world: \n" << plane_to_world_ << std::endl;
			for (int i = 0; i < adj_cross_pnts.size(); ++i)
			{
				around_polygon_pnts_new_.emplace_back(adj_cross_pnts[i].second);
			}
			std::cerr << "around_polygon_pnts_new_ num: " << around_polygon_pnts_new_.size() << std::endl;
			UpdateCornerPtsFromNewPolygon(around_polygon_pnts_new_);
		}
		else
		{
			//需要额外处理polygon的顶点，需要用拼接的方法处理，怎么拼接？
			std::cerr << "to do next..." << std::endl;
		}
		return true;
	}

	bool HWPlane::CheckCrossPntsWithPolygon(Eigen::Vector2f& ls, Eigen::Vector2f& le)
	{
		for (int i = 0, j = corner_pnts_.size() - 1; i < corner_pnts_.size(); j = i++)
		{
			Eigen::Vector2f ts = Eigen::Vector2f(corner_pnts_[j].x, corner_pnts_[j].y);
			Eigen::Vector2f te = Eigen::Vector2f(corner_pnts_[i].x, corner_pnts_[i].y);
			Eigen::Vector2f tmp_cross_pnt;
			if (ComputeTwoLineSegsCrossPnt(ls, le, ts, te, tmp_cross_pnt))
				return true;
		}
		return false;
	}

	bool HWPlane::CheckVecFlag(std::vector<bool>& flags)
	{
		bool flag = false;
		for (int i = 0; i < flags.size(); ++i)
		{
			if (!flags[i])
			{
				return flag;
			}
		}
		return !flag;
	}

	void HWPlane::SetAdjLinesDirect()
	{
		int lineseg_num = related_poly_corners_widths_2d_.size();
		int related_idx = related_poly_idxs_.size();
		//related_poly_idxs_它附近关联的src polygon的顶点，用于后续的操作，形成polygon圈
		related_polys_around_pnts_idx_.resize(lineseg_num);
		related_polys_around_pnts_pos_.resize(lineseg_num);	//表示它的交点
		if (lineseg_num == related_idx)
		{
			for (int i = 0; i < lineseg_num; ++i)
			{
				std::cerr << "lineseg_num->current idx: " 
					<< related_poly_idxs_[i] << std::endl;
				Eigen::Vector2f ls = related_poly_corners_widths_2d_[i].first;
				Eigen::Vector2f le = related_poly_corners_widths_2d_[i].second;
				std::cerr << "ls, le: " << ls[0] << " " << ls[1] << ", " << le[0] << " " << le[1] << std::endl;
				if ((le - ls).norm() < 1e-6)
				{
					continue;
				}
				//构建ls 和 le的方程和polygon进行求交，然后获取它们的方向
				Eigen::Vector2f ldir2d = (le - ls).normalized();
				Eigen::Vector2f nldir2d = Eigen::Vector2f(-ldir2d[1], ldir2d[0]);
				//通过src polygon和polygon的拼接线投影
				Eigen::Vector3f ldir3di, lpnt3di;
				GetExpandTgtIntersectionLine(related_poly_idxs_[i], ldir3di, lpnt3di);
				//转化到当前的坐标系中
				//Eigen::Vector2f ldir2d, lpnt2d;
				//ComputeWorldLf2PlaneLf(ldir3di, lpnt3di, ldir2d, lpnt2d);
				//方法弃用
				////获取离polygon较为近的半圆，判断它的方向，这个方向还是要重新处理一下
				//std::vector<int> idxs_move;
				//ComputePolygonMoveIdxs2Line(ldir3di, lpnt3di, idxs_move);
				////判断idxs_move的方向
				//ls和le跟polygon是否有交叉点

				Eigen::Vector2i s2e_idx = Eigen::Vector2i(-1, -1);
				Eigen::Vector2i s2e_idx_other = Eigen::Vector2i(-1, -1);
				Eigen::Vector2f ls_cross_pnt;
				Eigen::Vector2i ls_cross_idx;
				Eigen::Vector2f le_cross_pnt;
				Eigen::Vector2i le_cross_idx;
				if (ComputeSENearestLineCrossPnts(ldir2d, ls,
					ls_cross_idx, ls_cross_pnt, le_cross_idx, le_cross_pnt))
				{
					std::cerr << "ComputeSENearestLineCrossPnts: true" << std::endl;
					//找不到交线的话，原始的polygon投回去
					Eigen::Vector2i lr_pnts_idx(-1, -1);
					Eigen::Vector2f lpnt, rpnt;
					if (!ComputeLeftRightPointsPosLine2D(ldir2d, ls, lr_pnts_idx, lpnt, rpnt))
					{
						//check the line direct with polygon
						//一般情况是line在
						std::cerr << "lines lr is not existed..." << std::endl;
						continue;
					}
					float lr_dist = (rpnt - lpnt).norm();
					float line_dist = (le - ls).norm();
					float tmp_cross_dist = (le_cross_pnt - ls_cross_pnt).norm();
					std::cerr << "lr_dist, line_dist: " << lr_dist << ", " << line_dist << std::endl;
					if (lr_dist > line_dist)
					{
						//表示这个线段比polygon要短
						if (ComputeNearestLineCrossPnts(nldir2d, ls, ls_cross_idx, ls_cross_pnt)
							&& ComputeNearestLineCrossPnts(nldir2d, le, le_cross_idx, le_cross_pnt))
						{
							//表示它是两个交线
							//计算两个交点组成的方向
							Eigen::Vector2f cross_dir = le_cross_pnt - ls_cross_pnt;
							//获取交线在polygon方向的方向。它怎么获取？通过ls_cross_idx和le_cross_idx来计算
							//其中ls_cross_idx[0]->ls_cross_idx[1]就是polygon方向,其中le_cross_idx[0]->le_cross_idx[1]就是polygon方向
							//怎么构建这个方向,因为它是环形，所以它有两个方向，需要分开处理。
							//有两个方向
							if (ls_cross_idx == le_cross_idx)
							{
								//表示它们在同一个线段上
								//获取这个线段上的两个端点
								Eigen::Vector2f src_poly_ls = Eigen::Vector2f(corner_pnts_[ls_cross_idx[0]].x, corner_pnts_[ls_cross_idx[0]].y);
								Eigen::Vector2f src_poly_le = Eigen::Vector2f(corner_pnts_[ls_cross_idx[1]].x, corner_pnts_[ls_cross_idx[1]].y);
								Eigen::Vector2f src_poly_dir = le - ls;
								if (src_poly_dir.dot(cross_dir) < 0)
								{
									//换一个方向
									related_poly_corners_widths_2d_[i].first = le;
									related_poly_corners_widths_2d_[i].second = ls;
								}
							}
							else
							{
								//有两个split polygon idxs
								s2e_idx[0] = ls_cross_idx[1];
								s2e_idx[1] = le_cross_idx[0];
								//split polygon 
								std::vector<int> hf_poly_idxs0;
								std::cerr << "s2e_idx: " << s2e_idx[0] << " " << s2e_idx[1] << std::endl;
								ConstructHalfPolyIdxs(s2e_idx, hf_poly_idxs0);
								std::cerr << "hf_poly_idxs0: " << std::endl;
								ScoutPlaneVecIdxs(hf_poly_idxs0);

								std::vector<int> hf_poly_idxs1;
								s2e_idx_other[0] = le_cross_idx[1];
								s2e_idx_other[1] = ls_cross_idx[0];
								std::cerr << "s2e_idx_other: " << s2e_idx_other[0] << " " << s2e_idx_other[1] << std::endl;
								ConstructHalfPolyIdxs(s2e_idx_other, hf_poly_idxs1);
								std::cerr << "hf_poly_idxs1: " << std::endl;
								ScoutPlaneVecIdxs(hf_poly_idxs1);
								
								std::vector<Eigen::Vector2f> hf_poly_pnts0;
								std::vector<Eigen::Vector2f> hf_poly_pnts1;
								hf_poly_pnts0.emplace_back(ls_cross_pnt);
								for (int j = 0; j < hf_poly_idxs0.size(); ++j)
								{
									Eigen::Vector2f tmp_pnt = Eigen::Vector2f(corner_pnts_[hf_poly_idxs0[j]].x,
										corner_pnts_[hf_poly_idxs0[j]].y);
									hf_poly_pnts0.emplace_back(tmp_pnt);
								}
								hf_poly_pnts0.emplace_back(le_cross_pnt);
								hf_poly_pnts1.emplace_back(le_cross_pnt);
								for (int j = 0; j < hf_poly_idxs1.size(); ++j)
								{
									Eigen::Vector2f tmp_pnt = Eigen::Vector2f(corner_pnts_[hf_poly_idxs1[j]].x,
										corner_pnts_[hf_poly_idxs1[j]].y);
									hf_poly_pnts1.emplace_back(tmp_pnt);
								}
								hf_poly_pnts1.emplace_back(ls_cross_pnt);
								//处理两个polygon，
								Eigen::Vector2f mean_pnt = (ls + le) / 2;
								Eigen::Vector2f poly0_cross, poly1_cross;
								if (ComputeLine2NearestHalfPolyCrossPnt(nldir2d, mean_pnt, hf_poly_pnts0, poly0_cross)
									&& ComputeLine2NearestHalfPolyCrossPnt(nldir2d, mean_pnt, hf_poly_pnts1, poly1_cross))
								{
									float poly0_dist = (poly0_cross - mean_pnt).norm();
									float poly1_dist = (poly1_cross - mean_pnt).norm();
									if (poly0_dist < poly1_dist)
									{
										related_polys_around_pnts_idx_[i] = hf_poly_idxs0;
										related_polys_around_pnts_pos_.emplace_back(std::make_pair(ls_cross_pnt, le_cross_pnt));
									}
									else
									{
										//换一个方向
										Eigen::Vector2f tmp2dpnt = related_poly_corners_widths_2d_[i].first;
										related_poly_corners_widths_2d_[i].first = related_poly_corners_widths_2d_[i].second;
										related_poly_corners_widths_2d_[i].second = tmp2dpnt;
										related_polys_around_pnts_idx_[i] = hf_poly_idxs1;
										related_polys_around_pnts_pos_.emplace_back(std::make_pair(le_cross_pnt, ls_cross_pnt));
									}
								}
								else
								{
									std::cerr << "poly0_cross, poly1_cross: " << poly0_cross[0] << " " << poly0_cross[1] << ", "
										<< poly1_cross[0] << " " << poly1_cross[1] << std::endl;
									std::cerr << "no polygon cross existed..." << std::endl;
									continue;
								}
							}
						}
						else if (ComputeNearestLineCrossPnts(nldir2d, ls, ls_cross_idx, ls_cross_pnt)
							&& !ComputeNearestLineCrossPnts(nldir2d, le, le_cross_idx, le_cross_pnt))
						{
							//处理这个polygon，需要处理这个polygon
							float lp2le = (lpnt - le).norm();
							float rp2le = (rpnt - le).norm();
							int s_idx = -1;
							if (lp2le < rp2le)
							{
								//有两个split polygon idxs
								s_idx = lr_pnts_idx[0];
								//if (s_idx == ls_cross_idx[0])
								//{
								//	//反向,否者获取方向再验证，后面处理
								//	//换一个方向
								//	std::vector<int> hf_poly_idxs1 = std::vector<int> { ls_cross_idx[0] , ls_cross_idx[1] };
								//	Eigen::Vector2f tmp2dpnt = related_poly_corners_widths_2d_[i].first;
								//	related_poly_corners_widths_2d_[i].first = related_poly_corners_widths_2d_[i].second;
								//	related_poly_corners_widths_2d_[i].second = tmp2dpnt;
								//	related_polys_around_pnts_idx_[i] = hf_poly_idxs1;
								//	related_polys_around_pnts_pos_.emplace_back(std::make_pair(lpnt, ls_cross_pnt));
								//}
								//else
								//{
								//	//split polygon 
								//	std::vector<int> hf_poly_idxs0;
								//	s2e_idx[0] = ls_cross_idx[1];
								//	s2e_idx[1] = s_idx;	//()
								//	std::cerr << "s2e_idx: " << s2e_idx[0] << " " << s2e_idx[1] << std::endl;
								//	ConstructHalfPolyIdxs(s2e_idx, hf_poly_idxs0);
								//	std::cerr << "hf_poly_idxs0: " << std::endl;
								//	ScoutPlaneVecIdxs(hf_poly_idxs0);
								//	std::vector<int> hf_poly_idxs1;
								//	s2e_idx_other[0] = s_idx;
								//	s2e_idx_other[1] = ls_cross_idx[0];
								//	std::cerr << "s2e_idx_other: " << s2e_idx_other[0] << " " << s2e_idx_other[1] << std::endl;
								//	ConstructHalfPolyIdxs(s2e_idx_other, hf_poly_idxs1);
								//	std::cerr << "hf_poly_idxs1: " << std::endl;
								//	ScoutPlaneVecIdxs(hf_poly_idxs1);
								//	std::vector<Eigen::Vector2f> hf_poly_pnts0;
								//	std::vector<Eigen::Vector2f> hf_poly_pnts1;
								//	Eigen::Vector2f polyspnt = Eigen::Vector2f(corner_pnts_[s_idx].x, corner_pnts_[s_idx].y);
								//	hf_poly_pnts0.emplace_back(ls_cross_pnt);
								//	for (int j = 0; j < hf_poly_idxs0.size(); ++j)
								//	{
								//		Eigen::Vector2f tmp_pnt = Eigen::Vector2f(corner_pnts_[hf_poly_idxs0[j]].x,
								//			corner_pnts_[hf_poly_idxs0[j]].y);
								//		hf_poly_pnts0.emplace_back(tmp_pnt);
								//	}
								//	hf_poly_pnts0.emplace_back(polyspnt);
								//	hf_poly_pnts1.emplace_back(polyspnt);
								//	for (int j = 0; j < hf_poly_idxs1.size(); ++j)
								//	{
								//		Eigen::Vector2f tmp_pnt = Eigen::Vector2f(corner_pnts_[hf_poly_idxs1[j]].x,
								//			corner_pnts_[hf_poly_idxs1[j]].y);
								//		hf_poly_pnts1.emplace_back(tmp_pnt);
								//	}
								//	hf_poly_pnts1.emplace_back(ls_cross_pnt);
								//	//处理两个polygon，
								//	Eigen::Vector2f mean_pnt = (ls + le) / 2;
								//	Eigen::Vector2f poly0_cross, poly1_cross;
								//	if (ComputeLine2NearestHalfPolyCrossPnt(nldir2d, mean_pnt, hf_poly_pnts0, poly0_cross)
								//		&& ComputeLine2NearestHalfPolyCrossPnt(nldir2d, mean_pnt, hf_poly_pnts1, poly1_cross))
								//	{
								//		float poly0_dist = (poly0_cross - mean_pnt).norm();
								//		float poly1_dist = (poly1_cross - mean_pnt).norm();
								//		if (poly0_dist < poly1_dist)
								//		{
								//			related_polys_around_pnts_idx_[i] = hf_poly_idxs0;
								//			related_polys_around_pnts_pos_.emplace_back(std::make_pair(ls_cross_pnt, polyspnt));
								//		}
								//		else
								//		{
								//			//换一个方向
								//			Eigen::Vector2f tmp2dpnt = related_poly_corners_widths_2d_[i].first;
								//			related_poly_corners_widths_2d_[i].first = related_poly_corners_widths_2d_[i].second;
								//			related_poly_corners_widths_2d_[i].second = tmp2dpnt;
								//			related_polys_around_pnts_idx_[i] = hf_poly_idxs1;
								//			related_polys_around_pnts_pos_.emplace_back(std::make_pair(polyspnt, ls_cross_pnt));
								//		}
								//	}
								//	else
								//	{
								//		std::cerr << "poly0_cross, poly1_cross: " << poly0_cross[0] << " " << poly0_cross[1] << ", "
								//			<< poly1_cross[0] << " " << poly1_cross[1] << std::endl;
								//		std::cerr << "no polygon cross existed..." << std::endl;
								//		continue;
								//	}
								//}
							}
							else
							{
								//有两个split polygon idxs
								s_idx = lr_pnts_idx[1];
							}
							if (s_idx == ls_cross_idx[0])
							{
								//反向,否者获取方向再验证，后面处理
								//换一个方向
								std::vector<int> hf_poly_idxs1 = std::vector<int>{ ls_cross_idx[0] , ls_cross_idx[1] };
								Eigen::Vector2f tmp2dpnt = related_poly_corners_widths_2d_[i].first;
								related_poly_corners_widths_2d_[i].first = related_poly_corners_widths_2d_[i].second;
								related_poly_corners_widths_2d_[i].second = tmp2dpnt;
								related_polys_around_pnts_idx_[i] = hf_poly_idxs1;
								related_polys_around_pnts_pos_.emplace_back(std::make_pair(lpnt, ls_cross_pnt));
							}
							else
							{
								//split polygon 
								std::vector<int> hf_poly_idxs0;
								s2e_idx[0] = ls_cross_idx[1];
								s2e_idx[1] = s_idx;	//()
								std::cerr << "s2e_idx: " << s2e_idx[0] << " " << s2e_idx[1] << std::endl;
								ConstructHalfPolyIdxs(s2e_idx, hf_poly_idxs0);
								std::cerr << "hf_poly_idxs0: " << std::endl;
								ScoutPlaneVecIdxs(hf_poly_idxs0);

								std::vector<int> hf_poly_idxs1;
								s2e_idx_other[0] = s_idx;
								s2e_idx_other[1] = ls_cross_idx[0];
								std::cerr << "s2e_idx_other: " << s2e_idx_other[0] << " " << s2e_idx_other[1] << std::endl;
								ConstructHalfPolyIdxs(s2e_idx_other, hf_poly_idxs1);
								std::cerr << "hf_poly_idxs1: " << std::endl;
								ScoutPlaneVecIdxs(hf_poly_idxs1);

								std::vector<Eigen::Vector2f> hf_poly_pnts0;
								std::vector<Eigen::Vector2f> hf_poly_pnts1;
								Eigen::Vector2f polyspnt = Eigen::Vector2f(corner_pnts_[s_idx].x, corner_pnts_[s_idx].y);
								hf_poly_pnts0.emplace_back(ls_cross_pnt);
								for (int j = 0; j < hf_poly_idxs0.size(); ++j)
								{
									Eigen::Vector2f tmp_pnt = Eigen::Vector2f(corner_pnts_[hf_poly_idxs0[j]].x,
										corner_pnts_[hf_poly_idxs0[j]].y);
									hf_poly_pnts0.emplace_back(tmp_pnt);
								}
								hf_poly_pnts0.emplace_back(polyspnt);
								hf_poly_pnts1.emplace_back(polyspnt);
								for (int j = 0; j < hf_poly_idxs1.size(); ++j)
								{
									Eigen::Vector2f tmp_pnt = Eigen::Vector2f(corner_pnts_[hf_poly_idxs1[j]].x,
										corner_pnts_[hf_poly_idxs1[j]].y);
									hf_poly_pnts1.emplace_back(tmp_pnt);
								}
								hf_poly_pnts1.emplace_back(ls_cross_pnt);
								//处理两个polygon，
								Eigen::Vector2f mean_pnt = (ls + le) / 2;
								Eigen::Vector2f poly0_cross, poly1_cross;
								if (ComputeLine2NearestHalfPolyCrossPnt(nldir2d, mean_pnt, hf_poly_pnts0, poly0_cross)
									&& ComputeLine2NearestHalfPolyCrossPnt(nldir2d, mean_pnt, hf_poly_pnts1, poly1_cross))
								{
									float poly0_dist = (poly0_cross - mean_pnt).norm();
									float poly1_dist = (poly1_cross - mean_pnt).norm();
									if (poly0_dist < poly1_dist)
									{
										related_polys_around_pnts_idx_[i] = hf_poly_idxs0;
										related_polys_around_pnts_pos_.emplace_back(std::make_pair(ls_cross_pnt, polyspnt));
									}
									else
									{
										//换一个方向
										Eigen::Vector2f tmp2dpnt = related_poly_corners_widths_2d_[i].first;
										related_poly_corners_widths_2d_[i].first = related_poly_corners_widths_2d_[i].second;
										related_poly_corners_widths_2d_[i].second = tmp2dpnt;
										related_polys_around_pnts_idx_[i] = hf_poly_idxs1;
										related_polys_around_pnts_pos_.emplace_back(std::make_pair(polyspnt, ls_cross_pnt));
									}
								}
								else
								{
									std::cerr << "poly0_cross, poly1_cross: " << poly0_cross[0] << " " << poly0_cross[1] << ", "
										<< poly1_cross[0] << " " << poly1_cross[1] << std::endl;
									std::cerr << "no polygon cross existed..." << std::endl;
									continue;
								}
							}
						}
						else if (!ComputeNearestLineCrossPnts(nldir2d, ls, ls_cross_idx, ls_cross_pnt)
							&& ComputeNearestLineCrossPnts(nldir2d, le, le_cross_idx, le_cross_pnt))
						{
							//处理这个polygon，需要处理这个polygon
							float lp2ls = (lpnt - ls).norm();
							float rp2ls = (rpnt - ls).norm();
							int s_idx = -1;
							if (lp2ls < rp2ls)
							{
								s_idx = lr_pnts_idx[0];
							}
							else
							{
								//有两个split polygon idxs
								s_idx = lr_pnts_idx[1];
							}
							if (s_idx == le_cross_idx[0])
							{
								//保存正常方向,否者获取方向再验证，后面处理
								//换一个方向
								std::vector<int> hf_poly_idxs1 = std::vector<int>{ le_cross_idx[0] , le_cross_idx[1] };
								/*Eigen::Vector2f tmp2dpnt = related_poly_corners_widths_2d_[i].first;
								related_poly_corners_widths_2d_[i].first = related_poly_corners_widths_2d_[i].second;
								related_poly_corners_widths_2d_[i].second = tmp2dpnt;*/
								related_polys_around_pnts_idx_[i] = hf_poly_idxs1;
								related_polys_around_pnts_pos_.emplace_back(std::make_pair(lpnt, le_cross_pnt));
							}
							else
							{
								//split polygon 
								std::vector<int> hf_poly_idxs0;
								s2e_idx[0] = le_cross_idx[1];
								s2e_idx[1] = s_idx;	//()
								std::cerr << "s2e_idx: " << s2e_idx[0] << " " << s2e_idx[1] << std::endl;
								ConstructHalfPolyIdxs(s2e_idx, hf_poly_idxs0);
								std::cerr << "hf_poly_idxs0: " << std::endl;
								ScoutPlaneVecIdxs(hf_poly_idxs0);

								std::vector<int> hf_poly_idxs1;
								s2e_idx_other[0] = s_idx;
								s2e_idx_other[1] = le_cross_idx[0];
								std::cerr << "s2e_idx_other: " << s2e_idx_other[0] << " " << s2e_idx_other[1] << std::endl;
								ConstructHalfPolyIdxs(s2e_idx_other, hf_poly_idxs1);
								std::cerr << "hf_poly_idxs1: " << std::endl;
								ScoutPlaneVecIdxs(hf_poly_idxs1);

								std::vector<Eigen::Vector2f> hf_poly_pnts0;
								std::vector<Eigen::Vector2f> hf_poly_pnts1;
								Eigen::Vector2f polyspnt = Eigen::Vector2f(corner_pnts_[s_idx].x, corner_pnts_[s_idx].y);
								hf_poly_pnts0.emplace_back(le_cross_pnt);
								for (int j = 0; j < hf_poly_idxs0.size(); ++j)
								{
									Eigen::Vector2f tmp_pnt = Eigen::Vector2f(corner_pnts_[hf_poly_idxs0[j]].x,
										corner_pnts_[hf_poly_idxs0[j]].y);
									hf_poly_pnts0.emplace_back(tmp_pnt);
								}
								hf_poly_pnts0.emplace_back(polyspnt);
								hf_poly_pnts1.emplace_back(polyspnt);
								for (int j = 0; j < hf_poly_idxs1.size(); ++j)
								{
									Eigen::Vector2f tmp_pnt = Eigen::Vector2f(corner_pnts_[hf_poly_idxs1[j]].x,
										corner_pnts_[hf_poly_idxs1[j]].y);
									hf_poly_pnts1.emplace_back(tmp_pnt);
								}
								hf_poly_pnts1.emplace_back(le_cross_pnt);
								//处理两个polygon，
								Eigen::Vector2f mean_pnt = (ls + le) / 2;
								Eigen::Vector2f poly0_cross, poly1_cross;
								if (ComputeLine2NearestHalfPolyCrossPnt(nldir2d, mean_pnt, hf_poly_pnts0, poly0_cross)
									&& ComputeLine2NearestHalfPolyCrossPnt(nldir2d, mean_pnt, hf_poly_pnts1, poly1_cross))
								{
									float poly0_dist = (poly0_cross - mean_pnt).norm();
									float poly1_dist = (poly1_cross - mean_pnt).norm();
									if (poly0_dist < poly1_dist)
									{
										//换一个方向
										Eigen::Vector2f tmp2dpnt = related_poly_corners_widths_2d_[i].first;
										related_poly_corners_widths_2d_[i].first = related_poly_corners_widths_2d_[i].second;
										related_poly_corners_widths_2d_[i].second = tmp2dpnt;
										related_polys_around_pnts_idx_[i] = hf_poly_idxs0;
										related_polys_around_pnts_pos_.emplace_back(std::make_pair(polyspnt, le_cross_pnt));
									}
									else
									{
										related_polys_around_pnts_idx_[i] = hf_poly_idxs1;
										related_polys_around_pnts_pos_.emplace_back(std::make_pair(le_cross_pnt, polyspnt));
									}
								}
								else
								{
									std::cerr << "poly0_cross, poly1_cross: " << poly0_cross[0] << " " << poly0_cross[1] << ", "
										<< poly1_cross[0] << " " << poly1_cross[1] << std::endl;
									std::cerr << "no polygon cross existed..." << std::endl;
									continue;
								}
							}
						}
						else
						{
							std::cerr << "error line and polygon line no section..." << std::endl;
						}
					}
					else
					{
						//设置对polygon进行设置，获取poly
						std::vector<int> move_idxs;
						ComputePolygonMoveIdxsFromline2d(ldir2d, ls, move_idxs);
						//计算这个方向和原始的方向是否一致,如果
						int move_idx_num = move_idxs.size();
						if (move_idx_num < 2)
						{
							std::cerr << "move_idx_num lower than 2..." << std::endl;
							continue;
						}
						else
						{
							if (move_idxs[0] == lr_pnts_idx[0] && move_idxs.back() == lr_pnts_idx[1])
							{
								//分别处理方向一致
								Eigen::Vector2f tmp_ls_cross_pnt = Eigen::Vector2f(corner_pnts_[lr_pnts_idx[0]].x, corner_pnts_[lr_pnts_idx[0]].y);
								Eigen::Vector2f tmp_le_cross_pnt = Eigen::Vector2f(corner_pnts_[lr_pnts_idx[1]].x, corner_pnts_[lr_pnts_idx[1]].y);
								related_polys_around_pnts_idx_[i] = move_idxs;
								related_polys_around_pnts_pos_.emplace_back(std::make_pair(tmp_ls_cross_pnt, tmp_le_cross_pnt));
							}
							else if (move_idxs[0] == lr_pnts_idx[1] && move_idxs.back() == lr_pnts_idx[0])
							{
								//换一个方向
								Eigen::Vector2f tmp2dpnt = related_poly_corners_widths_2d_[i].first;
								related_poly_corners_widths_2d_[i].first = related_poly_corners_widths_2d_[i].second;
								related_poly_corners_widths_2d_[i].second = tmp2dpnt;
								related_polys_around_pnts_idx_[i] = move_idxs;
								Eigen::Vector2f tmp_ls_cross_pnt = Eigen::Vector2f(corner_pnts_[lr_pnts_idx[1]].x, corner_pnts_[lr_pnts_idx[1]].y);
								Eigen::Vector2f tmp_le_cross_pnt = Eigen::Vector2f(corner_pnts_[lr_pnts_idx[0]].x, corner_pnts_[lr_pnts_idx[0]].y);
								related_polys_around_pnts_pos_.emplace_back(std::make_pair(tmp_ls_cross_pnt, tmp_le_cross_pnt));
							}
							else
							{
								std::cerr << "move_idx error..." << std::endl;
								continue;
							}
						}
					}
					////这个非常重要，需要处理一下，表示已经有剪切现象
					////剪切的途中发现哪些polygon有问题这是直线级别的剪切,首尾有没有交点
					//if (CheckCrossPntsWithPolygon(ls, le))
					//{
					//	//形成新的polygon start 和 end的idx
					//	s2e_idx[0] = ls_cross_idx[0];
					//	s2e_idx[1] = le_cross_idx[1];
					//	if (s2e_idx[0] == s2e_idx[1])
					//	{
					//		s2e_idx[1] = le_cross_idx[0];
					//	}
					//	//形成polygon idx环状
					//	std::vector<int> hf_poly_idxs;
					//	ConstructHalfPolyIdxs(s2e_idx, hf_poly_idxs);
					//	//check polygon
					//	if (!CheckLineDirectWithPolygonDirect(hf_poly_idxs))
					//	{
					//		//换一个方向
					//		related_poly_corners_widths_2d_[i].first = le;
					//		related_poly_corners_widths_2d_[i].second = ls;
					//	}
					//}
					//else
					//{
					//}
				}
				else
				{
					std::cerr << "ComputeSENearestLineCrossPnts: false" << std::endl;
					//找不到交线的话，原始的polygon投回去
					Eigen::Vector2i lr_pnts_idx(-1, -1);
					Eigen::Vector2f lpnt, rpnt;
					if (!ComputeLeftRightPointsPosLine2D(ldir2d, ls, lr_pnts_idx, lpnt, rpnt))
					{
						//check the line direct with polygon
						//一般情况是line在
						std::cerr << "lines lr is not existed..." << std::endl;
						continue;
					}
					std::cerr << "lr_pnts_idx: " << lr_pnts_idx[0] << " " << lr_pnts_idx[1] << std::endl;
					float lr_dist = (rpnt - lpnt).norm();
					float line_dist = (le - ls).norm();
					if (lr_dist > line_dist)
					{
						//表示这个线段比polygon要短
						if (ComputeNearestLineCrossPnts(nldir2d, ls, ls_cross_idx, ls_cross_pnt)
							&& ComputeNearestLineCrossPnts(nldir2d, le, le_cross_idx, le_cross_pnt))
						{
							//表示它是两个交线
							//计算两个交点组成的方向
							Eigen::Vector2f cross_dir = le_cross_pnt - ls_cross_pnt;
							//获取交线在polygon方向的方向。它怎么获取？通过ls_cross_idx和le_cross_idx来计算
							//其中ls_cross_idx[0]->ls_cross_idx[1]就是polygon方向,其中le_cross_idx[0]->le_cross_idx[1]就是polygon方向
							//怎么构建这个方向,因为它是环形，所以它有两个方向，需要分开处理。
							//有两个方向
							if (ls_cross_idx == le_cross_idx)
							{
								//表示它们在同一个线段上
								//获取这个线段上的两个端点
								Eigen::Vector2f src_poly_ls = Eigen::Vector2f(corner_pnts_[ls_cross_idx[0]].x, corner_pnts_[ls_cross_idx[0]].y);
								Eigen::Vector2f src_poly_le = Eigen::Vector2f(corner_pnts_[ls_cross_idx[1]].x, corner_pnts_[ls_cross_idx[1]].y);
								Eigen::Vector2f src_poly_dir = le - ls;
								if (src_poly_dir.dot(cross_dir) < 0)
								{
									//换一个方向
									Eigen::Vector2f tmp2dpnt = related_poly_corners_widths_2d_[i].first;
									related_poly_corners_widths_2d_[i].first = related_poly_corners_widths_2d_[i].second;
									related_poly_corners_widths_2d_[i].second = tmp2dpnt;
								}
							}
							else
							{
								//有两个split polygon idxs
								s2e_idx[0] = ls_cross_idx[1];
								s2e_idx[1] = le_cross_idx[0];
								//split polygon 
								std::vector<int> hf_poly_idxs0;
								std::cerr << "s2e_idx 11: " << s2e_idx[0] << " " << s2e_idx[1] << std::endl;
								ConstructHalfPolyIdxs(s2e_idx, hf_poly_idxs0);
								std::cerr << "hf_poly_idxs0: " << std::endl;
								ScoutPlaneVecIdxs(hf_poly_idxs0);

								std::vector<int> hf_poly_idxs1;
								s2e_idx_other[0] = le_cross_idx[1];
								s2e_idx_other[1] = ls_cross_idx[0];
								std::cerr << "s2e_idx_other 11: " << s2e_idx_other[0] << " " << s2e_idx_other[1] << std::endl;
								ConstructHalfPolyIdxs(s2e_idx_other, hf_poly_idxs1);
								std::cerr << "hf_poly_idxs1: " << std::endl;
								ScoutPlaneVecIdxs(hf_poly_idxs1);

								std::vector<Eigen::Vector2f> hf_poly_pnts0;
								std::vector<Eigen::Vector2f> hf_poly_pnts1;
								hf_poly_pnts0.emplace_back(ls_cross_pnt);
								for (int j = 0; j < hf_poly_idxs0.size(); ++j)
								{
									Eigen::Vector2f tmp_pnt = Eigen::Vector2f(corner_pnts_[hf_poly_idxs0[j]].x,
										corner_pnts_[hf_poly_idxs0[j]].y);
									hf_poly_pnts0.emplace_back(tmp_pnt);
								}
								hf_poly_pnts0.emplace_back(le_cross_pnt);

								hf_poly_pnts1.emplace_back(le_cross_pnt);
								for (int j = 0; j < hf_poly_idxs1.size(); ++j)
								{
									Eigen::Vector2f tmp_pnt = Eigen::Vector2f(corner_pnts_[hf_poly_idxs1[j]].x,
										corner_pnts_[hf_poly_idxs1[j]].y);
									hf_poly_pnts1.emplace_back(tmp_pnt);
								}
								hf_poly_pnts1.emplace_back(ls_cross_pnt);
								//处理两个polygon，
								Eigen::Vector2f mean_pnt = (ls + le) / 2;
								Eigen::Vector2f poly0_cross, poly1_cross;
								if (ComputeLine2NearestHalfPolyCrossPnt(nldir2d, mean_pnt, hf_poly_pnts0, poly0_cross)
									&& ComputeLine2NearestHalfPolyCrossPnt(nldir2d, mean_pnt, hf_poly_pnts1, poly1_cross))
								{
									float poly0_dist = (poly0_cross - mean_pnt).norm();
									float poly1_dist = (poly1_cross - mean_pnt).norm();
									if (poly0_dist < poly1_dist)
									{
										related_polys_around_pnts_idx_[i] = hf_poly_idxs0;
										related_polys_around_pnts_pos_.emplace_back(std::make_pair(ls_cross_pnt, le_cross_pnt));
									}
									else
									{
										//换一个方向
										Eigen::Vector2f tmp2dpnt = related_poly_corners_widths_2d_[i].first;
										related_poly_corners_widths_2d_[i].first = related_poly_corners_widths_2d_[i].second;
										related_poly_corners_widths_2d_[i].second = tmp2dpnt;
										related_polys_around_pnts_idx_[i] = hf_poly_idxs1;
										related_polys_around_pnts_pos_.emplace_back(std::make_pair(le_cross_pnt, ls_cross_pnt));
									}
								}
								else
								{
									std::cerr << "poly0_cross, poly1_cross: " << poly0_cross[0] << " " << poly0_cross[1] << ", "
										<< poly1_cross[0] << " " << poly1_cross[1] << std::endl;
									std::cerr << "no polygon cross existed..." << std::endl;
									continue;
								}
							}
						}
						else if (ComputeNearestLineCrossPnts(nldir2d, ls, ls_cross_idx, ls_cross_pnt)
							&& !ComputeNearestLineCrossPnts(nldir2d, le, le_cross_idx, le_cross_pnt))
						{
							//处理这个polygon，需要处理这个polygon
							float lp2le = (lpnt - le).norm();
							float rp2le = (rpnt - le).norm();
							int s_idx = -1;
							if (lp2le < rp2le)
							{
								//有两个split polygon idxs
								s_idx = lr_pnts_idx[0];
							}
							else
							{
								//有两个split polygon idxs
								s_idx = lr_pnts_idx[1];
							}
							if (s_idx == ls_cross_idx[0])
							{
								//反向,否者获取方向再验证，后面处理
								//换一个方向
								std::vector<int> hf_poly_idxs1 = std::vector<int>{ ls_cross_idx[0] , ls_cross_idx[1] };
								Eigen::Vector2f tmp2dpnt = related_poly_corners_widths_2d_[i].first;
								related_poly_corners_widths_2d_[i].first = related_poly_corners_widths_2d_[i].second;
								related_poly_corners_widths_2d_[i].second = tmp2dpnt;
								related_polys_around_pnts_idx_[i] = hf_poly_idxs1;
								related_polys_around_pnts_pos_.emplace_back(std::make_pair(lpnt, ls_cross_pnt));
							}
							else
							{
								//split polygon 
								std::vector<int> hf_poly_idxs0;
								s2e_idx[0] = ls_cross_idx[1];
								s2e_idx[1] = s_idx;	//()
								std::cerr << "s2e_idx: " << s2e_idx[0] << " " << s2e_idx[1] << std::endl;
								ConstructHalfPolyIdxs(s2e_idx, hf_poly_idxs0);
								std::cerr << "hf_poly_idxs0: " << std::endl;
								ScoutPlaneVecIdxs(hf_poly_idxs0);

								std::vector<int> hf_poly_idxs1;
								s2e_idx_other[0] = s_idx;
								s2e_idx_other[1] = ls_cross_idx[0];
								std::cerr << "s2e_idx_other: " << s2e_idx_other[0] << " " << s2e_idx_other[1] << std::endl;
								ConstructHalfPolyIdxs(s2e_idx_other, hf_poly_idxs1);
								std::cerr << "hf_poly_idxs1: " << std::endl;
								ScoutPlaneVecIdxs(hf_poly_idxs1);

								std::vector<Eigen::Vector2f> hf_poly_pnts0;
								std::vector<Eigen::Vector2f> hf_poly_pnts1;
								Eigen::Vector2f polyspnt = Eigen::Vector2f(corner_pnts_[s_idx].x, corner_pnts_[s_idx].y);
								hf_poly_pnts0.emplace_back(ls_cross_pnt);
								for (int j = 0; j < hf_poly_idxs0.size(); ++j)
								{
									Eigen::Vector2f tmp_pnt = Eigen::Vector2f(corner_pnts_[hf_poly_idxs0[j]].x,
										corner_pnts_[hf_poly_idxs0[j]].y);
									hf_poly_pnts0.emplace_back(tmp_pnt);
								}
								hf_poly_pnts0.emplace_back(polyspnt);
								hf_poly_pnts1.emplace_back(polyspnt);
								for (int j = 0; j < hf_poly_idxs1.size(); ++j)
								{
									Eigen::Vector2f tmp_pnt = Eigen::Vector2f(corner_pnts_[hf_poly_idxs1[j]].x,
										corner_pnts_[hf_poly_idxs1[j]].y);
									hf_poly_pnts1.emplace_back(tmp_pnt);
								}
								hf_poly_pnts1.emplace_back(ls_cross_pnt);
								//处理两个polygon，
								Eigen::Vector2f mean_pnt = (ls + le) / 2;
								Eigen::Vector2f poly0_cross, poly1_cross;
								if (ComputeLine2NearestHalfPolyCrossPnt(nldir2d, mean_pnt, hf_poly_pnts0, poly0_cross)
									&& ComputeLine2NearestHalfPolyCrossPnt(nldir2d, mean_pnt, hf_poly_pnts1, poly1_cross))
								{
									float poly0_dist = (poly0_cross - mean_pnt).norm();
									float poly1_dist = (poly1_cross - mean_pnt).norm();
									if (poly0_dist < poly1_dist)
									{
										related_polys_around_pnts_idx_[i] = hf_poly_idxs0;
										related_polys_around_pnts_pos_.emplace_back(std::make_pair(ls_cross_pnt, polyspnt));
									}
									else
									{
										//换一个方向
										Eigen::Vector2f tmp2dpnt = related_poly_corners_widths_2d_[i].first;
										related_poly_corners_widths_2d_[i].first = related_poly_corners_widths_2d_[i].second;
										related_poly_corners_widths_2d_[i].second = tmp2dpnt;
										related_polys_around_pnts_idx_[i] = hf_poly_idxs1;
										related_polys_around_pnts_pos_.emplace_back(std::make_pair(polyspnt, ls_cross_pnt));
									}
								}
								else
								{
									std::cerr << "poly0_cross, poly1_cross: " << poly0_cross[0] << " " << poly0_cross[1] << ", "
										<< poly1_cross[0] << " " << poly1_cross[1] << std::endl;
									std::cerr << "no polygon cross existed..." << std::endl;
									continue;
								}
							}
						}
						else if (!ComputeNearestLineCrossPnts(nldir2d, ls, ls_cross_idx, ls_cross_pnt)
							&& ComputeNearestLineCrossPnts(nldir2d, le, le_cross_idx, le_cross_pnt))
						{
							//处理这个polygon，需要处理这个polygon
							float lp2ls = (lpnt - ls).norm();
							float rp2ls = (rpnt - ls).norm();
							int s_idx = -1;
							if (lp2ls < rp2ls)
							{
								s_idx = lr_pnts_idx[0];
							}
							else
							{
								//有两个split polygon idxs
								s_idx = lr_pnts_idx[1];
							}
							if (s_idx == le_cross_idx[0])
							{
								//保存正常方向,否者获取方向再验证，后面处理
								//换一个方向
								std::vector<int> hf_poly_idxs1 = std::vector<int>{ le_cross_idx[0] , le_cross_idx[1] };
								/*Eigen::Vector2f tmp2dpnt = related_poly_corners_widths_2d_[i].first;
								related_poly_corners_widths_2d_[i].first = related_poly_corners_widths_2d_[i].second;
								related_poly_corners_widths_2d_[i].second = tmp2dpnt;*/
								related_polys_around_pnts_idx_[i] = hf_poly_idxs1;
								related_polys_around_pnts_pos_.emplace_back(std::make_pair(lpnt, le_cross_pnt));
							}
							else
							{
								//split polygon 
								std::vector<int> hf_poly_idxs0;
								s2e_idx[0] = le_cross_idx[1];
								s2e_idx[1] = s_idx;	//()
								std::cerr << "s2e_idx: " << s2e_idx[0] << " " << s2e_idx[1] << std::endl;
								ConstructHalfPolyIdxs(s2e_idx, hf_poly_idxs0);
								std::cerr << "hf_poly_idxs0: " << std::endl;
								ScoutPlaneVecIdxs(hf_poly_idxs0);

								std::vector<int> hf_poly_idxs1;
								s2e_idx_other[0] = s_idx;
								s2e_idx_other[1] = le_cross_idx[0];
								std::cerr << "s2e_idx_other: " << s2e_idx_other[0] << " " << s2e_idx_other[1] << std::endl;
								ConstructHalfPolyIdxs(s2e_idx_other, hf_poly_idxs1);
								std::cerr << "hf_poly_idxs1: " << std::endl;
								ScoutPlaneVecIdxs(hf_poly_idxs1);

								std::vector<Eigen::Vector2f> hf_poly_pnts0;
								std::vector<Eigen::Vector2f> hf_poly_pnts1;
								Eigen::Vector2f polyspnt = Eigen::Vector2f(corner_pnts_[s_idx].x, corner_pnts_[s_idx].y);
								hf_poly_pnts0.emplace_back(le_cross_pnt);
								for (int j = 0; j < hf_poly_idxs0.size(); ++j)
								{
									Eigen::Vector2f tmp_pnt = Eigen::Vector2f(corner_pnts_[hf_poly_idxs0[j]].x,
										corner_pnts_[hf_poly_idxs0[j]].y);
									hf_poly_pnts0.emplace_back(tmp_pnt);
								}
								hf_poly_pnts0.emplace_back(polyspnt);
								hf_poly_pnts1.emplace_back(polyspnt);
								for (int j = 0; j < hf_poly_idxs1.size(); ++j)
								{
									Eigen::Vector2f tmp_pnt = Eigen::Vector2f(corner_pnts_[hf_poly_idxs1[j]].x,
										corner_pnts_[hf_poly_idxs1[j]].y);
									hf_poly_pnts1.emplace_back(tmp_pnt);
								}
								hf_poly_pnts1.emplace_back(le_cross_pnt);
								//处理两个polygon，
								Eigen::Vector2f mean_pnt = (ls + le) / 2;
								Eigen::Vector2f poly0_cross, poly1_cross;
								if (ComputeLine2NearestHalfPolyCrossPnt(nldir2d, mean_pnt, hf_poly_pnts0, poly0_cross)
									&& ComputeLine2NearestHalfPolyCrossPnt(nldir2d, mean_pnt, hf_poly_pnts1, poly1_cross))
								{
									float poly0_dist = (poly0_cross - mean_pnt).norm();
									float poly1_dist = (poly1_cross - mean_pnt).norm();
									if (poly0_dist < poly1_dist)
									{
										//换一个方向
										Eigen::Vector2f tmp2dpnt = related_poly_corners_widths_2d_[i].first;
										related_poly_corners_widths_2d_[i].first = related_poly_corners_widths_2d_[i].second;
										related_poly_corners_widths_2d_[i].second = tmp2dpnt;
										related_polys_around_pnts_idx_[i] = hf_poly_idxs0;
										related_polys_around_pnts_pos_.emplace_back(std::make_pair(polyspnt, le_cross_pnt));
									}
									else
									{
										related_polys_around_pnts_idx_[i] = hf_poly_idxs1;
										related_polys_around_pnts_pos_.emplace_back(std::make_pair(le_cross_pnt, polyspnt));
									}
								}
								else
								{
									std::cerr << "poly0_cross, poly1_cross: " << poly0_cross[0] << " " << poly0_cross[1] << ", "
										<< poly1_cross[0] << " " << poly1_cross[1] << std::endl;
									std::cerr << "no polygon cross existed..." << std::endl;
									continue;
								}
							}
						}
						else
						{
							std::cerr << "error line and polygon line no section..." << std::endl;
						}
					}
					else
					{
						//设置对polygon进行设置，获取poly
						std::cout << "the move idxs start..." << std::endl;
						std::vector<int> move_idxs;
						ComputePolygonMoveIdxsFromline2d(ldir2d, ls, move_idxs);
						//计算这个方向和原始的方向是否一致,如果
						int move_idx_num = move_idxs.size();
						if (move_idx_num < 2)
						{
							std::cerr << "move_idx_num lower than 2..." << std::endl;
							continue;
						}
						else
						{
							if (move_idxs[0] == lr_pnts_idx[0] && move_idxs.back() == lr_pnts_idx[1])
							{
								//分别处理方向一致
								Eigen::Vector2f tmp_ls_cross_pnt = Eigen::Vector2f(corner_pnts_[lr_pnts_idx[0]].x, corner_pnts_[lr_pnts_idx[0]].y);
								Eigen::Vector2f tmp_le_cross_pnt = Eigen::Vector2f(corner_pnts_[lr_pnts_idx[1]].x, corner_pnts_[lr_pnts_idx[1]].y);
								
								related_polys_around_pnts_idx_[i] = move_idxs;
								related_polys_around_pnts_pos_.emplace_back(std::make_pair(tmp_ls_cross_pnt, tmp_le_cross_pnt));
							}
							else if(move_idxs[0] == lr_pnts_idx[1] && move_idxs.back() == lr_pnts_idx[0])
							{
								//换一个方向
								Eigen::Vector2f tmp2dpnt = related_poly_corners_widths_2d_[i].first;
								related_poly_corners_widths_2d_[i].first = related_poly_corners_widths_2d_[i].second;
								related_poly_corners_widths_2d_[i].second = tmp2dpnt;
								related_polys_around_pnts_idx_[i] = move_idxs;
								Eigen::Vector2f tmp_ls_cross_pnt = Eigen::Vector2f(corner_pnts_[lr_pnts_idx[1]].x, corner_pnts_[lr_pnts_idx[1]].y);
								Eigen::Vector2f tmp_le_cross_pnt = Eigen::Vector2f(corner_pnts_[lr_pnts_idx[0]].x, corner_pnts_[lr_pnts_idx[0]].y);
								related_polys_around_pnts_pos_.emplace_back(std::make_pair(tmp_ls_cross_pnt, tmp_le_cross_pnt));
							}
							else
							{
								std::cerr << "move_idx error..." << std::endl;
								continue;
							}
						}

					}

					//{
					//	//首先查看这个线段是否和原始的polygon相交,不相交再处理
					//	//ls
					//	int iter_num = 5;
					//	int iter_idx = iter_num;//最多迭代5次
					//	while (iter_idx)
					//	{
					//		if (ComputeNearestLineCrossPnts(nldir2d, ls, ls_cross_idx, ls_cross_pnt)
					//			&& ComputeNearestLineCrossPnts(nldir2d, le, le_cross_idx, le_cross_pnt))
					//		{
					//			//找到顶点顺序
					//			//形成新的polygon start 和 end的idx
					//			s2e_idx[0] = ls_cross_idx[0];
					//			s2e_idx[1] = le_cross_idx[1];
					//			if (s2e_idx[0] == s2e_idx[1])
					//			{
					//				s2e_idx[1] = le_cross_idx[0];
					//			}
					//			//check the line direct with polygon
					//			//形成polygon idx环状
					//			std::vector<int> hf_poly_idxs;
					//			ConstructHalfPolyIdxs(s2e_idx, hf_poly_idxs);
					//			//check polygon
					//			////获取离polygon较为近的半圆，判断它的方向，这个方向还是要重新处理一下
					//			//现有的方法有bug
					//			if (!CheckLineDirectWithPolygonDirect(hf_poly_idxs))
					//			{
					//				//换一个方向
					//				related_poly_corners_widths_2d_[i].first = le;
					//				related_poly_corners_widths_2d_[i].second = ls;
					//			}
					//			break;
					//		}
					//		else
					//		{
					//			//找不到交线的话，原始的polygon投回去
					//			Eigen::Vector2i lr_pnts_idx(-1, -1);
					//			Eigen::Vector2f lpnt, rpnt;
					//			if (ComputeLeftRightPointsPosLine2D(ldir2d, lpnt2d, lr_pnts_idx, lpnt, rpnt))
					//			{
					//				//check the line direct with polygon
					//				//一般情况是line在
					//				break;
					//			}
					//			else
					//			{
					//				//距离变小
					//				Eigen::Vector2f tmp_ls = ls + (le - ls) / iter_num;
					//				Eigen::Vector2f tmp_le = le - (le - ls) / iter_num;
					//				ls = tmp_ls;
					//				le = tmp_le;
					//			}
					//		}
					//		--iter_idx;
					//	}
					//}

				}
				
			}
		}
	}

	void HWPlane::OpitmizeAdjLinesPolygon()
	{
		/*
		优化一下顺序
		*/

		/*std::string polygon1_path_new = "D:\\vc_project_new\\huawei_data_indoor\\room\\test_room_cam2\\new111.obj";
		planes_vec_[source_polygon_id_]->WriteSelectEdges3D(polygon1_path_new, polygon_split1_new);
		std::string polygon2_path_new = "D:\\vc_project_new\\huawei_data_indoor\\room\\test_room_cam2\\new222.obj";
		planes_vec_[source_polygon_id_]->WriteSelectEdges3D(polygon2_path_new, polygon_split2_new); */


		int lineseg_related_num = related_poly_corners_widths_2d_.size();
		for (int i = 0; i < lineseg_related_num; ++i)
		{
			std::cerr << "lineseg_related_num------>current idx: "
				<< related_poly_idxs_[i] << std::endl;
			std::string polygon1_path_new = "D:\\vc_project_new\\huawei_data_indoor\\thesis_room\\6\\dfwg_room\\11\\"
				+ std::to_string(related_poly_idxs_[i]) + ".obj";
			WriteSelectLines3D(polygon1_path_new, related_polys_around_pnts_idx_[i]);
			Eigen::Vector2f ls = related_poly_corners_widths_2d_[i].first;
			Eigen::Vector2f le = related_poly_corners_widths_2d_[i].second;
			std::cerr << "ls, le: " << ls[0] << " " << ls[1] << ", " << le[0] << " " << le[1] << std::endl;
			//对线段构建矩形
			int polypnts_num = related_polys_around_pnts_idx_[i].size();
			std::cerr << "related_polys_around_pnts_idx_" << "[" << i << "]: " << polypnts_num << std::endl;
			if (polypnts_num < 2)
				continue;
			std::vector<Eigen::Vector2f> rect_pnts;
			Eigen::Vector2f ldir2d = le - ls;
			int srcpolyidxs = related_polys_around_pnts_idx_[i][0];
			int srcpolyidxe = related_polys_around_pnts_idx_[i].back();
			Eigen::Vector2f tmp_ls_cross_pnt = Eigen::Vector2f(corner_pnts_[srcpolyidxs].x, corner_pnts_[srcpolyidxs].y);
			Eigen::Vector2f tmp_le_cross_pnt = Eigen::Vector2f(corner_pnts_[srcpolyidxe].x, corner_pnts_[srcpolyidxe].y);
			//获取矩形
			float l2line0 = Pnt2DDistToLine2DNew1(tmp_ls_cross_pnt, ls, ldir2d);
			float l2line1 = Pnt2DDistToLine2DNew1(tmp_le_cross_pnt, ls, ldir2d);
			std::cerr << "l2line0, l2line1: " << l2line0 << ", " << l2line1 << std::endl;
			float dist2line = 0.0;
			//获取dist2line的值
			int dist_line_flag = false;
			if (!ComputeNearestDist2ParallAdjLines(i, dist2line))
			{
				dist2line = (l2line0 + l2line1) / 4;
			}
			std::cout << "----------------: " << dist2line << std::endl;
			int failed_dist = dist2line / 2;
			if (ContructRectFromLS2D(ls, le, failed_dist, rect_pnts))
			{
				//通过rect_pnts 去优化 move_idxs
				Eigen::Vector2i s_lidx(-1, -1);
				Eigen::Vector2i e_lidx(-1, -1);
				/*std::cerr << "rect pnts: " << std::endl;
				for (int j = 0; j < rect_pnts.size(); ++j)
				{
					std::cerr << rect_pnts[j] << std::endl;
				}*/
				if (ComputeCrossLSIdxFromRect(rect_pnts, related_polys_around_pnts_idx_[i], s_lidx, e_lidx))
				{
					//构建新的idxs，用于后续的操作。
					std::cerr << "s_lidx, e_lidx: " << s_lidx[0] << " " << s_lidx[1] << 
						", " << e_lidx[0] << " " << e_lidx[1] << std::endl;
					if (s_lidx[1] < e_lidx[0])
					{
						//
						std::cerr << "update i: " << i << " moves_idxs_new" << std::endl;
						std::vector<int> moves_idxs_new;
						for (int j = s_lidx[1]; j <= e_lidx[0]; ++j)
						{
							moves_idxs_new.emplace_back(related_polys_around_pnts_idx_[i][j]);
						}
						related_polys_around_pnts_idx_[i] = moves_idxs_new;
					}
					else if (s_lidx == e_lidx)
					{
						//处理问题只有一个交点时候,找到两个polygon的首尾

					}
				}
			}
		}
	}

#if 1
	bool HWPlane::SetRelatedIdxAroundSrcPoly()
	{
		//设置顺序
		//因为有一个顺序，很重要的顺序related_polys_around_pnts_idx_,它表示这realted_poly_idxs的顺序
		//寻找最近的polygon idx
		std::cerr << "the initial around idxs: " << std::endl;
		ScoutPlaneVecIdxs(related_poly_idxs_);
		for (int i = 0; i < related_polys_around_pnts_idx_.size(); ++i)
		{
			ScoutPlaneVecIdxs(related_polys_around_pnts_idx_[i]);
		}
		std::cerr << "end the initial around idxs: " << std::endl;

		OpitmizeAdjLinesPolygon();
		std::cerr << "the optimized around idxs: " << std::endl;
		ScoutPlaneVecIdxs(related_poly_idxs_);
		for (int i = 0; i < related_polys_around_pnts_idx_.size(); ++i)
		{
			ScoutPlaneVecIdxs(related_polys_around_pnts_idx_[i]);
		}
		std::cerr << "end the optimized around idxs: " << std::endl;

		std::vector<int> all_polygon_sorted_idxs;
		int related_pnts_idxs_num = related_polys_around_pnts_idx_.size();
		//std::cout << "the related_pnts_idxs_num: " << related_pnts_idxs_num << std::endl;
		std::vector<bool> visited_flag(related_pnts_idxs_num, false);
		int cur_idx = 0;
		int cur_pnts_idx = 0;
		while (!CheckVecFlag(visited_flag) && cur_idx < related_pnts_idxs_num)
		{
			std::cout << "start cur_idx: " << cur_idx << std::endl;
			int start_idx = -1;
			std::vector<int> start_idxs;
			for (int i = 0; i < related_polys_around_pnts_idx_.size(); ++i)
			{
				if (visited_flag[i])
					continue;
				if (related_polys_around_pnts_idx_[i].empty())
					return false;
				if (related_polys_around_pnts_idx_[i][0] == cur_pnts_idx)
				{
					//start_idx = i;
					start_idxs.emplace_back(i);
				}
			}
			if (start_idxs.empty())
			{
				//++cur_idx;
				++cur_pnts_idx;
				continue;
			}
			else
			{
				std::cerr << "start start_idxs num: "<< start_idxs.size() << std::endl;
				/*for (int i = 0; i < start_idxs.size(); ++i)
				{
					std::cerr << start_idxs[i] << " ";
				}
				std::cerr << std::endl;
				std::cerr << "end start_idx idxs" << std::endl;*/
			}
			//对start_idxs进行排序
			std::vector<int> tmp_sorted_start_idxs;
			std::vector<bool> start_idxs_flag(start_idxs.size(), false);
			for (int i = 0; i < start_idxs.size(); ++i)
			{
				if (start_idxs_flag[i])
					continue;
				//因为它们起始的idx一样，但是它的end idx 不一样，可以通过idx来判断
				//int min_le_idx = related_polys_around_pnts_idx_.size() + 1;
				int min_le_num = corner_pnts_.size() + 1;
				int idx_j = -1;
				for (int j = 0; j < start_idxs.size(); ++j)
				{
					if (start_idxs_flag[j])
						continue;
					std::vector<int> poly_idxs = related_polys_around_pnts_idx_[start_idxs[j]];
					int tmp_cur_le_num = poly_idxs.size();
					if (tmp_cur_le_num < min_le_num)
					{
						min_le_num = tmp_cur_le_num;
						start_idx = start_idxs[j];
						idx_j = j;
					}
					/*int tmp_endidx = poly_idxs.back();
					std::cerr << "tmp_endidx: " << tmp_endidx << std::endl;
					if (tmp_endidx < min_le_idx)
					{
						min_le_idx = tmp_endidx;
						start_idx = start_idxs[j];
						idx_j = j;
					}*/
				}
				if (idx_j != -1)
				{
					start_idxs_flag[idx_j] = true;
					tmp_sorted_start_idxs.emplace_back(start_idx);
				}
			}
			for (int i = 0; i < start_idxs.size(); ++i)
			{
				if (!start_idxs_flag[i])
				{
					tmp_sorted_start_idxs.emplace_back(start_idxs[i]);
				}
			}

			/*std::cerr << "start tmp_sorted_start_idxs num: " << tmp_sorted_start_idxs.size() << std::endl;
			for (int i = 0; i < tmp_sorted_start_idxs.size(); ++i)
			{
				std::cerr << tmp_sorted_start_idxs[i] << " ";
			}
			std::cerr << std::endl;
			std::cerr << "end tmp_sorted_start_idxs idxs" << std::endl;*/

			for (int i = 0; i < tmp_sorted_start_idxs.size(); ++i)
			{
				//处理这个polygon idx
				//if (start_idxs_flag[i])
					//continue;
				int tmpidx = tmp_sorted_start_idxs[i];
				//std::cerr << "tmpidx: " << tmpidx << std::endl;
				visited_flag[tmpidx] = true;
				all_polygon_sorted_idxs.emplace_back(tmpidx);
			}
			++cur_idx;
			//std::cout << "end cur_idx: " << cur_idx << std::endl;
		}
		sorted_around_related_poly_idxs_ = all_polygon_sorted_idxs;
		//std::cerr << "the all_polygon_sorted_idxs: " << std::endl;
		//ScoutPlaneVecIdxs(all_polygon_sorted_idxs);
		std::vector<int> sorted_poly_idxs_tmp;
		for (int i = 0; i < all_polygon_sorted_idxs.size(); ++i)
		{
			sorted_poly_idxs_tmp.emplace_back(related_poly_idxs_[all_polygon_sorted_idxs[i]]);
		}
		std::cerr << "the sorted_poly_idxs_tmp: " << std::endl;
		ScoutPlaneVecIdxs(sorted_poly_idxs_tmp);
		//all_polygon_sorted_idxs
		return true;
	}
#endif

	bool HWPlane::ContructRectFromLS2D(Eigen::Vector2f& ls, Eigen::Vector2f& le, 
		float width, std::vector<Eigen::Vector2f>& rect_pnts)
	{
		//获取rect_pnts
		Eigen::Vector2f ldir2d = le - ls;
		if (ldir2d.norm() < 1e-6)
			return false;
		ldir2d.normalize();
		float k_dir2d = (le - ls).norm() / ldir2d.norm();
		Eigen::Vector2f mldir2d = Eigen::Vector2f(-ldir2d[1], ldir2d[0]);
		Eigen::Vector2f s0 = ls + width * mldir2d;
		Eigen::Vector2f s1 = s0 + k_dir2d * ldir2d;
		Eigen::Vector2f s2 = s1 - width * mldir2d * 2;
		Eigen::Vector2f s3 = s2 - k_dir2d* ldir2d;
		rect_pnts.emplace_back(s0);
		rect_pnts.emplace_back(s1);
		rect_pnts.emplace_back(s2);
		rect_pnts.emplace_back(s3);
		return true;
	}

	bool HWPlane::ComputeNearestDist2ParallAdjLines(int srcidx, float& dist)
	{
		if (srcidx < 0 || srcidx >= related_poly_corners_widths_2d_.size())
			return false;
		float min_dist = std::numeric_limits<float>::max();
		Eigen::Vector2f src_s = related_poly_corners_widths_2d_[srcidx].first;
		Eigen::Vector2f src_e = related_poly_corners_widths_2d_[srcidx].second;
		bool flag_computed = false;
		for (int i = 0; i < related_poly_corners_widths_2d_.size(); ++i)
		{
			//处理这个polygon
			if (i == srcidx)
				continue;
			Eigen::Vector2f t_s = related_poly_corners_widths_2d_[i].first;
			Eigen::Vector2f t_e = related_poly_corners_widths_2d_[i].second;
			//判断两个线段是否平行接近平
			Eigen::Vector2f tdir2d = t_e - t_s;
			tdir2d.normalize();
			Eigen::Vector2f sdir2d = src_e - src_s;
			sdir2d.normalize();
			//std::cerr << "ComputeNearestDist2ParallAdjLines src: " << sdir2d[0] << " " << sdir2d[1] << std::endl;
			//std::cerr << "ComputeNearestDist2ParallAdjLines tgt: " << tdir2d[0] << " " << tdir2d[1] << std::endl;
			if (std::abs(tdir2d.dot(sdir2d)) > 0.8)
			{
				float dist_ts = Pnt2DDistToLine2DNew1(t_s, src_s, sdir2d);
				float dist_te = Pnt2DDistToLine2DNew1(t_e, src_s, sdir2d);
				if (dist_ts > dist_te)
				{
					if (dist_te < min_dist)
					{
						min_dist = dist_te;
						flag_computed = true;
					}
				}
				else
				{
					if (dist_ts < min_dist)
					{
						min_dist = dist_ts;
						flag_computed = true;
					}
				}
			}
			else
			{
				continue;
			}
		}
		dist = min_dist;
		return flag_computed;
	}

	bool HWPlane::CheckLineDirectWithPolygonDirect(std::vector<int>& lines_idxs)
	{
		int line_num = lines_idxs.size();
		if (line_num < 2)
			return false;
		bool flag_direct = true;
		for (int i = 0; i < lines_idxs.size(); ++i)
		{
			int idx = (lines_idxs[0] + i) % corner_pnts_.size();
			if (idx != lines_idxs[i])
				return false;
		}
		return true;
	}

	void HWPlane::ConstructHalfPolyIdxs(Eigen::Vector2i& s2e, std::vector<int>& half_poly)
	{
		if (s2e[0] == s2e[1])
		{
			half_poly.emplace_back(s2e[0]);
		}
		else
		{
			if (s2e[0] > s2e[1])
			{
				for (int i = s2e[0]; i < corner_pnts_.size(); ++i)
				{
					half_poly.emplace_back(i);
				}
				for (int i = 0; i <= s2e[1]; ++i)
				{
					half_poly.emplace_back(i);
				}
			}
			else
			{
				for (int i = s2e[0]; i <= s2e[1]; ++i)
				{
					//std::cerr << i << std::endl;
					half_poly.emplace_back(i);
				}
			}
		}
	}

	bool HWPlane::CheckTwoPolysAdj(int src_idx, int tgt_idx, std::vector<std::vector<int> >& polygons_adjs)
	{
		if (src_idx < 0 || src_idx >= polygons_adjs.size())
			return false;
		for (int i = 0; i < polygons_adjs[src_idx].size(); ++i)
		{
			if (polygons_adjs[src_idx][i] == tgt_idx)
				return true;
		}
		return false;
	}

	bool HWPlane::SortRelatedPolyIdxBasedOnMinCor()
	{
		//这个坐标系需要重新处理
		if (!resorted_sor_corner_flag_)
			SortCornerPnts();
		
		std::cerr << "before sort the tgt polygon idx " << std::endl;
		print_polygon_idx(related_poly_idxs_);
		std::vector<int> resorted_related_poly_idxs;
		std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f> > lines_2d;	//ldir2d, lpnt2d
		/*std::vector<float> poly_areas2d;
		std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f> > lines_seg_2d;*/
		//对所有的t的polygon进行排序,直接和坐标系进行交叉
		for (int i = 0; i < related_poly_idxs_.size(); ++i)
		{
			Eigen::Vector3f ldir3di, lpnt3di;
			GetExpandTgtIntersectionLine(related_poly_idxs_[i], ldir3di, lpnt3di);
			//转化到当前的坐标系中
			Eigen::Vector2f ldir2d, lpnt2d;
			ComputeWorldLf2PlaneLf(ldir3di, lpnt3di, ldir2d, lpnt2d);
			std::pair<Eigen::Vector2f, Eigen::Vector2f> line2d 
				= std::make_pair(ldir2d, lpnt2d);
			lines_2d.emplace_back(line2d);
			/*float tmp_area = 0.0;
			Eigen::Vector2f ls(0.0, 0.0), le(0.0, 0.0);
			ComputePolyParamsOnSrcCoord(related_poly_idxs_[i], tmp_area, ls, le);
			poly_areas2d.emplace_back(tmp_area);
			std::pair<Eigen::Vector2f, Eigen::Vector2f> lsle2d
				= std::make_pair(ls, le);
			lines_seg_2d.emplace_back(lsle2d);*/
			//可以验证自己写的两个函数是否正确，后续再做
		}
		//计算所有t线段的方向向量，然后计算平均值，一般情况，大的线段保证大的权重
		bool x_flag = true;
		Eigen::Vector2f avarage_ldir;
		Eigen::Vector2f sum_dir(0.0, 0.0);
		for (int i = 0; i < lines_2d.size(); ++i)
		{
			Eigen::Vector2f tmp_ldir = Eigen::Vector2f(std::abs(lines_2d[i].first[0]), 
				std::abs(lines_2d[i].first[1]));
			sum_dir += tmp_ldir;
		}
		if (std::abs(sum_dir[1]) < 1e-6)
		{
			x_flag = false;
		}
		else
		{
			if (std::abs(sum_dir[1] / sum_dir[0]) < 1.0)
			{
				x_flag = false;
			}
		}
		//让其和x轴计算夹角
		Eigen::Vector2f lbasedir = Eigen::Vector2f(0.0, 1.0);
		Eigen::Vector2f lbasepnt = Eigen::Vector2f(0.0, 0.0);
		if (x_flag)
			lbasedir = Eigen::Vector2f(1.0, 0.0);
		//计算交点
		std::vector<Eigen::Vector2f> cross_pnts2d;
		if (x_flag)
		{
			//排序：使用它在x轴上的坐标进行排序
			for (int i = 0; i < lines_2d.size(); ++i)
			{
				Eigen::Vector2f line2d_dir = lines_2d[i].first;
				Eigen::Vector2f line2d_pnt = lines_2d[i].second;
				Eigen::Vector2f crosspnt = 
					ComputePlaneTwoLinesCrossPntNew(lbasepnt, lbasedir, line2d_pnt, line2d_dir);
				cross_pnts2d.emplace_back(crosspnt);
				std::cerr << "X: cross pnts: " << crosspnt[0] << " " << crosspnt[1] << std::endl;
			}
			//进行排序
			std::vector<int> resorted_idx;
			for (int i = 0; i < cross_pnts2d.size(); ++i)
			{
				resorted_idx.emplace_back(i);
			}
			for (int i = 0; i < resorted_idx.size(); ++i)
			{
				for (int j = i + 1; j < resorted_idx.size(); ++j)
				{
					float xi = cross_pnts2d[resorted_idx[i]][0];
					float xj = cross_pnts2d[resorted_idx[j]][0];
					if (xi > xj)
					{
						int tmp_idx = resorted_idx[i];
						resorted_idx[i] = resorted_idx[j];
						resorted_idx[j] = tmp_idx;
					}
				}
			}
			std::vector<int> resorted_idx_new;
			for (int i = 0; i < resorted_idx.size(); ++i)
			{
				resorted_idx_new.emplace_back(related_poly_idxs_[resorted_idx[i]]);
			}
			related_poly_idxs_ = resorted_idx_new;
		}
		else
		{
			//排序：使用它在y轴上的坐标进行排序
			for (int i = 0; i < lines_2d.size(); ++i)
			{
				Eigen::Vector2f line2d_dir = lines_2d[i].first;
				Eigen::Vector2f line2d_pnt = lines_2d[i].second;
				Eigen::Vector2f crosspnt =
					ComputePlaneTwoLinesCrossPntNew(lbasepnt, lbasedir, line2d_pnt, line2d_dir);
				cross_pnts2d.emplace_back(crosspnt);
				std::cerr << "Y: cross pnts: " << crosspnt[0] << " " << crosspnt[1] << std::endl;
			}
			//进行排序
			std::vector<int> resorted_idx;
			for (int i = 0; i < cross_pnts2d.size(); ++i)
			{
				resorted_idx.emplace_back(i);
			}
			for (int i = 0; i < resorted_idx.size(); ++i)
			{
				for (int j = i + 1; j < resorted_idx.size(); ++j)
				{
					float yi = cross_pnts2d[0][resorted_idx[i]];
					float yj = cross_pnts2d[0][resorted_idx[j]];
					if (yi > yj)
					{
						int tmp_idx = resorted_idx[i];
						resorted_idx[i] = resorted_idx[j];
						resorted_idx[j] = tmp_idx;
					}
				}
			}
			std::vector<int> resorted_idx_new;
			for (int i = 0; i < resorted_idx.size(); ++i)
			{
				resorted_idx_new.emplace_back(related_poly_idxs_[resorted_idx[i]]);
			}
			related_poly_idxs_ = resorted_idx_new;
		}

		//
		std::cerr << "end sort the tgt polygon idx " << std::endl;
		print_polygon_idx(related_poly_idxs_);
		return true;
	}

	void HWPlane::ComputeOriginPntMaxDist2Plane(float& dist_max_normal, int& idx_normal,
		float& dist_max_inverse_normal, int& idx_inverse_normal)
	{
		if (pnts_pos_origin_.empty() || corner_pnts_3d_.empty())
		{
			dist_max_normal = std::numeric_limits<float>::max();
			idx_normal = -1;
			dist_max_inverse_normal = std::numeric_limits<float>::max();
			idx_inverse_normal = -1;
			return;
		}
		
		Eigen::Vector3f plane_normal;
		plane_normal[0] = coeff_.x;
		plane_normal[1] = coeff_.y;
		plane_normal[2] = coeff_.z;
		std::cerr << "plane_normal: " << plane_normal.transpose() << std::endl;
		float dist_max_norm = std::numeric_limits<float>::lowest();
		int dist_max_norm_idx = -1;
		float dist_max_inverse_norm = std::numeric_limits<float>::lowest();
		int dist_max_inverse_norm_idx = -1;
		std::cerr << "pnts_pos_origin_ num: " << pnts_pos_origin_.size() << std::endl;
		for (int i = 0; i < pnts_pos_origin_.size(); ++i)
		{
			Eigen::Vector3f pnt_pos = Eigen::Vector3f(pnts_pos_origin_[i].x, 
				pnts_pos_origin_[i].y, pnts_pos_origin_[i].z);
			Eigen::Vector3f pnt_proj_pos;
			pnt_proj_pos = Pnt3dProjToPlane3D(pnt_pos);
			//Pnt3d2Plane3DCNDist(pnt_pos, center_pos, plane_normal);
			//LineIntersectPlanePntN(pnt_pos, plane_normal, center_pos, plane_normal, pnt_proj_pos);
			Eigen::Vector3f pnt_proj_pos_n = pnt_pos - pnt_proj_pos;
			float dist_pnt_proj_to_pnt = pnt_proj_pos_n.norm();
			//std::cerr << "i: " << i << std::endl;
			//std::cerr << "dist_pnt_proj_to_pnt: " << dist_pnt_proj_to_pnt << std::endl;
			if (dist_pnt_proj_to_pnt < 1e-6)
			{
				continue;
			}
			else
			{
				Eigen::Vector3f pnt_pos_normlize = pnt_proj_pos_n.normalized();
				if (plane_normal.dot(pnt_pos_normlize) > 0.0)
				{
					if (dist_pnt_proj_to_pnt > dist_max_norm)
					{
						dist_max_norm = dist_pnt_proj_to_pnt;
						dist_max_norm_idx = i;
					}
				}
				else
				{
					if (dist_pnt_proj_to_pnt > dist_max_inverse_norm)
					{
						dist_max_inverse_norm = dist_pnt_proj_to_pnt;
						dist_max_inverse_norm_idx = i;
					}
				}
			}
		}
		dist_max_normal = dist_max_norm;
		idx_normal = dist_max_norm_idx;
		dist_max_inverse_normal = dist_max_inverse_norm;
		idx_inverse_normal = dist_max_inverse_norm_idx;
	}

	float HWPlane::ComputeUEEnergyPnts2Plane()
	{
		/*if (pnts_pos_origin_.empty())
		{
			return 
		}*/
		float sum_value = 0.0;
		for (int i = 0; i < pnts_pos_origin_.size(); ++i)
		{
			Eigen::Vector3f tmp_pnt = Eigen::Vector3f(pnts_pos_origin_[i].x, 
				pnts_pos_origin_[i].y, pnts_pos_origin_[i].z);
			Eigen::Vector3f pnt_proj_pos;
			pnt_proj_pos = Pnt3dProjToPlane3D(tmp_pnt);
			float dist_value = (pnt_proj_pos - tmp_pnt).norm();
			sum_value += dist_value;
		}
		return sum_value;
	}

	Eigen::Matrix3f HWPlane::ComputeCovarianceMatrix(const std::vector<float3>& vertices)
	{
		//转化一下
		Eigen::Matrix3f covariance_matrix = Eigen::Matrix3f::Zero(3, 3);
		//std::cout << "initial covariance_matrix:\n" << covariance_matrix << std::endl;

		float meanx(0.0f), meany(0.0f), meanz(0.0f);
		int num_vers = vertices.size();
		for (int j = 0; j < vertices.size(); ++j)
		{
			//
			meanx += vertices[j].x;
			meany += vertices[j].y;
			meanz += vertices[j].z;
		}
		if (num_vers != 0)
		{
			meanx = meanx / num_vers;
			meany = meany / num_vers;
			meanz = meanz / num_vers;
		}

		//avg
		Eigen::Vector3f avg_vertex(meanx, meany, meanz);

		//cout avg_point
		//std::cout << "avg_point: " << avg_vertex << std::endl;

		for (int i = 0; i < num_vers; ++i)
		{
			//
			Eigen::Vector3f vertex(vertices[i].x, vertices[i].y, vertices[i].z);
			Eigen::Matrix3f multiplied_matrix = (vertex - avg_vertex)*(vertex - avg_vertex).transpose();
			//std::cout << "temp matrix: \n" << temp_matrix << std::endl;
			covariance_matrix = covariance_matrix + multiplied_matrix;
		}

		covariance_matrix = covariance_matrix / num_vers;

		return covariance_matrix;
	}

	void HWPlane::ComputeMatrixEigens(const Eigen::Matrix3f& matrix,
		Eigen::Matrix3f& eig_values, Eigen::Matrix3f& eig_vectors)
	{
		//分别计算特征向量，特征值
		Eigen::EigenSolver<Eigen::Matrix3f> es(matrix);
		eig_values = es.pseudoEigenvalueMatrix();
		eig_vectors = es.pseudoEigenvectors();
	}

	//--------------------------end fit plane----------------------//

	//-------------------------生成平面，并且采样平面---------------//
	//image
	void HWPlane::SetImageSize(int w, int h)
	{
		image_height_ = h;
		image_width_ = w;
	}

	void HWPlane::ComputeImageConvertParams()
	{
		float2 min_cor;
		float2 max_cor;

		Compute2DPntsBoxPos(min_cor, max_cor);

		std::cout << "the min cor is: " << min_cor.x << " " << min_cor.y << std::endl;
		std::cout << "the max cor is: " << max_cor.x << " " << max_cor.y << std::endl;

		if (min_cor.x < 0)
			move_x_ = std::abs(min_cor.x);
		if (min_cor.y < 0)
			move_y_ = std::abs(min_cor.y);

		float width_cor = max_cor.x - min_cor.x;
		float height_cor = max_cor.y - min_cor.y;

		//
		float diag_polygon_length = std::sqrtf(width_cor*width_cor + height_cor*height_cor);
		if (image_width_ > image_height_)
			polygon2image_scale_ = image_width_ / diag_polygon_length;
		else
			polygon2image_scale_ = image_height_ / diag_polygon_length;

		std::cout << "the scale is: " << polygon2image_scale_ << std::endl;
		std::cout << "the move x: " << move_x_ << std::endl;
		std::cout << "the move y: " << move_y_ << std::endl;
	}

	cv::Point2f HWPlane::PlaneCoordPnt2ImageCoordPnt(const Point_2d& plane_2d_pnt)
	{
		cv::Point2f pnt;
		pnt.x = (plane_2d_pnt.x() + move_x_) * polygon2image_scale_;
		pnt.y = -((plane_2d_pnt.y() + move_y_) * polygon2image_scale_)+ image_height_;

		return pnt;
	}

	Point_2d HWPlane::ImageCoordPnt2PlaneCoordPnt(const cv::Point2f& image_pnt)
	{
		if (std::abs(polygon2image_scale_) > 1e-4)
		{
			float x = image_pnt.x / polygon2image_scale_ - move_x_;
			float y = (image_height_ - image_pnt.y) / polygon2image_scale_ - move_y_;

			return Point_2d(x, y);
		}
		return Point_2d(0, 0);
	}

	//test plane
	void HWPlane::PaintEdgePntsIntoImgAndSave()
	{
#if 1
		//计算从原始的平面坐标到图像坐标转化的参数
		std::cout << "start compute convert params..." << std::endl;
		ComputeImageConvertParams();

		//opencv图片的坐标系是rows: 750, cols: 1000
		cv::Mat src(image_height_, image_width_, CV_8UC3);
		src.setTo(0);

		std::vector<cv::Point2i> img_pnts;
		for (int i = 0; i < sorted_border_pnts_2d_.size(); ++i)
		{
			cv::Point2f pnt = PlaneCoordPnt2ImageCoordPnt(sorted_border_pnts_2d_[i]);
			//std::cout<<""
			cv::Point2i img_pnt((int)pnt.x, (int)pnt.y);
			img_pnts.emplace_back(img_pnt);

			std::cout << "pnt " << i << ": " << img_pnt.x << " " << img_pnt.y << std::endl;

			image_coord_border_pnts_.push_back(Point2(pnt.x, pnt.y));
		}
		//system("pause");
		cv::polylines(src, img_pnts, true, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
#endif
	}
	//end test

	void HWPlane::GenerateEdgeImagePolygon()
	{
		//计算从原始的平面坐标到图像坐标转化的参数
		std::cout << "start compute convert params..." << std::endl;
		ComputeImageConvertParams();

		//opencv图片的坐标系是rows: 750, cols: 1000
		cv::Mat src(image_height_, image_width_, CV_8UC3);
		src.setTo(0);

		//std::vector<cv::Point2i> img_pnts;
		for (int i = 0; i < sorted_border_pnts_2d_.size(); ++i)
		{
			cv::Point2f pnt = PlaneCoordPnt2ImageCoordPnt(sorted_border_pnts_2d_[i]);
			cv::Point2i img_pnt((int)pnt.x, (int)pnt.y);
			std::cout << "pnt " << i << ": " << pnt.x << " " << pnt.y << std::endl;
			image_coord_border_pnts_.push_back(Point2(pnt.x, pnt.y));
		}
	}

	void HWPlane::TriangulatePolygon2D()
	{
		//Insert the polygons into a constrained triangulation
		//CDT cdt;
		
		cdt_.insert_constraint(image_coord_border_pnts_.vertices_begin(), image_coord_border_pnts_.vertices_end(), true);
		//Mark facets that are inside the domain bounded by the polygon
		mark_domains1(cdt_);

		int count = 0;
		for (Face_handle f : cdt_.finite_face_handles())
		{
			if (f->info().in_domain()) ++count;
		}

#ifdef LIULINGFEI
		std::cout << "There are " << count << " facets in the domain." << std::endl;
#endif

		//画出这些三角形
		//opencv图片的坐标系是rows: 1000, cols: 750
		cv::Mat src(image_height_, image_width_, CV_8UC3);
		src.setTo(0);
		std::vector<std::pair<cv::Point2f, cv::Point2f> > lines;

		std::vector<Point2> polygon_sample_pnts;

		int count1 = 0;
		for (Face_handle f : cdt_.finite_face_handles())
		{
			if (f->info().in_domain())
			{
				++count1;
				Point2 vertex0 = f->vertex(f->cw(0))->point();
				Point2 vertex1 = f->vertex(f->cw(1))->point();
				Point2 vertex2 = f->vertex(f->cw(2))->point();

				//Point2 sample_pnt = SamplePntsFromPolygon(vertex0, vertex1, vertex2);

				//画三角形网格的半边
				cv::Point2f half_vertex0(vertex0.x(), vertex0.y());
				cv::Point2f half_vertex1(vertex1.x(), vertex1.y());
				cv::Point2f half_vertex2(vertex2.x(), vertex2.y());
				cv::line(src, half_vertex0, half_vertex1, cv::Scalar(255, 255, 255), 1, CV_AA);
				cv::line(src, half_vertex1, half_vertex2, cv::Scalar(255, 255, 255), 1, CV_AA);
				cv::line(src, half_vertex2, half_vertex0, cv::Scalar(255, 255, 255), 1, CV_AA);
			}
		}

		SamplePntsFromPolygon();
		//将点画出来
		for (int i = 0; i < sample_polygon_vertices_.size(); ++i)
		{
			cv::Point2f pnt(sample_polygon_vertices_[i][0], sample_polygon_vertices_[i][1]);
			cv::circle(src, pnt, 1, cv::Scalar(255, 0, 0), -1); //第五个参数我设为-1，表明这是个实点。
		}

#ifdef LIULINGFEI
		std::cout << "the face number is: " << count1 << std::endl;
#endif
		//std::string path = "D:/vc_project_xht/back_up/test_for_plane/tri_img2.png";
		//cv::imwrite(path, src);
#ifdef LIULINGFEI
		std::cout << "end save the tri image!" << std::endl;
#endif
		//system("pause");
	}

	void HWPlane::SamplePntsFromTri(std::vector<Point2>& result_pnt,
		std::vector<Point2> tri_pnts, int point_density)
	{
		if (tri_pnts.size() != 3)
		{
			std::cout << "no tri!" << std::endl;
			return;
		}
		float area = tri_pnts[0][0] * (tri_pnts[1][1] - tri_pnts[2][1]) + tri_pnts[1][0] * (tri_pnts[2][1] - tri_pnts[0][1])
			+ tri_pnts[2][0] * (tri_pnts[0][1] - tri_pnts[1][1]);
		int points_num = abs(area * point_density);    // 点密度设置室内：10000
		float cx = (tri_pnts[0][0] + tri_pnts[1][0] + tri_pnts[2][0]) / 3;
		float cy = (tri_pnts[0][1] + tri_pnts[1][1] + tri_pnts[2][1]) / 3;
		//float cz = (tri_pnts[0][0] + tri_pnts[1][0] + tri_pnts[2][0]) / 3;

		for (int i = 0; i < 3; ++i)
		{
			int count = 0;

			//01，12，02这三个组合
			int idx1 = i, idx2 = i + 1;
			if (i == 2)
			{
				idx1 = 0;
				idx2 = 2;
			}
			srand(time(NULL));

			while (count < points_num)
			{
				float ab1 = tri_pnts[idx1][0] - cx;
				float ab2 = tri_pnts[idx1][1] - cy;
				float ac1 = tri_pnts[idx2][0] - cx;
				float ac2 = tri_pnts[idx2][1] - cy;

				float x = rand() / (RAND_MAX + 0.0);
				float y = rand() / (RAND_MAX + 0.0);
				float x1, y1;

				if (x + y > 1)
				{
					x1 = 1 - x;
					y1 = 1 - y;
				}
				else
				{
					x1 = x;
					y1 = y;
				}
				Point2 pnt(cx + ab1*x1 + ac1*y1,
					cy + ab2*x1 + ac2*y1);

				result_pnt.emplace_back(pnt);
				++count;
			}
		}
	}

	bool HWPlane::SamplePntsFromPolygon()
	{
		int max_count = 100000;
		//int iter_count = 0;
		for (Face_handle f : cdt_.finite_face_handles())
		{
			std::vector<Point2> tri_vertices;
			if (f->info().in_domain())
			{
				std::vector<Point2> tri;
				for (int i = 0; i < 3; ++i)
				{
					tri.emplace_back(f->vertex(f->ccw(i))->point());
				}
				SamplePntsFromTri(tri_vertices, tri, max_count);
				//iter_count++;
			}

			//存到polygon对象里面
			for (int i = 0; i < tri_vertices.size(); ++i)
			{
				sample_polygon_vertices_.emplace_back(tri_vertices[i]);
			}

			/*if (iter_count > 4)
				break;*/
		}
		//liulingfei
		return true;
	}

	//单纯用于测试均匀采样顶点的代码
	void HWPlane::SamplePntsFromTriTest()
	{
		//opencv图片的坐标系是rows: 1000, cols: 750
		cv::Mat src(image_width_, image_height_, CV_8UC3);
		src.setTo(0);
		//设置一个三角形顶点
		Point2 pnt0(10, 10);
		Point2 pnt1(600, 10);
		Point2 pnt2(10, 600);
		std::vector<Point2> result_pnts;
		std::vector<Point2> tri_pnts;
		tri_pnts.emplace_back(pnt0);
		tri_pnts.emplace_back(pnt1);
		tri_pnts.emplace_back(pnt2);
		SamplePntsFromTri(result_pnts, tri_pnts, 5000);

		//先画边
		//画三角形网格的半边
		cv::Point2f half_vertex0(pnt0.x(), pnt0.y());
		cv::Point2f half_vertex1(pnt1.x(), pnt1.y());
		cv::Point2f half_vertex2(pnt2.x(), pnt2.y());
		cv::line(src, half_vertex0, half_vertex1, cv::Scalar(255, 255, 255), 1, CV_AA);
		cv::line(src, half_vertex1, half_vertex2, cv::Scalar(255, 255, 255), 1, CV_AA);
		cv::line(src, half_vertex2, half_vertex0, cv::Scalar(255, 255, 255), 1, CV_AA);

		//画采样点
		for(int i = 0; i < result_pnts.size(); ++i)
		{
			cv::Point2f pnt(result_pnts[i][0], result_pnts[i][1]);
			cv::circle(src, pnt, 1, cv::Scalar(255, 0, 0), -1); //第五个参数我设为-1，表明这是个实点。
		}

		//std::string path = "D:/vc_project_xht/back_up/test_for_plane/line_img_test.png";
		//cv::imwrite(path, src);
	}

	//-------------------------生成平面，并且采样平面---------------//
}
