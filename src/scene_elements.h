#pragma once
#include<iostream>
#include<string>
#include<vector>
#include<fstream>
#include<Eigen\Core>
#include"model_cameras.h"
#include"scene_layout.h"
#include"opencv/cv.h"
#include"hw_polygon.h"

struct Ly3DToLy2D
{
	std::vector<std::pair<int, int> > L3ToL2Vec;
};

struct LineSegPnts2D {
	Eigen::Vector2f ls;
	Eigen::Vector2f le;
};

class SceneElements{
public:
	SceneElements();
	~SceneElements();
	void SetDir(std::string dir);

	void LoadSceneElements();

	void UpdateAllScenesLayoutRedauntLinesIdxs();

	const std::string& GetSceneElementsDir();

	void SamplePntsInLinesFromImgCoord();

	void SetLineScoreThreshold(std::string& sc);

	std::string GetLineScoreThreshold();

	void SetHWPlanesVec(std::vector<HW::HWPlane*>& plane_vec);

	/*
	目的：读取三维线段的layout，它和polygon的对象进行数据交流
	*/
	void SetWorldLyoutFromPolygonWindow(std::vector<std::vector<Eigen::Vector3f> >& lyout);

	ModelCameras* GetCamsData();
	std::vector<SceneLayout>& GetCamSceneLyData();
	
	/*
	目的：获取第img_idx的layout 第l_idx个线段的两个端点 
	*/
	bool GetImgLayoutLineSeg(int img_idx, int l_idx, Eigen::Vector3f& ls, Eigen::Vector3f& le);

	/*
	目的：获取第img_idx的layout 第l_idx个线段采样的顶点
	*/
	bool GetImgLayoutLineSegSamplePnts(int img_idx, int l_idx, std::vector<Eigen::Vector2i>& line_sample_pnts);

	/*
	目的：获取img_idx 的layout 所有的线段
	*/
	bool GetPickedImgLayoutLineSegs(int img_idx, 
		std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f> >& layout_pnts);

	/*
	目的：debug, 显示读取的线段和图像是否正确
	*/
	void ShowPickedImgLines(int idx);

	/*
	目的：debug, 显示读取的线段和图像是否正确
	*/
	void SaveAllImgsLines();

	/*
	目的：世界坐标系的顶点转化为对应的相机的图像坐标系的顶点
	*/
	Eigen::Vector2f WorldPointProjToPickedImg(Eigen::Vector3f& p3d, int img_idx);

	/*
	目的：显示从世界坐标系读取的ly投影到图像上的线段
	*/
	void ShowWorldLyProjectLine2PickedImg(int idx);

	/*
	目的：从世界坐标系读取的ly所有的线段投影到图像上的线段,将其画到img上
	*/
	void DrawAllWorldLyProjectLine2PickedImg(int idx, cv::Mat& img);

	/*
	目的：从原始的线段上读取线段,将其画到img上
	*/
	void DrawOriginCamLineSegPickedImg(int idx, cv::Mat& img);

	/*
	目的：显示从世界坐标系读取的ly所有的线段投影到图像上的线段,并显示出来
	*/
	void ShowAllWorldLyProjLyAndOriginLinesPickedImg(int idx);

	/*
	目的：获取world layout到各个图像上进行投射
	*/
	void RunWorldLy2AllCamsImgLy();

	/*
	目的：投射世界坐标系的layout到相机的图像上,生成图像的layout
	*/
	void ComputeWorldly2SelectedImgLy(int cam_idx, SceneLayout& scene2d_ly, Ly3DToLy2D& wly2cly);

	/*
	目的：world proj的线段和相机坐标系下的线段进行匹对，选取对应的线段
	*/
	bool GetImglineCorrespondingWorldLine2D(Eigen::Vector3f& s, Eigen::Vector3f& e, 
		cv::Mat& img, cv::Mat& depth_img, CameraModel& cam,
		std::vector<Eigen::Vector2f>& lseg_pnts);

	/*
	目的：3d的线段投影到相应的图像上
	输入：cam_idx
	输入：投影到图像的线段的两个端点
	*/
	void LineSeg2dProj2PickedCam(int cam_idx, Eigen::Vector3f& ls3d, Eigen::Vector3f& le3d,
		Eigen::Vector2f& ls2d, Eigen::Vector2f& le2d);

	/*
	目的：在图像上，从线段上抽取顶点, 线段有两个端点组成
	*/
	std::vector<cv::Point2i> ExtractPntsFromLineSegInImgCoord(Eigen::Vector2f& ls, Eigen::Vector2f& le, cv::Mat& img);

	/*
	目的：获取整数类型的顶点
	*/
	Eigen::Vector2i ConvertPntf2Pnti(Eigen::Vector2f& pnt);

	/*
	顶点在图像上，计算它们的顶点, 这些线段的坐标已经在图像里面了, 
	使用Bresenham快速画直线算法（可以谷歌）
	*/
	void ExtractPnts2DFromLineSegInImg(Eigen::Vector2i& ls, Eigen::Vector2i& le, 
		Eigen::Vector2i& mincor, Eigen::Vector2i& maxcor, std::vector<cv::Point2i>& pnts2d);

	/*
	目的：将矩形的线段分开为4个线段
	*/
	void SplitRectInto4LineSeg2D(Eigen::Vector2f& min_cor, Eigen::Vector2f& max_cor, std::vector<Eigen::Vector2f>& linesCors);

	/*
	目的：两个线段的距离, l_s原始线段，
	*/
	void LineSeg2LineSegDist0(Eigen::Vector2f& ls_s, Eigen::Vector2f& ls_e, Eigen::Vector2f& lt_s, Eigen::Vector2f& lt_e);

	/*
	目的：能量函数的两个线段的距离
	*/
	float LineSeg2LineSegDist(Eigen::Vector2f& ls_s, Eigen::Vector2f& ls_e,
		Eigen::Vector2f& lt_s, Eigen::Vector2f& lt_e, cv::Mat& img);

	/*
	目的：计算在图像坐标系下，两个线段的距离,
	*/
	float LineSeg2LineSegDistImgCoord(Eigen::Vector2f& ls_s, Eigen::Vector2f& ls_e,
		Eigen::Vector2f& lt_s, Eigen::Vector2f& lt_e, cv::Mat& img);

	/*
	目的：计算在图像坐标系下，两个线段的夹角,cos(l1,l2)的值
	*/
	float LineSeg2LineSegCrossValueImgCoord(Eigen::Vector2f& ls_s, Eigen::Vector2f& ls_e,
		Eigen::Vector2f& lt_s, Eigen::Vector2f& lt_e);

	/*
	目的：计算在图像坐标系下，两个线段的距离(单纯的像素顶点到直线的平均距离)
	*/
	float LineSeg2LineSegDistPntsImgCoord(Eigen::Vector2f& ls_s, Eigen::Vector2f& ls_e,
		Eigen::Vector2f& lt_s, Eigen::Vector2f& lt_e, cv::Mat& img);


	/*
	目的：从两个点获取函数 l_s, l_e,两个端点，获取函数f2d: f2d[0]*x + f2d[1]*y+f2d[2]=0;
	A:{a,b; c,d}X=D; 将(x0,x1);(x2,x3)带入
	*/
	bool ComputeFunctionFromTwoPnts(Eigen::Vector2f& l_s, Eigen::Vector2f& l_e, Eigen::Vector3f& f2d);

	/*
	目的：定制标准，就是两个线段是否距离最近的标准
	*/
	float Compute2Line2DScore(Eigen::Vector2f& ss, Eigen::Vector2f& se, Eigen::Vector2f& ts, Eigen::Vector2f& te);

	/*
	目的：点pnt2d到直线的距离 f_2d[0]*x + f_2d[1]*y+f_2d[2]=0;
	*/
	float Pnt2dToLineDist(Eigen::Vector2f pnt2d, Eigen::Vector3f f_2d);

	/*
	目的：点pnt2d到线段(ls le)的距离 f_2d[0]*x + f_2d[1]*y+f_2d[2]=0;
	*/
	float Pnt2dToLineSegDist(Eigen::Vector2f& pnt2d, Eigen::Vector2f& ls, Eigen::Vector2f& le);

	/*
	目的：计算点到点的距离
	*/
	float Pnt2d2Pnt2dDist(Eigen::Vector2f& pnt0, Eigen::Vector2f& pnt1);

	/*
	目的：计算点到直线的投影点,proj_pnt2d中保存
	*/
	void Pnt2dProj2LineSeg(Eigen::Vector2f& pnt2d, Eigen::Vector2f& ls, Eigen::Vector2f& le,
		Eigen::Vector2f& proj_pnt2d);

	/*
	目的：判断线段是否和矩形相交
	*/
	bool CheckLineSegIntersectRect(Eigen::Vector2f& ls, Eigen::Vector2f& le, Eigen::Vector2f& topleft, Eigen::Vector2f& bottomright);

	/*
	目的：判断线段是否和矩形相交,如果相交则计算出cross_pnt坐标
	*/
	int LineSegIntersectRectPnt(Eigen::Vector2f& ls, Eigen::Vector2f& le,
		Eigen::Vector2f& topleft, Eigen::Vector2f& bottomright, Eigen::Vector2f& cross_pnt0, Eigen::Vector2f& cross_pnt1);

	bool ComputeLineSegCrossLine2dNew(Eigen::Vector2f& ls, Eigen::Vector2f& le,
		Eigen::Vector2f& t_pnt, Eigen::Vector2f& t_dir, Eigen::Vector2f& cross_pnt);

	bool ComputeTwoLineSegsCrossPnt2d(Eigen::Vector2f& ls, Eigen::Vector2f& le,
		Eigen::Vector2f& ts, Eigen::Vector2f& te, Eigen::Vector2f& cross_pnt);

	Eigen::Vector2f ComputeTwoLinesCrossPnt2dNew(Eigen::Vector2f& sp, Eigen::Vector2f& sdir,
		Eigen::Vector2f& tp, Eigen::Vector2f& tdir);

	/*
	目的：判断顶点是否在矩形内部
	*/
	bool CheckPoint2dInRect(Eigen::Vector2f& pnt, Eigen::Vector2f& mincor, Eigen::Vector2f& maxcor);

	/*
	目的：判断顶点是否在线段的左侧-1,右侧为1，在线段上为0
	*/
	bool Point2dAtLineSegLeftRight(Eigen::Vector2f& pnt, Eigen::Vector2f& ls, Eigen::Vector2f& le);

	/*
	目的：判断两条线段是否相交，以及相交的顶点(ss,se)和(ts,te)
	*/
	bool CheckTwoLinesSegIntersect(Eigen::Vector2f& ss, Eigen::Vector2f& se, 
		Eigen::Vector2f& ts, Eigen::Vector2f& te);

	/*
	目的：判断两条线段是否相交，以及相交的顶点(ss,se)和(ts,te)
	*/
	Eigen::Vector2f ComputeTwoLinesCrossPnt(Eigen::Vector2f& ss, Eigen::Vector2f& se,
		Eigen::Vector2f& ts, Eigen::Vector2f& te);

	/*
	目的：判断一个线段是否在图像上找到最近的边以及他们之间的关系
	*/
	float FindNearestSegFromSelected2DLayout(int img_idx, 
		Eigen::Vector2f& ss, Eigen::Vector2f& se, Eigen::Vector2i& cmpidx);

	/*
	目的：判断一个线段是否在图像上找到最近的边以及他们之间的关系
	*/
	float FindPntNearestSegFromSelected2DLayout(int img_idx,
		Eigen::Vector2f& ss, Eigen::Vector2i& cmpidx);

	/*
	目的：debug读取的数据world lines的数据是否正确
	*/
	void WriteWorldSceneLinesIntoObj(const std::string& path);

	//
	std::vector<HW::HWPlane*> associated_planes_polygons_;

	//data
	SceneLayout scene_world_ly_data_;

	//保存从scene_world_ly_data_投射到相机坐标系图片上的layout的坐标点
	std::vector<SceneLayout> scene_cams_proj_ly_data_;
	
	//
	std::vector<Ly3DToLy2D> scene3D2scene2D_;

private:

	static bool CompareHWStrPairByIdxSceneCmp(const std::pair<int, std::string>& a,
		const std::pair<int, std::string>& b);

	ModelCameras model_cam_data_;
	std::vector<SceneLayout> scene_cams_ly_data_;
	std::string s_dir_;
	std::string line_score_threshold_;
};