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
	Ŀ�ģ���ȡ��ά�߶ε�layout������polygon�Ķ���������ݽ���
	*/
	void SetWorldLyoutFromPolygonWindow(std::vector<std::vector<Eigen::Vector3f> >& lyout);

	ModelCameras* GetCamsData();
	std::vector<SceneLayout>& GetCamSceneLyData();
	
	/*
	Ŀ�ģ���ȡ��img_idx��layout ��l_idx���߶ε������˵� 
	*/
	bool GetImgLayoutLineSeg(int img_idx, int l_idx, Eigen::Vector3f& ls, Eigen::Vector3f& le);

	/*
	Ŀ�ģ���ȡ��img_idx��layout ��l_idx���߶β����Ķ���
	*/
	bool GetImgLayoutLineSegSamplePnts(int img_idx, int l_idx, std::vector<Eigen::Vector2i>& line_sample_pnts);

	/*
	Ŀ�ģ���ȡimg_idx ��layout ���е��߶�
	*/
	bool GetPickedImgLayoutLineSegs(int img_idx, 
		std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f> >& layout_pnts);

	/*
	Ŀ�ģ�debug, ��ʾ��ȡ���߶κ�ͼ���Ƿ���ȷ
	*/
	void ShowPickedImgLines(int idx);

	/*
	Ŀ�ģ�debug, ��ʾ��ȡ���߶κ�ͼ���Ƿ���ȷ
	*/
	void SaveAllImgsLines();

	/*
	Ŀ�ģ���������ϵ�Ķ���ת��Ϊ��Ӧ�������ͼ������ϵ�Ķ���
	*/
	Eigen::Vector2f WorldPointProjToPickedImg(Eigen::Vector3f& p3d, int img_idx);

	/*
	Ŀ�ģ���ʾ����������ϵ��ȡ��lyͶӰ��ͼ���ϵ��߶�
	*/
	void ShowWorldLyProjectLine2PickedImg(int idx);

	/*
	Ŀ�ģ�����������ϵ��ȡ��ly���е��߶�ͶӰ��ͼ���ϵ��߶�,���仭��img��
	*/
	void DrawAllWorldLyProjectLine2PickedImg(int idx, cv::Mat& img);

	/*
	Ŀ�ģ���ԭʼ���߶��϶�ȡ�߶�,���仭��img��
	*/
	void DrawOriginCamLineSegPickedImg(int idx, cv::Mat& img);

	/*
	Ŀ�ģ���ʾ����������ϵ��ȡ��ly���е��߶�ͶӰ��ͼ���ϵ��߶�,����ʾ����
	*/
	void ShowAllWorldLyProjLyAndOriginLinesPickedImg(int idx);

	/*
	Ŀ�ģ���ȡworld layout������ͼ���Ͻ���Ͷ��
	*/
	void RunWorldLy2AllCamsImgLy();

	/*
	Ŀ�ģ�Ͷ����������ϵ��layout�������ͼ����,����ͼ���layout
	*/
	void ComputeWorldly2SelectedImgLy(int cam_idx, SceneLayout& scene2d_ly, Ly3DToLy2D& wly2cly);

	/*
	Ŀ�ģ�world proj���߶κ��������ϵ�µ��߶ν���ƥ�ԣ�ѡȡ��Ӧ���߶�
	*/
	bool GetImglineCorrespondingWorldLine2D(Eigen::Vector3f& s, Eigen::Vector3f& e, 
		cv::Mat& img, cv::Mat& depth_img, CameraModel& cam,
		std::vector<Eigen::Vector2f>& lseg_pnts);

	/*
	Ŀ�ģ�3d���߶�ͶӰ����Ӧ��ͼ����
	���룺cam_idx
	���룺ͶӰ��ͼ����߶ε������˵�
	*/
	void LineSeg2dProj2PickedCam(int cam_idx, Eigen::Vector3f& ls3d, Eigen::Vector3f& le3d,
		Eigen::Vector2f& ls2d, Eigen::Vector2f& le2d);

	/*
	Ŀ�ģ���ͼ���ϣ����߶��ϳ�ȡ����, �߶��������˵����
	*/
	std::vector<cv::Point2i> ExtractPntsFromLineSegInImgCoord(Eigen::Vector2f& ls, Eigen::Vector2f& le, cv::Mat& img);

	/*
	Ŀ�ģ���ȡ�������͵Ķ���
	*/
	Eigen::Vector2i ConvertPntf2Pnti(Eigen::Vector2f& pnt);

	/*
	������ͼ���ϣ��������ǵĶ���, ��Щ�߶ε������Ѿ���ͼ��������, 
	ʹ��Bresenham���ٻ�ֱ���㷨�����Թȸ裩
	*/
	void ExtractPnts2DFromLineSegInImg(Eigen::Vector2i& ls, Eigen::Vector2i& le, 
		Eigen::Vector2i& mincor, Eigen::Vector2i& maxcor, std::vector<cv::Point2i>& pnts2d);

	/*
	Ŀ�ģ������ε��߶ηֿ�Ϊ4���߶�
	*/
	void SplitRectInto4LineSeg2D(Eigen::Vector2f& min_cor, Eigen::Vector2f& max_cor, std::vector<Eigen::Vector2f>& linesCors);

	/*
	Ŀ�ģ������߶εľ���, l_sԭʼ�߶Σ�
	*/
	void LineSeg2LineSegDist0(Eigen::Vector2f& ls_s, Eigen::Vector2f& ls_e, Eigen::Vector2f& lt_s, Eigen::Vector2f& lt_e);

	/*
	Ŀ�ģ����������������߶εľ���
	*/
	float LineSeg2LineSegDist(Eigen::Vector2f& ls_s, Eigen::Vector2f& ls_e,
		Eigen::Vector2f& lt_s, Eigen::Vector2f& lt_e, cv::Mat& img);

	/*
	Ŀ�ģ�������ͼ������ϵ�£������߶εľ���,
	*/
	float LineSeg2LineSegDistImgCoord(Eigen::Vector2f& ls_s, Eigen::Vector2f& ls_e,
		Eigen::Vector2f& lt_s, Eigen::Vector2f& lt_e, cv::Mat& img);

	/*
	Ŀ�ģ�������ͼ������ϵ�£������߶εļн�,cos(l1,l2)��ֵ
	*/
	float LineSeg2LineSegCrossValueImgCoord(Eigen::Vector2f& ls_s, Eigen::Vector2f& ls_e,
		Eigen::Vector2f& lt_s, Eigen::Vector2f& lt_e);

	/*
	Ŀ�ģ�������ͼ������ϵ�£������߶εľ���(���������ض��㵽ֱ�ߵ�ƽ������)
	*/
	float LineSeg2LineSegDistPntsImgCoord(Eigen::Vector2f& ls_s, Eigen::Vector2f& ls_e,
		Eigen::Vector2f& lt_s, Eigen::Vector2f& lt_e, cv::Mat& img);


	/*
	Ŀ�ģ����������ȡ���� l_s, l_e,�����˵㣬��ȡ����f2d: f2d[0]*x + f2d[1]*y+f2d[2]=0;
	A:{a,b; c,d}X=D; ��(x0,x1);(x2,x3)����
	*/
	bool ComputeFunctionFromTwoPnts(Eigen::Vector2f& l_s, Eigen::Vector2f& l_e, Eigen::Vector3f& f2d);

	/*
	Ŀ�ģ����Ʊ�׼�����������߶��Ƿ��������ı�׼
	*/
	float Compute2Line2DScore(Eigen::Vector2f& ss, Eigen::Vector2f& se, Eigen::Vector2f& ts, Eigen::Vector2f& te);

	/*
	Ŀ�ģ���pnt2d��ֱ�ߵľ��� f_2d[0]*x + f_2d[1]*y+f_2d[2]=0;
	*/
	float Pnt2dToLineDist(Eigen::Vector2f pnt2d, Eigen::Vector3f f_2d);

	/*
	Ŀ�ģ���pnt2d���߶�(ls le)�ľ��� f_2d[0]*x + f_2d[1]*y+f_2d[2]=0;
	*/
	float Pnt2dToLineSegDist(Eigen::Vector2f& pnt2d, Eigen::Vector2f& ls, Eigen::Vector2f& le);

	/*
	Ŀ�ģ�����㵽��ľ���
	*/
	float Pnt2d2Pnt2dDist(Eigen::Vector2f& pnt0, Eigen::Vector2f& pnt1);

	/*
	Ŀ�ģ�����㵽ֱ�ߵ�ͶӰ��,proj_pnt2d�б���
	*/
	void Pnt2dProj2LineSeg(Eigen::Vector2f& pnt2d, Eigen::Vector2f& ls, Eigen::Vector2f& le,
		Eigen::Vector2f& proj_pnt2d);

	/*
	Ŀ�ģ��ж��߶��Ƿ�;����ཻ
	*/
	bool CheckLineSegIntersectRect(Eigen::Vector2f& ls, Eigen::Vector2f& le, Eigen::Vector2f& topleft, Eigen::Vector2f& bottomright);

	/*
	Ŀ�ģ��ж��߶��Ƿ�;����ཻ,����ཻ������cross_pnt����
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
	Ŀ�ģ��ж϶����Ƿ��ھ����ڲ�
	*/
	bool CheckPoint2dInRect(Eigen::Vector2f& pnt, Eigen::Vector2f& mincor, Eigen::Vector2f& maxcor);

	/*
	Ŀ�ģ��ж϶����Ƿ����߶ε����-1,�Ҳ�Ϊ1�����߶���Ϊ0
	*/
	bool Point2dAtLineSegLeftRight(Eigen::Vector2f& pnt, Eigen::Vector2f& ls, Eigen::Vector2f& le);

	/*
	Ŀ�ģ��ж������߶��Ƿ��ཻ���Լ��ཻ�Ķ���(ss,se)��(ts,te)
	*/
	bool CheckTwoLinesSegIntersect(Eigen::Vector2f& ss, Eigen::Vector2f& se, 
		Eigen::Vector2f& ts, Eigen::Vector2f& te);

	/*
	Ŀ�ģ��ж������߶��Ƿ��ཻ���Լ��ཻ�Ķ���(ss,se)��(ts,te)
	*/
	Eigen::Vector2f ComputeTwoLinesCrossPnt(Eigen::Vector2f& ss, Eigen::Vector2f& se,
		Eigen::Vector2f& ts, Eigen::Vector2f& te);

	/*
	Ŀ�ģ��ж�һ���߶��Ƿ���ͼ�����ҵ�����ı��Լ�����֮��Ĺ�ϵ
	*/
	float FindNearestSegFromSelected2DLayout(int img_idx, 
		Eigen::Vector2f& ss, Eigen::Vector2f& se, Eigen::Vector2i& cmpidx);

	/*
	Ŀ�ģ��ж�һ���߶��Ƿ���ͼ�����ҵ�����ı��Լ�����֮��Ĺ�ϵ
	*/
	float FindPntNearestSegFromSelected2DLayout(int img_idx,
		Eigen::Vector2f& ss, Eigen::Vector2i& cmpidx);

	/*
	Ŀ�ģ�debug��ȡ������world lines�������Ƿ���ȷ
	*/
	void WriteWorldSceneLinesIntoObj(const std::string& path);

	//
	std::vector<HW::HWPlane*> associated_planes_polygons_;

	//data
	SceneLayout scene_world_ly_data_;

	//�����scene_world_ly_data_Ͷ�䵽�������ϵͼƬ�ϵ�layout�������
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