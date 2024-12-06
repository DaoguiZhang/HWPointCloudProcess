#pragma once
#include<iostream>
#include<string>
#include<vector>
#include<fstream>
#include<Eigen\Core>
#include"model_cameras.h"

struct LinePoint {

	Eigen::Vector3f pnt_;
	std::vector<int> pnt2ls_;
	LinePoint() {}
	LinePoint(Eigen::Vector3f& pnt, std::vector<int>& pnt2ls)
	{
		pnt_ = pnt;
		pnt2ls_ = pnt2ls;
	}
	/*LinePoint(LinePoint& other)
	{
		pnt_ = other.pnt_;
		pnt2ls_ = other.pnt2ls_;
	}*/
};

struct PolygonLineId
{
	std::vector<int> Plane2Linesidx;
};

enum SceneLayDimType {
	k2DType,
	k3DType,
	KDotherType
};

class SceneLayout 
{
public:
	SceneLayout() 
	{
		ly_type_dimension_ = SceneLayDimType::k2DType;
	};

	//SceneLayout(SceneLayout& other);

	~SceneLayout() {};

	/*
	目的：设置layout的类型（主要是2d和3d的两种类型）
	输入：my_t
	*/
	void SetSceneLayoutType(SceneLayDimType& my_t);

	/*
	目的：获取layout的类型（主要是2d和3d的两种类型）
	输入：无
	*/
	SceneLayDimType GetSceneLayoutType();

	/*
	目的：获取layout的类型（主要是2d和3d的两种类型）
	输入：无
	*/
	void SetRemainedImgLinesIdx(std::vector<int>& remained_idxs);

	/*
	目的：提供接口，单个polygon的获取layout的线段（3D）
	输入：p0p1, p1p2, p2p3....
	*/
	void SetSceneLayout3D(std::vector<Eigen::Vector3f>& pnts);

	/*
	目的:设置polygon的idx
	*/
	void SetPlanePolygonsVecIdxs(std::vector<int>& idxs);

	/*
	目的:设置相机的idx
	*/
	void SetCamsVecIdxs(int idxs);

	/*
	目的：设置路径名
	*/
	void SetLayoutName(std::string& lyout_path);

	/*
	目的：提供接口，多个polygon的获取layout的线段（3D）
	输入：p0p1, p1p2, p2p3....
	*/
	void SetAllPolygonsSceneLayout3D(std::vector<std::vector<Eigen::Vector3f> >& pnts);

	/*
	目的：提供接口，获取layout的线段（2D）
	输入：p0p1, p1p2, p2p3....
	*/
	void SetSceneLayout2D(std::vector<Eigen::Vector2f>& pnts);

	void SetSceneLayout2DFromNetPnts(std::vector<std::vector<float> >& layout_pnts);

	void ReadLayout3D(const std::string& path);

	/*
	目的：直接获取log的线段
	*/
	void ReadLayout2D(const std::string& path);

	/*
	目的：直接读取线段，log文件
	*/
	bool ReadLayout2DFromNetPath(const std::string& path);
	
	void GetPickedLineSeg(int idx, Eigen::Vector3f& start_p, Eigen::Vector3f& end_p);
	
	void GetPickedLineSegSamplePnts(int idx, std::vector<Eigen::Vector2i>& lseg_pnts);

	//convert lines info to line pnts
	std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f> > GetAllLineSegsPnts();

	//将多个线段进行采样，然后获取线段上的顶点，组成集合
	void SamplePntsFromAllLineEndPnts();

	//在某一个线段上采样顶点，组成顶点集合
	void SamplePntsFromALineEndPnts(Eigen::Vector3f& s, Eigen::Vector3f& e, std::vector<Eigen::Vector2i>& img_pnts);

	int LinesSize();

	//判断两个线段是否为同一线段
	bool CheckSameLines(Eigen::Vector2f s0, Eigen::Vector2f e0, Eigen::Vector2f s1, Eigen::Vector2f e1, float threshold);

	//判断顶点是否为同一个顶点
	bool CheckSamePnts(Eigen::Vector2f s, Eigen::Vector2f e, float threshold);

	//找到对应的idx
	int FindVecIdx(std::vector<Eigen::Vector2f>& vec, Eigen::Vector2f s);

	/*
	目的：获取layout的路径
	*/
	const std::string& GetLayoutPath();

	//
	std::vector<LinePoint>& GetLayoutPnts();
	std::vector<Eigen::Vector2i>& GetLayoutLine2PntIdx();

	void UpdateRedauntLineSegments();

	/*
	目的：debug读取的数据world lines的数据是否正确
	*/
	void PrintLayoutDebug();

	/*
	目的：debug读取的数据world lines的数据是否正确
	*/
	void Write3DLinesIntoObj(const std::string& path);

	/*
	目的：debug读取的数据某个world line的数据是否正确
	*/
	void WriteSelected3DLineIntoObj(Eigen::Vector3f& s, Eigen::Vector3f& e, const std::string& path);

	/*
	运算符重载
	*/
	SceneLayout operator=(SceneLayout& other);

	SceneLayDimType ly_type_dimension_;

	//这是将线段进行聚类，同一个平面在一起，其中第一表示第一个平面polygon，第二个表示第二个平面polygon
	std::vector<PolygonLineId> planes_polygon_lines_idx_;
	std::vector<int> associated_polys_idxs_;	//单个图片,图片中那些polygon能看见,它用于后续的平面相交

	std::vector<int> remained_img_lines_segs_idxs_;	//删除多余的线段，剩下来的线段idx
	
	std::string layout_path_;

	int ly_cam_idx_ = -1;
	//保存顶点
	std::vector<LinePoint> pnts_;
	
	//保存顶点的索引（start idx和end idx）
	std::vector<Eigen::Vector2i> lines_segs_;
	std::vector<std::vector<Eigen::Vector2i> > img_lines_pnts_;	//线段上的裂化出来的顶点（它是在图像线段上采样的顶点集合）和lines_segs_进行对应

private:

	bool ReadLayout2DFromNetData(const std::string& path, std::vector<std::vector<float> >& lines_pnts);

	std::string ReplacePatternString(std::string& str, const std::string& old_str, const std::string& new_str);

	/*
	目的：去掉路径dir和文件名的后缀
	*/
	std::string ExtractFileNameNosufixNodir(std::string path);
};