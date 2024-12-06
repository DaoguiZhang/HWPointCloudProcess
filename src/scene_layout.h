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
	Ŀ�ģ�����layout�����ͣ���Ҫ��2d��3d���������ͣ�
	���룺my_t
	*/
	void SetSceneLayoutType(SceneLayDimType& my_t);

	/*
	Ŀ�ģ���ȡlayout�����ͣ���Ҫ��2d��3d���������ͣ�
	���룺��
	*/
	SceneLayDimType GetSceneLayoutType();

	/*
	Ŀ�ģ���ȡlayout�����ͣ���Ҫ��2d��3d���������ͣ�
	���룺��
	*/
	void SetRemainedImgLinesIdx(std::vector<int>& remained_idxs);

	/*
	Ŀ�ģ��ṩ�ӿڣ�����polygon�Ļ�ȡlayout���߶Σ�3D��
	���룺p0p1, p1p2, p2p3....
	*/
	void SetSceneLayout3D(std::vector<Eigen::Vector3f>& pnts);

	/*
	Ŀ��:����polygon��idx
	*/
	void SetPlanePolygonsVecIdxs(std::vector<int>& idxs);

	/*
	Ŀ��:���������idx
	*/
	void SetCamsVecIdxs(int idxs);

	/*
	Ŀ�ģ�����·����
	*/
	void SetLayoutName(std::string& lyout_path);

	/*
	Ŀ�ģ��ṩ�ӿڣ����polygon�Ļ�ȡlayout���߶Σ�3D��
	���룺p0p1, p1p2, p2p3....
	*/
	void SetAllPolygonsSceneLayout3D(std::vector<std::vector<Eigen::Vector3f> >& pnts);

	/*
	Ŀ�ģ��ṩ�ӿڣ���ȡlayout���߶Σ�2D��
	���룺p0p1, p1p2, p2p3....
	*/
	void SetSceneLayout2D(std::vector<Eigen::Vector2f>& pnts);

	void SetSceneLayout2DFromNetPnts(std::vector<std::vector<float> >& layout_pnts);

	void ReadLayout3D(const std::string& path);

	/*
	Ŀ�ģ�ֱ�ӻ�ȡlog���߶�
	*/
	void ReadLayout2D(const std::string& path);

	/*
	Ŀ�ģ�ֱ�Ӷ�ȡ�߶Σ�log�ļ�
	*/
	bool ReadLayout2DFromNetPath(const std::string& path);
	
	void GetPickedLineSeg(int idx, Eigen::Vector3f& start_p, Eigen::Vector3f& end_p);
	
	void GetPickedLineSegSamplePnts(int idx, std::vector<Eigen::Vector2i>& lseg_pnts);

	//convert lines info to line pnts
	std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f> > GetAllLineSegsPnts();

	//������߶ν��в�����Ȼ���ȡ�߶��ϵĶ��㣬��ɼ���
	void SamplePntsFromAllLineEndPnts();

	//��ĳһ���߶��ϲ������㣬��ɶ��㼯��
	void SamplePntsFromALineEndPnts(Eigen::Vector3f& s, Eigen::Vector3f& e, std::vector<Eigen::Vector2i>& img_pnts);

	int LinesSize();

	//�ж������߶��Ƿ�Ϊͬһ�߶�
	bool CheckSameLines(Eigen::Vector2f s0, Eigen::Vector2f e0, Eigen::Vector2f s1, Eigen::Vector2f e1, float threshold);

	//�ж϶����Ƿ�Ϊͬһ������
	bool CheckSamePnts(Eigen::Vector2f s, Eigen::Vector2f e, float threshold);

	//�ҵ���Ӧ��idx
	int FindVecIdx(std::vector<Eigen::Vector2f>& vec, Eigen::Vector2f s);

	/*
	Ŀ�ģ���ȡlayout��·��
	*/
	const std::string& GetLayoutPath();

	//
	std::vector<LinePoint>& GetLayoutPnts();
	std::vector<Eigen::Vector2i>& GetLayoutLine2PntIdx();

	void UpdateRedauntLineSegments();

	/*
	Ŀ�ģ�debug��ȡ������world lines�������Ƿ���ȷ
	*/
	void PrintLayoutDebug();

	/*
	Ŀ�ģ�debug��ȡ������world lines�������Ƿ���ȷ
	*/
	void Write3DLinesIntoObj(const std::string& path);

	/*
	Ŀ�ģ�debug��ȡ������ĳ��world line�������Ƿ���ȷ
	*/
	void WriteSelected3DLineIntoObj(Eigen::Vector3f& s, Eigen::Vector3f& e, const std::string& path);

	/*
	���������
	*/
	SceneLayout operator=(SceneLayout& other);

	SceneLayDimType ly_type_dimension_;

	//���ǽ��߶ν��о��࣬ͬһ��ƽ����һ�����е�һ��ʾ��һ��ƽ��polygon���ڶ�����ʾ�ڶ���ƽ��polygon
	std::vector<PolygonLineId> planes_polygon_lines_idx_;
	std::vector<int> associated_polys_idxs_;	//����ͼƬ,ͼƬ����Щpolygon�ܿ���,�����ں�����ƽ���ཻ

	std::vector<int> remained_img_lines_segs_idxs_;	//ɾ��������߶Σ�ʣ�������߶�idx
	
	std::string layout_path_;

	int ly_cam_idx_ = -1;
	//���涥��
	std::vector<LinePoint> pnts_;
	
	//���涥���������start idx��end idx��
	std::vector<Eigen::Vector2i> lines_segs_;
	std::vector<std::vector<Eigen::Vector2i> > img_lines_pnts_;	//�߶��ϵ��ѻ������Ķ��㣨������ͼ���߶��ϲ����Ķ��㼯�ϣ���lines_segs_���ж�Ӧ

private:

	bool ReadLayout2DFromNetData(const std::string& path, std::vector<std::vector<float> >& lines_pnts);

	std::string ReplacePatternString(std::string& str, const std::string& old_str, const std::string& new_str);

	/*
	Ŀ�ģ�ȥ��·��dir���ļ����ĺ�׺
	*/
	std::string ExtractFileNameNosufixNodir(std::string path);
};