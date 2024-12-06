//file contain common operation such as find string, split string and so on

#ifndef __HW_CMNS_H__
#define __HW_CMNS_H__

//#include<colmap/util/misc.h>
#include<boost/algorithm/string.hpp>
#include<vector>
#include<Eigen/Eigen>
#include<climits>

//#include "hw_serialization.h"

namespace HW
{
    const double KMIN_DOUBLE_THRESHOLD = 1e-8;
    const double KANGLE_THRESHOLD = 45.0;
    const unsigned int KMAXMUMLIMIT = std::numeric_limits<unsigned int>::max();
    const double KLINE3D_EPS = 1e-12;
    const double KL3D_EPIPOLAR_OVERLAP = 0.25;
    const double KMIN_DOUBLE_LINE_THRESHOLD = 1e-9;
	const double KMIN_DOUBLE_LIMIT_THRESHOLD = 1e-10;
    const double KMAX_DOUBLE_LIMIT_VALUE = std::numeric_limits<double>::max(); 
    const double KMIN_DOUBLE_LIMIT_VALUE = std::numeric_limits<double>::lowest();
	const float KMAX_FLOAT_LIMIT_VALUE = std::numeric_limits<float>::max();
	const float KMIN_FLOAT_LIMIT_VALUE = std::numeric_limits<float>::lowest();
	const float KMIN_FLOAT_THRESHOLD = 1e-6;
	const float KMIN_FLOAT_THRESHOLD_REFINED = 1e-8;
	const float KMIN_FLOAT_THRESHOLD_COARSE = 1e-4;
	const double CMATH_PI_V = 3.14159265358979323846;
	const float ANGLE_VERTICAL = 90.0;
	const float ANGLE_HALF_CIRCLE = 180.0;
	const int KMAX_USHORT_VALUE = 65535;

	//FLT_MAX
    enum HWPLYFormat
    {
        k_binary,
        k_assic
    };
    
    typedef struct HWColor
    {
        float r;
        float g;
        float b;
        float d;
    } HWColor;
    
    typedef struct HWVextex
    {
        float x;
        float y;
        float z;
    } HWVextex;
    
    typedef struct HWNormal
    {
        float nx;
        float ny;
        float nz;
    } HWNormal;
    
    typedef struct HWBox
    {
        Eigen::Vector3f min_;
        Eigen::Vector3f max_;
    } HWBox;
    
    struct HWLinePoint2D
    {
        Eigen::Vector3f pnt_;
        std::vector<int> pnt2ls_;   //point 2 line segment idx
        HWLinePoint2D(){}
        HWLinePoint2D(const HWLinePoint2D& other)
        {
            this->pnt_ = other.pnt_;
            this->pnt2ls_ = other.pnt2ls_;
        }
        HWLinePoint2D(Eigen::Vector3f& pnt, std::vector<int>& pnt2ls)
        {
            this->pnt_ = pnt;
            this->pnt2ls_ = pnt2ls;
        }
        HWLinePoint2D operator=(const HWLinePoint2D& other)
        {
            this->pnt_ = other.pnt_;
            this->pnt2ls_ = other.pnt2ls_;
            return *this;
        }
    };

    typedef HWLinePoint2D HWLinePoint3D;

    struct HWPolygonLineIdx
    {
        std::vector<int> polygon2linesidx_; //polygon 2 lines segment idx
    };

    enum HWSceneLayoutDimType
    {
        kScene2DType,
        kScene3DType,
        kSceneOtherType
    };

	struct HWMatch
	{
		// correspondence
		unsigned int src_camID_;
		unsigned int src_segID_;
		unsigned int tgt_camID_;
		unsigned int tgt_segID_;

		bool valid_match_;

		//check two lines if interior or exterior
		int is_polygon_interior_ = -1;
		Eigen::Vector2i adj_poly_idxs_;	//3d line index to two polygon idx

										// scores
		double overlap_score_;
		double score3D_;

		// depths (to do next...)
		double depth_p1_;
		double depth_p2_;
		double depth_q1_;
		double depth_q2_;

		//world coordiante pnts from two views
		Eigen::Vector3f world_p1_;
		Eigen::Vector3f world_p2_;
		Eigen::Vector3f world_q1_;
		Eigen::Vector3f world_q2_;
	};

	struct HWLineCMN2D
	{
		Eigen::Vector2f p_;	//pnt
		Eigen::Vector2f d_;
	};

    // sorting functions
    bool SortMatchesByIDs(const HWMatch m1, const HWMatch m2);

    //get idx
    //std::size_t GetIdxFromPairVecFirst(int idx, const std::vector<std::pair<int, >>& vec);

    bool IsNotSpacialSpace(const int c);

    std::string ReplaceStrWithNewStr(const std::string& str, const std::string& old_str,
        const std::string& new_str);

    std::string EnsureTrailingSlashHW(const std::string& str);

    std::string GetLeftSlashPathName(const std::string& path);

	std::string GetDirFromPathName(const std::string& path);

    std::string GetBaseName(const std::string& path);

    std::string GetPathPrefix(const std::string& path);

    std::string GetBaseNameWithoutSuffix(const std::string& path);

    std::vector<std::string> GetDirListFromDir(const std::string& path);

    std::vector<std::string> GetDirListFromDirRecursive(const std::string& path);

     /*
    path: get path files lists from dir(without recursive)
    */
    std::vector<std::string> GetFilesListFromDir(const std::string& path);
    /*
    path: get path files lists from dir(recursive)
    */
    std::vector<std::string> GetFilesListFromDirRecursive(const std::string& path);

    std::vector<std::string> SplitStrWithDelim(const std::string& str, const std::string& delim);

    /*
    find idx: -1 not found, or find the idx
    */
    int FindStrIdxFromVecStrs(const std::string& srcstr, const std::vector<std::string>& vecstrs);
	int FindStrIdxFromVecStrsNew(const std::string& srcstr, const std::vector<std::string>& vecstrs);

    void StringLeftTrim(std::string* str);
    void StringRightTrim(std::string* str);
    void StringTrimLR(std::string* str);

	/*
	vector contain same element
	*/
	bool ElementVectorContainsDuplicatedElement(std::vector<int>& nums_vec);

    /*
    change all string to lower letter
    */
   void StringToLowerLetters(std::string* str);
   void StringToUpperLetters(std::string* str);
   
    //sample pnts from line pnts
    void SamplePntsFromLineEndPnts2D(Eigen::Vector2f& s, Eigen::Vector2f& e, std::vector<Eigen::Vector2i>& pnts);

    bool CheckPoint2dInRectd(Eigen::Vector2f& pnt, Eigen::Vector2f& mincor, Eigen::Vector2f& maxcor);

    bool CheckPoint2dInRecti(Eigen::Vector2i& pnt, Eigen::Vector2i& mincor, Eigen::Vector2i& maxcor);

    //Ax+By+C=0 (A,B,C)
    Eigen::Vector3f ComputeFunctionFromPnts(const Eigen::Vector2f& s, const Eigen::Vector2f& e);

    //f1:Ax+By+C=0; f2: Ax+By+C=0
    bool ComputeLine2LineIntersection(const Eigen::Vector3f& f1, const Eigen::Vector3f& f2, Eigen::Vector2f& pnt);

    float LiangBarskVecsMax(const std::vector<float>& v);
    float LiangBarskVecsMin(const std::vector<float>& v);
    
    int LiangBarskyClipperAlgo(float xmin, float ymin, float xmax, float ymax,
		float x1, float y1, float x2, float y2, float& x1_new, float& y1_new, float& x2_new, float& y2_new);

	void rodrigues_to_matrix3d(double const* r, double* rot);

	void ConvertMatrix3d_to_Matrix3d_Transposed(double const* r, double* t_r);

	Eigen::Matrix3d ComputeSkewMatrixFromVector3d(const Eigen::Vector3d& v);

	Eigen::Vector3d RotationMatrixToAxisAngleD(const Eigen::Matrix3d& R);

	Eigen::Matrix3d AxisAngleToRotationMatrixD(const Eigen::Vector3d& v);

	bool IsStrFrontElementInteger(const std::string& e);
}

#endif
