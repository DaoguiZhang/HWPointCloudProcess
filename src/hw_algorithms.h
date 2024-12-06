/*
zdg
basical math algrithms 
*/
#ifndef __HW_ALGORITHMS_H__
#define __HW_ALGORITHMS_H__


#include<Eigen/Eigen>

namespace HW
{

	float ComputeAngleBetweenTwoLineSegs(const Eigen::Vector3f& l1s, const Eigen::Vector3f& l1e,
		const Eigen::Vector3f& l2s, const Eigen::Vector3f& l2e);

	float ComputeAngleFromTwoLinesVector3D(const Eigen::Vector3f& srcd, const Eigen::Vector3f& tgtd);

	float ComputeAngleFromTwoLinesVector2D(const Eigen::Vector2f& src_d, const Eigen::Vector2f& tgt_d);

	float Point3D2LineSegMinDistance(const Eigen::Vector3f& pnt, const Eigen::Vector3f& ls, const Eigen::Vector3f& le);

    Eigen::Matrix3f ComputeSkewMatrixFromLfVector(const Eigen::Vector3f& v);

    bool CheckPntOnLineSegOnCollinearPnts(const Eigen::Vector2f& p, const Eigen::Vector2f& s, const Eigen::Vector2f& e);

    //check line if intersect with rectangle ax+by+c=0 and mincor, maxcor
    bool CheckLineWithRect2D(const Eigen::Vector3f& l, const Eigen::Vector2f& mincor, const Eigen::Vector2f& maxcor);
    
    //l: line function; line segment: s, e
    bool ComputeLineLineSegmentIntersection(const Eigen::Vector3f& l, 
        const Eigen::Vector2f& s, const Eigen::Vector2f& e, Eigen::Vector2f& p);
    
    //line crop function: ax+by+c=0; mincor, maxcor
    bool ComputeRectCropLinePnts(const Eigen::Vector3f& l, const Eigen::Vector2f& mincor, 
        const Eigen::Vector2f& maxcor, Eigen::Vector2f& p0, Eigen::Vector2f& p1);
    //Eigen::Matrix3d ComputeCrossMatrixFromLVector(const Eigen::)
    //bool CheckPntOnLineSegment2D(Eigen::Vector2d x, Eigen::Vector2d);

    //function fn:ax+by+cz+d=0; o_origin: o_dir: pnt dir, pnt 
    bool LineIntersectPlanePnt(const Eigen::Vector3f& o_origin, const Eigen::Vector3f& o_dir, 
        const Eigen::Vector4f& fn, Eigen::Vector3f& p);

    //line function: lo, ln; plane function: po, pn; p intersection pnt
    bool LineIntersectPlanePntN(const Eigen::Vector3f& lo, const Eigen::Vector3f& ln, 
        const Eigen::Vector3f& po, const Eigen::Vector3f& pn, Eigen::Vector3f& p); 
    
    bool Ray2Plane3D(const Eigen::Vector3f& o, const Eigen::Vector3f& dir, 
        const Eigen::Vector4f fn, Eigen::Vector3f& pnt3d);

	Eigen::Vector3f Pnt3dProjToPlane(const Eigen::Vector3f& pnt, const Eigen::Vector4f& coeff);

	//pnt:p, plane: po, pn(center pnt and plane normal);
	float Pnt3d2Plane3DCNDist(const Eigen::Vector3f& p, 
		const Eigen::Vector3f& pc, const Eigen::Vector3f& pn);

    float PntDist2LineSegment2D(const Eigen::Vector2f& pnt2d, const Eigen::Vector2f& s, const Eigen::Vector2f& e);

    float PntDist2Line2DOld(const Eigen::Vector2f& pnt2d, const Eigen::Vector2f& s, const Eigen::Vector2f& e);

	float PntDist2Line2D(const Eigen::Vector2f& pnt2d, const Eigen::Vector2f& s, const Eigen::Vector2f& e);

	float PntDist2Line3D(const Eigen::Vector3f& pnt, const Eigen::Vector3f& s, const Eigen::Vector3f& e);

	void RodriguesRotationVector2RotationMatrix(const Eigen::Vector3f& rv, Eigen::Matrix3f& r);

	void RotationMatrix2RodriguesRotationVector(const Eigen::Matrix3f& R, Eigen::Vector3f& rv);

	Eigen::Matrix3f ComputeColVector2RowVector(const Eigen::Vector3f& c, const Eigen::Vector3f& r);

	float ComputeArctan2YX(float y, float x);

	bool CheckTwoPnt3dSame(const Eigen::Vector3f& pnt0, const Eigen::Vector3f& pnt1);

	bool FittingLine2DFromPnts2D(const std::vector<Eigen::Vector2f>& pnts, Eigen::Vector3f& fun_line);

	//compute ls le from pnts 2d
	bool FittingLineLsLePnts2dFromPnts2d2f(const std::vector<Eigen::Vector2f>& pnts, Eigen::Vector2f& ls, Eigen::Vector2f& le);

	//compute ls le from pnts 3d
	bool FittingLineLsLePnts3dFromPnts3d3f(const std::vector<Eigen::Vector3f>& pnts, Eigen::Vector3f& ls, Eigen::Vector3f& le);

	float ComputePolygon2DAreaFromPolygonPnts2D(const std::vector<Eigen::Vector2f>& poly_pnts);

	void ComputePnt3DProj2LineDir3DF(const Eigen::Vector3f& lpnt, const Eigen::Vector3f& ldir, const Eigen::Vector3f& pnt, Eigen::Vector3f& proj);

	void ComputePnt2Proj2Line2D(const Eigen::Vector2f& lpnt, const Eigen::Vector2f& ldir, const Eigen::Vector2f& pnt, Eigen::Vector2f& proj);

	float ComputePntToLineDist2D(const Eigen::Vector2f& lpnt, const Eigen::Vector2f& ldir, const Eigen::Vector2f& pnt);
	
	float ComputePntToLineFunction2D(const Eigen::Vector3f& line_fun, const Eigen::Vector2f& pnt);

	bool ComputeLineFunctionVerticeNormal(const Eigen::Vector3f& line_fun, Eigen::Vector2f& vertical_normal);

	bool ComputeLineFunLinedir2D(const Eigen::Vector3f& line_fun, Eigen::Vector2f& ldir);

	bool SampleSpntFromFunLineFun2D(const Eigen::Vector3f& line_fun, Eigen::Vector2f& lpnt);

	bool ComputePntProjToLineFunction2D(const Eigen::Vector3f& line_fun, const Eigen::Vector2f& pnt, Eigen::Vector2f& proj_pnt);
	
	bool  ConvertEndPntsToLineFun2D(const Eigen::Vector2f& ls2d, const Eigen::Vector2f& le2d, Eigen::Vector3f& line_fun);

	bool ConvertEndPntsToSPntLineDir2D(const Eigen::Vector2f& ls2d, const Eigen::Vector2f& le2d,
		Eigen::Vector2f& lpnt, Eigen::Vector2f& ldir);

	bool ConvertLineFunToSPntLineDir2D(const Eigen::Vector3f& line_fun, Eigen::Vector2f& lpnt, Eigen::Vector2f& ldir);

	bool ConvertSPntLineToLineFun2D(const Eigen::Vector2f& lpnt, const Eigen::Vector2f& ldir, Eigen::Vector3f& line_fun);

}



#endif
