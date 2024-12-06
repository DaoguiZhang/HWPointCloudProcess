#include "hw_algorithms.h"
#include "hw_cmns.h"


namespace HW
{
	#define M_Pi_ZDG    3.14159265358979323846

	bool CompareHWAlgorithmPairValueZDG(const std::pair<int, float>& a, const std::pair<int, float>&b)
	{
		return a.second < b.second;
	}

	float ComputeAngleBetweenTwoLineSegs(const Eigen::Vector3f& l1s, const Eigen::Vector3f& l1e,
		const Eigen::Vector3f& l2s, const Eigen::Vector3f& l2e)
	{
		Eigen::Vector3f l1_dir = (l1e - l1s).normalized();
		Eigen::Vector3f l2_dir = (l2e - l2s).normalized();
		float l1_l2_dot = l1_dir.dot(l2_dir) / l1_dir.norm() / l2_dir.norm();
		if (l1_l2_dot >= 1.0)
		{
			l1_l2_dot = 1.0;
		}
		else if (l1_l2_dot <= -1.0)
		{
			l1_l2_dot = -1.0;
		}
		float line_angle = std::acos(l1_l2_dot) * 180 / M_Pi_ZDG;
		return line_angle;
	}

	float ComputeAngleFromTwoLinesVector3D(const Eigen::Vector3f& srcd, const Eigen::Vector3f& tgtd)
	{
		float srcd_tgtd_dot = srcd.dot(tgtd) / srcd.norm() / tgtd.norm();
		if (srcd_tgtd_dot >= 1.0)
		{
			srcd_tgtd_dot = 1.0;
		}
		else if (srcd_tgtd_dot <= -1.0)
		{
			srcd_tgtd_dot = -1.0;
		}
		float line_angle = std::acos(srcd_tgtd_dot) * 180 / M_Pi_ZDG;
		return line_angle;
	}

	float ComputeAngleFromTwoLinesVector2D(const Eigen::Vector2f& src_d, const Eigen::Vector2f& tgt_d)
	{
		float src_tgt_dot = src_d.dot(tgt_d) / src_d.norm() / tgt_d.norm();
		if (src_tgt_dot >= 1.0)
		{
			src_tgt_dot = 1.0;
		}
		else if (src_tgt_dot <= -1.0)
		{
			src_tgt_dot = -1.0;
		}
		//printf("src_tgt_dot: %f\n", src_tgt_dot);
		float line_angle = std::acos(src_tgt_dot) * 180 / M_Pi_ZDG;
		return line_angle;
	}

	float Point3D2LineSegMinDistance(const Eigen::Vector3f& pnt, const Eigen::Vector3f& ls, const Eigen::Vector3f& le)
	{
		Eigen::Vector3f ld = le - ls;
		Eigen::Vector3f sp = pnt - ls;
		float t = sp.dot(ld) / ld.squaredNorm();
		float dist = std::numeric_limits<float>::max();
		if (t >= 0.0 && t <= 1.0)
		{
			dist = (t * ld - sp).norm();
		}
		else if (t < 0)
		{
			dist = sp.norm();
		}
		else
		{
			dist = (le - pnt).norm();
		}
		return dist;
	}

    Eigen::Matrix3f ComputeSkewMatrixFromLfVector(const Eigen::Vector3f& v)
    {
        Eigen::Matrix3f m;
	    m << 0, -v(2), v(1),
		    v(2), 0, -v(0),
		    -v(1), v(0), 0;
	    return m;
    }

    bool CheckPntOnLineSegOnCollinearPnts(const Eigen::Vector2f& p, const Eigen::Vector2f& s, const Eigen::Vector2f& e)
    {
        Eigen::Vector2f ps = s - p;
        Eigen::Vector2f pe = e - p;
        return (ps.dot(pe) < KLINE3D_EPS);
    }

    bool CheckLineWithRect2D(const Eigen::Vector3f& l, const Eigen::Vector2f& mincor, const Eigen::Vector2f& maxcor)
    {
        //do it 
        Eigen::Vector2f lb(mincor[0], mincor[1]);
        Eigen::Vector2f rb(maxcor[0], mincor[1]);
        Eigen::Vector2f rt(maxcor[0], maxcor[1]);
        Eigen::Vector2f lt(mincor[0], maxcor[1]);
        //
        if((l[0]*lb[0] + l[1]*lb[1]+l[2]) < 0 
            && (l[0]*rb[0] + l[1]*rb[1] + l[2]) < 0 
            && (l[0]*rt[0] + l[1]*rt[1] + l[2]) < 0 
            && (l[0]*lt[0] + l[1]*lt[1] + l[2]) < 0)
        {
            return false;
        }
        if((l[0]*lb[0] + l[1]*lb[1] + l[2]) > 0 
            && (l[0]*rb[0] + l[1]*rb[1] + l[2]) > 0 
            && (l[0]*rt[0] + l[1]*rt[1] + l[2]) > 0 
            && (l[0]*lt[0] + l[1]*lt[1] + l[2]) > 0)
        {
            return false;
        }
        return true;
    }

    bool ComputeLineLineSegmentIntersection(const Eigen::Vector3f& l, 
        const Eigen::Vector2f& s, const Eigen::Vector2f& e, Eigen::Vector2f& p)
    {
        if((e-s).norm() < KMIN_DOUBLE_THRESHOLD)
        {
            return false;
        }
        Eigen::Vector3f tl = ComputeFunctionFromPnts(s,e);
        bool linesect = ComputeLine2LineIntersection(l, tl, p);
        if(linesect)
        {
            //check it on line segment
            if(CheckPntOnLineSegOnCollinearPnts(p, s, e))
            {
                //std::cerr <<"p on line: " << p.transpose() << std::endl;
                return true;
            }
            else
            {
                //std::cerr <<"p out of line: " << p.transpose() << std::endl;
                return false;
            }
        }
        else
        {
            return false;
        }
    }

    bool ComputeRectCropLinePnts(const Eigen::Vector3f& l, const Eigen::Vector2f& mincor,
        const Eigen::Vector2f& maxcor, Eigen::Vector2f& p0, Eigen::Vector2f& p1)
    {
        //compute two pnts
        Eigen::Vector2f l0s = mincor; Eigen::Vector2f l0e(maxcor[0], mincor[1]);
        Eigen::Vector2f l1s(maxcor[0], mincor[1]); Eigen::Vector2f l1e = maxcor;
        Eigen::Vector2f l2s = maxcor; Eigen::Vector2f l2e(mincor[0], maxcor[1]);
        Eigen::Vector2f l3s(mincor[0], maxcor[1]); Eigen::Vector2f l3e = mincor;

        //same line, to do next
        if(!CheckLineWithRect2D(l, mincor, maxcor))
        {
            //std::cerr << "out of rect..." << std::endl;
            return false;
        }
        else
        {    
            std::vector<Eigen::Vector2f> lps;
            std::vector<bool> lfs;
            Eigen::Vector2f l0p;
            bool l0f = ComputeLineLineSegmentIntersection(l, l0s, l0e, l0p);
            Eigen::Vector2f l1p;
            bool l1f = ComputeLineLineSegmentIntersection(l, l1s, l1e, l1p);
            Eigen::Vector2f l2p;
            bool l2f = ComputeLineLineSegmentIntersection(l, l2s, l2e, l2p);
            Eigen::Vector2f l3p;
            bool l3f = ComputeLineLineSegmentIntersection(l, l3s, l3e, l3p);
            lps.emplace_back(l0p); lps.emplace_back(l1p);
            lps.emplace_back(l2p); lps.emplace_back(l3p);
            lfs.emplace_back(l0f); lfs.emplace_back(l1f);
            lfs.emplace_back(l2f); lfs.emplace_back(l3f);
            //std::cerr <<"l0p, l1p, l2p, l3p: " << l0p.transpose() <<", " << l1p.transpose() << ", " <<
            //    l2p.transpose() << ", " << l3p.transpose() << std::endl;
            bool first_f = false;
            bool second_f = false;
            for(std::size_t i = 0; i < lfs.size(); ++i)
            {
                if(lfs[i] && !first_f)
                {
                    p0 = lps[i];
                    first_f = true;
                    continue;
                }
                if(first_f && lfs[i])
                {
                    p1 = lps[i];
                    second_f = true;
                    break;
                }
            }
            if(first_f && second_f)
            {
                //std::cerr <<"get it..." << std::endl;
                return true;
            }
            return false;
        }
    }

    bool LineIntersectPlanePnt(const Eigen::Vector3f& o_origin, const Eigen::Vector3f& o_dir, 
        const Eigen::Vector4f& fn, Eigen::Vector3f& p)
    {
        Eigen::Vector3f ln(fn[0], fn[1], fn[2]);
        float d = fn[3];
        if(std::abs(o_dir.dot(ln)) < KLINE3D_EPS)
        {
            return false;
        }
        float t = -(o_origin.dot(ln) + d) / (o_dir.dot(ln));
        p = o_origin + o_dir * t;
        return true;
    }

    bool LineIntersectPlanePntN(const Eigen::Vector3f& lo, const Eigen::Vector3f& ln, 
        const Eigen::Vector3f& po, const Eigen::Vector3f& pn, Eigen::Vector3f& p)
    {
        if(std::abs(ln.dot(pn)) < KLINE3D_EPS)
        {
            return false;
        }
        double t = (po.dot(pn) -lo.dot(pn)) / (ln.dot(pn));
        p = lo + t * ln;
        return true;
    }

    bool Ray2Plane3D(const Eigen::Vector3f& o, const Eigen::Vector3f& dir, 
        const Eigen::Vector4f fn, Eigen::Vector3f& pnt3d)
    {
        Eigen::Vector3f ln(fn[0], fn[1], fn[2]);
        float d = fn[3];
        if(std::abs(dir.dot(ln)) < KLINE3D_EPS)
        {
            return false;
        }
		float t = -(o.dot(ln) + d) / (dir.dot(ln));
        if(t < KLINE3D_EPS) //has orientation
            return false;
        pnt3d = o + dir * t;
        return true;
    }

	Eigen::Vector3f Pnt3dProjToPlane(const Eigen::Vector3f& pnt, const Eigen::Vector4f& coeff)
	{
		float dist = (coeff[0] * pnt[0] + coeff[1] * pnt[1] + coeff[2] * pnt[2] + coeff[3]) /
			std::sqrtf(coeff[0]*coeff[0] + coeff[1]*coeff[1] + coeff[2]*coeff[2]);

		float k = -1 * dist / (std::sqrtf(coeff[0]*coeff[0] + coeff[1]*coeff[1] + coeff[2]*coeff[2]));
		Eigen::Vector3f result;
		result[0] = pnt[0] + k * coeff[0];
		result[1] = pnt[1] + k * coeff[1];
		result[2] = pnt[2] + k * coeff[2];
		return result;
	}

	float Pnt3d2Plane3DCNDist(const Eigen::Vector3f& p,
		const Eigen::Vector3f& pc, const Eigen::Vector3f& pn)
	{
		float sd = pn.norm();
		if (sd < KMIN_DOUBLE_THRESHOLD)
			return std::numeric_limits<float>::max();
		float sb = std::abs(pn.dot(p) - pn.dot(pc));
		float d = sb / sd;
		return d;
	}

    float PntDist2LineSegment2D(const Eigen::Vector2f& pnt2d, const Eigen::Vector2f& s, const Eigen::Vector2f& e)
    {
        if((e -s).norm() < KMIN_FLOAT_THRESHOLD)
        {
            return (pnt2d-s).norm();
        }
        float d = PntDist2Line2D(pnt2d, s, e);
		float s2e = (e - s).norm();
		float s2p = (pnt2d - s).norm();
		float e2p = (pnt2d - e).norm();
		float max2p = std::max(s2p, e2p);
        if(std::sqrt(max2p*max2p - d*d) > s2e)
        {
            return std::min(s2p, e2p);
        }
        else
        {
            return d;
        }
    }

	float PntDist2Line2DOld(const Eigen::Vector2f& pnt2d, const Eigen::Vector2f& s, const Eigen::Vector2f& e)
    {
        if((e -s).norm() < KLINE3D_EPS)
        {
            return (pnt2d-s).norm();
        }
        float v = std::abs((pnt2d[0]-s[0])*(e[1]-s[1]) + (pnt2d[1]-s[1])*(s[0]-e[0]));
        float d = v / (e-s).norm();
        return d;
    }

	float PntDist2Line2D(const Eigen::Vector2f& pnt2d, const Eigen::Vector2f& s, const Eigen::Vector2f& e)
	{
		if ((e - s).norm() < KLINE3D_EPS)
		{
			return (pnt2d - s).norm();
		}
		Eigen::Vector2f v = e - s;
		Eigen::Vector2f w = pnt2d - s;
		float c1 = w.dot(v);
		float c2 = v.dot(v);
		float b = c1 / c2;
		Eigen::Vector2f pb = s + b*v;
		float dist = (pnt2d - pb).norm();
		return dist;
	}

	float PntDist2Line3D(const Eigen::Vector3f& pnt, const Eigen::Vector3f& s, const Eigen::Vector3f& e)
	{
		Eigen::Vector3f v = e - s;
		Eigen::Vector3f w = pnt - s;
		float c1 = w.dot(v);
		float c2 = v.dot(v);
		float b = c1 / c2;
		Eigen::Vector3f pb = s + b*v;
		return (pb - pnt).norm();
	}

	void RodriguesRotationVector2RotationMatrix(const Eigen::Vector3f& rv, Eigen::Matrix3f& r)
	{
		//rodrigues vector to rotation matrix
		float theta = rv.norm();
		if (theta < KMIN_FLOAT_THRESHOLD)
		{
			r = Eigen::Matrix3f::Identity();
		}
		float ct = std::cosf(theta);
		float st = std::cosf(theta);
		Eigen::Vector3f u = rv / theta;
		Eigen::Matrix3f I = Eigen::Matrix3f::Identity();
		Eigen::Matrix3f uut = ComputeColVector2RowVector(u, u);
		Eigen::Matrix3f sku = ComputeSkewMatrixFromLfVector(u);
		r = I*ct + (1 - ct)*uut + sku*st;
	}

	void RotationMatrix2RodriguesRotationVector(const Eigen::Matrix3f& R, Eigen::Vector3f& rv)
	{
		//rotation matrix to rodrigues vector
		Eigen::Matrix3f A = (R - R.transpose()) / 2;
		Eigen::Vector3f R_p = Eigen::Vector3f(A(2, 1), A(0, 2), A(1, 0));
		float s = R_p.norm();
		float c = (R(0, 0) + R(1, 1) + R(2, 2) - 1) / 2;
		if (s < KMIN_FLOAT_THRESHOLD && std::abs(c - 1.0) < KMIN_FLOAT_THRESHOLD)
		{
			rv = Eigen::Vector3f::Zero();
			return;
		}
		if (s < KMIN_FLOAT_THRESHOLD && std::abs(c + 1.0) < KMIN_FLOAT_THRESHOLD)
		{
			//to do next...
			//Eigen::Vector3f V = r + Eigen::Matrix3f::Identity();
			//Eigen::Vector3f U = 
		}
		else
		{
			Eigen::Vector3f U = R_p / s;
			float theta = ComputeArctan2YX(s, c);
			rv = U*theta;
		}
	}

	Eigen::Matrix3f ComputeColVector2RowVector(const Eigen::Vector3f& c, const Eigen::Vector3f& r)
	{
		Eigen::Matrix3f m;
		m.block<1, 3>(0, 0) = c[0] * r.transpose();
		m.block<1, 3>(1, 0) = c[1] * r.transpose();
		m.block<1, 3>(2, 0) = c[2] * r.transpose();
		return m;
	}

	float ComputeArctan2YX(float y, float x)
	{
		if (x > 0.0)
		{
			return std::atan(y / x);
		}
		else if (x < 0.0)
		{
			return M_Pi_ZDG + std::atan(y / x);
		}
		else
		{
			if (y > 0)
			{
				return M_Pi_ZDG / 2;
			}
			else
			{
				return -M_Pi_ZDG / 2;
			}
		}
	}

	bool CheckTwoPnt3dSame(const Eigen::Vector3f& pnt0, const Eigen::Vector3f& pnt1)
	{
		if(std::abs(pnt0[0] - pnt1[0]) < KMIN_FLOAT_THRESHOLD
			&& std::abs(pnt0[1] - pnt1[1]) < KMIN_FLOAT_THRESHOLD
			&& std::abs(pnt0[2] - pnt1[2]) < KMIN_FLOAT_THRESHOLD)
		{
			return true;
		}
		return false;
	}

	bool FittingLine2DFromPnts2D(const std::vector<Eigen::Vector2f>& pnts, Eigen::Vector3f& fun_line)
	{
		int pnts_num = static_cast<int>(pnts.size());
		if (pnts_num < 2)
		{
			printf("no enough pnts number...\n");
			return false;
		}
		float sum_x = 0.0, sum_y = 0.0, sum_xy = 0.0, sum_x2 = 0.0, sum_y2 = 0.0;
		//fitting these points into line
		for (int i = 0; i < pnts.size(); ++i)
		{
			sum_x += pnts[i][0];
			sum_y += pnts[i][1];
			sum_xy += pnts[i][0] * pnts[i][1];
			sum_x2 += pnts[i][0] * pnts[i][0];
			sum_y2 += pnts[i][1] * pnts[i][1];
		}
		float a_x = sum_x / pnts_num;
		float a_y = sum_y / pnts_num;
		float a_xy = sum_xy / pnts_num;
		float a_x2 = sum_x2 / pnts_num;
		float a_y2 = sum_y2 / pnts_num;
		float A = -(a_xy - a_x * a_y);
		float B;
		float Bx = a_x2 - a_x * a_x;
		float By = a_y2 - a_y * a_y;
		if (std::abs(Bx) < std::abs(By))
		{
			//Line is more Vertical
			B = By;
			std::swap(A, B);
		}
		else
		{
			//Line is more Horizontal
			B = Bx;
		}
		float C = -(A*a_x + B*a_y);
		fun_line[0] = A;
		fun_line[1] = B;
		fun_line[2] = C;
		return true;
	}

	bool FittingLineLsLePnts2dFromPnts2d2f(const std::vector<Eigen::Vector2f>& pnts, Eigen::Vector2f& ls, Eigen::Vector2f& le)
	{
		Eigen::Vector3f ls_3d, le_3d;
		std::vector<Eigen::Vector3f> pnts3d;
		for (int i = 0; i < pnts.size(); ++i)
		{
			pnts3d.emplace_back(Eigen::Vector3f(pnts[i][0], pnts[i][1], 1.0));
		}
		FittingLineLsLePnts3dFromPnts3d3f(pnts3d, ls_3d, le_3d);
		ls = Eigen::Vector2f(ls_3d[0], ls_3d[1]);
		le = Eigen::Vector2f(le_3d[0], le_3d[1]);
		return true;
	}

	bool FittingLineLsLePnts3dFromPnts3d3f(const std::vector<Eigen::Vector3f>& pnts, Eigen::Vector3f& ls, Eigen::Vector3f& le)
	{
		Eigen::Vector3f centroid(0.0, 0.0, 0.0);
		int pnt_num = static_cast<int> (pnts.size());
		if (pnt_num < 2)
		{
			return false;
		}
		for (int i = 0; i < pnts.size(); ++i)
		{
			centroid += pnts[i];
		}
		centroid /= pnts.size();
		//compute the covariance matrix of the points
		Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
		for (int i = 0; i < pnts.size(); ++i)
		{
			cov += (pnts[i] - centroid)*(pnts[i] - centroid).transpose();
		}
		//compute the eigenvectors and eigenvalues of the convariance matrix
		Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigensolver(cov);
		Eigen::Vector3f ld = eigensolver.eigenvectors().col(2);

		////test
		//Eigen::Vector3f lv = eigensolver.eigenvalues();
		//Eigen::Vector3f lvec0 = eigensolver.eigenvectors().col(0);
		//Eigen::Vector3f lvec1 = eigensolver.eigenvectors().col(1);
		//Eigen::Vector3f lvec2 = eigensolver.eigenvectors().col(2);
		//std::cerr << "lv: " << lv.transpose() << std::endl;
		//std::cerr << "lvec 0: " << lvec0.transpose() << std::endl;
		//std::cerr << "lvec 1: " << lvec1.transpose() << std::endl;
		//std::cerr << "lvec 2: " << lvec2.transpose() << std::endl;
		//std::cerr << "ld: " << ld.transpose() << std::endl;
		////end test

		std::vector<std::pair<int, float> > t_vec;
		for (int i = 0; i < pnts.size(); ++i)
		{
			float t = (pnts[i].dot(ld) - centroid.dot(ld)) / ld.squaredNorm();
			std::pair<int, float> tv = std::make_pair(i, t);
			t_vec.emplace_back(tv);
		}
		std::sort(t_vec.begin(), t_vec.end(), CompareHWAlgorithmPairValueZDG);
		Eigen::Vector3f sp, ep;
		ComputePnt3DProj2LineDir3DF(centroid, ld, pnts[t_vec.front().first], sp);
		ComputePnt3DProj2LineDir3DF(centroid, ld, pnts[t_vec.back().first], ep);

		////compute the start pnt of the line
		//ls = centroid - ld*(centroid.dot(ld) / ld.dot(ld));
		////compute the end pnt of the line
		//le = centroid + ld*(centroid.dot(ld) / ld.dot(ld));
		ls = sp;
		le = ep;
		////compute the line function d
		//ls = line_s;
		//ldir = ld.normalized();
		//std::cerr << "Equation of the line: (" << ls.transpose() << ") + t*(" << ld.transpose() << ")" << std::endl;
		return true;
	}

	float ComputePolygon2DAreaFromPolygonPnts2D(const std::vector<Eigen::Vector2f>& poly_pnts)
	{
		int pnts_num = static_cast<int>(poly_pnts.size());
		//计算polygon 2d的面积大小
		float area_tmp = 0.0;
		for (int i = 0; i < poly_pnts.size(); ++i)
		{
			area_tmp += (poly_pnts[i][0] * poly_pnts[(i + 1) % pnts_num][1]
				- poly_pnts[(i + 1) % pnts_num][0] * poly_pnts[i][1]);
		}
		float polygon_area_2d = 0.5 * std::abs(area_tmp);
		return polygon_area_2d;
	}

	void ComputePnt3DProj2LineDir3DF(const Eigen::Vector3f& lpnt, const Eigen::Vector3f& ldir, const Eigen::Vector3f& pnt, Eigen::Vector3f& proj)
	{
		double t = (pnt.dot(ldir) - lpnt.dot(ldir)) / ldir.squaredNorm();
		//std::cerr <<"t: " << t <<std::endl;
		proj = t * ldir + lpnt;
	}

	void ComputePnt2Proj2Line2D(const Eigen::Vector2f& lpnt, const Eigen::Vector2f& ldir, const Eigen::Vector2f& pnt, Eigen::Vector2f& proj)
	{
		//计算pi平面
		//std::cerr << "lpnt: "<< lpnt.transpose() <<std::endl;
		//std::cerr << "ldir: "<< ldir.transpose() <<std::endl;
		//std::cerr << "pnt: "<< pnt.transpose() <<std::endl;
		float t = (pnt.dot(ldir) - lpnt.dot(ldir)) / ldir.squaredNorm();
		//std::cerr <<"t: " << t <<std::endl;
		proj = t * ldir + lpnt;
	}

	float ComputePntToLineDist2D(const Eigen::Vector2f& lpnt, const Eigen::Vector2f& ldir, const Eigen::Vector2f& pnt)
	{
		//start to compute dist between pnt and line(lpnt, ldir)
		Eigen::Vector2f proj;
		ComputePnt2Proj2Line2D(lpnt, ldir, pnt, proj);
		//std::cerr << "proj: "<< proj.transpose() <<std::endl;
		float d = (proj - pnt).norm();
		return d;
	}

	float ComputePntToLineFunction2D(const Eigen::Vector3f& line_fun, const Eigen::Vector2f& pnt)
	{
		Eigen::Vector2f line_coeff = Eigen::Vector2f(line_fun[0], line_fun[1]);
		float line_denom = line_coeff.norm();
		if (line_denom < KMIN_FLOAT_THRESHOLD_REFINED)
		{
			return KMAX_FLOAT_LIMIT_VALUE;
		}
		float line_nom = line_fun[0] * pnt[0] + line_fun[1] * pnt[1] + line_fun[2];
		float vv = std::abs(line_nom) / line_denom;
		return vv;
	}

	bool ComputeLineFunctionVerticeNormal(const Eigen::Vector3f& line_fun, Eigen::Vector2f& vertical_normal)
	{
		Eigen::Vector2f line_coeff = Eigen::Vector2f(line_fun[0], line_fun[1]);
		float line_denom = line_coeff.norm();
		if (line_denom < KMIN_FLOAT_THRESHOLD_REFINED)
		{
			return false;
		}
		line_coeff.normalize();
		vertical_normal = line_coeff;
		return true;
	}

	bool ComputeLineFunLinedir2D(const Eigen::Vector3f& line_fun, Eigen::Vector2f& ldir)
	{
		Eigen::Vector2f line_normal = Eigen::Vector2f(line_fun[0], line_fun[1]);
		float line_denom = line_normal.norm();
		if (line_denom < KMIN_FLOAT_THRESHOLD_REFINED)
		{
			return false;
		}
		line_normal.normalize();
		ldir[0] = -line_normal[1];
		ldir[1] = line_normal[0];
		return true;
	}

	bool SampleSpntFromFunLineFun2D(const Eigen::Vector3f& line_fun, Eigen::Vector2f& lpnt)
	{
		if (std::abs(line_fun[0]) < KMIN_FLOAT_THRESHOLD_REFINED
			&& std::abs(line_fun[1]) < KMIN_FLOAT_THRESHOLD_REFINED)
		{
			return false;
		}
		if (std::abs(line_fun[0]) > std::abs(line_fun[1]))
		{
			lpnt[0] = -line_fun[2] / line_fun[0];
			lpnt[1] = 0;
		}
		else
		{
			lpnt[0] = 0;
			lpnt[1] = -line_fun[2] / line_fun[1];
		}
		return true;
	}

	bool ComputePntProjToLineFunction2D(const Eigen::Vector3f& line_fun, const Eigen::Vector2f& pnt, Eigen::Vector2f& proj_pnt)
	{
		float dist2line = ComputePntToLineFunction2D(line_fun, pnt);
		if (dist2line == KMAX_FLOAT_LIMIT_VALUE)
		{
			return false;
		}
		Eigen::Vector2f l_vertical_dir;
		bool flag = ComputeLineFunctionVerticeNormal(line_fun, l_vertical_dir);
		if (flag)
		{
			proj_pnt = pnt + dist2line * l_vertical_dir;
			return true;
		}
		else
		{
			return false;
		}
	}

	bool  ConvertEndPntsToLineFun2D(const Eigen::Vector2f& ls2d, const Eigen::Vector2f& le2d, Eigen::Vector3f& line_fun)
	{
		if (std::abs((le2d - ls2d).norm()) < KMIN_FLOAT_THRESHOLD_REFINED)
			return false;
		line_fun[0] = le2d[1] - ls2d[1];
		line_fun[1] = ls2d[0] - le2d[0];
		line_fun[2] = le2d[0] * ls2d[1] - ls2d[0] * le2d[1];
		return true;
	}
	
	bool ConvertEndPntsToSPntLineDir2D(const Eigen::Vector2f& ls2d, const Eigen::Vector2f& le2d,
		Eigen::Vector2f& lpnt, Eigen::Vector2f& ldir)
	{
		if ((le2d - ls2d).norm() < KMIN_FLOAT_THRESHOLD_REFINED)
		{
			ldir = Eigen::Vector2f(0.0, 0.0);
			lpnt = Eigen::Vector2f(0.0, 0.0);
			return false;
		}
		ldir = le2d - ls2d;
		ldir.normalize();
		lpnt = ls2d;
		return true;
	}

	bool ConvertLineFunToSPntLineDir2D(const Eigen::Vector3f& line_fun, Eigen::Vector2f& lpnt, Eigen::Vector2f& ldir)
	{
		//SamplePntsFromLineEndPnts2D
	 	bool lp_flag = SampleSpntFromFunLineFun2D(line_fun, lpnt);
		bool ld_flag = ComputeLineFunLinedir2D(line_fun, ldir);
		if (lp_flag&&ld_flag)
		{
			return true;
		}
		return false;
	}

	bool ConvertSPntLineToLineFun2D(const Eigen::Vector2f& lpnt, const Eigen::Vector2f& ldir, Eigen::Vector3f& line_fun)
	{
		if (ldir[0] < KMIN_FLOAT_THRESHOLD_REFINED
			&& ldir[1] < KMIN_FLOAT_THRESHOLD_REFINED)
		{
			return false;
		}
		Eigen::Vector2f end_pnt = lpnt + ldir;
		bool line_flag = ConvertEndPntsToLineFun2D(lpnt, end_pnt, line_fun);
		return line_flag;
	}
}
