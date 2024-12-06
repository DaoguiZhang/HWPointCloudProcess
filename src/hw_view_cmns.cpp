#include"hw_view_cmns.h"
#include<iostream>

namespace HW
{
	bool ComparePairValueZDG(const std::pair<int, float>& a, const std::pair<int, float>&b)
	{
		return a.second < b.second;
	}

	bool IsHWPoint3dValidZDG(const Eigen::Vector3f& v_p3d)
	{
		if (v_p3d[0] == v_p3d[0]
			&& v_p3d[1] == v_p3d[1]
			&& v_p3d[2] == v_p3d[2])
		{
			return true;
		}
		return false;
	}

	bool IsHWLineSegmentValidZDG(const std::pair<Eigen::Vector3f, Eigen::Vector3f>& lineseg)
	{
		return (IsHWPoint3dValidZDG(lineseg.first) && IsHWPoint3dValidZDG(lineseg.second));
	}

	bool FittingLine3dFromPnts3d3f(const std::vector<Eigen::Vector3f>& pnts, Eigen::Vector3f& ls, Eigen::Vector3f& le)
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

		std::vector<std::pair<int, float>> t_vec;
		for (int i = 0; i < pnts.size(); ++i)
		{ 
			float t = (pnts[i].dot(ld) - centroid.dot(ld)) / ld.squaredNorm();
			std::pair<int, float> tv = std::make_pair(i, t);
			t_vec.emplace_back(tv);
		}
		std::sort(t_vec.begin(), t_vec.end(), ComparePairValueZDG);
		Eigen::Vector3f sp,ep;
		ComputePntProj2Line3DF(centroid, ld, pnts[t_vec.front().first], sp);
		ComputePntProj2Line3DF(centroid, ld, pnts[t_vec.back().first], ep);

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

	bool FittingLine3dFromPnts3d3d(const std::vector<Eigen::Vector3d>& pnts, Eigen::Vector3d& pnt, Eigen::Vector3d& ld)
	{
		Eigen::Vector3d centroid(0.0, 0.0, 0.0);
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
		Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
		for (int i = 0; i < pnts.size(); ++i)
		{
			cov += (pnts[i] - centroid)*(pnts[i] - centroid).transpose();
		}
		//compute the eigenvectors and eigenvalues of the convariance matrix
		Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(cov);
		Eigen::Vector3d fld = eigensolver.eigenvectors().col(2);
		ld = fld.normalized();
		pnt = centroid;
		return true;
	}

	Eigen::Vector3d GetWPntFromImgPntDByRKMatrix(const Eigen::Matrix3d& K, const Eigen::Matrix4d& RT, const Eigen::Vector3d& img_pnt)
	{
		double const fx = K(0, 0);		//fx
		double const fy = K(1, 1);		//fy
		double const cx = K(0, 2);		//cx
		double const cy = K(1, 2);		//cy
		Eigen::Vector3d pc;
		pc[0] = img_pnt[2] * (img_pnt[0] - cx) / fx;	//in camera coordinate
		pc[1] = img_pnt[2] * (img_pnt[1] - cy) / fy;	//in camera coordinate
		pc[2] = img_pnt[2];	//in camera coordinate
		//compute the pnt in world coordinate
		Eigen::Matrix3d RTR = RT.topLeftCorner(3, 3);
		Eigen::Vector3d RTT = RT.topRightCorner(3, 1);
		Eigen::Matrix3d RTRT = RTR.transpose();

		Eigen::Vector3d pw = RTRT*(pc - RTT);
		return pw;
#if 0
		//double const fx = K_(0, 0);		//fx
		//double const fy = K_(1, 1);		//fy
		//double const cx = K_(0, 2);		//cx
		//double const cy = K_(1, 2);		//cy

		//Eigen::Vector3d pc;
		//pc[0] = d * (x_ - cx) / fx;	//in camera coordinate
		//pc[1] = d * (y_ - cy) / fy;	//in camera coordinate
		//pc[2] = z_;	//in camera coordinate

		//			//compute the pnt in world coordinate
		//Eigen::Matrix3d RTR = RT_.topLeftCorner(3, 3);
		//Eigen::Vector3d RTT = RT_.topRightCorner(3, 1);
		//Eigen::Matrix3d RTRT = RTR.transpose();

		//Eigen::Vector3d pw = RTRT*(pc - RTT);
#endif
	}

	float ComputePnt3dToLineSegment3DDistF(const Eigen::Vector3f& ls, const Eigen::Vector3f& le, const Eigen::Vector3f& pnt)
	{
		Eigen::Vector3f proj_pnt;
		Eigen::Vector3f ldir = (le - ls).normalized();
		ComputePntProj2Line3DF(ls, ldir, pnt, proj_pnt);
		float pnt2proj_dist = (pnt - proj_pnt).norm();
		float pnt2ls_dist = (pnt - ls).norm();
		float pnt2le_dist = (pnt - le).norm();
		float vmin_v = std::min(pnt2proj_dist, pnt2ls_dist);
		vmin_v = std::min(vmin_v, pnt2le_dist);
		return vmin_v;
	}

	void ComputePntProj2Line3DF(const Eigen::Vector3f& lpnt, const Eigen::Vector3f& ldir, const Eigen::Vector3f& pnt, Eigen::Vector3f& proj)
	{
		double t = (pnt.dot(ldir) - lpnt.dot(ldir)) / ldir.squaredNorm();
		//std::cerr <<"t: " << t <<std::endl;
		proj = t * ldir + lpnt;
	}

	double ComputePnt3DToLine3DDistF(const Eigen::Vector3f& lpnt, const Eigen::Vector3f& ldir, const Eigen::Vector3f& pnt)
	{
		Eigen::Vector3f proj;
		ComputePntProj2Line3DF(lpnt, ldir, pnt, proj);
		//std::cerr << "proj: "<< proj.transpose() <<std::endl;
		double d = (proj - pnt).norm();
		return d;
	}

	Eigen::Matrix3d GetPluckerKMatrix(const Eigen::Matrix3d& km)
	{
		Eigen::Matrix3d pluckerk;
		double fx = km(0, 0);
		double fy = km(1, 1);
		double cx = km(0, 2);
		double cy = km(1, 2);
		pluckerk << fy, 0, 0,
			0, fx, 0,
			-fy*cx, -fx*cy, fx*fy;
		return pluckerk;
	}

	Eigen::Matrix<double, 6, 6> GetPluckerTransformMatrix(const Eigen::Matrix4d& T)
	{
		Eigen::Matrix<double, 6, 6> temp;
		temp.setZero();
		temp.block<3, 3>(0, 0) = T.block<3, 3>(0, 0);
		temp.block<3, 3>(0, 3) = ComputeSkewMatrixFromVector3d(T.block<3, 1>(0, 3))*T.block<3, 3>(0, 0);
		temp.block<3, 3>(3, 3) = T.block<3, 3>(0, 0);
		return temp;
	}

	Eigen::Matrix<double, 6, 1> OrthLineToPluckerLine(const Eigen::Vector4d& orthline)
	{
		/*double s1 = std::sin(orthline[0]);
		double c1 = std::cos(orthline[0]);
		double s2 = std::sin(orthline[1]);
		double c2 = std::cos(orthline[1]);
		double s3 = std::sin(orthline[2]);
		double c3 = std::cos(orthline[2]);
		Eigen::Matrix3d R;
		R <<
			c2*c3, s1*s2*c3 - c1*s3, c1*s2*c3 + s1*s3,
			c2*s3, s1*s2*s3 + c1*c3, c1*s2*s3 - s1*c3,
			-s2, s1*c2, c1*c2;
		double w1 = std::cos(orthline[3]);
		double w2 = std::sin(orthline[3]);
		Eigen::Matrix<double, 6, 1> plucker_line;
		plucker_line.head(3) = w1*R.col(0);
		plucker_line.tail(3) = w2*R.col(1);
		return plucker_line;*/

		double s1 = sin(orthline[0]);
		double c1 = cos(orthline[0]);
		double s2 = sin(orthline[1]);
		double c2 = cos(orthline[1]);
		double s3 = sin(orthline[2]);
		double c3 = cos(orthline[2]);
		Eigen::Matrix3d R;
		R <<
			c2 * c3, s1 * s2 * c3 - c1 * s3, c1 * s2 * c3 + s1 * s3,
			c2 * s3, s1 * s2 * s3 + c1 * c3, c1 * s2 * s3 - s1 * c3,
			-s2, s1 * c2, c1 * c2;
		double w1 = cos(orthline[3]);
		double w2 = sin(orthline[3]);
		Eigen::Matrix<double, 6, 1> plucker_line;
		plucker_line.head(3) = w1 * R.col(0);
		plucker_line.tail(3) = w2 * R.col(1);
		return plucker_line;

	}

	Eigen::Matrix<double, 6, 1> OrthLineToPluckerLineNew(const Eigen::Vector4d& orthline)
	{
		Eigen::Vector3d rw = orthline.head(3);
		Eigen::Matrix3d R = AxisAngleToRotationMatrixD(rw);
		double w1 = std::cos(orthline[3]);
		double w2 = std::sin(orthline[3]);
		Eigen::Matrix<double, 6, 1> plucker_line;
		plucker_line.head(3) = w1*R.col(0);
		plucker_line.tail(3) = w2*R.col(1);
		return plucker_line;
	}

	Eigen::Vector4d PluckerLineToOrthLine(Eigen::Matrix<double, 6, 1> pluckerline)
	{
		Eigen::Matrix3d so3_r = GetOrthRMatrixFromPlucker(pluckerline);
		Eigen::Vector3d sor_w = RotationMatrixToAxisAngleD(so3_r);
		Eigen::Vector3d ln = pluckerline.head(3);
		Eigen::Vector3d ld = pluckerline.tail(3);
		Eigen::Vector2d lsigma = Eigen::Vector2d(ln.norm(), ld.norm());
		Eigen::Vector2d lw = lsigma / lsigma.norm();
		double lphi = std::acos(lw[0]);
		Eigen::Vector4d ortho_line = Eigen::Vector4d(sor_w[0], sor_w[1], sor_w[2], lphi);
		return ortho_line;
	}

	Eigen::Matrix3d PluckerLineToOrthLineR(Eigen::Matrix<double, 6, 1> pluckerline)
	{
		Eigen::Matrix3d R;
		Eigen::Vector3d n = pluckerline.head(3);
		Eigen::Vector3d d = pluckerline.tail(3);
		Eigen::Vector3d n0 = n;
		Eigen::Vector3d d0 = d;
		n.normalize();
		d.normalize();
		R.col(0) = n;
		R.col(1) = d;
		R.col(2) = n0.cross(d0) / (n0.cross(d0).norm());
		return R;
	}

	Eigen::Matrix2d PluckerLineToOrthLineW(Eigen::Matrix<double, 6, 1> pluckerline)
	{
		Eigen::Matrix2d temp;
		double nnorm = pluckerline.head(3).norm();
		double dnorm = pluckerline.tail(3).norm();
		double fenmu = sqrt(nnorm*nnorm + dnorm*dnorm);
		temp << nnorm / fenmu, -dnorm / fenmu, dnorm / fenmu, nnorm / fenmu;
		return temp;
	}

	Eigen::Matrix<double, 6, 4> ComputeJacobianFromPlukerToOrth(const Eigen::Matrix3d& u, const Eigen::Matrix2d& w)
	{
		double w1 = w(0, 0);
		double w2 = w(1, 0);
		Eigen::Vector3d u1 = u.col(0);
		Eigen::Vector3d u2 = u.col(1);
		Eigen::Vector3d u3 = u.col(2);
		Eigen::Matrix<double, 6, 4> temp;
		temp.setZero();
		temp.block<3, 1>(0, 1) = -w1 * u3;
		temp.block<3, 1>(0, 2) = w1 * u2;
		temp.block<3, 1>(0, 3) = -w2 * u1;
		temp.block<3, 1>(3, 0) = w2 * u3;
		temp.block<3, 1>(3, 2) = -w2 * u1;
		temp.block<3, 1>(3, 3) = w1 * u2;
		return temp;
	}

	void UpdateOrthCoordDFromDeltaD(Eigen::Vector4d& D, Eigen::Vector4d& deltaD, Eigen::Vector4d& plusD)
	{
		// ref: 2001, Adrien Bartol,Peter Sturm ,Structure-From-Motion Using Lines: Representation, Triangulation and Bundle Adjustment

		// theta --> U,  phi --> W
		Eigen::Vector3d theta = D.head(3);

		double phi = D(3);
		//Vector3d theta = orth.head(3);
		//double phi = orth[3];
		double s1 = sin(theta[0]);
		double c1 = cos(theta[0]);
		double s2 = sin(theta[1]);
		double c2 = cos(theta[1]);
		double s3 = sin(theta[2]);
		double c3 = cos(theta[2]);
		Eigen::Matrix3d R;
		R <<
			c2 * c3, s1 * s2 * c3 - c1 * s3, c1 * s2 * c3 + s1 * s3,
			c2 * s3, s1 * s2 * s3 + c1 * c3, c1 * s2 * s3 - s1 * c3,
			-s2, s1 * c2, c1 * c2;
		double w1 = cos(phi);
		double w2 = sin(phi);

		// update
		Eigen::Vector3d _delta_theta = deltaD.head(3);
		double _delta_phi = deltaD(3);
		Eigen::Matrix3d Rz;
		Rz << cos(_delta_theta(2)), -sin(_delta_theta(2)), 0,
			sin(_delta_theta(2)), cos(_delta_theta(2)), 0,
			0, 0, 1;

		Eigen::Matrix3d Ry;
		Ry << cos(_delta_theta(1)), 0., sin(_delta_theta(1)),
			0., 1., 0.,
			-sin(_delta_theta(1)), 0., cos(_delta_theta(1));

		Eigen::Matrix3d Rx;
		Rx << 1., 0., 0.,
			0., cos(_delta_theta(0)), -sin(_delta_theta(0)),
			0., sin(_delta_theta(0)), cos(_delta_theta(0));
		R = R * Rx * Ry * Rz;

		Eigen::Matrix2d W;
		W << w1, -w2, w2, w1;
		Eigen::Matrix2d delta_W;
		delta_W << cos(_delta_phi), -sin(_delta_phi), sin(_delta_phi), cos(_delta_phi);
		W = W * delta_W;

		// U' -- > theta'. W' --> phi'

		Eigen::Vector3d u1 = R.col(0);
		Eigen::Vector3d u2 = R.col(1);
		Eigen::Vector3d u3 = R.col(2);
		plusD[0] = atan2(u2(2), u3(2));
		plusD[1] = asin(-u1(2));
		plusD[2] = atan2(u1(1), u1(0));

		plusD[3] = asin(W(1, 0));

		//////////////////////////////////////////////////////////
		/*
		// SO3参数方法，得到的雅克比更上面一样的。用上面的形式就OK。
		Eigen::Map<const Eigen::Vector3d> theta(x);
		double phi = *(x + 3);
		double s1 = sin(theta[0]);
		double c1 = cos(theta[0]);
		double s2 = sin(theta[1]);
		double c2 = cos(theta[1]);
		double s3 = sin(theta[2]);
		double c3 = cos(theta[2]);
		Matrix3d R;
		R <<
		c2 * c3,   s1 * s2 * c3 - c1 * s3,   c1 * s2 * c3 + s1 * s3,
		c2 * s3,   s1 * s2 * s3 + c1 * c3,   c1 * s2 * s3 - s1 * c3,
		-s2,                  s1 * c2,                  c1 * c2;

		Sophus::SO3<double> U = Sophus::SO3<double>::exp(theta);
		Sophus::SO3<double> U1(R);

		std::cout << U.matrix() << "\n\n" <<U1.matrix()<<"\n\n"<<R<<"\n\n";

		std::cout << theta <<"\n\n" << U1.log() << "\n\n"<<  Sophus::SO3<double>::exp(U1.log()).matrix() << "\n\n";
		*/
	}


	Eigen::Matrix<double, 6, 1> PluckerLineNormalized(Eigen::Matrix<double, 6, 1> pluckerline)
	{
		/*Eigen::Vector3d ln = pluckerline.head(3);
		Eigen::Vector3d ld = pluckerline.tail(3);*/
		Eigen::Matrix<double, 6, 1> plu_line_normalized = pluckerline / pluckerline.norm();
		return plu_line_normalized;
	}

	Eigen::Matrix<double, 6, 1> PluckerLineDirNormalized(Eigen::Matrix<double, 6, 1> pluckerline)
	{
		Eigen::Vector3d ld = pluckerline.tail(3);
		Eigen::Vector3d ln = pluckerline.head(3);
		double lscale = 1 / ld.norm();
		Eigen::Matrix<double, 6, 1> plucker_d_line = pluckerline * lscale;
		return plucker_d_line;
	}

	Eigen::Matrix2d GetOrthWFromPlucker(const Eigen::Matrix<double, 1, 6>& pluckerline)
	{
		Eigen::Matrix2d temp;
		double nnorm = pluckerline.head(3).norm();
		double dnorm = pluckerline.tail(3).norm();
		double denom = std::sqrt(nnorm*nnorm + dnorm*dnorm);
		temp << nnorm / denom, -dnorm / denom, dnorm / denom, nnorm / denom;
		//temp << nnorm / denom, dnorm / denom, -dnorm / denom, nnorm / denom;	//check its

		return temp;
	}

	Eigen::Matrix3d GetOrthRMatrixFromPlucker(const Eigen::Matrix<double, 6, 1>& pluckerline)
	{
		Eigen::Matrix3d R;
		Eigen::Vector3d n = pluckerline.head(3);
		Eigen::Vector3d d = pluckerline.tail(3);
		Eigen::Vector3d nn = n.normalized();
		Eigen::Vector3d dn = d.normalized();
		R.col(0) = nn;
		R.col(1) = dn;
		R.col(2) = n.cross(d) / ((n.cross(d)).norm());
		return R;
	}

	//Ax+By+C =0
	Eigen::Vector3d GetLineFuncFromLine2Pnts2d(const Eigen::Vector2d& ls, const Eigen::Vector2d& le)
	{
		double A = le[1] - ls[1];
		double B = ls[0] - le[0];
		double C = le[0] * ls[1] - ls[0] * le[1];
		return Eigen::Vector3d(A, B, C);
	}

	void FileLineBufferProcess(std::string& line, const std::string& comment_str)
	{
		for (int i = 0; i < line.length(); ++i)
		{
			if (line[i] == ';' || line[i] == '\t'
				|| line[i] == ',' || line[i] == '\r')
				line[i] = ' ';
		}
		line.erase(0, line.find_first_not_of(" "));
		line.erase(line.find_last_not_of(" ") + 1);
		int n_comment_start = line.find_first_of(comment_str);
		if (n_comment_start != std::string::npos)
			line.erase(n_comment_start);
	}

	Eigen::Matrix3d VectorColMultiplyVectorRow3D(const Eigen::Vector3d& colv, const Eigen::Vector3d& rowv)
	{
		Eigen::Matrix3d tmp_m;
		tmp_m.block<1, 3>(0, 0) = colv[0] * rowv;
		tmp_m.block<1, 3>(1, 0) = colv[1] * rowv;
		tmp_m.block<1, 3>(2, 0) = colv[2] * rowv;
		return tmp_m;
	}

	Eigen::Matrix<double, 6, 1> GetPluckerLineFromLineEndPnts(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2)
	{
		Eigen::Matrix<double, 6, 1> plucker_line;
		Eigen::Vector3d plucker_ld = p2 - p1;
		plucker_ld.normalize();
		Eigen::Vector3d plucker_lm = p1.cross(plucker_ld);
		plucker_line.block<3, 1>(0, 0) = plucker_lm;
		plucker_line.block<3, 1>(3, 0) = plucker_ld;
		return plucker_line;
	}

	Eigen::Vector3d VectorDotMultiplyMatrix3d(const Eigen::Vector3d& v, const Eigen::Matrix3d& m)
	{
		Eigen::Vector3d tmp_vec0 = v[0] * m.block<1, 3>(0, 0);
		Eigen::Vector3d tmp_vec1 = v[1] * m.block<1, 3>(1, 0);
		Eigen::Vector3d tmp_vec2 = v[2] * m.block<1, 3>(2, 0);
		Eigen::Vector3d res = tmp_vec0 + tmp_vec1 + tmp_vec2;
		return res;
	}

	bool GetPairFromVectorPairsByPairFirst(const std::vector<std::pair<int, int> >& pairs_vec, int value_first, std::pair<int, int>& my_pair)
	{
		for (int i = 0; i < pairs_vec.size(); ++i)
		{
			if (pairs_vec[i].first == value_first)
			{
				my_pair = pairs_vec[i];
				return true;
			}
		}
		return false;
	}

	bool GetPairFromVectorPairsByPairSecond(const std::vector<std::pair<int, int> >& pairs_vec, int value_second, std::pair<int, int>& my_pair)
	{
		for (int i = 0; i < pairs_vec.size(); ++i)
		{
			if (pairs_vec[i].second == value_second)
			{
				my_pair = pairs_vec[i];
				return true;
			}
		}
		return false;
	}
}