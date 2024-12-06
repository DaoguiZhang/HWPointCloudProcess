#include"hw_cmns.h"
#include<boost/filesystem.hpp>
#include<unordered_set>

namespace HW
{
    bool SortMatchesByIDs(const HWMatch m1, const HWMatch m2)
    {
        if(m1.tgt_camID_ < m2.tgt_camID_)
            return true;
        else if(m1.tgt_camID_ == m2.tgt_camID_ && m1.tgt_segID_ < m2.tgt_segID_)
            return true;
        else
            return false;
    }

    bool IsNotSpacialSpace(const int c)
    {
        return (c != ' ' && c != '\r' && c != '\t' && c != '\n' );
    }

    //replace old str with new str
    std::string ReplaceStrWithNewStr(const std::string& str, const std::string& old_str,
        const std::string& new_str)
    {
        if(old_str.empty())
            return str;
        std::size_t position = 0;
        std::string mod_str = str;
        while((position = mod_str.find(old_str, position)) != std::string::npos)
        {
            mod_str.replace(position, old_str.size(), new_str);
            position += new_str.size();
        }
        return mod_str;
    }

    std::string EnsureTrailingSlashHW(const std::string& str) 
    {
        if (str.length() > 0) 
        {
            if (str.back() != '/') 
            {
                return str + "/";
            }
        } 
        else 
        {
            return str + "/";
        }
        return str;
    }

    /*
    path: get left slash path
    */
    std::string GetLeftSlashPathName(const std::string& path)
    {
        std::string new_path = ReplaceStrWithNewStr(path, "\\", "/");
        return new_path;
    }

	std::string GetDirFromPathName(const std::string& path)
	{
		std::string prefix_dir = path.substr(0, path.find_last_of("/"));
		return prefix_dir;
	}

    std::string GetBaseName(const std::string& path)
    {
        std::vector<std::string> names = SplitStrWithDelim(path, "/");
        return names.back();
    }

    std::string GetPathPrefix(const std::string& path)
    {
        std::string prefix = path.substr(0, path.find_last_of("."));
        return prefix;
    }

    std::string GetBaseNameWithoutSuffix(const std::string& path)
    {
        std::string str_name = path.substr(path.find_last_of("/") + 1, 
            path.find_last_of(".") - path.find_last_of("/") - 1);
        return str_name;
    }
    
    std::vector<std::string> GetDirListFromDir(const std::string& path)
    {
        std::vector<std::string> file_list;
        for(boost::filesystem::directory_iterator it = boost::filesystem::directory_iterator(path);
            it != boost::filesystem::directory_iterator(); ++it)
        {
            if(boost::filesystem::is_directory(*it))
            {
                const boost::filesystem::path filepath = *it;
                file_list.push_back(filepath.string());
            }    
        }
        return file_list;
    }

    std::vector<std::string> GetDirListFromDirRecursive(const std::string& path)
    {
        std::vector<std::string> file_list;
        for(boost::filesystem::recursive_directory_iterator it = boost::filesystem::recursive_directory_iterator(path);
            it != boost::filesystem::recursive_directory_iterator(); ++it)
        {
            if(boost::filesystem::is_directory(*it))
            {
                const boost::filesystem::path file_path = *it;
                file_list.emplace_back(file_path.string());
            }
        }
        return file_list;
    }

    /*
    path: get path files lists from dir(without recursive)
    */
    std::vector<std::string> GetFilesListFromDir(const std::string& path)
    {
        std::vector<std::string> file_list;
        for(boost::filesystem::directory_iterator it = boost::filesystem::directory_iterator(path);
            it != boost::filesystem::directory_iterator(); ++it)
        {
            if(boost::filesystem::is_regular_file(*it))
            {
                const boost::filesystem::path filepath = *it;
                file_list.push_back(filepath.string());
            }    
        }
        return file_list;
    }

    std::vector<std::string> GetFilesListFromDirRecursive(const std::string& path)
    {
        std::vector<std::string> file_list;
        for(boost::filesystem::recursive_directory_iterator it = boost::filesystem::recursive_directory_iterator(path);
            it != boost::filesystem::recursive_directory_iterator(); ++it)
        {
            if(boost::filesystem::is_regular_file(*it))
            {
                const boost::filesystem::path file_path = *it;
                file_list.emplace_back(file_path.string());
            }
        }
        return file_list;
    }

    std::vector<std::string> SplitStrWithDelim(const std::string& str, const std::string& delim)
    {
        std::vector<std::string> elems;
        boost::split(elems, str, boost::is_any_of(delim), boost::token_compress_on);
        return elems;
    }

    int FindStrIdxFromVecStrs(const std::string& srcstr, const std::vector<std::string>& vecstrs)
    {
        if(srcstr.empty())
            return -1;
        for(std::size_t i = 0; i < vecstrs.size(); ++i)
        {
            if(srcstr.compare(vecstrs[i]) == 0)
            {
                return i;
            }
        }
        return -1;
    }

	int FindStrIdxFromVecStrsNew(const std::string& srcstr, const std::vector<std::string>& vecstrs)
	{
		if (srcstr.empty())
			return -1;
		for (std::size_t i = 0; i < vecstrs.size(); ++i)
		{
			if (srcstr == vecstrs[i])
			{
				return i;
			}
		}
		return -1;
	}

    void StringLeftTrim(std::string* str)
    {
        str->erase(str->begin(),
             std::find_if(str->begin(), str->end(), IsNotSpacialSpace));
    }

    void StringRightTrim(std::string* str)
    {
        str->erase(std::find_if(str->rbegin(), str->rend(), IsNotSpacialSpace).base(),
             str->end());
    }

    void StringTrimLR(std::string* str)
    {
        StringLeftTrim(str);
        StringRightTrim(str);
    }

	bool ElementVectorContainsDuplicatedElement(std::vector<int>& nums_vec)
	{
		std::unordered_set<int> nums;
		for (int i = 0; i < nums_vec.size(); ++i)
		{
			if (nums.find(nums_vec[i]) != nums.end())
			{
				return true;
			}
			nums.insert(nums_vec[i]);
		}
		return false;
	}

    void StringToLowerLetters(std::string* str)
    {
        std::transform(str->begin(), str->end(), str->begin(), ::tolower);
    }

    void StringToUpperLetters(std::string* str)
    {
        std::transform(str->begin(), str->end(), str->begin(), ::toupper);
    }

    std::vector<std::string> ReadTextFileLines(const std::string& path) 
    {
        std::ifstream file(path);
        //CHECK(file.is_open()) << path;
        std::string line;
        std::vector<std::string> lines;
        while (std::getline(file, line)) 
        {
            StringTrimLR(&line);
            if (line.empty()) 
            {
                continue;
            }

            lines.push_back(line);
        }
        return lines;
    }

    void SamplePntsFromLineEndPnts2D(Eigen::Vector2f& s, Eigen::Vector2f& e, std::vector<Eigen::Vector2i>& pnts)
    {
        //to do next
        //start compute cross with height and width
        if(!pnts.empty())
        {
            pnts.clear();
        }
        int x0 = static_cast<int>(s[0]);
        int y0 = static_cast<int>(s[1]);
        int x1 = static_cast<int>(e[0]);
        int y1 = static_cast<int>(e[1]);
        int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
        int dy = std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
        int err = (dx > dy ? dx : -dy) / 2, e2;
        for(;;)
        {
            pnts.emplace_back(Eigen::Vector2i(x0, y0));
            if(x0 == x1 && y0 == y1)
            {
                break;
            }
            e2 = err;
            if(e2 > -dx)
            {
                err -= dy;
                x0 += sx;
            }
            if(e2 < dy)
            {
                err += dx;
                y0 += sy;
            }
        }
    }

    bool CheckPoint2dInRectd(Eigen::Vector2f& pnt, Eigen::Vector2f& mincor, Eigen::Vector2f& maxcor)
    {
        if(pnt[0] >= mincor[0] && pnt[0] <= maxcor[0]
            && pnt[1] >= mincor[1] && pnt[1] <= maxcor[1])
        {
            return true;
        }
        return false;
    }

    bool CheckPoint2dInRecti(Eigen::Vector2i& pnt, Eigen::Vector2i& mincor, Eigen::Vector2i& maxcor)
    {
        if(pnt[0] >= mincor[0] && pnt[0] <= maxcor[0]
            && pnt[1] >= mincor[1] && pnt[1] <= maxcor[1])
        {
            return true;
        }
        return false;
    }

    //Ax+By+C=0(A,B,C)
    Eigen::Vector3f ComputeFunctionFromPnts(const Eigen::Vector2f& s, const Eigen::Vector2f& e)
    {
        float A = e[1] - s[1];
        float B = s[0] - e[0];
        float C = -(A*s[0] + B*s[1]);
        return Eigen::Vector3f(A,B,C);
    }

    bool ComputeLine2LineIntersection(const Eigen::Vector3f& f1, const Eigen::Vector3f& f2, Eigen::Vector2f& pnt)
    {
        float det = f1[0]*f2[1] - f2[0]*f1[1];
        if(std::abs(det) < KMIN_DOUBLE_LINE_THRESHOLD)
        {
            return false;
        }
        else
        {
            float x = (f1[1]*f2[2] - f2[1]*f1[2]) / det;
            float y = (f2[0]*f1[2] - f1[0]*f2[2]) / det;
            pnt[0] = x;
            pnt[1] = y;
            return true;
        }
    }

    float LiangBarskVecsMax(const std::vector<float>& v)
    {
    	float vmax = 0.0;
    	for(std::size_t i = 0; i < v.size(); ++i)
    	{
    		if(vmax < v[i])
    		{
    			vmax = v[i];
    		}
    	}
    	return vmax;
    }

    float LiangBarskVecsMin(const std::vector<float>& v)
    {
    	float vmin = 1.0;
    	for(std::size_t i = 0; i < v.size(); ++i)
    	{
    		if(vmin > v[i])
    		{
    			vmin = v[i];
    		}
    	}
    	return vmin;
    }

    //0 reject, 1 clip
    int LiangBarskyClipperAlgo(float xmin, float ymin, float xmax, float ymax,
		float x1, float y1, float x2, float y2, float& x1_new, float& y1_new, float& x2_new, float& y2_new)
    {
    	//defining variables
		float xd = x2 - x1; float yd = y2 - y1;
		float p1 = -xd; float p2 = xd; float p3 = -yd; float p4 = yd;
		float q1 = x1 - xmin; float q2 = xmax - x1;
		float q3 = y1 - ymin; float q4 = ymax - y1;
		float t1 = 0; float t2 = 1;
    	std::vector<float> negarray;
    	std::vector<float> posarray;
    	if((p1 == 0.0 && q1 < 0.0) || (p3 == 0.0 && q3 < 0.0))
    	{
    		//reject
    		//std::cerr << "reject ... " << std::endl;
    		return 0;	//reject
    	}
    	if(p1 != 0)
    	{
			float r1 = q1 / p1;
			float r2 = q2 / p2;
    		if(p1 < 0)
    		{
    			negarray.push_back(r1);
    			posarray.push_back(r2);
    		}
    		else
    		{
    			posarray.push_back(r1);
    			negarray.push_back(r2);
    		}
    	}
    	if(p3 != 0)
    	{
			float r3 = q3 / p3;
			float r4 = q4 / p4;
    		if(p3 < 0)
    		{
    			negarray.push_back(r3);
    			posarray.push_back(r4);
    		}
    		else
    		{
    			posarray.push_back(r3);
    			negarray.push_back(r4);
    		}
    	}
		float rn1 = LiangBarskVecsMax(negarray);
		float rn2 = LiangBarskVecsMin(posarray);
    	if(rn1 > rn2)
    	{
    		//reject
    		//std::cerr << "reject ... " << std::endl;
    		return 0;
    	}
    	//compute new pnts
    	x1_new = x1 + p2 * rn1;
    	y1_new = y1 + p4 * rn1;
    	x2_new = x1 + p2 * rn2;
    	y2_new = y1 + p4 * rn2;

    	return 1;
    }

	void rodrigues_to_matrix3d(double const* r, double* m)
	{
#if 1
		/* Obtain angle from vector length. */
		double a = std::sqrt(r[0] * r[0] + r[1] * r[1] + r[2] * r[2]);
		/* Precompute sine and cosine terms. */
		double ct = (a == 0.0) ? 0.5f : (1.0f - std::cos(a)) / (2.0 * a);
		double st = (a == 0.0) ? 1.0 : std::sin(a) / a;
		/* R = I + st * K + ct * K^2 (with cross product matrix K). */
		m[0] = 1.0 - (r[1] * r[1] + r[2] * r[2]) * ct;
		m[1] = r[0] * r[1] * ct - r[2] * st;
		m[2] = r[2] * r[0] * ct + r[1] * st;
		m[3] = r[0] * r[1] * ct + r[2] * st;
		m[4] = 1.0f - (r[2] * r[2] + r[0] * r[0]) * ct;
		m[5] = r[1] * r[2] * ct - r[0] * st;
		m[6] = r[2] * r[0] * ct - r[1] * st;
		m[7] = r[1] * r[2] * ct + r[0] * st;
		m[8] = 1.0 - (r[0] * r[0] + r[1] * r[1]) * ct;
#else
		/* Obtain angle from vector length. */
		double theta = std::sqrt(r[0] * r[0] + r[1] * r[1] + r[2] * r[2]);
		Eigen::Vector3d u;
		u(0) = r[0] / theta;
		u(1) = r[1] / theta;
		u(2) = r[2] / theta;
		double c = std::cos(theta);
		double s = std::sin(theta);
		double t = 1 - c;
		m[0] = c + u(0)*u(0)*t;
		m[1] = u(0)*u(1)*t - u(2)*s;
		m[2] = u(0)*u(2)*t + u(1)*s;
		m[3] = u(0)*u(1)*t + u(2)*s;
		m[4] = c + u(1)*u(1)*t;
		m[5] = u(1)*u(2)*t - u(0)*s;
		m[6] = u(0)*u(2)*t - u(1)*s;
		m[7] = u(1)*u(2)*t + u(0)*s;
		m[8] = c + u(2)*u(2)*t;
#endif
	}

	void ConvertMatrix3d_to_Matrix3d_Transposed(double const* r, double* t_r)
	{
		t_r[0] = r[0]; t_r[1] = r[3]; t_r[2] = r[6];		
		t_r[3] = r[1]; t_r[4] = r[4]; t_r[5] = r[7];
		t_r[6] = r[2]; t_r[7] = r[5]; t_r[8] = r[8];
	}

	Eigen::Matrix3d ComputeSkewMatrixFromVector3d(const Eigen::Vector3d& v)
	{
		Eigen::Matrix3d m;
		m << 0, -v(2), v(1),
			v(2), 0, -v(0),
			-v(1), v(0), 0;
		return m;
	}

	Eigen::Vector3d RotationMatrixToAxisAngleD(const Eigen::Matrix3d& R)
	{
#if 0
		double tr = R(0, 0) + R(1, 1) + R(2, 2);
		double theta = std::acos((tr - 1) / 2);
		double ux = (R(2, 1) - R(1, 2)) / (2 * std::sin(theta));
		double uy = (R(0, 2) - R(2, 0)) / (2 * std::sin(theta));
		double uz = (R(1, 0) - R(0, 1)) / (2 * std::sin(theta));
		Eigen::Vector3d v = Eigen::Vector3d(ux*theta, uy*theta, uz*theta);
		return v;
#else
		double tr = R.trace();
		double theta;
		if (tr > 3 - KMIN_DOUBLE_LIMIT_THRESHOLD)
		{
			//case when the trace is close to 3 i.e, angle close to 0
			theta = 0.0;
		}
		else if (tr < -1 + KMIN_DOUBLE_LIMIT_THRESHOLD)
		{
			//case when the trace is close to -1 i.e angle close to pi
			theta = CMATH_PI_V;
		}
		else
		{
			//general case
			theta = std::acos((tr - 1) / 2);
		}
		Eigen::Vector3d v;
		if (theta < KMIN_DOUBLE_LIMIT_THRESHOLD)
		{
			v = Eigen::Vector3d(0.0, 0.0, 0.0);
		}
		else if (theta > CMATH_PI_V - KMIN_DOUBLE_LIMIT_THRESHOLD)
		{
			//case when the angle is close to pi
			Eigen::Vector3d diag = R.diagonal();
			int idx = 0;
			if (diag(1) > diag(idx))
			{
				idx = 1;
			}
			if (diag(2) > diag(idx))
			{
				idx = 2;
			}
			Eigen::Vector3d temp = 0.5*std::sqrt((diag(idx) + 1) / (1 - tr)) * (R.col(idx) + Eigen::Vector3d::Unit(idx));
			v = temp.normalized() * CMATH_PI_V;
		}
		else
		{
			//general case
			v = (theta / (2 * std::sin(theta)))*Eigen::Vector3d(R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1));
		}
		return v;
#endif
	}

	Eigen::Matrix3d AxisAngleToRotationMatrixD(const Eigen::Vector3d& v)
	{
#if 0
		double theta = v.norm();
		Eigen::Vector3d u = v / theta;
		double c = std::cos(theta);
		double s = std::sin(theta);
		double t = 1 - c;
		Eigen::Matrix3d R;
		R(0, 0) = c + u(0)*u(0)*t;
		R(1, 1) = c + u(1)*u(1)*t;
		R(2, 2) = c + u(2)*u(2)*t;
		R(0, 1) = u(0)*u(1)*t - u(2)*s;
		R(0, 2) = u(0)*u(2)*t + u(1)*s;
		R(1, 0) = u(0)*u(1)*t + u(2)*s;
		R(1, 2) = u(1)*u(2)*t - u(0)*s;
		R(2, 0) = u(0)*u(2)*t - u(1)*s;
		R(2, 1) = u(1)*u(2)*t + u(0)*s;
		return R;
#else
		double theta = v.norm();
		if (theta < KMIN_DOUBLE_LIMIT_THRESHOLD)
		{
			//case when the angle is close to 0
			return Eigen::Matrix3d::Identity();
		}
		else
		{
			//general case
			Eigen::Vector3d axis = v / theta;
			Eigen::Matrix3d axis_cross;
			axis_cross << 0.0, -axis(2), axis(1),
				axis(2), 0.0, -axis(0),
				-axis(1), axis(0), 0.0;
			return Eigen::Matrix3d::Identity() + std::sin(theta)*axis_cross + (1 - std::cos(theta))*axis_cross*axis_cross;
		}
#endif
		
	}

	bool IsStrFrontElementInteger(const std::string& e)
	{
		if (e.empty())
			return false;
		if (e[0] >= '0' && e[0] <= '9')
			return true;
		return false;
	}
}
