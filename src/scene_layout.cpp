#include"scene_layout.h"


//SceneLayout::SceneLayout(SceneLayout& other)
//{
//	//拷贝构造函数
//	pnts_ = other.GetLayoutPnts();
//	lines_segs_ = other.GetLayoutLine2PntIdx();
//}

void SceneLayout::SetSceneLayoutType(SceneLayDimType& my_t)
{
	ly_type_dimension_ = my_t;
}

SceneLayDimType SceneLayout::GetSceneLayoutType()
{
	return ly_type_dimension_;
}

void SceneLayout::SetSceneLayout3D(std::vector<Eigen::Vector3f>& pnts)
{
	//
	if (pnts.size() < 2)
		return;
	for (int i = 0; i < pnts.size(); ++i)
	{
		LinePoint p;
		p.pnt_ = pnts[i];
		pnts_.emplace_back(p);
	}
	for (int i = 0; i < pnts.size() - 1; ++i)
	{
		Eigen::Vector2i p_idx;
		p_idx[0] = i;
		p_idx[1] = i + 1;
		lines_segs_.emplace_back(p_idx);
	}
}

void SceneLayout::SetPlanePolygonsVecIdxs(std::vector<int>& idxs)
{
	associated_polys_idxs_ = idxs;
}

void SceneLayout::SetCamsVecIdxs(int idxs)
{
	ly_cam_idx_ = idxs;
}

void SceneLayout::SetLayoutName(std::string& lyout_path)
{
	layout_path_ = lyout_path;
}

void SceneLayout::SetAllPolygonsSceneLayout3D(std::vector<std::vector<Eigen::Vector3f> >& pnts)
{
	//
	//std::vector<Eigen::Vector3f> pnts;
	int idx = 0;
	//这是对polygon的线段
	for (int i = 0; i < pnts.size(); ++i)
	{
		PolygonLineId polygon_line;
		for (int j = 0; j < pnts[i].size(); ++j)
		{
			LinePoint p;
			p.pnt_ = pnts[i][j];
			pnts_.emplace_back(p);
		}
		int line_pnt_count = idx;
		for (; line_pnt_count < pnts_.size() - 1; ++line_pnt_count)
		{
			Eigen::Vector2i p_idx;
			p_idx[0] = line_pnt_count;
			p_idx[1] = line_pnt_count + 1;
			lines_segs_.emplace_back(p_idx);
			//将其放入到一个平面
			polygon_line.Plane2Linesidx.emplace_back(lines_segs_.size() - 1);
		}
		if (line_pnt_count == (pnts_.size() - 1))	//因为是polygon，首尾形成一个边
		{
			Eigen::Vector2i p_idx;
			p_idx[0] = line_pnt_count;
			p_idx[1] = idx;
			lines_segs_.emplace_back(p_idx);
			//将其放入到一个平面
			polygon_line.Plane2Linesidx.emplace_back(lines_segs_.size() - 1);
		}
		planes_polygon_lines_idx_.emplace_back(polygon_line);
		idx += pnts[i].size();
	}
}

void SceneLayout::SetSceneLayout2D(std::vector<Eigen::Vector2f>& pnts)
{
	//
	if (pnts.size() < 2)
		return;
	for (int i = 0; i < pnts.size(); ++i)
	{
		LinePoint p;
		p.pnt_ = Eigen::Vector3f(pnts[i][0], pnts[i][1], 1.0);
		pnts_.emplace_back(p);
	}
	for (int i = 0; i < pnts.size() - 1; ++i)
	{
		Eigen::Vector2i p_idx;
		p_idx[0] = i;
		p_idx[1] = i + 1;
		lines_segs_.emplace_back(p_idx);
	}
}

void SceneLayout::SetSceneLayout2DFromNetPnts(std::vector<std::vector<float> >& layout_pnts)
{
	std::vector<Eigen::Vector2f> laypnts;
	std::vector<Eigen::Vector2i> lines_idx;
	for (int i = 0; i < layout_pnts.size(); ++i)
	{
		std::vector<float> line_pnts = layout_pnts[i];
		Eigen::Vector2f spnt = Eigen::Vector2f(line_pnts[0], line_pnts[1]);
		Eigen::Vector2f epnt = Eigen::Vector2f(line_pnts[2], line_pnts[3]);
		laypnts.emplace_back(spnt);
		laypnts.emplace_back(epnt);
		Eigen::Vector2i l_idx = Eigen::Vector2i(2 * i, 2 * i + 1);
		lines_idx.emplace_back(l_idx);
	}
	pnts_.clear();
	lines_segs_.clear();
	for (int i = 0; i < laypnts.size(); ++i)
	{
		LinePoint p;
		p.pnt_ = Eigen::Vector3f(laypnts[i][0], laypnts[i][1], 0.0);
		pnts_.emplace_back(p);
	}
	lines_segs_ = lines_idx;
	SceneLayDimType value = SceneLayDimType::k2DType;
	SetSceneLayoutType(value);
}

void SceneLayout::ReadLayout3D(const std::string& path)
{
	//
	std::cout << "the currrent reading log name is: " << path << std::endl;

	std::ifstream fh(path);
	if (fh.is_open())
	{
		std::string line_path;
		std::getline(fh, line_path);
		std::cout << "ReadLayout3D the line path: " << line_path << std::endl;

		std::string line_head;
		std::getline(fh, line_head);
		std::stringstream ss(line_head);
		int points_num = 0;
		int lines_num = 0;
		ss >> points_num >> lines_num;
		std::cout << "the points num , lines num: " << points_num << ", " << lines_num << std::endl;

		for (int i = 0; i < points_num; ++i)
		{
			//从文件中读取顶点的坐标
			std::string line_buffer;
			std::getline(fh, line_buffer);
			std::stringstream ss_f(line_buffer);
			float x, y, z;
			ss_f >> x >> y >> z;
			std::cout << x << " " << y << " " << z << std::endl;
			LinePoint p;
			p.pnt_[0]= x;
			p.pnt_[1] = y;
			p.pnt_[2] = z;
			//将顶点加入到数据结构中
			pnts_.emplace_back(p);
		}

		for (int i = 0; i < lines_num; ++i)
		{
			//从文件中读取顶点的索引
			std::string line_buffer;
			std::getline(fh, line_buffer);
			std::stringstream ss_i(line_buffer);
			int idx_s, idx_e;
			ss_i >> idx_s >> idx_e;

			std::cout << idx_s << " " << idx_e << std::endl;
			Eigen::Vector2i p_idx;
			p_idx[0] = idx_s;
			p_idx[1] = idx_e;

			//获取边的索引
			lines_segs_.emplace_back(p_idx);
		}
		fh.close();
	}
}

void SceneLayout::ReadLayout2D(const std::string& path)
{
	std::cout << "the currrent reading log name is: " << path << std::endl;

	std::ifstream fh(path);
	if (fh.is_open())
	{
		std::string line_path;
		std::getline(fh, line_path);
		//std::cout << "ReadLayout2D the line path: " << line_path << std::endl;
		std::string line_head;
		std::getline(fh, line_head);
		std::stringstream ss(line_head);
		int points_num = 0;
		int lines_num = 0;
		ss >> points_num >> lines_num;
		std::cout << "the points num , lines num: " << points_num << ", " << lines_num << std::endl;

		//读取顶点
		for (int i = 0; i < points_num; ++i)
		{
			//从文件中读取顶点的坐标
			std::string line_buffer;
			std::getline(fh, line_buffer);
			std::stringstream ss_f(line_buffer);
			float x, y;
			ss_f >> x >> y;
			std::cout << x << " " << y << std::endl;
			LinePoint p;
			p.pnt_[0] = x;
			p.pnt_[1] = y;
			p.pnt_[2] = 0.0;

			//将顶点加入到数据结构中
			pnts_.emplace_back(p);
		}

		for (int i = 0; i < lines_num; ++i)
		{
			//从文件中读取顶点的索引
			std::string line_buffer;
			std::getline(fh, line_buffer);
			std::stringstream ss_i(line_buffer);
			int idx_s, idx_e;
			ss_i >> idx_s >> idx_e;

			std::cout << idx_s << " " << idx_e << std::endl;
			Eigen::Vector2i p_idx;
			p_idx[0] = idx_s;
			p_idx[1] = idx_e;
			if (idx_s == -1 || idx_e == -1)
				continue;
			lines_segs_.emplace_back(p_idx);
		}
		fh.close();
	}
	
	layout_path_ = path;
}

bool SceneLayout::ReadLayout2DFromNetPath(const std::string& path)
{
	std::vector<std::vector<float> > lines_pnts;
	if (ReadLayout2DFromNetData(path, lines_pnts))
	{
		SetSceneLayout2DFromNetPnts(lines_pnts);
		//PrintLayoutDebug();
		return true;
	}
	return false;
}

bool SceneLayout::ReadLayout2DFromNetData(const std::string& path, std::vector<std::vector<float> >& lines_pnts)
{
	std::cout << "the currrent reading log name is: " << path << std::endl;
	std::ifstream fh(path);
	if (fh.is_open())
	{
		if (!fh.eof())
		{
			std::string file_path;
			std::getline(fh, file_path);
			std::cerr << "the file path: " << file_path << std::endl;

			std::string line;
			std::getline(fh, line);
			std::stringstream ss(line);
			int lines_num;
			ss >> lines_num;
			std::cout << "line num: " << lines_num << std::endl;
			for (int i = 0; i < lines_num; ++i)
			{
				std::vector<float> line_pnt(4, 0.0);
				std::string linebuf;
				std::getline(fh, linebuf);
				if (fh.eof())
				{
					std::cerr << "file: end eof" << std::endl;
					return true;
				}
				std::stringstream ssbuf(linebuf);
				ssbuf >> line_pnt[0] >> line_pnt[1] >> line_pnt[2] >> line_pnt[3];
				std::cerr << "line pnt: " << line_pnt[0] << ", " << line_pnt[1] << ", "
					<< line_pnt[2] << ", " << line_pnt[3] << std::endl;
				lines_pnts.emplace_back(line_pnt);
			}
		}
		fh.close();
		return true;
	}
	return false;
}

void SceneLayout::GetPickedLineSeg(int idx, Eigen::Vector3f& start_p, Eigen::Vector3f& end_p)
{
	if (idx < 0 || idx >= lines_segs_.size())
		return;
	start_p = pnts_[lines_segs_[idx][0]].pnt_;
	end_p = pnts_[lines_segs_[idx][1]].pnt_;
}

void SceneLayout::GetPickedLineSegSamplePnts(int idx, std::vector<Eigen::Vector2i>& lseg_pnts)
{
	if (idx < 0 || idx >= img_lines_pnts_.size())
		return;
	lseg_pnts = img_lines_pnts_[idx];
}

std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f> > SceneLayout::GetAllLineSegsPnts()
{
	std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f> > lines_pnts;
	for (std::size_t i = 0; i < lines_segs_.size(); ++i)
	{
		//std::vector<Eigen::Vector2f, Eigen::Vector2f> tmp_pnts;
		int sid = lines_segs_[i][0];
		int eid = lines_segs_[i][1];
		Eigen::Vector2f sp = Eigen::Vector2f(pnts_[sid].pnt_[0], pnts_[sid].pnt_[1]);
		Eigen::Vector2f ep = Eigen::Vector2f(pnts_[eid].pnt_[0], pnts_[eid].pnt_[1]);
		lines_pnts.emplace_back(std::make_pair(sp, ep));
	}
	return lines_pnts;
}

int SceneLayout::LinesSize()
{
	return lines_segs_.size();
}

void SceneLayout::SamplePntsFromAllLineEndPnts()
{
	for (int i = 0; i < lines_segs_.size(); ++i)
	{
		std::vector<Eigen::Vector2i> line_img_pnts;
		Eigen::Vector3f start_p = pnts_[lines_segs_[i][0]].pnt_;
		Eigen::Vector3f end_p = pnts_[lines_segs_[i][1]].pnt_;
		SamplePntsFromALineEndPnts(start_p, end_p, line_img_pnts);
		img_lines_pnts_.emplace_back(line_img_pnts);
	}
}

void SceneLayout::SamplePntsFromALineEndPnts(Eigen::Vector3f& s, Eigen::Vector3f& e, std::vector<Eigen::Vector2i>& img_pnts)
{
	img_pnts.clear();
	int x0 = s[0];
	int y0 = s[1];
	int x1 = e[0];
	int y1 = e[1];
	int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
	int dy = std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
	int err = (dx > dy ? dx : -dy) / 2, e2;
	for (;;)
	{
		img_pnts.emplace_back(Eigen::Vector2i(x0, y0));
		if (x0 == x1 && y0 == y1) break;
		e2 = err;
		if (e2 > -dx)
		{
			err -= dy;
			x0 += sx;
		}
		if (e2 < dy)
		{
			err += dx;
			y0 += sy;
		}
	}
}

bool SceneLayout::CheckSameLines(Eigen::Vector2f s0, Eigen::Vector2f e0, Eigen::Vector2f s1, Eigen::Vector2f e1, float threshold)
{
	float dis00 =std::sqrtf((s0 - s1).dot(s0 - s1));
	float dis11 = std::sqrtf((e0 - e1).dot(e0 - e1));

	float dis01 = std::sqrtf((s0 - e1).dot(s0 - e1));
	float dis10 = std::sqrtf((s1 - e0).dot(s1 - e0));

	if (std::abs(dis00) < threshold && std::abs(dis11) < threshold)
	{
		return true;
	}
	else if (std::abs(dis01) < threshold && std::abs(dis10) < threshold)
	{
		return true;
	}

	return false;

}

bool SceneLayout::CheckSamePnts(Eigen::Vector2f s, Eigen::Vector2f e, float threshold)
{
	float dis = std::sqrtf((s - e).dot(s - e));
	if (std::abs(dis) < threshold)
	{
		return true;
	}
	return false;
}

int SceneLayout::FindVecIdx(std::vector<Eigen::Vector2f>& vec, Eigen::Vector2f s)
{
	float threshold = 2.0;
	for (int i = 0; i < vec.size(); ++i)
	{
		if (CheckSamePnts(vec[i], s, threshold))
		{
			return i;
		}
	}
	return -1;
}

const std::string& SceneLayout::GetLayoutPath()
{
	return layout_path_;
}

std::string SceneLayout::ReplacePatternString(std::string& str, const std::string& old_str, const std::string& new_str)
{
	while (true)
	{
		std::string::size_type pos(0);
		if ((pos = str.find(old_str)) != std::string::npos)
		{
			str.replace(pos, old_str.length(), new_str);
		}
		else
		{
			break;
		}
	}
	return str;
}

std::string SceneLayout::ExtractFileNameNosufixNodir(std::string path)
{
	//获取文件名
	std::string file_name = path.substr(path.find_last_of("\\") + 1, path.size() - path.find_last_of("\\") - 1);
	//std::cout << "the file name is: " << file_name << std::endl;
	std::string file_name_nosufix = file_name.substr(0, file_name.find_last_of("."));
	//std::cout << "the log_img_name_nosufix name is: " << file_name_nosufix << std::endl;
	return file_name_nosufix;
}

std::vector<LinePoint>& SceneLayout::GetLayoutPnts()
{
	return pnts_;
}

std::vector<Eigen::Vector2i>& SceneLayout::GetLayoutLine2PntIdx()
{
	return lines_segs_;
}

void SceneLayout::UpdateRedauntLineSegments()
{
	std::cerr << "SceneLayout:  filter the lines" << std::endl;
	for (std::vector<Eigen::Vector2i>::iterator iter = this->lines_segs_.begin();
		iter != this->lines_segs_.end();)
	{
		if ((*iter)[0] == (*iter)[1])
			iter = this->lines_segs_.erase(iter);
		else
			iter++;
	}
	std::cerr << "SceneLayout: end filter the lines" << std::endl;
}

void SceneLayout::PrintLayoutDebug()
{
	std::cerr << "Print Layout Start: " << std::endl;
	for (int i = 0; i < pnts_.size(); ++i)
	{
		std::cerr << pnts_[i].pnt_[0] << " " << pnts_[i].pnt_[1]
			<< " " << pnts_[i].pnt_[2] << std::endl;
	}
	for (int i = 0; i < lines_segs_.size(); ++i)
	{
		std::cerr << lines_segs_[i][0] << " " << lines_segs_[i][1] << std::endl;
	}
	std::cerr << "End Print Layout." << std::endl;
}

void SceneLayout::Write3DLinesIntoObj(const std::string& path)
{
	//
	std::ofstream fh(path);
	for (int i = 0; i < pnts_.size(); i++)
	{
		Eigen::Vector3f start_pnt = pnts_[i].pnt_;
		fh << "v " << start_pnt[0] << " " << start_pnt[1] << " " << start_pnt[2] << std::endl;
	}
	for (int i = 0; i < lines_segs_.size(); i++) {
		fh << "l " << lines_segs_[i][0] + 1 << " " << lines_segs_[i][1] + 1 << "\n";
	}
	fh.close();
}

void SceneLayout::WriteSelected3DLineIntoObj(Eigen::Vector3f& s, Eigen::Vector3f& e, const std::string& path)
{
	std::ofstream fh(path);
	fh << "v " << s[0] << " " << s[1] << " " << s[2] << std::endl;
	fh << "v " << e[0] << " " << e[1] << " " << e[2] << std::endl;
	fh << "l " << 1 << " " << 2 << "\n";
	fh.close();
}

SceneLayout SceneLayout::operator=(SceneLayout& other)
{
	SceneLayout c;
	c.lines_segs_ = other.lines_segs_;
	c.pnts_ = other.pnts_;
	c.ly_cam_idx_ = other.ly_cam_idx_;
	c.ly_type_dimension_ = other.ly_type_dimension_;
	c.associated_polys_idxs_ = other.associated_polys_idxs_;
	c.remained_img_lines_segs_idxs_ = other.remained_img_lines_segs_idxs_;
	c.layout_path_ = other.layout_path_;
	return c;
}