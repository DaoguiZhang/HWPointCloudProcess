#include"hw_scene_layout3d.h"

namespace HW
{
    HWSceneLayout3D::HWSceneLayout3D()
    {

    }

    HWSceneLayout3D::~HWSceneLayout3D()
    {

    }

    void HWSceneLayout3D::ReadLayout3D(const std::string& path)
    {
        std::cerr <<"start to load layout 3d from log file..." << std::endl;
        std::ifstream fh(path);
        if(fh.is_open())
        {
            std::string line_path;
            std::getline(fh, line_path);
            std::string line_head;
            std::getline(fh, line_head);
            std::stringstream ss(line_head);
            int pnts_num = 0;
            int lines_num = 0;
            ss >> pnts_num >> lines_num;
            std::cerr <<"pnts num , lines num: " << pnts_num << ", " << lines_num << std::endl;
            for(int i = 0; i < pnts_num; ++i)
            {
                std::string line;
                std::getline(fh, line);
                std::stringstream ssf(line);
                float x, y, z;
                ssf >> x >> y >> z;
                HWLinePoint3D p;
                p.pnt_[0] = x;
                p.pnt_[1] = y;
                p.pnt_[2] = z;
                pnts_.emplace_back(p);
            }
            for(int i = 0; i < lines_num; ++i)
            {
                std::string line;
                std::getline(fh, line);
                std::stringstream ssi(line);
                int idx_s, idx_e;
                ssi >> idx_s >> idx_e;
                Eigen::Vector2i pidx;
                pidx[0] = idx_s;
                pidx[1] = idx_e;
                lines_segs_.emplace_back(pidx);
            }
            fh.close();
        }
        //set base name, do next time...
    }

    void HWSceneLayout3D::SetSceneLayout3D(std::vector<std::vector<Eigen::Vector3f> >& pnts)
    {
        for(std::size_t i = 0; i < pnts.size(); ++i)
        {
            int pnum = static_cast<int>(pnts[i].size());
            if(pnum < 2)
            {
                continue;
            }
            for(std::size_t j = 0; j < pnts[i].size(); ++j)
            {
                //add to pnts_, to do next
            }
            type_ = HWSceneLayoutDimType::kScene3DType;
        }
    }
    
    HWSceneLayout3D HWSceneLayout3D::operator = (HWSceneLayout3D& other)
    {
        HWSceneLayout3D c;
        c.lines_segs_ = other.lines_segs_;
        c.pnts_ = other.pnts_;
        c.type_ = other.type_;
		c.layout2d_id_ = other.layout2d_id_;
        return c;
    }

}

