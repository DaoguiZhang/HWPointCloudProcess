#ifndef HW_SCENE_LAYOUT3D_H
#define HW_SCENE_LAYOUT3D_H
#include"hw_cmns.h"
#include"model_cameras.h"
#include"hw_scenes_cams.h"

namespace HW
{
    //class HWScenesCams;
    class HWSceneLayout3D
    {
    public:
        HWSceneLayout3D();
        ~HWSceneLayout3D();
        void ReadLayout3D(const std::string& path);
        //single polygon layout
        void SetSceneLayout3D(std::vector<std::vector<Eigen::Vector3f> >& pnts);
        HWSceneLayout3D operator = (HWSceneLayout3D& other);
        
		int layout2d_id_;

    private:
        
        HWSceneLayoutDimType type_;
        std::vector<HWLinePoint3D> pnts_;
        std::vector<Eigen::Vector2i> lines_segs_;   //s idx and e idx
    };
}

#endif