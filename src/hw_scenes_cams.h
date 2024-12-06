#ifndef HW_SCENES_CAMS_H
#define HW_SCENES_CAMS_H

//#include"base/camera.h"
#include"hw_cmns.h"
//#include"hw_colmap_convert.h"
#include"model_cameras.h"
//#include"util/bitmap.h"
#include"hw_image.h"
#include<map>
#include<opencv2/opencv.hpp>

/*
cams and image store
*/

namespace HW
{
    typedef struct HWCamsIdxGroup
    {
	    std::vector<int> cams_idxs_;
    } HWCamsIdxGroup;

    class HWScenesCams
    {
    public:

		const float cam_pos_to_cam_pos_threshold = 3.0;	//in three meters 
		const float lines_descriptor_matches_dist_threshold = 25.0;	//in 

		//typedef unsigned int HWImageIDType;
        HWScenesCams();
        ~HWScenesCams();

        void SetImagesDir(const std::string& imgs_dir);
        void SetCamsDir(const std::string& cams_dir);
        void SetImagesOutDir(const std::string& images_dir);
        void SetCamsOutDir(const std::string& cams_dir);
        void LoadElements();
		void LoadElementsOld();

		//inferece from set data
		void SetCamerasModels(std::vector<CameraModel>& cams_models);
		void SetHWImageModelsFromPaths(std::vector<std::string>& images_paths);

        void SetLayoutIdFromImageIdx(std::size_t imgidx, unsigned int lid);
		
		void SetLayoutIdByImageIdx(unsigned int imgidx, unsigned int lyid);
		void SetLayoutIdByImageId(unsigned int imgid, unsigned int lyid);
		void SetLayoutIdByCamIdx(unsigned int camidx, unsigned int lyid);
		void SetLayoutIdByCamId(unsigned int camid, unsigned int lyid);

		//
		void UpdateLinesSegsDetectedFromImgid(int imgid);
		//
		void UpdateSiftImageFromImgid(int imgid);

		//
		void UpdateImageLoadedFromImgid(int imgid);

		//Set keypoint 
		void SetImageSiftKeyPointsFromImgid(int imgid, const std::vector<cv::KeyPoint>& key_positions);

		//Set image network points
		void SetImageNetworkPointsFromImgid(int imgid, const std::vector<cv::Point2f>& img_pnts);

		//Write image network into image by imgid
		void WriteImageNetworkPntsIntoImageByImgid(int imgid, const std::string& path);

		//Get ImageNetwork loaded state
		bool GetImageNetworkPointsLoadedFromImgid(int imgid);

        //get cam id from image id(cam id is cams_models_ id)
        unsigned int GetCamIdFromImageId(int imgid);

		//get layout id from image id
		unsigned int GetLayoutIdFromCamId(int camid);

        //get image idx from cam id()
        unsigned int GetImageidFromCamId(int camid);

		//get cam idx from cam id
		unsigned int GetCamidxFromCamId(int camid);

        //aggregate cams id
        void GroupCamsIdBasedOnCamDirect();
		void GroupCamsIdBasedOnLSDLinesParis();	//based on lines pairs

		//load all images from image paths
		void UpdateImagesModelLoadImgs();

		void MatchLinesBasedOnLinesFeatureLbd(int src_id, int tgt_id, 
			std::vector<cv::DMatch>& lines2lines);

		void MatchLinesBasedOnLinesFeatureLsd(int src_id, int tgt_id,
			std::vector<cv::DMatch>& lines2lines);

		void GroupCamsIdBasedOnLSDLinesParisAndCamParams();
        //void Convert2ColCams(std::vector<colmap::Camera>& ccams);
        std::size_t GetCamerasNum();
		const std::size_t GetCamerasNum() const;
        std::size_t GetImagesNum();
		const std::size_t GetImagesNum() const;

        const std::vector<CameraModel>& GetCamerasModels();
		const std::vector<CameraModel>& GetCamerasModels() const;
        const std::vector<HWImage>& GetImagesModels();
		const std::vector<HWImage>& GetImagesModels() const;
        //
        const CameraModel& GetCameraModelFromCamId(int cam_id);
		const CameraModel& GetCameraModelFromCamId(int cam_id) const;
        const HWImage& GetImageFromImgId(int img_id);
		const HWImage& GetImageFromImgId(int img_id) const;

        const std::vector<std::string>& GetCamerasPaths();
        const std::vector<std::string>& GetImagesPaths();

        const std::map<unsigned int, HWCamsIdxGroup>& GetCamsGroups();
        
        void SaveSelectedCamsPoseIntoObj(const std::string& path,  
            const std::vector<CameraModel>& cams, float aixlen);
        void SaveAllCamsPoseIntoObj(const std::string& path, float aixlen);
		void UpdateCamsModelsId2ImagesModelsId();

    private:
        	
        //std::string GetBaseNameFromPath(const std::string& path);
        bool CompareBaseNameFromTwoPaths(const std::string& spath, const std::string& tpath);
        
        int CompareBaseNameVec(const std::string& str, const std::vector<std::string>& strs_vec);
        
		int GetImageIdxFromImgId(int imgid);
		static bool CompareHWStrPairByIdxCmp(const std::pair<int, std::string>& a, const std::pair<int, std::string>& b);
        //paths dir for image and cameras
        std::string image_dir_;
	    std::string cams_dir_;
        std::string images_out_dir_;
        std::string cams_out_dir_;

	    //cams paths data
	    std::vector<std::string> cams_path_;
	    std::vector<std::string> images_files_path_;
	
	    //cams datas loaded from cams paths: note: first image idx, second HWCameraModel
	    std::vector<CameraModel> cams_models_; //first: image_idx(images vec index), second hwcam

        //std::map<int, HWCameraModel> cams_models_;
        std::vector<HWImage> images_vec_;

	    //std::map<unsigned int, colmap::Bitmap> images_vec_;
        std::map<unsigned int, unsigned int> imagesToCamsId_;   //image to cams

	    //match point from feature numbers
	    int match_pnts_num_;
	    float group_threshold_;
	
	    //group all cams, first camid, second camid vector
	    std::map<unsigned int, HWCamsIdxGroup> cams_groups_;
    };
}

#endif