#ifndef HWSFM_BA_TYPES_HEADER
#define HWSFM_BA_TYPES_HEADER

#include <algorithm>

namespace HWSFM
{
	namespace BA
	{
		/** Camera representation for bundle adjustment. */
		struct Camera
		{
			Camera(void);

			double focal_length = 0.0;
			double distortion[2];
			double translation[3];
			double rotation[9];
			bool is_constant = false;
		};

		/** Camera representation for bundle adjustment. */
		struct CameraExtrPnt
		{
			//w0, w1, w2, t0, t1, t2, p0, p1, p2
			CameraExtrPnt(void)
			{
				std::fill(this->cam_pnt_params_, this->cam_pnt_params_ + 9, 0.0);
			}
			double cam_pnt_params_[9];
			//bool is_constant = false;
		};

		/** 3D point representation for bundle adjustment. */
		struct Point3D
		{
			double pos[3];
			bool is_constant = false;
		};

		/** line 3D representation for bundle adjustment. */
		struct PluckerLinePoint3D
		{
			//should meet [3]*[3]+[4]*[4]+[5]*[5] = 1
			//[0]*[3]+[1]*[4]+[2]*[5] = 0
			double plucker_line[6];
			bool is_constant = false;
		};

		/** Observation of a 3D point for a camera. */
		struct Observation
		{
			double pos[2];
			int camera_id;
			int point_id;
		};

		struct LineObservation
		{
			double ls[3];
			double le[3];
			int camera_id;
			int line3d_id;	//point to PluckerLinePoint3D
		};

		/* ------------------------ Implementation ------------------------ */

		inline
			Camera::Camera(void)
		{
			std::fill(this->distortion, this->distortion + 2, 0.0);
			std::fill(this->translation, this->translation + 3, 0.0);
			std::fill(this->rotation, this->rotation + 9, 0.0);
		}
	}
}

#endif /* SFM_BA_TYPES_HEADER */

