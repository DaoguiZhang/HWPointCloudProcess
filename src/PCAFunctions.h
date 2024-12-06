#ifndef _PCA_FUNCTIONS_H_
#define _PCA_FUNCTIONS_H_
#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

#include "nanoflann.hpp"
#include "utils.h"

using namespace cv;
using namespace std;
using namespace nanoflann;

enum PLANE_MODE { PLANE, SURFACE };

struct PCAInfo
{
	float lambda0, scale;
	cv::Matx31d normal, planePt;
	std::vector<int> idxAll, idxIn;

	PCAInfo &operator =(const PCAInfo &info)
	{
		this->lambda0 = info.lambda0;
		this->normal = info.normal;
		this->idxIn = info.idxIn;
		this->idxAll = info.idxAll;
		this->scale = scale;
		return *this;
	}
};

class PCAFunctions
{
public:
	PCAFunctions(void) {};
	~PCAFunctions(void) {};

	void PCA(PtCloud<float> &cloud, int k, std::vector<PCAInfo> &pcaInfos);

	void RDPCA(PtCloud<float> &cloud, int k, std::vector<PCAInfo> &pcaInfos);

	void PCASingle(std::vector<std::vector<float> > &pointData, PCAInfo &pcaInfo);

	void RDPCASingle(std::vector<std::vector<float> > &pointData, PCAInfo &pcaInfo);

	void MCMD_OutlierRemoval(std::vector<std::vector<float> > &pointData, PCAInfo &pcaInfo);

	float meadian(std::vector<float> dataset);
};

#endif //_PCA_FUNCTIONS_H_
