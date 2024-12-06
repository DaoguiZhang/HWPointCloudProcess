#define GLEW_STATIC
#include <gl/glew.h>
#include <gl/glut.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/LU> 

#include <iomanip>
#include <iostream>
#include <fstream>
#include <io.h>
#include <direct.h>
#include <unordered_set>
//#include<hw_cmns.h>
#include<model_cameras.h>

#include "technique.h"
#include "mesh.h"

struct CameraIntrinsic
{
    int width = 640;
    int height = 480;
    float fx = 640 * 0.5;
    float fy = 640 * 0.5;
    float cx = width / 2;
    float cy = height / 2;
    //float fovy = 90.0f / 180.0f * 3.1415926f;
    //float fy = height / 2.0f / tan(fovy / 2.0f);
    //float fx = fy;
} camIntr;

#if 0
int width = camIntr.height, height = camIntr.width;
//float fx = 640 * 0.8, fy = 640 * 0.8, cx = width / 2, cy = height / 2;
float fx = camIntr.fx, fy = camIntr.fy, cx = width / 2, cy = height / 2;
#else
int width = 1148, height = 862;
float fx = 1148 / 2, fy = 1148 / 2, cx = width / 2, cy = height / 2;
//int width = 640, height = 480;
//float fx = 640 / 2, fy = 640 / 2, cx = width / 2, cy = height / 2;
#endif

//std::string workspace = "F:\\dataset\\CG\\E1_Hall";
//std::string workspace = "F:\\dataset\\CG\\BYS\\F1\\2019-12-17_14.30.00";
//std::string workspace = "F:\\dataset\\CG\\BYS\\F2\\2019-12-17_15.14.44";
//std::string workspace = "F:\\dataset\\CG\\BYS\\F2\\2019-12-17_14.56.10";
std::string workspace = "D:\\vc_project_new\\huawei_data_indoor\\room\\E1_obj";

std::string objFolder = "obj-all";
//std::string objFolder = "obj_quality2";
//std::string imgFolderSuffix = "flitered1000_newtool_poly_add";
std::string imgFolderSuffix = "poly_image";

//std::string poseRcwFile = workspace + "\\pointclouds\\E1\\imagesPoseRwc_filtered_newTool.txt";
std::string cam_params_dir = workspace + "\\scene";
std::string camposeUsedRwcFile = workspace + "\\cams_output\\imagesPoseRwc_poly_out.txt";

std::string poseRcwFile = workspace + "\\pointclouds\\filtered_for_camera_add\\cameraPoseRcw_NewPoseGen.txt";
std::string poseUsedRwcFile = workspace + "\\pointclouds\\filtered5000\\imagesPoseRwc_flitered1000_newtool_poly_add.txt";

Mesh *pMesh;
Technique *pShaderMgr;

GLuint textureColor;
GLuint textureDepth;
GLuint textureNormal;
glm::mat4 modelView;
glm::mat4 projection;
cv::Mat colorImg(height, width, CV_8UC3);
cv::Mat depthImg(height, width, CV_32FC1);
cv::Mat normalImg(height, width, CV_8UC3);

void RenderSceneNavVis(int ind, int cam_id);

void readCameraPose(const std::string poseFile, std::vector<Eigen::Matrix4f>& poses);

//added by zdg
void readCamPoseFromUAV(const std::string& filepath, Eigen::Matrix4f& pose);
void GetFilesFromCurrentDir(const std::string & path, std::vector<std::string> & files);
void readCamPoseAndIntrinsinc(const std::string& file_dir, std::vector<CameraModel>& cams_params,
	std::vector<Eigen::Vector2i>& images_width_heights);
//end by zdg

void readCameraPoseRwc(const std::string poseFile, std::vector<Eigen::Matrix4f>& poses);

void readGroundTruth(const std::string poseFile, std::vector<Eigen::Matrix4f>& poses);

void writeCameraPose(const std::string file, std::vector<Eigen::Matrix4f>& poses);

bool poseRcw2poseRwc(const Eigen::Matrix4f& poseRcw, Eigen::Matrix4f& poseRwc);

void generateImages(const std::vector<Eigen::Matrix4f>& posesRcwAll,
    std::vector<Eigen::Matrix4f>& posesRwcUsed);

void generateImagesFromCamModels(const std::vector<CameraModel>& cams_params, 
	const std::vector<Eigen::Vector2i>& images_width_heights, std::vector<Eigen::Matrix4f>& posesRwcUsed);

bool RenderScene(int idx);

void InpaintImage(cv::Mat& imageIn, cv::Mat& imageOut);

void RegisteredImgs(cv::Mat& colorDepthImg, cv::Mat& colorImg, cv::Mat& depthImg)
{
    colorDepthImg.create(height, width, CV_8UC3);

    cv::Mat grayImg(height, width, CV_8UC1);
    cv::Mat grayDepthImg(height, width, CV_8UC1);
    depthImg.convertTo(grayDepthImg, CV_8UC1, 1.0f, 0);
    cv::cvtColor(colorImg, grayImg, CV_RGB2GRAY);
    for (int r = 0; r < colorDepthImg.rows; ++r)
    {
        for (int c = 0; c < colorDepthImg.cols; ++c)
        {
            cv::Vec3b& pixel = colorDepthImg.at<cv::Vec3b>(r, c);
            pixel[0] = 20;
            pixel[1] = grayImg.at<uchar>(r, c);
            pixel[2] = grayDepthImg.at<uchar>(r, c);
        }
    }
}

const static GLuint attachment_buffers[] = {
    GL_COLOR_ATTACHMENT0_EXT,
    GL_COLOR_ATTACHMENT1_EXT,
    GL_COLOR_ATTACHMENT2_EXT,
    GL_COLOR_ATTACHMENT3_EXT,
    GL_COLOR_ATTACHMENT4_EXT,
    GL_COLOR_ATTACHMENT5_EXT,
    GL_COLOR_ATTACHMENT6_EXT,
    GL_COLOR_ATTACHMENT7_EXT
};

int main(int argc, char **argv)
{
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB | GLUT_DEPTH);
    //glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_GLUTMAINLOOP_RETURNS);

    glutInitWindowSize(width, height);
    glutCreateWindow("DepthTextureGenerator");
    GLenum res = glewInit();
    if (res != GLEW_OK) {
        std::cout << "error" << std::endl;
        return false;
    }

    pShaderMgr = new Technique();

    if (!pShaderMgr->Init()) {
        std::cout << "Error initializing the lighting technique" << std::endl;
        return false;
    }

    pMesh = new Mesh();

    std::string meshFileName = workspace + "\\" + objFolder + "\\textured.obj";
    printf("loading mesh...    %s\n", meshFileName.c_str());
    pMesh->LoadMesh(meshFileName, false);

    glGenTextures(1, &textureColor);
    glBindTexture(GL_TEXTURE_2D, textureColor);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glGenTextures(1, &textureDepth);
    glBindTexture(GL_TEXTURE_2D, textureDepth);
	//glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE16UI_EXT, width, height, 0, GL_LUMINANCE_INTEGER_EXT, GL_UNSIGNED_SHORT, NULL);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE32UI_EXT, width, height, 0, GL_LUMINANCE_INTEGER_EXT, GL_FLOAT, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glGenTextures(1, &textureNormal);
    glBindTexture(GL_TEXTURE_2D, textureNormal);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    GLuint fbo, rbo;
    glGenFramebuffersEXT(1, &fbo);
    glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo);
    glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_2D, textureColor, 0);
    glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT1_EXT, GL_TEXTURE_2D, textureDepth, 0);
    glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT2_EXT, GL_TEXTURE_2D, textureNormal, 0);

    glGenRenderbuffersEXT(1, &rbo);
    glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, rbo);
    //glRenderbufferStorageEXT(GL_RENDERBUFFER_EXT, GL_DEPTH_COMPONENT24, width, height);
	glRenderbufferStorageEXT(GL_RENDERBUFFER_EXT, GL_DEPTH_COMPONENT32, width, height);
    glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, 0);

    glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, rbo);
    glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, GL_RENDERBUFFER_EXT, rbo);
    glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, 0);

    pShaderMgr->Enable();

    // 单向渲染
    //glFrontFace(GL_CCW);
    //glCullFace(GL_BACK);
    //glEnable(GL_CULL_FACE);

    // 1:使用生成相机位姿; 0:使用NavVis位姿
#if 0
    std::vector<Eigen::Matrix4f> posesRcw;
#if  0
	readCameraPose(poseRcwFile, posesRcw);
    //readCameraPoseRwc(poseRcwFile, posesRcw);
    //readGroundTruth(poseRcwFile, posesRcw);
	std::vector<Eigen::Matrix4f> posesRwcUsed;
	//set fx, fy, cx, cy, width, height
	generateImages(posesRcw, posesRwcUsed);
	writeCameraPose(poseUsedRwcFile, posesRwcUsed);
	printf("write camera pose to %s.\n", poseUsedRwcFile.c_str());

#else
	std::vector<CameraModel> cams_params;
	std::vector<Eigen::Vector2i> images_width_heights;
	readCamPoseAndIntrinsinc(cam_params_dir, cams_params, images_width_heights);
	std::vector<Eigen::Matrix4f> posesRwcUsed;
	generateImagesFromCamModels(cams_params, images_width_heights, posesRwcUsed);
	writeCameraPose(camposeUsedRwcFile, posesRwcUsed);
	printf("write camera pose to %s.\n", camposeUsedRwcFile.c_str());

	/*std::string uav_path = "D:\\vc_project_new\\Dongfangwanguo_UAV_LiDAR_Pano\\UAV\\scene\\DJI_0159.CAM";
	std::cout << "the uav path is: " << uav_path << std::endl;
	Eigen::Matrix4f tmp_rcw;
	readCamPoseFromUAV(uav_path, tmp_rcw);
	std::cout << "tmp_rcw: " << tmp_rcw << std::endl;
	posesRcw.emplace_back(tmp_rcw);
	std::vector<Eigen::Matrix4f> posesRwcUsed;
	generateImages(posesRcw, posesRwcUsed);
	std::string tmp_poseUsedRwcFile = "D:\\vc_projects\\LearnOpengl\\LearnOpengl\\resources\\cams.txt";
	writeCameraPose(tmp_poseUsedRwcFile, posesRwcUsed);
	printf("write camera pose to %s.\n", tmp_poseUsedRwcFile.c_str());*/

#endif
#else
    // 使用NavVis相机位姿
    for (int i = 15; i < 281; i++) {
        for (int cam_id = 0; cam_id < 6; cam_id++) {
            std::ifstream fs;
            char path[256];
            //sprintf(path, "F:\\dataset\\CG\\2019-04-28_14.26.45_garage\\scene\\%05d-cam%d.CAM", i,cam_id);
            sprintf(path, "%s\\scene\\%05d-cam%d.CAM", workspace.c_str(), i, cam_id);
            printf("open pose %s\n", path);
            fs.open(path);
            modelView = glm::mat4(1.0f);
            for (int j = 0; j < 3; j++) {
                fs >> modelView[3][j];// 
            }

            for (int k = 0; k < 3; k++) {
                for (int j = 0; j < 3; j++) {
                    fs >> modelView[j][k];// 
                }
            }

			std::cout << "cam: \n" << modelView[0][0] << " " << modelView[0][1] << " " << modelView[0][2] << " " << modelView[0][3] << std::endl
				<< modelView[1][0] << " " << modelView[1][1] << " " << modelView[1][2] << " " << modelView[1][3] << std::endl
				<< modelView[2][0] << " " << modelView[2][1] << " " << modelView[2][2] <<" "<< modelView[2][3] << std::endl
				<< modelView[3][0] << " " << modelView[3][1] << " " << modelView[3][2] << " " << modelView[3][3] << std::endl;

            //// 图像旋转，XY坐标调换，X取负——>navvis鱼眼图旋转90度。（注意：Tcw也变换，Twc不用）
            //for (int i = 0; i < 4; i++) {
            //    float temp = modelView[i][0];
            //    modelView[i][0] = -modelView[i][1];
            //    modelView[i][1] = temp;
            //}

            modelView[0][1] *= -1;
            modelView[1][1] *= -1;
            modelView[2][1] *= -1;
            modelView[3][1] *= -1;
            modelView[3][2] *= -1;
            modelView[0][2] *= -1;
            modelView[1][2] *= -1;
            modelView[2][2] *= -1;

            //modelView = glm::mat4(1.0f);
            //modelView = glm::translate(modelView, glm::vec3(0.0f, 0.0f, -3.0f));
            ////modelView = glm::rotate(modelView, glm::radians(-25.0f), glm::vec3(1.0f, 0.0f, 0.0f));
            ////modelView = glm::rotate(modelView, glm::radians(180.0f), glm::vec3(1.0f, 0.0f, 0.0f));

            //modelView = glm::rotate(modelView, glm::radians(1.0f* i * 10), glm::vec3(0.0f, 1.0f, 0.0f));
            //modelView[3][2] *= -1;
            /*glm::vec3 rot = glm::vec3(modelView[0][0], modelView[0][1], modelView[0][2]);
            modelView = glm::rotate(modelView, glm::radians(180.0f),rot);*/
            fs.close();
            float far = 100.0f;
            float near = 0.1f;
            /*float L = -(cx)* near / fx;
            float T = -(cy)* near / fy;
            float R = (width - cx) * near / fx;
            float B = (height - cy) * near / fy;
            projection[0][0] = 2 * near / (R - L);
            projection[0][1] = 0.0f;
            projection[0][2] = 0.0f;
            projection[0][3] = 0.0f;

            projection[1][0] = 0.0f;
            projection[1][1] = 2 * near / (T - B);;
            projection[1][2] = 0.0f;
            projection[1][3] = 0.0f;

            projection[2][0] = -(R + L) / (R - L);
            projection[2][1] = -(T + B) / (T - B);
            projection[2][2] = (far + near) / (far - near);
            projection[2][3] = 1.0f;

            projection[3][0] = 0.0f;
            projection[3][1] = 0.0f;
            projection[3][2] = -2.0f * far * near / (far - near);
            projection[3][3] = 0.0f;*/
            projection[0][0] = 2 * fx / width;
            projection[0][1] = 0.0f;
            projection[0][2] = 0.0f;
            projection[0][3] = 0.0f;

            projection[1][0] = 0.0f;
            projection[1][1] = 2 * fy / height;
            projection[1][2] = 0.0f;
            projection[1][3] = 0.0f;

            projection[2][0] = (1.0f - 2 * cx / width);
            projection[2][1] = 2 * cy / height - 1.0f;
            projection[2][2] = -(far + near) / (far - near);
            projection[2][3] = -1.0f;

            projection[3][0] = 0.0f;
            projection[3][1] = 0.0f;
            projection[3][2] = -2.0f * far * near / (far - near);
            projection[3][3] = 0.0f;

            RenderSceneNavVis(i, cam_id);
            //cv::Mat colorDepthImg, colorImgRead(height, width, CV_8UC3);
            //char colorPath[256];
            //sprintf(colorPath, "C:\\Users\\xht\\Desktop\\upload_TexOutput\\image\\rec_%d.jpg", i);
            //colorImgRead = cv::imread(colorPath, cv::IMREAD_COLOR);
            //RegisteredImgs(colorDepthImg, colorImgRead, depthImg);
            //cv::imshow("colorDepthImg", colorDepthImg);
            //cv::waitKey(0);
        }
    }
#endif

}

bool RenderScene(int idx)
{
    glDrawBuffers(3, attachment_buffers);
    glPushAttrib(GL_VIEWPORT_BIT);

    glViewport(0, 0, width, height);
    //glViewport(cx - width / 2.0, cy - height / 2.0, width, height);

    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
    glClearDepth(1.0f);

    glEnable(GL_NORMALIZE);
    //glDisable(GL_CULL_FACE);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    //glDepthFunc(GL_GEQUAL);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    pShaderMgr->SetModelView(modelView);
    pShaderMgr->SetProjection(projection);

    pMesh->Render();

    glPopAttrib();
    //glDrawBuffers(1, attachment_buffers);
    glFinish();
    //glutSwapBuffers();

    glBindTexture(GL_TEXTURE_2D, textureColor);
    glGetTexImage(GL_TEXTURE_2D, 0, GL_RGB, GL_UNSIGNED_BYTE, colorImg.data);
    glBindTexture(GL_TEXTURE_2D, 0);

    glBindTexture(GL_TEXTURE_2D, textureDepth);
    //glGetTexImage(GL_TEXTURE_2D, 0, GL_LUMINANCE_INTEGER_EXT, GL_UNSIGNED_SHORT, depthImg.data);
	glGetTexImage(GL_TEXTURE_2D, 0, GL_LUMINANCE_INTEGER_EXT, GL_FLOAT, depthImg.data);
    glBindTexture(GL_TEXTURE_2D, 0);

    glBindTexture(GL_TEXTURE_2D, textureNormal);
    glGetTexImage(GL_TEXTURE_2D, 0, GL_RGB, GL_UNSIGNED_BYTE, normalImg.data);
    glBindTexture(GL_TEXTURE_2D, 0);

    std::vector<int> pngCompressionParams;
    pngCompressionParams.push_back(CV_IMWRITE_PNG_COMPRESSION);
    pngCompressionParams.push_back(0);

    //cv::imshow("colorImg", colorImg);

    // 渲染出图需要Y轴flip，坐标系不一致
    cv::flip(colorImg, colorImg, 0);
    cv::flip(depthImg, depthImg, 0);
    cv::flip(normalImg, normalImg, 0);

    cv::imshow("flip", colorImg);
	//cv::waitKey(0);

    //// 调试用，实际只需flip，不可transpose，需在pose上变换
    //cv::Mat colorImgSaved(width, height, CV_8UC3);
    //cv::transpose(colorImg, colorImgSaved);
    //cv::Mat depthImgSaved(width, height, CV_16UC1);
    //cv::transpose(depthImg, depthImgSaved);
    //cv::imshow("colorImgSaved", colorImgSaved);

    //cv::waitKey();

    // 彩色图筛选
#if 1
    int zeroPixels = 0;
    int thresholdColor = width * height * 0.1;                // 最大空洞比例10%
    for (int i = 0; i < colorImg.rows; i++) {
        for (int j = 0; j < colorImg.cols; j++) {
            const cv::Vec3b& rgb = colorImg.at<cv::Vec3b>(i, j);
            if (rgb[0] == 0 && rgb[1] == 0 && rgb[2] == 0)    // 空洞区域
                zeroPixels++;
            if (zeroPixels > thresholdColor)
                return false;
        }
    }
#endif

    // 深度图筛选
#if 1
    int filteredDepthLow = 0;
    int filteredDepthMid = 0;
    int filteredDepthHigh = 0;
    int thresholdDepthLow = width * height * 0.3;             // 深度值小于阈值比例30%
    int thresholdDepthMid = width * height * 0.6;             // 深度值小于阈值比例60%
    int thresholdDepthHigh = width * height * 0.9;            // 深度值小于阈值比例90%
    for (int i = 0; i < depthImg.rows; i++) {
        for (int j = 0; j < depthImg.cols; j++) {
            const ushort& depth = depthImg.at<ushort>(i, j);
            if (depth != 0 && depth < 1000)                   // 深度值<1m，判断距离过近
                filteredDepthLow++;
            else if (depth != 0 && depth < 3000)
                filteredDepthMid++;
            else if (depth != 0 && depth < 5000)
                filteredDepthHigh++;
            if (filteredDepthLow > thresholdDepthLow
                || filteredDepthMid > thresholdDepthMid
                || filteredDepthHigh > thresholdDepthHigh)
                return false;
        }
    }
#endif

    std::string colorImgDir = workspace + "\\rendering\\color_" + imgFolderSuffix + "\\";
    std::string depthImgDir = workspace + "\\rendering\\depth_" + imgFolderSuffix + "\\";
    if (_access(colorImgDir.c_str(), 0) == -1) {
        mkdir(colorImgDir.c_str());
    }
    if (_access(depthImgDir.c_str(), 0) == -1) {
        mkdir(depthImgDir.c_str());
    }

    std::stringstream colorImgName;
    std::stringstream depthImgName;
    colorImgName << colorImgDir << std::setw(9) << std::setfill('0') << idx << ".jpg";
    depthImgName << depthImgDir << std::setw(9) << std::setfill('0') << idx << ".png";
    cv::imwrite(colorImgName.str(), colorImg);
    cv::imwrite(depthImgName.str(), depthImg, pngCompressionParams);

    // 验证图像&pose用
#if 0
    std::string dolorImgDirFlip = workspace + "\\rendering\\color_Flip_" + imgFolderSuffix + "\\";
    std::string depthImgDirFlip = workspace + "\\rendering\\depth_Flip_" + imgFolderSuffix + "\\";
    if (_access(dolorImgDirFlip.c_str(), 0) == -1) {
        mkdir(dolorImgDirFlip.c_str());
    }
    if (_access(depthImgDirFlip.c_str(), 0) == -1) {
        mkdir(depthImgDirFlip.c_str());
    }

    std::stringstream dolorImgNameFlip;
    std::stringstream depthImgNameFlip;
    dolorImgNameFlip << dolorImgDirFlip << std::setw(9) << std::setfill('0') << idx << ".jpg";
    depthImgNameFlip << depthImgDirFlip << std::setw(9) << std::setfill('0') << idx << ".png";
    cv::imwrite(dolorImgNameFlip.str(), colorImg);
    cv::imwrite(depthImgNameFlip.str(), depthImg, pngCompressionParams);
#endif

    // 图像修复
#if 0
    cv::Mat colorImgInpainted = cv::Mat(colorImgSaved.size(), CV_8UC3);

    InpaintImage(colorImgSaved, colorImgInpainted);
    std::string colorImgInpaintedDir = workspace + "\\rendering\\colorInpainted_" + imgFolderSuffix + "\\";
    if (_access(colorImgInpaintedDir.c_str(), 0) == -1) {
        mkdir(colorImgInpaintedDir.c_str());
    }
    std::stringstream colorImgInpaintedName;
    colorImgInpaintedName << colorImgInpaintedDir << std::setw(9) << std::setfill('0') << idx << ".jpg";
    cv::imwrite(colorImgInpaintedName.str(), colorImgInpainted, pngCompressionParams);
#endif

    return true;
}

void RenderSceneNavVis(int ind, int cam_id)
{
    glDrawBuffers(3, attachment_buffers);
    glPushAttrib(GL_VIEWPORT_BIT);
#if 1
    glViewport(0, 0, width, height);
#endif
#if 0
    glViewport(cx - width / 2.0, cy - height / 2.0, width, height);
#endif

    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
    glClearDepth(1.0f);


    glEnable(GL_NORMALIZE);
    //glDisable(GL_CULL_FACE);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    //glDepthFunc(GL_GEQUAL);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    pShaderMgr->SetModelView(modelView);
    pShaderMgr->SetProjection(projection);

    pMesh->Render();

    glPopAttrib();
    //glDrawBuffers(1, attachment_buffers);
    glFinish();
    //glutSwapBuffers();

    glBindTexture(GL_TEXTURE_2D, textureColor);
    glGetTexImage(GL_TEXTURE_2D, 0, GL_RGB, GL_UNSIGNED_BYTE, colorImg.data);
    glBindTexture(GL_TEXTURE_2D, 0);

    glBindTexture(GL_TEXTURE_2D, textureDepth);
    //glGetTexImage(GL_TEXTURE_2D, 0, GL_LUMINANCE_INTEGER_EXT, GL_UNSIGNED_SHORT, depthImg.data);
	glGetTexImage(GL_TEXTURE_2D, 0, GL_LUMINANCE_INTEGER_EXT, GL_FLOAT, depthImg.data);
    glBindTexture(GL_TEXTURE_2D, 0);

    glBindTexture(GL_TEXTURE_2D, textureNormal);
    glGetTexImage(GL_TEXTURE_2D, 0, GL_RGB, GL_UNSIGNED_BYTE, normalImg.data);
    glBindTexture(GL_TEXTURE_2D, 0);

    // 1:输出pose对应图像；0:输出正常角度图像
#if 1

    //cv::imshow("colorImg", colorImg);
    // 渲染出图需要Y轴flip，坐标系不一致
    cv::flip(colorImg, colorImg, 0);
    cv::flip(depthImg, depthImg, 0);
    cv::flip(normalImg, normalImg, 0);
    cv::imshow("flip", colorImg);
#else
    // 输出正常角度图像/normal
    // 调试用，实际只需flip，不可transpose，需在pose上变换
    cv::Mat colorImgSaved(width, height, CV_8UC3);
    cv::transpose(colorImg, colorImgSaved);
    /*cv::Mat depthImgSaved(width, height, CV_16UC1);
    cv::transpose(depthImg, depthImgSaved);*/

	cv::Mat depthImgSaved(width, height, CV_32FC1);
	cv::transpose(depthImg, depthImgSaved);

    cv::Mat normalImgSaved(width, height, CV_8UC3);
    cv::transpose(normalImg, normalImgSaved);
#endif

	cv::imshow("flip", colorImg);
	//cv::waitKey(0);
    // 打印normal值（左上、左下、中间三个区域）
#if 0
    for (int i = 0; i < 10; i++) {
        for (int j = 0; j < 10; j++) {
            const cv::Vec3b& bgr = normalImgSaved.at<cv::Vec3b>(i, j);
            printf("RGB(%d, %d) = (%d, %d, %d)\n", i, j, bgr[0], bgr[1], bgr[2]);
            printf("nor(%d, %d) = (%f, %f, %f)\n", i, j, float(bgr[0]) / 255.0 * 2.0 - 1.0, float(bgr[1]) / 255.0 * 2.0 - 1.0, float(bgr[2]) / 255.0 * 2.0 - 1.0);
        }
    }
    //绘制矩形
    cv::Rect rectLeftTop(0, 0, 10, 10);
    cv::rectangle(normalImgSaved, rectLeftTop, cv::Scalar(255, 255, 255));
    for (int i = normalImgSaved.rows - 10; i < normalImgSaved.rows; i++) {
        for (int j = 0; j < 10; j++) {
            const cv::Vec3b& bgr = normalImgSaved.at<cv::Vec3b>(i, j);
            printf("RGB(%d, %d) = (%d, %d, %d)\n", i, j, bgr[0], bgr[1], bgr[2]);
            printf("nor(%d, %d) = (%f, %f, %f)\n", i, j, float(bgr[0]) / 255.0 * 2.0 - 1.0, float(bgr[1]) / 255.0 * 2.0 - 1.0, float(bgr[2]) / 255.0 * 2.0 - 1.0);
        }
    }
    cv::Rect rectLeftBottom(0, normalImgSaved.rows - 10, 10, 10);
    cv::rectangle(normalImgSaved, rectLeftBottom, cv::Scalar(255, 255, 255));
    for (int i = normalImgSaved.rows / 2 - 5; i < normalImgSaved.rows / 2 + 5; i++) {
        for (int j = normalImgSaved.cols / 2 - 5; j < normalImgSaved.cols / 2 + 5; j++) {
            const cv::Vec3b& bgr = normalImgSaved.at<cv::Vec3b>(i, j);
            printf("RGB(%d, %d) = (%d, %d, %d)\n", i, j, bgr[0], bgr[1], bgr[2]);
            printf("nor(%d, %d) = (%f, %f, %f)\n", i, j, float(bgr[0]) / 255.0 * 2.0 - 1.0, float(bgr[1]) / 255.0 * 2.0 - 1.0, float(bgr[2]) / 255.0 * 2.0 - 1.0);
        }
    }
    cv::Rect rectCentrl(normalImgSaved.cols / 2 - 5, normalImgSaved.rows / 2 - 5, 10, 10);
    cv::rectangle(normalImgSaved, rectCentrl, cv::Scalar(255, 255, 255));
    cv::imshow("transpose", normalImgSaved);
    cv::waitKey(1);
#endif

    // filter images
#if 0
    {
        int zeroPixels = 0;
        int thresholdColor = width * height * 0.1;
        for (int i = 0; i < colorImgSaved.rows; i++) {
            for (int j = 0; j < colorImgSaved.cols; j++) {
                cv::Vec3b rgb = colorImgSaved.at<cv::Vec3b>(i, j);
                if (rgb[0] == 0 && rgb[1] == 0 && rgb[2] == 0)    // 空洞区域
                    zeroPixels++;
                if (zeroPixels > thresholdColor)
                    return;
            }
        }

        int filteredDepth = 0;
        int thresholdDepth = width * height * 0.3;
        for (int i = 0; i < depthImgSaved.rows; i++) {
            for (int j = 0; j < depthImgSaved.cols; j++) {
                uchar depth = depthImgSaved.at<uchar>(i, j);
                if (depth < 1 && depth != 0)                   // 深度值<0.5m，判断距离过近
                    filteredDepth++;
                if (zeroPixels > thresholdDepth)
                    return;
            }
        }
    }
#endif

    std::vector<int> pngCompressionParams;
    pngCompressionParams.push_back(CV_IMWRITE_PNG_COMPRESSION);
    pngCompressionParams.push_back(0);

    const std::string colorFolder = workspace + "\\rendering\\color_" + imgFolderSuffix;
    const std::string depthFolder = workspace + "\\rendering\\depth_" + imgFolderSuffix;
    const std::string normalFolder = workspace + "\\rendering\\normal_" + imgFolderSuffix;

    if (_access(colorFolder.c_str(), 0) == -1) {
        mkdir(colorFolder.c_str());
    }
    if (_access(depthFolder.c_str(), 0) == -1) {
        mkdir(depthFolder.c_str());
    }
    if (_access(normalFolder.c_str(), 0) == -1) {
        mkdir(normalFolder.c_str());
    }

    char colorDir[256], depthDir[256], normalDir[256];
    //sprintf(depthDir, "%s\\depth_%d_%d.png", depthFolder.c_str(), ind, cam_id);
	sprintf(depthDir, "%s\\depth_%d_%d.tiff", depthFolder.c_str(), ind, cam_id);
    
	sprintf(colorDir, "%s\\color_%d_%d.jpg", colorFolder.c_str(), ind, cam_id);
    sprintf(normalDir, "%s\\normal_%d_%d.jpg", normalFolder.c_str(), ind, cam_id);
    //sprintf(colorDir, "F:\\dataset\\CG\\E1_Hall\\rendering\\color\\color_%d_%d.png", ind, cam_id);
	cv::imwrite(colorDir, colorImg);

    //cv::imwrite(colorDir, colorImgSaved);
    //cv::imwrite(normalDir, normalImgSaved);
	//cv::imwrite(depthDir, depthImgSaved, pngCompressionParams);
    //cv::imwrite(depthDir, depthImgSaved);
    printf("save image to %s\n", colorDir);

}

void readCameraPose(const std::string fileName, std::vector<Eigen::Matrix4f>& poses)
{
    std::ifstream fs(fileName);
    if (!fs.is_open()) {
        printf("error : open file %s  filed!\n", fileName.c_str());
        return;
    }

    while (!fs.eof()) {
        Eigen::Matrix4f pose;
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                fs >> pose(i, j);
            }
        }
        poses.push_back(pose);
    }
    return;
}

void readCamPoseFromUAV(const std::string& filepath, Eigen::Matrix4f& pose)
{
	std::ifstream fs(filepath);
	std::cout << filepath << std::endl;
	//std::cout << "asdfsdfsdfas" << std::endl;
	if (!fs.is_open()) {
		printf("error : open file %s  filed!\n", filepath.c_str());
		return;
	}
	//std::cout << "111111111111111111" << std::endl;
	if (!fs.eof()) {

		Eigen::Vector3f Twc;
		fs >> Twc[0] >> Twc[1] >> Twc[2];

		Eigen::Matrix3f Rwc;
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				fs >> Rwc(i, j);
			}
		}
		
		Eigen::Matrix4f cam_extr;
		for (int i = 0; i < 3; i++) {
			cam_extr(i, 3) = Twc[i];
		}

		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				cam_extr(i, j) = Rwc(i, j);
			}
		}
		cam_extr(3, 0) = 0;
		cam_extr(3, 1) = 0;
		cam_extr(3, 2) = 0;
		cam_extr(3, 3) = 1;
		//std::cout << "the extrinsic is: " << cam_extr << std::endl;
		pose = cam_extr;
	}
	//std::cout << "pose: \n" << pose << std::endl;
}

void GetFilesFromCurrentDir(const std::string & path, std::vector<std::string> & files)
{
	//文件句柄  
	long long hFile = 0;
	//文件信息，_finddata_t需要io.h头文件  
	struct _finddata_t fileinfo;
	std::string p;
	int i = 0;
	if ((hFile = _findfirst(p.assign(path).append("\\*").c_str(), &fileinfo)) != -1)
	{
		do
		{
			if (!(fileinfo.attrib & _A_SUBDIR))
			{
				files.push_back(p.assign(path).append("/").append(fileinfo.name));
			}
		} while (_findnext(hFile, &fileinfo) == 0);
		_findclose(hFile);
	}
}


void readCamPoseAndIntrinsinc(const std::string& file_dir, 
	std::vector<CameraModel>& cams_params, std::vector<Eigen::Vector2i>& images_width_heights)
{
	std::vector<std::string> files;
	GetFilesFromCurrentDir(file_dir, files);
	if (files.empty())
		std::cout << "LoadCamsFromCAMDIR: file empty..." << std::endl;
	//
	std::vector<std::string> images_paths;
	std::vector<std::string> cams_files;
	for (int i = 0; i < files.size(); ++i)
	{
		if (files[i].find(".CAM") != std::string::npos ||
			files[i].find(".cam") != std::string::npos)
			cams_files.emplace_back(files[i]);
		if (files[i].find(".JPG") != std::string::npos
			|| files[i].find(".jpg") != std::string::npos
			|| files[i].find(".png") != std::string::npos
			|| files[i].find(".PNG") != std::string::npos)
			images_paths.emplace_back(files[i]);
	}
	std::sort(cams_files.begin(), cams_files.end());
	std::sort(images_paths.begin(), images_paths.end());
	std::cout << "cams files size is: " << cams_files.size() << std::endl;
	cams_params.clear();
	images_width_heights.clear();
	for (int i = 0; i < cams_files.size(); ++i)
	{
		CameraModel tmp_model;
		tmp_model.LoadCamFromCam(cams_files[i]);
		//std::cout << "the cam img 333: " << i <<":  " << tmp_model.cx_ << std::endl;

		//get corresponding data
		/*
		get tmp_model.image from images_paths_ i
		to do next (provide compare images_paths with cams_files[i])
		*/
		tmp_model.image_id_ = i;
		tmp_model.cam_id_ = i;

		//get the images width and height
		cv::Mat img = cv::imread(images_paths[i]);
		std::cout << "the cam img path: " << images_paths[i] << std::endl;
		Eigen::Vector2i image_width_height;
		image_width_height[0] = img.cols;
		image_width_height[1] = img.rows;
		cams_params.emplace_back(tmp_model);
		images_width_heights.emplace_back(image_width_height);
	}

}

void readGroundTruth(const std::string fileName, std::vector<Eigen::Matrix4f>& poses)
{
    std::ifstream fs(fileName);
    if (!fs.is_open()) {
        printf("error : open file %s  filed!\n", fileName.c_str());
        return;
    }

    while (!fs.eof()) {
        std::string imgName;
        fs >> imgName;
        Eigen::Vector3f Twc;
        fs >> Twc[0] >> Twc[1] >> Twc[2];
        Eigen::Matrix3f Rcw;
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                fs >> Rcw(i, j);
            }
        }

        Eigen::Vector3f Tcw = -Rcw * Twc;

        Eigen::Matrix4f pose;
        for (int i = 0; i < 3; i++) {
            pose(i, 3) = Tcw[i];
        }
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                pose(i, j) = Rcw(i, j);
            }
        }
        pose(3, 0) = 0;
        pose(3, 1) = 0;
        pose(3, 2) = 0;
        pose(3, 3) = 1;
        poses.push_back(pose);
    }
    return;
}

void readCameraPoseRwc(const std::string fileName, std::vector<Eigen::Matrix4f>& poses)
{
    std::ifstream fs(fileName);
    if (!fs.is_open()) {
        printf("error : open file %s  filed!\n", fileName.c_str());
        return;
    }

    while (!fs.eof()) {
        Eigen::Vector3f Twc;
        Eigen::Matrix3f Rwc;
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                fs >> Rwc(i, j);
            }
            fs >> Twc[i];
        }

        Eigen::Matrix3f Rcw = Rwc.inverse();
        Eigen::Vector3f Tcw = -Rcw * Twc;

        Eigen::Matrix4f pose;
        for (int i = 0; i < 3; i++) {
            pose(i, 3) = Tcw[i];
        }
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                pose(i, j) = Rcw(i, j);
            }
        }
        pose(3, 0) = 0;
        pose(3, 1) = 0;
        pose(3, 2) = 0;
        pose(3, 3) = 1;
        poses.push_back(pose);
    }
    return;
}

void writeCameraPose(const std::string fileName, std::vector<Eigen::Matrix4f>& poses)
{
    std::ofstream out(fileName);
    if (!out.is_open()) {
        printf("error : open file %s  filed!\n", fileName.c_str());
        return;
    }

    for (int i = 0; i < poses.size(); i++) {
        const Eigen::Matrix4f& pose = poses[i];
        for (int j = 0; j < 4; j++) {
            for (int k = 0; k < 4; k++) {
                out << std::setw(10) << std::setiosflags(std::ios::fixed)
                    << std::setprecision(6) << pose(j, k) << '\t';
            }
        }
        out << '\n';
    }
    out.close();
    return;
}

void generateImages(const std::vector<Eigen::Matrix4f>& posesRcwAll,
    std::vector<Eigen::Matrix4f>& posesRwcUsed)
{
    posesRwcUsed.clear();
    int idx = 0;
	//std::cout
    for (const Eigen::Matrix4f poseRcw : posesRcwAll) {
		//std::cout <<"pose rcw: " << poseRcw << std::endl;
        modelView = glm::mat4(1.0f);
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                modelView[j][i] = poseRcw(i, j);
            }
        }

        modelView[0][1] *= -1;
        modelView[1][1] *= -1;
        modelView[2][1] *= -1;
        modelView[3][1] *= -1;
        modelView[3][2] *= -1;
        modelView[0][2] *= -1;
        modelView[1][2] *= -1;
        modelView[2][2] *= -1;

        float far = 500.0f;
        float near = 0.1f;

        projection[0][0] = 2 * fx / width;
        projection[0][1] = 0.0f;
        projection[0][2] = 0.0f;
        projection[0][3] = 0.0f;

        projection[1][0] = 0.0f;
        projection[1][1] = 2 * fy / height;
        projection[1][2] = 0.0f;
        projection[1][3] = 0.0f;

        projection[2][0] = (1.0f - 2 * cx / width);
        projection[2][1] = 2 * cy / height - 1.0f;
        projection[2][2] = -(far + near) / (far - near);
        projection[2][3] = -1.0f;

        projection[3][0] = 0.0f;
        projection[3][1] = 0.0f;
        projection[3][2] = -2.0f * far * near / (far - near);
        projection[3][3] = 0.0f;

        if (RenderScene(idx)) {

			std::cout << "222222222222222222222" << std::endl;
            Eigen::Matrix4f poseRwc;
            if (!poseRcw2poseRwc(poseRcw, poseRwc)) {
                printf("error: poseRcw2poseRwc failed!\n");
            }
            posesRwcUsed.push_back(poseRwc);
            idx++;
            printf("generate %4d images.\n", idx);
        }

        //// just for debug or test
        //if (idx > 72)
        //    return;
    }
    return;
}

void generateImagesFromCamModels(const std::vector<CameraModel>& cams_params,
	const std::vector<Eigen::Vector2i>& images_width_heights, std::vector<Eigen::Matrix4f>& posesRwcUsed)
{
	posesRwcUsed.clear();
	int idx = 0;
	//std::cout
	for (const CameraModel poseModel : cams_params) {
		//std::cout <<"pose rcw: " << poseRcw << std::endl;
		Eigen::Matrix4f poseRcw = poseModel.cam_pose_.inverse();
		//Eigen::Matrix4f poseRcw = poseModel.cam_pose_;
		std::cout <<"pose rcw: " << poseRcw << std::endl;
		modelView = glm::mat4(1.0f);
		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 4; j++) {
				modelView[j][i] = poseRcw(i, j);
			}
		}

		modelView[0][1] *= -1;
		modelView[1][1] *= -1;
		modelView[2][1] *= -1;
		modelView[3][1] *= -1;
		modelView[3][2] *= -1;
		modelView[0][2] *= -1;
		modelView[1][2] *= -1;
		modelView[2][2] *= -1;

		float far = 500.0f;
		float near = 0.1f;

		fx = poseModel.fx_;
		fy = poseModel.fy_;
		cx = poseModel.cx_;
		cy = poseModel.cy_;
		width = images_width_heights[idx][0];
		height = images_width_heights[idx][1];
		std::cerr << "fx, fy, cx, cy, width, height: " <<
			fx << ", " << fy << ", " << cx << ", " << cy << ", "
			<< width << ", " << height << std::endl;

		projection[0][0] = 2 * fx / width;
		projection[0][1] = 0.0f;
		projection[0][2] = 0.0f;
		projection[0][3] = 0.0f;

		projection[1][0] = 0.0f;
		projection[1][1] = 2 * fy / height;
		projection[1][2] = 0.0f;
		projection[1][3] = 0.0f;

		projection[2][0] = (1.0f - 2 * cx / width);
		projection[2][1] = 2 * cy / height - 1.0f;
		projection[2][2] = -(far + near) / (far - near);
		projection[2][3] = -1.0f;

		projection[3][0] = 0.0f;
		projection[3][1] = 0.0f;
		projection[3][2] = -2.0f * far * near / (far - near);
		projection[3][3] = 0.0f;

		if (RenderScene(idx)) {

			std::cout << "222222222222222222222" << std::endl;
			Eigen::Matrix4f poseRwc;
			if (!poseRcw2poseRwc(poseRcw, poseRwc)) {
				printf("error: poseRcw2poseRwc failed!\n");
			}
			posesRwcUsed.push_back(poseRwc);
			idx++;
			printf("generate %4d images.\n", idx);
		}

		//// just for debug or test
		//if (idx > 72)
		//    return;
	}
	system("pause");
	return;
}

void InpaintImage(cv::Mat& imageIn, cv::Mat& imageOut)
{
    cv::Mat imageMask = cv::Mat(imageIn.size(), CV_8UC1, cv::Scalar::all(0));
    for (int i = 0; i < imageIn.rows; i++) {
        for (int j = 0; j < imageIn.cols; j++) {
            const cv::Vec3b& rgb = imageIn.at<cv::Vec3b>(i, j);
            if (rgb[0] == 0 && rgb[1] == 0 && rgb[2] == 0)    // 空洞区域
                imageMask.at<uchar>(i, j) = 255;
        }
    }
    //cv::imshow("maskOri", imageMask);
    //对Mask膨胀处理，增加Mask面积
    cv::Mat Kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::dilate(imageMask, imageMask, Kernel);
    //cv::imshow("maskDilate", imageMask);

    // 计算连通域，评估空洞大小和形状(只修补小空洞)
    cv::Mat maskLabels, stats, centroids;
    int numLabels = cv::connectedComponentsWithStats(imageMask, maskLabels, stats, centroids);

    // 连通域可视化
#if 0
    std::vector<cv::Vec3b> colors;                  //用于区分不同连通域的颜色向量
    colors.push_back(cv::Vec3b(0, 0, 0));           //将背景设为黑色
    cv::RNG rng;			                            //生成一个随机数对象
    for (int i = 1; i < numLabels; i++)             //每一个连通区域，随机生成一种颜色
    {
        int b = rng.uniform(0, 255);
        int g = rng.uniform(0, 255);
        int r = rng.uniform(0, 255);
        colors.push_back(cv::Vec3b(b, g, r));       //按连通域索引顺序，将其代表颜色依次放入向量colors中
    }
    cv::Mat imageMaskColored = cv::Mat::zeros(imageMask.size(), CV_8UC3);     //定义显示的连通图
    for (int row = 0; row < imageMask.rows; row++)
    {
        for (int col = 0; col < imageMask.cols; col++)
        {
            int labelIndex = maskLabels.at<int>(row, col);    //获取label_indexs中该点处的值，也就是该点所处于的连通区域的索引
            if (0 == labelIndex)                   //如果该点为背景区域，则跳过
            {
                continue;
            }
            //将该点所处于的连通区域的代表颜色，赋值给要显示图像该点的像素值
            imageMaskColored.at<cv::Vec3b>(row, col) = colors[labelIndex];
        }
    }
    imshow("imageMaskColored", imageMaskColored);

    for (int j = 1; j < numLabels; j++)
    {
        //获取每个连通域的中心坐标
        int cx = centroids.at<double>(j, 0);
        int cy = centroids.at<double>(j, 1);
        //获取每个连通域的统计信息
        int connected_x = stats.at<int>(j, cv::CC_STAT_LEFT);          //外接矩形左上角的x坐标
        int connected_y = stats.at<int>(j, cv::CC_STAT_TOP);	           //外接矩形左上角的y坐标
        int connected_height = stats.at<int>(j, cv::CC_STAT_HEIGHT);   //外接矩形的高度
        int connected_width = stats.at<int>(j, cv::CC_STAT_WIDTH);     //外接矩形的宽度
        int connected_area = stats.at<int>(j, cv::CC_STAT_AREA);	       //连通域的面积
        if (connected_area < 100)
            continue;
        //绘制每个连通域的中心点
        cv::circle(imageMaskColored, cv::Point(cx, cy), 1, cv::Scalar(0, 255, 0));
        //绘制每个连通域的外接矩形
        cv::Rect rect(connected_x, connected_y, connected_width, connected_height);
        cv::rectangle(imageMaskColored, rect, cv::Scalar(colors[j][0], colors[j][1], colors[j][2]));
        //在图像上输出文字
        cv::putText(imageMaskColored, cv::format("idx_%d", j), cv::Point(cx, cy), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255, 255, 255));
    }
    cv::imshow("connected_image", imageMaskColored);
#endif // 0

    std::unordered_set<int> unInpaintLabels;
    for (int i = 1; i < numLabels; i++) {                           // label 0为背景
        const int area = stats.at<int>(i, cv::CC_STAT_AREA);        // 连通域的面积
        const int height = stats.at<int>(i, cv::CC_STAT_HEIGHT);
        const int width = stats.at<int>(i, cv::CC_STAT_WIDTH);

        // 不修补区域判断条件
        //printf("label: %d, area: %d, height: %d, width: %d, height/width: %d, width/height: %d.\n", i, area, height, width, height / width, width / height);
        if (area > 1500                                             // 空洞区域大
            && (height / width < 5 && width / height < 5)           // 非长条形
            && height * width / area < 2) {                         // 空洞占矩形区域比例超过50%
            unInpaintLabels.insert(i);
            //printf("inserted label %d, uninpaint.\n", i);
        }
    }
    // mask修改，不符合修补条件的mask区域置零
    for (int i = 0; i < imageMask.rows; i++) {
        for (int j = 0; j < imageMask.cols; j++) {
            const int label = maskLabels.at<int>(i, j);
            if (unInpaintLabels.count(label) > 0) {
                imageMask.at<uchar>(i, j) = 0;
            }
        }
    }

    //图像修复
    cv::inpaint(imageIn, imageMask, imageOut, 5, cv::INPAINT_TELEA);

    //cv::imshow("imageIn", imageIn);
    //cv::imshow("Mask", imageMask);
    //cv::imshow("imageOut", imageOut);

    //cv::waitKey();

    return;
}

bool poseRcw2poseRwc(const Eigen::Matrix4f& poseRcw, Eigen::Matrix4f& poseRwc)
{
    Eigen::Matrix3f Rcw;
    Rcw << poseRcw(0, 0), poseRcw(0, 1), poseRcw(0, 2),
        poseRcw(1, 0), poseRcw(1, 1), poseRcw(1, 2),
        poseRcw(2, 0), poseRcw(2, 1), poseRcw(2, 2);

    Eigen::Vector3f Tcw(poseRcw(0, 3), poseRcw(1, 3), poseRcw(2, 3));
    //Eigen::Matrix3f Rwc;
    //bool invertible;
    //Rcw.computeInverseWithCheck(Rwc, invertible);
    //if (!invertible) {
    //    printf("error: Rcw is invertible!\n");
    //}
    const Eigen::Matrix3f Rwc = Rcw.inverse();

    const Eigen::Vector3f Twc = -Rwc * Tcw;       //tcw = -Rcw * twc

    poseRwc << Rwc(0, 0), Rwc(0, 1), Rwc(0, 2), Twc[0],
        Rwc(1, 0), Rwc(1, 1), Rwc(1, 2), Twc[1],
        Rwc(2, 0), Rwc(2, 1), Rwc(2, 2), Twc[2],
        0, 0, 0, 1;

    return true;
}

