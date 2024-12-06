#include <iostream>
#include <fstream>
#include <io.h>
#include <direct.h>
#include <string>

#include <Eigen/Core>
#include <Eigen/LU> 
#include <Eigen/Geometry> 
#include <opencv2/opencv.hpp>

using namespace std;

struct CameraIntrinsics
{
    double fx;
    double fy;
    double cx;
    double cy;
    int width;
    int height;
};
// 图像坐标到相机坐标
bool i2c(const Eigen::Vector2d &p_i, const double &depth, 
    Eigen::Vector3d *p_c, const CameraIntrinsics &intrinsics)
{
    const int x = static_cast<int>(p_i[0]);
    const int y = static_cast<int>(p_i[1]);

    if (x < 0 || x >= intrinsics.width
        || y < 0 || y >= intrinsics.height) {
        return false;
    }

    Eigen::Vector2d p_n;
    p_n[0] = (p_i[0] - intrinsics.cx) / intrinsics.fx;
    p_n[1] = (p_i[1] - intrinsics.cy) / intrinsics.fy;

    *p_c = depth * p_n.homogeneous();

    return true;
}

// 相机坐标到世界坐标
void c2w(const Eigen::Vector3d &p_c, Eigen::Vector3d *p_w,
    const Eigen::Matrix3d &rotation, const Eigen::Vector3d &translation) {
   // Rwc
    const Eigen::Vector3d p_r = rotation * p_c;
    *p_w = p_r + translation;
}

// 图像坐标到世界坐标
bool i2w(const Eigen::Vector2d &p_i, const double &depth, Eigen::Vector3d *p_w,
    const CameraIntrinsics &intrinsics, const Eigen::Matrix3d &rotation, const Eigen::Vector3d &translation) {
    Eigen::Vector3d p_c;
    
    if (!i2c(p_i, depth, &p_c, intrinsics)) {
        return false;
    }
    c2w(p_c, p_w, rotation, translation);

    return true;
}

// 世界坐标到相机坐标
void w2UTM(const Eigen::Vector3d &p_w, Eigen::Vector3d *p_u,
    const Eigen::Matrix3d &rotation, const Eigen::Vector3d &translation, const Eigen::Vector3d &shift)
{
    const Eigen::Vector3d p_r = rotation * p_w;
    *p_u = p_r + translation + shift;
}

// 世界坐标到相机坐标(Rwc)
void w2c(const Eigen::Vector3d &p_w, Eigen::Vector3d *p_c,
    const Eigen::Matrix3d rotation, const Eigen::Vector3d &translation)
{
    // Rwc
    const Eigen::Vector3d p_r = p_w - translation;
    *p_c = rotation.inverse() * p_r;
}

// 相机坐标到图像坐标
bool c2i(const Eigen::Vector3d &p_c, Eigen::Vector2d *p_i,
    const CameraIntrinsics &intrinsics)
{
    if (p_i == nullptr) {
        printf("p_i == nullptr!\n");
        return false;
    }

    double depth = p_c[2];
    (*p_i)[0] = p_c[0] / depth * intrinsics.fx + intrinsics.cx;
    (*p_i)[1] = p_c[1] / depth * intrinsics.fy + intrinsics.cy;

    return true;
}

// 世界坐标到图像坐标
bool w2i(const Eigen::Vector3d &p_w, Eigen::Vector2d *p_i, double &depth,
    const CameraIntrinsics &intrinsics, const Eigen::Matrix3d &rotation, const Eigen::Vector3d &translation)
{
    Eigen::Vector3d p_c;
    w2c(p_w, &p_c, rotation, translation);

    if (!c2i(p_c, p_i, intrinsics)) {
        return false;
    }

    return true;
}

bool depthImg2PointCloud(const string depthFileName)
{
    CameraIntrinsics intrinsics;
    const Eigen::Matrix3d rotation;
    const Eigen::Vector3d translation;

    cv::Mat depthImgSrc = cv::imread(depthFileName, CV_LOAD_IMAGE_UNCHANGED);
    if (depthImgSrc.data == NULL) {
        printf("Error read depth image!\n");
        return false;
    }
    cv::Mat depthImg = cv::Mat::zeros(480, 640, CV_16UC1);
    cv::resize(depthImgSrc, depthImg, depthImg.size());

    //pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (int i = 0; i < depthImg.rows - 1; i++) {
        for (int j = 0; j < depthImg.cols - 1; j++) {
            const Eigen::Vector2d p_i(static_cast<double>(j), static_cast<double>(i));

            Eigen::Vector3d p_w;
            unsigned short depthShort = depthImg.at<unsigned short>(i, j);
            double depth = (double)depthShort / 1000;

            if (!i2w(p_i, depth, &p_w, intrinsics, rotation, translation)) {
                continue;
            }

            //pointcloud->push_back(pcl::PointXYZ(p_w[0], p_w[1], p_w[2]));
        }
    }
}

// position:Twc, rotarion:Rwc
void drawCameraPose(std::string filename, Eigen::Vector3f position, Eigen::Matrix3f rotation)
{
    //std::string filename = saveDir + "/camera.ply";
    float pointDIst = 0.1;    // 坐标轴点分布间隔，单位m
    std::ofstream out(filename);
    std::vector<Eigen::Vector3f> vec;
    std::vector<Eigen::Vector3i> color;
    for (int i = 0; i < 100; i++) {
        Eigen::Vector3f p = position + i * pointDIst * rotation.col(0);
        vec.push_back(p);
        color.push_back(Eigen::Vector3i(255, 0, 0));
        p = position + i * pointDIst * rotation.col(1);
        vec.push_back(p);
        color.push_back(Eigen::Vector3i(0, 255, 0));
        p = position + i * pointDIst * rotation.col(2);
        vec.push_back(p);
        color.push_back(Eigen::Vector3i(0, 0, 255));
    }

    out << "ply";
    out << "\nformat " << "ascii" << " 1.0";
    out << "\nelement vertex " << vec.size();
    out << "\nproperty float x"
        "\nproperty float y"
        "\nproperty float z";
    out << "\nproperty uchar red"
        "\nproperty uchar green"
        "\nproperty uchar blue";
    out << "\nend_header\n";
    for (int i = 0; i < vec.size(); i++) {
        out << vec[i][0] << " " << vec[i][1] << " " << vec[i][2] << " "
            << color[i][0] << " " << color[i][1] << " " << color[i][2] << std::endl;
    }
    out.close();
    return;
}

void drawNavVisQueryPoses()
{
    const string poseName = "F:\\dataset\\CG\\E1_Hall\\scene\\query_107-111_cam1\\cameraPoseQryRcw.txt";
    std::ifstream fs(poseName);
    if (!fs.is_open()) {
        printf("error : open file %s  filed!\n", poseName.c_str());
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
        Eigen::Matrix3f Rwc = Rcw.inverse();

        std::string filename = "F:\\dataset\\CG\\E1_Hall\\scene\\query_107-111_cam1\\" + imgName + "_camera.ply";
        drawCameraPose(filename, Twc, Rwc);
    }
    return;
}

void drawImgRwcPoses()
{
    const string workspace = "F:\\dataset\\CG\\E1_Hall\\pointclouds\\E1\\filtered\\filtered";

    const string poseName = workspace + "\\imagePoseRwc_NewPoseGen.txt";
    std::ifstream fs(poseName);
    if (!fs.is_open()) {
        printf("error : open file %s  filed!\n", poseName.c_str());
        return;
    }
    int idx = 0;
    while (!fs.eof()) {
        Eigen::Vector3f Twc;
        Eigen::Matrix3f Rwc;
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                fs >> Rwc(i, j);
            }
            fs >> Twc[i];
        }
        float unused;
        fs >> unused >> unused >> unused >> unused;

        std::string filename = workspace + "\\" + std::to_string(idx) + "_camera.ply";
        drawCameraPose(filename, Twc, Rwc);
        idx++;
    }
    return;
}

void drawSinglePose()
{

#if 1
    Eigen::Vector3f Tcw(1.846690, -10.581500, 13.015900);
    Eigen::Matrix3f Rcw;
    Rcw << -0.649585, -0.760182, -0.012680,
        0.020304, -0.034018, 0.999214,
        -0.7600172, 0.648818, 0.037533;

    Eigen::Matrix3f Rwc = Rcw.inverse();
    Eigen::Vector3f Twc = -Rwc * Tcw;
    //Eigen::Vector3f Twc(368445.458892 - 368394, 3459121.715102 - 3459110, 17.707578);
#else
    Eigen::Vector3f Twc(-19.249960, -36.248432, 1.439999);
    Eigen::Matrix3f Rwc;
    Rwc << -0.000000, 0.258819, 0.965926, -1.000000, 0.000000, 0.000000, -0.000000, -0.965926, 0.258819;
    //Rwc << 1, 0, 0, 0, 1, 0, 0, 0, 1;      // 原始相机位置，无旋转
#endif

    std::string filename = "F:\\dataset\\CG\\E1_Hall\\scene\\query_107-111_cam1\\GT_2130_camera_Rcwnew.ply";
    //std::string filename = "F:\\dataset\\CG\\BYS\\F1\\2019-12-17_14.30.00\\scene\\query_0-179_cam1\\30_camera.ply";
    drawCameraPose(filename, Twc, Rwc);

    return;

}

int main(int argc, char **argv)
{
    // NavVis pose生成与测试共用，0：测试；1：生成
#if 0
    drawImgRwcPoses();
    return 0;

#else
    const string workspace = "F:\\dataset\\CG\\ZDGC\\B1\\2020-03-30_20.26.27\\scene";
    const string poseOriName = "\\cameraPoseOriRTcw.txt";
    const string poseQryName = "\\cameraPoseQryRcw.txt";
    const int imgBeg = 0;
    const int imgEnd = 193;
    const int camId = 4;

    const string queryFolder = workspace + "\\query_" + to_string(imgBeg) + "-" + to_string(imgEnd - 1) + "_cam" + to_string(camId);
    if (_access(queryFolder.c_str(), 0) == -1) {
        mkdir(queryFolder.c_str());
    }
    const string imgOriFolder = queryFolder + "\\imgOri";
    if (_access(imgOriFolder.c_str(), 0) == -1) {
        mkdir(imgOriFolder.c_str());
    }
    const string imgQryFolder = queryFolder + "\\imgQry";
    if (_access(imgQryFolder.c_str(), 0) == -1) {
        mkdir(imgQryFolder.c_str());
    }

    const string poseOriFullName = queryFolder + poseOriName;
    ofstream outOri;
    outOri.open(poseOriFullName);

    const string poseQryFullName = queryFolder + poseQryName;
    ofstream outQry;
    outQry.open(poseQryFullName);

    for (int i = imgBeg; i < imgEnd; i++) {
        char id[256];
        sprintf(id, "%05d-cam%d", i, camId);
        string poseFullName = workspace + "\\" + id + ".CAM";
        string imgFullName = workspace + "\\" + id + ".png";
        string imgName = string(id) + ".jpg";
        string imgOriFullName = imgOriFolder + "\\" + id + ".jpg";
        string imgQryFullName = imgQryFolder + "\\" + id + ".jpg";
        ifstream in(poseFullName);
        if (!in) {
            continue;
        }

        cv::Mat imgOri = cv::imread(imgFullName, cv::IMREAD_COLOR);
        cv::imwrite(imgOriFullName, imgOri);    // 先保存原始图像

        // 图像旋转，图像和pose单独操作，只要保证最后pose&图像匹配即可
        cv::Mat imgTrans, imgQry;
        cv::transpose(imgOri, imgTrans);
        cv::flip(imgTrans, imgTrans, 1);
        cv::resize(imgTrans, imgQry, cv::Size(480, 640));
        cv::imwrite(imgQryFullName, imgQry);

        Eigen::Vector3f Tcw;
        Eigen::Matrix3f Rcw;

        for (int j = 0; j < 3; j++) {
            in >> Tcw[j];
        }

        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 3; k++) {
                in >> Rcw(j, k);
            }
        }

        outOri << imgName << '\t';
        outOri << setiosflags(ios::fixed) << setprecision(6) 
            << Tcw[0] << '\t' << Tcw[1] << '\t' << Tcw[2] << '\t';
        outOri << setiosflags(ios::fixed) << setprecision(6)
            << Rcw(0, 0) << '\t' << Rcw(0, 1) << '\t' << Rcw(0, 2) << '\t'
            << Rcw(1, 0) << '\t' << Rcw(1, 1) << '\t' << Rcw(1, 2) << '\t'
            << Rcw(2, 0) << '\t' << Rcw(2, 1) << '\t' << Rcw(2, 2) << '\n';

#if 0
        // 图像旋转，XY坐标调换，X取负——>navvis鱼眼图旋转90度。（注意：Tcw也变换，Twc不用）
        Eigen::Vector3f TcwTrans;
        Eigen::Matrix3f RcwTrans;
        TcwTrans[0] = -Tcw[1];
        TcwTrans[1] = Tcw[0];
        TcwTrans[2] = Tcw[2];
        RcwTrans << -Rcw(1, 0), -Rcw(1, 1), -Rcw(1, 2),
            Rcw(0, 0), Rcw(0, 1), Rcw(0, 2),
            Rcw(2, 0), Rcw(2, 1), Rcw(2, 2);

        // 图像旋转，pose跟着变。统一用变换后的R、T计算
        // 或者Twc用原始RcwTcw计算(RT一致即可，变换前后计算结果一致)，RwcTrans用Rwc变换
        // [R|T]矩阵，RTwc列变换，Tx&Ty不变，RTcw行变换，Tx&Ty在同一行，跟着变
        Eigen::Vector3f TwcTrans;
        Eigen::Matrix3f RwcTrans;
        RwcTrans = RcwTrans.inverse();
        TwcTrans = -RwcTrans * TcwTrans;    // Twc = -Rwc * Tcw;

        outQry << imgName << '\t';
        outQry << setiosflags(ios::fixed) << setprecision(6)
            << TwcTrans[0] << '\t' << TwcTrans[1] << '\t' << TwcTrans[2] << '\t';
        outQry << setiosflags(ios::fixed) << setprecision(6)
            << RcwTrans(0, 0) << '\t' << RcwTrans(0, 1) << '\t' << RcwTrans(0, 2) << '\t'
            << RcwTrans(1, 0) << '\t' << RcwTrans(1, 1) << '\t' << RcwTrans(1, 2) << '\t'
            << RcwTrans(2, 0) << '\t' << RcwTrans(2, 1) << '\t' << RcwTrans(2, 2) << '\n';
#endif

        Eigen::Matrix3f Rwc = Rcw.inverse();
        Eigen::Vector3f Twc = -Rwc * Tcw;

        // Rwc xy调换，x取负
        for (int i = 0; i < 3; i++) {
            float temp = Rwc(i, 1);
            Rwc(i, 1) = Rwc(i, 0);
            Rwc(i, 0) = -temp;
        }

        // 保存ground truth[Twc, Rcw]
        outQry << imgName << '\t';
        outQry << setiosflags(ios::fixed) << setprecision(6)
            << Twc[0] << '\t' << Twc[1] << '\t' << Twc[2] << '\t';
        outQry << setiosflags(ios::fixed) << setprecision(6)
            << Rwc(0, 0) << '\t' << Rwc(1, 0) << '\t' << Rwc(2, 0) << '\t'
            << Rwc(0, 1) << '\t' << Rwc(1, 1) << '\t' << Rwc(2, 1) << '\t'
            << Rwc(0, 2) << '\t' << Rwc(1, 2) << '\t' << Rwc(2, 2) << '\n';

        printf("generate query image %s.\n", imgName);
    }
    return 0;
#endif
}
