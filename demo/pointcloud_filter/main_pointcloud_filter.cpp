#include "filter.h"
#include "camera_pose_generation.h"

#include <io.h>
#include <direct.h>

#define MIN_POINTS_SAVED    5000      // voxel size 为0.01时，删除一千点以下的类
// 1000:行人残影;
#define MAX_POINTS_OBJECT   50000     // voxel size 为0.01时，五万点以下的类视为物体
#define SEARCH_RADIUS       0.05f     // 聚类搜索距离
#define MIN_CLUSTER_SIZE    100
#define MAX_CLUSTER_SIZE    100000000

// E1:[0, 0.1]; ZDGC_F2:[0.05, -0.05]
#define GROUND_PLANE_MAX_Z  0.05f
#define GROUND_PLANE_MIN_Z  -0.05f

int main(int argc, char** argv)
{
    //std::string workspace = "F:\\dataset\\CG\\E1_Hall\\pointclouds\\E1\\filtered";
    //std::string workspace = "F:\\dataset\\CG\\BYS\\F1\\2019-12-17_14.30.00\\pointclouds";
    std::string workspace = "F:\\dataset\\CG\\ZDGC\\B1\\2020-03-30_20.26.27\\pointclouds";
    //std::string fileName = workspace + "\\pointcloud_simp0.01.ply";
    //std::string fileName = workspace + "\\pointcloud_simp0.01_segForCamera.ply";
    std::string fileName = workspace + "\\forCamera_add.ply";
    printf("workspace : %s\nfileName  : %s\n", workspace.c_str(), fileName.c_str());

    std::string outFolder = workspace + "\\filtered";
    if (_access(outFolder.c_str(), 0) == -1) {
        mkdir(outFolder.c_str());
    }

    // 0:只进行相机位姿生成, 1:点云滤波+生成位姿
#if 0
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr inputCloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PLYReader reader;
    reader.read(fileName, *inputCloud);
    printf("PointCloud before filtering has %d  data points.\n", inputCloud->points.size());

    pcl::PLYWriter writer;

    // VoxelGrid下采样，小于4cm时，叶节点数int值溢出，采样失效
#if 0    
    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<pcl::PointXYZRGB> vg;         //体素栅格下采样对象
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    vg.setInputCloud(cloud);
    vg.setLeafSize(0.04f, 0.04f, 0.04f);         //设置采样的体素大小
    vg.filter(*cloud_filtered);
    std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size() << " data points." << std::endl;
#endif

    // SACSegmentation提取平面，受点分布影响，地面提取会歪
#if 0
    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr planeCloud(new pcl::PointCloud<pcl::PointXYZRGB>());

    seg.setOptimizeCoefficients(true);          //设置对估计的模型参数进行优化处理
    seg.setModelType(pcl::SACMODEL_PLANE);      //设置分割模型
    seg.setMethodType(pcl::SAC_RANSAC);         //设置随机参数估计方法
    seg.setMaxIterations(100);                  //设置最大迭代次数
    seg.setDistanceThreshold(0.15);             //设置判断是否为模型内点的距离阈值

    int i = 0, numPoints = (int)inputCloud->points.size();
    //while (cloud_filtered->points.size() > 0.3 * numPoints)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud(inputCloud);
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size() == 0) {
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            //break;
        }

        //移去平面局内点，提取剩余点云
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        extract.setInputCloud(inputCloud);
        extract.setIndices(inliers);
        extract.setNegative(false);             //设置提取内点而非外点
        // Get the points associated with the planar surface
        extract.filter(*planeCloud);            //提取输出存储到cloud_plane
        std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size() << " data points." << std::endl;
        std::stringstream ssPlane;
        ssPlane << outFolder << "\\cloud_plane.ply";
        writer.write<pcl::PointXYZRGB>(ssPlane.str(), *planeCloud, false);

        // Remove the planar inliers, extract the rest
        extract.setNegative(true);
        extract.filter(*fCloud);
        *cloud_input = *fCloud;
    }
#endif

    if (inputCloud->points.size() > INT_MAX) {
        printf("point size is greater than INT_MAX!\n");
        return 0;
    }

    // 识别地面点
    pcl::PointIndices::Ptr groundPlane(new pcl::PointIndices);
    for (int i = 0; i < inputCloud->points.size(); i++) {
        if (inputCloud->points[i].z < GROUND_PLANE_MAX_Z
            && inputCloud->points[i].z > GROUND_PLANE_MIN_Z) {
            groundPlane->indices.push_back(i);
        }
    }
    // 提取地面点
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr planeCloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    pcl::ExtractIndices<pcl::PointXYZRGBNormal> extract;
    extract.setInputCloud(inputCloud);
    extract.setIndices(groundPlane);
    extract.setNegative(false);                 //设置提取内点而非外点
    extract.filter(*planeCloud);
    printf("PointCloud representing the planar component has %d points.\n", planeCloud->points.size());

    std::stringstream ssPlane;
    ssPlane << outFolder << "\\cloud_plane.ply";
    writer.write<pcl::PointXYZRGBNormal>(ssPlane.str(), *planeCloud, false);

    // 提取剩下的点云
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr remainingCloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    extract.setNegative(true);
    extract.filter(*remainingCloud);

    // 移除地面上的离群点
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr savedPlane(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr removedPlane(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    ClusteringSegmentation(planeCloud,
        savedPlane,
        removedPlane,
        SEARCH_RADIUS,
        MIN_CLUSTER_SIZE,
        MAX_CLUSTER_SIZE,
        MIN_POINTS_SAVED);

    std::stringstream ssPlaneSaved;
    ssPlaneSaved << outFolder << "\\cloud_plane_saved.ply";
    writer.write<pcl::PointXYZRGBNormal>(ssPlaneSaved.str(), *savedPlane, false);
    std::stringstream ssPlaneRemoved;
    ssPlaneRemoved << outFolder << "\\cloud_plane_removed.ply";
    writer.write<pcl::PointXYZRGBNormal>(ssPlaneRemoved.str(), *removedPlane, false);

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr structureCloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr objectsCloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr removedCloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

    // 不保存聚类中间结果，只保存最终结果
#if 1
    ClusteringSegmentation(remainingCloud,
        structureCloud,
        removedCloud,
        SEARCH_RADIUS,
        MIN_CLUSTER_SIZE,
        MAX_CLUSTER_SIZE,
        MIN_POINTS_SAVED,
        objectsCloud,
        MAX_POINTS_OBJECT);

#else   
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(cloud_remaining);       //创建点云索引向量，用于存储实际的点云信息

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance(CLUSTER_TOLERANCE);  //设置近邻搜索的搜索半径
    ec.setMinClusterSize(MIN_CLUSTER_SIZE);     //设置一个聚类需要的最少点数目
    ec.setMaxClusterSize(MAX_CLUSTER_SIZE);     //设置一个聚类需要的最大点数目
    ec.setSearchMethod(tree);                   //设置点云的搜索机制
    ec.setInputCloud(cloud_remaining);
    ec.extract(cluster_indices);
    std::cout << "end EuclideanClusterExtraction. Extracted  : " << cluster_indices.size() << "clusters." << std::endl;

    //迭代访问点云索引cluster_indices，直到分割出所有聚类
    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        if (it->indices.size() < MIN_POINTS_SAVED) {
            std::cout << "cluster removerd has " << it->indices.size() << "points" << std::endl;
            for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
                cloud_removed->points.push_back(cloud_remaining->points[*pit]);
            }
            continue;
        }

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
        //创建新的点云数据集cloud_cluster，将所有当前聚类写入到点云数据集中
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
            cloud_cluster->points.push_back(cloud_remaining->points[*pit]);
            cloud_saved->points.push_back(cloud_remaining->points[*pit]);
        }
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points." << std::endl;
        std::stringstream ss;
        ss << workspace << "\\filtered\\cloud_cluster_" << j << ".ply";
        writer.write<pcl::PointXYZRGB>(ss.str(), *cloud_cluster, false);
        j++;
    }
#endif // 0

    // 把地面添加到滤波后的点云
    for (int it = 0; it < savedPlane->size(); ++it) {
        structureCloud->points.push_back(savedPlane->points[it]);
    }
    printf("PointCloud removed %d points.\nPointCloud saved   %d points.\n", removedCloud->points.size(), structureCloud->points.size());

    std::stringstream ssRemoved;
    ssRemoved << outFolder << "\\cloud_removed.ply";
    writer.write<pcl::PointXYZRGBNormal>(ssRemoved.str(), *removedCloud, false);
    std::stringstream ssStructure;
    ssStructure << outFolder << "\\cloud_structure.ply";
    writer.write<pcl::PointXYZRGBNormal>(ssStructure.str(), *structureCloud, false);
    std::stringstream ssObjects;
    ssObjects << outFolder << "\\cloud_objects.ply";
    writer.write<pcl::PointXYZRGBNormal>(ssObjects.str(), *objectsCloud, false);

#else 
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr savedPlane(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr structureCloud = nullptr;
    pcl::PLYReader reader;
    reader.read(fileName, *savedPlane);
    printf("PointCloud before filtering has %d  data points.\n", savedPlane->points.size());
#endif

    // 生成相机位姿，并保存, Rcw & Tcw
    vector<Eigen::Matrix4f> posesRcw;
    vector<Eigen::Matrix4f> posesRwc;
    // 使用滤波后的点云，去除行人影响
    GenerateCameraPose(savedPlane, posesRcw, posesRwc, outFolder, structureCloud);
    if (posesRcw.size() != posesRwc.size()) {
        printf("error: posesRcw.size() = %d, posesRwc.size() = %d\n", posesRcw.size(), posesRwc.size());
    }
    printf("generate %d camera poses.\n", posesRcw.size());
    std::stringstream poseFileRcw;
    poseFileRcw << outFolder << "\\cameraPoseRcw_NewPoseGen.txt";
    WriteCameraPose(poseFileRcw.str(), posesRcw);
    printf("save camera pose to %s.\n", poseFileRcw.str().c_str());

    std::stringstream poseFileRwc;
    poseFileRwc << outFolder << "\\cameraPoseRwc_NewPoseGen.txt";
    WriteCameraPose(poseFileRwc.str(), posesRwc);
    printf("save camera pose to %s.\n", poseFileRwc.str().c_str());

    return 0;
}
