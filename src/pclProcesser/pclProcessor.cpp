
#include "pclProcessor.h"

#include <pcl-1.13/pcl/visualization/pcl_visualizer.h>

pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("PCL Viewer"));
pcl::visualization::PCLVisualizer::Ptr view_allcloud(new pcl::visualization::PCLVisualizer("Viewer"));

pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
// pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
int minClusterSize = 10;
int maxClusterSize = 50;
float regionDistanceThresh = 10;
float pnt_color_thresh = 5;
float Distance = 1;

float RadiusSearch = 0.018;
float MinNeighborsInRadius = 5;
bool paramsChanged = true;
bool paramsChange = true;

void convertToRGBPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud);

void keyboardEventOccurred_1(const pcl::visualization::KeyboardEvent &event, void *)
{
    if (event.getKeySym() == "u")
    {
        RadiusSearch += 0.001;
        std::cout << "RadiusSearch: " << RadiusSearch << std::endl;
    }
    else if (event.getKeySym() == "i")
    {
        RadiusSearch -= 0.001;
        if (RadiusSearch < 0)
            RadiusSearch = 0;
        std::cout << "RadiusSearch: " << RadiusSearch << std::endl;
    }
    else if (event.getKeySym() == "k")
    {
        MinNeighborsInRadius += 0.5;
        if (MinNeighborsInRadius < 0)
            MinNeighborsInRadius = 0;
        std::cout << "MinNeighborsInRadius: " << MinNeighborsInRadius << std::endl;
    }
    else if (event.getKeySym() == "l")
    {
        MinNeighborsInRadius -= 0.5;
        if (MinNeighborsInRadius < 0)
            MinNeighborsInRadius = 0;
        std::cout << "MinNeighborsInRadius: " << MinNeighborsInRadius << std::endl;
    }
    paramsChange = true;
}

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void *)
{
    if (event.getKeySym() == "Up")
    {
        minClusterSize += 10;
        std::cout << "MinClusterSize: " << minClusterSize << std::endl;
    }
    else if (event.getKeySym() == "Down")
    {
        minClusterSize -= 10;
        if (minClusterSize < 0)
            minClusterSize = 0;
        std::cout << "MinClusterSize: " << minClusterSize << std::endl;
    }
    else if (event.getKeySym() == "Right")
    {
        maxClusterSize += 10;
        std::cout << "MaxClusterSize: " << maxClusterSize << std::endl;
    }
    else if (event.getKeySym() == "Left")
    {
        maxClusterSize -= 10;
        if (maxClusterSize < 0)
            maxClusterSize = 0;
        std::cout << "MaxClusterSize: " << maxClusterSize << std::endl;
    }
    else if (event.getKeySym() == "n")
    {
        regionDistanceThresh -= 0.1;
        if (regionDistanceThresh < 0)
            regionDistanceThresh = 0;
        std::cout << "regionDistanceThresh: " << regionDistanceThresh << std::endl;
    }
    else if (event.getKeySym() == "m")
    {
        regionDistanceThresh += 0.1;
        if (regionDistanceThresh < 0)
            regionDistanceThresh = 0;
        std::cout << "regionDistanceThresh: " << regionDistanceThresh << std::endl;
    }
    else if (event.getKeySym() == "o")
    {
        pnt_color_thresh -= 0.005;
        if (pnt_color_thresh < 0)
            pnt_color_thresh = 0;
        std::cout << "pnt_color_thresh: " << pnt_color_thresh << std::endl;
    }
    else if (event.getKeySym() == "p")
    {
        pnt_color_thresh += 0.005;
        if (pnt_color_thresh < 0)
            pnt_color_thresh = 0;
        std::cout << "pnt_color_thresh: " << pnt_color_thresh << std::endl;
    }
    else if (event.getKeySym() == "r")
    {
        int minClusterSize = 10;
        int maxClusterSize = 50;
        int regionDistanceThresh = 10;
        float pnt_color_thresh = 0.01;
        std::cout << "Reset: " << std::endl;
    }
    else if (event.getKeySym() == "v")
    {
        Distance -= 0.001;
        if (Distance < 0)
            Distance = 0;
        std::cout << "Distance: " << Distance << std::endl;
    }
    else if (event.getKeySym() == "b")
    {
        Distance += 0.001;
        if (Distance < 0)
            Distance = 0;
        std::cout << "Distance: " << Distance << std::endl;
    }

    paramsChanged = true;
}

std::vector<pcl::PointIndices> RGBgraowing(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
    reg.setInputCloud(cloud);
    // 设置颜色阈值和其他参数
    reg.setMinClusterSize(20);
    reg.setMaxClusterSize(300);
    reg.setDistanceThreshold(2);
    reg.setPointColorThreshold(1.33);
    reg.setSearchMethod(pcl::search::Search<pcl::PointXYZRGB>::Ptr(new pcl::search::KdTree<pcl::PointXYZRGB>));
    reg.setRegionColorThreshold(regionDistanceThresh);

    std::vector<pcl::PointIndices> clusters;
    reg.extract(clusters);
    return clusters;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr RGBgraowing_p(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
    reg.setInputCloud(cloud);
    // 设置颜色阈值和其他参数
    reg.setMinClusterSize(minClusterSize);
    reg.setMaxClusterSize(maxClusterSize);
    reg.setDistanceThreshold(Distance);
    reg.setPointColorThreshold(pnt_color_thresh);
    reg.setSearchMethod(pcl::search::Search<pcl::PointXYZRGB>::Ptr(new pcl::search::KdTree<pcl::PointXYZRGB>));
    reg.setRegionColorThreshold(regionDistanceThresh);

    std::vector<pcl::PointIndices> clusters;
    reg.extract(clusters);
    return reg.getColoredCloud();
}

// pcl::PointCloud<pcl::PointXYZRGB>::Ptr convertToRGBPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud);

void color_based_RegionGrowing(std::vector<cv::Vec6f> Points)
{
    std::cout << "color_based_RegionGrowing ran successfully";

    // pcl::search::Search<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    // pcl::search::Search<pcl::PointXYZ>::Ptr treeE(new pcl::search::KdTree<pcl::PointXYZ>);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudD(new pcl::PointCloud<pcl::PointXYZ>);

    int j = 0;
    for (std::vector<cv::Vec6f>::iterator it = Points.begin(); it != Points.end(); ++it)
    {
        pcl::PointXYZRGB point;
        pcl::PointXYZ PointT;
        PointT.x = point.x = (*it)[0];
        PointT.y = point.y = (*it)[1];
        PointT.z = point.z = (*it)[2];

        point.r = static_cast<uint8_t>((*it)[3]);
        point.g = static_cast<uint8_t>((*it)[4]);
        point.b = static_cast<uint8_t>((*it)[5]);

        cloud->push_back(point);
        cloudD->push_back(PointT);
        j++;
    }
    cout << "num_j=" << j << endl;
    // 移除outliner
    //  创建离群点滤波器对象
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_fed(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::Indices inliers;

    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    // build the filter
    outrem.setInputCloud(cloudD);
    outrem.setRadiusSearch(0.01);
    outrem.setMinNeighborsInRadius(2);
    outrem.setKeepOrganized(true);
    // apply filter
    outrem.filter(inliers);
    // inliers=outrem.getRemovedIndices();
    pcl::IndicesPtr inliersPtr = std::make_shared<pcl::Indices>(inliers);

    // 创建一个 ExtractIndices 对象
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    // 设置输入点云
    extract.setInputCloud(cloudD);

    // 设置要提取的点的索引列表
    extract.setIndices(inliersPtr); // 使用之前获取的离群点索引列表

    // 创建一个新的点云对象，用于存储提取的点
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    // 应用滤波器，提取点云中的指定点
    extract.filter(*cloud_fed);

    int i = 0;
    std::vector<cv::Vec6f> return_points;
    for (const pcl::PointXYZ point : cloud_fed->points)
    {
        pcl::PointXYZRGB tmp_pnt;
        tmp_pnt.x = point.x;
        tmp_pnt.y = point.y;
        tmp_pnt.z = point.z;

        tmp_pnt.r = static_cast<uint8_t>(255 * (1 - tmp_pnt.x / 4.0));
        tmp_pnt.g = static_cast<uint8_t>(0);
        float chanB = 255 * (tmp_pnt.z / 4.0) + 65;
        tmp_pnt.b = static_cast<uint8_t>(chanB > 255 ? 255 : chanB);

        cloud_filtered->push_back(tmp_pnt);
    }
    cout << "num=" << i << endl;
    view_allcloud->removeAllShapes();
    CompressPointCloud(cloud_filtered);

    view_allcloud->removeAllPointClouds();
    view_allcloud->addPointCloud(cloud_filtered);
    view_allcloud->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3);

    view_allcloud->spinOnce();

    // Growing(cloud_filtered);
    // GrowingAndDetecting(cloud_filtered);
    // colorGrowing(cloud_filtered);

    // houghLineDetection(colorGrowing(cloud_filtered));
}

// 点云层压缩
// compress points cloud into mutiple layer former
void CompressPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    for (auto pt_it = cloud->begin(); pt_it != cloud->end(); pt_it++)
    {
        if (pt_it->y <= -0.5)
        {
            pt_it->y = -0.5;
        }

        // else if (-1.2 < pt_it->y <= -1.1)
        //     pt_it->y = -1.1;
        // else if (-1.1 < pt_it->y <= -1)
        //     pt_it->y = -1;
        // else if (-1 < pt_it->y <= -0.9)
        //     pt_it->y = -0.9;
        // else if (-0.9 < pt_it->y <= -0.8)
        //     pt_it->y = -0.8;
        // else if (-0.8 < pt_it->y <= -0.7)
        //     pt_it->y = -0.7;
        // else if (-0.7 < pt_it->y <= -0.6)
        //     pt_it->y = 0.6;
        // else if (-0.6 < pt_it->y <= -0.5)
        //     pt_it->y = -0.5;

        else if (-0.5 < pt_it->y && pt_it->y <= -0.4)
            pt_it->y = -0.4;
        else if (-0.4 < pt_it->y && pt_it->y <= -0.3)
            pt_it->y = -0.3;
        else if (-0.3 < pt_it->y && pt_it->y <= -0.2)
        {
            pt_it->y = -0.2;
        }

        // else if (-0.2 < pt_it->y <= -0.1)
        //     pt_it->y = -0.1;
        else
        {
            pt_it->y = 0;
        }
    }
}

/*without extracting cluster*/
void Growing(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudd)
{
    viewer->registerKeyboardCallback(keyboardEventOccurred, nullptr);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5);

    while (!viewer->wasStopped())
    {
        if (paramsChanged)
        {

            std::vector<pcl::PointIndices> clusters;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored(new pcl::PointCloud<pcl::PointXYZRGB>);
            colored = RGBgraowing_p(cloudd);
            // clusters = RGBgraowing(cloudd);

            /*
                        for (int i = 0; i < cloudd->size(); ++i)
                        {
                            pcl::PointXYZRGB point = (*cloudd)[i];
                            std::cout << "Point " << i << " - RGB Color: (" << static_cast<float>(point.x)
                                      << ", " << static_cast<float>(point.y) << ", " << static_cast<float>(point.z) << ")" << std::endl;
                            std::cout << "Point " << i << " - RGB Color: (" << static_cast<int>(point.r)
                                      << ", " << static_cast<int>(point.g) << ", " << static_cast<int>(point.b) << ")" << std::endl;
                        }

                         viewer->addPointCloud(cloudd);
                        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cluster");
                                viewer->spin();
            */
            // Update the viewer
            viewer->removeAllPointClouds();
            // viewer->addPointCloud(colored_cloud, std::to_string(i) + "_cloud");

            // 可视化显示每个聚类
            int clusterId = 0;

            // for (const pcl::PointIndices &cluster : clusters)
            // {
            //     pcl::PointCloud<pcl::PointXYZRGB>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            //     pcl::ExtractIndices<pcl::PointXYZRGB> extract;
            //     extract.setInputCloud(cloudd);
            //     extract.setIndices(std::make_shared<const pcl::PointIndices>(cluster));
            //     extract.filter(*clusterCloud);
            //     // 定义点云的颜色，例如，这里将点云设置为红色
            //     int red = static_cast<int>(255./clusterId); // 红色分量
            //     int green = static_cast<int>(255./clusterId);   // 绿色分量
            //     int blue = clusterId*5;   // 蓝色分量
            //     // 创建自定义颜色处理器，将颜色应用于整个点云
            //     pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> color_handler(red,green,blue);
            //     // pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZRGB> colorHandler(clusterCloud);
            //     viewer->addPointCloud(clusterCloud,color_handler, "cluster" + std::to_string(clusterId));
            //     viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cluster" + std::to_string(clusterId));
            //     clusterId++;
            // }

            if (colored == nullptr)
            {
                cout << "no cluster" << endl;
                paramsChanged = false;
                continue;
            }
            viewer->addPointCloud(colored);
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5);

            viewer->spinOnce();
            paramsChanged = false;
        }
        viewer->spinOnce();
    }
}

/*extract cluster and fit line*/
void GrowingAndDetecting(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudd)
{
    viewer->registerKeyboardCallback(keyboardEventOccurred, nullptr);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5);

    while (!viewer->wasStopped())
    {
        if (paramsChanged)
        {

            std::vector<pcl::PointIndices> clusters;

            clusters = RGBgraowing(cloudd);

            /*
                        for (int i = 0; i < cloudd->size(); ++i)
                        {
                            pcl::PointXYZRGB point = (*cloudd)[i];
                            std::cout << "Point " << i << " - RGB Color: (" << static_cast<float>(point.x)
                                      << ", " << static_cast<float>(point.y) << ", " << static_cast<float>(point.z) << ")" << std::endl;
                            std::cout << "Point " << i << " - RGB Color: (" << static_cast<int>(point.r)
                                      << ", " << static_cast<int>(point.g) << ", " << static_cast<int>(point.b) << ")" << std::endl;
                        }

                         viewer->addPointCloud(cloudd);
                        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cluster");
                                viewer->spin();
            */
            // Update the viewer
            viewer->removeAllPointClouds();
            // viewer->addPointCloud(colored_cloud, std::to_string(i) + "_cloud");

            // 可视化显示每个聚类
            int clusterId = 1;

            if (!clusters.empty())
            {
                for (const pcl::PointIndices &cluster : clusters)
                {
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
                    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
                    extract.setInputCloud(cloudd);
                    extract.setIndices(std::make_shared<const pcl::PointIndices>(cluster));
                    extract.filter(*clusterCloud);
                    // 定义点云的颜色，例如，这里将点云设置为红色
                    int red = static_cast<uint8_t>(255. / clusterId); // 红色分量
                    // int green = static_cast<uint8_t>(255. / clusterId); // 绿色分量
                    int blue = static_cast<uint8_t>(255 - 255. / clusterId); // 蓝色分量
                    // 创建自定义颜色处理器，将颜色应用于整个点云
                    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> color_handler(clusterCloud, red, 0, blue);
                    // pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZRGB> colorHandler(clusterCloud);
                    viewer->addPointCloud(clusterCloud, color_handler, "cluster" + std::to_string(clusterId));
                    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cluster" + std::to_string(clusterId));
                    clusterId++;
                }
            }

            viewer->spinOnce();
            paramsChanged = false;
        }
        viewer->spinOnce();
    }
}

/*detect line*/
void houghLineDetection(vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters)
{
    viewer->removeAllPointClouds();

    vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> lineCloud;
    for (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &clusterCloud : clusters)
    {

        pcl::SampleConsensusModelLine<pcl::PointXYZRGB>::Ptr model(new pcl::SampleConsensusModelLine<pcl::PointXYZRGB>(clusterCloud));
        pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransac(model);
        ransac.setDistanceThreshold(0.04);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr lines(new pcl::PointCloud<pcl::PointXYZRGB>);

        int color_factor = 0;

        while (clusterCloud->points.size() > 10)
        {
            ransac.computeModel();

            std::vector<int> inliers;
            ransac.getInliers(inliers);

            if (inliers.size() == 0)
            {
                break;
            }

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr line(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::ExtractIndices<pcl::PointXYZRGB> extract;
            extract.setInputCloud(clusterCloud);
            pcl::PointIndices::Ptr inliers_ptr(new pcl::PointIndices());
            inliers_ptr->indices = inliers;
            extract.setIndices(inliers_ptr);
            extract.filter(*line);

            lines->points.insert(lines->points.end(), line->points.begin(), line->points.end());

            extract.setNegative(true);

            lineCloud.push_back(line);
            extract.filter(*clusterCloud);
            color_factor++;
            std::cout << "line num=" << color_factor << endl;
        }
    }
    visualizeCloud(lineCloud);
}

void visualizeCloud(vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> lineClusters)
{
    viewer->removeAllShapes();
    for (size_t i = 0; i < lineClusters.size(); ++i)
    {
        // add point cloud cluster
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr clusterCloud = lineClusters[i];
        pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZRGB> color_handler(clusterCloud);
        viewer->addPointCloud(clusterCloud, color_handler, "cluster" + std::to_string(i));
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cluster" + std::to_string(i));
        // add bounding box
        // addBoundingBox(clusterCloud, i);
    }
    // 显示可视化器
    viewer->spinOnce();
}

std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> colorGrowing(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{

    std::vector<pcl::PointIndices> clusters;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> grownClusters;
    clusters = RGBgraowing(cloud);

    if (!clusters.empty())
    {
        for (const pcl::PointIndices &cluster : clusters)
        {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::ExtractIndices<pcl::PointXYZRGB> extract;
            extract.setInputCloud(cloud);
            extract.setIndices(std::make_shared<const pcl::PointIndices>(cluster));
            extract.filter(*clusterCloud);
            grownClusters.push_back(clusterCloud);
        }
    }
    return grownClusters;
}

// 边界框添加函数
void addBoundingBox(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb, int i)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud_rgb, *cloud);
    pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
    feature_extractor.setInputCloud(cloud);
    feature_extractor.compute();

    pcl::PointXYZ min_point_OBB;
    pcl::PointXYZ max_point_OBB;
    pcl::PointXYZ position_OBB;
    bool OBB_FLAG = true;

    Eigen::Matrix3f rotational_matrix_OBB;
    OBB_FLAG = feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
    if (OBB_FLAG)
    {
        cout << "get OBB sucessfully" << endl;
        Eigen::Vector3f position(position_OBB.x, position_OBB.y, position_OBB.z);
        Eigen::Quaternionf rotation_quaternion(rotational_matrix_OBB);
        // viewer.addPointCloud(cloud, "point_cloud");
        // viewer->addCube(position, rotation_quaternion, max_point_OBB.x - min_point_OBB.x, max_point_OBB.y - min_point_OBB.y, max_point_OBB.z - min_point_OBB.z, "OBB" + std::to_string(i));
        // viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "OBB" + std::to_string(i));
        // viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "OBB" + std::to_string(i));

        view_allcloud->addCube(position, rotation_quaternion, max_point_OBB.x - min_point_OBB.x, max_point_OBB.y - min_point_OBB.y, max_point_OBB.z - min_point_OBB.z, "OBB" + std::to_string(i));
        view_allcloud->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "OBB" + std::to_string(i));
        view_allcloud->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "OBB" + std::to_string(i));
    }
    else
    {
        cout << "Failed to get OBB" << endl;
    }
}

void GrowingNormalBase(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudd)
{
    float NumberOfNeighbours = 10.0,
          SmoothnessThreshold = 15.0 / 180.0 * M_PI, CurvatureThreshold = 1.0;
    int KSearch = 5;
    while (true)
    {
        cout << "go on?" << endl;
        int go;
        cin >> go;
        if (go)
        {
            std::cout << "input KSearch, NumberOfNeighbours, SmoothnessThreshold, CurvatureThreshold:" << endl;
            std::cin >> CurvatureThreshold >> SmoothnessThreshold >> NumberOfNeighbours >> KSearch;
            std::cin >> KSearch >> NumberOfNeighbours >> SmoothnessThreshold >> CurvatureThreshold;
            std::cout << "CurvatureThreshold=" << CurvatureThreshold << " "
                      << "SmoothnessThreshold=" << SmoothnessThreshold << endl;
            std::cout << "NumberOfNeighbours=" << NumberOfNeighbours << " "
                      << "KSearch=" << KSearch << endl;
        }

        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        ne.setInputCloud(cloudd);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        ne.setSearchMethod(tree);
        ne.setKSearch(KSearch); // 设置搜索邻域大小
        ne.compute(*normals);

        // 创建一个区域生长对象：
        pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
        reg.setInputCloud(cloudd);
        reg.setInputNormals(normals);

        // 设置区域生长参数：
        reg.setMinClusterSize(5);                        // 设置最小点云簇的大小
        reg.setMaxClusterSize(1000);                     // 设置最大点云簇的大小
        reg.setSearchMethod(tree);                       // 设置搜索方法
        reg.setNumberOfNeighbours(NumberOfNeighbours);   // 设置邻居点的数量
        reg.setSmoothnessThreshold(SmoothnessThreshold); // 设置平滑度阈值
        reg.setCurvatureThreshold(CurvatureThreshold);   // 设置曲率阈值

        // 执行区域生长聚类：
        std::vector<pcl::PointIndices> clusters;
        reg.extract(clusters);

        viewer->removeAllPointClouds();

        // 可视化
        if (!clusters.empty())
        {
            int clusterId = 1;
            for (const pcl::PointIndices &cluster : clusters)
            {
                pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::ExtractIndices<pcl::PointXYZ> extract;
                extract.setInputCloud(cloudd);
                extract.setIndices(std::make_shared<const pcl::PointIndices>(cluster));
                extract.filter(*clusterCloud);
                // 定义点云的颜色，例如，这里将点云设置为红色
                int red = static_cast<uint8_t>(255. / clusterId); // 红色分量
                // int green = static_cast<uint8_t>(255. / clusterId); // 绿色分量
                int blue = static_cast<uint8_t>(255 - 255. / clusterId); // 蓝色分量
                // 创建自定义颜色处理器，将颜色应用于整个点云
                pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler(clusterCloud, red, 0, blue);
                // pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZRGB> colorHandler(clusterCloud);
                viewer->addPointCloud(clusterCloud, color_handler, "cluster" + std::to_string(clusterId));
                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cluster" + std::to_string(clusterId));
                clusterId++;
            }
        }
        viewer->spinOnce();
    }
}

// void visualizePointCloud(const std::map<float, std::vector<Eigen::Vector4f>> &classifiedVectors)
// {
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
//     viewer->removeAllPointClouds();
//     int color = 0;
//     for (const auto &pair : classifiedVectors)
//     {
//         uint8_t r = (color * 113) % 256;
//         uint8_t g = (color * 179) % 256;
//         uint8_t b = (color * 233) % 256;
//         color++;

//         for (const auto &vec : pair.second)
//         {
//             pcl::PointXYZRGB point;
//             point.x = vec[0];
//             point.y = vec[1];
//             point.z = vec[2];
//             uint32_t rgb = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
//             point.rgb = *reinterpret_cast<float *>(&rgb);
//             cloud->points.push_back(point);
//         }
//     }

//     // pcl::visualization::PCLVisualizer viewer("4D Vector Visualization");
//     viewer->addPointCloud<pcl::PointXYZRGB>(cloud, "cloud");
//     viewer->setBackgroundColor(0, 0, 0); // 设置背景为黑色
//     viewer->addCoordinateSystem(1.0);
//     viewer->initCameraParameters();

//     viewer->spinOnce();

//     return;
// }