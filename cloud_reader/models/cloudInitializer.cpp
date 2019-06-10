#include "cloudInitializer.h"


int CloudInitializer::searchForZClosestPoint(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud)
{
    int closest_index;
    double z_min = 10000;
    for (int i = 0; i < cloud->size(); i++)
    {
        if (cloud->points[i].z < z_min && cloud->points[i].z > 0.1)
        {
            closest_index = i;
            z_min = cloud->points[i].z;
        }
    }
    return closest_index;
}

int CloudInitializer::searchForZFarestPoint(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud)
{
    int farest_index;
    double z_max = 0.1;
    for (int i = 0; i < cloud->size(); i++)
    {
        if (cloud->points[i].z > z_max)
        {
            farest_index = i;
            z_max = cloud->points[i].z;
        }
    }
    return farest_index;
}

int CloudInitializer::searchForXFarestNegativePoint(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud)
{
    int index;
    double x_min = 0;
    for (int i = 0; i < cloud->size(); i++)
    {
        if (cloud->points[i].x < x_min)
        {
            index = i;
            x_min = cloud->points[i].x;
        }
    }
    return index;
}

int CloudInitializer::searchForXFarestPositivePoint(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud)
{
    int index;
    double x_max = 0;
    for (int i = 0; i < cloud->size(); i++)
    {
        if (cloud->points[i].x > x_max)
        {
            index = i;
            x_max = cloud->points[i].x;
        }
    }
    return index;
}

int CloudInitializer::searchForYHighestPoint(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud)
{
    int index = 0;
    double y_min = -100;
    for (int i = 0; i < cloud->size(); i++)
    {
        if (cloud->points[i].y > y_min)
        {
            index = i;
            y_min = cloud->points[i].y;
        }
    }
    return index;
}

int CloudInitializer::searchForYLowestPoint(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud)
{
    int index = 0;
    double y_max = 0;
    for (int i = 0; i < cloud->size(); i++)
    {
        if (cloud->points[i].y < y_max)
        {
            index = i;
            y_max = cloud->points[i].y;
        }
    }
    return index;
}

//CALUCLATES TABLE SIZE AND FILTERS IT AND WRITEES THAT CLOUD
void CloudInitializer::CalculateTableSizeAndFilter(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, const char *table_cloud_name)
{
    //REMOVE POINTS THAT ARE NOT NEEDED
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_clipped(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PassThrough<pcl::PointXYZRGBA> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("y");
    pass.setFilterLimitsNegative(false);
    pass.setFilterLimits(-0.3, 0.4);
    pass.filter(*cloud_clipped);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_voxelised(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::VoxelGrid<pcl::PointXYZRGBA> vg;
    vg.setInputCloud(cloud_clipped);
    vg.setLeafSize(0.01f, 0.01f, 0.01f);
    vg.filter(*cloud_voxelised);

    pcl::search::KdTree<pcl::PointXYZRGBA> kdtree;
    kdtree.setInputCloud(cloud_voxelised);
    std::vector<int> pointIdxSearch;
    std::vector<float> pointSquaredDistance;
    kdtree.radiusSearch(CloudInitializer::searchForZClosestPoint(cloud_voxelised), 1, pointIdxSearch, pointSquaredDistance);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZRGBA>);
    cloud2->width = pointIdxSearch.size();
    cloud2->height = 1;
    cloud2->is_dense = false;
    for (int i = 0; i < pointIdxSearch.size(); i++)
        cloud2->push_back(cloud_voxelised->points[pointIdxSearch[i]]);

    float length = pcl::geometry::distance(cloud2->points[CloudInitializer::searchForZFarestPoint(cloud2)], cloud2->points[CloudInitializer::searchForZClosestPoint(cloud2)]);
    float width = pcl::geometry::distance(cloud2->points[CloudInitializer::searchForXFarestPositivePoint(cloud2)], cloud2->points[CloudInitializer::searchForXFarestNegativePoint(cloud2)]);

    std::cout << "Table Size: " << int(length * 100) << " X " << int(width * 100) << std::endl;

    //SET WHATS IMPORTANT
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(cloud2->points[CloudInitializer::searchForZClosestPoint(cloud2)].z, cloud2->points[CloudInitializer::searchForZFarestPoint(cloud2)].z);
    pass.filter(*cloud);

    pass.setFilterFieldName("x");
    pass.setFilterLimits(cloud2->points[CloudInitializer::searchForXFarestNegativePoint(cloud2)].x, cloud2->points[CloudInitializer::searchForXFarestPositivePoint(cloud2)].x);
    pass.filter(*cloud);

    //Create cloud with only table and object
    pcl::io::savePCDFileASCII("../clouds/apenas_com_mesa/" + std::string(table_cloud_name), *cloud);
std:
    cout << "Saved new cloud at: "
         << "../clouds/apenas_com_mesa/" + std::string(table_cloud_name) << std::endl;
}

void CloudInitializer::FindObjectAndCreateFile(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, const char *filename)
{
    //RANSAC TABLE
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    //filter table top and remove it
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_on_plane(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_on_plane);

    //calculate points above tabletop
    double a = coefficients->values[0];
    double b = coefficients->values[1];
    double c = coefficients->values[2];
    double d = coefficients->values[3];
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_object(new pcl::PointCloud<pcl::PointXYZRGBA>);
    for (int i = 0; i < cloud_on_plane->size(); i++)
    {
        pcl::PointXYZRGBA point = cloud_on_plane->points[i];
        if (pcl::pointToPlaneDistanceSigned(point, a, b, c, d) > -0.01)
        {
            cloud_object->push_back(point);
        }
    }

    //Removal of Outliners
    pcl::RadiusOutlierRemoval<pcl::PointXYZRGBA> sor;
    sor.setInputCloud(cloud_object);
    sor.setRadiusSearch(0.01);
    sor.setMinNeighborsInRadius(20);
    sor.filter(*cloud_object);

    //CALCULATE COLOR INTENSITY
    double r_mean = 0.0;
    double g_mean = 0.0;
    double b_mean = 0.0;
    for (int i = 0; i < cloud_object->size(); i++)
    {
        r_mean += cloud_object->points[i].r;
        g_mean += cloud_object->points[i].g;
        b_mean += cloud_object->points[i].b;
    }
    r_mean = r_mean / cloud_object->size();
    g_mean = g_mean / cloud_object->size();
    b_mean = b_mean / cloud_object->size();

    double width = pcl::geometry::distance(cloud_object->points[searchForXFarestPositivePoint(cloud_object)], cloud_object->points[searchForXFarestNegativePoint(cloud_object)]);
    double height = pcl::geometry::distance(cloud_object->points[searchForYHighestPoint(cloud_object)], cloud_object->points[searchForYLowestPoint(cloud_object)]);

    int volume = CloudInitializer::calculateCloudVolume(cloud_object);

    std::string pathString = "../clouds/objectos_isolados/" + std::string(filename) + ".txt";
    const char *path = pathString.c_str();
    std::cout << "Writing file to: " << path << std::endl;
    ofstream newFile;
    newFile.open(path, fstream::out);
    newFile << filename << " " << std::round(height * 100) << " " << std::round(width * 100) << " " << volume << " " << std::round(r_mean) << " " << std::round(g_mean) << " " << std::round(b_mean);
    newFile.close();

    // pcl::visualization::PCLVisualizer *viewer = new pcl::visualization::PCLVisualizer("3D Viewer");
    // viewer->setBackgroundColor(0, 0, 0);s
    // viewer->addPointCloud<pcl::PointXYZRGBA>(cloud_object);
    // viewer->addCoordinateSystem(1.0);
    // viewer->initCameraParameters();

    // while (!viewer->wasStopped())
    // {
    //     viewer->spinOnce(100);
    //     boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    // }
}

int CloudInitializer::calculateCloudVolume(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud)
{

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr copy_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::copyPointCloud(*cloud, *copy_cloud);
    pcl::VoxelGrid<pcl::PointXYZRGBA> vg;
    vg.setInputCloud(copy_cloud);
    vg.setLeafSize(0.01f, 0.01f, 0.01f);
    vg.filter(*copy_cloud);

    return copy_cloud->size();
}

ObjectOnTable* findObjectsOnTopOfTable(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud)
{
    ObjectOnTable *objs = new ObjectOnTable();
    
    return objs;
}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr CloudInitializer::initializePointCloud(const char *filename, const char *table_cloud_name)
{
    // loading the clouds
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>(filename, *cloud) == -1)
    {
        PCL_ERROR("Couldn't read file. \n");
        return NULL;
    }

    CloudInitializer::CalculateTableSizeAndFilter(cloud, table_cloud_name);
    return cloud;
}
