#include <iostream>
#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>
#include <pcl/common/eigen.h>
#include <boost/thread/thread.hpp>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/geometry.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <iostream>
#include <fstream>

struct ObjectOnTable {
  int colors[3];
  int height;
  int width;
  int volume;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;
};

class CloudInitializer
{

private:
    int searchForZClosestPoint(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);
    int searchForZFarestPoint(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);
    int searchForXFarestNegativePoint(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);
    int searchForXFarestPositivePoint(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);
    int searchForYHighestPoint(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);
    int searchForYLowestPoint(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);
    //CALUCLATES TABLE SIZE AND FILTERS IT AND WRITES THAT CLOUD
    void CalculateTableSizeAndFilter(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, const char *table_cloud_name);
    int calculateCloudVolume(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);

    ObjectOnTable* findObjectsOnTopOfTable(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);

public:
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr initializePointCloud(const char *filename, const char *table_cloud_name);
    void FindObjectAndCreateFile(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, const char *filename);
};