
#include "readwrite.h"

#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/console/parse.h>

// --------------------
// -----Parameters-----
// --------------------
float angular_resolution = 0.5f;
float support_size = 0.2f;

bool setUnseenToMaxRange = false;

void printUsage(const char *progName) {
    std::cout << "\n\nUsage: " << progName << " [options] <scene.ply|las|laz>\n\n"
              << "Options:\n"
              << "-------------------------------------------\n"
              << "-r <float>   angular resolution in degrees (default " << angular_resolution << ")\n"
              << "-m           Treat all unseen points as maximum range readings\n"
              << "-s <float>   support size for the interest points (diameter of the used sphere - "
              << "default " << support_size << ")\n"
              << "-h           this help\n"
              << "\n\n";
}

int
main(int argc, char **argv) {

    if (argc < 2) {
        printUsage(argv[0]);
        return 0;
    }
    if (pcl::console::find_argument(argc, argv, "-h") >= 0) {
        printUsage(argv[0]);
        return 0;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    //pcl::PointCloud<pcl::PointXYZ> cloud;
    std::string filename = argv[1];
    las2pcd<pcl::PointXYZ>(filename, cloud, true);
    //readPC<pcl::PointXYZ>(filename, *cloud);
    std::cout << "points: " << cloud->size() << std::endl;


/*
  // Visualization of keypoints along with the original cloud
  pcl::visualization::PCLVisualizer viewer("PCL Viewer");
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler (cloud_temp, 0, 255, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> cloud_color_handler (cloud, 255, 255, 0);
  viewer.setBackgroundColor( 0.0, 0.0, 0.0 );
  viewer.addPointCloud(cloud, "cloud");
  viewer.addPointCloud(cloud_temp, keypoints_color_handler, "keypoints");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints");

  while(!viewer.wasStopped ())
  {
  viewer.spinOnce ();
  }
*/


    return 0;

}
