
#include "readwrite.h"

#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/console/parse.h>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>

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

    pcl::PointCloud<pcl::Narf36>::Ptr narf_descriptor_ptr_master (new pcl::PointCloud<pcl::Narf36>);
    pcl::PointCloud<pcl::Narf36>::Ptr sift_descriptor_ptr_master (new pcl::PointCloud<pcl::Narf36>);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_descriptor_ptr_master (new pcl::PointCloud<pcl::FPFHSignature33>);
    pcl::PointCloud< pcl::Histogram<153>  >::Ptr spin_descriptor_ptr_master (new pcl::PointCloud<pcl::Histogram<153> >);

    pcl::PointCloud<pcl::PointXYZ>::Ptr sift_KEYPOINTS_cloud_master (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr susan_KEYPOINTS_cloud_master (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr iss_KEYPOINTS_cloud_master (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr harris_KEYPOINTS_cloud_master (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr trajkovic_KEYPOINTS_cloud_master (new pcl::PointCloud<pcl::PointXYZ>);

    std::map<std::string,  pcl::PointCloud<pcl::PointXYZ>::Ptr > keypoints;
    keypoints["SIFT"] = sift_KEYPOINTS_cloud_master;
    keypoints["ISS"] = iss_KEYPOINTS_cloud_master;
    keypoints["HARRIS"] = harris_KEYPOINTS_cloud_master;
    keypoints["trajkovic"] = trajkovic_KEYPOINTS_cloud_master;
    keypoints["SUSAN"] = susan_KEYPOINTS_cloud_master;

    //pcl::PointCloud<pcl::PointXYZ> cloud;
    for(int i =1; i< argc; i++) {
        std::string filename = argv[i];
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZI>);
        if(readPC(filename, cloud_in) < 0 ){
            pcl::console::print_error("Error reading %s image\n", filename.c_str());
            return(-1);
        };
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_max(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
        pcl::console::print_warn("  reading %s image\n", filename.c_str());
        if( getMaxImageWithNormals(filename, cloud_in, cloud_max, cloud_normals) < 0 ){
            pcl::console::print_error("Error reading max grid %s image\n", filename.c_str());
            return(-1);
        };

        if(i==1){




            pcl::console::print_warn("Calculating Keypoints at master object.\n");
            las2keypoints<pcl::PointXYZ>(filename,  cloud_max, cloud_normals,
                                         sift_KEYPOINTS_cloud_master,
                                         susan_KEYPOINTS_cloud_master,
                                         iss_KEYPOINTS_cloud_master,
                                         harris_KEYPOINTS_cloud_master,
                                         trajkovic_KEYPOINTS_cloud_master,
                                         support_size= 15.0f);
            for (const auto& [key, value] : keypoints)
                std::cout << '[' << key << "] = " << value->size() << "; " << std::endl;


            return(0);
        } else {
//
//            pcl::PointCloud<pcl::Narf36>::Ptr  narf_descriptor_ptr_slave (new pcl::PointCloud<pcl::Narf36>);
//            pcl::PointCloud<pcl::Narf36>::Ptr  sift_descriptor_ptr_slave (new pcl::PointCloud<pcl::Narf36>);
//            pcl::PointCloud<pcl::FPFHSignature33>::Ptr  fpfh_descriptor_ptr_slave (new pcl::PointCloud<pcl::FPFHSignature33>);
//
//            pcl::PointCloud<pcl::PointXYZ>::Ptr narf_KEYPOINTS_cloud_slave (new pcl::PointCloud<pcl::PointXYZ>);
//            pcl::PointCloud<pcl::PointXYZ>::Ptr sift_KEYPOINTS_cloud_slave (new pcl::PointCloud<pcl::PointXYZ>);
//            pcl::PointCloud<pcl::PointXYZ>::Ptr susan_KEYPOINTS_cloud_slave (new pcl::PointCloud<pcl::PointXYZ>);
//
//            las2keypoints<pcl::PointXYZ>(filename, true,
//                                         narf_KEYPOINTS_cloud_slave,
//                                         sift_KEYPOINTS_cloud_slave,
//                                         susan_KEYPOINTS_cloud_slave,
//                                         narf_descriptor_ptr_slave,
//                                         sift_descriptor_ptr_slave,
//                                         fpfh_descriptor_ptr_slave, 10.0f);
//            // Perform alignment
//            pcl::PointCloud<pcl::PointXYZ>::Ptr object_aligned (new pcl::PointCloud<pcl::PointXYZ>);
//            pcl::console::print_highlight ("Starting alignment...\n");
//            pcl::SampleConsensusPrerejective<pcl::PointXYZ ,pcl::PointXYZ ,pcl::FPFHSignature33> align;
//            align.setInputSource (narf_KEYPOINTS_cloud_slave);
//            align.setSourceFeatures ( fpfh_descriptor_ptr_master);
//            align.setInputTarget (narf_KEYPOINTS_cloud_master);
//            align.setTargetFeatures ( fpfh_descriptor_ptr_master);
//            align.setMaximumIterations (50000); // Number of RANSAC iterations
//            align.setNumberOfSamples (3); // Number of points to sample for generating/prerejecting a pose
//            align.setCorrespondenceRandomness (5); // Number of nearest features to use
//            align.setSimilarityThreshold (0.9f); // Polygonal edge length similarity threshold
//            align.setMaxCorrespondenceDistance (2.5f * 0.5); // Inlier threshold
//            align.setInlierFraction (0.25f); // Required inlier fraction for accepting a pose hypothesis
//            {
//                pcl::ScopeTime t("Alignment");
//                align.align (*object_aligned);
//            }
//
//            if (align.hasConverged ())
//            {
//                // Print results
//                printf ("\n");
//                Eigen::Matrix4f transformation = align.getFinalTransformation ();
//                pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (0,0), transformation (0,1), transformation (0,2));
//                pcl::console::print_info ("R = | %6.3f %6.3f %6.3f | \n", transformation (1,0), transformation (1,1), transformation (1,2));
//                pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (2,0), transformation (2,1), transformation (2,2));
//                pcl::console::print_info ("\n");
//                pcl::console::print_info ("t = < %0.3f, %0.3f, %0.3f >\n", transformation (0,3), transformation (1,3), transformation (2,3));
//                pcl::console::print_info ("\n");
//                pcl::console::print_info ("Inliers: %i/%i\n", align.getInliers ().size (), narf_KEYPOINTS_cloud_master->size ());
//
//            }
//            else
//            {
//                pcl::console::print_error ("Alignment failed!\n");
//                return (1);
//            }

        }
    }



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
