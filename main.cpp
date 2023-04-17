
#include "readwrite.h"
#include "keypoints2features.h"

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

    std::map<std::string,  pcl::PointIndicesConstPtr  > *keypointsPtr = new std::map<std::string,  pcl::PointIndicesConstPtr  >;
    std::map<std::string,  pcl::IndicesPtr  > keypoints; // = *keypointsPtr;

    float support_size= 10.0f;
    for(int i=1; i< argc; i++) {

        std::string filename = argv[i];
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZI>);
        if(readPC(filename, cloud_in) < 0 ){
            pcl::console::print_error("Error reading %s image\n", filename.c_str());
            return -1;
        }

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_max(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_keypointsTmp(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_keypoints(new pcl::PointCloud<pcl::PointXYZ>);

        pcl::console::print_warn("  %d of %d\n", i,  argc);
        if(i==1){

            pcl::console::print_warn("  reading %s image\n", filename.c_str());
            if( getMaxImageWithNormals(filename, cloud_in, cloud_max, cloud_normals) < 0 ){
                pcl::console::print_error("Error reading max grid %s image\n", filename.c_str());
                return(-1);
            }

            pcl::console::print_warn("Calculating Keypoints at master object.\n");
            las2keypoints(filename,  cloud_max, cloud_normals, keypoints, support_size= support_size);


            for (const auto& [key, value] : keypoints)
                std::cout << '[' << key << "] = " << value->size() << "; " << std::endl;

            keypoints2features(filename, support_size,
                               keypoints["ISS"],
                               cloud_max, cloud_normals,
                               fpfh_descriptor_ptr_master,
                               spin_descriptor_ptr_master );

            pcl::ExtractIndices<pcl::PointXYZI> filter;
           filter.setInputCloud (cloud_max);
            filter.setIndices ( keypoints["ISS"]);
            // Extract the points in cloud_in referenced by indices_in as a separate point cloud:
            filter.filter (*cloud_keypointsTmp);
            pcl::copyPointCloud(*cloud_keypointsTmp, *cloud_keypoints);
            std::cout << "hello  " << std::endl;

        } else {
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_max_slave(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_slave(new pcl::PointCloud<pcl::Normal>);
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_keypointsTmp_slave(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_keypoints_slave(new pcl::PointCloud<pcl::PointXYZ>);


            pcl::console::print_warn("  reading %s image\n", filename.c_str());
            if( getMaxImageWithNormals(filename, cloud_in, cloud_max_slave, cloud_normals_slave) < 0 ){
                pcl::console::print_error("Error reading max grid %s image\n", filename.c_str());
                return(-1);
            }

            pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_descriptor_ptr_slave (new pcl::PointCloud<pcl::FPFHSignature33>);
            pcl::PointCloud< pcl::Histogram<153>  >::Ptr spin_descriptor_ptr_slave (new pcl::PointCloud<pcl::Histogram<153> >);

            pcl::console::print_warn("Calculating Keypoints at slave object.\n");
            std::map<std::string,  pcl::IndicesPtr  > keypoints_slave; // = *keypointsPtr;

            las2keypoints(filename,  cloud_max_slave, cloud_normals_slave,
                          keypoints_slave,
                          support_size= support_size);

            for (const auto& [key, value] : keypoints_slave)
                std::cout << '[' << key << "] = " << value->size() << "; " << std::endl;



            keypoints2features(filename, support_size,
                               keypoints["ISS"],
                               cloud_max_slave, cloud_normals_slave,
                               fpfh_descriptor_ptr_slave,
                               spin_descriptor_ptr_slave  );

            pcl::ExtractIndices<pcl::PointXYZI> filter;
            filter.setInputCloud (cloud_max_slave);
//            filter.setIndices (static_cast<const pcl::IndicesConstPtr>(&keypoints_slave["ISS"]));
//            // Extract the points in cloud_in referenced by indices_in as a separate point cloud:
//            filter.filter (*cloud_keypointsTmp_slave);
//            pcl::copyPointCloud(*cloud_keypointsTmp_slave, *cloud_keypoints_slave);

        }

    }


    return 0;

}
