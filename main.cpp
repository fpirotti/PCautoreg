
#include "readwrite.h"
#include "keypoints2features.h"
#include "alignRANSAC.h"

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

int run(int argc, char **argv, float support_size= 5.0f, float max_window_res= .2f)
{


    pcl::console::print_highlight (string_format("============ %.2f window size...\n",
                                                 support_size).c_str() );
    appendLineToFile("log.txt", string_format("============ %.2f window size...\n",
                                              support_size).c_str() );

    //pcl::PointCloud<pcl::Narf36>::Ptr narf_descriptor_ptr_master (new pcl::PointCloud<pcl::Narf36>);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_descriptor_ptr_master (new pcl::PointCloud< pcl::FPFHSignature33 >);
    pcl::PointCloud< pcl::Histogram<153>  >::Ptr spin_descriptor_ptr_master (new pcl::PointCloud< pcl::Histogram<153> >);

    std::map<std::string,  pcl::IndicesPtr  > keypoints; // = *keypointsPtr;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_max(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_keypointsTmp(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_keypoints(new pcl::PointCloud<pcl::PointXYZ>);

    std::vector<float> xcheck{120.2, 337.26, 85.566, 308.15};
    std::vector<float> ycheck{67.3,   41.5, 169.85,  141.71};



    for(int i=1; i< argc; i++) {

        pcl::console::print_warn("  %d of %d clouds\n", i,  argc-1);

        std::string filename = argv[i];
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZI>);
        if(readPC(filename, cloud_in) < 0 ){
            pcl::console::print_error("Error reading %s image\n", filename.c_str());
            return -1;
        }

        if(i == 1){

            if( getMaxImageWithNormals(filename, cloud_in, cloud_max, cloud_normals, max_window_res) < 0 ){
                pcl::console::print_error("Error reading max grid %s image\n", filename.c_str());
                return(-1);
            }

            pcl::console::print_warn("Calculating Keypoints at REFERENCE object.\n");
            las2keypoints(filename,  cloud_max, cloud_normals, keypoints, support_size= support_size);


            for (const auto& [key, value] : keypoints)
                std::cout << '[' << key << "] = " << value->size() << "; " << std::endl;


        } else {

            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_max_slave(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_slave(new pcl::PointCloud<pcl::Normal>);
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_keypointsTmp_slave(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_keypoints_slave(new pcl::PointCloud<pcl::PointXYZ>);

            pcl::console::print_warn("  reading %s image\n", filename.c_str() );
            if( getMaxImageWithNormals(filename, cloud_in, cloud_max_slave, cloud_normals_slave, max_window_res) < 0 ) {
                pcl::console::print_error("Error reading max grid %s image\n", filename.c_str());
                return(-1);
            }

            pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_descriptor_ptr_slave (new pcl::PointCloud<pcl::FPFHSignature33>);
            pcl::PointCloud< pcl::Histogram<153>  >::Ptr spin_descriptor_ptr_slave (new pcl::PointCloud<pcl::Histogram<153> >);

            pcl::console::print_warn("Calculating Keypoints at REGISTERED object.\n");
            std::map<std::string,  pcl::IndicesPtr  > keypoints_slave; // = *keypointsPtr;

            las2keypoints(filename,  cloud_max_slave, cloud_normals_slave,
                          keypoints_slave,  support_size= support_size);


            for (const auto& [key, value] : keypoints_slave){

                std::cout << '[' << key << "] = " << value->size() << "; " << std::endl;

                /////////////// REFERENCE
                std::cout << "REFERENCE " << std::endl;
                keypoints2features(filename, support_size,
                                   keypoints[ key.c_str() ],
                                   cloud_max, cloud_normals,
                                   fpfh_descriptor_ptr_master,
                                   spin_descriptor_ptr_master );


                pcl::ExtractIndices<pcl::PointXYZI> filter;
                filter.setInputCloud (cloud_max);
                filter.setIndices ( keypoints[ key.c_str() ] );
                // Extract the points in cloud_in referenced by indices_in as a separate point cloud:
                filter.filter (*cloud_keypointsTmp);
                pcl::copyPointCloud(*cloud_keypointsTmp, *cloud_keypoints);

                /////////////// REGISTERED

                std::cout << "REGISTERED " << std::endl;
                keypoints2features(filename, support_size,
                                   value,
                                   cloud_max_slave, cloud_normals_slave,
                                   fpfh_descriptor_ptr_slave,
                                   spin_descriptor_ptr_slave  );

                filter.setInputCloud (cloud_max_slave);
                filter.setIndices (value);
                filter.filter (*cloud_keypointsTmp_slave);

                pcl::copyPointCloud(*cloud_keypointsTmp_slave, *cloud_keypoints_slave);
                //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_max_slave_xyz (new pcl::PointCloud<pcl::PointXYZ>);
                //pcl::copyPointCloud(*cloud_max_slave, *cloud_max_slave_xyz);



                appendLineToFile("log.txt", string_format("RANSAC with %s ....\n%d|%d cloud keypoints (master|slave),  ...\n%d|%d features FPFH \n%d|%d features SPIN\n",
                                                          key.c_str(),
                                                          cloud_keypoints->size(),
                                                          cloud_keypoints_slave->size(),
                                                          fpfh_descriptor_ptr_master->size() ,
                                                          fpfh_descriptor_ptr_slave->size() ,
                                                          spin_descriptor_ptr_master->size() ,
                                                          spin_descriptor_ptr_slave->size()  ) );

                alignRANSAC ransac;
                ransac.setMasterCloud(cloud_keypoints);
                ransac.setSlaveCloud(cloud_keypoints_slave);
                ransac.setCloud2Register( cloud_max_slave);
                char nm[90];
                sprintf(nm, "%02d_%s_",  (int)support_size, key.c_str() );
                ransac.setOutName( pcl::getFilenameWithoutExtension(filename).append( nm ) );
                ransac.align_FPFH( fpfh_descriptor_ptr_master, fpfh_descriptor_ptr_slave);
                ransac.align_SPIN( spin_descriptor_ptr_master, spin_descriptor_ptr_slave);
                ransac.checkAccuracy(xcheck, ycheck);
            }


        }

    }


    return 0;

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


    appendLineToFile("log.txt", "STARTING \n", true);

    run(argc, argv, 10);
    run(argc, argv, 15);


}