
#include "readwrite.h"
#include "keypoints2features.h"
#include "alignRANSAC.h"

#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/console/parse.h>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include "common.h"

// #define  PCL_ALWAYS_(...){  \                                   \
// appendLineToFile(logfile, ... );                              \
// }                                                              
// --------------------
// -----Parameters-----
// --------------------

bool setUnseenToMaxRange = false;
float angular_resolution = 0.5f;
float support_size = 5.f;
float range_image_grid_res = .5f; // x10 less of support size this can be changed by -s

 
string stringpath = dateTimeToString(now(), "%Y-%m-%d-%X");

std::string logfile = dateTimeToString(now(), "%Y-%m-%d-%X-LOG.txt");
int status = mkdir(stringpath.c_str(),0777);

void printUsage(const char *progName) {
    std::cout << "\n\nUsage: " << progName << " [options] <reference.ply|las|laz> <registered1.ply|las|laz> <registeredN.ply|las|laz>\n\n"
              << "Options:\n"
              << "-------------------------------------------\n"
            //  << "-r <float>   angular resolution in degrees (default " << angular_resolution << ")\n" 
              << "-g <float>   grid sampling distance for max points (default " << range_image_grid_res << ")\n" 
              << "-s <float>   support size for the interest points (diameter of the used sphere - NB the range image will be 10x less this diameter. Suggested to set it in a way to make sure to catch at least 100 points"
              << "default " << support_size << ")\n"
              << "-h           this help\n"
              << "\n\n";
}

int run(int argc, char **argv ) //ss is the x of higher resolution of range image to radius - should always be >> 1
{

    
    //float max_window_res= support_size/10;
    appendLineToFile(logfile, string_format("============ %.2f window size... and support size %.3f\n",
                                       range_image_grid_res, support_size).c_str() );

    //pcl::PointCloud<pcl::Narf36>::Ptr narf_descriptor_ptr_master (new pcl::PointCloud<pcl::Narf36>);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_descriptor_ptr_master (new pcl::PointCloud< pcl::FPFHSignature33 >);
    pcl::PointCloud< pcl::Histogram<153>  >::Ptr spin_descriptor_ptr_master (new pcl::PointCloud< pcl::Histogram<153> >);

    std::map<std::string,  pcl::IndicesPtr  > keypoints; // = *keypointsPtr;

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_max(new pcl::PointCloud<pcl::PointXYZINormal>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_keypointsTmp(new pcl::PointCloud<pcl::PointXYZINormal>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_keypoints(new pcl::PointCloud<pcl::PointXYZ>);

    std::vector<float> xcheck{120.2, 337.26, 85.566, 308.15};
    std::vector<float> ycheck{67.3,   41.5, 169.85,  141.71};



    for(int i=1; i< argc; i++) {

        pcl::console::print_warn("  %d of %d clouds\n", i,  argc-1);

        std::string filename(argv[i]); 
        
       
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZINormal>);
        if(readPC(filename, cloud_in) < 0 ){
            pcl::console::print_error("Error reading %s image\n", filename.c_str());
            return -1;
        }

        if(i == 1){

            if( getMaxImageWithNormals(filename, cloud_in, cloud_max) < 0 ){
                pcl::console::print_error("Error reading max grid %s image\n", filename.c_str());
                return(-1);
            }
            
            appendLineToFile(logfile, string_format("Calculating Keypoints at REFERENCE object: %s.\n",filename.c_str() )  );
            
            pcl::console::print_warn("Calculating Keypoints at REFERENCE object.\n");
            las2keypoints(filename,  cloud_max,   keypoints, support_size= support_size);


            for (const auto& [key, value] : keypoints)
                std::cout << '[' << key << "] = " << value->size() << "; " << std::endl;


        } else {

            pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_max_slave(new pcl::PointCloud<pcl::PointXYZINormal>);
            pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_slave(new pcl::PointCloud<pcl::Normal>);
            pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_keypointsTmp_slave(new pcl::PointCloud<pcl::PointXYZINormal>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_keypoints_slave(new pcl::PointCloud<pcl::PointXYZ>);

            pcl::console::print_warn("  reading %s image\n", filename.c_str() );
            if( getMaxImageWithNormals(filename, cloud_in, cloud_max_slave ) < 0 ) {
                pcl::console::print_error("Error reading max grid %s image\n", filename.c_str());
                return(-1);
            }

            pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_descriptor_ptr_slave (new pcl::PointCloud<pcl::FPFHSignature33>);
            pcl::PointCloud< pcl::Histogram<153>  >::Ptr spin_descriptor_ptr_slave (new pcl::PointCloud<pcl::Histogram<153> >);

            
            appendLineToFile(logfile, string_format("Calculating Keypoints at REGISTERED object. %s.\n",filename.c_str() )  );
            
            pcl::console::print_warn("Calculating Keypoints at REGISTERED object.\n");
            std::map<std::string,  pcl::IndicesPtr  > keypoints_slave; // = *keypointsPtr;

            las2keypoints(filename,  cloud_max_slave, 
                          keypoints_slave,  support_size= support_size);


            for (const auto& [key, value] : keypoints_slave){

                std::cout << '[' << key << "] = " << value->size() << "; " << std::endl;

              
                /////////////// REFERENCE
                ////// copy normals alas they have to be in a specific PCL type
                pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>); 
                pcl::copyPointCloud(*cloud_max, *cloud_normals); 
                
                std::cout << "REFERENCE " << std::endl;
                std::string kk = std::string(key);
                kk.append("_REFERENCE");
                keypoints2features(kk, support_size,
                                   keypoints[ key.c_str() ],
                                   cloud_max, cloud_normals,
                                   fpfh_descriptor_ptr_master,
                                   spin_descriptor_ptr_master );

              
                pcl::ExtractIndices<pcl::PointXYZINormal> filter;
                filter.setInputCloud (cloud_max);
                filter.setIndices ( keypoints[ key.c_str() ] );
                // Extract the points in cloud_in referenced by indices_in as a separate point cloud:
                filter.filter (*cloud_keypointsTmp);
                pcl::copyPointCloud(*cloud_keypointsTmp, *cloud_keypoints);

                /////////////// REGISTERED
                
                ////// copy normals alas they have to be in a specific PCL type
                pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_slave (new pcl::PointCloud<pcl::Normal>); 
                pcl::copyPointCloud(*cloud_max_slave, *cloud_normals_slave); 
                
                
                std::cout << "REGISTERED " << std::endl;
                std::string kk2 = std::string(key);
                kk2.append("_REGISTERED");
                keypoints2features(kk2, support_size,
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



                appendLineToFile(logfile, string_format("RANSAC with %s ....\n%d|%d cloud keypoints (master|slave),  ...\n%d|%d features FPFH \n%d|%d features SPIN\n",
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
                
                ransac.setOutName(  pcl::getFilenameWithoutExtension(filename).append("_").append(key).append("_ss").append( std::to_string((int)support_size) ) );
                
               // ransac.align_FPFH( fpfh_descriptor_ptr_master, fpfh_descriptor_ptr_slave);
                ransac.align_SPIN( spin_descriptor_ptr_master, spin_descriptor_ptr_slave);
               // ransac.checkAccuracy(xcheck, ycheck);
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
    int j=0;
    if (pcl::console::parse (argc, argv, "-s", support_size) >= 0){ 
      std::cout << "Setting support size to "<<support_size<<".\n";
      j++; 
      j++; // jump two of input arguments
    } else {
      std::cout << "Support size to default "<< support_size <<".\n";
    } 
    
    if (pcl::console::parse (argc, argv, "-g", range_image_grid_res) >= 0){ 
      std::cout << "Setting range_image_grid_res to "<<range_image_grid_res<<".\n";
      j++; 
      j++; // jump two of input arguments
    } else {
      std::cout << "range_image_grid_res to default "<< range_image_grid_res <<".\n";
    } 

    argv+=j;
    argc-=j;
    appendLineToFile(logfile, string_format("STARTING \nrange_image_grid_res= %.3f\n support_size= %.3f\n expected points= %.3f\n", 
                                      support_size, 
                                      range_image_grid_res, 
                                      pow((support_size/range_image_grid_res),2) ), true);

    run(argc, argv);
    //run(argc, argv, 15);


}