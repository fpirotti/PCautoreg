//
// Created by pirotti on 17/04/23.
//

#include "keypoints2features.h"
int  keypoints2features2(std::string &filename,
                        float support_size
) {
    std::cout << "hello  there" << std::endl;
    return(0);
}
int  keypoints2features(std::string filename,
                        float support_size,
                        pcl::IndicesPtr  cloud_keypoints_indices ,
                        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_max,
                        pcl::PointCloud<pcl::Normal>::Ptr cloud_normal,
                        pcl::PointCloud<pcl::FPFHSignature33>::Ptr  FPFH_signature33,
                        pcl::PointCloud<pcl::Histogram<153> >::Ptr  SPIN_signature
) {

    auto t1 = std::chrono::high_resolution_clock::now();
    auto t2 = std::chrono::high_resolution_clock::now();

    if(cloud_max->size()==0){
        pcl::console::print_error("Max grid not set, did you provide the max grid cloud?\n");
        return(-1);
    }
    if(cloud_normal->size()==0){
        pcl::console::print_error("Normals not set, did you provide the normals cloud?\n");
        return(-1);
    }
    if(cloud_normal->size()!=cloud_max->size()){
        pcl::console::print_error("Normals and max grid cloud do not have the same size (%d points and %d points respectively)\n",
                                  cloud_normal->size(), cloud_max->size() );
        return(-1);
    }
    if(cloud_keypoints_indices == nullptr || cloud_keypoints_indices->size()==0){
        pcl::console::print_error("No keypoints provided, will return ALL points (%d)\n", cloud_max->size() );
    }
////////////////////////////////////
/// ok now we calculate FPFH features ///
////////////////////////////////////

    std::cout << "Getting FPFH FEATURES ..." << std::endl;
    t1 = std::chrono::high_resolution_clock::now();

    if(FPFH_signature33== nullptr){
        pcl::console::print_warn("FPFH descriptor vector is null, skipping this descriptor\n");
        return(0);
    }

    pcl::search::KdTree<pcl::PointXYZI>::Ptr ktree(new pcl::search::KdTree<pcl::PointXYZI> ());
    pcl::FPFHEstimation<pcl::PointXYZI, pcl::Normal, pcl::FPFHSignature33> fpfh;
    fpfh.setInputCloud(cloud_max );
    fpfh.setInputNormals(cloud_normal);
    fpfh.setSearchMethod(ktree);
    fpfh.setRadiusSearch(support_size);


    if(cloud_keypoints_indices == nullptr || cloud_keypoints_indices->size()==0){
        pcl::console::print_error("No keypoints provided, will return ALL points (%d)\n", cloud_max->size() );
    } else {
        fpfh.setIndices( cloud_keypoints_indices);
    }

    fpfh.compute(* FPFH_signature33);
    t2 = std::chrono::high_resolution_clock::now();

    pcl::console::print_highlight( "Finished calculating  %d FPFH FEATURES   ... in %d seconds \n" ,
    FPFH_signature33->size()  ,
               std::chrono::duration_cast<std::chrono::seconds>(t2 - t1).count());





////////////////////////////////////
/// ok now we calculate SPIN features ///
////////////////////////////////////

    std::cout << "Getting SPIN FEATURES ..." << std::endl;
    t1 = std::chrono::high_resolution_clock::now();

    if(FPFH_signature33== nullptr){
        pcl::console::print_warn("FPFH descriptor vector is null, skipping this descriptor\n");
        return(0);
    }


    typedef pcl::Histogram<153> SpinImage;

    pcl::SpinImageEstimation<pcl::PointXYZI, pcl::Normal, SpinImage> si;
    si.setInputCloud(cloud_max);
    si.setInputNormals(cloud_normal);
    // Radius of the support cylinder.
    si.setRadiusSearch(support_size);
    // Set the resolution of the spin image (the number of bins along one dimension).
    // Note: you must change the output histogram size to reflect this.
    si.setImageWidth(8);
    //pcl::IndicesPtr ptr(cloud_keypoints_indices);

    if(cloud_keypoints_indices == nullptr || cloud_keypoints_indices->size()==0){
        pcl::console::print_error("No keypoints provided, will return ALL points (%d)\n", cloud_max->size() );
    } else {
        si.setIndices( cloud_keypoints_indices);
    }

    si.compute(*SPIN_signature);
    t2 = std::chrono::high_resolution_clock::now();

    pcl::console::print_highlight( "Finished calculating  %d SPIN FEATURES   ... in %d seconds \n" ,
                                   FPFH_signature33->size()  ,
                                   std::chrono::duration_cast<std::chrono::seconds>(t2 - t1).count());



    return(0);
}