//
// Created by pirotti on 07/04/23.
//

#ifndef COFIT_READWRITE_H
#define COFIT_READWRITE_H

#include "lasreader.hpp"
#include "laswriter.hpp"
#include <iostream>
#include <filesystem>
#include <chrono>
#include <pcl/memory.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/common/common.h>
#include <pcl/common/file_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/pcl_macros.h>
#include <pcl/io/ply/ply_parser.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include "pcl/io/ply_io.h"
#include <pcl/range_image/range_image.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/narf.h>
#include <pcl/features/narf_descriptor.h>
#include <pcl/features/spin_image.h>

#include <pcl/filters/grid_minimum.h>

#include "grid_maximum.h"

//#include <pcl/features/range_image_border_extractor.h>

#include <pcl/filters/passthrough.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/range_image/range_image.h>
#include <pcl/io/pcd_io.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/susan.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/keypoints/trajkovic_3d.h>
#include <pcl/keypoints/trajkovic_2d.h>
#include <pcl/console/parse.h>
#include <pcl/common/file_io.h>
#define PBSTR "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||"
#define PBWIDTH 60



inline void printProgress(double percentage) {
    float val = (percentage * 100);
    int lpad = (int) (percentage * PBWIDTH);
    int rpad = PBWIDTH - lpad;
   // printf("%3.1f%%", val);
    printf("\r%3.1f%% [%.*s%*s]", val, lpad, PBSTR, rpad, "");
    fflush(stdout);
}

int readPC(std::string filename, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
        bool calcNormals = true, bool forceCalcNormals = false, bool savetopcd=true);

int getMaxImageWithNormals(std::string filename,
                           pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in,
                           pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out,
                           pcl::PointCloud<pcl::Normal>::Ptr cloud_norm);

template<typename PointT> inline int las2keypoints(  std::string filename,
                                                    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_las_gridMax,
                                                    pcl::PointCloud<pcl::Normal>::Ptr cloud_Normals_ptr,
                                                    pcl::PointCloud<pcl::PointXYZ>::Ptr sift_keypointsCloud_ptr= nullptr,
                                                    pcl::PointCloud<pcl::PointXYZ>::Ptr susan_keypointsCloud_ptr= nullptr,
                                                   pcl::PointCloud<pcl::PointXYZ>::Ptr ISS_keypointsCloud_ptr= nullptr,
                                                   pcl::PointCloud<pcl::PointXYZ>::Ptr Harris_keypointsCloud_ptr= nullptr,
                                                   pcl::PointCloud<pcl::PointXYZ>::Ptr Trajkovic_keypointsCloud_ptr= nullptr,
                                                   float support_size=5.0f ) {




    auto t1 = std::chrono::high_resolution_clock::now();
    auto t2 = std::chrono::high_resolution_clock::now();

    /// --------------------------------
    /// -----Extract SIFT keypoints-----
    /// --------------------------------

    pcl::console::print_warn( "Getting SIFT KEYPOINTS...\n " );
    t1 = std::chrono::high_resolution_clock::now();

    constexpr float min_scale = 10.0f;
    constexpr int n_octaves = 6;
    constexpr int n_scales_per_octave = 10;
    constexpr float min_contrast = 5.0f;
    pcl::SIFTKeypoint <pcl::PointXYZI, pcl::PointWithScale> sift;
    pcl::PointCloud<pcl::PointWithScale> result;
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI> ());
    sift.setSearchMethod(tree);

    sift.setScales(min_scale, n_octaves, n_scales_per_octave);
    sift.setMinimumContrast(min_contrast);
    sift.setInputCloud(cloud_las_gridMax);
    sift.compute(result);
    pcl::copyPointCloud(result, *sift_keypointsCloud_ptr);
    pcl::io::savePCDFile (pcl::getFilenameWithoutExtension(filename).append( "__SIFT.pcd" ),
                          result, false);

    t2 = std::chrono::high_resolution_clock::now();
    PCL_WARN  (" --- Finished calculating %d SIFT KEYPOINTS  after %d seconds \n",
               result.size (),
            std::chrono::duration_cast<std::chrono::seconds>(t2 - t1).count() );
    t1 = std::chrono::high_resolution_clock::now();

    /// --------------------------------
    /// -----Extract HARRIS keypoints-----
    /// --------------------------------

    pcl::console::print_warn( "Getting harris KEYPOINTS...\n " );
    t1 = std::chrono::high_resolution_clock::now();

    pcl::PointCloud<pcl::PointXYZI>::Ptr pp (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::HarrisKeypoint3D<pcl::PointXYZI,  pcl::PointXYZI,   pcl::Normal> harris;
    pcl::search::KdTree<pcl::PointXYZI>::Ptr harris_tree(new pcl::search::KdTree<pcl::PointXYZI> ());
    harris.setSearchMethod(harris_tree);
    harris.setRadius(5);
    harris.setNormals(cloud_Normals_ptr);
    harris.setInputCloud(cloud_las_gridMax);
    harris.compute(*pp);
    pcl::copyPointCloud(*pp, *Harris_keypointsCloud_ptr);
    pcl::io::savePCDFile (pcl::getFilenameWithoutExtension(filename).append( "__HARRIS.pcd" ),
                          *pp, false);

    t2 = std::chrono::high_resolution_clock::now();
    PCL_WARN  (" --- Finished calculating %d harris KEYPOINTS  after %d seconds \n",
               pp->size (),
               std::chrono::duration_cast<std::chrono::seconds>(t2 - t1).count() );
    t1 = std::chrono::high_resolution_clock::now();


    /// --------------------------------
    /// -----Extract ISS keypoints-----
    /// --------------------------------

    pcl::console::print_warn( "Getting ISS KEYPOINTS...\n " );
    t1 = std::chrono::high_resolution_clock::now();

   // pcl::PointCloud<pcl::PointXYZ>::Ptr pp_iss (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ISSKeypoint3D<pcl::PointXYZI, pcl::PointXYZ> iss;
    pcl::search::KdTree<pcl::PointXYZI>::Ptr iss_tree(new pcl::search::KdTree<pcl::PointXYZI> ());
    iss.setSearchMethod(iss_tree);
    iss.setSalientRadius(support_size/2);
    iss.setNonMaxRadius(1);
    iss.setNormals(cloud_Normals_ptr);
    iss.setInputCloud(cloud_las_gridMax);
    iss.compute(*ISS_keypointsCloud_ptr);
    //pcl::copyPointCloud(*pp, *ISS_keypointsCloud_ptr);
    pcl::io::savePCDFile (pcl::getFilenameWithoutExtension(filename).append( "__ISS.pcd" ),
                          *ISS_keypointsCloud_ptr, false);

    t2 = std::chrono::high_resolution_clock::now();
    PCL_WARN  (" --- Finished calculating %d ISS KEYPOINTS  after %d seconds \n",
               ISS_keypointsCloud_ptr->size (),
               std::chrono::duration_cast<std::chrono::seconds>(t2 - t1).count() );
    t1 = std::chrono::high_resolution_clock::now();

    return(0);

}

template<typename PointT> inline int  keypoints2features(std::string filename,
                                                   pcl::PointCloud<pcl::PointXYZ>::Ptr keypointsCloud_ptr= nullptr,
                                                   pcl::PointCloud<pcl::FPFHSignature33>::Ptr  FPFH_signature33= nullptr,
                                                   pcl::PointCloud<pcl::Histogram<153> >::Ptr  SPIN_signature = nullptr,
                                                         float support_size=5.0f ) {


    auto t1 = std::chrono::high_resolution_clock::now();
    auto t2 = std::chrono::high_resolution_clock::now();

    ////////////////////////////////////
    /// ok now we calculate FPFH features ///
    ////////////////////////////////////

//    std::cout << "Getting FPFH FEATURES ..." << std::endl;
//    t1 = std::chrono::high_resolution_clock::now();
//
//    if(fpfh_descriptor_ptr== nullptr){
//        pcl::console::print_warn("\nFPFH descriptor vector is null, skipping this descriptor\n");
//        return(0);
//    }
//
//    pcl::search::KdTree<pcl::PointXYZINormal>::Ptr ktree(new pcl::search::KdTree<pcl::PointXYZINormal> ());
//    pcl::FPFHEstimation<pcl::PointXYZINormal, pcl::PointXYZINormal, pcl::FPFHSignature33> fpfh;
//    fpfh.setInputCloud(cloud_wNormals_ptr );
//    fpfh.setInputNormals(cloud_wNormals_ptr);
//    fpfh.setSearchMethod(ktree);
//    fpfh.setRadiusSearch(support_size);
//    //fpfh.setIndices( keypoint_indices_intVector_Ptr);
//
//    fpfh.compute(* sift_descriptor_ptr);
//    t2 = std::chrono::high_resolution_clock::now();
//
//    std::cout << "Finished calculating " <<  fpfh_descriptor_ptr->size() << " FPFH FEATURES  ... final size= " <<  fpfh_descriptor_ptr->points.size() << " after "
//              << std::chrono::duration_cast<std::chrono::seconds>(t2 - t1).count() << " seconds. " << std::endl;



    t2 = std::chrono::high_resolution_clock::now();
    std::cout << "Finished creating range images after "
              << std::chrono::duration_cast<std::chrono::seconds>(t2 - t1).count() << " seconds. " << std::endl;


    return 0;
}

#endif //COFIT_READWRITE_H
