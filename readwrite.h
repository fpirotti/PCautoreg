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
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>

#include <pcl/filters/grid_minimum.h>

#include <pcl/features/narf.h>
#include <pcl/features/narf_descriptor.h>
#include <pcl/features/spin_image.h>
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
#include <pcl/keypoints/harris_6d.h>
#include <pcl/keypoints/trajkovic_3d.h>
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


template<typename PointT> inline int las2keypoints(std::string filename,
                                                   bool savetopcd= false,
                                                    pcl::PointCloud<pcl::PointXYZ>::Ptr sift_keypointsCloud_ptr= nullptr,
                                                    pcl::PointCloud<pcl::PointXYZ>::Ptr susan_keypointsCloud_ptr= nullptr,
                                                   pcl::PointCloud<pcl::PointXYZ>::Ptr ISS_keypointsCloud_ptr= nullptr,
                                                   pcl::PointCloud<pcl::PointXYZ>::Ptr Harris_keypointsCloud_ptr= nullptr,
                                                   pcl::PointCloud<pcl::PointXYZ>::Ptr Trajkovic_keypointsCloud_ptr= nullptr,
                                                   float support_size=5.0f ) {



    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZI>);
  //  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_normals(new pcl::PointCloud<pcl::PointXYZI>);
    if(readPC(filename, cloud_in) < 0 ){
        pcl::console::print_error("Error reading %s image\n", filename.c_str());
        return(-1);
    };

    auto t1 = std::chrono::high_resolution_clock::now();
    auto t2 = std::chrono::high_resolution_clock::now();

    pcl::PointXYZI  minPt, maxPt;

    pcl::getMinMax3D(*cloud_in, minPt, maxPt);

    double centerx = minPt.x + (maxPt.x - minPt.x)/2;
    double centery = minPt.y + (maxPt.y - minPt.y)/2;



    //////////////////////////////
    //////  TO IMAGE  ///////
    //////////////////////////////

    std::cout << "Getting RANGE image" << std::endl;
    t1 = std::chrono::high_resolution_clock::now();

    int gridn = 50;
    float stepx = (maxPt.x  - minPt.x)/(float)gridn;
    float stepy = (maxPt.y  - minPt.y)/(float)gridn;

    float sizex = maxPt.x - minPt.x;
    float sizey = maxPt.y - minPt.y;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_las_gridMax(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::GridMaximum<pcl::PointXYZI> maxFilter(1.0f); //(*cloud_in);
    maxFilter.setInputCloud(cloud_in);
    maxFilter.filter(*cloud_las_gridMax);



    pcl::io::savePCDFile(pcl::getFilenameWithoutExtension(filename).append( std::string("_maxImage.pcd") ),
                         *cloud_las_gridMax, false);

//    pcl::copyPointCloud(range_image_COPY, cloud_wNormals);
//    pcl::copyPointCloud(range_image_COPY, *cloud_XYZI_ptr);
//    for(size_t i =0; i < cloud_XYZI_ptr->size(); i++){
//        cloud_XYZI_ptr->points[i].intensity = range_image_COPY[i].range;
//    }
//
//    pcl::io::savePCDFile(pcl::getFilenameWithoutExtension(filename).append( std::string("_rangeImage_XYZI.pcd") ),
//                         *cloud_XYZI_ptr, true);

    t2 = std::chrono::high_resolution_clock::now();
    PCL_WARN  ("Finished calculating max image: saved to  %s after %d seconds \n",
              pcl::getFilenameWithoutExtension(filename).append( std::string("_maxImage.pcd") ).c_str(),
              std::chrono::duration_cast<std::chrono::seconds>(t2 - t1).count() );


    /// --------------------------------------------------------
    /// -----Extract NORMALS of MAX image  ------------ -------
    /// --------------------------------------------------------
    std::cout << " --- Getting NORMALS ---" << std::endl;
    t1 = std::chrono::high_resolution_clock::now();
    pcl::PointCloud<pcl::Normal>::Ptr cloud_Normals_ptr(new pcl::PointCloud<pcl::Normal>);
    pcl::copyPointCloud(*cloud_las_gridMax, *cloud_Normals_ptr);
    pcl::NormalEstimationOMP<pcl::PointXYZI,pcl::Normal> nest;
    nest.setRadiusSearch ( support_size/2.0);
    nest.setInputCloud (cloud_las_gridMax);
    nest.compute (*cloud_Normals_ptr);
    t2 = std::chrono::high_resolution_clock::now();
    pcl::io::savePCDFile(pcl::getFilenameWithoutExtension(filename).append( std::string("_MAXImage_wNormals.pcd") ),
                         *cloud_Normals_ptr, false);
    PCL_WARN  ("Finished calculating NORMALS IN image: saved to  %s after %d seconds \n",
               pcl::getFilenameWithoutExtension(filename).append( std::string("_MAXImage_wNormals.pcd") ).c_str(),
               std::chrono::duration_cast<std::chrono::seconds>(t2 - t1).count() );


    /// --------------------------------
    /// -----Extract SIFT keypoints-----
    /// --------------------------------

    pcl::console::print_warn( "Getting SIFT KEYPOINTS...\n " );
    t1 = std::chrono::high_resolution_clock::now();

    constexpr float min_scale = 1.0f;
    constexpr int n_octaves = 6;
    constexpr int n_scales_per_octave = 10;
    constexpr float min_contrast = 0.5f;
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
    PCL_WARN  ("Finished calculating %d SIFT KEYPOINTS  after %d seconds \n",
               result.size (),
            std::chrono::duration_cast<std::chrono::seconds>(t2 - t1).count() );
    t1 = std::chrono::high_resolution_clock::now();

    /// --------------------------------
    /// -----Extract SUSAN keypoints-----
    /// --------------------------------

    pcl::console::print_warn( "Getting SUSAN KEYPOINTS...\n " );
    t1 = std::chrono::high_resolution_clock::now();

    pcl::SUSANKeypoint<pcl::PointXYZI, pcl::PointXYZ> susan;
    pcl::search::KdTree<pcl::PointXYZI>::Ptr susan_tree(new pcl::search::KdTree<pcl::PointXYZI> ());
    susan.setSearchMethod(susan_tree);
    susan.setRadius(1);
    susan.setNormals(cloud_Normals_ptr);
    susan.setInputCloud(cloud_las_gridMax);
    susan.compute(*susan_keypointsCloud_ptr);

    pcl::io::savePCDFile (pcl::getFilenameWithoutExtension(filename).append( "__SUSAN.pcd" ),
                          *susan_keypointsCloud_ptr, false);


    t2 = std::chrono::high_resolution_clock::now();
    PCL_WARN  ("Finished calculating %d SUSAN KEYPOINTS  after %d seconds \n",
               susan_keypointsCloud_ptr->size (),
               std::chrono::duration_cast<std::chrono::seconds>(t2 - t1).count() );
    t1 = std::chrono::high_resolution_clock::now();



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

    return(0);


     t2 = std::chrono::high_resolution_clock::now();
     std::cout << "Finished creating range images after " << std::chrono::duration_cast<std::chrono::seconds>(t2-t1).count() << " seconds. " << std::endl;


    return 0;
}

#endif //COFIT_READWRITE_H
