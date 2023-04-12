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
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/features/normal_3d.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/filters/crop_box.h>

#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>

#include <pcl/features/narf.h>
#include <pcl/features/narf_descriptor.h>

#include <pcl/filters/passthrough.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/range_image/range_image.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/features/narf_descriptor.h>
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

template<typename PointT> inline int readPC(std::string filename, pcl::PointCloud<PointT> &cloud);


template<typename PointT> inline int las2keypoints(std::string filename, pcl::PointCloud<pcl::PointXYZ>::Ptr KEYPOINTS_cloud, bool savetopcd= false,
                                                    pcl::PointCloud<pcl::Narf36>::Ptr narf_descriptor_pointer= nullptr,
                                                    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_descriptor_ptr= nullptr, float support_size=5.0f ) {



    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_las(new pcl::PointCloud<pcl::PointXYZI>);
  //  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_normals(new pcl::PointCloud<pcl::PointXYZI>);

    std::cout << "Reading " << filename << " - extension: " << pcl::getFileExtension(filename) << std::endl;

    auto t1 = std::chrono::high_resolution_clock::now();
    auto t2 = std::chrono::high_resolution_clock::now();
    //double centerx = 0, centery = 0, centerz = 0;
    double topx = -DBL_MAX, topy = -DBL_MAX, topz = -DBL_MAX;
    double minx = DBL_MAX, miny = DBL_MAX, minz = DBL_MAX;
    float stepx = 0, stepy = 0;
    bool alreadyExists = false;
//    std::string fileWithoutExt = pcl::getFilenameWithoutExtension(filename).append(".pcd");
//
//    if( std::filesystem::exists( std::filesystem::path(fileWithoutExt) ) ){
//
//        std::cout << "Reading file  " << fileWithoutExt << " exists, reading it... remove file if this is not intended "  << std::endl;
//
//        if(  pcl::io::loadPCDFile( fileWithoutExt, *cloud_with_normals )==0 ){
//            std::cout << " Successfully read " << fileWithoutExt   << std::endl;
//            alreadyExists=true;
//        }
//    }

    if(!alreadyExists && (pcl::getFileExtension(filename)=="las" || pcl::getFileExtension(filename)=="LAS"
                      || pcl::getFileExtension(filename)=="laz" || pcl::getFileExtension(filename)=="LAZ")  ){

        std::cout << " Reading  " << filename << " - extension: " << pcl::getFileExtension(filename) << std::endl;
        LASreadOpener lasreadopener;
        lasreadopener.set_file_name(filename.c_str());
        LASreader *lasreader = lasreadopener.open();
        if (lasreader == 0) {
            std::cout << "Could not open  " << filename << " - extension: " << pcl::getFileExtension(filename) << std::endl;
            return (-1);
        }
        int every = lasreader->npoints / 1000;
        int n = 0;

        cloud_las->points.resize(lasreader->npoints);
        cloud_las->width = lasreader->npoints;
        cloud_las->height = 1;

        while (lasreader->read_point()) {

            if (n % every == 0) {
                printProgress((double) n / lasreader->npoints);
            }
            cloud_las->points[n].x = lasreader->point.X* lasreader->header.x_scale_factor;
            cloud_las->points[n].y = lasreader->point.Y* lasreader->header.x_scale_factor;
            cloud_las->points[n].z = lasreader->point.Z* lasreader->header.x_scale_factor;
            cloud_las->points[n].intensity = lasreader->point.intensity;
            // laswriter->write_point(&lasreader->point);
            n++;
        }

        t2 = std::chrono::high_resolution_clock::now();
        std::cout << "\nFinished after " << std::chrono::duration_cast<std::chrono::seconds>(t2 - t1).count() << " seconds. "
                  << std::endl;
        lasreader->close();
        delete lasreader;


//        if (savetopcd) {
//            std::cout << "Saving to PCD file" << std::endl;
//            t1 = std::chrono::high_resolution_clock::now();
//            pcl::io::savePCDFile(pcl::getFilenameWithoutExtension(filename).append(".pcd"), *cloud_with_normals, true);
//            t2 = std::chrono::high_resolution_clock::now();
//            std::cout << "Finished saving to  " << pcl::getFilenameWithoutExtension(filename).append(".pcd") << " after "
//                      << std::chrono::duration_cast<std::chrono::seconds>(t2 - t1).count() << " seconds. " << std::endl;
//        }


    }

    pcl::PointXYZI  minPt, maxPt;
    pcl::getMinMax3D(*cloud_las, minPt, maxPt);

    topx = maxPt.x; topy = maxPt.y; topz = maxPt.z;
    minx = minPt.x; miny = minPt.y; minz = minPt.z;

    double centerx = minPt.x + (maxPt.x - minPt.x)/2;
    double centery = minPt.y + (maxPt.y - minPt.y)/2;



    //////////////////////////////
    //////  RANGE IMAGES   ///////
    //////////////////////////////

    std::cout << "Getting RANGE image" << std::endl;
    t1 = std::chrono::high_resolution_clock::now();

    int gridn = 3;
    stepx = (topx - minx)/(float)gridn;
    stepy = (topy - miny)/(float)gridn;
    // We now want to create a range image from the above point cloud, with a 1deg angular resolution
    float angularResolution = (float) (  0.2f * (M_PI/180.0f));  //   0.1 degree in radians
    float maxAngleWidth     = (float) (360.0f * (M_PI/180.0f));  // 360.0 degree in radians
    float maxAngleHeighttot     = (float) (8.0f * (M_PI/180.0f));  // 360.0 degree in radians
    float maxAngleHeight    = (float) (180.0f * (M_PI/180.0f));  // 180.0 degree in radians
    pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;

    float noiseLevel=0.00;
    float minRange = 0.0f;
    int borderSize = 1;

    float sizex = maxPt.x - minPt.x;
    float sizey = maxPt.y - minPt.y;
    float sizemax = sizex;
    if(sizex < sizey) sizemax=sizey;


    //pcl::RangeImage::Ptr rangeImage_ptr  (new pcl::RangeImage);

    pcl::RangeImage::Ptr range_image_ptr (new pcl::RangeImage);
    pcl::RangeImage& range_image = *range_image_ptr;
    pcl::RangeImage::Ptr range_image_COPY_ptr (new pcl::RangeImage);
    pcl::RangeImage& range_image_COPY = *range_image_COPY_ptr;
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_wNormals_ptr (new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>& cloud_wNormals = *cloud_wNormals_ptr;

    Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(centerx, centery, maxPt.z+sizemax*10  );
    range_image.createFromPointCloud(*cloud_las, 0.0005, maxAngleWidth, maxAngleHeighttot,
                                    sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);

    range_image.setUnseenToMaxRange ();
    char outname_narf[100];
    sprintf(outname_narf, "__keypoints_NARF.pcd" );


    char outname_sift[100];
    sprintf(outname_sift, "__keypoints_SIFT.pcd"  );

    pcl::Indices ind; //std::vector<int>
    pcl::removeNaNFromPointCloud(range_image, range_image_COPY, ind);


    pcl::io::savePCDFile(pcl::getFilenameWithoutExtension(filename).append( std::string("_rangeImage.pcd") ),
                         range_image_COPY, true);

    pcl::copyPointCloud(range_image_COPY, cloud_wNormals);

    t2 = std::chrono::high_resolution_clock::now();
    std::cout << "Finished calculating range image: saved to  "  << " after "
              << std::chrono::duration_cast<std::chrono::seconds>(t2 - t1).count() << " seconds. " << std::endl;


    /// --------------------------------------------------------
    /// -----Extract NORMALS of rangeimage  ------------ -------
    /// --------------------------------------------------------
    std::cout << "Getting NORMALS..." << std::endl;
    t1 = std::chrono::high_resolution_clock::now();

    pcl::NormalEstimationOMP<pcl::PointNormal,pcl::PointNormal> nest;
    nest.setRadiusSearch ( support_size/2.0);
    nest.setInputCloud (cloud_wNormals_ptr);
    nest.compute (*cloud_wNormals_ptr);

    t2 = std::chrono::high_resolution_clock::now();
    std::cout << "Finished calculating  NORMALS after "
              << std::chrono::duration_cast<std::chrono::seconds>(t2 - t1).count() << " seconds. Saving point cloud..." << std::endl;

    pcl::io::savePCDFile(pcl::getFilenameWithoutExtension(filename).append( std::string("_rangeImage_wNormals.pcd") ),
                         cloud_wNormals, true);


    /// --------------------------------
    /// -----Extract SIFT keypoints-----
    /// --------------------------------
//    std::cout << "Getting SIFT KEYPOINTS..." << std::endl;
//    t1 = std::chrono::high_resolution_clock::now();
//
//    // Parameters for sift computation
//    constexpr float min_scale = 10.0f;
//    constexpr int n_octaves = 6;
//    constexpr int n_scales_per_octave = 10;
//    constexpr float min_contrast = 0.5f;
//
//
//    // Estimate the sift interest points using Intensity values from RGB values
//    pcl::SIFTKeypoint<pcl::PointNormal, pcl::PointWithScale> sift;
//    pcl::PointCloud<pcl::PointWithScale> result;
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal> ());
//    sift.setSearchMethod(tree);
//    sift.setScales(min_scale, n_octaves, n_scales_per_octave);
//    sift.setMinimumContrast(min_contrast);
//    sift.setInputCloud(cloud_wNormals_ptr);
//    sift.compute(result);
//
//    // Copying the pointwithscale to pointxyz so as visualize the cloud
//    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_temp_sift (new pcl::PointCloud<pcl::PointNormal>);
//    copyPointCloud(result, *cloud_temp_sift);
//
//    // Saving the resultant cloud
//    std::cout << "Resulting sift points are of size: " << cloud_temp_sift->size () <<std::endl;
//
//    pcl::io::savePCDFile (pcl::getFilenameWithoutExtension(filename).append( std::string(outname_sift) ),
//                          *cloud_temp_sift, true);
//
//    t2 = std::chrono::high_resolution_clock::now();
//    std::cout << "Finished calculating SIFT saved to  " << outname_sift << " after "
//              << std::chrono::duration_cast<std::chrono::seconds>(t2 - t1).count() << " seconds. " << std::endl;

    std::cout << "Getting NARF KEYPOINTS..." << std::endl;
    t1 = std::chrono::high_resolution_clock::now();

    /// --------------------------------
    /// -----Extract NARF keypoints-----
    /// --------------------------------
    pcl::RangeImageBorderExtractor range_image_border_extractor;
    pcl::NarfKeypoint narf_keypoint_detector;
    narf_keypoint_detector.setRangeImageBorderExtractor (&range_image_border_extractor);
    narf_keypoint_detector.setRangeImage (&range_image);
    narf_keypoint_detector.getParameters ().support_size = support_size;

    pcl::PointCloud<int> keypoint_indices;
    narf_keypoint_detector.compute (keypoint_indices);


    if(keypoint_indices.size()==0){
        pcl::console::print(pcl::console::L_WARN, "No keypoints found using %d points in range image. Try increasing support size or decrease angular resolution of range image.\n", range_image.size()  );
        return(0);
    } else {
        pcl::console::print(pcl::console::L_INFO, "%d keypoints found using %d points in range image.", keypoint_indices.size(), range_image.size()  );

    }
    pcl::PointCloud<pcl::PointXYZ>  &keypointsCloud = *KEYPOINTS_cloud;
    keypointsCloud.resize (keypoint_indices.size ());



    std::cout << "Getting NARF FEATURES ..." << std::endl;
    t1 = std::chrono::high_resolution_clock::now();
    /// ------------------------------------------------------
    /// -----Extract NARF descriptors for interest points-----
    /// ------------------------------------------------------
//    std::vector<int> keypoint_indices2;
//    keypoint_indices2.resize (keypoint_indices.size ());
//    for (unsigned int i=0; i<keypoint_indices.size (); ++i) // This step is necessary to get the right vector type
//        keypoint_indices2[i]=keypoint_indices[i];

    std::vector<pcl::Narf*> feature_list;

    Eigen::Vector3f feature_pos;
    int nn=0;
    for (std::size_t i=0; i<keypoint_indices.size (); ++i) {
        feature_pos = range_image[keypoint_indices[i]].getVector3fMap();
        pcl::Narf* feature = new pcl::Narf;
        keypointsCloud[i].getVector3fMap() = feature_pos;
        feature->extractFromRangeImageWithBestRotation(range_image, feature_pos, 36, support_size);
        nn++;
    }

    pcl::PointCloud<pcl::Narf36> narf_descriptors;
    narf_descriptors.resize(feature_list.size());
    for (std::size_t i = 0; i < feature_list.size(); ++i)
    {
        feature_list[i]->copyToNarf36(narf_descriptors[i]);
    }

    pcl::console::print_warn( "\nExtracted %d NARF features for %d keypoints.\n " ,
                              feature_list.size (),
                              keypoint_indices.size ());


    pcl::io::savePCDFileASCII (pcl::getFilenameWithoutExtension(filename).append( std::string("__NARF_KEYPOINTS.pcd") ),
                               keypointsCloud);
//    pcl::io::savePCDFileASCII (pcl::getFilenameWithoutExtension(filename).append( std::string("__NARF_Features.pcd") ),
//                               narf_descriptors);

    t2 = std::chrono::high_resolution_clock::now();
    std::cout << "Finished calculating NARF saved to  " << outname_sift << " after "
              << std::chrono::duration_cast<std::chrono::seconds>(t2 - t1).count() << " seconds. " << std::endl;


   ////////////////////////////////////
   /// ok now we calculate FPFH features ///
   ////////////////////////////////////

    std::cout << "Getting FPFH FEATURES ..." << std::endl;
    t1 = std::chrono::high_resolution_clock::now();

    pcl::FPFHEstimationOMP<pcl::PointNormal, pcl::PointNormal, pcl::FPFHSignature33> fpfh;
    fpfh.setInputCloud(cloud_wNormals_ptr);
    fpfh.setInputNormals(cloud_wNormals_ptr);
    fpfh.setSearchMethod(tree);
    fpfh.setRadiusSearch(2);
    //fpfh.setIndices( keypoint_indices);
    fpfh.compute(* fpfh_descriptor_ptr );
    t2 = std::chrono::high_resolution_clock::now();

    std::cout << "Finished calculating " <<  keypoint_indices.size() << " FPFH FEATURES  ... final size= " <<  fpfh_descriptor_ptr->points.size() << " after "
              << std::chrono::duration_cast<std::chrono::seconds>(t2 - t1).count() << " seconds. " << std::endl;

    return(0);
    //t1 = std::chrono::high_resolution_clock::now();
    //std::cout << "Creating range images " << std::endl;
    for(int x=0; x<gridn; x++){
        for(int y=0; y<gridn; y++){

            //////////////////////////////
            /// CALCULATING RANGE IMAGES  /////
            //////////////////////////////

//            pcl::RangeImage rangeImage;
//
//
//            pcl::CropBox<pcl::PointXYZ> cb;
//            cb.setInputCloud (cloud);
//            Eigen::Vector4f min_pt ((minx+stepx*x)-10,  (minx+stepx*x)+10 , -FLT_MAX, 1.0f);
//            Eigen::Vector4f max_pt ((miny+stepy*y)-10 , (miny+stepy*y)+10 ,  FLT_MAX, 1.0f);
//
//            // Cropbox slighlty bigger then bounding box of points
//            cb.setMin (min_pt);
//            cb.setMax (max_pt);
//
//            // Cloud
//            pcl::PointCloud<pcl::PointXYZ> cloud_local;
//            cb.filter (cloud_local);
//
//            pcl::PointXYZ  minPtLoc, maxPtLoc;
//            pcl::getMinMax3D(cloud_local, minPtLoc, maxPtLoc);
//
//
//            Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(minx+stepx*x, miny+stepy*y,
//                                                                               maxPtLoc.z+30 );
//            rangeImage.createFromPointCloud(*cloud, angularResolution, maxAngleWidth, maxAngleHeight,
//                                            sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
//            char outname[100];
//            sprintf(outname, "__range_x%d_y%d.pcd", x, y );
//            char outname_narf[100];
//            sprintf(outname_narf, "__keypoints_NARF_x%d_y%d.pcd", x, y );
//
//            char outname_sift[100];
//            sprintf(outname_sift, "__keypoints_SIFT_x%d_y%d.pcd", x, y );




        }
    }

     t2 = std::chrono::high_resolution_clock::now();
     std::cout << "Finished creating range images after " << std::chrono::duration_cast<std::chrono::seconds>(t2-t1).count() << " seconds. " << std::endl;


    return 0;
}

#endif //COFIT_READWRITE_H
