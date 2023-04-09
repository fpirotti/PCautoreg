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
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>

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


template<typename PointT> inline int las2pcd(std::string filename,  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, bool savetopcd=false) {
    LASreadOpener lasreadopener;
    lasreadopener.set_file_name(filename.c_str());
    LASreader *lasreader = lasreadopener.open();

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointXYZINormal>);

    if (lasreader == 0) {
        std::cout << "Could not open  " << filename << " - extension: " << pcl::getFileExtension(filename) << std::endl;
        return (-1);
    }
    //LASwriteOpener laswriteopener;
    //laswriteopener.set_file_name("compressed.laz");
    //LASwriter* laswriter = laswriteopener.open(&lasreader->header);

    std::cout << "Reading " << filename << " - extension: " << pcl::getFileExtension(filename) << " \nnpoints="
              << lasreader->npoints << std::endl;
    int every = lasreader->npoints / 1000;
    int n = 0;
    std::cout << "Size before " << cloud->size() << std::endl;
    cloud->points.resize(lasreader->npoints);
    cloud->width = lasreader->npoints;
    cloud->height = 1;
    std::cout << "Size after " << cloud->size() << std::endl;
    auto t1 = std::chrono::high_resolution_clock::now();

    //double centerx = 0, centery = 0, centerz = 0;
    double topx = -DBL_MAX, topy = -DBL_MAX, topz = -DBL_MAX;
    double minx = DBL_MAX, miny = DBL_MAX, minz = DBL_MAX;
    float stepx = 0, stepy = 0;
    bool alreadyExists = false;
    std::string fileWithoutExt = pcl::getFilenameWithoutExtension(filename).append(".pcd");
    if( std::filesystem::exists( std::filesystem::path(fileWithoutExt) ) ){

        std::cout << "Reading file  " << fileWithoutExt << " exists, reading it... remove file if this is not intended "  << std::endl;

        if(  pcl::io::loadPCDFile( , *cloud_with_normals )==0 ){
            std::cout << " Successfully read " << fileWithoutExt   << std::endl;
            alreadyExists=true;
        }
    }

    pcl::PointXYZINormal minPt, maxPt;
    pcl::getMinMax3D(*cloud_with_normals, minPt, maxPt);

    topx = maxPt.x; topy = maxPt.y; topz = maxPt.z;
    minx = minPt.x; miny = minPt.y; minz = minPt.z;

    if(!alreadyExists){

        while (lasreader->read_point()) {

            if (n % every == 0) {
                printProgress((double) n / lasreader->npoints);
            }

            if (lasreader->point.X > topx) topx = lasreader->point.X;
            if (lasreader->point.Y > topy) topy = lasreader->point.Y;
            if (lasreader->point.Z > topz) topz = lasreader->point.Z;
            if (lasreader->point.X < minx) minx = lasreader->point.X;
            if (lasreader->point.Y < miny) miny = lasreader->point.Y;
            if (lasreader->point.Z < minz) minz = lasreader->point.Z;


//        centerx += lasreader->point.X * lasreader->header.x_scale_factor;
//        centery += lasreader->point.Y * lasreader->header.y_scale_factor;
//        centerz += lasreader->point.Z * lasreader->header.z_scale_factor;

            cloud->points[n].x = lasreader->point.X;
            cloud->points[n].y = lasreader->point.Y;
            cloud->points[n].z = lasreader->point.Z;
            // cloud.points[n].intensity = lasreader->point.intensity;
            // laswriter->write_point(&lasreader->point);
            n++;
        }

    }

    auto t2 = std::chrono::high_resolution_clock::now();
    std::cout << "\nFinished after " << std::chrono::duration_cast<std::chrono::seconds>(t2 - t1).count() << " seconds. "
              << std::endl;
    //laswriter->close();
    //delete laswriter;
    lasreader->close();
    delete lasreader;

    //////////////////////////////
    /// CALCULATING NORMALS /////
    std::cout << "Start calculating NORMALS " << std::endl;

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

    t1 = std::chrono::high_resolution_clock::now();
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    //ne.setRadiusSearch(1);
    ne.setSearchMethod(tree);
    ne.setKSearch(20);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    ne.compute(*cloud_normals);

    pcl::concatenateFields(*cloud, *cloud_normals, *cloud_with_normals);

    t2 = std::chrono::high_resolution_clock::now();
    std::cout << "Finished Normal calculation "
                 " after " << std::chrono::duration_cast<std::chrono::seconds>(t2 - t1).count() << " seconds. "
              << std::endl;

    if (savetopcd) {
        std::cout << "Saving to PCD file" << std::endl;
        t1 = std::chrono::high_resolution_clock::now();
        pcl::io::savePCDFile(pcl::getFilenameWithoutExtension(filename).append(".pcd"), *cloud_with_normals);
        t2 = std::chrono::high_resolution_clock::now();
        std::cout << "Finished saving to  " << pcl::getFilenameWithoutExtension(filename).append(".pcd") << " after "
                  << std::chrono::duration_cast<std::chrono::seconds>(t2 - t1).count() << " seconds. " << std::endl;
    }


    //////////////////////////////
    /// GRID OF RANGE IMAGES /////
    int gridn = 5;
    stepx = (topx - minx)/(float)gridn;
    stepy = (topy - miny)/(float)gridn;
    // We now want to create a range image from the above point cloud, with a 1deg angular resolution
    float angularResolution = (float) (  0.1f * (M_PI/180.0f));  //   0.1 degree in radians
    float maxAngleWidth     = (float) (360.0f * (M_PI/180.0f));  // 360.0 degree in radians
    float maxAngleHeight    = (float) (180.0f * (M_PI/180.0f));  // 180.0 degree in radians
    pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
    float noiseLevel=0.00;
    float minRange = 0.0f;
    float support_size = 0.2f;
    int borderSize = 1;

     t1 = std::chrono::high_resolution_clock::now();
    std::cout << "Creating range images " << std::endl;
    for(int x=0; x<gridn; x++){
        for(int y=0; y<gridn; y++){
            Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(minx+stepx*x, miny+stepy*y,
                                                                               topz + (topz-minz) );
            pcl::RangeImage rangeImage;
            rangeImage.createFromPointCloud(*cloud, angularResolution, maxAngleWidth, maxAngleHeight,
                                            sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
            char outname[100];
            printf(outname, "__range_x%d_y%d.pcd", x, y );
            pcl::io::savePCDFileASCII (pcl::getFilenameWithoutExtension(filename).append( std::string(outname) ), rangeImage);

            std::cout << "Range image " << x+x*y << " of 100 " << std::endl;
        }
    }
     t2 = std::chrono::high_resolution_clock::now();
    std::cout << "Finished creating range images after " << std::chrono::duration_cast<std::chrono::seconds>(t2-t1).count() << " seconds. " << std::endl;

return(0);

    //std::cout << rangeImage << "\n";

    t1 = std::chrono::high_resolution_clock::now();

    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
    fpfh.setInputCloud (cloud);
    fpfh.setInputNormals (cloud_normals);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs (new pcl::PointCloud<pcl::FPFHSignature33> ());

    fpfh.setKSearch(20);
    fpfh.compute (*fpfhs);
    t2 = std::chrono::high_resolution_clock::now();
    std::cout << "Finished FPFHE "
                 " after " << std::chrono::duration_cast<std::chrono::seconds>(t2-t1).count() << " seconds. " << std::endl;





//
//
//
//    t1 = std::chrono::high_resolution_clock::now();
//    std::cout << "Keypoints from range image.... " << std::endl;
//
//    pcl::RangeImageBorderExtractor range_image_border_extractor;
//    pcl::NarfKeypoint narf_keypoint_detector (&range_image_border_extractor);
//    narf_keypoint_detector.setRangeImage (&rangeImage);
//    narf_keypoint_detector.getParameters ().support_size = support_size;
//    //narf_keypoint_detector.getParameters ().add_points_on_straight_edges = true;
//    //narf_keypoint_detector.getParameters ().distance_for_additional_points = 0.5;
//
//    pcl::PointCloud<int> keypoint_indices;
//    narf_keypoint_detector.compute (keypoint_indices);
//    t2 = std::chrono::high_resolution_clock::now();
//    std::cout << "Finished keypoints after " << std::chrono::duration_cast<std::chrono::seconds>(t2-t1).count() << " seconds. " << std::endl;
//    std::cout << "Found "<<keypoint_indices.size ()<<" key points.\n";
//
//    pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_ptr (new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::PointCloud<pcl::PointXYZ>& keypoints = *keypoints_ptr;
//    keypoints.resize (keypoint_indices.size ());
//    for (std::size_t i=0; i<keypoint_indices.size (); ++i)
//        keypoints[i].getVector3fMap () = rangeImage[keypoint_indices[i]].getVector3fMap ();
//
//
//    pcl::io::savePCDFileASCII (pcl::getFilenameWithoutExtension(filename).append("__keypoints.pcd"), keypoints);


    return 0;
}

#endif //COFIT_READWRITE_H
