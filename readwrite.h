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
#include <pcl/features/normal_3d.h>
#include <pcl/features/narf_descriptor.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>

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
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

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

    auto t2 = std::chrono::high_resolution_clock::now();
    //double centerx = 0, centery = 0, centerz = 0;
    double topx = -DBL_MAX, topy = -DBL_MAX, topz = -DBL_MAX;
    double minx = DBL_MAX, miny = DBL_MAX, minz = DBL_MAX;
    float stepx = 0, stepy = 0;
    bool alreadyExists = false;
    std::string fileWithoutExt = pcl::getFilenameWithoutExtension(filename).append(".pcd");
    if( std::filesystem::exists( std::filesystem::path(fileWithoutExt) ) ){

        std::cout << "Reading file  " << fileWithoutExt << " exists, reading it... remove file if this is not intended "  << std::endl;

        if(  pcl::io::loadPCDFile( fileWithoutExt, *cloud )==0 ){
            std::cout << " Successfully read " << fileWithoutExt   << std::endl;
            alreadyExists=true;
        }
    }

    if(!alreadyExists){

        while (lasreader->read_point()) {

            if (n % every == 0) {
                printProgress((double) n / lasreader->npoints);
            }

//            if (lasreader->point.X > topx) topx = lasreader->point.X;
//            if (lasreader->point.Y > topy) topy = lasreader->point.Y;
//            if (lasreader->point.Z > topz) topz = lasreader->point.Z;
//            if (lasreader->point.X < minx) minx = lasreader->point.X;
//            if (lasreader->point.Y < miny) miny = lasreader->point.Y;
//            if (lasreader->point.Z < minz) minz = lasreader->point.Z;


//        centerx += lasreader->point.X * lasreader->header.x_scale_factor;
//        centery += lasreader->point.Y * lasreader->header.y_scale_factor;
//        centerz += lasreader->point.Z * lasreader->header.z_scale_factor;

            cloud->points[n].x = lasreader->point.X* lasreader->header.x_scale_factor;
            cloud->points[n].y = lasreader->point.Y* lasreader->header.x_scale_factor;
            cloud->points[n].z = lasreader->point.Z* lasreader->header.x_scale_factor;
            // cloud.points[n].intensity = lasreader->point.intensity;
            // laswriter->write_point(&lasreader->point);
            n++;
        }
         t2 = std::chrono::high_resolution_clock::now();
        std::cout << "\nFinished after " << std::chrono::duration_cast<std::chrono::seconds>(t2 - t1).count() << " seconds. "
                  << std::endl;
        //laswriter->close();
        //delete laswriter;
        lasreader->close();
        delete lasreader;


        if (savetopcd) {
            std::cout << "Saving to PCD file" << std::endl;
            t1 = std::chrono::high_resolution_clock::now();
            pcl::io::savePCDFile(pcl::getFilenameWithoutExtension(filename).append(".pcd"), *cloud, true);
            t2 = std::chrono::high_resolution_clock::now();
            std::cout << "Finished saving to  " << pcl::getFilenameWithoutExtension(filename).append(".pcd") << " after "
                      << std::chrono::duration_cast<std::chrono::seconds>(t2 - t1).count() << " seconds. " << std::endl;
        }


    }

    pcl::PointXYZ  minPt, maxPt;
    pcl::getMinMax3D(*cloud, minPt, maxPt);

    topx = maxPt.x; topy = maxPt.y; topz = maxPt.z;
    minx = minPt.x; miny = minPt.y; minz = minPt.z;




    //////////////////////////////
    /// GRID OF RANGE IMAGES /////
    int gridn = 3;
    stepx = (topx - minx)/(float)gridn;
    stepy = (topy - miny)/(float)gridn;
    // We now want to create a range image from the above point cloud, with a 1deg angular resolution
    float angularResolution = (float) (  0.2f * (M_PI/180.0f));  //   0.1 degree in radians
    float maxAngleWidth     = (float) (360.0f * (M_PI/180.0f));  // 360.0 degree in radians
    float maxAngleHeight    = (float) (180.0f * (M_PI/180.0f));  // 180.0 degree in radians
    pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
    float noiseLevel=0.00;
    float minRange = 0.0f;
    int borderSize = 1;





    t1 = std::chrono::high_resolution_clock::now();
    std::cout << "Creating range images " << std::endl;
    for(int x=0; x<gridn; x++){
        for(int y=0; y<gridn; y++){

            //////////////////////////////
            /// CALCULATING RANGE IMAGES  /////
            //////////////////////////////
            
            Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(minx+stepx*x, miny+stepy*y,
                                                                               topz + (topz-minz) );
            pcl::RangeImage rangeImage;

            pcl::CropBox<> ((minx+stepx*x)-10, (minx+stepx*x)+10 );

            pcl::CropBox<pcl::PointXYZ> cb(true);
            cb.setInputCloud (input);
            Eigen::Vector4f min_pt ((minx+stepx*x)-10,  (minx+stepx*x)+10 , -FLT_MAX, 1.0f);
            Eigen::Vector4f max_pt ((miny+stepy*y)-10 , (miny+stepy*y)+10 ,  FLT_MAX, 1.0f);

            // Cropbox slighlty bigger then bounding box of points
            cb.setMin (min_pt);
            cb.setMax (max_pt);

            // Cloud
            pcl::PointCloud<pcl::PointXYZ> cloud_local;
            cb.filter (cloud_local);

            rangeImage.createFromPointCloud(*cloud, angularResolution, maxAngleWidth, maxAngleHeight,
                                            sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
            char outname[100];
            sprintf(outname, "__range_x%d_y%d.pcd", x, y );
            char outname_narf[100];
            sprintf(outname_narf, "__keypoints_NARF_x%d_y%d.pcd", x, y );

            char outname_sift[100];
            sprintf(outname_sift, "__keypoints_SIFT_x%d_y%d.pcd", x, y );

 
            pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_sift_ptr (new pcl::PointCloud<pcl::PointXYZI>);
            pcl::PointCloud<pcl::PointXYZI>& cloud_sift = *point_cloud_sift_ptr;

            pcl:copyPointCloud(rangeImage, cloud_sift);
            for (std::size_t i=0; i<rangeImage.size (); ++i)
                cloud_sift[i].intensity = rangeImage[i].range;

 
            pcl::io::savePCDFile (pcl::getFilenameWithoutExtension(filename).append( std::string(outname) ),
                                  cloud_sift, true);
 
 
            // Parameters for sift computation
            constexpr float min_scale = 10.0f;
            constexpr int n_octaves = 6;
            constexpr int n_scales_per_octave = 10;
            constexpr float min_contrast = 0.5f;


            // Estimate the sift interest points using Intensity values from RGB values
            pcl::SIFTKeypoint<pcl::PointXYZI, pcl::PointWithScale> sift;
            pcl::PointCloud<pcl::PointWithScale> result;
            pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI> ());
            sift.setSearchMethod(tree);
            sift.setScales(min_scale, n_octaves, n_scales_per_octave);
            sift.setMinimumContrast(min_contrast);
            sift.setInputCloud(point_cloud_sift_ptr);
            sift.compute(result);

            // Copying the pointwithscale to pointxyz so as visualize the cloud
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_temp_sift (new pcl::PointCloud<pcl::PointXYZI>);
            copyPointCloud(result, *cloud_temp_sift);

            // Saving the resultant cloud
            std::cout << "Resulting sift points are of size: " << cloud_temp_sift->size () <<std::endl;

            pcl::io::savePCDFile (pcl::getFilenameWithoutExtension(filename).append( std::string(outname_sift) ),
                                  *cloud_temp_sift, true);

            //////////////////////////////
            /// CALCULATING NARF KEYPOINTS ////
            //////////////////////////////
            pcl::RangeImageBorderExtractor range_image_border_extractor;
            pcl::NarfKeypoint narf_keypoint_detector;
            narf_keypoint_detector.setRangeImageBorderExtractor (&range_image_border_extractor);
            narf_keypoint_detector.setRangeImage (&rangeImage);
            narf_keypoint_detector.getParameters ().support_size = 0.5f;

            pcl::PointCloud<int> keypoint_indices;
            narf_keypoint_detector.compute (keypoint_indices);

            pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_ptr (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>& keypoints = *keypoints_ptr;
            keypoints.resize (keypoint_indices.size ());
            for (std::size_t i=0; i<keypoint_indices.size (); ++i)
                keypoints[i].getVector3fMap () = rangeImage[keypoint_indices[i]].getVector3fMap ();



            std::vector<int> keypoint_indices2;
            keypoint_indices2.resize (keypoint_indices.size ());
            for (unsigned int i=0; i<keypoint_indices.size (); ++i) // This step is necessary to get the right vector type
                keypoint_indices2[i]=keypoint_indices[i];

            pcl::NarfDescriptor narf_descriptor (&rangeImage, &keypoint_indices2);
            narf_descriptor.getParameters ().support_size = 1;
            narf_descriptor.getParameters ().rotation_invariant = true;

            pcl::PointCloud<pcl::Narf36> narf_descriptors;
            narf_descriptor.compute (narf_descriptors);
            std::cout << "Extracted "<<narf_descriptors.size ()<<" NARF descriptors for "
                      <<keypoint_indices2.size ()<< " keypoints.\n";


            pcl::io::savePCDFile (pcl::getFilenameWithoutExtension(filename).append( std::string(outname_narf) ),
                                  keypoints, true);




        }
    }

     t2 = std::chrono::high_resolution_clock::now();
     std::cout << "Finished creating range images after " << std::chrono::duration_cast<std::chrono::seconds>(t2-t1).count() << " seconds. " << std::endl;


    return 0;
}

#endif //COFIT_READWRITE_H
