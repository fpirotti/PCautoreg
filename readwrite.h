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

#include <pcl/filters/grid_minimum.h>

#include "grid_maximum.h"

//#include <pcl/features/range_image_border_extractor.h>

#include <pcl/filters/passthrough.h>
#include <pcl/registration/icp.h>
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
                           pcl::PointCloud<pcl::Normal>::Ptr cloud_norm,
                           float max_window_res=1.0f );

int las2keypoints(   std::string filename,
                     pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_las_gridMax,
                     pcl::PointCloud<pcl::Normal>::Ptr cloud_Normals_ptr,
                      std::map<std::string,  pcl::IndicesPtr  > &keypointMap,
                     float support_size=5.0f );

#endif //COFIT_READWRITE_H
