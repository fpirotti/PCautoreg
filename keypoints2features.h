//
// Created by pirotti on 17/04/23.
//

#ifndef PCAUTOREG_KEYPOINTS2FEATURES_H
#define PCAUTOREG_KEYPOINTS2FEATURES_H


#include <iostream>
#include <filesystem>
#include <chrono>
#include <pcl/memory.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/common/common.h>
#include <pcl/common/file_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/pcl_macros.h>

#include <pcl/console/parse.h>

#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/narf.h>
#include <pcl/features/narf_descriptor.h>
#include <pcl/features/spin_image.h>

int  keypoints2features(std::string filename,
                        float support_size,
                        pcl::IndicesPtr  cloud_keypoints_indices= nullptr,
                        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_max= nullptr,
                        pcl::PointCloud<pcl::Normal>::Ptr cloud_normal= nullptr,
                        pcl::PointCloud<pcl::FPFHSignature33>::Ptr  FPFH_signature33= nullptr,
                        pcl::PointCloud<pcl::Histogram<153> >::Ptr  SPIN_signature = nullptr
) ;
int  keypoints2features2(std::string &filename,
                        float support_size
) ;
#endif //PCAUTOREG_KEYPOINTS2FEATURES_H
