//
// Created by pirotti on 17/04/23.
//

#ifndef PCAUTOREG_ALIGNRANSAC_H
#define PCAUTOREG_ALIGNRANSAC_H

#include <iostream>
#include <filesystem>
#include <chrono>
#include <pcl/memory.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/file_io.h>
#include <pcl/console/parse.h>
#include <pcl/common/time.h>
#include <pcl/registration/sample_consensus_prerejective.h>

typedef pcl::PointXYZ PointNT;
typedef pcl::PointCloud<PointNT> PointCloudT;
typedef pcl::FPFHSignature33 FeatureT_fpfh;
typedef pcl::PointCloud<FeatureT_fpfh> FeatureCloudT_fpfh;

class alignRANSAC {

protected:
    PointCloudT::Ptr object; // (new PointCloudT);
    PointCloudT::Ptr object_aligned ;
    PointCloudT::Ptr scene ;
    FeatureCloudT_fpfh::Ptr object_features ;
    FeatureCloudT_fpfh::Ptr scene_features;

public:
     alignRANSAC () :
        object(new PointCloudT),
        object_aligned (new PointCloudT),
         scene (new PointCloudT),
        object_features (new FeatureCloudT_fpfh),
        scene_features (new FeatureCloudT_fpfh)
    {}  ;

     int  align() ;
};


#endif //PCAUTOREG_ALIGNRANSAC_H
