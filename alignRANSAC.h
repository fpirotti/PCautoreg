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

class alignRANSAC {

protected:
    int type; // 1 == FPFH, 2=SPIN
    PointCloudT::Ptr object; // (new PointCloudT);
    PointCloudT::Ptr object_aligned ;
    PointCloudT::Ptr scene ;


public:
    enum featureType { FPFH = 1, SPIN=2 };
     alignRANSAC () :
        type(0),
        object(new PointCloudT),
        object_aligned (new PointCloudT),
        scene (new PointCloudT)
    {

        PointCloudT::Ptr object_aligned (new PointCloudT);

    }  ;
    void setMasterCloud(PointCloudT::Ptr pc){
        this->scene = pc;
    };
    void setSlaveCloud(PointCloudT::Ptr pc){
        this->object = pc;
    };

    Eigen::Matrix4f *  align_SPIN( pcl::PointCloud< pcl::Histogram<153> >::Ptr scene_features, pcl::PointCloud< pcl::Histogram<153> >::Ptr object_features ){

        typedef pcl::Histogram<153> FeatureT;
        typedef pcl::PointCloud<FeatureT> FeatureCloudT;

        pcl::console::print_highlight ("Starting alignment...\n");
        pcl::SampleConsensusPrerejective<PointNT,PointNT,FeatureT> align;
        align.setInputSource (object);
        align.setSourceFeatures (object_features);
        align.setInputTarget (scene);
        align.setTargetFeatures (scene_features);
        align.setMaximumIterations (50000); // Number of RANSAC iterations
        align.setNumberOfSamples (3); // Number of points to sample for generating/prerejecting a pose
        align.setCorrespondenceRandomness (5); // Number of nearest features to use
        align.setSimilarityThreshold (0.95f); // Polygonal edge length similarity threshold
        align.setMaxCorrespondenceDistance (2.0f ); // Inlier threshold
        align.setInlierFraction (0.25f); // Required inlier fraction for accepting a pose hypothesis
        {
            pcl::ScopeTime t("Alignment");
            align.align (*object_aligned);
        }
        if(align.hasConverged()){
            Eigen::Matrix4f *transformation = new Eigen::Matrix4f( align.getFinalTransformation () );
            printResult( *transformation );
            return transformation;
        }   else {
            pcl::console::print_error ("Alignment failed!\n");
        }
        return nullptr;
        // Perform alignment
    };

Eigen::Matrix4f *  align_FPFH( pcl::PointCloud<pcl::FPFHSignature33>::Ptr scene_features, pcl::PointCloud<pcl::FPFHSignature33>::Ptr object_features ){

        typedef pcl::FPFHSignature33 FeatureT;
        typedef pcl::PointCloud<FeatureT> FeatureCloudT;

        pcl::console::print_highlight ("Starting alignment...\n");
        pcl::SampleConsensusPrerejective<PointNT,PointNT,FeatureT> align;
        align.setInputSource (object);
        align.setSourceFeatures (object_features);
        align.setInputTarget (scene);
        align.setTargetFeatures (scene_features);
        align.setMaximumIterations (50000); // Number of RANSAC iterations
        align.setNumberOfSamples (3); // Number of points to sample for generating/prerejecting a pose
        align.setCorrespondenceRandomness (5); // Number of nearest features to use
        align.setSimilarityThreshold (0.95f); // Polygonal edge length similarity threshold
        align.setMaxCorrespondenceDistance (2.0f ); // Inlier threshold
        align.setInlierFraction (0.25f); // Required inlier fraction for accepting a pose hypothesis
        {
            pcl::ScopeTime t("Alignment");
            align.align (*object_aligned);
        }
        if(align.hasConverged()){
            Eigen::Matrix4f *transformation = new Eigen::Matrix4f( align.getFinalTransformation () );
            printResult( *transformation );
            return transformation;
        }   else {
            pcl::console::print_error ("Alignment failed!\n");
        }
        return nullptr;
        // Perform alignment
    };

    void printResult(Eigen::Matrix4f transformation){
        printf ("\n");
        pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (0,0), transformation (0,1), transformation (0,2));
        pcl::console::print_info ("R = | %6.3f %6.3f %6.3f | \n", transformation (1,0), transformation (1,1), transformation (1,2));
        pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (2,0), transformation (2,1), transformation (2,2));
        pcl::console::print_info ("\n");
        pcl::console::print_info ("t = < %0.3f, %0.3f, %0.3f >\n", transformation (0,3), transformation (1,3), transformation (2,3));
        pcl::console::print_info ("\n");
        pcl::console::print_info ("Inliers: %i/%i\n", align.getInliers ().size (), object->size ());

    }

};


#endif //PCAUTOREG_ALIGNRANSAC_H
