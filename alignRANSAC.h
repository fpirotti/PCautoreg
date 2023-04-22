//
// Created by pirotti on 17/04/23.
//

#ifndef PCAUTOREG_ALIGNRANSAC_H
#define PCAUTOREG_ALIGNRANSAC_H

#include <iostream>
#include <filesystem>
#include <chrono>
#include <pcl/memory.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/impl/point_types.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/file_io.h>
#include <pcl/console/parse.h>
#include <pcl/common/time.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/sample_consensus_prerejective.h>


typedef pcl::PointXYZ PointNT;
typedef pcl::PointXYZI PointNTi;
typedef pcl::PointCloud<PointNT> PointCloudT;
typedef pcl::PointCloud<PointNTi> PointCloudTi;

class alignRANSAC {

protected:
    int type; // 1 == FPFH, 2=SPIN
    PointCloudT::Ptr object; // (new PointCloudT);
    PointCloudT::Ptr object_aligned ;
    PointCloudTi::Ptr cloud2register ;
    PointCloudT::Ptr scene ;
    std::string  filename;

public:
    enum featureType { FPFH = 1, SPIN=2 };
     alignRANSAC () :
        type(0),
        object(new PointCloudT),
        object_aligned (new PointCloudT),
        cloud2register(nullptr),
        scene (new PointCloudT),
        filename("0")
    {
        PointCloudT::Ptr object_aligned (new PointCloudT);
    }  ;
    void setMasterCloud(PointCloudT::Ptr pc){
        this->scene = pc;
    };
    void setSlaveCloud(PointCloudT::Ptr pc){
        this->object = pc;
    };
    void setOutName(std::string filename){
        this->filename = filename;
    };
    template <typename PointT> inline void
    setCloud2Register(std::shared_ptr<pcl::PointCloud<PointT>> cloud_in){
        this->cloud2register = cloud_in;
    };

    Eigen::Matrix4f *  align_SPIN( pcl::PointCloud< pcl::Histogram<153> >::Ptr scene_features, pcl::PointCloud< pcl::Histogram<153> >::Ptr object_features ){

        typedef pcl::Histogram<153> FeatureT;
        typedef pcl::PointCloud<FeatureT> FeatureCloudT;

        pcl::console::print_highlight ("Starting alignment with %d and %d features respectively for source and target...\n",
                                       object_features->size(), scene_features->size() );

        pcl::SampleConsensusPrerejective<PointNT,PointNT,FeatureT> align;
        align.setInputSource (object);
        align.setSourceFeatures (object_features);
        align.setInputTarget (scene);
        align.setTargetFeatures (scene_features);
        align.setMaximumIterations (50000); // Number of RANSAC iterations
        align.setNumberOfSamples (5); // Number of points to sample for generating/prerejecting a pose
        align.setCorrespondenceRandomness (5); // Number of nearest features to use
        align.setSimilarityThreshold (0.65f); // Polygonal edge length similarity threshold
        align.setMaxCorrespondenceDistance (14.0f ); // Inlier threshold
        align.setInlierFraction (0.05f); // Required inlier fraction for accepting a pose hypothesis
        {
            pcl::ScopeTime t("Alignment");
            align.align (*object_aligned);
        }
        if(align.hasConverged()){
            if(filename=="0"){
                filename="__aligned_wSPIN.pcd";
            } else {
                filename = pcl::getFilenameWithoutExtension(filename).append( "__aligned_wSPIN.pcd" );
            }
            Eigen::Matrix4f *transformation = new Eigen::Matrix4f( align.getFinalTransformation () );
            printResult( *transformation );
            pcl::io::savePCDFile (filename,  *object_aligned, false);

            return transformation;
        }   else {
            pcl::console::print_error ("Alignment failed!\n");
        }
        return nullptr;
        // Perform alignment
    };

    Eigen::Matrix4f *  align_FPFH( pcl::PointCloud<pcl::FPFHSignature33>::Ptr scene_features, pcl::PointCloud<pcl::FPFHSignature33>::Ptr object_features ){

        typedef pcl::FPFHSignature33 FeatureT;
       // typedef pcl::PointCloud<FeatureT> FeatureCloudT;

    pcl::console::print_highlight ("Scene sizes %d %d..\n", this->scene->size(), scene_features->size() );
    pcl::console::print_highlight ("Object sizes %d %d..\n", this->object->size(), object_features->size() );

        pcl::console::print_highlight ("Starting alignment...\n");
        pcl::SampleConsensusPrerejective<PointNT,PointNT,FeatureT> align;
        align.setInputSource (object);
        align.setSourceFeatures (object_features);
        align.setInputTarget (scene);
        align.setTargetFeatures (scene_features);
        align.setMaximumIterations (50000); // Number of RANSAC iterations
        align.setNumberOfSamples (3); // Number of points to sample for generating/prerejecting a pose
        align.setCorrespondenceRandomness (5); // Number of nearest features to use
        align.setSimilarityThreshold (0.55f); // Polygonal edge length similarity threshold
        align.setMaxCorrespondenceDistance (4.0f ); // Inlier threshold
        align.setInlierFraction (0.05f); // Required inlier fraction for accepting a pose hypothesis
        {
            pcl::ScopeTime t("Alignment");
            align.align (*object_aligned);
        }
        if(align.hasConverged()){
            if(filename=="0"){
                filename="__aligned_wFPFH.pcd";
            } else {
                filename = pcl::getFilenameWithoutExtension(filename).append( "__aligned_wFPFH.pcd" );
            }

            Eigen::Matrix4f *transformation = new Eigen::Matrix4f( align.getFinalTransformation () );
            printResult( *transformation );
            saveResult( *transformation );
            return transformation;
        }   else {
            pcl::console::print_error ("Alignment failed!\n");
        }
        return nullptr;
        // Perform alignment
    };

    void saveResult(Eigen::Matrix4f transformation){
        if(cloud2register!= nullptr){
            PointCloudTi::Ptr transformed_cloud(new PointCloudTi );
            pcl::console::print_highlight("Saving aligned cloud with %d points.\n", cloud2register->size() );
            pcl::transformPointCloud (*cloud2register, *transformed_cloud, transformation);
            pcl::io::savePCDFile (filename,  *transformed_cloud, false);
        } else {
            pcl::console::print_warn("No point cloud to transform was provided, will not save any point cloud. Use 'setCloud2Register' if you "
                                          " want to apply the RANSAC transformation to a cloud.\n");
        }
    };
    void printResult(Eigen::Matrix4f transformation){
        printf ("\n");
        pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (0,0), transformation (0,1), transformation (0,2));
        pcl::console::print_info ("R = | %6.3f %6.3f %6.3f | \n", transformation (1,0), transformation (1,1), transformation (1,2));
        pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (2,0), transformation (2,1), transformation (2,2));
        pcl::console::print_info ("\n");
        pcl::console::print_info ("t = < %0.3f, %0.3f, %0.3f >\n", transformation (0,3), transformation (1,3), transformation (2,3));
        pcl::console::print_info ("\n");
        //pcl::console::print_info ("Inliers: %i/%i\n", align.getInliers ().size (), object->size ());

    }

};


#endif //PCAUTOREG_ALIGNRANSAC_H
