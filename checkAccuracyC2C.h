//
// Created by pirotti on 23/04/23.
//

#ifndef PCAUTOREG_CHECKACCURACYC2C_H
#define PCAUTOREG_CHECKACCURACYC2C_H

#include <pcl/filters/crop_box.h>
#include <iostream>
#include <math.h>
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
#include <float.h>
#include "common.h"
//
//120.2 <<  67.3
//337.26 << 41.5
//85.566, 169.85, 63
//308.15, 141.71, 51


typedef pcl::PointXYZ PointNT;
typedef pcl::PointXYZI PointNTi;
typedef pcl::PointCloud<PointNT> PointCloudT;
typedef pcl::PointCloud<PointNTi> PointCloudTi;

class checkAccuracyC2C {

public:

    PointCloudT::Ptr registered ;
    PointCloudT::Ptr reference ;
    PointCloudT::Ptr registered_crop ;
    PointCloudT::Ptr reference_crop ;
    PointCloudT::Ptr  gcp;
    float  radius = 1.0f;
    std::string outputText;
    checkAccuracyC2C () :
            registered(new PointCloudT),
            reference(new PointCloudT),
            registered_crop(new PointCloudT),
            reference_crop (new PointCloudT),
            gcp (new PointCloudT),
            outputText("pt N.\tMean E\tSD E\tRMSE\tN.\n") {};

    void setMasterCloud(PointCloudT::Ptr pc){
        this->reference = pc;
    };
    void setSlaveCloud(PointCloudT::Ptr pc){
        this->registered = pc;
    };

    void addGCP(PointNT pc){
        this->gcp->push_back( pc) ;
    };
    void setRadius(float rad){
        this->radius = rad ;
    };

    void cropInBox(float minx, float miny, float maxx, float maxy,
                   PointCloudT::Ptr input,PointCloudT::Ptr output){
        pcl::CropBox<PointNT> cropBoxFilter (true);
        cropBoxFilter.setInputCloud (input);
        Eigen::Vector4f min_pt (minx, miny, -FLT_MAX, -FLT_MAX);
        Eigen::Vector4f max_pt (maxx, maxy, FLT_MAX, FLT_MAX);

        // Cropbox slighlty bigger then bounding box of points
        cropBoxFilter.setMin (min_pt);
        cropBoxFilter.setMax (max_pt);
        // Cloud
        cropBoxFilter.filter (*output);
    };

    void assess(){

        if(this->gcp->size()==0){
            pcl::console::print_warn("No GCPs provided... did you run 'addGCP'? ");
            return;
        }


        for(int i=0; i < this->gcp->size(); i++){
            cropInBox(this->gcp->points[i].x - radius,  this->gcp->points[i].y - radius,
                      this->gcp->points[i].x + radius,  this->gcp->points[i].y + radius, reference, reference_crop );
            cropInBox(this->gcp->points[i].x - radius,  this->gcp->points[i].y - radius,
                      this->gcp->points[i].x + radius,  this->gcp->points[i].y + radius, registered, registered_crop );

            if(this->registered_crop->size() < 30){
                pcl::console::print_warn("At least 30 points per cropped cloud are required for statistics... \n"
                                         "%d points found in registered cloud -- is your radius large enough to catch points? ", this->registered_crop->size() );
                outputText.append( string_format("pt%2d\tNULL\n", i+1) );
                continue;
            }
            if(this->reference_crop->size() < 30){
                pcl::console::print_warn("At least 30 points per cropped cloud are required for statistics... \n"
                                         "%d points found in reference cloud -- is your radius large enough to catch points? Did you run 'cropInBox'? ", this->reference_crop->size() );
                outputText.append( string_format("pt%2d\tNULL\n", i+1) );
                continue;
            }


            pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

            kdtree.setInputCloud (reference_crop);
            std::vector<float> distances(registered_crop->size());

            pcl::PointCloud<pcl::PointXYZ>::iterator searchPoint;
            for ( searchPoint = registered_crop->points.begin(); searchPoint != registered_crop->points.end (); ++searchPoint){

                std::vector<int> pointIdxRadiusSearch;
                std::vector<float> pointRadiusSquaredDistance;
                if ( kdtree.radiusSearch (*searchPoint , radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
                {
                    distances.push_back( pointRadiusSquaredDistance[0] );
                }
            }
            std::vector<float> stats = calculateStat(distances);
            outputText.append( string_format("pt%02d\t%.3f\t%.3f\t%.3f\t%d\n", i+1, stats[0], stats[1], stats[2], registered_crop->points.size() ) );

        }

    };

};


#endif //PCAUTOREG_CHECKACCURACYC2C_H
