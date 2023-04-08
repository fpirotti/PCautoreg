//
// Created by pirotti on 07/04/23.
//

#ifndef COFIT_READWRITE_H
#define COFIT_READWRITE_H

#include "lasreader.hpp"
#include "laswriter.hpp"
#include <iostream>

#include <pcl/memory.h>
#include <pcl/pcl_macros.h>
#include <pcl/io/ply/ply_parser.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/common/file_io.h>
#include "pcl/io/ply_io.h"

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


template<typename PointT> inline int las2pcd(std::string filename, pcl::PointCloud<PointT> &cloud)
{
    LASreadOpener lasreadopener;
    lasreadopener.set_file_name( filename.c_str() );
    LASreader* lasreader = lasreadopener.open();
    if(lasreader==0){
        std::cout << "Could not open  " << filename << " - extension: " << pcl::getFileExtension(filename) << std::endl;
        return(-1);
    }
    //LASwriteOpener laswriteopener;
    //laswriteopener.set_file_name("compressed.laz");
    //LASwriter* laswriter = laswriteopener.open(&lasreader->header);

    std::cout << "Reading " << filename << " - extension: " << pcl::getFileExtension(filename) << " \nnpoints=" << lasreader->npoints << std::endl;
    int every = lasreader->npoints/1000;
    int n=0;
    std::cout << "Size before " << cloud.size()  << std::endl;
    cloud.points.resize(lasreader->npoints);
    std::cout << "Size after " << cloud.size()   << std::endl;

    while (lasreader->read_point()) {

        if(n%every==0){
            printProgress( (double)n/lasreader->npoints );
        }
        cloud.points[n].x = lasreader->point.X;
        cloud.points[n].y = lasreader->point.Y;
        cloud.points[n].z = lasreader->point.Z;
       // laswriter->write_point(&lasreader->point);
        n++;
    }
    std::cout << "Finished " << std::endl;
    //laswriter->close();
    //delete laswriter;
    lasreader->close();
    delete lasreader;

    return 0;
}

#endif //COFIT_READWRITE_H
