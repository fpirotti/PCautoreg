/*
===============================================================================

	FILE:  readwrite.cpp

	CONTENTS:

		see corresponding header file

	PROGRAMMERS:


	COPYRIGHT:

		(c) 2023, Francesco Pirotti

		This is free software; you can redistribute and/or modify it under the
		terms of the GNU Lesser General Licence as published by the Free Software
		Foundation. See the LICENSE.txt file for more information.

		This software is distributed WITHOUT ANY WARRANTY and without even the
		implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

	CHANGE HISTORY:

		see corresponding header file

===============================================================================
*/
#include "readwrite.h"


int readPC(std::string filename, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                  bool calcNormals, bool forceCalcNormals, bool savetopcd){
    bool alreadyExists = false;

    auto t1 = std::chrono::high_resolution_clock::now();
    auto t2 = std::chrono::high_resolution_clock::now();

    std::string fileWithoutExt = pcl::getFilenameWithoutExtension(filename).append(".pcd");

    if( std::filesystem::exists( std::filesystem::path(fileWithoutExt) ) ){

        std::cout << "Reading file  " << fileWithoutExt << " exists, reading it... remove file if this is not intended "  << std::endl;

        if(  pcl::io::loadPCDFile( fileWithoutExt, *cloud )==0 ){
            std::cout << " Successfully read " << fileWithoutExt   << std::endl;
            alreadyExists=true;
        }
        return(0);
    }

    if(!alreadyExists && (pcl::getFileExtension(filename)=="las" || pcl::getFileExtension(filename)=="LAS"
                          || pcl::getFileExtension(filename)=="laz" || pcl::getFileExtension(filename)=="LAZ")  )
    {

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

        cloud->points.resize(lasreader->npoints);
        cloud->width = lasreader->npoints;
        cloud->height = 1;

        while (lasreader->read_point()) {

            if (n % every == 0) {
                printProgress((double) n / lasreader->npoints);
            }
            cloud->points[n].x = lasreader->point.X* lasreader->header.x_scale_factor;
            cloud->points[n].y = lasreader->point.Y* lasreader->header.x_scale_factor;
            cloud->points[n].z = lasreader->point.Z* lasreader->header.x_scale_factor;
            cloud->points[n].intensity = lasreader->point.intensity;
            // laswriter->write_point(&lasreader->point);
            n++;
        }

        t2 = std::chrono::high_resolution_clock::now();
        std::cout << "\nFinished reading LAS after " << std::chrono::duration_cast<std::chrono::seconds>(t2 - t1).count() << " seconds. "
                  << std::endl;
        lasreader->close();
        delete lasreader;



        if (savetopcd) {
            std::cout << "Saving to PCD file" << std::endl;
            t1 = std::chrono::high_resolution_clock::now();
            pcl::io::savePCDFile(pcl::getFilenameWithoutExtension(filename).append(".pcd"), *cloud, true);
            t2 = std::chrono::high_resolution_clock::now();
            pcl::console::print_warn(  "Finished saving to %s in %d seconds \n", pcl::getFilenameWithoutExtension(filename).append(std::string(".pcd") ).c_str() ,
                                       std::chrono::duration_cast<std::chrono::seconds>(t2 - t1).count()  );
        }


        return(0);
    }

    return(-1);
}


int getMaxImageWithNormals(std::string filename,
                           pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in,
                            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out,
                            pcl::PointCloud<pcl::Normal>::Ptr cloud_norm){



    auto t1 = std::chrono::high_resolution_clock::now();
    auto t2 = std::chrono::high_resolution_clock::now();

    //pcl::PointXYZI  minPt, maxPt;

    //pcl::getMinMax3D(*cloud_in, minPt, maxPt);

 
    std::cout << "Getting RANGE image" << std::endl;
    t1 = std::chrono::high_resolution_clock::now();


    pcl::GridMaximum<pcl::PointXYZI> maxFilter(1.0f); //(*cloud_in);
    maxFilter.setInputCloud(cloud_in);
    maxFilter.filter(*cloud_out);



    pcl::io::savePCDFile(pcl::getFilenameWithoutExtension(filename).append( std::string("_maxImage.pcd") ),
                         *cloud_out, false);

    t2 = std::chrono::high_resolution_clock::now();
    PCL_WARN  ("Finished calculating max image: saved to  %s after %d seconds \n",
               pcl::getFilenameWithoutExtension(filename).append( std::string("_maxImage.pcd") ).c_str(),
               std::chrono::duration_cast<std::chrono::seconds>(t2 - t1).count() );


    /// --------------------------------------------------------
    /// -----Extract NORMALS of MAX image  ------------ -------
    /// --------------------------------------------------------
    std::cout << " --- Getting NORMALS, this step is required ---" << std::endl;
    t1 = std::chrono::high_resolution_clock::now();

    pcl::NormalEstimationOMP<pcl::PointXYZI,pcl::Normal> nest;
    nest.setRadiusSearch ( 4);
    nest.setInputCloud (cloud_out);
    nest.compute (*cloud_norm);
    t2 = std::chrono::high_resolution_clock::now();
    pcl::io::savePCDFile(pcl::getFilenameWithoutExtension(filename).append( std::string("_MAXImage_wNormals.pcd") ),
                         *cloud_norm, false);
    PCL_WARN  ("Finished calculating NORMALS IN image: saved to  %s after %d seconds \n",
               pcl::getFilenameWithoutExtension(filename).append( std::string("_MAXImage_wNormals.pcd") ).c_str(),
               std::chrono::duration_cast<std::chrono::seconds>(t2 - t1).count() );

    return(0);
}