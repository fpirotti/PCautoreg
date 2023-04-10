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


template<typename PointT> inline int readPC(std::string filename, pcl::PointCloud<PointT> &cloud){


    std::cout << "Reading " << filename << " - extension: " << pcl::getFileExtension(filename) << std::endl;
    if( pcl::getFileExtension(filename)=="las"  ){
        std::cout << "Reading " << filename << " - extension: " << pcl::getFileExtension(filename) << std::endl;

    }
    return(0);

    if (pcl::io::loadPLYFile<pcl::PointXYZ>(filename, *cloud) == -1) // load the file
    {
        PCL_ERROR ("Couldn't read PLY file\n");
        return -1;
    }


    if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *cloud) == -1) // load the file
    {
        PCL_ERROR ("Couldn't read file\n");
        return -1;
    }
}


