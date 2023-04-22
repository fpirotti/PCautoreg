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
                            pcl::PointCloud<pcl::Normal>::Ptr cloud_norm,
                            float max_window_res  ){

    auto t1 = std::chrono::high_resolution_clock::now();
    auto t2 = std::chrono::high_resolution_clock::now();

    //pcl::PointXYZI  minPt, maxPt;
    //pcl::getMinMax3D(*cloud_in, minPt, maxPt);

    std::cout << "Getting RANGE image" << std::endl;
    t1 = std::chrono::high_resolution_clock::now();

    if( std::filesystem::exists( pcl::getFilenameWithoutExtension(filename).append( std::string("_maxImage.pcd") ) ) )
    {
        pcl::io::loadPCDFile( pcl::getFilenameWithoutExtension(filename).append( std::string("_maxImage.pcd") ) ,  *cloud_out );

    } else {

        pcl::GridMaximum<pcl::PointXYZI> maxFilter(max_window_res); //(*cloud_in);
        maxFilter.setInputCloud(cloud_in);
        maxFilter.filter(*cloud_out);

        pcl::io::savePCDFile(pcl::getFilenameWithoutExtension(filename).append( std::string("_maxImage.pcd") ),
                             *cloud_out, false);

    }

    t2 = std::chrono::high_resolution_clock::now();
    PCL_WARN  ("Finished calculating max image using %.2f resolution: saved to  %s after %d seconds \n",
               max_window_res,
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



int las2keypoints( std::string filename,
                     pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_las_gridMax,
                     pcl::PointCloud<pcl::Normal>::Ptr cloud_Normals_ptr,
                     std::map<std::string,  pcl::IndicesPtr  > &keypointMap,
                      float support_size ) {




      pcl::PointCloud<pcl::PointXYZ>::Ptr KEYPOINTS_cloud   (new pcl::PointCloud<pcl::PointXYZ>);
      auto t1 = std::chrono::high_resolution_clock::now();
      auto t2 = std::chrono::high_resolution_clock::now();

      /// --------------------------------
      /// -----Extract SIFT keypoints-----
      /// --------------------------------

      pcl::console::print_warn( "Getting SIFT KEYPOINTS...\n " );
      t1 = std::chrono::high_resolution_clock::now();

        constexpr float min_scale = 2.0f;
        constexpr int n_octaves = 6;
        constexpr int n_scales_per_octave = 10;
        constexpr float min_contrast = 5.0f;
        pcl::SIFTKeypoint <pcl::PointXYZI, pcl::PointWithScale> sift;

        pcl::PointCloud<pcl::PointWithScale> result;
        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI> ());
        sift.setSearchMethod(tree);

        sift.setScales(min_scale, n_octaves, n_scales_per_octave);
        sift.setMinimumContrast(min_contrast);
        sift.setInputCloud(cloud_las_gridMax);
        sift.compute(result);

        int K = result.size();
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

      std::vector<int> pointIdxNKNSearch(1);
      std::vector<float> pointNKNSquaredDistance(1 );
      pcl::PointCloud<pcl::PointXYZ>::Ptr tmpcloudXYZ (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::copyPointCloud(*cloud_las_gridMax, *tmpcloudXYZ);
      kdtree.setInputCloud (tmpcloudXYZ);
      pcl::IndicesPtr sift_KeypointsIndices(new pcl::Indices) ;

      for(int i=0; i<result.size(); i++){
          pcl::PointXYZ  pt;
          pt.x = result.points[i].x;
          pt.y = result.points[i].y;
          pt.z = result.points[i].z;
          kdtree.nearestKSearch( pt , 1, pointIdxNKNSearch, pointNKNSquaredDistance);
          sift_KeypointsIndices->push_back( pointIdxNKNSearch[0] );
      }

        keypointMap.insert( std::pair<std::string, pcl::IndicesPtr >("SIFT", sift_KeypointsIndices ) );

        pcl::io::savePCDFile (pcl::getFilenameWithoutExtension(filename).append( "__SIFT.pcd" ),
                              result, false);

        t2 = std::chrono::high_resolution_clock::now();
        PCL_WARN  (" --- Finished calculating %d SIFT KEYPOINTS  after %d seconds \n",
                   result.size (),
                   std::chrono::duration_cast<std::chrono::seconds>(t2 - t1).count() );
        t1 = std::chrono::high_resolution_clock::now();


    /// --------------------------------
    /// -----Extract HARRIS keypoints-----
    /// --------------------------------

       pcl::console::print_warn("Getting harris KEYPOINTS...\n ");
       t1 = std::chrono::high_resolution_clock::now();

       pcl::PointCloud<pcl::PointXYZI>::Ptr pp(new pcl::PointCloud<pcl::PointXYZI>);
       pcl::HarrisKeypoint3D<pcl::PointXYZI, pcl::PointXYZI, pcl::Normal> harris;
       pcl::search::KdTree<pcl::PointXYZI>::Ptr harris_tree(new pcl::search::KdTree<pcl::PointXYZI>());
       harris.setSearchMethod(harris_tree);
       harris.setNonMaxSupression(true);
       harris.setThreshold(1e-6);

       harris.setRadius(5);
       harris.setNormals(cloud_Normals_ptr);
       harris.setInputCloud(cloud_las_gridMax);
       harris.compute(*pp);
       KEYPOINTS_cloud->clear();
       pcl::copyPointCloud(*pp, *KEYPOINTS_cloud);
       pcl::io::savePCDFile(pcl::getFilenameWithoutExtension(filename).append("__HARRIS.pcd"),
                            *pp, false);
      pcl::IndicesPtr  ind(new pcl::Indices) ;
      pcl::Indices  indConst =      harris.getKeypointsIndices()->indices;
      for(int i=0; i < indConst.size(); i++ ){
          ind->push_back( indConst.at(i)  ) ;
      }


      keypointMap.insert( std::pair<std::string, pcl::IndicesPtr >("HARRIS", ind ) );
      // keypointMap["HARRIS"] = harris.getKeypointsIndices();
       t2 = std::chrono::high_resolution_clock::now();
       PCL_WARN  (" --- Finished calculating %d == %d harris KEYPOINTS  after %d seconds \n",
                  pp->size(), keypointMap["HARRIS"]->size(),
                  std::chrono::duration_cast<std::chrono::seconds>(t2 - t1).count());

    t1 = std::chrono::high_resolution_clock::now();


    /// --------------------------------
    /// -----Extract ISS keypoints-----
    /// --------------------------------

      KEYPOINTS_cloud->clear();
      pcl::console::print_warn("Getting ISS KEYPOINTS...\n ");
      t1 = std::chrono::high_resolution_clock::now();

      // pcl::PointCloud<pcl::PointXYZ>::Ptr pp_iss (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::ISSKeypoint3D<pcl::PointXYZI, pcl::PointXYZ> iss;
      pcl::search::KdTree<pcl::PointXYZI>::Ptr iss_tree(new pcl::search::KdTree<pcl::PointXYZI>());
      iss.setSearchMethod(iss_tree);
      iss.setSalientRadius(support_size / 9);
      iss.setNonMaxRadius(1);
      iss.setNormals(cloud_Normals_ptr);
      iss.setInputCloud(cloud_las_gridMax);
      iss.compute(*KEYPOINTS_cloud);
      //pcl::copyPointCloud(*pp, *ISS_keypointsCloud_ptr);
      pcl::io::savePCDFile(pcl::getFilenameWithoutExtension(filename).append("__ISS.pcd"),
                           *KEYPOINTS_cloud, false);

      pcl::IndicesPtr  ind_iss(new pcl::Indices) ;
      pcl::Indices  indConst_iss =      iss.getKeypointsIndices()->indices;
      for(int i=0; i < indConst_iss.size(); i++ ){
          ind_iss->push_back( indConst_iss.at(i)  ) ;
      }

     // keypointMap["ISS"] = iss.getKeypointsIndices() ;
      keypointMap.insert( std::pair<std::string, pcl::IndicesPtr >("ISS", ind_iss) );
      t2 = std::chrono::high_resolution_clock::now();
      PCL_WARN  (" --- Finished calculating %d ISS KEYPOINTS  after %d seconds \n",
                 KEYPOINTS_cloud->size(),
                 std::chrono::duration_cast<std::chrono::seconds>(t2 - t1).count());
      t1 = std::chrono::high_resolution_clock::now();

    return(0);

}