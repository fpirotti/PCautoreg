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


int readPC(std::string filename, pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud,
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
                           pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_in,
                            pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_out,
                           // pcl::PointCloud<pcl::Normal>::Ptr cloud_norm,
                            float max_window_res  ){

    auto t1 = std::chrono::high_resolution_clock::now();
    auto t2 = std::chrono::high_resolution_clock::now();

    //pcl::PointXYZINormal  minPt, maxPt;
    //pcl::getMinMax3D(*cloud_in, minPt, maxPt);

    std::cout << "Getting MAX image" << std::endl;
    t1 = std::chrono::high_resolution_clock::now();
    auto outName = pcl::getFilenameWithoutExtension(filename).append( string_format("_maxImage_%06.2f.pcd", max_window_res ) ) ;
    
    if( std::filesystem::exists(outName) )
    {
        std::cout << "MAX image exists, reading it directly!" << std::endl;
        pcl::io::loadPCDFile( outName ,  *cloud_out );

    } else {

        std::cout << "MAX image does NOT exist, creating!" << std::endl;
        pcl::GridMaximum<pcl::PointXYZINormal> maxFilter(max_window_res); //(*cloud_in);
        maxFilter.setInputCloud(cloud_in);
        maxFilter.filter(*cloud_out);
    }

    t2 = std::chrono::high_resolution_clock::now();
    
    PCL_ALWAYS  ("Finished calculating max image using %.2f resolution: saved to  %s after %d seconds \n",
               max_window_res,
               pcl::getFilenameWithoutExtension(filename).append( std::string("_maxImage.pcd") ).c_str(),
               std::chrono::duration_cast<std::chrono::seconds>(t2 - t1).count() );
    
    std::cout << " --- Getting NORMALS, this step is required ---" << std::endl;
    
    pcl::NormalEstimationOMP<pcl::PointXYZINormal,pcl::PointXYZINormal> nest;
    nest.setRadiusSearch ( max_window_res*5 );
    nest.setInputCloud (cloud_out);
    nest.compute (*cloud_out);
    t2 = std::chrono::high_resolution_clock::now();

    PCL_ALWAYS  ("Finished calculating NORMALS  after %d seconds \n", 
               std::chrono::duration_cast<std::chrono::seconds>(t2 - t1).count() );
     
    for (size_t i = 0; i < cloud_out->points.size(); i++) { 
      cloud_out->points[i].intensity = cloud_out->points[i].curvature;
    }
    
    pcl::io::savePCDFile(outName, *cloud_out, false);
    return(0);

}



int las2keypoints( std::string filename,
                     pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_las_gridMax,
                     //pcl::PointCloud<pcl::Normal>::Ptr cloud_Normals_ptr,
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

      const float min_scale = 0.1f;
      const int n_octaves = 3;
      const int n_scales_per_octave = 4;
      const float min_contrast = 0.001f;
      
        pcl::SIFTKeypoint <pcl::PointXYZINormal, pcl::PointWithScale> sift;

        pcl::PointCloud<pcl::PointWithScale> result;
        pcl::search::KdTree<pcl::PointXYZINormal>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZINormal> ());
        sift.setSearchMethod(tree);

        sift.setScales(min_scale, n_octaves, n_scales_per_octave);
        sift.setMinimumContrast(min_contrast);
        sift.setInputCloud(cloud_las_gridMax);
        sift.compute(result);

        int K = result.size();
        
        pcl::console::print_info( "Found %d SIFT KEYPOINTS...\n ", K );
        
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
          appendLineToFile("sift.txt", string_format("%.f;%.f;%.f",pt.x,pt.y,pt.z ) );
          kdtree.nearestKSearch( pt , 1, pointIdxNKNSearch, pointNKNSquaredDistance);
          sift_KeypointsIndices->push_back( pointIdxNKNSearch[0] );
      }

        keypointMap.insert( std::pair<std::string, pcl::IndicesPtr >("SIFT", sift_KeypointsIndices ) );


        pcl::io::savePCDFile(pcl::getFilenameWithoutExtension(filename).append(string_format("_%02d__SIFT.pcd", (int)support_size ) ),
                         result, false);

        t2 = std::chrono::high_resolution_clock::now();

        appendLineToFile("log.txt", string_format(" --- Finished calculating %d SIFT KEYPOINTS  after %d seconds \n",
                                                  keypointMap["SIFT"]->size(),
                                                  std::chrono::duration_cast<std::chrono::seconds>(t2 - t1).count() )  );

        t1 = std::chrono::high_resolution_clock::now();


    /// --------------------------------
    /// -----Extract HARRIS keypoints-----
    /// --------------------------------

       pcl::console::print_warn("Getting harris KEYPOINTS...\n ");
       t1 = std::chrono::high_resolution_clock::now();

       pcl::PointCloud<pcl::PointXYZINormal>::Ptr pp(new pcl::PointCloud<pcl::PointXYZINormal>);
       pcl::HarrisKeypoint3D<pcl::PointXYZINormal, pcl::PointXYZINormal, pcl::Normal> harris;
       pcl::search::KdTree<pcl::PointXYZINormal>::Ptr harris_tree(new pcl::search::KdTree<pcl::PointXYZINormal>());
       harris.setSearchMethod(harris_tree);
       harris.setNonMaxSupression(true);
       harris.setThreshold(1e-6);

       harris.setRadius(5);
       //harris.setNormals(*cloud_las_gridMax);
       harris.setInputCloud(cloud_las_gridMax);
       harris.compute(*pp);
       KEYPOINTS_cloud->clear();
       pcl::copyPointCloud(*pp, *KEYPOINTS_cloud);


       pcl::io::savePCDFile(pcl::getFilenameWithoutExtension(filename).append(string_format("_%02d__HARRIS.pcd", (int)support_size  ) ),
                         *pp, false);

      pcl::IndicesPtr  ind(new pcl::Indices) ;
      pcl::Indices  indConst =      harris.getKeypointsIndices()->indices;
      for(int i=0; i < indConst.size(); i++ ){
          ind->push_back( indConst.at(i)  ) ;
      }


      keypointMap.insert( std::pair<std::string, pcl::IndicesPtr >("HARRIS", ind ) );
      // keypointMap["HARRIS"] = harris.getKeypointsIndices();
       t2 = std::chrono::high_resolution_clock::now();

    appendLineToFile("log.txt", string_format(" --- Finished calculating  %d harris KEYPOINTS  after %d seconds \n",
             keypointMap["HARRIS"]->size(),
            std::chrono::duration_cast<std::chrono::seconds>(t2 - t1).count())  );

    t1 = std::chrono::high_resolution_clock::now();


    /// --------------------------------
    /// -----Extract ISS keypoints-----
    /// --------------------------------

      KEYPOINTS_cloud->clear();
      pcl::console::print_warn("Getting ISS KEYPOINTS...\n ");
      t1 = std::chrono::high_resolution_clock::now();

      // pcl::PointCloud<pcl::PointXYZ>::Ptr pp_iss (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::ISSKeypoint3D<pcl::PointXYZINormal, pcl::PointXYZ> iss;
      pcl::search::KdTree<pcl::PointXYZINormal>::Ptr iss_tree(new pcl::search::KdTree<pcl::PointXYZINormal>());
      iss.setSearchMethod(iss_tree);
      iss.setSalientRadius(support_size / 9);
      iss.setNonMaxRadius(1);
      //iss.setNormals(cloud_Normals_ptr);
      iss.setInputCloud(cloud_las_gridMax);
      iss.compute(*KEYPOINTS_cloud);
      //pcl::copyPointCloud(*pp, *ISS_keypointsCloud_ptr);
      pcl::io::savePCDFile(pcl::getFilenameWithoutExtension(filename).append(string_format("_%02d__ISS.pcd", (int)support_size ) ),
                           *KEYPOINTS_cloud, false);

      pcl::IndicesPtr  ind_iss(new pcl::Indices) ;
      pcl::Indices  indConst_iss =      iss.getKeypointsIndices()->indices;
      for(int i=0; i < indConst_iss.size(); i++ ){
          ind_iss->push_back( indConst_iss.at(i)  ) ;
      }

     // keypointMap["ISS"] = iss.getKeypointsIndices() ;
      keypointMap.insert( std::pair<std::string, pcl::IndicesPtr >("ISS", ind_iss) );
      t2 = std::chrono::high_resolution_clock::now();

      appendLineToFile("log.txt", string_format(" --- Finished calculating %d ISS KEYPOINTS  after %d seconds \n",
                                              keypointMap["ISS"]->size(),
                                              std::chrono::duration_cast<std::chrono::seconds>(t2 - t1).count() )  );
      t1 = std::chrono::high_resolution_clock::now();

    return(0);

}