//
// Created by pirotti on 14/04/23.
//

#pragma once

#include <pcl/filters/filter.h>
#include <pcl/filters/filter_indices.h>

namespace pcl
{
    /** \brief GridMinimum assembles a local 2D grid over a given PointCloud, and downsamples the data.
      *
      * The GridMinimum class creates a *2D grid* over the input point cloud
      * data. Then, in each *cell* (i.e., 2D grid element), all the points
      * present will be *downsampled* with the minimum z value. This grid minimum
      * can be useful in a number of topographic processing tasks such as crudely
      * estimating ground returns, especially under foliage.
      *
      * \author Bradley J Chambers
      * \ingroup filters
      */
    template <typename PointT>
    class GridMaximum: public FilterIndices<PointT>
    {
    protected:
        using Filter<PointT>::filter_name_;
        using Filter<PointT>::getClassName;
        using Filter<PointT>::input_;
        using Filter<PointT>::indices_;

        using PointCloud = typename FilterIndices<PointT>::PointCloud;

    public:
        /** \brief Empty constructor. */
        GridMaximum (const float resolution)
        {
            setResolution (resolution);
            filter_name_ = "GridMaximum";
        }

        /** \brief Destructor. */
        ~GridMaximum () override = default;

        /** \brief Set the grid resolution.
          * \param[in] resolution the grid resolution
          */
        inline void
        setResolution (const float resolution)
        {
            resolution_ = resolution;
            // Use multiplications instead of divisions
            inverse_resolution_ = 1.0f / resolution_;
        }

        /** \brief Get the grid resolution. */
        inline float
        getResolution () { return (resolution_); }

    protected:
        /** \brief The resolution. */
        float resolution_;

        /** \brief Internal resolution stored as 1/resolution_ for efficiency reasons. */
        float inverse_resolution_;

        /** \brief Downsample a Point Cloud using a 2D grid approach
          * \param[out] output the resultant point cloud message
          */
        void
        applyFilter (PointCloud &output) override;

        /** \brief Filtered results are indexed by an indices array.
          * \param[out] indices The resultant indices.
          */
        void
        applyFilter (Indices &indices) override
        {
            applyFilterIndices (indices);
        }

        /** \brief Filtered results are indexed by an indices array.
          * \param[out] indices The resultant indices.
          */
        void
        applyFilterIndices (Indices &indices);

    };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/filters/impl/grid_maximum.hpp>
#endif
