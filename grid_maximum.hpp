//
// Created by pirotti on 14/04/23.
//

#ifndef PCAUTOREG_GRID_MAXIMUM_HPP
#define PCAUTOREG_GRID_MAXIMUM_HPP


#include <pcl/common/common.h>
#include <pcl/common/io.h>
#include <pcl/common/point_tests.h> // for isXYZFinite
#include "grid_maximum.h"

struct point_index_idx
{
    unsigned int idx;
    unsigned int cloud_point_index;

    point_index_idx (unsigned int idx_, unsigned int cloud_point_index_) : idx (idx_), cloud_point_index (cloud_point_index_) {}
    bool operator < (const point_index_idx &p) const { return (idx < p.idx); }
    bool operator > (const point_index_idx &p) const { return (idx > p.idx); }
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>  void pcl::GridMaximum<PointT>::applyFilter  (PointCloud &output)
{
    // Has the input dataset been set already?
    if (!input_)
    {
        PCL_WARN ("[pcl::%s::applyFilter] No input dataset given!\n", getClassName ().c_str ());
        output.width = output.height = 0;
        output.clear ();
        return;
    }

    Indices indices;

    output.is_dense = true;
    applyFilterIndices (indices);
    pcl::copyPointCloud<PointT> (*input_, indices, output);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::GridMaximum<PointT>::applyFilterIndices (Indices &indices)
{
    indices.resize (indices_->size ());
    int oii = 0;

    // Get the Maximum and maximum dimensions
    Eigen::Vector4f min_p, max_p;
    getMinMax3D<PointT> (*input_, *indices_, min_p, max_p);

    // Check that the resolution is not too small, given the size of the data
    std::int64_t dx = static_cast<std::int64_t> ((max_p[0] - min_p[0]) * inverse_resolution_)+1;
    std::int64_t dy = static_cast<std::int64_t> ((max_p[1] - min_p[1]) * inverse_resolution_)+1;

    if ((dx*dy) > static_cast<std::int64_t> (std::numeric_limits<std::int32_t>::max ()))
    {
        PCL_WARN ("[pcl::%s::applyFilter] Leaf size is too small for the input dataset. Integer indices would overflow.\n", getClassName ().c_str ());
        return;
    }

    Eigen::Vector4i min_b, max_b, div_b, divb_mul;

    // Compute the Maximum and maximum bounding box values
    min_b[0] = static_cast<int> (std::floor (min_p[0] * inverse_resolution_));
    max_b[0] = static_cast<int> (std::floor (max_p[0] * inverse_resolution_));
    min_b[1] = static_cast<int> (std::floor (min_p[1] * inverse_resolution_));
    max_b[1] = static_cast<int> (std::floor (max_p[1] * inverse_resolution_));

    // Compute the number of divisions needed along all axis
    div_b = max_b - min_b + Eigen::Vector4i::Ones ();
    div_b[3] = 0;

    // Set up the division multiplier
    divb_mul = Eigen::Vector4i (1, div_b[0], 0, 0);

    std::vector<point_index_idx> index_vector;
    index_vector.reserve (indices_->size ());

    // First pass: go over all points and insert them into the index_vector vector
    // with calculated idx. Points with the same idx value will contribute to the
    // same point of resulting CloudPoint
    for (const auto& index : (*indices_))
    {
        if (!input_->is_dense)
            // Check if the point is invalid
            if (!isXYZFinite ((*input_)[index]))
                continue;

        int ijk0 = static_cast<int> (std::floor ((*input_)[index].x * inverse_resolution_) - static_cast<float> (min_b[0]));
        int ijk1 = static_cast<int> (std::floor ((*input_)[index].y * inverse_resolution_) - static_cast<float> (min_b[1]));

        // Compute the grid cell index
        int idx = ijk0 * divb_mul[0] + ijk1 * divb_mul[1];
        index_vector.emplace_back(static_cast<unsigned int> (idx), index);
    }

    // Second pass: sort the index_vector vector using value representing target cell as index
    // in effect all points belonging to the same output cell will be next to each other
    std::sort (index_vector.begin (), index_vector.end (), std::less<point_index_idx> ());

    // Third pass: count output cells
    // we need to skip all the same, adjacenent idx values
    unsigned int total = 0;
    unsigned int index = 0;

    // first_and_last_indices_vector[i] represents the index in index_vector of the first point in
    // index_vector belonging to the voxel which corresponds to the i-th output point,
    // and of the first point not belonging to.
    std::vector<std::pair<unsigned int, unsigned int> > first_and_last_indices_vector;

    // Worst case size
    first_and_last_indices_vector.reserve (index_vector.size ());
    while (index < index_vector.size ())
    {
        unsigned int i = index + 1;
        while (i < index_vector.size () && index_vector[i].idx == index_vector[index].idx)
            ++i;
        ++total;
        first_and_last_indices_vector.emplace_back(index, i);
        index = i;
    }

    // Fourth pass: locate grid Maximums
    indices.resize (total);

    index = 0;

    for (const auto &cp : first_and_last_indices_vector)
    {
        unsigned int first_index = cp.first;
        unsigned int last_index = cp.second;
        unsigned int max_index = index_vector[first_index].cloud_point_index;
        float max_z = (*input_)[index_vector[first_index].cloud_point_index].z;

        for (unsigned int i = first_index + 1; i < last_index; ++i)
        {
            if ((*input_)[index_vector[i].cloud_point_index].z > max_z)
            {
                max_z = (*input_)[index_vector[i].cloud_point_index].z;
                max_index = index_vector[i].cloud_point_index;
            }
        }

        indices[index] = max_index;

        ++index;
    }

    oii = indices.size ();

    // Resize the output arrays
    indices.resize (oii);
}



#define PCL_INSTANTIATE_GridMaximum(T) template class PCL_EXPORTS pcl::GridMaximum<T>;

#endif //PCAUTOREG_GRID_MAXIMUM_HPP
