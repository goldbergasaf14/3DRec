/**
 * Contains different filter methods that available in the PCL library.
 * Asaf Goldberg & Erez Bursztyn
 * March 2013
 */

#ifndef FILTERHANDLER_H_
#define FILTERHANDLER_H_

#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/pcl_base.h>
#include "../include/median_filter.h"

using namespace pcl;
using namespace std;

class FilterHandler
{



	typedef typename pcl::PointXYZ pointType;

	public:

	// FIELDS:

		PointCloud<pointType>::Ptr _cloud;	// base cloud
		string _output;						// path to save filtered image


	// METHODS:

		/**
		 * Constructor.
		 * Gets a point cloud and output path as arguments.
		 */
		FilterHandler(PointCloud<pointType>::Ptr cloud, string output);

		/**
		 * Destructor.
		 */
		virtual ~FilterHandler();

		/**
		 * Implements the PassThrough filter.
		 * Gets as arguments - output path, field to filter, from limit and to limit.
		 * Returns a pointer to the filtered cloud.
		 */
		PointCloud<pointType>::Ptr passThroughFilter(string fieldName,
													 double fromFilterLimit,
													 double toFilterLimit,
													 bool negativeLimits);

		/**
		 * Implements the Voxel Grid Filter.
		 * Gets the leafs size as arguments (floating point).
		 * Returns a pointer to the filtered cloud.
		 */
		PointCloud<pointType>::Ptr voxelGridFilter(float xLeafSize,
												   float yLeafSize,
												   float zLeafSize);

		/**
		 * Implements the Median Filter.
		 * Gets the largest value one dexel is allowed to move as argument (floating point), and the window size (integer).
		 * Returns a pointer to the filtered cloud.
		 */
		PointCloud<pointType>::Ptr medianFilter(float maxAllowedMovement, int windowSize);

		// --> any further filter implementations should go here
};

#endif /* FILTERHANDLER_H_ */
