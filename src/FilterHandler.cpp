/**
 * Contains different filter methods that available in the PCL library [implementation].
 * Asaf Goldberg & Erez Bursztyn
 * March 2013
 */

#include "../include/FilterHandler.h"
#include <pcl/filters/filter.h>


typedef typename pcl::PointXYZ pointType;


// METHODS:

	/**
	 * Constructor.
	 * Gets a point cloud and output path as arguments.
	 */
	FilterHandler::FilterHandler(PointCloud<pointType>::Ptr cloud, string output) : _cloud(cloud), _output(output)
	{}

	/**
	 * Destructor.
	 */
	FilterHandler::~FilterHandler()
	{}

	/**
	 * Implements the PassThrough filter.
	 * Gets as arguments - output path, field to filter, from limit and to limit.
	 * Creating the output on the disk and returns filtered point cloud.
	 */
	PointCloud<pointType>::Ptr FilterHandler::passThroughFilter(string fieldName,
																double fromFilterLimit,
																double toFilterLimit,
																bool negativeLimits)
	{
		PassThrough<pointType> pass;
		pass.setInputCloud(_cloud);
		pass.setFilterFieldName(fieldName);
		pass.setFilterLimits(fromFilterLimit, toFilterLimit);
		pass.setFilterLimitsNegative(negativeLimits);
		pass.filter(*_cloud);
		io::savePCDFile(_output, *_cloud, true);
		return _cloud;
	}


	/**
	 * Implements the Voxel Grid Filter.
	 * Gets the leafs size as arguments (floating point).
	 * Returns a pointer to the filtered cloud.
	 */
	PointCloud<pointType>::Ptr FilterHandler::voxelGridFilter(float xLeafSize,
											   	   	   	      float yLeafSize,
											   	   	   	      float zLeafSize)
	{
		VoxelGrid<pointType> sor;
		sor.setInputCloud(_cloud);
		sor.setLeafSize(xLeafSize, yLeafSize, zLeafSize);
		sor.filter(*_cloud);
		io::savePCDFile(_output, *_cloud, true);
		return _cloud;
	}

	/**
	 * Implements the Median Filter.
	 * Gets the largest value one dexel is allowed to move as argument (floating point), and the window size (integer).
	 * Returns a pointer to the filtered cloud.
	 * Relies on the PCL 1.7 implementation. [not exists in 1.6]
	 */
	PointCloud<pointType>::Ptr FilterHandler::medianFilter(float maxAllowedMovement, int windowSize)
	{
		PointCloud<pointType>::Ptr original = _cloud->makeShared(); 		// getting the cloud from pointer
		PointCloud<pointType>::Ptr filtered (new PointCloud<pointType>);	// will hold the filtered cloud

		// Copy everything from the original cloud to a new cloud (will be the filtered)
		copyPointCloud (*original, *filtered);

		for (int y = 0; y < filtered->height; ++y)
		for (int x = 0; x < filtered->width; ++x)
		  if (pcl::isFinite ((*original)(x, y)))
		  {
			std::vector<float> vals;
			vals.reserve (windowSize * windowSize);
			// Fill in the vector of values with the depths around the interest point
			for (int y_dev = -windowSize/2; y_dev <= windowSize/2; ++y_dev)
			  for (int x_dev = -windowSize/2; x_dev <= windowSize/2; ++x_dev)
			  {
				if (x + x_dev >= 0 && x + x_dev < filtered->width &&
					y + y_dev >= 0 && y + y_dev < filtered->height &&
					pcl::isFinite ((*original)(x+x_dev, y+y_dev)))
				  vals.push_back ((*original)(x+x_dev, y+y_dev).z);
			  }

			if (vals.size () == 0)
			  continue;

			// The output depth will be the median of all the depths in the window
			partial_sort (vals.begin (), vals.begin () + vals.size () / 2 + 1, vals.end ());
			float new_depth = vals[vals.size () / 2];
			// Do not allow points to move more than the set max_allowed_movement_
			if (fabs (new_depth - (*original)(x, y).z) < maxAllowedMovement)
			  filtered->operator ()(x, y).z = new_depth;
			else
			  filtered->operator ()(x, y).z = (*original)(x, y).z +
								 maxAllowedMovement * (new_depth - (*original)(x, y).z) / fabsf (new_depth - (*original)(x, y).z);
		  }
		io::savePCDFile(_output, *filtered, true);
		return filtered;
	}


// End of FilterHandler.cpp //
