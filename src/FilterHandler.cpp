/**
 * Contains different filter methods that available in the PCL library [implementation].
 * Asaf Goldberg & Erez Bursztyn
 * March 2013
 */

#include "../include/FilterHandler.h"

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
	 */
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

// End of FilterHandler.cpp //
