/**
 * Contains different filter methods that available in the PCL library [implementation].
 * Asaf Goldberg & Erez Bursztyn
 * March 2013
 */

#include "../include/FilterHandler.h"


// METHODS:

	/**
	 * Constructor.
	 * Gets a point cloud and output path as arguments.
	 */
	FilterHandler::FilterHandler(PointCloud<PointXYZ>::Ptr cloud, string output) : _cloud(cloud), _output(output)
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
	PointCloud<PointXYZ>::Ptr FilterHandler::passThroughFilter(string fieldName, double fromFilterLimit, double toFilterLimit)
	{
		PassThrough<PointXYZ> pass;
		pass.setInputCloud(_cloud);
		pass.setFilterFieldName(fieldName);
		pass.setFilterLimits(fromFilterLimit, toFilterLimit);
		pass.filter(*_cloud);
		io::savePCDFile(_output, *_cloud, true);
		return _cloud;
	}

// End of FilterHandler.cpp //
