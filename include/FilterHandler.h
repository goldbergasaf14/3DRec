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

using namespace pcl;
using namespace std;

class FilterHandler
{
	public:

	// FIELDS:

		PointCloud<PointXYZ>::Ptr _cloud;	// base cloud
		string _output;						// path to save filtered image

	// METHODS:

		/**
		 * Constructor.
		 * Gets a point cloud and output path as arguments.
		 */
		FilterHandler(PointCloud<PointXYZ>::Ptr cloud, string output);

		/**
		 * Destructor.
		 */
		virtual ~FilterHandler();

		/**
		 * Implements the PassThrough filter.
		 * Gets as arguments - output path, field to filter, from limit and to limit.
		 * Returns a pointer to the filtered cloud.
		 */
		PointCloud<PointXYZ>::Ptr passThroughFilter(string fieldName, double fromFilterLimit, double toFilterLimit);


		// --> any further filter implementations should go here
};

#endif /* FILTERHANDLER_H_ */
