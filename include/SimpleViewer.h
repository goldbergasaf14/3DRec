/**
 * Basic OpenNI viewer with grabber.
 * Asaf Goldberg & Erez Bursztyn
 * March 2013
 */

#ifndef SIMPLEVIEWER_H_
#define SIMPLEVIEWER_H_

#include <iostream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace pcl;
using namespace std;

class SimpleViewer
{
	public:

	// FIELDS:

		visualization::CloudViewer _viewer; // basic viewer
		string _filename; 					// output file name

	// METHODS:

		/**
		 * Constructor.
		 * Gets output name as argument.
		 */
		SimpleViewer(string filename);

		/**
		 * Destructor.
		 */
		virtual ~SimpleViewer();

		/**
		 * Feeding up the viewer with the updated cloud.
		 */
		void cloudView(const PointCloud<PointXYZ>::ConstPtr &cloud);

		/**
		 * The life-cycle of the viewer.
		 */
		void run();


};

#endif /* SIMPLEVIEWER_H_ */
