/**
 * Set of methods for calculating normals.
 * Asaf Goldberg & Erez Bursztyn
 * March 2013
 */

#ifndef NORMALCALCULATOR_H_
#define NORMALCALCULATOR_H_

#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>

using namespace pcl;

class NormalCalculator
{
	public:

	// FIELDS:

		PointCloud<PointXYZ>::Ptr _cloud;				// base cloud
		NormalEstimation<PointXYZ, Normal> _normalEst;	// normal estimation
		PointCloud<Normal>::Ptr _normals;				// normals cloud
		search::KdTree<PointXYZ>::Ptr _kdtree;			// kdtree

	// METHODS:

		/**
		 * Constructor.
		 * Gets point cloud as argument.
		 */
		NormalCalculator(PointCloud<PointXYZ>::Ptr cloud);

		/**
		 * Destructor.
		 */
		virtual ~NormalCalculator();

		/**
		 * Feeding up the normal estimation fields before calculating.
		 */
		void getReady(int k_numberOfNeighbors);

		/**
		 * Activates the 'compute' method.
		 */
		void calculate();

		/**
		 * Finalizing the calculation while feeding the cloud with the
		 * normals information.
		 */
		PointCloud<PointNormal>::Ptr mergeCloudWithNormals();


};

#endif /* NORMALCALCULATOR_H_ */
