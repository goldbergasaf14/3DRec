/**
 * Set of methods for calculating normals [implementation].
 * Asaf Goldberg & Erez Bursztyn
 * March 2013
 */

#include "../include/NormalCalculator.h"

// METHODS:

	NormalCalculator::NormalCalculator(PointCloud<PointXYZ>::Ptr cloud) :
												_cloud(cloud),
												_normals(new PointCloud<Normal>),
												_kdtree(new search::KdTree<PointXYZ>)
	{}

	/**
	 * Destructor.
	 */
	NormalCalculator::~NormalCalculator()
	{}


	/**
	 * Feeding up the normal estimation fields before calculating.
	 */
	void NormalCalculator::getReady(int k_numberOfNeighbors)
	{
		_kdtree->setInputCloud(_cloud);
		_normalEst.setInputCloud(_cloud);
		_normalEst.setSearchMethod(_kdtree);
		_normalEst.setKSearch(k_numberOfNeighbors);
	}

	/**
	 * Activates the 'compute' method.
	 */
	void NormalCalculator::calculate()
	{
		_normalEst.compute(*_normals);
	}

	/**
	 * Finalizing the calculation while feeding the cloud with the
	 * normals information.
	 */
	PointCloud<PointNormal>::Ptr NormalCalculator::mergeCloudWithNormals()
	{
		PointCloud<PointNormal>::Ptr cloud_with_normals (new PointCloud<PointNormal>);
		concatenateFields(*_cloud, *_normals, *cloud_with_normals);
		return cloud_with_normals;
	}


// End of NormalCalculator.cpp //
