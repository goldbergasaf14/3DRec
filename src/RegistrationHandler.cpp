/**
 * Performing registration algorithms on given clouds [implementation].
 * Asaf Goldberg & Erez Bursztyn
 * March 2013
 */

#include "../include/RegistrationHandler.h"

// METHODS:

	/**
	 * Constructor.
	 * Gets two point clouds as arguments.
	 */
	RegistrationHandler::RegistrationHandler(PointCloud<PointXYZ>::Ptr cloud1, PointCloud<PointXYZ>::Ptr cloud2) :
											_cloud1(cloud1), _cloud2(cloud2)
	{}

	/**
	 * Destructor.
	 */
	RegistrationHandler::~RegistrationHandler()
	{}

	/**
	 * Performing the icp algorithms on the given clouds.
	 * Return icp type with all the
	 */
	IterativeClosestPoint<PointXYZ, PointXYZ> RegistrationHandler::performICP()
	{
		IterativeClosestPoint<PointXYZ, PointXYZ> icp;
		icp.setInputCloud(_cloud1);
		icp.setInputTarget(_cloud2);
		PointCloud<pcl::PointXYZ> final;
		icp.align(final);
		_result = final.makeShared();
		return icp;
	}


// End of RegistrationHandler.cpp //
