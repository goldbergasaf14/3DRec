/**
 * Performing registration algorithms on given clouds.
 * Asaf Goldberg & Erez Bursztyn
 * March 2013
 */

#ifndef REGISTRATIONHANDLER_H_
#define REGISTRATIONHANDLER_H_

#include <pcl/registration/icp.h>

using namespace pcl;
using namespace std;


class RegistrationHandler
{
	public:

	// FIELDS:

		PointCloud<PointXYZ>::Ptr _cloud1;		// point cloud 1
		PointCloud<PointXYZ>::Ptr _cloud2;		// point cloud 2
		PointCloud<PointXYZ>::Ptr _result;		// result point cloud

	// METHODS:

		/**
		 * Constructor.
		 * Gets two point clouds as arguments.
		 */
		RegistrationHandler(PointCloud<PointXYZ>::Ptr cloud1, PointCloud<PointXYZ>::Ptr cloud2);

		/**
		 * Destructor.
		 */
		virtual ~RegistrationHandler();

		/**
		 * Performing the icp algorithms on the given clouds.
		 * Return icp type with all the
		 */
		IterativeClosestPoint<PointXYZ, PointXYZ> performICP();

		// --> any further registration algorithms implementations should go here.

};

#endif /* REGISTRATIONHANDLER_H_ */

