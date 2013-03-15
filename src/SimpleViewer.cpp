/**
 * Basic OpenNI viewer with grabber [implementation].
 * Asaf Goldberg & Erez Bursztyn
 * March 2013
 */

#include "../include/SimpleViewer.h"


// METHODS:

	/**
	 * Constructor.
	 * Gets output name as argument.
	 */
	SimpleViewer::SimpleViewer(string filename) : _viewer ("close window to acquire"),
												  _filename(filename)
	{}

	/**
	 * Destructor.
	 */
    SimpleViewer::~SimpleViewer()
    {
    	//_viewer.~CloudViewer();
    }

	/**
	 * Feeding up the viewer with the updated cloud.
	 */
	void SimpleViewer::cloudView(const PointCloud<PointXYZ>::ConstPtr &cloud)
	{
		if ( !_viewer.wasStopped() )
			_viewer.showCloud(cloud);
		pcl::io::savePCDFile(_filename, *cloud, true);
	}

	/**
	 * Feeding up the viewer with the updated cloud.
	 */
	void SimpleViewer::cloudViewNoSave(const PointCloud<PointXYZ>::ConstPtr &cloud)
	{
		while ( !_viewer.wasStopped() )
			_viewer.showCloud(cloud);
	}

	/**
	 * The life-cycle of the viewer.
	 */
	void SimpleViewer::run()
	{

		Grabber * interface = new OpenNIGrabber();
		boost::function<void (const PointCloud<pcl::PointXYZ>::ConstPtr&)> f =
				boost::bind (&SimpleViewer::cloudView, this, _1);
		interface->registerCallback (f);
		interface->start();
		while ( !_viewer.wasStopped() )
		{
			boost::this_thread::sleep(boost::posix_time::seconds(30));
		}
		interface->stop();
	}


// End of SimpleViewer.cpp //
