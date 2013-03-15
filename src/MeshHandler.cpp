/**
 * Handling the process to create a mesh from pcd file [implementation].
 * Asaf Goldberg & Erez Bursztyn
 * March 2013
 */

#include "../include/MeshHandler.h"

// METHODS:

	/**
	 * Empty constructor.
	 */
	MeshHandler::MeshHandler()
	{}

	/**
	 * Constructor.
	 * Gets filename as argument.
	 */
	MeshHandler::MeshHandler(string filename) : _filename(filename), _cloud(new PointCloud<PointXYZ>)
	{}

	/**
	 * Constructor.
	 * Gets cloud as argument.
	 */
	MeshHandler::MeshHandler(PointCloud<PointXYZ>::Ptr cloud) : _cloud(cloud)
	{}

	/**
	 * Destructor.
	 */
	MeshHandler::~MeshHandler()
	{}

	/**
	 * Starting the standard viewer and saves pcd file when closing it.
	 * Returns cloud of the acquired image.
	 */
	void MeshHandler::acquirePCD()
	{
	    SimpleViewer viewer(_filename);
	    viewer.run();
	}

	/**
	 * Creating a point cloud from the given pcd file.
	 */
	PointCloud<PointXYZ>::Ptr MeshHandler::createCloud()
	{
		sensor_msgs::PointCloud2 cloud_blob;
		io::loadPCDFile(_filename, cloud_blob);
		fromROSMsg(cloud_blob, *_cloud);
		return _cloud;
	}

	/**
	 * Building the mesh using the triangles algorithm.
	 * Gets the algorithms parameters as arguments.
	 */
	PolygonMesh MeshHandler::buildMesh(string output, double searchRadius,
													  double multiplierNeighbors,
													  int maxNearestNeighbors,
													  double maximumSurfaceAngle,
													  double minimumAngle,
													  double maximumAngle,
													  bool normalConsistency)
	{
		  search::KdTree<PointNormal>::Ptr kdtree (new search::KdTree<PointNormal>);
		  kdtree->setInputCloud(_cloud_w_normals);
		  GreedyProjectionTriangulation<PointNormal> gp3;
		  PolygonMesh triangles;
		  gp3.setSearchRadius(searchRadius);
		  gp3.setMu(multiplierNeighbors);
		  gp3.setMaximumNearestNeighbors(maxNearestNeighbors);
		  gp3.setMaximumSurfaceAngle(maximumSurfaceAngle);
		  gp3.setMinimumAngle(minimumAngle);
		  gp3.setMaximumAngle(maximumAngle);
		  gp3.setNormalConsistency(normalConsistency);
		  gp3.setInputCloud(_cloud_w_normals);
		  gp3.setSearchMethod (kdtree);
		  gp3.reconstruct (triangles);
		  vector<int> parts = gp3.getPartIDs();
		  vector<int> states = gp3.getPointStates();
		  io::saveVTKFile(output, triangles);
		  return triangles;
	}

	/**
	 * Open visualization window displaying the given mesh argument.
	 */
	void MeshHandler::showMesh(PolygonMesh mesh)
	{
		visualization::PCLVisualizer * mv = new visualization::PCLVisualizer("Mesh View");
		mv->addPolygonMesh(mesh, "Mesh");
		while(!mv->wasStopped())
		{
			mv->spin();
		}
		delete mv;
	}


// End of MeshHandler.cpp //
