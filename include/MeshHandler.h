/**
 * Handling the process to create a mesh from pcd file.
 * Asaf Goldberg & Erez Bursztyn
 * March 2013
 */

#ifndef MESHHANDLER_H_
#define MESHHANDLER_H_

#include <string>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <vtkSmartPointer.h>
#include <vtkSmartPointerBase.h>
#include "../include/SimpleViewer.h"
#include "../include/FilterHandler.h"
#include "../include/NormalCalculator.h"

using namespace pcl;
using namespace std;


class MeshHandler
{
	public:

	// FIELDS:

		string _filename; 								// path to pcd file
		PointCloud<PointXYZ>::Ptr _cloud;				// point cloud
		PointCloud<PointNormal>::Ptr _cloud_w_normals; 	// point cloud with normals

	// METHODS:

		/**
		 * Empty constructor.
		 */
		MeshHandler();

		/**
		 * Constructor.
		 * Gets filename as argument.
		 */
		MeshHandler(string filename);

		/**
		 * Constructor.
		 * Gets cloud as argument.
		 */
		MeshHandler(PointCloud<PointXYZ>::Ptr cloud);

		/**
		 * Destructor.
		 */
		virtual ~MeshHandler();

		/**
		 * Starting the standard viewer and saves pcd file when closing it.
		 * Returns cloud of the acquired image.
		 */
		void acquirePCD ();

		/**
		 * Creating a point cloud from the given pcd file.
		 */
		PointCloud<PointXYZ>::Ptr createCloud();

		/**
		 * Building the mesh using the triangles algorithm.
		 * Gets the algorithms parameters as arguments.
		 */
		PolygonMesh buildMesh(string output, double searchRadius,
											 double multiplierNeighbors,
											 int maxNearestNeighbors,
											 double maximumSurfaceAngle,
											 double minimumAngle,
											 double maximumAngle,
											 bool normalConsistency);

		/**
		 * Open visualization window displaying the given mesh argument.
		 */
		void showMesh(PolygonMesh mesh);

};

#endif /* MESHHANDLER_H_ */
