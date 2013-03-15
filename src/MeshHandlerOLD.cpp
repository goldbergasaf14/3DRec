/**
 * Handles basic mesh operations.
 * Asaf Goldberg & Erez Bursztyn
 * March 2013
 */

#include <iostream>
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
#include <pcl/filters/passthrough.h>
#include "../include/SimpleViewer.h"
#include "../include/FilterHandler.h"
#include "../include/NormalCalculator.h"

using namespace pcl;
using namespace std;

/**
 * Starting the standard viewer and saves pcd file when closing it.
 * Returns cloud of the acquired image.
 * filename - Name of pcd file to save.
 */
void acquirePCD (string filename)
{
    SimpleViewer viewer(filename);
    viewer.run();
}

/**
 * Creating a point cloud from the given pcd file.
 */
PointCloud<PointXYZ>::Ptr createCloud(string filename)
{
	PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);
	sensor_msgs::PointCloud2 cloud_blob;
	io::loadPCDFile(filename, cloud_blob);
	fromROSMsg(cloud_blob, *cloud);
	return cloud;
}

/**
 * Building the mesh using the triangles algorithm.
 * Gets the algorithms parameters as arguments.
 */
PolygonMesh buildMesh(PointCloud<PointNormal>::Ptr cloud, string output, double searchRadius,
																		 double multiplierNeighbors,
																		 int maxNearestNeighbors,
																		 double maximumSurfaceAngle,
																		 double minimumAngle,
																		 double maximumAngle,
																		 bool normalConsistency)
{
	  search::KdTree<PointNormal>::Ptr kdtree (new search::KdTree<PointNormal>);
	  kdtree->setInputCloud(cloud);
	  GreedyProjectionTriangulation<PointNormal> gp3;
	  PolygonMesh triangles;
	  gp3.setSearchRadius(searchRadius);
	  gp3.setMu(multiplierNeighbors);
	  gp3.setMaximumNearestNeighbors(maxNearestNeighbors);
	  gp3.setMaximumSurfaceAngle(maximumSurfaceAngle);
	  gp3.setMinimumAngle(minimumAngle);
	  gp3.setMaximumAngle(maximumAngle);
	  gp3.setNormalConsistency(normalConsistency);
	  gp3.setInputCloud(cloud);
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
void showMesh(PolygonMesh mesh)
{
	visualization::PCLVisualizer * mv = new visualization::PCLVisualizer("Mesh View");
	mv->addPolygonMesh(mesh, "Mesh");
	while(!mv->wasStopped())
	{
		mv->spin();
	}
}

int mainOLD(int argc, char** argv)
{
		cout << "creating cloud...";
	PointCloud<PointXYZ>::Ptr cloud = createCloud("bun.pcd");
		cout << "DONE" << endl;
		cout << "filtering cloud...";
	FilterHandler * filter = new FilterHandler(cloud, "bun_filtered.pcd");
	cloud = filter->passThroughFilter("z", 0.0, 1);
		cout << "DONE" << endl;
		cout << "calculating normals...";
	NormalCalculator * normals = new NormalCalculator(cloud);
	normals->getReady(20);
	normals->calculate();
		cout << "DONE" << endl;
		cout << "merging normals with cloud...";
	PointCloud<PointNormal>::Ptr cloud_w_normals = normals->mergeCloudWithNormals();
		cout << "DONE" << endl;
		cout << "building the mesh...";
	PolygonMesh mesh = buildMesh(cloud_w_normals, "bun.vtk", 0.025, 2.5, 100, M_PI/4, M_PI/18, 2*M_PI/3, false);
		cout << "DONE" << endl;
	showMesh(mesh);
	return 0;
}
