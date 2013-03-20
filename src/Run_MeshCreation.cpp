#include "../include/SimpleViewer.h"
#include "../include/FilterHandler.h"
#include "../include/NormalCalculator.h"
#include "../include/MeshHandler.h"

int main(int argc, char** argv)
{
	cout << "creating cloud...";
	 MeshHandler * mh = new MeshHandler("cup1.pcd");
	 mh->createCloud();
	cout << "DONE" << endl;
	cout << "filtering cloud...";
     FilterHandler * filter = new FilterHandler(mh->_cloud, "cup1_filtered.pcd");
     //mh->_cloud = filter->passThroughFilter("y", -1, 1, false);
     //mh->_cloud = filter->voxelGridFilter(0.01f, 0.01f, 0.01f);
     mh->_cloud = filter->medianFilter(10 ,5);
     mh->_cloud = filter->passThroughFilter("z", -1, 1, false);
     mh->_cloud = filter->passThroughFilter("x", -1, 1, false);
     mh->_cloud = filter->voxelGridFilter(0.01f, 0.01f, 0.01f);
	cout << "DONE" << endl;

	cout << "calculating normals...";
	 NormalCalculator * normals = new NormalCalculator(mh->_cloud);
	 normals->getReady(20);
	 normals->calculate();
	cout << "DONE" << endl;
	cout << "merging normals with cloud...";
	 mh->_cloud_w_normals = normals->mergeCloudWithNormals();
	cout << "DONE" << endl;
	cout << "building the mesh...";
	 PolygonMesh mesh = mh->buildMesh("cup1.vtk", 0.025, 2.5, 100, M_PI/4, M_PI/18, 2*M_PI/3, false);
	cout << "DONE" << endl;
	 mh->showMesh(mesh);
	 return 0;
}
