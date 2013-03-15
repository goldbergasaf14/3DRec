#include "../include/SimpleViewer.h"
#include "../include/FilterHandler.h"
#include "../include/NormalCalculator.h"
#include "../include/MeshHandler.h"

int main$(int argc, char** argv)
{
	cout << "creating cloud...";
	 MeshHandler * mh = new MeshHandler("er2.pcd");
	 mh->createCloud();
	cout << "DONE" << endl;
	cout << "filtering cloud...";
     FilterHandler * filter = new FilterHandler(mh->_cloud, "er2_filtered.pcd");
     mh->_cloud = filter->passThroughFilter("z", 0.0, 1);
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
	 PolygonMesh mesh = mh->buildMesh("er2.vtk", 0.025, 2.5, 100, M_PI/4, M_PI/18, 2*M_PI/3, false);
	cout << "DONE" << endl;
	 mh->showMesh(mesh);
	 return 0;
}
