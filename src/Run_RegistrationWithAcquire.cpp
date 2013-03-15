#include "../include/SimpleViewer.h"
#include "../include/FilterHandler.h"
#include "../include/NormalCalculator.h"
#include "../include/MeshHandler.h"
#include "../include/RegistrationHandler.h"

int mainnnn(int argc, char** argv)
{
	cout << "creating first cloud...";
	 MeshHandler * mh1 = new MeshHandler("cup1.pcd");
	 mh1->acquirePCD();
	 mh1->createCloud();
	cout << "DONE" << endl;
	cout << "filtering first cloud...";
     FilterHandler * filter1 = new FilterHandler(mh1->_cloud, "cup1_filtered.pcd");
     mh1->_cloud = filter1->passThroughFilter("z", 0.0, 1, false);
     mh1->_cloud = filter1->voxelGridFilter(0.01f, 0.01f, 0.01f);
	cout << "DONE" << endl;
	cout << "creating second cloud...";
	 MeshHandler * mh2 = new MeshHandler("cup2.pcd");
	 mh2->acquirePCD();
	 mh2->createCloud();
	cout << "DONE" << endl;
	cout << "filtering second cloud...";
     FilterHandler * filter2 = new FilterHandler(mh2->_cloud, "cup2_filtered.pcd");
     mh2->_cloud = filter2->passThroughFilter("z", 0.0, 1, false);
     mh2->_cloud = filter2->voxelGridFilter(0.01f, 0.01f, 0.01f);
	cout << "DONE" << endl;
	cout << "transforming clouds...";
	 RegistrationHandler * rh = new RegistrationHandler(mh1->_cloud, mh2->_cloud);
	 IterativeClosestPoint<PointXYZ, PointXYZ> icp = rh->performICP();
	cout << "DONE" << endl;
	cout << "::building mesh for transformed cloud::" << endl;
	cout << "calculating normals...";
	 MeshHandler * r_mh = new MeshHandler(rh->_result);
	 NormalCalculator * normals = new NormalCalculator(r_mh->_cloud);
	 normals->getReady(20);
	 normals->calculate();
	cout << "DONE" << endl;
	cout << "merging normals with cloud...";
	 r_mh->_cloud_w_normals = normals->mergeCloudWithNormals();
	cout << "DONE" << endl;
	cout << "building the mesh...";
	 PolygonMesh mesh = r_mh->buildMesh("cup-reg.vtk", 0.025, 2.5, 100, M_PI/4, M_PI/18, 2*M_PI/3, false);
	cout << "DONE" << endl;
	 r_mh->showMesh(mesh);
	 return 0;
}
