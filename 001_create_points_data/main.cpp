// #include <pcl/point_types.h>
// #include <pcl/visualization/pcl_visualizer.h>

#include <string>
#include <tuple>
#include <Eigen/Core>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>

auto read_point_cloud(const std::string& filename, const std::string& filename2){
    const pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
    const int retval = pcl::io::loadPLYFile(filename, *pointcloud);
    if (retval == -1 || pointcloud->size() <= 0)
    {
        PCL_ERROR("File load error.");
        exit(-1);
	}
	else {
		printf("load point cloud\n");
	}

	const pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);
	const int r = pcl::io::loadPLYFile(filename2, *mesh); // http://docs.pointclouds.org/trunk/group__io.html#ga0cfc645cc531647728e16088b6342204
	if (r == -1 || mesh->polygons.size() <= 0) {
		PCL_ERROR("File load error mesh.");
		exit(-1);
	}
	else {
		printf("load mesh\nfaces: %d, vertecies: %d\n", mesh->polygons.size(), mesh->cloud.data.size());
		
	}

	// メッシュ生成 http://pointclouds.org/documentation/tutorials/greedy_projection.php
	// Load input file into a PointCloud<T> with an appropriate type
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	//sensor_msgs::PointCloud2 cloud_blob;
	//pcl::io::loadPCDFile("bun0.pcd", cloud_blob);
	//pcl::fromROSMsg(cloud_blob, *cloud);
	//* the data should be available in cloud
	pcl::io::loadPCDFile("bun0.pcd", *cloud);

	// Normal estimation*
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);
	n.setInputCloud(cloud);
	n.setSearchMethod(tree);
	n.setKSearch(20);
	n.compute(*normals);
	//* normals should not contain the point normals + surface curvatures

	// Concatenate the XYZ and normal fields*
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	//pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
	pcl::concatenateFields<pcl::PointXYZ, pcl::Normal, pcl::PointNormal>(*cloud, *normals, *cloud_with_normals);
	//* cloud_with_normals = cloud + normals

	// Create search tree*
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloud_with_normals);

	// Initialize objects
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	pcl::PolygonMesh triangles;

	// Set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius(0.025);

	// Set typical values for the parameters
	gp3.setMu(2.5);
	gp3.setMaximumNearestNeighbors(100);
	gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
	gp3.setMinimumAngle(M_PI / 18); // 10 degrees
	gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees
	gp3.setNormalConsistency(false);

	// Get result
	gp3.setInputCloud(cloud_with_normals);
	gp3.setSearchMethod(tree2);
	gp3.reconstruct(triangles);

	// Additional vertex information
	std::vector<int> parts = gp3.getPartIDs();
	std::vector<int> states = gp3.getPointStates();

	for (auto v : parts) {
		printf("parts: %d\n", v);
	}
	pcl::io::savePLYFile("output_0.ply", triangles);
	printf("done\n");

    return pointcloud;
}

int main (int argc, char *argv[]){
    const auto scene1 = read_point_cloud("usagi.ply", "usagi_m.ply");
    // メッシュデータがあるとエラー。
    // meshlabのデフォルトではメッシュが無くてもproperty list uchar int vertex_indicesの項目が出るが、これも削除しないと以下のエラー出る
    // [pcl::PLYReader] usagi.ply:9: property 'list uint8 int32 vertex_indices' of element 'face' is not handled
    // ファイルパスが間違っていても以下のエラーが出るので注意 https://github.com/PointCloudLibrary/pcl/issues/2763
    // [pcl::PLYReader] usagi.ply:1: parse error: couldn't read the magic string

    return 1;
}
