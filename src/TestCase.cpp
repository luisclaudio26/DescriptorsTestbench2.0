#include "../inc/TestCase.h"
#include "../inc/preprocessing.h"

#include <string>
#include <iostream>

#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/visualization/pcl_visualizer.h>

static void loadCloud(const std::string& path, const std::string& gt, Cloud& out)
{
	std::cout<<"Loading model\n";

	//load model
	pcl::PolygonMesh mesh; pcl::io::loadPolygonFilePLY(path, mesh);

	//initialize point clouds
	out.points.p = pcl::PointCloud<pcl::PointXYZRGB>::Ptr( new pcl::PointCloud<pcl::PointXYZRGB>() );
	out.points.n = pcl::PointCloud<pcl::Normal>::Ptr( new pcl::PointCloud<pcl::Normal>() );
	out.keypoints.p = pcl::PointCloud<pcl::PointXYZRGB>::Ptr( new pcl::PointCloud<pcl::PointXYZRGB>() );
	out.keypoints.n = pcl::PointCloud<pcl::Normal>::Ptr( new pcl::PointCloud<pcl::Normal>() );

	out.meshes = mesh.polygons; //TODO: this is a copy, but maybe could be a move

	pcl::fromPCLPointCloud2<pcl::PointXYZRGB>(mesh.cloud, *(out.points.p));
}

static void preprocessCloud(Cloud& C)
{
	using namespace Preprocessing;

	std::cout<<"Computing basic values\n";
	C.resolution = computeResolution(C.points.p);
	C.area = computeArea(C);
	C.support_radius = computeSupportRadius(C.area);
	std::cout<<"\tRes = "<<C.resolution<<", support radius = "<<C.support_radius<<std::endl;

	std::cout<<"Cleaning outliers\n";
	cleanOutliers(C.points.p, C.support_radius); //THIS OPERATION INVALIDATES THE MESH!

	std::cout<<"Computing normals\n";
	float normal_radius = C.resolution * Parameters::getNormalRadiusFactor();
	computeNormals(C.points.p, normal_radius, C.points.n);
	cleanNaNNormals(C.points.p, C.points.n);

	std::cout<<"Extracting keypoints\n";
	extractKeypoints(C.points, C.resolution, C.support_radius, C.keypoints);
}

//----------------------------------
//-------- FROM TESTCASE.H ---------
//----------------------------------
void TestCase::visualize()
{
	pcl::visualization::PCLVisualizer viewer;
	viewer.setBackgroundColor(0.0f, 0.0f, 0.1f);

	//points
	for(auto p = scene.points.p->begin(); p != scene.points.p->end(); ++p)
		p->r = p->g = p->b = 255;

	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb_scene(scene.points.p);
	viewer.addPointCloud<pcl::PointXYZRGB>(scene.points.p, rgb_scene, "scene");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.0f, "scene");

	//keypoints	
	for(auto p = scene.keypoints.p->begin(); p != scene.keypoints.p->end(); ++p) {
		p->r = p->b = 0;
		p->g = 255;
	}

	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb_scene_kp(scene.keypoints.p);
	viewer.addPointCloud<pcl::PointXYZRGB>(scene.keypoints.p, rgb_scene_kp, "scene_kp");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3.5f, "scene_kp");

	//Won't work :( Check this after
	//viewer.addCorrespondences<pcl::PointXYZRGB>(scene, model, correspondences);

	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::milliseconds(10));
	}
}

void TestCase::preprocess()
{
	preprocessCloud(scene);
	for(auto c = models.begin(); c != models.end(); ++c)
		preprocessCloud(*c);
}

void TestCase::loadTestCasesFromEXP(const std::string& path, std::vector<TestCase>& out)
{
	std::fstream in(path.c_str(), std::ios_base::in);
	out.clear();

	while(in.good() && !in.eof())
	{
		out.push_back( TestCase() ); 
		TestCase& cur = out.back();
		
		std::string scene_name; int n_models;
		in >> scene_name >> n_models;

		cur.name = scene_name;

		//jump straightly to the next line
		in.ignore(10, '\n');
		
		for (int i = 0; i < n_models; i++)
		{
			cur.models.push_back( Cloud() ); 
			Cloud& model = cur.models.back();

			std::string path_model, gt_path;
			std::getline(in, path_model);
			std::getline(in, gt_path);

			loadCloud(path_model, gt_path, model);
		}

		std::string scene_path; std::getline(in, scene_path);
		loadCloud(scene_path, std::string(), cur.scene);
	}

	in.close();
}