#ifndef _TEST_CASE_H_
#define _TEST_CASE_H_

#include <pcl/PolygonMesh.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <vector>

//Structure for Precision-Recall curve
typedef struct {
	float p, r;
} PREntry;

typedef struct {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr points;
	std::vector<pcl::Vertices> meshes;
	float area, resolution, support_radius;

	//if Groundtruth is an identity matrix 
	//(check with .isIdentity()), then it is not
	//valid. It will happen for target clouds
	//(that is, a scene). Otherwise, groundtruth
	//maps this cloud to its position on the scene.
	Eigen::Matrix4f groundtruth;
} Cloud;

class TestCase
{
private:
	Cloud scene; std::vector<Cloud> models;

	//the name of this test case. For display purposes only.
	std::string name;

public:
	//Loads .EXP file and fills the OUT vector
	//with the test cases described in it
	static void loadTestCasesFromEXP(const std::string& path, std::vector<TestCase>& out);

	//---------------------------
	//------- benchmarks --------
	//---------------------------
	void descriptiveness(std::vector<PREntry>& out);
};

#endif