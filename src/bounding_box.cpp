#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/registration/ndt.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/common/pca.h>
#include <pcl/common/common.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

//#include <pcl_tools/ndt_mapping.h>

using namespace std;

typedef pcl::PointXYZ PointT;

struct pose
{
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;
};

class Find_box{
public:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber cloud_sub;
  ros::Publisher map_pub, map_accumulated_pub, current_pub, first_cloud_pub;
  
  pcl::PointCloud<PointT>::Ptr cloudSegmented;
  pcl::PointCloud<PointT> map, add;

  Eigen::Vector4f pcaCentroid;
  Eigen::Matrix3f covariance;

  Find_box():
	pnh_("~"),
	nh_("~")
  {

  }
  void setInputCloud(pcl::PointCloud<PointT>::Ptr& input){
	cloudSegmented = input;
  }

  void compute(){

	pcl::compute3DCentroid(*cloudSegmented, pcaCentroid);
	computeCovarianceMatrixNormalized(*cloudSegmented, pcaCentroid, covariance);
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
	Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
	eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));

	// pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPCAprojection (new pcl::PointCloud<pcl::PointXYZ>);
	// pcl::PCA<pcl::PointXYZ> pca;
	// pca.setInputCloud(cloudSegmented);
	// pca.project(*cloudSegmented, *cloudPCAprojection);
	// std::cerr << std::endl << "EigenVectors: " << pca.getEigenVectors() << std::endl;
	// std::cerr << std::endl << "EigenValues: " << pca.getEigenValues() << std::endl;
	// Eigen::Matrix3f eigenVectorsPCA = pca.getEigenVectors();
	
    // Transform the original cloud to the origin where the principal components correspond to the axes.
	Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
	projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
	projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointsProjected (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::transformPointCloud(*cloudSegmented, *cloudPointsProjected, projectionTransform);
	// Get the minimum and maximum points of the transformed cloud.
	pcl::PointXYZ minPoint, maxPoint;
	pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
	
	const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());

	// Final transform
	const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA); //Quaternions are a way to do rotations https://www.youtube.com/watch?v=mHVwd8gYLnI
	const Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();

	// This viewer has 4 windows, but is only showing images in one of them as written here.
	pcl::visualization::PCLVisualizer *visu;
	visu = new pcl::visualization::PCLVisualizer ("PlyViewer");
	int mesh_vp_1, mesh_vp_2, mesh_vp_3, mesh_vp_4;
	visu->createViewPort (0.0, 0.5, 0.5, 1.0,  mesh_vp_1);
	visu->createViewPort (0.5, 0.5, 1.0, 1.0,  mesh_vp_2);
	visu->createViewPort (0.0, 0, 0.5, 0.5,  mesh_vp_3);
	visu->createViewPort (0.5, 0, 1.0, 0.5, mesh_vp_4);

	pcl::visualization::PointCloudColorHandlerCustom<PointT>  ColorHandlerXYZ(cloudSegmented, 255, 0, 0); 
	visu->addPointCloud(cloudSegmented, ColorHandlerXYZ, "bboxedCloud", mesh_vp_1);
	visu->addCoordinateSystem(1.0, 0);
	
	visu->addCube(bboxTransform, bboxQuaternion, maxPoint.x - minPoint.x, maxPoint.y - minPoint.y, maxPoint.z - minPoint.z, "bbox", mesh_vp_1);

	visu->addCube(4.8962,5.25494,-1.69376,-1.13877,-0.796634,-0.547576,0,1,0,"cube", mesh_vp_1);
	

	visu->spin();
  }
  
  void map_cb(const sensor_msgs::PointCloud2::ConstPtr& input){
	//t = input->header.stamp
	  
	  
	  
	}
	
	//ndt_map.
   
  };


int
main (int argc, char** argv)
{
  ros::init(argc, argv, "ndt_mapping");

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1 (new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<PointT>);
  //cout << "引数: " << argc << endl;
  if(pcl::io::loadPCDFile (argv[1], *cloud1) == -1){
	PCL_ERROR ("Couldn't read file");
    return(1);
  }
  if(pcl::io::loadPCDFile (argv[2], *cloud2) == -1){
	PCL_ERROR ("Couldn't read file");
    return(1);
  }

  pcl::visualization::PCLVisualizer *visu;
  visu = new pcl::visualization::PCLVisualizer ("PlyViewer");
  
  pcl::visualization::PointCloudColorHandlerCustom<PointT>  ColorHandlerXYZG(cloud1, 0, 255, 0); 
  visu->addPointCloud(cloud1, ColorHandlerXYZG, "bboxedCloud1", 0);
  pcl::visualization::PointCloudColorHandlerCustom<PointT>  ColorHandlerXYZR(cloud2, 255, 0, 0); 
  visu->addPointCloud(cloud2, ColorHandlerXYZR, "bboxedCloud2", 0);
  visu->addCoordinateSystem(1.0, 0);
  // visu->addCube(4.8962,5.25494,-1.69376,-1.13877,-0.796634,-0.547576,0,1,0,"cube1", 0);
  // visu->addCube(5.20082, 5.5297,-1.7461,-1.22005,-0.749802,-0.47001 ,1,0,0,"cube2", 0);
  visu->addCube(19.8001,20.2,-1.36884,-0.802587,-0.901797,-0.607553,0,1,0,"cube1", 0);
  visu->addCube(20.5006,20.83,-1.11978,-0.610007,-1.0978,-0.820057 ,1,0,0,"cube2", 0);

  visu->spin();
  

  // Find_box fb;
  
  // fb.setInputCloud(cloud1);
  // fb.compute();

  // while(ros::ok()){
  // 	ros::spinOnce();

  // }
  
  return (0);
}
