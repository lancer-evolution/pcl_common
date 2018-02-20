#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/registration/ndt.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
//#include <pcl/ros/conversions.h>//使えない
#include <pcl_conversions/pcl_conversions.h>
//#include <pcl/filters/approximate_voxel_grid.h>
//#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <nav_msgs/Odometry.h>

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

class Ndt_Mapper{
public:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber cloud_sub;
  ros::Publisher map_pub, map_accumulated_pub, current_pub, first_cloud_pub;
  
  pcl::PointCloud<PointT>::Ptr first_cloud_ptr, target_cloud_ptr, target_cloud_filtered_accumulated, target_cloud_filtered, output_cloud_ptr, output_cloud_filtered;
  pcl::PointCloud<PointT> map, add;
  pcl::VoxelGrid<PointT> voxel_filter;
  ros::Time t;
  ros::Time current_scan_time;

  bool first_msg;
  // Default values
  int max_iter;        // Maximum iterations
  double ndt_res;      // Resolution
  double step_size;   // Step size
  double trans_eps;  // Transformation epsilon
  double leaf_size, map_leaf_size;
  double min_scan_range;		// LiDARからの距離
  double min_add_scan_shift;	// 更新のためのLiDARの移動量
  string base_link_, current_frame_, sub_name;
  pose previous_pose, guess_pose, guess_pose_odom, current_pose, current_pose_odom, ndt_pose, key_pose, localizer_pose;

  double fitness_score;
  bool has_converged;
  int final_num_iteration;

  double diff;
  double diff_x, diff_y, diff_z, diff_yaw;  // current_pose - previous_pose
  tf::Transform velodyne_tf;
  tf::StampedTransform transform, transform_scaled;
  tf::TransformListener listener;
  tf::TransformBroadcaster br;
  //Eigen::Matrix4f t_localizer;
  Eigen::Affine3d t_localizer;

  nav_msgs::Odometry odom;
  sensor_msgs::PointCloud2 cloud2;

  Ndt_Mapper():
	pnh_("~"),
	nh_("~"),
	first_msg(true)
  {
	pnh_.param("max_iter", max_iter, 30);
	pnh_.param("ndt_res", ndt_res, 1.0);
	pnh_.param("step_size", step_size, 0.1);
	pnh_.param("trans_eps", trans_eps, 0.01);
	pnh_.param("leaf_size", leaf_size, 2.0);
	pnh_.param("map_leaf_size", map_leaf_size, 0.1);
	pnh_.param("min_scan_range", min_scan_range, 3.0);
	pnh_.param("min_add_scan_shift", min_add_scan_shift, 0.1);
	pnh_.param("base_link_", base_link_, string("map"));
	pnh_.param("current_frame_", current_frame_, string("velodyne"));
	pnh_.param("sub_name", sub_name, string("/velodyne_points"));
	
	cloud_sub = nh_.subscribe(sub_name, 1, &Ndt_Mapper::map_cb, this);
	map_pub = nh_.advertise<pcl::PointCloud<PointT> > ( "ndt_map", 1 );
	//map_accumulated_pub = nh_.advertise<pcl::PointCloud<PointT> > ( "reference_map", 1 );
	current_pub = nh_.advertise<sensor_msgs::PointCloud2> ( "current_cloud", 1 );
	//first_cloud_pub = nh_.advertise<sensor_msgs::PointCloud2> ( "first_cloud", 1 );

	first_cloud_ptr = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
	target_cloud_ptr = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
	target_cloud_filtered = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
	target_cloud_filtered_accumulated = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);

	// base to current
	velodyne_tf = tf::Transform(
								tf::createQuaternionFromRPY(0, 0.0, 0),
								tf::Vector3(0, 0.0, 0.0)
								);
	t_localizer.matrix() = Eigen::Matrix4d::Identity();
	  
	br.sendTransform(tf::StampedTransform(velodyne_tf, ros::Time::now(), base_link_, current_frame_));

	diff = diff_x = diff_y = diff_z = 0.0;
	
	key_pose.x = 0.0;
	key_pose.y = 0.0;
	key_pose.z = 0.0;
	key_pose.roll = 0.0;
	key_pose.pitch = 0.0;
	key_pose.yaw = 0.0;
  }
  void map_cuttter(pcl::PointCloud<PointT>::Ptr& input){
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud (input);
	pass.setFilterFieldName ("x");
	pass.setFilterLimits (-2, 20); // -30, 50
	pass.filter (*input);
	pass.setInputCloud (input);
	pass.setFilterFieldName ("y");
	pass.setFilterLimits (-6, 6); // -30, 20
	pass.filter (*input);
	pass.setInputCloud (input);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (-4, 10); // -5, 10
	pass.filter (*input);

  }
  
  void mapping(pcl::PointCloud<PointT>::Ptr& input, pcl::PointCloud<PointT>::Ptr& target, double leaf){
	cout << "target: " << target->size()  << endl;
	*target += *input;
	cout << "target + input: " << target->size()  << endl;
	voxel_filter.setLeafSize (leaf, leaf, leaf);
    voxel_filter.setInputCloud (target);
    voxel_filter.filter (*target);
  }

  void map_cb(const sensor_msgs::PointCloud2::ConstPtr& input){
	//t = input->header.stamp;
	t = ros::Time::now();
	current_scan_time = input->header.stamp;
	
	pcl::PointCloud<PointT> input_cloud, scan;
	pcl::fromROSMsg(*input, input_cloud);

	PointT p;
	double r;
	for (pcl::PointCloud<PointT>::const_iterator item = input_cloud.begin(); item != input_cloud.end(); item++)
	  {
		p.x = (double)item->x;
		p.y = (double)item->y;
		p.z = (double)item->z;

		r = sqrt(pow(p.x, 2.0) + pow(p.y, 2.0));
		if (r > min_scan_range)
		  {
			scan.push_back(p);
		  }
	  }
	
	pcl::PointCloud<PointT>::Ptr scan_ptr(new pcl::PointCloud<PointT>(scan));

	pcl::PointCloud<PointT>::Ptr filtered_scan_ptr(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr transformed_scan_ptr(new pcl::PointCloud<PointT>());
	pcl::PointCloud<PointT>::Ptr filtered_map_ptr(new pcl::PointCloud<PointT>());
	
	pcl::PointCloud<PointT>::Ptr output_cloud_filtered(new pcl::PointCloud<PointT>(input_cloud));
	pcl::PointCloud<PointT>::Ptr output_cloud_ptr(new pcl::PointCloud<PointT>(input_cloud));
	
	//map_cuttter(scan_ptr);
	
	if(first_msg){
	  cout << "first message" << endl;
	  map += *scan_ptr;
	  // *first_cloud_ptr = *input_cloud_ptr;
	  // *target_cloud_ptr = *input_cloud_ptr;
	  // *target_cloud_filtered = *filtered_cloud_ptr;
	  // *target_cloud_filtered_accumulated = *filtered_cloud_ptr;
	  first_msg = false;
	}else{
	  cout << "NDT Algorithm" << endl;
	  pcl::PointCloud<pcl::PointXYZ>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZ>(map));

	  if(leaf_size > 0.001){
		voxel_filter.setLeafSize (leaf_size, leaf_size, leaf_size);
		voxel_filter.setInputCloud (scan_ptr);
		voxel_filter.filter (*filtered_scan_ptr);
	  }else{
		*filtered_scan_ptr = *scan_ptr;
	  }
	  
	  output_cloud_ptr = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
	  pcl::NormalDistributionsTransform<PointT, PointT> ndt;
	  //cout << target_cloud_filtered->size() << endl;
	  //cout << &target_cloud_filtered << endl;
	  //cout << filtered_cloud_ptr->size() << endl;
	  ndt.setResolution(ndt_res);
	  ndt.setInputSource(filtered_scan_ptr);
	  ndt.setInputTarget(map_ptr); // target_cloud_filtered
	  ndt.setMaximumIterations(max_iter);
	  ndt.setStepSize(step_size);
	  ndt.setTransformationEpsilon(trans_eps);

	  // guess_pose.x = previous_pose.x + diff_x;
	  // guess_pose.y = previous_pose.y + diff_y;
	  // guess_pose.z = previous_pose.z + diff_z;
	  // guess_pose.roll = previous_pose.roll;
	  // guess_pose.pitch = previous_pose.pitch;
	  // guess_pose.yaw = previous_pose.yaw + diff_yaw;

	  // odom_calc(current_scan_time);

	  // Set initial alignment estimate found using robot odometry.
	  // Eigen::AngleAxisf init_rotation (0.6931, Eigen::Vector3f::UnitZ ());
	  // Eigen::Translation3f init_translation (1.79387, 0.720047, 0);
	  // Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix ();

	  ndt.align(*output_cloud_ptr, t_localizer.matrix().cast<float>());
	  fitness_score = ndt.getFitnessScore();
	  
	  // 推定された位置
	  t_localizer.matrix() = ndt.getFinalTransformation().cast<double>();
	  has_converged = ndt.hasConverged();
      final_num_iteration = ndt.getFinalNumIteration();  
	  std::cout << "N.D.T. has converged:" << has_converged
				<< " score: " << fitness_score << std::endl;

	  current_pose.x = t_localizer.translation().x();
	  current_pose.y = t_localizer.translation().y();
	  current_pose.z = t_localizer.translation().z();
	  // current_pose.roll = ndt_pose.roll;
	  // current_pose.pitch = ndt_pose.pitch;
	  // current_pose.yaw = ndt_pose.yaw;

	  // 元の点群を座標変換
	  pcl::transformPointCloud(*scan_ptr, *transformed_scan_ptr, t_localizer);
	  
	  //tf::Transform transform_tf;
	  // 推定位置をtfに変換->broadcast
	  tf::transformEigenToTF(t_localizer, velodyne_tf);	  
	  br.sendTransform(tf::StampedTransform(velodyne_tf, t, base_link_, current_frame_));

	  // キーフレームと変化量が大きい時に更新 default:1[m]
	  double shift = sqrt(pow(current_pose.x - key_pose.x, 2.0) + pow(current_pose.y - key_pose.y, 2.0));
	  if (shift >= min_add_scan_shift)
		{
		  map += *transformed_scan_ptr;
		  key_pose.x = current_pose.x;
		  key_pose.y = current_pose.y;
		  key_pose.z = current_pose.z;
		  key_pose.roll = current_pose.roll;
		  key_pose.pitch = current_pose.pitch;
		  key_pose.yaw = current_pose.yaw;
		  cout << "Map Updated!" << endl;
		  // isMapUpdate = true;
		}
	  
	  // visualize用にmapをfiltering
	  if(map_leaf_size > 0.001){
		voxel_filter.setLeafSize (map_leaf_size, map_leaf_size, map_leaf_size);
		voxel_filter.setInputCloud (map_ptr);
		voxel_filter.filter (*filtered_map_ptr);
	  }else{
		*filtered_map_ptr = *map_ptr;
	  }

	  // sensor_msgs::PointCloud2 first_cloud_pointcloud2;
	  // pcl::toROSMsg(*first_cloud_ptr, first_cloud_pointcloud2);
	  // first_cloud_pointcloud2.header.frame_id = base_link_;
	  // first_cloud_pointcloud2.header.stamp = t;
	  // first_cloud_pub.publish(first_cloud_pointcloud2);

	  sensor_msgs::PointCloud2::Ptr current_msg_ptr(new sensor_msgs::PointCloud2); 
	  pcl::toROSMsg(*filtered_scan_ptr, *current_msg_ptr);
	  current_msg_ptr->header.frame_id = current_frame_;
	  current_msg_ptr->header.stamp = t;
	  current_pub.publish(*current_msg_ptr);

	  // sensor_msgs::PointCloud2 target_cloud_pointcloud2;
	  // pcl::toROSMsg(*target_cloud_filtered_accumulated, target_cloud_filtered_pointcloud2);
	  // target_cloud_filtered_pointcloud2.header.frame_id = base_link_;
	  // target_cloud_filtered_pointcloud2.header.stamp = t;
	  // map_accumulated_pub.publish(target_cloud_filtered_pointcloud2);
	  

	  sensor_msgs::PointCloud2::Ptr map_msg_ptr(new sensor_msgs::PointCloud2);
	  pcl::toROSMsg(*filtered_map_ptr, *map_msg_ptr);
	  map_msg_ptr->header.frame_id = base_link_;
	  map_msg_ptr->header.stamp = t;
	  map_pub.publish(*map_msg_ptr);
	  
	  
	  
	  

	  
	  
	  
	  
	}
	
	//ndt_map.
	
  }


	
  };


int
main (int argc, char** argv)
{
  ros::init(argc, argv, "ndt_mapping");

  Ndt_Mapper ndt_map;

  while(ros::ok()){
	ros::spinOnce();

  }
  return (0);
}
