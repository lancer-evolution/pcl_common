// ROS core
#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>

// PCL includes
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <Eigen/Geometry>

using namespace std;

/**
\author Radu Bogdan Rusu
@b pointcloud_to_pcd is a simple node that retrieves a ROS point cloud message and saves it to disk into a PCD (Point
Cloud Data) file format.
**/
class PointCloudToPCD
{
  protected:
    ros::NodeHandle nh_;

  private:
    std::string prefix_;
    bool binary_;
    bool compressed_;
    std::string fixed_frame_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

  public:
    string cloud_topic_;

    ros::Subscriber sub_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input;
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud;

    ////////////////////////////////////////////////////////////////////////////////
    // Callback
  
  //cloud_cb (const pcl::PCLPointCloud2::ConstPtr& cloud)
  void cloud_cb(const sensor_msgs::PointCloud2::ConstPtr& cloud)
  {
	if ((cloud->width * cloud->height) == 0)
	  return;

	ROS_INFO ("Received %d data points in frame %s with the following fields: %s",
			  (int)cloud->width * cloud->height,
			  cloud->header.frame_id.c_str (),
			  pcl::getFieldsList (*cloud).c_str ());

	Eigen::Vector4f v = Eigen::Vector4f::Zero ();
	Eigen::Quaternionf q = Eigen::Quaternionf::Identity ();

	pcl::PointCloud<pcl::PointXYZ> input_cloud;
    transformed_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromROSMsg(*cloud, input_cloud);
	cloud_input = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>(input_cloud));
	
	if (!fixed_frame_.empty ()) {
	  // canTransformでは"/"が使えないのでそれの対策
	  std::string t = "/";  // 検索文字列
	  int pos = fixed_frame_.find(t);  // 検索文字列が見つかった位置 （pos == 1）
	  int len = t.length(); // 検索文字列の長さ （len == 2）
	  if (pos != std::string::npos) {
		fixed_frame_.replace(pos, len, ""); // s == "a|b"
	  }
	  string target_frame = cloud->header.frame_id;
	  pos = target_frame.find(t);  // 検索文字列が見つかった位置 （pos == 1）
	  len = t.length(); // 検索文字列の長さ （len == 2）
	  if (pos != std::string::npos) {
		target_frame.replace(pos, len, ""); // s == "a|b"
	  }
		
	  if (!tf_buffer_.canTransform (fixed_frame_, target_frame, cloud->header.stamp, ros::Duration (3.0))) {
		ROS_WARN("Could not get transform!");
		return;
	  }

	  Eigen::Affine3d transform;
	  Eigen::Affine3f transform_f;
	  transform = tf2::transformToEigen (tf_buffer_.lookupTransform (fixed_frame_, target_frame, cloud->header.stamp));
	  transform_f = transform.cast<float>();
	  
	  v = Eigen::Vector4f::Zero ();
	  //v.head<3> () = transform.translation ().cast<float> ();
	  //q = transform.rotation ().cast<float> ();


	  // Executing the transformation
	  
	  
	  // You can either apply transform_1 or transform_2; they are the same
	  pcl::transformPointCloud (*cloud_input, *transformed_cloud, transform);
	  transformed_cloud->header.stamp = cloud_input->header.stamp;
	  transformed_cloud->header.frame_id = cloud_input->header.frame_id;
	}else{
	  transformed_cloud = cloud_input;
	}

	std::stringstream ss;
	ss << prefix_ << cloud->header.stamp << ".pcd";
	ROS_INFO ("Data saved to %s", ss.str ().c_str ());

	pcl::PCDWriter writer;
	if(binary_)
	  {
		if(compressed_)
		  {
			writer.writeBinaryCompressed (ss.str (), *transformed_cloud);
		  }
		else
		  {
			writer.writeBinary (ss.str (), *transformed_cloud);
		  }
	  }
	else
	  {
		writer.writeASCII (ss.str (), *transformed_cloud, 8);
	  }

  }

    ////////////////////////////////////////////////////////////////////////////////
    PointCloudToPCD () : binary_(false), compressed_(false), tf_listener_(tf_buffer_)
    {
      // Check if a prefix parameter is defined for output file names.
      ros::NodeHandle priv_nh("~");
      if (priv_nh.getParam ("prefix", prefix_))
        {
          ROS_INFO_STREAM ("PCD file prefix is: " << prefix_);
        }
      else if (nh_.getParam ("prefix", prefix_))
        {
          ROS_WARN_STREAM ("Non-private PCD prefix parameter is DEPRECATED: "
                           << prefix_);
        }

      priv_nh.getParam ("fixed_frame", fixed_frame_);
      priv_nh.getParam ("binary", binary_);
      priv_nh.getParam ("compressed", compressed_);
      if(binary_)
	{
	  if(compressed_)
	    {
	      ROS_INFO_STREAM ("Saving as binary compressed PCD");
	    }
	  else
	    {
	      ROS_INFO_STREAM ("Saving as binary PCD");
	    }
	}
      else
	{
	  ROS_INFO_STREAM ("Saving as binary PCD");
	}

      cloud_topic_ = "input";

      sub_ = nh_.subscribe (cloud_topic_, 1,  &PointCloudToPCD::cloud_cb, this);
      ROS_INFO ("Listening for incoming data on topic %s",
                nh_.resolveName (cloud_topic_).c_str ());
    }
};

/* ---[ */
int
  main (int argc, char** argv)
{
  ros::init (argc, argv, "pointcloud_to_pcd", ros::init_options::AnonymousName);

  PointCloudToPCD b;
  ros::spin ();

  return (0);
}
