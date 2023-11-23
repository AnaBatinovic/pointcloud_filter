#ifndef PC_PUB_SUB_H_
#define PC_PUB_SUB_H_

#include "ros/ros.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/CompressedImage.h"
#include "geometry_msgs/PointStamped.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"

#include <pcl_ros/point_cloud.h>

#include <pcl_ros/impl/transforms.hpp>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/common/intersections.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

// Segmentation msgs
#include <semantic_segmentation_ros/SegmentationNameMask.h>
#include <semantic_segmentation_ros/SegmentationNameMaskArray.h>
#include <semantic_segmentation_ros/SegmentationObject.h>
#include <semantic_segmentation_ros/SegmentationObjectArray.h>


// Visualisation 
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <vector>

using namespace std;

class PC_PUB_SUB 
{
	public:

		int nContours = 0;

		PC_PUB_SUB (ros::NodeHandle& nodeHandle,
            string pointcloud_sub_topic,
            string mask_sub_topic,
			string mask_pub_topic,
            string filtered_pointcloud_pub_topic,
            string closest_point_distance_pub_topic,
            string object_centroid_pub_topic,
			string object_pub_topic,
			string object_marker_pub_topic);

		virtual ~PC_PUB_SUB();

		void registerPointCloudSubscriber(string topic);
		void registerNameMaskSubscriber(string topic);
		void registerPointCloudPublisher(string topic);
		void registerTestImagePublisher(string topic);
		void registerDistancePublisher(string topic);
		void registerObjectCentroidPublisher(string topic);
		void registerObjectArrayPublisher(string topic);
		void visualizationPublisher (string topic);

		void rosPointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& ros_msg);
		void resetNewMeasurementFlag();
		bool newMeasurementRecieved();
		void rosNameMaskCallback(const semantic_segmentation_ros::SegmentationNameMaskArray &ros_msg);

		void publishPointCloud(
			pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud, string camera_frame);
		
		void publishDistance(double distance);
		void publishTestImage(sensor_msgs::Image ros_msg);
		void publishObjectCentroid(geometry_msgs::PointStamped centroid);
		void publishObjectArray(semantic_segmentation_ros::SegmentationObjectArray object_array);
		void publishObjectCentroidVector(const vector<double> &centroid);
		void visualizeObjectArray(semantic_segmentation_ros::SegmentationObjectArray object_array, string frame);

		void processRosImage(const sensor_msgs::Image::ConstPtr& ros_msg, vector < vector <int>> & mask);
			
		pcl::PointCloud<pcl::PointXYZ>::Ptr getOrganizedCloudPtr();
		vector< vector< vector <int>>> getMask();
		vector< string> getName();

	private:
		ros::NodeHandle nodeHandle_;
		ros::Subscriber sub_pc2_;
		ros::Subscriber sub_mask_;
		ros::Publisher pub_pc2_;
		ros::Publisher pub_distance_;
		ros::Publisher pub_object_centroid_;
		ros::Publisher pub_object_array_;
		ros::Publisher pub_patch_centroid_filtered_;
		ros::Publisher pub_object_;
		ros::Publisher pub_object_marker_;
		ros::Publisher pub_test_image_;
		
		pcl::PointCloud<pcl::PointXYZ>::Ptr organizedCloudPtr;
		vector< vector <int>> mask_;
		vector< vector< vector <int>>> mask_array_;
		vector< vector <int>> patch_mask_;
		double closest_point_distance;
		bool _newMeasurement = false;
		string object_name_;
		vector <string> object_name_array_;
		std::map<std::string, int> dictionary = {
        {"pipe", 1},
        {"ladder", 2}};
};

#endif /* PC_PUB_SUB_H_ */
