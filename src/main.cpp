#include <iostream>
#include "pointcloud_filter.h"

using namespace std;

int main(int argc, char** argv) {

	// ------------ realsense ----------------
	string pointcloud_sub_topic = "/red/camera/depth_registered/points";
	string mask_sub_topic = "/red/segmentation/mask";
	string mask_pub_topic = "/red/segmentation/test_image";
	string filtered_pointcloud_pub_topic = "pc_filter/points";
	string closest_point_distance_pub_topic = "pc_filter/closest_point_distance";
	string object_centroid_pub_topic = "pc_filter/object_centroid";
	string object_pub_topic = "pc_filter/object";
	string object_marker_pub_topic = "pc_filter/object_centroid_marker";
	string camera_frame = "red/camera";
	string world_frame = "mavros/world";

	PointcloudFilter::filter ( 	argc, argv, pointcloud_sub_topic, mask_sub_topic, 
								mask_pub_topic,
								filtered_pointcloud_pub_topic, 
								closest_point_distance_pub_topic, 
								object_centroid_pub_topic,
								object_pub_topic,
								object_marker_pub_topic,
								camera_frame, world_frame);

}
