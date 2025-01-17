/*
 * Find distance of the segmented object
 * Code from pointcloud_filter.cpp
 *  Created on: Mar 28, 2023
 *      Author: Ana Milas
 */

#include "pointcloud_filter.h"
#include <pcl/filters/statistical_outlier_removal.h>

#define NO_CONOUTS_ERROR -1

void PointcloudFilter::filter ( int argc, char** argv, 
								string pointcloud_sub_topic, 
								string mask_sub_topic,
								string mask_pub_topic,
								string filtered_pointcloud_pub_topic, 
								string closest_point_distance_pub_topic,
								string object_centroid_pub_topic,
								string object_pub_topic,
								string object_marker_pub_topic,
								string camera_frame,
								string world_frame) 
{
	ros::init(argc, argv, "pc_filter");
	ros::NodeHandle nodeHandle("~");

	// Read params from launch files
	// string pointcloud_sub_topic, mask_sub_topic, camera_frame, world_frame;
    nodeHandle.param<string>("depth_topic", pointcloud_sub_topic, "depth_point_cloud");
	nodeHandle.param<string>("mask_topic", mask_sub_topic, "mask");
	nodeHandle.param<string>("camera_frame", camera_frame, "camera");
	nodeHandle.param<string>("world_frame", world_frame, "world");

	PC_PUB_SUB pcl_pub_sub(	nodeHandle, pointcloud_sub_topic, mask_sub_topic, mask_pub_topic,
							filtered_pointcloud_pub_topic, closest_point_distance_pub_topic,
						    object_centroid_pub_topic, object_pub_topic, object_marker_pub_topic);
	ros::Rate loop_rate(20);
	ros::Duration(3.0).sleep();
	tf::TransformListener listener;
	semantic_segmentation_ros::SegmentationObject object;
	semantic_segmentation_ros::SegmentationObjectArray objectArray;

	while(nodeHandle.ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
		
		pcXYZ::Ptr originalCloud = pcl_pub_sub.getOrganizedCloudPtr();
		// objectArray.objects.clear();
		if(!originalCloud || originalCloud->points.size() == 0) {
			ROS_WARN_THROTTLE(1.0, "Point cloud not received!");
			continue;
		}
		if(pcl_pub_sub.getMask().size() == 0) {
			ROS_WARN_THROTTLE(1.0, "Mask not received!");
			continue;
		}
		pcXYZ::Ptr accumulatedCloud (new pcXYZ);
		// For each mask
		for (int i = 0; i < pcl_pub_sub.getMask().size(); i++) {

			pcXYZ::Ptr filteredCloud ( new pcXYZ ), maskCloud( new pcXYZ );

			filteredCloud = removeNonMaskValues(originalCloud, pcl_pub_sub.getMask().at(i));
			// std::cout << "Point cloud size: " << filteredCloud->size() << std::endl;
			filteredCloud = removeNaNValues(filteredCloud);
			// Find object pose in camera frame
			std::vector<double> minDistances, objectCentroid;
			minDistances = findClosestDistance(filteredCloud);
			objectCentroid = findCentroid(filteredCloud);
			
			// Transform centroid and distance to global frame
			geometry_msgs::PointStamped objectTransformedCentroid;
			objectTransformedCentroid = transformCentroid(objectCentroid, world_frame, 
				camera_frame, listener);

			object.name = pcl_pub_sub.getName().at(i);
			object.point = objectTransformedCentroid;
			//  Check if the centroid is already in the objectArray and updated
			if (!updateCentroidAveragePositionAndReturnIfSuccessful(objectArray, object))
			{
				// If false, add the current centroid to the objectArray
				objectArray.objects.push_back(object);
			}	

			// Publish the absolute distance
			pcl_pub_sub.publishDistance(minDistances[3]);
			
			// std::cout << "Point cloud size: " << maskCloud->size() << std::endl;
			// Transform filtered cloud and publish it
			pcXYZ::Ptr transformedFilteredCloud (new pcXYZ);
			transformedFilteredCloud = transformCloud(filteredCloud, world_frame, listener);
			*accumulatedCloud += *transformedFilteredCloud;
			pcl_pub_sub.publishPointCloud(accumulatedCloud, world_frame);
		}
		// Visualize object array in global frame
		// pcl_pub_sub.publishObjectCentroid(objectTransformedCentroid);
		pcl_pub_sub.visualizeObjectArray(objectArray, world_frame);
		// Publish SegmenatationObject
		pcl_pub_sub.publishObjectArray(objectArray);
	}
}

geometry_msgs::PointStamped PointcloudFilter::transformCentroid(
	std::vector<double> centroid, string world_frame, 
	string camera_frame, tf::TransformListener &tf_listener) 
{
	if (centroid.empty()) return geometry_msgs::PointStamped();
	try
	{
		ros::Time now = ros::Time::now();
		tf::StampedTransform transform;
		tf_listener.waitForTransform(world_frame, camera_frame, now, ros::Duration(2.0));
		tf_listener.lookupTransform(world_frame, camera_frame, now, transform);
	}
		catch (tf::TransformException ex)
	{
		ROS_ERROR("%s",ex.what());
	}
	geometry_msgs::PointStamped objectCentroidLocal, objectCentroidGlobal;
	objectCentroidLocal.header.frame_id = camera_frame;
    objectCentroidLocal.point.x = centroid[0];
    objectCentroidLocal.point.y = centroid[1];
    objectCentroidLocal.point.z = centroid[2];
	tf_listener.transformPoint(world_frame, objectCentroidLocal, objectCentroidGlobal);
	return objectCentroidGlobal;
}

pcXYZ::Ptr PointcloudFilter::transformCloud(pcXYZ::Ptr inputCloud, 
	string goal_frame, tf::TransformListener &tf_listener) 
{
	pcXYZ::Ptr outputCloud (new pcXYZ);
	ros::Time now = ros::Time::now();
	tf::StampedTransform transform;
	tf_listener.waitForTransform(goal_frame, inputCloud->header.frame_id, now, ros::Duration(2.0));
	tf_listener.lookupTransform(goal_frame, inputCloud->header.frame_id, now, transform);
	
	pcl_ros::transformPointCloud(*inputCloud, *outputCloud, transform);
	return outputCloud;
}

pcXYZ::Ptr PointcloudFilter::doOutlierFiltering( pcXYZ::Ptr inputCloud , ros::NodeHandle& nh)
{
	double meanK = 50, stddevMulThres = 1;
	nh.getParam("brick/outlier/mean_k", meanK);
	nh.getParam("brick/outlier/stddev_multiplier_thresh", stddevMulThres);

	pcXYZ::Ptr cloud_filtered {new pcXYZ};
	// Create the filtering object
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud (inputCloud);
	sor.setMeanK (meanK);
	sor.setStddevMulThresh (stddevMulThres);
	sor.filter (*cloud_filtered);

	return cloud_filtered;
}

double PointcloudFilter::findClosestX(pcXYZ::Ptr inputCloud)
{
	double inf_x = 99999.9;
	double min_x = inf_x;
	if(!inputCloud || inputCloud->points.size() == 0)	return -1;
	else {
		for (int i = 0; i < inputCloud->points.size(); i++) {
			double x_i = inputCloud->points[i].x;
			if (x_i < min_x) {
				min_x = x_i;
			}
		}
	}
	return min_x;
}

std::vector<double> PointcloudFilter::findClosestDistance(pcXYZ::Ptr inputCloud)
{
	double inf_distance = 99999.9;
	double min_distance = inf_distance;
	double min_x = 0.0;
	double min_y = 0.0;
	double min_z = 0.0;

	if(!inputCloud || inputCloud->points.size() == 0)	return std::vector<double> {-1, -1, -1, -1};
	else {
		for (int i = 0; i < inputCloud->points.size(); i++) {
			double x = inputCloud->points[i].x;
			double y = inputCloud->points[i].y;
			double z = inputCloud->points[i].z;
			double distance_i = sqrt ( x*x + y*y + z*z );
			if (distance_i < min_distance) {
				min_distance = distance_i;
				min_x = x;
				min_y = y;
				min_z = z;
			}
		}
	}
	//cout << "closest point: (" << min_x << ", " << min_y << ", " << min_z << ")" << endl << endl;
	return std::vector<double> {min_x, min_y, min_z, min_distance};
}

std::vector<double> PointcloudFilter::findCentroid(pcXYZ::Ptr inputCloud)
{
    std::vector<double> centroid{0,0,0};
	// Return an empty vector
    if (!inputCloud || inputCloud->points.size() == 0) return std::vector<double>();
    else {
        for (int i = 0; i < inputCloud->points.size(); i++) {
            centroid[0] += inputCloud->points[i].x;
            centroid[1] += inputCloud->points[i].y;
            centroid[2] += inputCloud->points[i].z;
        }
        centroid[0] /= inputCloud->points.size();
        centroid[1] /= inputCloud->points.size();
        centroid[2] /= inputCloud->points.size();
    }
    return centroid;
}

bool PointcloudFilter::updateCentroidAveragePositionAndReturnIfSuccessful(
		semantic_segmentation_ros::SegmentationObjectArray &objectArray, 
		semantic_segmentation_ros::SegmentationObject &object)
{
	double objectRadius = 3.0;
	// Do not add object that are above 3.0 meteres
	// if (object.point.point.z > 6.0) return true;
	// {

	// }
	// Check if new centroid exist in the list 
	for (int i = 0; i < objectArray.objects.size(); i++)
	{
		if (objectArray.objects[i].name == object.name &&
		fabs(objectArray.objects[i].point.point.x - object.point.point.x) < objectRadius &&
		fabs(objectArray.objects[i].point.point.y - object.point.point.y) < objectRadius &&
		fabs(objectArray.objects[i].point.point.z - object.point.point.z) < objectRadius) 
		{
			// Point is inside the radius, update the position
			ROS_WARN("Object already in the array. Update position.");
			// Update the average position u
			objectArray.objects[i].point.point.x = 
				(objectArray.objects[i].point.point.x + object.point.point.x) / 2;
			objectArray.objects[i].point.point.y = 
				(objectArray.objects[i].point.point.y + object.point.point.y) / 2;
			objectArray.objects[i].point.point.z = 
				(objectArray.objects[i].point.point.z + object.point.point.z) / 2;
			return true;
		}
	}
	return false;	
}

vector <Eigen::Vector3d> PointcloudFilter::pointIndicesToInlierPoints (
	pcXYZ::Ptr inputCloud, pcl::PointIndices::Ptr inliers )
{
	Eigen::Vector3d tempPoint;
	vector <Eigen::Vector3d> inlierPoints;
	for(int inlierCounter = 0; inlierCounter < inliers->indices.size(); inlierCounter++) {
		tempPoint << 	inputCloud->points[inliers->indices[inlierCounter]].x,
						inputCloud->points[inliers->indices[inlierCounter]].y,
						inputCloud->points[inliers->indices[inlierCounter]].z;
		inlierPoints.push_back ( tempPoint );
	}
	return inlierPoints;
}

pcXYZ::Ptr PointcloudFilter::removeNaNValues ( pcXYZ::Ptr inputCloud )
{
	vector<int> indices;
	pcXYZ::Ptr tempCloud ( new pcXYZ );
	pcl::removeNaNFromPointCloud ( *inputCloud, *tempCloud, indices );
	return tempCloud;
}

pcXYZ::Ptr PointcloudFilter::removeNonMaskValues(pcXYZ::Ptr inputCloud,
                                                 vector<vector<int>> mask)
{
	
	pcXYZ::Ptr tempCloud ( new pcXYZ );
	*tempCloud = *inputCloud;

	tempCloud->clear();
	tempCloud->header = inputCloud->header;
	tempCloud->height = 1;
	tempCloud->width = 0;
	for (int i = 0; i < mask.size(); i++) {
		for (int j = 0; j < mask[0].size(); j++) {
			// std::cout << mask[i][j] << std::endl;
			if (mask[i][j] > 0) {
				tempCloud->width++;
				tempCloud->points.push_back(inputCloud->at(j,i));
			}
		}
	}
	return tempCloud;
}


