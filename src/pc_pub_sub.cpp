#include "pc_pub_sub.h"
static const std::string OPENCV_WINDOW = "Image window";
PC_PUB_SUB::PC_PUB_SUB(	ros::NodeHandle& nodeHandle,
                        string pointcloud_sub_topic,
                        string mask_sub_topic,
						string mask_pub_topic,
                        string filtered_pointcloud_pub_topic,
                        string closest_point_distance_pub_topic,
                        string object_centroid_pub_topic,
						string object_pub_topic,
						string object_marker_pub_topic)
{
	nodeHandle_ = nodeHandle;
	registerPointCloudSubscriber(pointcloud_sub_topic);
	if (mask_sub_topic != "none")
		registerNameMaskSubscriber(mask_sub_topic);
	registerPointCloudPublisher(filtered_pointcloud_pub_topic);
	registerDistancePublisher(closest_point_distance_pub_topic);
	registerTestImagePublisher(mask_pub_topic);
	registerObjectCentroidPublisher(object_centroid_pub_topic);
	registerObjectArrayPublisher(object_pub_topic);
	visualizationPublisher(object_marker_pub_topic);
}

PC_PUB_SUB::~PC_PUB_SUB() 
{
}

void PC_PUB_SUB::registerPointCloudSubscriber(string topic) 
{
	sub_pc2_ = nodeHandle_.subscribe(topic, 1, &PC_PUB_SUB::rosPointCloudCallback, this);
}
void PC_PUB_SUB::registerNameMaskSubscriber(string topic) 
{
	sub_mask_ = nodeHandle_.subscribe(topic, 1, &PC_PUB_SUB::rosNameMaskCallback, this);
}
void PC_PUB_SUB::registerPointCloudPublisher(string topic) 
{
	pub_pc2_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>(topic, 1000);
}
void PC_PUB_SUB::registerDistancePublisher(string topic)
{
	pub_distance_ = nodeHandle_.advertise<std_msgs::Float32>(topic, 1000);
}
void PC_PUB_SUB::registerTestImagePublisher(string topic)
{
	pub_test_image_ = nodeHandle_.advertise<sensor_msgs::Image>(topic, 1000);
}
void PC_PUB_SUB::registerObjectCentroidPublisher(string topic)
{
    pub_object_centroid_ = nodeHandle_.advertise<geometry_msgs::PointStamped>(topic, 1000);
}
void PC_PUB_SUB::registerObjectArrayPublisher(string topic)
{
    pub_object_ = nodeHandle_.advertise<semantic_segmentation_ros::SegmentationObjectArray>(topic, 10);
}
void PC_PUB_SUB::visualizationPublisher(string topic)
{
	pub_object_marker_ = nodeHandle_.advertise<visualization_msgs::Marker>(topic, 10);	
} 
void PC_PUB_SUB::rosPointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& ros_msg) 
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_msg(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);

	vector<int> indices;

	pcl::fromROSMsg(*ros_msg, *pcl_msg);

	this->organizedCloudPtr = pcl_msg;
	_newMeasurement = true;
}
void PC_PUB_SUB::resetNewMeasurementFlag()
{
	_newMeasurement = false;
}

bool PC_PUB_SUB::newMeasurementRecieved()
{
	return _newMeasurement;
}

void PC_PUB_SUB::rosNameMaskCallback(const semantic_segmentation_ros::SegmentationNameMaskArray &ros_msg)
{
	// Clear vectors to fill them
	object_name_array_.clear();
	mask_array_.clear();
	cv_bridge::CvImagePtr cv_ptr;
	
	// Get every mask and name and fill out vectors 
	for (int i = 0; i < ros_msg.masks.size(); i++) {
		try
		{
			// publishTestImage(ros_msg.masks.at(0).mask);
			// publishTestImage(ros_msg.masks.at(1).mask);
			cv_ptr = cv_bridge::toCvCopy(ros_msg.masks.at(i).mask, sensor_msgs::image_encodings::MONO8);
		}
			catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
		}
		// Update GUI Window
		// cv::imshow(OPENCV_WINDOW, cv_ptr->image);
		// cv::waitKey(3);

		cv::Mat image;
		image = cv_ptr->image;
		int width = image.cols;
		int height = image.rows;
		int _stride = image.step;
		uint8_t *myData = image.data;
		vector <vector <int>> image_mat;
		int sum = 0;
		for (int i = 0; i < height; i++) {
			vector <int> temp_vec;
			for (int j = 0; j < width; j++) {
				int d = *(image.data + i * width +j);
				temp_vec.push_back( *(image.data + i * width +j));
			}
			image_mat.push_back(temp_vec);
		}
		object_name_ = ros_msg.masks.at(i).name;
		mask_ = image_mat;	
		object_name_array_.push_back(object_name_);
		mask_array_.push_back(mask_);
	}
}

void PC_PUB_SUB::processRosImage(const sensor_msgs::Image::ConstPtr &ros_msg, vector<vector<int> > &mask)
{
	// std::cout << ros_msg->data.size() << std::endl;
	cv::Mat image = cv::imdecode(cv::Mat(ros_msg->data), cv::IMREAD_GRAYSCALE);
	int width = image.cols;
	int height = image.rows;
	int _stride = image.step;

	uint8_t *myData = image.data;
	vector <vector <int>> image_mat;
	int sum = 0;
	for (int i = 0; i < height; i++) {
		vector <int> temp_vec;
		for (int j = 0; j < width; j++) {
			int d = *(image.data + i * width +j);

			if (d > 255 || d < 0) {
				sum++;
			}
			temp_vec.push_back( *(image.data + i * width +j));
		}
		image_mat.push_back(temp_vec);
	}

    mask_ = image_mat;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PC_PUB_SUB::getOrganizedCloudPtr()
{
	return organizedCloudPtr;
}

vector< vector< vector <int>>> PC_PUB_SUB::getMask() {
	return mask_array_;
}
vector< string> PC_PUB_SUB::getName() {
	return object_name_array_;
}
void PC_PUB_SUB::publishPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud, string camera_frame) 
{
	sensor_msgs::PointCloud2::Ptr ros_msg(new sensor_msgs::PointCloud2);
	pcl::toROSMsg(*pointCloud, *ros_msg);

	std_msgs::Header head;
	head.stamp = ros::Time::now();
	//head.frame_id = "camera_link";
	//head.frame_id = "camera_color_optical_frame";
	head.frame_id = camera_frame;
	ros_msg->header = head;

	pub_pc2_.publish(*ros_msg);
}
void PC_PUB_SUB::publishDistance(double distance) 
{
	std_msgs::Float32::Ptr msg(new std_msgs::Float32);

	msg->data = distance;
	pub_distance_.publish(*msg);
}
void PC_PUB_SUB::publishTestImage(sensor_msgs::Image ros_msg) 
{
	pub_test_image_.publish(ros_msg);
}
void PC_PUB_SUB::publishObjectCentroid(geometry_msgs::PointStamped centroid)
{
   pub_object_centroid_.publish(centroid);
}

void PC_PUB_SUB::publishObjectCentroidVector(const vector< double> &centroid)
{
  geometry_msgs::PointStamped msg;
   msg.header.stamp = ros::Time::now();
   msg.point.x = centroid[0];
   msg.point.y = centroid[1];
   msg.point.z = centroid[2];
   pub_object_centroid_.publish(msg);
}

void PC_PUB_SUB::publishObjectArray(semantic_segmentation_ros::SegmentationObjectArray object_array)
{
	pub_object_.publish(object_array);
}


void PC_PUB_SUB::visualizeCentorid(geometry_msgs::PointStamped point, string frame, int number)
{
	// Visualisation marker
	visualization_msgs::Marker marker;
	marker.header.stamp = ros::Time::now();
	marker.header.frame_id = frame;
	marker.id = 1;
	marker.ns = "point" + std::to_string(number);
	marker.type = 2;
	marker.action = 0;
	marker.pose.position.x = point.point.x;
	marker.pose.position.y = point.point.y;
	marker.pose.position.z = point.point.z;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 0.25;
	marker.scale.y = 0.25;
	marker.scale.z = 0.25;
	marker.color.a = 1.0;
	marker.color.r = 1.0 / number; 
	marker.color.g = 0.549;
	marker.color.b = 0.0;
	marker.lifetime = ros::Duration(10);
	pub_object_marker_.publish(marker);
}