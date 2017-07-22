/*
 * ground_filter.cpp
 *
 * Created on	: May 19, 2017
 * Author	: Patiphon Narksri					
 */
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <velodyne_pointcloud/point_types.h>
#include <opencv/cv.h>

#include <boost/lexical_cast.hpp> //U
#include <boost/chrono.hpp> //U
#include <iostream> //U

enum Label
{
        GROUND = 0,
        VERTICAL = 1,
        UNKNOWN = 3
};

class GroundFilter
{
public:
	
	GroundFilter();

private:

	ros::NodeHandle n;
        ros::Subscriber sub;
	ros::Publisher vertical_points_pub;
	ros::Publisher ground_points_pub;

        std::string     point_topic;
	int 		sensor_model;
	double 		sensor_height;
	double 		max_slope;
	int 		min_point;
	double 		clipping_thres;
	double 		gap_thres;
	double 		point_distance;
        bool            floor_removal; 

	int 		vertical_res;
	int 		horizontal_res;
	double 		limiting_ratio;
	cv::Mat 	index_map;
	Label 		class_label[64];
	double 		optimal_radius[64];

	//These will be deleted
	int		original_point;
	int		remaining_point;
	int		point_after_tf;

	boost::chrono::high_resolution_clock::time_point t1;
	boost::chrono::high_resolution_clock::time_point t2;
	boost::chrono::nanoseconds elap_time;

	void initLabelArray(int model);
	void initRadiusArray(double radius[], int model);
	void initDepthMap(int width);
	void publishPoint(const pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::ConstPtr &msg,
				int index[], int &index_size, 
				pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &topic);


	void velodyneCallback(const pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::ConstPtr &msg);
	void groundSeparate(const pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::ConstPtr &msg, 
				pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &vertical_points, 
				pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &ground_points);

};

GroundFilter::GroundFilter() : n("~")
{

	n.param<std::string>("point_topic", point_topic, "/points_raw");
	//If it set to False it will publish the original PointCloud
 	n.param("remove_floor",  floor_removal,  true);
	//Can be selected between 16, 32 and 64 (Have never tested on 16 though)
        n.param("sensor_model", sensor_model, 64);
	//This is the height of Velodyne measured from center of Velodyne to ground
        n.param("sensor_height", sensor_height, 1.72);
	//Maximum allowable slope i.e., any surface steeper than this angle[deg] will not be removed
        n.param("max_slope", max_slope, 3.0);

	//These parameters have been tested to be the optimal values for this algorithm
	//Shouldn't have to be changed for normal use
	n.param("point_distance", point_distance, 0.05);
        n.param("min_point", min_point, 4);
	n.param("clipping_thres", clipping_thres, 1.72);
	n.param("gap_thres", gap_thres, 0.15);

	//Number of laser rays in vertical direction
	vertical_res 	= 64;
	//Number of laser rays in horizontal direction (will be recalculated every rotation)
	horizontal_res 	= 2000;
	//Use tan instead of max_slope in degree for more efficient computation
	limiting_ratio 	= tan(20.0*M_PI/180);

	vertical_res = sensor_model;
	initLabelArray(sensor_model);
	limiting_ratio = tan(max_slope*M_PI/180);
	initRadiusArray(optimal_radius, sensor_model); 	
       
	sub = n.subscribe(point_topic, 10, &GroundFilter::velodyneCallback, this);
        vertical_points_pub = n.advertise<sensor_msgs::PointCloud2>("/points_lanes", 10);
        ground_points_pub = n.advertise<sensor_msgs::PointCloud2>("/points_ground", 10);
}
//This loop calculate the expected spaced between consecutive rings
void GroundFilter::initRadiusArray(double radius[], int model)
{
	for (int i = 0; i < model; i++)
	{
		radius[i] = i*0.004 + 0.00005;
	}
}

//Create an enum array to store the current status of each point in the same bearing angle
void GroundFilter::initLabelArray(int model)
{
	for(int a = 0; a < vertical_res; a++)
	{
		class_label[a] = UNKNOWN;
	}
}

//Create a depth map that has a size of vertical_res x horizontal_res
void GroundFilter::initDepthMap(int width)
{
	const int mOne = -1;
	index_map = cv::Mat_<int>(vertical_res, width, mOne);
}

//Used for publish the separated PointCloud to a defined topic
void GroundFilter::publishPoint(const pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::ConstPtr &msg,
				int index[], int &index_size, 
				pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &topic)
{

	velodyne_pointcloud::PointXYZIR point;
	for (int i = 0; i < index_size; i++)
	{
		point.x = msg->points[index[i]].x;
		point.y = msg->points[index[i]].y;
		point.z = msg->points[index[i]].z;
		point.intensity = msg->points[index[i]].intensity;
		point.ring = msg->points[index[i]].ring;
		topic.push_back(point);

		remaining_point++;
	}
	index_size = 0;	

}

//Main calculation is done in this function
void GroundFilter::groundSeparate(const pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::ConstPtr &msg, 
			pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &vertical_points, 
			pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &ground_points)
{

        velodyne_pointcloud::PointXYZIR point;
	
        horizontal_res = int(msg->points.size() / vertical_res);
        initDepthMap(horizontal_res);

	original_point = msg->points.size();
	remaining_point = 0;
	point_after_tf = 0;	

	//This conversion has some losses
	//Convert coordinate of each point from Cartesian to Spherical (XYZ -> DepthMap)
        for (int i = 0; i < msg->points.size(); i++)
        {
                double u = atan2(msg->points[i].y,msg->points[i].x) * 180/M_PI;
                if (u < 0) u = 360 + u;  
                int column = horizontal_res - (int)((double)horizontal_res * u / 360.0) - 1;   
                int row = vertical_res - 1 - msg->points[i].ring;
		index_map.at<int>(row, column) = i;
        }

	//Iterate through each bearing angle (each horizontal angle)
	for (int i = 0; i < horizontal_res; i++)
        {
                Label point_class[vertical_res];
		int unknown_index[vertical_res];
		int point_index[vertical_res];
		int unknown_index_size = 0;
		int point_index_size = 0;
		double z_ref = 0;
		double r_ref = 0;
		//Initialize the enum array to be UKNOWN in every elements before calculation
		//Of each bearing angle
		std::copy(class_label, class_label + vertical_res, point_class); 

		//Iterate through each vertical angle (each laser ray) starting from lowest ray
		for (int j = vertical_res - 1; j >= 0; j--)
                {
			//If the point has already been processed and already classified
			//It will not be processed again
                        if (index_map.at<int>(j,i) > -1 && point_class[j] == UNKNOWN)
                        {
				point_after_tf++;
				double x0 = msg->points[index_map.at<int>(j, i)].x;
				double y0 = msg->points[index_map.at<int>(j, i)].y;
				double z0 = msg->points[index_map.at<int>(j, i)].z;
				double r0 = sqrt(x0*x0 + y0*y0);
				double r_diff = r0 - r_ref;
				double z_diff = fabs(z0 - z_ref);
				double pair_angle = z_diff/r_diff;
				//Check if the angle between the current and the previous point is less than a defined maximum_slope
				//If the angle is less than maximum_slope, add the current point to "Candidate group"
				if (((pair_angle > 0 && pair_angle < limiting_ratio) && z_diff < gap_thres && z0 < clipping_thres - sensor_height) || point_index_size == 0)
				{
					r_ref = r0;
					z_ref = z0;
					point_index[point_index_size] = j;
					point_index_size++;
				} else {
					//If the angle exceeds the maximum slope
					//Check number of point in "Candidate group", if exceeds the minimum_point threshold
					//Publish them as ground points
					if (point_index_size > min_point)
					{
						for (int m = 0; m < point_index_size; m++)
						{
								
								int index = index_map.at<int>(point_index[m],i);
								point.x = msg->points[index].x;
								point.y = msg->points[index].y;
								point.z = msg->points[index].z;
								point.intensity = msg->points[index].intensity;
								point.ring = msg->points[index].ring;
								ground_points.push_back(point);
								point_class[point_index[m]] = GROUND;

								remaining_point++;
						}
						point_index_size = 0;
					//If the number of point in "Candidate group" is less than the threshold, continue the calculation
					} else {
						for (int m = 0; m < point_index_size; m++)
						{
							int index = index_map.at<int>(point_index[m],i);
							point.z = msg->points[index].z;
							//Publish every points that are higher than clipping_threshold as vertical points
							if (point.z > clipping_thres - sensor_height)
							{
								point.x = msg->points[index].x;
								point.y = msg->points[index].y;
								point.intensity = msg->points[index].intensity;
								point.ring = msg->points[index].ring;
								vertical_points.push_back(point);
								point_class[point_index[m]] = VERTICAL;


								remaining_point++;
							//Mark the remaining points as Unknown points for further calculation
							} else {
								unknown_index[unknown_index_size] = index;
								unknown_index_size++;
							}
						}
						point_index_size = 0;
					}

					r_ref = r0;
					z_ref = z0;
					point_index[point_index_size] = j;
					point_index_size++;
				}
  			}
			//If the highest ray is reached
                        if (j == 0)
                        {
				//First, check if the "Candidate group" contain any point, if so classify them using
				//the same criteria as above
				if (point_index_size != 0)
				{
					if (point_index_size > min_point)
					{
						for (int m = 0; m < point_index_size; m++)
						{
								
								int index = index_map.at<int>(point_index[m],i);
								point.x = msg->points[index].x;
								point.y = msg->points[index].y;
								point.z = msg->points[index].z;
								point.intensity = msg->points[index].intensity;
								point.ring = msg->points[index].ring;
								ground_points.push_back(point);
								point_class[point_index[m]] = GROUND;

								remaining_point++;
						}
						point_index_size = 0;
					} else {
						for (int m = 0; m < point_index_size; m++)
						{
							int index = index_map.at<int>(point_index[m],i);
							point.z = msg->points[index].z;
							if (point.z > clipping_thres - sensor_height)
							{
								point.x = msg->points[index].x;
								point.y = msg->points[index].y;
								point.intensity = msg->points[index].intensity;
								point.ring = msg->points[index].ring;
								vertical_points.push_back(point);
								point_class[point_index[m]] = VERTICAL;

								remaining_point++;
							} else {
								unknown_index[unknown_index_size] = index;
								unknown_index_size++;
							}
						}
						point_index_size = 0;
					}
				} 
				//Lastly, the remaining unknown points are checked using different approach
				//Check if the radial distance between two consecutive points is less than 
				//point_distance threshold, if so classify them as vertical
				double centroid = 0;
				int centroid_ring = 0;
				int cluster_index[vertical_res];
				int cluster_index_size = 0;
				for (int m = unknown_index_size - 1; m >= 0; m--)
				{
					double x0 = msg->points[unknown_index[m]].x;
					double y0 = msg->points[unknown_index[m]].y;
					double r0 = sqrt(x0*x0 + y0*y0);
					double r_diff = fabs(r0 - centroid);
					//point_distance = 0.2;
					point_distance = optimal_radius[centroid_ring];
					if ((r_diff < point_distance) || cluster_index_size == 0)
					{
						cluster_index[cluster_index_size] = unknown_index[m];
						cluster_index_size++;
						centroid = r0;
						centroid_ring = msg->points[unknown_index[m]].ring;
					} else {
						if(cluster_index_size > 1)
						{
							publishPoint(msg, cluster_index	, cluster_index_size, vertical_points);
						} else {
							publishPoint(msg, cluster_index, cluster_index_size, ground_points);
						}
						
						cluster_index[cluster_index_size] = unknown_index[m];
						cluster_index_size++;
						centroid = r0;
					}
					if (m == 0)
					{
						if(cluster_index_size > 1)
						{
							publishPoint(msg, cluster_index, cluster_index_size, vertical_points);
						} else {
							publishPoint(msg, cluster_index, cluster_index_size, ground_points);
						}
					}
				}
                        }
                }
	}
}

void GroundFilter::velodyneCallback(const pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::ConstPtr &msg)
{
	t1 = boost::chrono::high_resolution_clock::now();
	
	pcl::PointCloud<velodyne_pointcloud::PointXYZIR> vertical_points;
	pcl::PointCloud<velodyne_pointcloud::PointXYZIR> ground_points;
	vertical_points.header = msg->header;
        ground_points.header = msg->header;
        vertical_points.clear();
        ground_points.clear();

	groundSeparate(msg, vertical_points, ground_points);

	if (!floor_removal)
	{
		vertical_points = *msg;
	} 
	
	vertical_points_pub.publish(vertical_points);
        ground_points_pub.publish(ground_points);

	t2 = boost::chrono::high_resolution_clock::now();
        elap_time = (boost::chrono::duration_cast<boost::chrono::nanoseconds>(t2-t1));
        //std::cout << "Computational time for each frame is " << elap_time <<std::endl;
        std::cout << "Original point is " << original_point << " The remaining point is " << remaining_point << 
	" Lost point is " << original_point - remaining_point << " Point after transform " << point_after_tf<<
	" Real missing point is " << point_after_tf - remaining_point  << std::endl;
}

int main(int argc, char **argv)
{

        ros::init(argc, argv, "ground_filter");
	GroundFilter node;
        ros::spin();

	return 0;

}
