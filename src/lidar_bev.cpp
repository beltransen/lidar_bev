#include <lidar_bev/cloud_filter.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <sys/stat.h>
#include <sstream>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

using namespace std;
using namespace cv;

image_transport::Publisher bird_view_pub, bird_ground_pub;
ros::Publisher cloud_pub, ground_cloud_pub;
CloudFilter filter;

double camera_fov;
double intensity_threshold;

//floor removal parameters
bool remove_floor;
double cell_size,height_threshold,max_height,cell_size_height_map;
int grid_dim,grid_dim_height_map;
int ground_cell_span;
int num_slices;

void cloud_callback(const sensor_msgs::PointCloud2Ptr & cloud_msg){

    // Change the intensity field name, so we can use it with pcl point type
    cloud_msg->fields[3].name = "intensity";

    // Convert cloud msg to pcl type
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::moveFromROSMsg(*cloud_msg, *cloud_ptr);

    // Update the filter cloud
    filter.setInputCloud(cloud_ptr);

    // Remove points out of the camera FOV
    //    filter.filterFOV(camera_fov);

    std::shared_ptr<Mat> bird_ground = filter.birdGround(cell_size, ground_cell_span, grid_dim);
    // Remove floor points
    if(remove_floor){
        filter.removeFloor(cell_size_height_map, height_threshold, grid_dim_height_map);
    }

    std::shared_ptr<Mat> bird_view = filter.birdView(cell_size, max_height, grid_dim, false);

    int grid_cells = grid_dim / cell_size; // Number of col/rows of the birdview

    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    for(int i=0; i<bird_ground->rows; i++){
        float* row_ptr = bird_ground->ptr<float>(i);
        for(int j=0; j<bird_ground->cols; j++){
            float z = row_ptr[j] - filter.getBaseVeloTF().getOrigin().z();
            double x = (grid_cells/2. - i)*cell_size;
            double y = (grid_cells/2. - j)*cell_size;

            // Push the ground XYZ point
            pcl::PointXYZ point;
            point.x = x;
            point.y = y;
            point.z = z;
            ground_cloud_ptr->push_back(point);
        }
    }

    // Publish the ground pointcloud
    sensor_msgs::PointCloud2 ground_ros;
    pcl::toROSMsg(*ground_cloud_ptr, ground_ros);
    ground_ros.header = cloud_msg->header;
    ground_cloud_pub.publish(ground_ros);

    // Publish bird_view img
    cv_bridge::CvImage cv_bird_view;
    cv_bird_view.header = cloud_msg->header;
    cv_bird_view.encoding = "bgr8";
    cv_bird_view.image = *bird_view;
    bird_view_pub.publish(cv_bird_view.toImageMsg());

    // Publish ground img
    cv_bridge::CvImage cv_ground_view;
    cv_ground_view.header = cloud_msg->header;
    cv_ground_view.encoding = "32FC1";
    cv_ground_view.image = *bird_ground;
    bird_ground_pub.publish(cv_ground_view.toImageMsg());

    // Publish the filtered cloud
    cloud_pub.publish(*cloud_ptr);
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "lidar_bev");
    ros::NodeHandle public_nh, private_nh("~");
    image_transport::ImageTransport it(public_nh);

    int planes;
    double h_res, low_opening, v_res;
    float max_expected_intensity_;

    string cloud_topic;
    private_nh.param<string>("cloud_topic", cloud_topic, "/kitti/velo/pointcloud");
    string output_topic;
    private_nh.param<string>("output_topic", output_topic, "filtered_cloud");
    string lidar_tf_frame;
    private_nh.param<string>("lidar_tf_frame", lidar_tf_frame, "/velo_link");
    string camera_tf_frame;
    private_nh.param<string>("camera_tf_frame", camera_tf_frame, "/camera_color_left");
    private_nh.param("camera_fov", camera_fov, 110.0);
    private_nh.param("intensity_threshold", intensity_threshold, 0.05);
    private_nh.param("planes", planes, 32);
    private_nh.param("h_res", h_res, 0.0034889);
    public_nh.param("cell_size", cell_size, 0.1);
    public_nh.param("/lidar_bev/cell_size", cell_size, 0.1);
    public_nh.param("/lidar_bev/ground_cell_span", ground_cell_span, 40);
    private_nh.param("cell_size_height_map", cell_size_height_map, 0.25);
    private_nh.param("height_threshold", height_threshold, 0.10);
    private_nh.param("max_height", max_height, 3.0);//not used, dont know if useful, there are buses that are quite high
    private_nh.param("num_slices", num_slices, 3);//not used, dont know if useful, there are buses that are quite high
    public_nh.param("grid_dim", grid_dim, 1000);//300*cell_size = total pointcloud size
    public_nh.param("/lidar_birdview/grid_dim", grid_dim, 1000);//300*cell_size = total pointcloud size
    private_nh.param("grid_dim_height_map", grid_dim_height_map, 300);//300*cell_size = total pointcloud size
    private_nh.param("low_opening", low_opening, 24.9);//-24.9 for 64 planes
    private_nh.param("v_res", v_res, 0.4);//0.4 for 64 planes
    private_nh.param("max_expected_intensity", max_expected_intensity_, 1.0f);
    private_nh.param("remove_floor", remove_floor, false);

    // Init the lidar - camera transformation for the filter
    filter.setIntensityNormalization(max_expected_intensity_);
    filter.initTF(lidar_tf_frame, camera_tf_frame);
    filter.initMaxPointsMap(grid_dim, cell_size, 0, max_height, num_slices, planes, low_opening, h_res, v_res);

    bird_view_pub = it.advertise("bird_view", 1);
    bird_ground_pub = it.advertise("bird_ground", 1);
    ground_cloud_pub = public_nh.advertise<sensor_msgs::PointCloud2> ("ground_cloud", 1);
    cloud_pub = public_nh.advertise<pcl::PointCloud<pcl::PointXYZI>>(output_topic, 1);
    ros::Subscriber cloud_sub = public_nh.subscribe(cloud_topic, 1, cloud_callback);

    while (ros::ok())
    {
        waitKey(10);
        ros::spinOnce();
    }

    destroyAllWindows();
    return 0;
}
