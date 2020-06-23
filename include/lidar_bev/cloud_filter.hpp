#ifndef CLOUD_FILTER_HPP
#define CLOUD_FILTER_HPP

#include <iostream>
#include <fstream>
#include <sstream>
#include <memory>
#include <cmath>
#include <random>

#include <opencv2/opencv.hpp>

#include <ros/package.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_listener.h>


class CloudFilter
{
public:
    CloudFilter();
    CloudFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud);
    ~CloudFilter();

    void setInputCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud);

    void setIntensityNormalization(float max_intensity){ max_expected_intensity_ = max_intensity;}

    /* Remove the points that are not in the camera FOV (angle in degrees) */
    void filterFOV(double horizontal_fov);

    /* Remove the floor points using a heightmap algorithm */
    void removeFloor(double cell_size, double height_threshold, int grid_dim);

    /* get a 1-channel 2D grid containing the height of the ground plane at each cell */
    std::shared_ptr<cv::Mat> birdGround(double bv_cell_size, int ground_cell_span, double grid_dim);

    /* get the 2D grid birdview of the cloud with 3 channels (height, density, intensity) */
    std::shared_ptr<cv::Mat> birdView(double cell_size, double max_height, int num_slices, double grid_dim, bool sample_points=false);

    /* Remove all points below the given intensity threshold */
    void filterIntensities(double intensity_threshold);

    /* Wait for the transform lidar -> camera and update velo_cam_transform_ */
    void initTF(std::string lidar_frame, std::string camera_frame);

    /* Manually set tf transforms */
    void setVeloToCamTransform(tf::StampedTransform velo_cam_transform);
    void setVeloToBaseTransform(tf::StampedTransform base_velo_transform);

    /* Load values to the max points map, to later compute the density */
    void initMaxPointsMap(int grid_dim, float cell_size, float z_min, float z_max, int num_slices, int planes,
                          float low_angle, float h_res, float v_res);

    tf::StampedTransform getBaseVeloTF(){
        return base_velo_transform_;
    }

private:
    // Cloud to be processed and updated in place
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr_;

    // Transform between the veloyne and the camera. Needed to remove points out of the camera range.
    tf::StampedTransform velo_cam_transform_;
    // Transform between the veloyne and the camera. Needed to know the height
    tf::StampedTransform base_velo_transform_;
    // Trasform listener to get the TFs
    tf::TransformListener* tf_;

    // Float storing max expected intensity value on input pointcloud
    float max_expected_intensity_;

    // matrix to keep the map of the max points that each cell can have
    std::vector<std::vector<std::vector<float> > > max_points_map_;

    /* Return true if the point is inside the camera FOV. Used to filter points in filterFOV */
    bool pointInCameraFov(pcl::PointXYZI p, double horizontal_fov);

    /* Return true if the point is inside a cell considered not ground. Used to filter points in removeFloor */
    bool filterGround(pcl::PointXYZI p,int grid_dim,const std::vector<std::vector<float> > &min,const std::vector<std::vector<float> > &max,const std::vector<std::vector<float> > &init ,const double &height_threshold,const double &cell_size);

};

#endif
