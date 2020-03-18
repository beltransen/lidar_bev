#include <lidar_bev/cloud_filter.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

using namespace std;

CloudFilter filter;

double camera_fov;
// Floor removal parameters
double cell_size_height_map;
int grid_dim_height_map;
double height_threshold;
// Birdview parameters
double cell_size;
int ground_cell_span;
int grid_dim;
double max_height;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_bev");
    ros::NodeHandle public_nh, private_nh("~");

    int planes;
    double h_res;
    double v_res;
    double low_opening;
    bool crop_180;
    bool remove_floor;

	string rosbag_name;
	string saving_path;
	string output_path;
    private_nh.param<string>("rosbag_name", rosbag_name, "/data/temp/kitti_obj_training.bag");
    private_nh.param<string>("output_path", output_path, "/media/datasets/birdview_paper/training_images/");
    string cloud_topic;
    private_nh.param<string>("cloud_topic", cloud_topic, "/velodyne_points");
    string lidar_tf_frame;
    private_nh.param<string>("lidar_tf_frame", lidar_tf_frame, "lidar");
    string camera_tf_frame;
    private_nh.param<string>("camera_tf_frame", camera_tf_frame, "stereo_camera");
    private_nh.param("camera_fov", camera_fov, 110.0);
    private_nh.param("planes", planes, 64);
    private_nh.param("h_res", h_res, 0.2); // 0.0034889);
    private_nh.param("v_res", v_res, 0.4); // 0.4 for 64 planes
    private_nh.param("low_opening", low_opening, 24.9); // -24.9 for 64 planes
    private_nh.param("cell_size", cell_size, 0.1);
    private_nh.param("ground_cell_span", ground_cell_span, 40);
    private_nh.param("grid_dim", grid_dim, 10); //300*cell_size = total pointcloud size
    private_nh.param("max_height", max_height, 3.0);
    private_nh.param("height_threshold", height_threshold, 0.10);
    private_nh.param("cell_size_height_map", cell_size_height_map, 0.25);
    private_nh.param("grid_dim_height_map", grid_dim_height_map, 300);//300*cell_size = total pointcloud size
    private_nh.param("crop_180", crop_180, true); // Get only the fron of the vehicle
    private_nh.param("remove_floor", remove_floor, false);

    // Init the lidar - base transformation for the filter
    tf::StampedTransform base_velo_transform;
    // WARNING: lidar height fixed
    base_velo_transform.setOrigin(tf::Vector3(0.0, 0.0, 1.73));
    filter.setVeloToBaseTransform(base_velo_transform);
    filter.initMaxPointsMap(grid_dim, cell_size, 0, max_height, planes, low_opening, h_res, v_res);

    string bagpath = rosbag_name;
    rosbag_name.erase(0, rosbag_name.find_last_of("/") + 1);
    rosbag_name.erase(rosbag_name.find_last_of("."), string::npos);
    saving_path = output_path + rosbag_name;
    cout << "Saving imgs in: " << saving_path << endl;
    mkdir(saving_path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

    rosbag::Bag bag;
    cout << "Opening bag file: " << bagpath << endl;

    bag.open(bagpath, rosbag::bagmode::Read);

    // Load and store the tf transforms in a map
    // Key: timestamp
    // Value: transform
    cout << "Reading tf data..." << endl;
    std::map<ros::Time, tf::StampedTransform> tf_map;

    std::vector<std::string> topics = {"/tf"};
    rosbag::View tf_view(bag, rosbag::TopicQuery(topics));
    int tfcount = 0;
    for (auto m : tf_view)
    {
        tf::tfMessagePtr transforms = m.instantiate<tf::tfMessage>();
        for (const auto& trans : transforms->transforms)
        {
            if (trans.header.frame_id == camera_tf_frame && trans.child_frame_id == lidar_tf_frame)
            {
                tf::StampedTransform tf_transform;
                tf::transformStampedMsgToTF(trans, tf_transform);
                tf_map[tf_transform.stamp_] = tf_transform;
                ++tfcount;
            }
        }
    }
    cout << "Number of tf transforms: " << tfcount << endl;

    cout << "Processing pointclouds..." << endl;
    // Get all clouds and iterate the msgs
    topics = {cloud_topic};
    rosbag::View cloud_view(bag, rosbag::TopicQuery(topics));
    int frames = 0;
    for (auto m : cloud_view)
    {
        sensor_msgs::PointCloud2Ptr cloud_msg = m.instantiate<sensor_msgs::PointCloud2>();
        if (cloud_msg != NULL)
        {
            // Process the cloud msg
            // Change the intensity field name, so we can use it with pcl point type
            cloud_msg->fields[3].name = "intensity";
            // Convert cloud msg to pcl type
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::moveFromROSMsg(*cloud_msg, *cloud_ptr);

            // Update the filter cloud
            filter.setInputCloud(cloud_ptr);

            // Get the transform from the tf map at the cloud timestamp
            tf::StampedTransform velo_cam_transform = tf_map[cloud_msg->header.stamp];
            // Update lidar transform in filter
            filter.setVeloToCamTransform(velo_cam_transform);

            // Remove points out of the camera FOV
            filter.filterFOV(camera_fov);

            // Compute the ground birdview
            std::shared_ptr<cv::Mat> bird_ground = filter.birdGround(cell_size, ground_cell_span, grid_dim);

            // Remove floor points
            if(remove_floor){
                filter.removeFloor(cell_size_height_map, height_threshold, grid_dim_height_map);
            }

            // Compute the birdview
            std::shared_ptr<cv::Mat> bird_view = filter.birdView(cell_size, max_height, grid_dim);

            cv::Mat final_birdview = bird_view->clone();
            cv::Mat final_birdground = bird_ground->clone();
            if (crop_180)
            {
                // Crop the birdview in half (only the front of the vehicle)
                final_birdview = final_birdview(cv::Rect(0, 0, final_birdview.cols, final_birdview.rows/2));
                final_birdground = final_birdground(cv::Rect(0, 0, final_birdground.cols, final_birdground.rows/2));
            }

            // Save the image in disk
            stringstream saving_absolute;
            saving_absolute << saving_path << "/";
            saving_absolute << setfill('0') << setw(6) << frames;
            saving_absolute << ".png";
            cout << "Saving img: " << saving_absolute.str() << endl;
            cv::imwrite(saving_absolute.str(), final_birdview);

            // Save the ground bird image in disk
            stringstream saving_ground_absolute;
            saving_ground_absolute << saving_path << "/ground_";
            saving_ground_absolute << setfill('0') << setw(6) << frames;
            saving_ground_absolute << ".txt";
            cout << "Saving img: " << saving_ground_absolute.str() << endl;

            ofstream fout(saving_ground_absolute.str());

            if(!fout){
                cout<<"File Not Opened"<<endl;
            }else{
                for(int i=0; i<final_birdground.rows; i++)
                {
                    for(int j=0; j<final_birdground.cols; j++)
                    {
                        fout<<final_birdground.at<float>(i,j)<<" ";
                    }
                    fout<<endl;
                }

                fout.close();
            }

            // Increase index
            frames++;
        }
    }

    bag.close();
    cout << "Total frames " << frames << endl;

    return 0;
}
