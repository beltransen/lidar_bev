#include <lidar_bev/cloud_filter.hpp>

#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_msgs/TFMessage.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <camera_info_manager/camera_info_manager.h>

#include <sstream>
#include <vector>
#include <string>
#include <errno.h>

using namespace std;

CloudFilter filter;

const int MAX_CROP = 70;
const int MIN_CROP = -70;

double camera_fov;

// Floor removal parameters
double cell_size_height_map;
int grid_dim_height_map;
double height_threshold;

// Birdview parameters
double cell_size;
int ground_cell_span;
int grid_dim;
int grid_min_x, grid_max_x, grid_min_y, grid_max_y;
double max_height;
double min_height;
int num_slices;

typedef cv::Vec<char, 6> Vec6c;

tf::Transform read_calib(string calib_name){
    sensor_msgs::CameraInfo camera_info_left, camera_info_right;

    tf::Matrix3x3 velo2cam_rot_;
    tf::Vector3 velo2cam_origin_;
    // tf::Quaternion velo2cam_quat_;

    // tf::Matrix3x3 imu2velo_rot_;
    // tf::Vector3 imu2velo_origin_;

    FILE * calib_file = fopen(calib_name.c_str(),"r");
    char str[255];
    double trash;
    double P[12], P2[12];
    int read = fscanf(calib_file, "%s %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
                       str,
                       &trash,    &trash,    &trash,    &trash,
                       &trash,    &trash,    &trash,    &trash,
                       &trash,    &trash,    &trash,    &trash);
    read = fscanf(calib_file, "%s %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
                   str,
                   &trash,    &trash,    &trash,    &trash,
                   &trash,    &trash,    &trash,    &trash,
                   &trash,    &trash,    &trash,    &trash);
    read = fscanf(calib_file, "%s %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
                   str,
                   &P[0],    &P[1],    &P[2],    &P[3],
                   &P[4],    &P[5],    &P[6],    &P[7],
                   &P[8],    &P[9],    &P[10],   &P[11]);
    for (int ix=0; ix<12; ix++){
        camera_info_left.P[ix] = P[ix];
    }

    read = fscanf(calib_file, "%s %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
                   str,
                   &P2[0],    &P2[1],    &P2[2],    &P2[3],
                   &P2[4],    &P2[5],    &P2[6],    &P2[7],
                   &P2[8],    &P2[9],    &P2[10],   &P2[11]);
    for (int ix=0; ix<12; ix++){
        camera_info_right.P[ix] = P2[ix];
    }

    camera_info_right.P[3] -= camera_info_left.P[3];


    //Velo To cam
    //R0_rect
    double R0[9];
    read = fscanf(calib_file, "%s %lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
                   str,
                   &R0[0],    &R0[1],    &R0[2],
                   &R0[3],    &R0[4],    &R0[5],
                   &R0[6],    &R0[7],    &R0[8]);

    //Tr_velo_to_cam
    double TR[12];
    read = fscanf(calib_file, "%s %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
                   str,
                   &TR[0],    &TR[1],    &TR[2],    &TR[3],
                   &TR[4],    &TR[5],    &TR[6],    &TR[7],
                   &TR[8],    &TR[9],    &TR[10],   &TR[11]);

    velo2cam_rot_.setValue(TR[0], TR[1], TR[2],
                           TR[4], TR[5], TR[6],
                           TR[8], TR[9], TR[10]);

    velo2cam_origin_.setValue(TR[3]-P[3]/P[0], TR[7]-P[7]/P[5], TR[11]-P[11]);
    // velo2cam_rot_.getRotation(velo2cam_quat_);
    tf::Transform velo2cam;
    velo2cam.setOrigin(velo2cam_origin_);
    velo2cam.setBasis(velo2cam_rot_);

    // double IMU[12];
    // fscanf(calib_file, "%s %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
    //                str,
    //                &IMU[0],    &IMU[1],    &IMU[2],    &IMU[3],
    //                &IMU[4],    &IMU[5],    &IMU[6],    &IMU[7],
    //                &IMU[8],    &IMU[9],    &IMU[10],   &IMU[11]);

    // imu2velo_rot_.setValue(IMU[0], IMU[1], IMU[2],
    //                        IMU[4], IMU[5], IMU[6],
    //                        IMU[8], IMU[9], IMU[10]);

    // imu2velo_origin_.setValue(IMU[3], IMU[7], IMU[11]);


    fclose(calib_file);
    return velo2cam;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr getPointCloud(std::string path){
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    std::fstream file(path.c_str(), std::ios::in | std::ios::binary);
    if(file.good()){
        file.seekg(0, std::ios::beg);
        int i;
        for (i = 0; file.good() && !file.eof(); i++) {
            pcl::PointXYZI point;
            file.read((char *) &point.x, 3*sizeof(float));
            file.read((char *) &point.intensity, sizeof(float));
            cloud->push_back(point);
        }
        file.close();
    }

    return cloud;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "lidar_bev");
    ros::NodeHandle public_nh, private_nh("~");

    int planes;
    double h_res;
    double v_res;
    double low_opening;
    bool split_channels;
    bool get_ground;
    bool remove_floor;

    string package_dir = ros::package::getPath("lidar_bev");
	string kitti_dir;
    string split_dir;
    string saving_path;
    private_nh.param<string>("kitti_dir", kitti_dir, "kitti");
    private_nh.param<string>("split", split_dir, "training");
    private_nh.param<string>("saving_path", saving_path, "bev_images");
    private_nh.param("camera_fov", camera_fov, 110.0);
    private_nh.param("planes", planes, 64);
    private_nh.param("h_res", h_res, 0.2); // 0.0034889);
    private_nh.param("v_res", v_res, 0.4); // 0.4 for 64 planes
    private_nh.param("low_opening", low_opening, 24.9); // -24.9 for 64 planes
    private_nh.param("cell_size", cell_size, 0.1);
    private_nh.param("ground_cell_span", ground_cell_span, 40);
    private_nh.param("grid_dim", grid_dim, 10); //300*cell_size = total pointcloud size
    private_nh.param("grid_min_y", grid_min_y, MIN_CROP); // enable non-square BEV images by cropping on the left
    private_nh.param("grid_max_y", grid_max_y, MAX_CROP); // enable non-square BEV images by cropping on the right
    private_nh.param("grid_min_x", grid_min_x, MIN_CROP); // enable non-square BEV images by cropping on the bottom
    private_nh.param("grid_max_x", grid_max_x, MAX_CROP); // enable non-square BEV images by cropping on the top
    private_nh.param("max_height", max_height, 3.0);
    private_nh.param("min_height", min_height, -9999.9);
    private_nh.param("num_slices", num_slices, 3);
    private_nh.param("height_threshold", height_threshold, 0.10);
    private_nh.param("cell_size_height_map", cell_size_height_map, 0.25);
    private_nh.param("grid_dim_height_map", grid_dim_height_map, 300);//300*cell_size = total pointcloud size
    private_nh.param("split_channels", split_channels, false); // If True, force save channels as different images
    private_nh.param("get_ground", get_ground, false); // Generate the ground estimation file
    private_nh.param("remove_floor", remove_floor, false);

    if(split_dir.compare("training") != 0 && split_dir.compare("testing") != 0){
        cout << "[ERROR] Invalid split directory " << split_dir << endl;
        return 1;
    }

    // Init the lidar - base transformation for the filter
    tf::StampedTransform base_velo_transform;

    // WARNING: lidar height fixed
    base_velo_transform.setOrigin(tf::Vector3(0.0, 0.0, 1.73));
    filter.setVeloToBaseTransform(base_velo_transform);
    filter.initMaxPointsMap(grid_dim, cell_size, 0, max_height, num_slices, planes, low_opening, h_res, v_res);
    cout << "Saving imgs in: " << saving_path << endl;
    mkdir(saving_path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

    // Load and store the tf transforms in a map
    // Key: timestamp
    // Value: transform
    cout << "Reading tf data..." << endl;
    std::map<ros::Time, tf::StampedTransform> tf_map;

    struct dirent *entry;
    string calib_dir = package_dir + "/" + kitti_dir + "/" + split_dir+ "/calib/";
    cout << calib_dir << endl;
    DIR *dir = opendir(calib_dir.c_str());

    if(dir == NULL){
        cout << "[ERROR] Unable to read calibration files." << endl;
        cout << strerror(errno)<<endl;
        return 1;
    }

    // Parse all calibration files and build corresponding map
    int tfcount = 0;
    while ((entry = readdir(dir)) != NULL) {
        string file_name = calib_dir + entry->d_name;
        tf::Transform velo2cam = read_calib(file_name);

        string frame_num = file_name.erase(0, file_name.find_last_of("/") + 1);
        frame_num.erase(frame_num.find_last_of("."), string::npos);
        if (frame_num.length() != 6){
            continue;
        }

        tf::StampedTransform tf_transform(velo2cam, ros::Time(stoi(frame_num)), "stereo_camera", "velodyne");
        tf_map[tf_transform.stamp_] = tf_transform;
        ++tfcount;
    }

    if (tfcount == 0){
        cout << "[ERROR] Unable to read calibration files." << endl;
        return 1;
    }else{
        cout << "Finished processing calibration files " << endl;
    }

    cout << "Processing pointclouds..." << endl;

    // Build list of channels to compute
    std::vector<std::string> channel_names;
    if (min_height != -9999.9){
        channel_names.push_back("min_height");
    }
    channel_names.push_back("max_height");
    for (int i = 0; i < num_slices; i++){
        channel_names.push_back("dchop_" + to_string(i) + "_" + to_string(i+1));
    }
    channel_names.push_back("avg_intensity");

    int frames = 0;
    string velo_dir = package_dir + "/" + kitti_dir + "/" + split_dir + "/velodyne/";
    // cout << velo_dir << endl;
    DIR *dir_velo= opendir(velo_dir.c_str());
    while ((entry = readdir(dir_velo)) != NULL) {
        string pc_dir = velo_dir + entry->d_name;
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr = getPointCloud(pc_dir);

        string frame_num = pc_dir.erase(0, pc_dir.find_last_of("/") + 1);
        frame_num.erase(frame_num.find_last_of("."), string::npos);
        if (frame_num.length() != 6){
            continue;
        }

        if (cloud_ptr != NULL){
            bool already_computed = true;
            for (int i = 0; i < channel_names.size(); ++i){
                std::stringstream saving_absolute;
                saving_absolute << saving_path << "/";
                saving_absolute << setfill('0') << setw(6) << frame_num;
                saving_absolute << "_" << channel_names[i] << ".png";
                ifstream f(saving_absolute.str());
                if(!f.good()){
                    already_computed = false;
                    break;
                }
                f.close();
            }

            // Skip iteration if existing output
            if(already_computed){
                cout << "Skipping frame " << frame_num << endl;
                continue;
            }
            // Update the filter cloud
            filter.setInputCloud(cloud_ptr);

            // Get the transform from the tf map at the cloud timestamp
            tf::StampedTransform velo_cam_transform = tf_map[ros::Time(stoi(frame_num))];
            // Update lidar transform in filter
            filter.setVeloToCamTransform(velo_cam_transform);

            // Remove points out of the camera FOV
            filter.filterFOV(camera_fov);

            // Remove floor points
            if(remove_floor){
                filter.removeFloor(cell_size_height_map, height_threshold, grid_dim_height_map);
            }

            // Compute the birdview
            std::shared_ptr<cv::Mat> bird_view = filter.birdView(cell_size, max_height, num_slices, grid_dim);

            cv::Mat final_birdview = bird_view->clone();

            // Crop to desired region
            // if(grid_min_x!=MIN_CROP || grid_min_y!=MIN_CROP || grid_max_x!=MAX_CROP || grid_max_y!=MAX_CROP){
            assert(grid_min_x>=-grid_dim/2.);
            assert(grid_min_y>=-grid_dim/2.);
            assert(grid_max_x<=grid_dim/2.);
            assert(grid_max_y<=grid_dim/2.);
            assert(grid_min_y<grid_max_y);
            assert(grid_min_x<grid_max_x);

            int x_max = final_birdview.rows/2.-grid_min_x/cell_size;
            int y_min = final_birdview.cols/2.+grid_min_y/cell_size;

            int h_pixels = (grid_max_x-grid_min_x)/cell_size;
            int w_pixels = (grid_max_y-grid_min_y)/cell_size;

            int x_min = x_max - h_pixels;

            final_birdview = final_birdview(cv::Rect(y_min, x_min, w_pixels, h_pixels));

            if(get_ground){
                std::stringstream saving_ground_absolute;
                saving_ground_absolute << saving_path << "/ground_";
                saving_ground_absolute << setfill('0') << setw(6) << frame_num;
                saving_ground_absolute << ".txt";

                // Compute the ground birdview
                std::shared_ptr<cv::Mat> bird_ground = filter.birdGround(cell_size, ground_cell_span, grid_dim);
                cv::Mat final_birdground = bird_ground->clone();

                // Crop to desired region
                final_birdground = final_birdground(cv::Rect(y_min, x_min, w_pixels, h_pixels));

                // Save the ground bird image in disk
                ofstream fout(saving_ground_absolute.str());
                // cout << saving_ground_absolute.str() << endl;
                if(!fout){
                    cout<<"[ERROR] Unable to create ground file at " << saving_ground_absolute.str() << ". Skipping." <<endl;
                }else{
                    for(int i=0; i<final_birdground.rows; i++)
                    {
                        for(int j=0; j<final_birdground.cols; j++)
                        {
                            fout<<final_birdground.at<float>(i,j)<<" ";
                        }
                        fout<<endl;
                    }
                }
                fout.close();
                bird_ground->release();
                final_birdground.release();
            }

            // Save birdview images
            std::vector<cv::Mat> channels;
            cv::split(final_birdview, channels);
            if (min_height == -9999.9){
                channels.erase(channels.begin()); // First channel is min_height
            }

            if (channels.size() == 3 && !split_channels){
                // Single png image
                cv::Mat three_ch;
                cv::merge(channels, three_ch);

                stringstream saving_absolute;
                saving_absolute << saving_path << "/";
                saving_absolute << setfill('0') << setw(6) << frame_num;
                saving_absolute << ".png";

                cv::imwrite(saving_absolute.str(), three_ch);
            } else {
                // Save image channels separately
                for (int i = 0; i < channels.size(); ++i){
                    stringstream saving_absolute;
                    saving_absolute << saving_path << "/";
                    saving_absolute << setfill('0') << setw(6) << frame_num;
                    saving_absolute << "_" << channel_names[i] << ".png";
                    cv::imwrite(saving_absolute.str(), channels[i]);
                }
            }
            cout << "Finished processing frame " <<  frame_num << endl;

            // Increase index
            final_birdview.release();
            bird_view->release();
            frames++;
        }
    }

    if (frames == 0){
        cout << "[ERROR] Unable to process lidar files." << endl;
        return 1;
    }

    cout << "[SUCCESS] Process fininshed. Total number of frames: " << frames << endl;

    return 0;
}
