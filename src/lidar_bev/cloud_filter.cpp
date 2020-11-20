#include "lidar_bev/cloud_filter.hpp"


CloudFilter::CloudFilter(){
    max_expected_intensity_ = 1.0f;
}

CloudFilter::CloudFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud){
    max_expected_intensity_ = 1.0f;
    setInputCloud(input_cloud);
}

CloudFilter::~CloudFilter(){
    cloud_ptr_.reset();
}

void CloudFilter::setInputCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud){
    cloud_ptr_.reset();
    cloud_ptr_ = input_cloud;
}

/* Return true if the point is inside the camera FOV. Used to filter points in filterFOV */
bool CloudFilter::pointInCameraFov(pcl::PointXYZI p, double horizontal_fov){
    // Translate the point to the camera frame
    p.x -= velo_cam_transform_.getOrigin().x();
    p.y -= velo_cam_transform_.getOrigin().y();

    // Automatically remove points behind the car or too far
    if (p.x < 0 || p.x > 100) return false;

    //double angle = (p.x != 0.0) ? std::atan(p.y / p.x) : 0.0;
    //return (abs(angle) < (horizontal_fov / 2.0));

    // Instead of computing the atan, just compare the relation between x/y
    return (abs(p.y) < (horizontal_fov/90.0) * p.x);
}

/* Remove the points that are not in the camera FOV */
void CloudFilter::filterFOV(double horizontal_fov){
    // Resize the cloud to make it non-organized and work faster
    cloud_ptr_->width = cloud_ptr_->width * cloud_ptr_->height;
    cloud_ptr_->height = 1;

    // Avoid pcl filters if possible because they copy the cloud inside.
    // Points can be removed using std algorithms (erase + remove_if)
    cloud_ptr_->points.erase(std::remove_if(cloud_ptr_->points.begin(),
                                            cloud_ptr_->points.end(),
                                            [&](const pcl::PointXYZI& p){return !pointInCameraFov(p, horizontal_fov);}),
                             cloud_ptr_->points.end()
                             );
    cloud_ptr_->width = cloud_ptr_->points.size();
}

/* Remove all points above the given intensity threshold */
bool CloudFilter::filterGround(pcl::PointXYZI p, int grid_dim, const std::vector<std::vector<float> > &min, const std::vector<std::vector<float> > &max, const std::vector<std::vector<float> > &init , const double &height_threshold, const double &cell_size){
    int x = ((grid_dim / 2) + p.x / cell_size);
    int y = ((grid_dim / 2) + p.y / cell_size);
    if (x >= 0 && x < grid_dim && y >= 0 && y < grid_dim && init[x][y])
        return ((max[x][y] - min[x][y] < height_threshold) );
}

/* Remove the floor points using a heightmap algorithm */
void CloudFilter::removeFloor(double cell_size, double height_threshold, int grid_dim){

    // Resize the cloud to make it non-organized and work faster
    cloud_ptr_->width = cloud_ptr_->width * cloud_ptr_->height;
    cloud_ptr_->height = 1;

    std::vector<std::vector<float> > min;min.resize(grid_dim,std::vector<float>(grid_dim, 0));
    std::vector<std::vector<float> > max;max.resize(grid_dim,std::vector<float>(grid_dim, 0));
    std::vector<std::vector<float> > init;init.resize(grid_dim,std::vector<float>(grid_dim, 0));


    // build height map
    for (unsigned i = 0; i < cloud_ptr_->points.size(); ++i)
    {
        int x = ((grid_dim / 2) + cloud_ptr_->points[i].x / cell_size);
        int y = ((grid_dim / 2) + cloud_ptr_->points[i].y / cell_size);
        if (x >= 0 && x < grid_dim && y >= 0 && y < grid_dim)
        {
            if (!init[x][y])
            {
                min[x][y] = cloud_ptr_->points[i].z;
                max[x][y] = cloud_ptr_->points[i].z;
                init[x][y] = true;
            }
            else
            {
                min[x][y] = std::min(min[x][y], cloud_ptr_->points[i].z);
                max[x][y] = std::max(max[x][y], cloud_ptr_->points[i].z);
            }
        }
    }

    // Avoid pcl filters if possible because they copy the cloud inside.
    // Points can be removed using std algorithms (erase + remove_if)
    cloud_ptr_->points.erase(std::remove_if(cloud_ptr_->points.begin(),
                                            cloud_ptr_->points.end(),
                                            [&](const pcl::PointXYZI& p)
    {return filterGround(p,grid_dim,min,max,init,height_threshold,cell_size);}),
                             cloud_ptr_->points.end()
                             );
    cloud_ptr_->width = cloud_ptr_->points.size();

}

std::shared_ptr<cv::Mat> CloudFilter::birdGround(double bv_cell_size, int ground_cell_span, double grid_dim){
    int grid_cells = grid_dim / bv_cell_size; // Number of col/rows of the birdview
    int ground_cells = floor(grid_cells / ground_cell_span); // Number of cols/rows of the auxiliary ground bird view
    std::shared_ptr<cv::Mat> bird_ground(new cv::Mat(grid_cells, grid_cells, CV_32FC1));
    std::shared_ptr<cv::Mat> small_ground(new cv::Mat(ground_cells, ground_cells, CV_32FC1));
    std::shared_ptr<cv::Mat> median_ground(new cv::Mat(ground_cells, ground_cells, CV_32FC1));
    float** aux_ground = new float*[ground_cells];

    // Allocate memory
    for (int i = 0; i < ground_cells; ++i)
    {
        aux_ground[i] = new float[ground_cells];
    }

    // Init the grids
    for (int i = 0; i < ground_cells; ++i)
    {
        for (int j = 0; j < ground_cells; ++j){
            aux_ground[i][j] = 9999.9;
        }
    }

    // Fill the aux grid
    for (const auto& point : cloud_ptr_->points)
    {
        if(point.z < -3.0) continue;
        float z = point.z + base_velo_transform_.getOrigin().z();
        int x = grid_cells/2 - point.x/bv_cell_size;
        int y = grid_cells/2 - point.y/bv_cell_size;

        int x_ground = (double)x/ground_cell_span -.5f;
        int y_ground = (double)y/ground_cell_span -.5f;

        // Store min value of pixels belonging to the area
        if (x_ground >= 0 && x_ground < ground_cells && y_ground >= 0 && y_ground < ground_cells){
            // Update cell max height
            aux_ground[x_ground][y_ground] = std::min(aux_ground[x_ground][y_ground], z);
        }
    }

    // Hack to fill points near and below the car (5x5 area centered on the lidar)
    for (int i = 0; i < ground_cells; ++i){
        for (int j = 0; j < ground_cells; ++j){
            float z = aux_ground[i][j];
            if(fabs(z)>0.2 && fabs((i *ground_cell_span -grid_cells/2.)* bv_cell_size)<5. &&
                    fabs((j *ground_cell_span -grid_cells/2.)* bv_cell_size)<5.){
                aux_ground[i][j] = 0;
            }
        }
    }
    // Copy the aux grid to mat
    for (int i = 0; i < ground_cells; ++i){
        float* row_ptr = small_ground->ptr<float>(i);
        for (int j = 0; j < ground_cells; ++j){
            *row_ptr++ = (float) aux_ground[i][j];
        }
    }

    // Apply median filter
    medianBlur(*small_ground, *median_ground, 3);

    // Fill the bird view mat
    for (int i = 0; i < grid_cells; ++i){
        float* row_ptr = bird_ground->ptr<float>(i);
        for (int j = 0; j < grid_cells; ++j){
            int x_ground = (double)i/ground_cell_span -.5f; // TODO Review grid access
            int y_ground = (double)j/ground_cell_span -.5f;
            *row_ptr++ = median_ground->at<float>(x_ground, y_ground);
        }
    }

    for (int i = 0; i < ground_cells; ++i){
        delete [] aux_ground[i];
    }
    delete [] aux_ground;

    return bird_ground;
}
std::shared_ptr<cv::Mat> CloudFilter::birdView(double cell_size, double max_height, std::map<string, int> channels, double grid_dim, bool sample_points){
    // TODO: Add an interpolation to fill the unknown cells

    int grid_cells = grid_dim / cell_size;


    std::map<string,int>::iterator it;

    int max_height_slices = 1; // Always computed
    int min_height_slices = 1; // Always computed
    int density_slices = 0;
    int intensity_slices = 0;
    int stdev_height_slices = 0;

    it = channels.find(DENSITY_CHANNEL);
    if(it!=channels.end()){
        density_slices = it->second;
    }
    it = channels.find(INTENSITY_CHANNEL);
    if(it!=channels.end()){
        intensity_slices = it->second;
    }
    it = channels.find(HEIGHT_DEV_CHANNEL);
    if(it!=channels.end()){
        stdev_height_slices = it->second;
    }

    int num_channels = density_slices + intensity_slices + max_height_slices + min_height_slices + stdev_height_slices;
    int num_slices = 3;

    cout << "Num channels " << num_channels << endl;

    // The channels are: intensity, min_height, max_height, density per slice.
    std::shared_ptr<cv::Mat> bird_view(new cv::Mat(grid_cells, grid_cells, CV_8UC(num_channels)));

    std::random_device rd;
    std::mt19937 mt(rd());
    std::uniform_real_distribution<double> dist(0.0, 1.0);


    int*** density = new int**[grid_cells];
    float*** height = new float**[grid_cells];
    std::vector<float>*** i_points;
    std::vector<float>*** z_points;

    for (int i = 0; i < grid_cells; ++i)
    {
        height[i] = new float*[grid_cells];
        density[i] = new int*[grid_cells];
        i_points[i] = new vector<float>*[grid_cells];
        z_points[i] = new vector<float>*[grid_cells];

        for (int j = 0; j < grid_cells; ++j){
            height[i][j] = new float[2];
            density[i][j] = new int[density_slices];
            i_points[i][j] = new vector<float>[stdev_height_slices];
            z_points[i][j] = new vector<float>[stdev_height_slices];
        }
    }

    // Init the grids
    for (int i = 0; i < grid_cells; ++i)
    {
        for (int j = 0; j < grid_cells; ++j)
        {
            for (int k = 0; k < density_slices; ++k){
                density[i][j][k] = 0;
            }

            height[i][j][0] = 9999.9;
            height[i][j][1] = -9999.9;
        }
    }

    // Fill the grids
    for (const auto& point : cloud_ptr_->points){
        float z = point.z + base_velo_transform_.getOrigin().z();
        if (z < max_height){
            int x = grid_cells/2 - point.x/cell_size;
            int y = grid_cells/2 - point.y/cell_size;

            if (x >= 0 && x < grid_cells && y >= 0 && y < grid_cells){

                // Update cell min height
                height[x][y][0] = std::min(height[x][y][0], z);
                // Update cell max height
                height[x][y][1] = std::max(height[x][y][1], z);
                // Increment the number of points in the cell
                for (int k = 0; k < density_slices; k++){
                    if (z > k*max_height/density_slices and z < (k+1)*max_height/density_slices){
                        density[x][y][k]++;
                    }
                }
                for (int k = 0; k < intensity_slices; k++){
                    if (z > k*max_height/intensity_slices and z < (k+1)*max_height/intensity_slices){
                        // Update the cell intensity sum to later compute the mean
                        i_points[x][y][k].push_back(point.intensity);
                    }
                }
                for (int k = 0; k < stdev_height_slices; k++){
                    if (z > k*max_height/stdev_height_slices and z < (k+1)*max_height/stdev_height_slices){
                        // Update the cell height sum to later compute the mean and stdev
                        z_points[x][y][k].push_back(z);
                    }
                }
            }
        }
    }

    // Fill the bird view mat
    uchar* mat_ptr = bird_view->data;
    for (int i = 0; i < grid_cells; ++i){
        for (int j = 0; j < grid_cells; ++j){
            // Set unknown heights (still in -9999.9) to 0
            height[i][j][0] = std::max(height[i][j][0], (float)0.0);
            height[i][j][1] = std::max(height[i][j][1], (float)0.0);
            // Normalize the height to 0-255
            *mat_ptr++ = (uchar)(255 * height[i][j][0] / max_height);
            *mat_ptr++ = (uchar)(255 * height[i][j][1] / max_height);

            for (int k = 0; k < density_slices; ++k){
                // Limit the density to 255 (we should never have that many points in the same cell...)
                int norm_factor = max_points_map_[i][j][k];
                int points_cell_pixel = (float)density[i][j][k] / norm_factor * 255;
                *mat_ptr++ = (uchar)std::min(points_cell_pixel, 255);
            }
            for (int k = 0; k < intensity_slices; ++k){
                // Compute the intensity mean for that cell and slice given the point count (density) and normalize it to 0-255
                float cell_intensity_mean = 0.f;
                if(i_points[i][j][k].size()){
                    float norm_factor = 255/max_expected_intensity_;
                    cell_intensity_mean = std::accumulate(i_points[i][j][k].begin(), i_points[i][j][k].end(), 0.0) / i_points[i][j][k].size();
                    cell_intensity_mean = cell_intensity_mean * norm_factor;
                }
                *mat_ptr++ = (uchar) std::min(cell_intensity_mean, 255.f);
            }
            for (int k = 0; k < stdev_height_slices; ++k){
                // Compute the stdev for that cell and slice given the point count and normalize it to 0-255
                float stdev = 0.f;
                if(z_points[i][j][k].size()>0){
                    float norm_factor = (float) max_height/stdev_height_slices * 255;
                    float cell_height_mean = std::accumulate( z_points[i][j][k].begin(), z_points[i][j][k].end(), 0.0) / z_points[i][j][k].size();
                    std::vector<float> diff(z_points[i][j][k].size());
                    std::transform(z_points[i][j][k].begin(), z_points[i][j][k].end(), diff.begin(),
                                   std::bind2nd(std::minus<float>(), cell_height_mean));
                    float sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
                    stdev = std::sqrt(sq_sum / z_points[i][j][k].size());
                    stdev = stdev * norm_factor;
                }
                *mat_ptr++ = (uchar) std::min(stdev, 255.f);
            }
        }
    }

    for (int i = 0; i < grid_cells; ++i){
        for (int j = 0; j < grid_cells; ++j){
            delete [] density[i][j];
            delete [] height[i][j];
            delete [] i_points[i][j];
            delete [] z_points[i][j];
        }
    }

    for (int i = 0; i < grid_cells; ++i){
        delete [] density[i];
        delete [] height[i];
        delete [] i_points[i];
        delete [] z_points[i];
    }

    delete [] density;
    delete [] height;
    delete [] i_points;
    delete [] z_points;

    return bird_view;
}

/* Remove all points above the given intensity threshold */
void CloudFilter::filterIntensities(double intensity_threshold){
    // Resize the cloud to make it non-organized and work faster
    cloud_ptr_->width = cloud_ptr_->width * cloud_ptr_->height;
    cloud_ptr_->height = 1;

    // Avoid pcl filters if possible because they copy the cloud inside.
    // Points can be removed using std algorithms (erase + remove_if)
    cloud_ptr_->points.erase(std::remove_if(cloud_ptr_->points.begin(),
                                            cloud_ptr_->points.end(),
                                            [&intensity_threshold](const pcl::PointXYZI& p){return p.intensity > intensity_threshold;}),
                             cloud_ptr_->points.end()
                             );
    cloud_ptr_->width = cloud_ptr_->points.size();
}


/* Wait for the transform lidar -> camera and update velo_cam_transform_ */
void CloudFilter::initTF(std::string lidar_frame, std::string camera_frame){
    if (!tf_) tf_ = new tf::TransformListener;
    bool tf_error = true;
    while(tf_error)
    {
        try
        {
            tf_->waitForTransform(lidar_frame, camera_frame, ros::Time(0), ros::Duration(5));
            tf_->lookupTransform(lidar_frame, camera_frame, ros::Time(0), velo_cam_transform_);
            tf_->waitForTransform("base_footprint", lidar_frame, ros::Time(0), ros::Duration(5));
            tf_->lookupTransform("base_footprint", lidar_frame, ros::Time(0), base_velo_transform_);
            tf_error = false;
        }
        catch (tf::TransformException ex)
        {
            ROS_WARN("%s",ex.what());
            //throw (ex);
        }
    }
    std::cout << "New transform: " << velo_cam_transform_.getOrigin().x() << ", " << velo_cam_transform_.getOrigin().y() << std::endl;
}

void CloudFilter::setVeloToCamTransform(tf::StampedTransform velo_cam_transform){
    velo_cam_transform_ = velo_cam_transform;
}

void CloudFilter::setVeloToBaseTransform(tf::StampedTransform base_velo_transform){
    base_velo_transform_ = base_velo_transform;
}


void CloudFilter::initMaxPointsMap(int grid_dim, float cell_size, float z_min, float z_max, int num_slices, int planes,
                                   float low_angle, float h_res, float v_res){
    std::stringstream map_path;
    map_path << ros::package::getPath("lidar_bev") << "/maps/";
    std::stringstream velo_h;
    velo_h << std::fixed;
    velo_h << std::setprecision(2);
    velo_h << base_velo_transform_.getOrigin().z();
    std::stringstream python_cmd;
    python_cmd << ros::package::getPath("lidar_bev") << "/scripts/max_points_map.py";
    python_cmd << " --maps " << map_path.str();
    python_cmd << " --map_size " << grid_dim;
    python_cmd << " --cell_size " << cell_size;
    python_cmd << " --min_height " << z_min;
    python_cmd << " --max_height " << z_max;
    python_cmd << " --num_slices " << num_slices;
    python_cmd << " --num_planes " << planes;
    python_cmd << " --velo_minangle " <<  low_angle;
    python_cmd << " --velo_hres " << h_res;
    python_cmd << " --velo_vres " << v_res;
    python_cmd << " --velo_height " << velo_h.str();
    std::cout << "Required max_points map not found, creating map..." << std::endl;
    std::cout << python_cmd.str() << std::endl;
    int python_result = system(python_cmd.str().c_str());

    // Resize the matrix
    int grid_cells = grid_dim / cell_size;
    max_points_map_.resize(grid_cells);
    for (int i = 0; i < max_points_map_.size(); ++i)
    {
      max_points_map_[i].resize(grid_cells);
      for (int j = 0; j < max_points_map_[i].size(); ++j){
        max_points_map_[i][j].resize(num_slices);
      }
    }

    for (int n = 0; n < num_slices; ++n){
        std::stringstream file_name;
        file_name << std::fixed;
        file_name << std::setprecision(2);
        file_name << map_path.str() << grid_dim << "_";
        file_name << cell_size << "_";
        file_name << planes << "_";
        file_name << velo_h.str();
        file_name << "_slice" << n << "_map.txt";

        std::ifstream f(file_name.str());
        if (!f.good()) {
            std::cout << "[ERROR] Cannot read "<< file_name.str() << ". Exiting..." << std::endl;
            exit(-1);
        }

        // Load map from file
        std::string line;
        int i = 0, j = 0;

        while (std::getline(f, line))
        {
          float value;
          int j = 0;
          std::stringstream ss(line);
          while (ss >> value)
          {
            max_points_map_[i][j][n] = value;
            ++j;
          }
          ++i;
        }
        f.close();
    }
}
