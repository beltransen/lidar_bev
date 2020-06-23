*lidar_bev* is a [ROS](https://www.ros.org/) package that computes the Bird's Eye View of a LiDAR pointcloud. It follows the encoding defined in [BirdNet](https://arxiv.org/abs/1805.01195). This representation is also used by the newer [BirdNet+](https://arxiv.org/abs/2003.04188).

This package was developed at the [Intelligent Systems Laboratory](http://www.uc3m.es/islab), Universidad Carlos III de Madrid, by Jorge Beltrán, Carlos Guindel, Francisco M. Moreno, Irene Cortés, and Daniel Cruzado.

### Where to start?
- If you want to generate BEV images for training and validation of models on the KITTI Object Benchmark, please run:
```
roslaunch lidar_bev offline_bev.launch
```

- For online operation, a ROS node is provided. Launch files for different LiDAR devices are located in `launch/`.
- The files in `scripts/` are mainly intended to perform the post-processing/refinement stage of the original BirdNet in order to obtain rotated boxes from the Faster R-CNN axis-aligned detections. This has been deprecated by BirdNet+.

### Run configuration

#### Bird's Eye View parameters
There are many parameters to configure the BEV. Default values lead to results published in BirdNet and BirdNet+. These are: *camera_fov*, *planes*, *h_res*, *v_res*, *low_opening*, *cell_size*, *grid_dim*, *grid_min_x*, *grid_max_x*, *grid_min_y*, *grid_max_y*, *max_height*, *min_height*, *num_slices*, *height_threshold*, *cell_size_height_map*, *grid_dim_height_map*, *get_ground*, *remove_floor*.

#### Offline mode
* *kitti_dir*: Path to `kitti/object` folder.
* *split_dir*: KITTI split set. Can be *training* or *testing*.
* *saving_path*: folder name (or full path) to save resulting images. If no full path is provided, the folder will be created in `$HOME/.ros/`

#### Online mode (ROS node)
* *lidar_tf_frame*: name of the LiDAR frame
* *camera_tf_frame*: name of the camera frame

##### Inputs (Subscribed Topics)
* *cloud_topic* ([sensor_msgs::PointCloud2](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointCloud2.html)): LiDAR pointcloud

##### Outputs (Published Topics)
* *bird_view* ([sensor_msgs/Image](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Image.html)): BEV image
* *bird_ground* ([sensor_msgs/Image](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Image.html)): Image representing the ground estimation
* *ground_cloud* ([sensor_msgs::PointCloud2](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointCloud2.html)): Pointcloud to visualize the ground estimation in Rviz.

### Dependencies

This repository depends on [ROS](http://ros.org/). ROS *Kinetic* and newer versions should work. Additionally, an auxiliary script is used to compute the density channel of the BEV (*max_points_map.py*). To setup the dependencies, please run:
```
pip install progress numpy
```
##### ROS Dependencies
* [cv_bridge](http://wiki.ros.org/cv_bridge)
* [image_transport](http://wiki.ros.org/image_transport)
* [pcl_ros](http://wiki.ros.org/pcl_ros)
* [sensor_msgs](http://wiki.ros.org/sensor_msgs)
* [tf](http://wiki.ros.org/tf)

### Citation

If you find this module useful, consider citing one of our works:

##### The original BirdNet, where this BEV encoding was introduced.
```
@inproceedings{Beltran2018,
    title = {{BirdNet: a 3D Object Detection Framework from LiDAR information}},
    author = {Beltrán, Jorge and Guindel, Carlos and Moreno, Francisco Miguel and Cruzado, Daniel and García, Fernando and de la Escalera, Arturo},
    booktitle = {Proc. IEEE International Conference on Intelligent Transportation Systems (ITSC)},
    pages = {3517--3523},
    publisher = {IEEE},
    year = {2018}
}
```

##### The next generation, BirdNet+.
```
@misc{Barrera2020,
    title = {{BirdNet+: End-to-End 3D Object Detection in LiDAR Bird's Eye View}},
    author = {Barrera, Alejandro and Guindel, Carlos and Beltrán, Jorge and García, Fernando},
    booktitle = {arXiv:2003.04188 [cs.CV]},
    url = {http://arxiv.org/abs/2003.04188},
    year = {2020}
}
```



