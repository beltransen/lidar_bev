*lidar_bev* is a [ROS](https://www.ros.org/) package that computes the Bird's Eye View of a LiDAR pointcloud. It follows the encoding defined in [BirdNet](https://arxiv.org/abs/1805.01195). This representation is also used by the newer [BirdNet+](https://arxiv.org/abs/2003.04188).

This package was developed at the [Intelligent Systems Laboratory](http://www.uc3m.es/islab), Universidad Carlos III de Madrid, by Jorge Beltrán, Carlos Guindel, Francisco M. Moreno, and Daniel Cruzado.

### Where to start?
- If you want to generate BEV images for training and validation of a detection algorithm (e.g., from the KITTI dataset), take a look at `launch/offline_birdview.launch`. You will need to create a rosbag file including the LiDAR point cloud (by default, the topic is `velodyne_points`, and scans from a lidar HDL-64E are expected).
- It is also possible to perform the conversion online for inference using the other launch files located in `launch/`, one per lidar device.
- The files in `scripts/` are mainly intended to perform the post-processing/refinement stage of the original BirdNet in order to obtain rotated boxes from the Faster R-CNN axis-aligned detections. This has been deprecated by BirdNet+.


### Dependencies

To compute the density channel of the BEV, an auxiliary script is used: *max_points_map.py*. This script requires the package [progress](https://pypi.org/project/progress/) to be installed. To setup the dependency, please run:
```
pip install progress
```

### Citation

If you find this module useful, consider citing one of our works:

#### The original BirdNet, where this BEV encoding was introduced.
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

#### The next generation, BirdNet+.
```
@misc{Barrera2020,
    title = {{BirdNet+: End-to-End 3D Object Detection in LiDAR Bird's Eye View}},
    author = {Barrera, Alejandro and Guindel, Carlos and Beltrán, Jorge and García, Fernando},
    booktitle = {arXiv:2003.04188 [cs.CV]},
    url = {http://arxiv.org/abs/2003.04188},
    year = {2020}
}
```



