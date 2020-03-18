#!/usr/bin/env python

# draw_bev_detections.py: Node for visualization of the resulting detections

import sys
import rospy
import datetime
import math
import numpy as np
from numpy.linalg import inv
import cv2
import tf
import message_filters
from sensor_msgs.msg import Image, CameraInfo
from perception_msgs.msg import ObstacleList
from cv_bridge import CvBridge, CvBridgeError
from perception_msgs._classes import CLASSES, CLASS_COLOR, CLASS_THRESHOLDS

bvres = 0.05
only_front = False
is_kitti_gt = False

listener = tf.TransformListener()

def _project_obstacle_to_cam(obs, P2_full):

    yaw = obs.yaw

    x = obs.location.x
    y = obs.location.y
    z = obs.location.z
    if  is_kitti_gt:
        z += obs.height/2

    l = obs.length
    h = obs.height
    w = obs.width

    centroid = np.array([x, y])

    corners = np.array([
        [x - l / 2., y + w / 2.],
        [x + l / 2., y + w / 2.],
        [x + l / 2., y - w / 2.],
        [x - l / 2., y - w / 2.]
    ])

    # Compute rotation matrix
    c, s = np.cos(yaw), np.sin(yaw)
    R = np.array([[c, -s], [s, c]])

    # Rotate all corners at once by yaw
    rot_corners = np.dot(corners - centroid, R.T) + centroid

    x1 = rot_corners[0,0]
    x2 = rot_corners[1,0]
    x3 = rot_corners[2,0]
    x4 = rot_corners[3,0]

    y1 = rot_corners[0,1]
    y2 = rot_corners[1,1]
    y3 = rot_corners[2,1]
    y4 = rot_corners[3,1]

    # Project the 8 vertices of the prism
    vertices = []
    vertices.append(np.array([x1, y1, z+h/2, 1]))
    vertices.append(np.array([x2, y2, z+h/2, 1]))
    vertices.append(np.array([x3, y3, z+h/2, 1]))
    vertices.append(np.array([x4, y4, z+h/2, 1]))

    vertices.append(np.array([x1, y1, z-h/2, 1]))
    vertices.append(np.array([x2, y2, z-h/2, 1]))
    vertices.append(np.array([x3, y3, z-h/2, 1]))
    vertices.append(np.array([x4, y4, z-h/2, 1]))

    image_pts = np.empty((0, 3), dtype=np.float64)

    for point in vertices:
        image_point = np.dot(P2_full, point)
        image_point /= image_point[2]  # TODO: Scale?
        # print 'velo {} image {}'.format(point, image_point)
        # cv2.circle(left_image, (int(image_point[0]), int(image_point[1])), 6, (0, 0, point[0]/20.0*255), -1)
        image_pts = np.vstack((image_pts, image_point))

    # Extreme object points in the image
    image_u1 = np.min(image_pts[:, 0])
    image_v1 = np.min(image_pts[:, 1])

    image_u2 = np.max(image_pts[:, 0])
    image_v2 = np.max(image_pts[:, 1])

    return (int(image_u1), int(image_v1)), (int(image_u2), int(image_v2))

def callback(obstaclelist, bird_view, image_color=None, cinfo_msg=None):
    bridge = CvBridge()
    bv_image = []
    if image_color is not None:
        rgb_image = []
    try:
        bv_image = bridge.imgmsg_to_cv2(bird_view, "bgr8")
        if image_color is not None:
            rgb_image = bridge.imgmsg_to_cv2(image_color, "bgr8")
    except CvBridgeError as e:
        print(e)
        sys.exit(-1)
    if image_color is not None:
        rgb_draw = rgb_image.copy()
    draw = bv_image.copy() # TODO Remove or paint received detections

    trans = []
    rot = []
    try:
        # print 'Looking for TF from {} to {}'.format('velodyne', 'stereo_camera')
        # listener.waitForTransform(image_color.header.frame_id, obstaclelist.header.frame_id, rospy.Time(0), rospy.Duration(20.0))
        # (trans, rot) = listener.lookupTransform(image_color.header.frame_id, obstaclelist.header.frame_id, rospy.Time(0))
        listener.waitForTransform(camera_tf_frame, lidar_tf_frame, rospy.Time(0), rospy.Duration(5.0))
        (trans, rot) = listener.lookupTransform(camera_tf_frame, lidar_tf_frame, rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException) as e:
        print e
        pass

    # numpy arrays to 4x4 transform matrix
    trans_mat = tf.transformations.translation_matrix(trans)
    rot_mat = tf.transformations.quaternion_matrix(rot)
    # create a 4x4 matrix
    v2cam = np.dot(trans_mat, rot_mat)

    if image_color is not None:
        # Camera info to numpy
        P2 = np.array(cinfo_msg.P).reshape(-1, 4)
        P2_full = np.dot(P2, v2cam)

    print 'trans {} '.format(trans_mat)

    for obj in obstaclelist.obstacles:
        # Filter detections with a very low score
        if obj.score < 0.9: # TODO Remove?
            continue

        if is_kitti_gt: # KITTI Grount Truth to lidar coordinates
            velo_ctr = np.dot(inv(v2cam), np.array([obj.location.x, obj.location.y, obj.location.z, 1]))
            obj.location.x = velo_ctr[0]
            obj.location.y = velo_ctr[1]
            obj.location.z = velo_ctr[2] + obj.height/2
            obj.yaw = -obj.yaw - np.pi/2

        centroid = [obj.location.x, obj.location.y]
        length = obj.length # TODO Review. Changed on 1/05/2018 Formerly was obj.width
        width = obj.width # TODO Review. Changed on 1/05/2018 Formerly was obj.length
        yaw = obj.yaw # TODO Review. Changed on 1/05/2018

        # Compute the four vertices coordinates
        corners = np.array([[centroid[0] - length / 2., centroid[1] + width / 2.],
                            [centroid[0] + length / 2., centroid[1] + width / 2.],
                            [centroid[0] + length / 2., centroid[1] - width / 2.],
                            [centroid[0] - length / 2., centroid[1] - width / 2.]])


        # Compute rotation matrix
        c, s = np.cos(yaw), np.sin(yaw)
        R = np.array([[c, -s], [s, c]])

        # Rotate all corners at once by yaw
        rotated_corners = np.dot(corners - centroid, R.T) + centroid

        # For every vertex
        xs = draw.shape[1] / 2 - rotated_corners[:, 1] / bvres
        ys = (draw.shape[0] if not only_front else draw.shape[0]/2) - rotated_corners[:, 0] / bvres

        pt1 = np.array([xs[0], ys[0]])
        pt2 = np.array([xs[1], ys[1]])
        pt3 = np.array([xs[2], ys[2]])
        pt4 = np.array([xs[3], ys[3]])

        ctr = np.array([pt1, pt2, pt3, pt4]).reshape((-1, 1, 2)).astype(np.int32)

        try:
            color = CLASS_COLOR[CLASSES.index(obj.kind_name)] if obj.kind is not 99999 else [1.0, 0.0, 0.0]
            cv2.drawContours(draw, [ctr], -1, (color[2], color[1], color[0]), 2)

            if image_color is not None and obj.location.x > 0:
                rgb_p1, rgb_p2 = _project_obstacle_to_cam(obj, P2_full)
                cv2.rectangle(rgb_draw, rgb_p1, rgb_p2, (color[2], color[1], color[0]), 2)
        except ValueError:
            print 'WARNING. Unknown class \'{}\'. Object ignored'.format(obj.kind_name)

    # cv2.imshow('BEV', bv_image)
    # cv2.imshow('BEV detections', draw)
    # cv2.imshow('Image detections', rgb_draw)
    cv2.imwrite('/home/beltransen/birdview/paper_results/bv_{}.png'.format(bird_view.header.seq), bv_image , [cv2.IMWRITE_PNG_COMPRESSION, 9])
    cv2.imwrite('/home/beltransen/birdview/paper_results/bv_det_{}.png'.format(bird_view.header.seq), draw , [cv2.IMWRITE_PNG_COMPRESSION, 9])

    if image_color is not None:
        cv2.imwrite('/home/beltransen/birdview/paper_results/rgb_{}.png'.format(bird_view.header.seq), rgb_draw , [cv2.IMWRITE_PNG_COMPRESSION, 9])
    # cv2.imwrite('bv_det_{}.png'.format(bird_view.header.seq), draw , [cv2.IMWRITE_PNG_COMPRESSION, 9])

    white = draw.copy()
    white[np.where((white == [0,0,0] ).all(axis = 2))] = [255, 255, 255]
    # cv2.imshow('BEV detections white', white)
    cv2.waitKey(1)


def main(args):
    global bvres, lidar_tf_frame, camera_tf_frame, max_height, only_front, is_kitti_gt
    # Initializes and cleanup ros node
    rospy.init_node('draw_bev_detections', anonymous=True)
    rospy.loginfo("[draw_bev_detections] Ready")
    bvres = rospy.get_param("~cell_size", 0.05)
    max_height = rospy.get_param("~max_height", 3.0)
    obstacles_topic = rospy.get_param("~obstacles_list", '/obstacle_list')
    with_rgb_image = rospy.get_param("~with_rgb_image", False)
    image_color_topic = rospy.get_param("~image_color", '/stereo_camera/left/image_rect_color')
    cinfo_topic = rospy.get_param("~cinfo_topic", '/stereo_camera/left/camera_info')
    birdview_topic = rospy.get_param("~bird_view", '/bird_view')
    lidar_tf_frame = rospy.get_param("~lidar_tf_frame", 'velodyne')
    camera_tf_frame = rospy.get_param("~camera_tf_frame", 'stereo_camera')
    only_front = rospy.get_param("~only_front", False)
    is_kitti_gt = rospy.get_param("~is_kitti_gt", False)

    obstacle_sub = message_filters.Subscriber(obstacles_topic, ObstacleList)
    birdview_sub = message_filters.Subscriber(birdview_topic, Image)

    if with_rgb_image:
        rgb_sub = message_filters.Subscriber(image_color_topic, Image)
        info_sub = message_filters.Subscriber(cinfo_topic, CameraInfo)
        ts = message_filters.TimeSynchronizer([obstacle_sub, birdview_sub, rgb_sub, info_sub ], 10)
    else:
        ts = message_filters.TimeSynchronizer([obstacle_sub, birdview_sub], 10)
    ts.registerCallback(callback)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)