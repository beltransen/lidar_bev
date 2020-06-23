# birdview_detection_refiner.py: Class for bounding box refinement. It obtains rotated boxes from BirdNet 1 axis-aligned outputs.

import math
import numpy as np
import cv2
from shapely.geometry import Polygon

class BirdviewDetectionRefiner:

    def __init__(self, bv_image, bv_ground, bvres, lidar_h, only_front=False):

        self.only_front = only_front
        self.bv_image = bv_image
        self.bv_ground = bv_ground
        self.bvres = bvres
        self.lidar_h = lidar_h
        (self.bvheight, self.bvwidth, self.channels) = self.bv_image.shape

    def refine_detection(self, obj):
        # Bird view: Vertices
        bv_x1 = obj.bbox.x_offset
        bv_y1 = obj.bbox.y_offset
        bv_x2 = bv_x1 + obj.bbox.width
        bv_y2 = bv_y1 + obj.bbox.height
        bv_yaw = obj.alpha # 3.1
        bv_points = [bv_y1, bv_y2, bv_x1, bv_x2]

        bv_ctr_x = (bv_x1 + bv_x2) / 2.0
        bv_ctr_y = (bv_y1 + bv_y2) / 2.0

        bv_width = bv_x2 - bv_x1 #97
        bv_height = bv_y2 - bv_y1 #66

        # Compute object and ground height
        object_height, ground_height = self._compute_height(bv_points, 1.0, 1.4, 1.9, 2.3)

        effective_bvheight = self.bvheight / 2 if not self.only_front else self.bvheight

        # Compute object centroid
        velo_ctr_x = -(bv_ctr_y - effective_bvheight) * self.bvres
        velo_ctr_y = -(bv_ctr_x - self.bvwidth / 2) * self.bvres
        velo_ctr_z = object_height / 2 + ground_height - self.lidar_h

        object_width = self.bvres * bv_width
        object_length = self.bvres * bv_height

        # Fill object with raw values from detection
        obj.location.x = velo_ctr_x
        obj.location.y = velo_ctr_y
        obj.location.z = velo_ctr_z
        obj.height = object_height
        obj.width = object_width
        obj.length = object_length

        # Refine obstacle
        if obj.kind_name == 'Car':
            object_width = 1.8 # Fixed width for cars.
            ref_bv_width = int(1.8 / self.bvres)
            ref_bv_height = self._compute_bbox_height(bv_height, bv_width, object_width, bv_yaw)

            object_length = self.bvres * ref_bv_height

            if object_height<=0 or object_width<=0 or object_length<=0:
                raise Exception('Non-positive dimensions')

            # Update object 3D location and size
            obj.height = object_height
            obj.width = object_width
            obj.length = object_length

        elif obj.kind_name == 'Pedestrian':
            object_width = 0.6 # Fixed width for bikes.
            ref_bv_height = self._compute_bbox_height(bv_height, bv_width, object_width, bv_yaw)

            object_length = self.bvres * ref_bv_height

            # Update object 3D location and size
            obj.width = object_width
            obj.length = object_length

        elif obj.kind_name == 'Cyclist':
            object_width = 0.6 # Fixed width for bikes.
            ref_bv_height = self._compute_bbox_height(bv_height, bv_width, object_width, bv_yaw)

            object_length = self.bvres * ref_bv_height

            # Update object 3D location and size
            obj.width = object_width
            obj.length = object_length

        obj.yaw = -obj.alpha - math.pi/2
        obj.alpha = -obj.alpha - math.pi/2

    def nofloor_birdview(self):
        no_floor_img = np.copy(self.bv_image)
        lower = np.array([50, 0, 0])
        upper = np.array([255, 255, 255])
        mask = cv2.inRange(no_floor_img, lower, upper)
        # Bitwise-AND mask and original image
        no_floor_img = cv2.bitwise_and(no_floor_img, no_floor_img, mask=mask)
        return no_floor_img

    # Refinement functions
    def _get_rotated_box(self, centroid, length, width, yaw):
        yaw = -yaw
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

        pt1 = np.array(rotated_corners[0])
        pt2 = np.array(rotated_corners[1])
        pt3 = np.array(rotated_corners[2])
        pt4 = np.array(rotated_corners[3])

        ctr = np.array([pt1, pt2, pt3, pt4]).reshape((-1, 1, 2)).astype(np.int32)
        return ctr

    def _compute_bbox_height(self, height, width, ref_width, yaw):
        yaw -= math.pi / 2
        fixed_width = int(ref_width / self.bvres)

        cosy = np.cos(yaw)
        siny = np.sin(yaw)
        cosyrect = np.cos(yaw+np.pi/2)
        sinyrect = np.sin(yaw+np.pi/2)

        # Compute the two possible car lengths
        l0 = abs((height - abs(cosyrect*fixed_width))/cosy) if cosy!=0 else 0 # TODO Check if 0 is right value
        l1 = abs((width - abs(sinyrect*fixed_width))/siny) if siny!=0 else 0 # TODO Check if 0 is right value

        # Compute vertexes for the aligned bbox
        aligned_v0 = (-height/2, -width/2)
        aligned_v1 = (-height/2, +width/2)
        aligned_v2 = (+height/2, +width/2)
        aligned_v3 = (+height/2, -width/2)

        # Compute rotated vertexes for the bbox 0
        alignedl0_corners = np.array([[-l0/2, -fixed_width/2],
                            [-l0/2, +fixed_width/2],
                            [+l0/2, +fixed_width/2],
                            [+l0/2, -fixed_width/2]])

        # Compute rotated vertexes for the bbox 1
        alignedl1_corners = np.array([[-l1/2, -fixed_width/2],
                                      [-l1/2, +fixed_width/2],
                                      [+l1/2, +fixed_width/2],
                                      [+l1/2, -fixed_width/2]])

        # Compute rotation matrix
        c, s = np.cos(yaw), np.sin(yaw)
        R = np.array([[c, -s], [s, c]])

        # Rotate all corners at once by yaw
        rot0_corners = np.dot(alignedl0_corners, R.T)
        rot1_corners = np.dot(alignedl1_corners, R.T)

        # Compute IoU between aligned bbox and the two refined proposals
        aligned_poly = Polygon([aligned_v0, aligned_v1, aligned_v2, aligned_v3])
        poly0 = Polygon([tuple(rot0_corners[0]), tuple(rot0_corners[1]), tuple(rot0_corners[2]), tuple(rot0_corners[3])])
        poly1= Polygon([tuple(rot1_corners[0]), tuple(rot1_corners[1]), tuple(rot1_corners[2]), tuple(rot1_corners[3])])

        intersection_p0 = poly0.intersection(aligned_poly).area
        intersection_p1 = poly1.intersection(aligned_poly).area

        iou_p0 = intersection_p0/(aligned_poly.area + poly0.area - intersection_p0)
        iou_p1 = intersection_p1/(aligned_poly.area + poly1.area - intersection_p1)

        # Return the car length with higher IoU with the aligned bbox
        return l0 if iou_p0>iou_p1 else l1


    def _get_true_h(self, H, w, yaw):
        yaw -= math.pi / 2

        print w * math.sin(yaw)
        print H * math.cos(yaw)
        # TODO: Fix this workarounds
        if abs(math.cos(yaw)) < 0.01:
            return H
        if H - abs(w * math.sin(yaw)) < 0:
            return H
        return int((H - abs(w * math.sin(yaw))) / abs(math.cos(yaw)))

    def _compute_height(self, points, hard_low, soft_low, soft_high, hard_high):
        # Determine object height from the bv image
        roi = self.bv_image[int(points[0]):int(points[1]), int(points[2]):int(points[3])]
        minVal, maxVal, minLoc, maxLoc = cv2.minMaxLoc(roi[:, :, 0])

        roi_height = self.bv_ground[int(points[0]):int(points[1]), int(points[2]):int(points[3])]
        minValH, maxValH, minLocH, maxLocH = cv2.minMaxLoc(roi_height)
        ground_height = minValH

        # Remove nonsense proposals with no points
        if maxVal == 0:
            return -1, -1

        object_height = ((maxVal-ground_height) / 255) * 3.0  # from the ground

        if hard_low >= 0:
            return min(max(object_height, soft_low), soft_high), ground_height# if \
        else:
            return object_height, ground_height
