import OGM as ogm
import cv2
import numpy as np
import matplotlib.pyplot as plt

import open3d as o3d


class ShapeDetector:
	def __init__(self):
		pass
	def detect(self, c):
		# initialize the shape name and approximate the contour
		shape = "unidentified"
		peri = cv2.arcLength(c, True)
		approx = cv2.approxPolyDP(c, 0.04 * peri, True)
		# if the shape is a triangle, it will have 3 vertices
		if len(approx) == 3:
			shape = "triangle"
		# if the shape has 4 vertices, it is either a square or
		# a rectangle
		elif len(approx) == 4:
			# compute the bounding box of the contour and use the
			# bounding box to compute the aspect ratio
			(x, y, w, h) = cv2.boundingRect(approx)
			ar = w / float(h)
			# a square will have an aspect ratio that is approximately
			# equal to one, otherwise, the shape is a rectangle
			shape = "square" if ar >= 0.95 and ar <= 1.05 else "rectangle"
		# if the shape is a pentagon, it will have 5 vertices
		elif len(approx) == 5:
			shape = "pentagon"
		# otherwise, we assume the shape is a circle
		else:
			shape = "circle"
		# return the name of the shape
		return shape


def main():
    #points, keyframe, mean = ogm.save_data(100, 1.5, './../maps/xyz_rotated.txt', './../maps/KF_trajectory.txt')


# FILTER
    #points, keyframe, mean = ogm.save_data(100, 1.5, './../maps/xyz_rotated.txt', './../maps/KF_trajectory.txt')
    # pcd = o3d.geometry.PointCloud()  
    # a = np.zeros(shape=(len(points), 3))     
    # # dado conferido  
    # for i in range(len(points)):
    #     a[i, 0] = points[i][1]    
    #     a[i, 1] = points[i][2]
    #     a[i, 2] = points[i][3]         
    # pcd.points = o3d.utility.Vector3dVector(a)
    # print(pcd)
    # img = ogm.draw_2d_point_cloud2(a, keyframe)
    # cl, ind = pcd.remove_statistical_outlier(nb_neighbors=2, std_ratio=2)
    # print(cl)
    # xyz_load = np.asarray(cl.points)
    # b = np.zeros(shape=(len(xyz_load), 3)) 
    # for i in range(len(xyz_load)):
    #     b[i, 0] = xyz_load[i][0]    
    #     b[i, 1] = xyz_load[i][1]
    #     b[i, 2] = xyz_load[i][2]    
    
    # c = ogm.filter_ground_points(b, mean/4)
    # d = ogm.filter_roof_points(c, mean)
    
    # img2 = ogm.draw_2d_point_cloud2(b, keyframe)



    # cv2.imshow('original', img)
    # cv2.imshow('saporra', img2)
    # cv2.waitKey(0)

    
    # FIRST APPROACH
    # data treatment
    points, keyframe, mean = ogm.save_data(200, 2.5, './xyz_rotated_1.txt', './../maps/KF_trajectory.txt')
    # excluding ground points
    points_2d = ogm.filter_roof_points(points, mean*1.5)
    #points_3d = ogm.filter_ground_points(points, mean/2.5)
    # draw KF and map points
    img = ogm.draw_2d_point_cloud(points_2d, keyframe)
    cv2.imshow('original', img)

    # my proposal - draw OGM
    #img_2 = ogm.bounding_box(keyframe, img, 20)
    #cv2.imwrite('OGM.png', img_2) 

    # line proposal
    #img_3 = ogm.draw_lines(points_2d, keyframe)

    # kernel = np.ones((2,10),np.uint8)
    # img = cv2.dilate(img,kernel,iterations = 1)

    #cv2.imshow('my proposal', img_2)
    #cv2.imshow('line proposal', img_3)
    cv2.waitKey(0)

if __name__ == "__main__":

    main()


# draw only KF
#img = ogm.draw_2d_keyframes(points_2d, keyframe)

# SECOND APPROACH
# points, keyframe, mean = ogm.save_data(100, 1.5, './../../../maps/landmarks_and_timestamp.txt', './../../../maps/keyframe_trajectory.txt')
# points_2d = ogm.filter_ground_points(points, mean/2)
# points_3d = ogm.filter_roof_points(points_2d, mean*2)

# img = ogm.draw_2d_point_cloud(points_3d, keyframe)

# img = ogm.draw_lines2(keyframe, img)

# kernel = np.ones((2,2),np.uint8)
# img = cv2.dilate(img,kernel,iterations = 1)



# cv2.imshow('original', img)
# cv2.waitKey(0)