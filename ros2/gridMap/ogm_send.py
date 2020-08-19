import OGM as ogm
import cv2
import numpy as np
import matplotlib.pyplot as plt
import imutils

import open3d as o3d

def bounding_box(im):
    gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY) # convert to grayscale
    blur = cv2.blur(gray, (5, 5)) # blur the image
    ret, thresh = cv2.threshold(blur, 50, 255, cv2.THRESH_BINARY)
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # create hull array for convex hull points
    hull = []
    # calculate points for each contour
    for i in range(len(contours)):
        # creating convex hull object for each contour
        hull.append(cv2.convexHull(contours[i], False))
    # create an empty black image
    drawing = np.zeros((thresh.shape[0], thresh.shape[1], 3), np.uint8)
    # draw contours and hull points
    for i in range(len(contours)):
        color_contours = (0, 255, 0) # green - color for contours
        color = (255, 0, 0) # blue - color for convex hull
        # draw ith contour
        cv2.drawContours(drawing, contours, i, color_contours, 1, 8, hierarchy)
        # draw ith convex hull object
        cv2.drawContours(drawing, hull, i, color, -1, 8)
    return drawing

def main():
   
# # TOP FILTER
    points, keyframe, mean = ogm.save_data_2(150, 4.5, './xyz_rotated.txt', './KF_trajectory.txt')
    #points, keyframe, mean = ogm.save_data_2(300, 2.5, './../maps/xyz_simulator.txt', './../maps/KF_trajectory.txt')
    im = ogm.draw_2d_point_cloud2(points, keyframe)

    # # filter by radius and density
    # pcd = o3d.geometry.PointCloud()    
    # pcd.points = o3d.utility.Vector3dVector(points)
    # cl, ind = pcd.remove_statistical_outlier(nb_neighbors=2, std_ratio=1)
    # xyz_load = np.asarray(cl.points)
    # # excluding ground points
    # points_2d = ogm.filter_ground_points(xyz_load, mean)
    # # excluding roof points
    # points_3d = ogm.filter_roof_points(points_2d, mean*5)

    # #visualizacao, ignorar
    # img = ogm.draw_2d_point_cloud2(points_3d, keyframe)
    # img3 = ogm.draw_2d_point_cloud2(points, keyframe)
    # img7 = ogm.draw_2d_keyframes(points_3d, keyframe)
    # img4 = ogm.draw_lines2(keyframe, img3)
    # # dilate point cloud 
    # kernel = np.ones((5,5),np.uint8)
    # image = cv2.dilate(img,kernel,iterations = 1)
    # # contour
    # resized = imutils.resize(image, width=300)
    # ratio = image.shape[0] / float(resized.shape[0])
    # gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
    # blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    # thresh = cv2.threshold(blurred, 50, 255, cv2.THRESH_BINARY)[1]
    # cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # cnts = imutils.grab_contours(cnts)
    # for c in cnts:
    #     M = cv2.moments(c)
    #     c = c.astype("float")
    #     c *= ratio
    #     c = c.astype("int")
    #     im = cv2.drawContours(image, [c], -1, (0, 255, 0), 2)


    # drawing = bounding_box(im)
    cv2.imshow('imasg',im)
    



    # # dilate keyframes
    # kernel2 = np.ones((30,30),np.uint8)
    # img5 = cv2.dilate(img3,kernel2,iterations = 3)
    # # contour
    # resized = imutils.resize(img5, width=300)
    # ratio = img5.shape[0] / float(resized.shape[0])
    # gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
    # blurred = cv2.GaussianBlur(gray, (7, 7), 0)
    # thresh = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY)[1]
    # cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # cnts = imutils.grab_contours(cnts)
    # for c in cnts:
    #     M = cv2.moments(c)
    #     c = c.astype("float")
    #     c *= ratio
    #     c = c.astype("int")
    #     cv2.drawContours(image, [c], -1, (0, 255, 0), 2)

    #cv2.imshow('img',img)
    #cv2.imshow('imasdg',img5)
    cv2.waitKey(0)   


if __name__ == "__main__":

    main()

