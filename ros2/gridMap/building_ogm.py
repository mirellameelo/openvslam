import OGM as ogm
import cv2
import numpy as np
import matplotlib.pyplot as plt
import imutils

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
   
# # TOP FILTER
    #points, keyframe, mean = ogm.save_data_2(200, 2.5, './xyz_rotated_1.txt', './KF_rotated_1.txt')
    points, keyframe, mean = ogm.save_data_2(300, 2.5, './../maps/xyz_simulator.txt', './../maps/KF_trajectory.txt')
    im = ogm.draw_2d_point_cloud2(points, keyframe)


    # excluding ground points
    points_2d = ogm.filter_ground_points(points, mean/3)
    points_3d = ogm.filter_roof_points(points_2d, mean*2)
    img2 = ogm.draw_2d_point_cloud2(points_3d, keyframe) 

    pcd = o3d.geometry.PointCloud()    
    pcd.points = o3d.utility.Vector3dVector(points_3d)

    cl, ind = pcd.remove_statistical_outlier(nb_neighbors=2, std_ratio=1)
    xyz_load = np.asarray(cl.points)

    img = ogm.draw_2d_point_cloud2(xyz_load, keyframe)

    img3 = ogm.draw_2d_keyframes(xyz_load, keyframe)
    img4 = ogm.draw_lines2(keyframe, img3)
    # cv2.imwrite('please.png', img3)

    # points, keyframe, mean = ogm.save_data_2(200, 2.5, './xyz_rotated_1.txt', './KF_rotated_1.txt')
    # points_2d = ogm.get_ground_points(points, mean*1.2)
    # im = ogm.draw_2d_point_cloud2(points_2d, keyframe)
   
    kernel = np.ones((4,4),np.uint8)
    image = cv2.dilate(img,kernel,iterations = 1)
    # load the image and resize it to a smaller factor so that
    # the shapes can be approximated better
    resized = imutils.resize(image, width=300)
    ratio = image.shape[0] / float(resized.shape[0])
    gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (7, 7), 0)
    thresh = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY)[1]
    cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    sd = ShapeDetector()
    for c in cnts:
        M = cv2.moments(c)
        shape = sd.detect(c)
        c = c.astype("float")
        c *= ratio
        c = c.astype("int")
        cv2.drawContours(image, [c], -1, (0, 255, 0), 2)
    cv2.imshow("Image", image)
    #cv2.imwrite('coisalinda.png', image)

    

    # contorno dos retangulos
    # gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    # contours, hierarchy = cv2.findContours(gray,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)[-2:]
    # idx =0 
    # for cnt in contours:
    #     idx += 1
    #     x,y,w,h = cv2.boundingRect(cnt)
    #     roi=image[y:y+h,x:x+w]
    #     cv2.rectangle(image,(x,y),(x+w,y+h),(200,0,0),1)


    kernel2 = np.ones((30,30),np.uint8)
    img5 = cv2.dilate(img3,kernel2,iterations = 3)
    resized = imutils.resize(img5, width=300)
    ratio = img5.shape[0] / float(resized.shape[0])
    gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (7, 7), 0)
    thresh = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY)[1]
    cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    sd = ShapeDetector()
    for c in cnts:
        M = cv2.moments(c)
        shape = sd.detect(c)
        c = c.astype("float")
        c *= ratio
        c = c.astype("int")
        cv2.drawContours(image, [c], -1, (0, 255, 0), 2)

    cv2.imshow('img',image)
    cv2.imshow('imasdg',img5)

    new_color = [image.shape[0], image.shape[1]]
    new_color = np.zeros((image.shape[0], image.shape[1], 3), np.uint8) 

    # # somar cor dos pixels:
    # for x in range(image.shape[0]):
    #     for y in range(image.shape[1]):
    #         color_img_1 = image[x, y]
    #         color_img_2 = img5[x, y]
    #         if color_img_1[0] > 180 and color_img_2[2] > 240:
    #             a = 100
    #             b = 100
    #             c = 100
    #             new_color[x,y] = (a, b, c)
    #         elif color_img_1[0] > 180 and color_img_2[2] < 10:
    #             a = 255
    #             b = 0
    #             c = 0
    #             new_color[x,y] = (a, b, c)

    # cv2.imshow('new',new_color)

    cv2.waitKey(0)   






    # FIRST APPROACH
    # data treatment
    # excluding ground points
    #points_2d = ogm.filter_roof_points(points, mean*1.5)
    #points_3d = ogm.filter_ground_points(points, mean/2.5)
    # draw KF and map points
    #img = ogm.draw_2d_point_cloud(points_2d, keyframe)
    #cv2.imshow('original', img)

    # my proposal - draw OGM
    #img_2 = ogm.bounding_box(keyframe, img, 20)
    #cv2.imwrite('OGM.png', img_2) 

    # line proposal
    #img_3 = ogm.draw_lines(points_2d, keyframe)

    # kernel = np.ones((2,10),np.uint8)
    # img = cv2.dilate(img,kernel,iterations = 1)

    #cv2.imshow('my proposal', img)
    #cv2.imshow('imag2', img2)
    #cv2.imshow('line proposal', img3)
    #cv2.waitKey(0)

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