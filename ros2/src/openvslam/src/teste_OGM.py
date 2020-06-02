import OGM as ogm
import cv2
import numpy as np


# PROXIMOS PASSOS: 
# automatizar o limite de filtrar ground points
# on the fly?
# carregar grid no rviz
# carregar localizacao
# calibrar camera do celular
# fazer video .mp4 com o celular da casa


# data treatment
points, keyframe = ogm.save_data(40, 2.0, './../../../maps/landmarks_and_timestamp.txt', './../../../maps/keyframe_trajectory.txt')

# excluding ground points
points_2d = ogm.filter_ground_points(points, 30)

# draw only KF
#img = ogm.draw_2d_keyframes(points_2d, keyframe)

# draw KF and lmap points
img = ogm.draw_2d_point_cloud(points_2d, keyframe)
cv2.imshow('original', img)

# my proposal - draw OGM
img_2 = ogm.bounding_box(keyframe, img, 8)

# line proposal
#img_3 = ogm.draw_lines(points_2d, keyframe)

# kernel = np.ones((2,10),np.uint8)
# img = cv2.dilate(img,kernel,iterations = 1)

cv2.imshow('my proposal', img_2)
#cv2.imshow('line proposal', img_3)
cv2.waitKey(0)

