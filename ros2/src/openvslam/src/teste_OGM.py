import OGM as ogm
import cv2
import numpy as np

points, keyframe = ogm.save_data(500, 1.5, './../../../maps/landmarks_and_timestamp.txt', './../../../maps/keyframe_trajectory.txt')

points_2d = ogm.filter_ground_points(points, 40)
img = ogm.draw_2d_point_cloud(points, keyframe)


kf_and_points = []
x = []
z = []

a = []
a = np.shape(img)

# kernel = np.ones((5,5),np.uint8)
# img = cv2.erode(img,kernel,iterations = 1)
# img = cv2.dilate(img,kernel,iterations = 1)


# for i in range(len(points_2d)):
#     x.append(points_2d[i][2])
#     z.append(points_2d[i][3])

# for i in range(len(keyframe)):
#     KF_id = keyframe[i][0]
#     for a in range(len(points_2d)):
#         if points_2d[a][0] == KF_id:
#             c = int(keyframe[i][2])
#             d = int(keyframe[i][4])
#             e = int(points_2d[a][1])
#             f = int(points_2d[a][3])
#             ray_points = ogm.get_line_bresenham( [c, d], [e,f] )
#             #print(ray_points)
#             for b in range(len(ray_points)):
#                 img[ray_points[b][0], ray_points[b][1]] = (105, 255, 100)

cv2.imshow('hello hello remember me?', img)
cv2.waitKey(0)


# ros2 run openvslam run_localization -v  $HOME/openvslam/build/orb_vocab/orb_vocab.dbow2 -c $HOME/openvslam/build/aist_living_lab_1/config.yaml --map-db /home/mirellameelo/openvslam/ros2/maps/mymap.msg