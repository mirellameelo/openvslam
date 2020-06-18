import OGM as ogm
import cv2
import numpy as np
import matplotlib.pyplot as plt

import open3d as o3d



def main():

    points, keyframe, mean = ogm.save_data(100, 1.5, './../../../maps/landmarks_and_timestamp.txt', './../../../maps/keyframe_trajectory.txt')
    pcd = o3d.geometry.PointCloud()  
    a = np.zeros(shape=(len(points), 3))     
    # dado conferido  
    for i in range(len(points)):
        a[i, 0] = points[i][1]    
        a[i, 1] = points[i][2]
        a[i, 2] = points[i][3]         
    pcd.points = o3d.utility.Vector3dVector(a)
    print(pcd)
    img = ogm.draw_2d_point_cloud2(a, keyframe)
    cl, ind = pcd.remove_statistical_outlier(nb_neighbors=2, std_ratio=2)
    print(cl)
    xyz_load = np.asarray(cl.points)
    b = np.zeros(shape=(len(xyz_load), 3)) 
    for i in range(len(xyz_load)):
        b[i, 0] = xyz_load[i][0]    
        b[i, 1] = xyz_load[i][1]
        b[i, 2] = xyz_load[i][2]    
    
    c = ogm.filter_ground_points(b, mean/4)
    d = ogm.filter_roof_points(c, mean)
    
    img2 = ogm.draw_2d_point_cloud2(b, keyframe)

    
    # print(max(a[:,0]))
    # outrem = cloud.make_RadiusOutlierRemoval()
    # outrem.set_radius_search(20)
    # outrem.set_MinNeighborsInRadius(1)
    # cloud_filtered = outrem.filter()
    # print(outrem)
    # for i in range(0, cloud_filtered.size):
    #     print('x: ' + str(cloud_filtered[i][0]) + ', y : ' + str(
    #         cloud_filtered[i][1]) + ', z : ' + str(cloud_filtered[i][2]))


    cv2.imshow('original', img)
    cv2.imshow('saporra', img2)
    cv2.waitKey(0)

if __name__ == "__main__":

    main()


# PROXIMOS PASSOS: 
# automatizar o limite de filtrar ground points
# on the fly?
# carregar grid no rviz
# carregar localizacao
# calibrar camera do celular
# fazer video .mp4 com o celular da casa

# # FIRST APPROACH
# # data treatment
# points, keyframe, mean = ogm.save_data(40, 1.5, './../../../maps/landmarks_and_timestamp.txt', './../../../maps/keyframe_trajectory.txt')
# # excluding ground points
# points_2d = ogm.filter_ground_points(points, mean)
# # draw KF and map points
# img = ogm.draw_2d_point_cloud(points, keyframe)
# cv2.imshow('original', img)

# # my proposal - draw OGM
# img_2 = ogm.bounding_box(keyframe, img, 20)
# cv2.imwrite('OGM.png', img_2) 
# print(img_2.shape)

# # line proposal
# img_3 = ogm.draw_lines(points_2d, keyframe)

# # kernel = np.ones((2,10),np.uint8)
# # img = cv2.dilate(img,kernel,iterations = 1)

# cv2.imshow('my proposal', img_2)
# cv2.imshow('line proposal', img_3)
# cv2.waitKey(0)



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