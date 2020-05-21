import numpy as np
import cv2
import math

def save_data(scale_factor, quartile, path_to_3d_points, path_to_KF):

    print('The input data must be a .txt file, organized as follows: \n-> path_to_3d_points: [1st keyframe id] [x position] [y position] [z position]\n->') 
    print('path_to_KF: [keyframe id] [timestamp] [x_position] [y position] [z position] [] [] [] ')

    # scale_factor affects the window size to vizualization
    scale_factor = scale_factor
    # quartile affects the treatment of the raw data. For the biggest values, more chances of outliers. Suggested: 1.5
    quartile = quartile

    # vectors related to 3D points 
    first_keyframe_id = []
    keyframe_point_id = []
    x_3d_position_raw = []
    y_3d_position_raw = []
    z_3d_position_raw = []
    x_3d_position = []
    y_3d_position = []
    z_3d_position = []
    x_3d = []
    y_3d = []
    z_3d = []
    # output 1: data treated. List: ( first keyframe id, x position, y position, z position )
    points_3d_data = []

    # vectors related to keyframe
    keyframe_id = []
    x_KF_position = []
    y_KF_position = []
    z_KF_position = []
    KF_timestamp = []
    # output 2: data treated. List: ( keyframe id, timestamp, x position, y position, z position )
    keyframe_data = []

    # read 3D points data from txt
    data_point_related = open(path_to_3d_points, 'r').readlines()
    for line in data_point_related:
        line_splited = line.split(' ')  
        # convert txt em int/float and save into vectors
        first_keyframe_id.append(int(line_splited[0]))
        x_3d_position_raw.append(float(line_splited[1]) * scale_factor)
        y_3d_position_raw.append(float(line_splited[2]) * scale_factor)
        z_3d_position_raw.append(float(line_splited[3]) * scale_factor)

    # data treatment
    threshhold_x_sup = np.mean(x_3d_position_raw) - quartile * np.std(x_3d_position_raw)
    threshhold_x_inf = np.mean(x_3d_position_raw) + quartile * np.std(x_3d_position_raw)

    threshhold_y_sup = np.mean(y_3d_position_raw) - quartile * np.std(y_3d_position_raw)
    threshhold_y_inf = np.mean(y_3d_position_raw) + quartile * np.std(y_3d_position_raw)

    threshhold_z_sup = np.mean(z_3d_position_raw) - quartile * np.std(z_3d_position_raw)
    threshhold_z_inf = np.mean(z_3d_position_raw) + quartile * np.std(z_3d_position_raw)

    # save 3d point data treated 
    for i in xrange(len(x_3d_position_raw)):
        if x_3d_position_raw[i] > threshhold_x_sup and  y_3d_position_raw[i] > threshhold_y_sup and z_3d_position_raw[i] > threshhold_z_sup:
            if x_3d_position_raw[i] < threshhold_x_inf and y_3d_position_raw[i] < threshhold_y_inf and z_3d_position_raw[i] < threshhold_z_inf:
                keyframe_point_id.append(first_keyframe_id[i])
                x_3d_position.append(x_3d_position_raw[i])
                y_3d_position.append(y_3d_position_raw[i])
                z_3d_position.append(z_3d_position_raw[i])

    # shifting x and z axis to origin sistem coordinator
    shift_x = abs(min(x_3d_position))
    shift_y = abs(min(y_3d_position))
    shift_z = abs(min(z_3d_position))
    for x_3d_position in x_3d_position:
        x_3d.append(x_3d_position + shift_x)
    for y_3d_position in y_3d_position:
        y_3d.append(y_3d_position + shift_y)
    for z_3d_position in z_3d_position:
        z_3d.append(z_3d_position + shift_z)

    # save 3d point data treated
    for i in xrange(len(x_3d)):
        points_3d_data.append((keyframe_point_id[i], x_3d[i] , y_3d[i], z_3d[i]))

    #read keyframes data from txt
    data_keyframe_related = open(path_to_KF).readlines()
    for line in data_keyframe_related:
        line_splited = line.split(' ')
        keyframe_id.append(int(line_splited[0]))
        KF_timestamp.append(float(line_splited[1]))
        x_KF_position.append(float(line_splited[2]) * scale_factor + shift_x)
        y_KF_position.append(float(line_splited[3]) * scale_factor + shift_y)
        z_KF_position.append(float(line_splited[4]) * scale_factor + shift_z)

    # save keyframe data treated 
    for i in xrange(len(x_KF_position)):
        keyframe_data.append(( keyframe_id[i], KF_timestamp[i], x_KF_position[i] , y_KF_position[i], z_KF_position[i]))

    #return 3d point data and keyframe data
    return points_3d_data, keyframe_data



def draw_2d_point_cloud(points_3d_data, keyframe_data):

    print('Use the output data from save_data() function.')

    # vectors related to 2d points position
    x_3d = []
    z_3d = []

    # vectors related to 2d keyframe position
    x_keyframe = []
    z_keyframe = []

    # saving 2d points position
    for i in xrange(len(points_3d_data)):
        x_3d.append(points_3d_data[i][1])
        z_3d.append(points_3d_data[i][3])

    # OGM windown size taking into consideration the scale factor
    windown_size_x = int(max(x_3d)) + 10
    windown_size_z = int(max(z_3d)) + 10
    img = [windown_size_x, windown_size_z]
    img = np.zeros((windown_size_x, windown_size_z, 3), np.uint8)

    # drawing 2d points position in red color
    for i in xrange(len(x_3d)):    
        img[int(x_3d[i]),int(z_3d[i])] = (0,0, 255)   

    # saving vectors to 2d keyframe position
    for i in xrange(len(keyframe_data)):
        x_keyframe.append(keyframe_data[i][2])
        z_keyframe.append(keyframe_data[i][4])

    print(len(z_keyframe))
    print(len(x_keyframe))
    print(len(x_3d))
    print(len(z_3d))
    # drawing 2d keyframe positon in light blue color
    for i in xrange(len(x_keyframe)):
        img[int(x_keyframe[i]), int(z_keyframe[i])] = (255, 255, 0)

    # showing the image result
    cv2.imshow('3D points and keyframes.', img)
    cv2.waitKey(0)
