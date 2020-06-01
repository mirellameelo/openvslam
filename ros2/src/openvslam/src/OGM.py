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
    KF_timestamp = []
    x_KF_position = []
    y_KF_position = []
    z_KF_position = []
    x_KF = []
    y_KF = []
    z_KF = []
    KF_quarternion_x = []
    KF_quarternion_y = []
    KF_quarternion_z = []
    KF_quarternion_w = []
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

    # read keyframes data from txt
    data_keyframe_related = open(path_to_KF).readlines()
    for line in data_keyframe_related:
        line_splited = line.split(' ')
        keyframe_id.append(int(line_splited[0]))
        KF_timestamp.append(float(line_splited[1]))
        x_KF_position.append(float(line_splited[2]) * scale_factor)
        y_KF_position.append(float(line_splited[3]) * scale_factor)
        z_KF_position.append(float(line_splited[4]) * scale_factor)
        KF_quarternion_x.append(float(line_splited[5]))
        KF_quarternion_y.append(float(line_splited[6]))
        KF_quarternion_z.append(float(line_splited[7]))
        KF_quarternion_w.append(float(line_splited[8]))
        
    # 3D points data treatment
    threshhold_x_sup = np.mean(x_3d_position_raw) - quartile * np.std(x_3d_position_raw)
    threshhold_x_inf = np.mean(x_3d_position_raw) + quartile * np.std(x_3d_position_raw)

    threshhold_y_sup = np.mean(y_3d_position_raw) - quartile * np.std(y_3d_position_raw)
    threshhold_y_inf = np.mean(y_3d_position_raw) + quartile * np.std(y_3d_position_raw)

    threshhold_z_sup = np.mean(z_3d_position_raw) - quartile * np.std(z_3d_position_raw)
    threshhold_z_inf = np.mean(z_3d_position_raw) + quartile * np.std(z_3d_position_raw)

    # save 3d point data treated 
    for i in range(len(x_3d_position_raw)):
        if x_3d_position_raw[i] > threshhold_x_sup and  y_3d_position_raw[i] > threshhold_y_sup and z_3d_position_raw[i] > threshhold_z_sup:
            if x_3d_position_raw[i] < threshhold_x_inf and y_3d_position_raw[i] < threshhold_y_inf and z_3d_position_raw[i] < threshhold_z_inf:
                keyframe_point_id.append(first_keyframe_id[i])
                x_3d_position.append(x_3d_position_raw[i])
                y_3d_position.append(y_3d_position_raw[i])
                z_3d_position.append(z_3d_position_raw[i])

    # shifting x and z axis to origin sistem coordinator
    shift_x_point = min(x_3d_position)
    shift_y_point = min(y_3d_position)
    shift_z_point = min(z_3d_position)
    
    shift_x_KF = min(x_KF_position)
    shift_y_KF = min(y_KF_position)
    shift_z_KF = min(z_KF_position)

    shift_x = abs(min(shift_x_point, shift_x_KF)) + 10
    shift_y = abs(min(shift_y_point, shift_y_KF)) + 10
    shift_z = abs(min(shift_z_point, shift_z_KF)) + 10

    for x_3d_position in x_3d_position:
        x_3d.append(x_3d_position + shift_x)
    for y_3d_position in y_3d_position:
        y_3d.append(y_3d_position + shift_y)
    for z_3d_position in z_3d_position:
        z_3d.append(z_3d_position + shift_z)

    for x_KF_position in x_KF_position:
        x_KF.append(x_KF_position + shift_x)
    for y_KF_position in y_KF_position:
        y_KF.append(y_KF_position + shift_y)
    for z_KF_position in z_KF_position:
        z_KF.append(z_KF_position + shift_z)

    # save 3d point data treated
    for i in range(len(x_3d)):
        points_3d_data.append([keyframe_point_id[i], x_3d[i] , y_3d[i], z_3d[i]])

    # save keyframe data treated 
    for i in range(len(x_KF)):
        keyframe_data.append([ keyframe_id[i], KF_timestamp[i], x_KF[i], 
                                y_KF[i], z_KF[i], KF_quarternion_x[i], 
                                KF_quarternion_y[i], KF_quarternion_z[i], KF_quarternion_w[i] ])

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
    for i in range(len(points_3d_data)):
        x_3d.append(points_3d_data[i][1])
        z_3d.append(points_3d_data[i][3])

    # saving 2d keyframe position
    for i in range(len(keyframe_data)):
        x_keyframe.append(keyframe_data[i][2])
        z_keyframe.append(keyframe_data[i][4])

    # OGM windown size taking into consideration the scale factor
    a = int(max(x_keyframe))
    b = int(max(z_keyframe))
    c = int(max(x_3d))
    d = int(max(z_3d))

    windown_size_x = max(a,c) + 10
    windown_size_z = max(b,d) + 10
    img = [windown_size_x, windown_size_z]
    img = np.zeros((windown_size_x, windown_size_z, 3), np.uint8)


    # drawing 2d points position in red color
    for i in range(len(x_3d)):    
        img[int(x_3d[i]),int(z_3d[i])] = (0,0, 255)   


    # drawing 2d keyframe positon in light blue color
    for i in range(len(x_keyframe)):
        img[int(x_keyframe[i]), int(z_keyframe[i])] = (255, 255, 0)

    return img



def draw_2d_keyframes(points_3d_data, keyframe_data):

    print('Use the output data from save_data() function.')

    # vectors related to 2d points position
    x_3d = []
    z_3d = []

    # vectors related to 2d keyframe position
    x_keyframe = []
    z_keyframe = []

    # saving 2d points position
    for i in range(len(points_3d_data)):
        x_3d.append(points_3d_data[i][1])
        z_3d.append(points_3d_data[i][3])

    # OGM windown size taking into consideration the scale factor
    windown_size_x = int(max(x_3d)) + 10
    windown_size_z = int(max(z_3d)) + 10
    #if imgg.all == None:
    img = [windown_size_x, windown_size_z]
    img = np.zeros((windown_size_x, windown_size_z, 3), np.uint8)
    #else:
    #    img = imgg


    # saving vectors to 2d keyframe position
    for i in range(len(keyframe_data)):
        x_keyframe.append(keyframe_data[i][2])
        z_keyframe.append(keyframe_data[i][4])

    # drawing 2d keyframe positon in light blue color
    for i in range(len(x_keyframe)):
       img[int(x_keyframe[i]), int(z_keyframe[i])] = (255, 255, 0)
    
    kernel = np.ones((20,20),np.uint8)
    #img = cv2.erode(img,kernel,iterations = 1)
    #img = cv2.dilate(img,kernel,iterations = 1)
    kernel = np.ones((20,20),np.uint8)
    #img = cv2.erode(img,kernel,iterations = 1)

    return img



def filter_ground_points(points_3d_data, threshhold):

    print('Use the output data from save_data() function.')
    ground = []
    for i in range(len(points_3d_data)):
        if points_3d_data[i][2] > threshhold:
            ground.append(points_3d_data[i])
    
    return ground


def draw_lines(points_3d_data, keyframe):
    
     # vectors related to 2d points position
    x_3d = []
    z_3d = []

    # vectors related to 2d keyframe position
    x_keyframe = []
    z_keyframe = []
    KF_id = []

    # saving 2d points position
    for i in range(len(points_3d_data)):
        x_3d.append(points_3d_data[i][1])
        z_3d.append(points_3d_data[i][3])
 

    # OGM windown size taking into consideration the scale factor
    windown_size_x = int(max(x_3d)) + 10
    windown_size_z = int(max(z_3d)) + 10
    img = [windown_size_x, windown_size_z]
    img = np.zeros((windown_size_x, windown_size_z, 3), np.uint8)

    for i in range(len(keyframe)):
        KF_id = keyframe[i][0]
        for a in range(len(points_3d_data)):
            if points_3d_data[a][0] == KF_id:
                c = int(keyframe[i][2])
                d = int(keyframe[i][4])
                e = int(points_3d_data[a][1])
                f = int(points_3d_data[a][3])
                ray_points = get_line_bresenham( [c, d], [e,f] )
                #print(ray_points)
                for b in range(len(ray_points)):
                    img[ray_points[b][0], ray_points[b][1]] = (105, 255, 100)

    return img

def get_line_bresenham(start, end):
    """Bresenham's Line Algorithm
    Produces a list of tuples from start and end
    """
    # Setup initial conditions
    x1, y1 = start
    x2, y2 = end
    dx = x2 - x1
    dy = y2 - y1

    #
    # Determine how steep the line is
    is_steep = abs(dy) > abs(dx) #output: true or false

    # Rotate line
    if is_steep: #if true, swap start and end points
        x1, y1 = y1, x1
        x2, y2 = y2, x2

    # Swap start and end points if necessary and store swap state
    swapped = False
    if x1 > x2:
        x1, x2 = x2, x1
        y1, y2 = y2, y1
        swapped = True

    # Recalculate differentials
    dx = x2 - x1
    dy = y2 - y1

    # Calculate error
    error = int(dx / 2.0)
    ystep = 1 if y1 < y2 else -1

    # Iterate over bounding box generating points between start and end
    y = y1
    points = []
    for x in range(x1, x2 + 1):
        coord = (y, x) if is_steep else (x, y)
        points.append(coord)
        error -= abs(dy)
        if error < 0:
            y += ystep
            error += dx

    # Reverse the list if the coordinates were swapped
    if swapped:
        points.reverse()

    return points #return the points that indicate the LINE


def bounding_box(keyframe_data, image, threshhold):

    bound_box = []
    z_KF = []
    img = image
    value = 10
    # saving 2d points position
    for i in range(len(keyframe_data)):
        num_of_landmarks = 0
        value = 1

        # se o numero de landmarks still small, keep growing
        while num_of_landmarks < threshhold:

            start_x = int(keyframe_data[i][2] - value)
            end_x = int(keyframe_data[i][2] + value + 1)
            start_z = int(keyframe_data[i][4] - value)
            end_z = int(keyframe_data[i][4] + value + 1)
            for b in range(start_x, end_x):
                for c in range(start_z, end_z):
                    color = img[b, c]
                    if color[2] == 255:
                        #print('keyframe_pixel: ', keyframe_data[i][2])
                        #print(' pixel: [', b, ', ', c, end='], ')
                        #print(color)
                        num_of_landmarks = num_of_landmarks + 1

            if num_of_landmarks >= threshhold: 
                for d in range(start_x, end_x):
                    for e in range(start_z, end_z):
                        img[d, e] = (255, 255, 0)   
            else:
                value = value + 1     
        #bound_box_x.append(keyframe_data[i][2])
        #        bound_box_x_size.append(value)
                    # save data - ponto e tamanho vertical e horizontal
                    #print(value)

    #for d in range(start_x, end_x):
    #    for e in range(start_z, end_z):
    #        img[d, e] = (255, 255, 0)
                
    return img