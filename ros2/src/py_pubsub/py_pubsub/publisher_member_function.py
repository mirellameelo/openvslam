import rclpy
from rclpy.node import Node
import numpy as np
import cv2
import math
import os
import time
from ament_index_python.packages import get_package_share_directory
# mudar isso p grid
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from std_msgs.msg import String
from std_msgs.msg import Header
from datetime import datetime

import launch.actions
import launch_ros.actions

class PubSub(Node):
    def __init__(self):
        super().__init__('PubSub')
        self.subscription = self.create_subscription(Odometry, 'odometry', self.listener_callback, 10)
        self.marker_pub = self.create_publisher(Odometry,'marker')
        # grid
        self.map_pub = self.create_publisher(OccupancyGrid,'topic')

        msg = OccupancyGrid()
        img = cv2.imread('OGM.png')
        myvec = []
        msg.info.resolution = 0.05
        msg.info.width = img.shape[1]
        msg.info.height = img.shape[0]
        msg.info.origin.position.x = 0.0
        msg.info.origin.position.y = 0.0
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.x = 0.0
        msg.info.origin.orientation.y= 0.0
        msg.info.origin.orientation.z = 0.0
        msg.info.origin.orientation.w = 1.0
        for j in range(img.shape[0]):
            for k in range(img.shape[1]):
                # branco
                if img[j, k][0] > 200:
                    myvec.append(0)
                else:
                    myvec.append(100)
        msg.data= myvec
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        self.map_pub.publish(msg)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, odom):
        marker = Odometry()
        marker.header.frame_id = "map"
        #marker.type = Marker.ARROW
        #marker.scale.x = 1.0
        #marker.scale.y = 0.1
        #marker.scale.z = 0.1
        #marker.color.r = 1.0
        #marker.color.g = 0.0
        #marker.color.b = 0.0
        #marker.color.a = 1.0
        marker.pose.pose.position.x = 0.0 #odom.pose.pose.position.x
        marker.pose.pose.position.y = 0.0 #odom.pose.pose.position.z
        marker.pose.pose.position.z = 0.0 # odom.pose.pose.position.z

        marker.pose.pose.orientation.x = 0.0 #odom.pose.orientation.x
        marker.pose.pose.orientation.y= 0.0 #odom.pose.orientation.y
        marker.pose.pose.orientation.z = 0.0 #odom.pose.orientation.z
        marker.pose.pose.orientation.w = 0.1 #odom.pose.orientation.w

        self.marker_pub.publish(marker)
        self.get_logger().info('I hear: "%s" ' % odom.pose.pose.position)

def main(args=None):
    rclpy.init(args=args)

    PubSub_ = PubSub()

    rclpy.spin(PubSub_)

    PubSub_.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()




#THE EASIER CODE FOR PUBLISING A STRING
# rclpy.init()
# node=rclpy.create_node('minimal_publisher')
# chatter_pub=node.create_publisher(String,'topic')
# msg=String()
# i=1
# while True:
#     msg.data='Hello World: {0}'.format(i)
#     i+=1
#     print('Publishing: "{0}"'.format(msg.data))
#     chatter_pub.publish(msg)
#     time.sleep(0.5)



# class MinimalPublisher(Node):

#     def __init__(self):
#         super().__init__('minimal_publisher')
#         self.publisher_ = self.create_publisher(OccupancyGrid, 'topic', 10)
#         timer_period = 0.5  # seconds
#         self.timer = self.create_timer(timer_period, self.timer_callback)
#         self.i = 0
#         default_yaml = os.path.join(get_package_share_directory('publi'), 'param','map.yaml')

#     def timer_callback(self):
#         msg = OccupancyGrid()
#         msg.data = (1, 1)
#         self.publisher_.publish(msg)
#         self.get_logger().info('Publishing: "%s"' % msg.data)
#         self.i += 1


# def main(args=None):
#     points, keyframe = save_data(40, 2.0, '/home/mirellameelo/openvslam/ros2/maps/landmarks_and_timestamp.txt', '/home/mirellameelo/openvslam/ros2/maps/keyframe_trajectory.txt')
#     # draw KF and lmap points
#     img = cv2.imread('OGM.png') 
#     print(img.shape)

#     rclpy.init(args=args)

#     minimal_publisher = MinimalPublisher()

#     rclpy.spin(minimal_publisher)

#     # Destroy the node explicitly
#     # (optional - otherwise it will be done automatically
#     # when the garbage collector destroys the node object)
#     minimal_publisher.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
# def listener_callback(self, msg):
#        get_logger().info('I heard: "%s"' % msg.data)
#     # data treatment
#     main()


# def save_data(scale_factor, quartile, path_to_3d_points, path_to_KF):

#     print('The input data must be a .txt file, organized as follows: \n-> path_to_3d_points: [1st keyframe id] [x position] [y position] [z position]\n->') 
#     print('path_to_KF: [keyframe id] [timestamp] [x_position] [y position] [z position] [] [] [] ')

#     # scale_factor affects the window size to vizualization
#     scale_factor = scale_factor
#     # quartile affects the treatment of the raw data. For the biggest values, more chances of outliers. Suggested: 1.5
#     quartile = quartile

#     # vectors related to 3D points 
#     first_keyframe_id = []
#     keyframe_point_id = []
#     x_3d_position_raw = []
#     y_3d_position_raw = []
#     z_3d_position_raw = []
#     x_3d_position = []
#     y_3d_position = []
#     z_3d_position = []
#     x_3d = []
#     y_3d = []
#     z_3d = []

#     # output 1: data treated. List: ( first keyframe id, x position, y position, z position )
#     points_3d_data = []

#     # vectors related to keyframe
#     keyframe_id = []
#     KF_timestamp = []
#     x_KF_position = []
#     y_KF_position = []
#     z_KF_position = []
#     x_KF = []
#     y_KF = []
#     z_KF = []
#     KF_quarternion_x = []
#     KF_quarternion_y = []
#     KF_quarternion_z = []
#     KF_quarternion_w = []
#     # output 2: data treated. List: ( keyframe id, timestamp, x position, y position, z position )
#     keyframe_data = []

#     # read 3D points data from txt
#     data_point_related = open(path_to_3d_points, 'r').readlines()
#     for line in data_point_related:
#         line_splited = line.split(' ')  
#         # convert txt em int/float and save into vectors
#         first_keyframe_id.append(int(line_splited[0]))
#         x_3d_position_raw.append(float(line_splited[1]) * scale_factor)
#         y_3d_position_raw.append(float(line_splited[2]) * scale_factor)
#         z_3d_position_raw.append(float(line_splited[3]) * scale_factor)

#     # read keyframes data from txt
#     data_keyframe_related = open(path_to_KF).readlines()
#     for line in data_keyframe_related:
#         line_splited = line.split(' ')
#         keyframe_id.append(int(line_splited[0]))
#         KF_timestamp.append(float(line_splited[1]))
#         x_KF_position.append(float(line_splited[2]) * scale_factor)
#         y_KF_position.append(float(line_splited[3]) * scale_factor)
#         z_KF_position.append(float(line_splited[4]) * scale_factor)
#         KF_quarternion_x.append(float(line_splited[5]))
#         KF_quarternion_y.append(float(line_splited[6]))
#         KF_quarternion_z.append(float(line_splited[7]))
#         KF_quarternion_w.append(float(line_splited[8]))
        
#     # 3D points data treatment
#     threshhold_x_sup = np.mean(x_3d_position_raw) - quartile * np.std(x_3d_position_raw)
#     threshhold_x_inf = np.mean(x_3d_position_raw) + quartile * np.std(x_3d_position_raw)

#     threshhold_y_sup = np.mean(y_3d_position_raw) - quartile * np.std(y_3d_position_raw)
#     threshhold_y_inf = np.mean(y_3d_position_raw) + quartile * np.std(y_3d_position_raw)

#     threshhold_z_sup = np.mean(z_3d_position_raw) - quartile * np.std(z_3d_position_raw)
#     threshhold_z_inf = np.mean(z_3d_position_raw) + quartile * np.std(z_3d_position_raw)

#     # save 3d point data treated 
#     for i in range(len(x_3d_position_raw)):
#         if x_3d_position_raw[i] > threshhold_x_sup and  y_3d_position_raw[i] > threshhold_y_sup and z_3d_position_raw[i] > threshhold_z_sup:
#             if x_3d_position_raw[i] < threshhold_x_inf and y_3d_position_raw[i] < threshhold_y_inf and z_3d_position_raw[i] < threshhold_z_inf:
#                 keyframe_point_id.append(first_keyframe_id[i])
#                 x_3d_position.append(x_3d_position_raw[i])
#                 y_3d_position.append(y_3d_position_raw[i])
#                 z_3d_position.append(z_3d_position_raw[i])

#     # shifting x and z axis to origin sistem coordinator
#     shift_x_point = min(x_3d_position)
#     shift_y_point = min(y_3d_position)
#     shift_z_point = min(z_3d_position)
    
#     shift_x_KF = min(x_KF_position)
#     shift_y_KF = min(y_KF_position)
#     shift_z_KF = min(z_KF_position)

#     shift_x = abs(min(shift_x_point, shift_x_KF)) + 10
#     shift_y = abs(min(shift_y_point, shift_y_KF)) + 10
#     shift_z = abs(min(shift_z_point, shift_z_KF)) + 10

#     for x_3d_position in x_3d_position:
#         x_3d.append(x_3d_position + shift_x)
#     for y_3d_position in y_3d_position:
#         y_3d.append(y_3d_position + shift_y)
#     for z_3d_position in z_3d_position:
#         z_3d.append(z_3d_position + shift_z)

#     for x_KF_position in x_KF_position:
#         x_KF.append(x_KF_position + shift_x)
#     for y_KF_position in y_KF_position:
#         y_KF.append(y_KF_position + shift_y)
#     for z_KF_position in z_KF_position:
#         z_KF.append(z_KF_position + shift_z)

#     # save 3d point data treated
#     for i in range(len(x_3d)):
#         points_3d_data.append([keyframe_point_id[i], x_3d[i] , y_3d[i], z_3d[i]])

#     # save keyframe data treated 
#     for i in range(len(x_KF)):
#         keyframe_data.append([ keyframe_id[i], KF_timestamp[i], x_KF[i], 
#                                 y_KF[i], z_KF[i], KF_quarternion_x[i], 
#                                 KF_quarternion_y[i], KF_quarternion_z[i], KF_quarternion_w[i] ])

#     #return 3d point data and keyframe data
#     return points_3d_data, keyframe_data




# def draw_2d_point_cloud(points_3d_data, keyframe_data):

#     print('Use the output data from save_data() function.')

#     # vectors related to 2d points position
#     x_3d = []
#     z_3d = []

#     # vectors related to 2d keyframe position
#     x_keyframe = []
#     z_keyframe = []

#     # saving 2d points position
#     for i in range(len(points_3d_data)):
#         x_3d.append(points_3d_data[i][1])
#         z_3d.append(points_3d_data[i][3])

#     # saving 2d keyframe position
#     for i in range(len(keyframe_data)):
#         x_keyframe.append(keyframe_data[i][2])
#         z_keyframe.append(keyframe_data[i][4])

#     # OGM windown size taking into consideration the scale factor
#     a = int(max(x_keyframe))
#     b = int(max(z_keyframe))
#     c = int(max(x_3d))
#     d = int(max(z_3d))

#     windown_size_x = max(a,c) + 10
#     windown_size_z = max(b,d) + 10
#     img = [windown_size_x, windown_size_z]
#     img = np.zeros((windown_size_x, windown_size_z, 3), np.uint8)


#     # drawing 2d points position in red color
#     for i in range(len(x_3d)):    
#         img[int(x_3d[i]),int(z_3d[i])] = (0,0, 255)   


#     # drawing 2d keyframe positon in light blue color
#     for i in range(len(x_keyframe)):
#         img[int(x_keyframe[i]), int(z_keyframe[i])] = (255, 255, 0)

#     return img

# def bounding_box(keyframe_data, image, threshhold):

#     bound_box = []
#     z_KF = []
#     img = image
#     value = 10
#     vector_start_x = []
#     vector_end_x = []
#     vector_start_z = []
#     vector_end_z = []
#     # saving 2d points position
#     for i in range(len(keyframe_data)):
#         num_of_landmarks = 0
#         value = 1

#         # se o numero de landmarks still small, keep growing
#         while num_of_landmarks < threshhold:

#             start_x = int(keyframe_data[i][2] - value)
#             end_x = int(keyframe_data[i][2] + value + 1)
#             start_z = int(keyframe_data[i][4] - value)
#             end_z = int(keyframe_data[i][4] + value + 1)
#             for b in range(start_x, end_x):
#                 for c in range(start_z, end_z):
#                     color = img[b, c]
#                     if color[2] == 255:
#                         num_of_landmarks = num_of_landmarks + 1
#             # if red points are not enough, ignore this KF
#             if value > threshhold + 5:
#                 break

#             elif num_of_landmarks >= threshhold: 
#                 vector_start_x.append(start_x)
#                 vector_end_x.append(end_x)
#                 vector_start_z.append(start_z)
#                 vector_end_z.append(end_z)
#             else:
#                 value = value + 1     

#     windown_size_x = (img.shape)[0]
#     windown_size_z = (img.shape)[1]
#     #if imgg.all == None:
#     img2 = [windown_size_x, windown_size_z]
#     img2 = np.zeros((windown_size_x, windown_size_z, 3), np.uint8)
    
#     for i in range(len(vector_end_z)):
#         for d in range(vector_start_x[i], vector_end_x[i]):
#             for e in range(vector_start_z[i], vector_end_z[i]):
#                 img2[d, e] = (255, 255, 255)  

#     return img2
