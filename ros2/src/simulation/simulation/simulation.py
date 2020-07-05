import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from .simConst import *
from .sim import *

#simRemoteApi.start(19999)

class simulation(Node):
    def __init__(self):
        super().__init__('simulation')
        bridge = CvBridge()
        self.image = self.create_publisher(Image,'/video/image_raw')
        msg = Image()
        clientID = simxStart('127.0.0.1',19999,True,True,5000,5)
        res,objs = simxGetObjects(clientID,sim_handle_all,simx_opmode_blocking)
        er , cam_handle   = simxGetObjectHandle(clientID, 'kinect_rgb', simx_opmode_blocking) 
        er, t_rightWheel  = simxGetObjectHandle(clientID, 'wheel_right_joint', simx_opmode_blocking)
        er, t_leftWheel   = simxGetObjectHandle(clientID, 'wheel_left_joint', simx_opmode_blocking) 
        simxSetJointTargetVelocity(clientID, t_rightWheel, 1, simx_opmode_streaming)
        simxSetJointTargetVelocity(clientID, t_leftWheel, 1, simx_opmode_streaming)
        

        while(True):

            err, resolution, colorCam  = simxGetVisionSensorImage(clientID, cam_handle, 0, simx_opmode_oneshot_wait)
            image_from_v_rep = cv2.imread('/home/mirellameelo/simulator/src/simulation/simulation/OGM.png')
            
            
            img = np.array(colorCam, dtype=np.uint8)
            
            img.resize([resolution[1], resolution[0], 3])
            #img3 = np.flipud(img)
            msg = bridge.cv2_to_imgmsg(np.flipud(img), encoding="rgb8")
            cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
            self.image.publish(msg)



def main(args=None):

    simxFinish(-1) 

    rclpy.init(args=args)

    simulation_ = simulation()

    rclpy.spin(simulation_)

    #simulation_.destroy_node()
    #rclpy.shutdown()


if __name__ == '__main__':
    main()




# UNCOMMENT
# sim.simxFinish(-1) # just in case, close all opened connections
# clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to CoppeliaSim

# print ('Connected to remote API server')
# # Now try to retrieve data in a blocking fashion (i.e. a service call):
# res,objs = sim.simxGetObjects(clientID,sim.sim_handle_all,sim.simx_opmode_blocking)
# er, t_rightWheel  = sim.simxGetObjectHandle(clientID, 'wheel_right_joint', sim.simx_opmode_blocking)
# er, t_leftWheel   = sim.simxGetObjectHandle(clientID, 'wheel_left_joint', sim.simx_opmode_blocking) 
# er , cam_handle   = sim.simxGetObjectHandle(clientID, 'kinect_rgb', sim.simx_opmode_blocking) 


# while(True):
#     err, resolution, colorCam  = sim.simxGetVisionSensorImage(clientID, cam_handle, 0, sim.simx_opmode_oneshot_wait)
#     img = np.array(colorCam, dtype=np.uint8)
#     print(img.shape)
#     img.resize([resolution[1], resolution[0], 3])
#     img2 = np.flipud(img)
#     img3 = img2[...,::-1].copy()
#     cv2.imshow('original', img3)
#     cv2.waitKey(1)

#     sim.simxSetJointTargetVelocity(clientID, t_rightWheel, 1, sim.simx_opmode_streaming)
#     sim.simxSetJointTargetVelocity(clientID, t_leftWheel, 1, sim.simx_opmode_streaming)
    