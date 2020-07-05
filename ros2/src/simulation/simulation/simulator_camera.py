import sim
import time
import cv2
import numpy as np
import rospy
#simRemoteApi.start(19999)

sim.simxFinish(-1) # just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to CoppeliaSim

print ('Connected to remote API server')
# Now try to retrieve data in a blocking fashion (i.e. a service call):
res,objs = sim.simxGetObjects(clientID,sim.sim_handle_all,sim.simx_opmode_blocking)
er, t_rightWheel  = sim.simxGetObjectHandle(clientID, 'wheel_right_joint', sim.simx_opmode_blocking)
er, t_leftWheel   = sim.simxGetObjectHandle(clientID, 'wheel_left_joint', sim.simx_opmode_blocking) 
er , cam_handle   = sim.simxGetObjectHandle(clientID, 'kinect_rgb', sim.simx_opmode_blocking) 


while(True):
    err, resolution, colorCam  = sim.simxGetVisionSensorImage(clientID, cam_handle, 0, sim.simx_opmode_oneshot_wait)
    img = np.array(colorCam, dtype=np.uint8)
    print(img.shape)
    img.resize([resolution[1], resolution[0], 3])
    img2 = np.flipud(img)
    img3 = img2[...,::-1].copy()
    cv2.imshow('original', img3)
    cv2.waitKey(1)

    #sim.simxSetJointTargetVelocity(clientID, t_rightWheel, 1, sim.simx_opmode_streaming)
    #sim.simxSetJointTargetVelocity(clientID, t_leftWheel, 1, sim.simx_opmode_streaming)
    