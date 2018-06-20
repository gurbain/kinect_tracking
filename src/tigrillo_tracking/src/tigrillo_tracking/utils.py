import numpy as np
import os
import psutil
import rospy as ros
import time


def cleanup():

    ros.logwarn('Quitting and killing all children processes!')
    process = psutil.Process()
    children = process.children(recursive=True)
    time.sleep(0.2)
    for p in children:
        p.kill()
    process.kill()

def norm(v):
    norm = np.linalg.norm(v)
    if norm == 0: 
       return v
    return v / norm


def mkdir(path):
    """ Create a directory if it does not exist yet """

    if not os.path.exists(path):
        os.makedirs(path)


def R_to_q(R):

    w = np.math.sqrt(float(1)+R[0,0]+R[1,1]+R[2,2])*0.5
    x = (R[2,1]-R[1,2])/(4*w)
    y = (R[0,2]-R[2,0])/(4*w)
    z = (R[1,0]-R[0,1])/(4*w)
    q = np.array([x, y, z, w])

    return norm(q)