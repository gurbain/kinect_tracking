import numpy as np
import pickle
import rospy as ros
import rospkg
import sys
import traceback

from  tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

import utils

class CustomTF:

    def __init__(self, save_tf=True):

        rp = rospkg.RosPack()
        path = rp.get_path('tigrillo_tracking') + "/data"
        utils.mkdir(path)
        self.save_filename = path + "/tf_calib.pkl"

        self.save_tf = save_tf
        self.p_world = None
        self.p_cam = None

    def __rigid_transform_3D(self):

        assert len(self.p_world) == len(self.p_cam)

        N = self.p_cam.shape[0]; # total points

        centroid_cam = np.mean(self.p_cam, axis=0)
        centroid_world = np.mean(self.p_world, axis=0)


        # centre the points
        P_world = self.p_world - np.tile(centroid_world, (N, 1))
        P_cam = self.p_cam - np.tile(centroid_cam, (N, 1))

        pickle.dump([P_world, P_cam], open("ici.pkl", "wb"))

        # # Pad matrices
        # pad = lambda x: np.hstack([x, np.ones((x.shape[0], 1))])
        # unpad = lambda x: x[:,:-1]
        # P_world = pad(P_world)
        # P_cam = pad(P_cam)

        # # Solve least square problem
        # P_cam = np.hstack((P_cam, np.ones((N, 1))))
        # print P_cam, P_world.shape
        # M = np.linalg.lstsq(P_cam, P_world)[0]
        # print M.shape
        # R = M[0:3,:]
        # t = M[3, :]
        # print R.shape, P_cam[0, 0:3].shape, t.shape
        # print R.dot(P_cam[0, 0:3]) + t, P_world[0]
        # print M.dot(P_cam[0, 0:3]), P_world[0]

        # print unpad(np.dot(BB, A))
        # print unpad(BB)
        # print unpad(BB) - unpad(np.dot(BB, A))
        # utils.cleanup()# print AA, BB

        # dot is matrix multiplication for array
        H = np.transpose(P_cam).dot(P_world)
        U, S, Vt = np.linalg.svd(H)
        R = Vt.T * U.T

        # special reflection case
        if np.linalg.det(R) < 0:
           ros.logwarn("Reflection detected")
           Vt[2,:] *= -1
           R = Vt.T * U.T

        t = -R.dot(centroid_cam.T) + centroid_world.T
        M = np.hstack((R, t.reshape((3,1))))
        print M

        # Get error
        reference = self.p_world.T
        cam_padded = np.hstack((self.p_cam, np.ones((N,1)))).T
        reconstruct = M.dot(cam_padded)
        difference = reference - reconstruct

        # reference = unpad(P_world)
        # reconstruct = unpad(np.dot(A, P_cam.T).T)
        # difference = reference - reconstruct 

        print reference
        print reconstruct
        print difference
        print np.min(difference), np.max(difference), np.mean(difference)

        return M

    def __save(self, M):

        ros.logwarn("Saving translation and rotation from camera to world coordinates")
        pickle.dump(M, open(self.save_filename, "wb"))

    def __load(self):

        ros.logwarn("Loading translation and rotation from camera to world coordinates")
        return pickle.load(open(self.save_filename, "rb"))

    def append(self, p_world, p_cam):

        try:
            # Add Noise??
            #p_world =  np.array(p_world) + 0.01 * np.random.rand(1, 3)

            if self.p_world is None:
                self.p_world = np.array(p_world)
            else:
                self.p_world = np.vstack((self.p_world, np.array(p_world)))

            if self.p_cam is None:
                self.p_cam = np.array(p_cam)
            else:
                self.p_cam = np.vstack((self.p_cam, np.array(p_cam)))

        except Exception, err:
            traceback.print_exc(file=sys.stdout)
            utils.cleanup()

    def get_tf_cam_world(self, load_tf=False):

        try:
            if load_tf == False:
                M = self.__rigid_transform_3D()
                print M
                if self.save_tf:
                    self.__save(M)
            else:
                M = self.__load()

        except Exception, err:
            traceback.print_exc(file=sys.stdout)
            utils.cleanup()

        return M