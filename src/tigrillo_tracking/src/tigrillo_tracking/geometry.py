import numpy as np
import pickle
import rospy as ros
import rospkg
import sys
import traceback

from  tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

import utils

np.set_printoptions(precision=2, suppress=True)

class CustomTF:

    def __init__(self, save_tf=True):

        rp = rospkg.RosPack()
        path = rp.get_path('tigrillo_tracking') + "/data"
        utils.mkdir(path)
        self.save_filename = path + "/tf_calib.pkl"

        self.save_tf = save_tf
        self.p_world = None
        self.p_cam = None
        self.M = None

    def __rigid_transform_3D(self, A, B):

        assert len(A) == len(B)

        N = A.shape[0]; # total points

        centroid_A = np.mean(A, axis=0)
        centroid_B = np.mean(B, axis=0)
        
        # centre the points
        AA = A - np.tile(centroid_A, (N, 1))
        BB = B - np.tile(centroid_B, (N, 1))

        # dot is matrix multiplication for array
        H = np.transpose(BB) * AA

        U, S, Vt = np.linalg.svd(H)

        R = Vt.T * U.T

        # special reflection case
        if np.linalg.det(R) < 0:
           print "Reflection detected"
           Vt[2,:] *= -1
           R = Vt.T * U.T

        t = -R*centroid_B.T + centroid_A.T

        M = np.hstack((R, t.reshape((3,1))))

        # Get error
        reference = self.p_world.T
        cam_padded = np.hstack((self.p_cam, np.ones((N,1)))).T
        reconstruct = M.dot(cam_padded)
        difference = reference - reconstruct

        print "R"
        print R
        print "\nt"
        print t
        print "\nREFERENCE"
        print reference.T
        print "\nRECONSTRUCTION"
        print reconstruct.T
        print "\nDIFFERENCE"
        print difference.T
        print "\nSTATS: Min, Max, Mean"
        print np.min(difference), np.max(difference), np.mean(difference)

        self.M = M

    def __save(self):

        ros.logwarn("Saving translation and rotation from camera to world coordinates")
        pickle.dump([self.M, self.p_world, self.p_cam], open(self.save_filename, "wb"))

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
                self.__rigid_transform_3D(np.mat(self.p_world), np.mat(self.p_cam))
                if self.save_tf:
                    self.__save()
            else:
                [self.M, self.p_world, self.p_cam] = self.__load()
            
            return self.M

        except Exception, err:
            traceback.print_exc(file=sys.stdout)
            utils.cleanup()
