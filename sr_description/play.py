'''play.py

   Node:        /generator
   Publish:     /joint_states           sensor_msgs/JointState

'''

import rclpy
import numpy as np

from GeneratorNode     import GeneratorNode
from KinematicChain    import KinematicChain
from TransformHelpers  import *

N_SHARED_JOINTS = 8
N_FINGER_JOINTS = 4




def dedup(lst):
    lookup = set()
    return [x for x in lst if not (x in lookup or lookup.add(x))]

FINGER_CENTERS = np.array([-0.1, -0.1, -0.1, -0.1]).reshape((-1,1))

class Trajectory():

    def __init__(self, node):
        self.pachain = KinematicChain(node, 'world', 'rh_manipulator')
        self.thchain = KinematicChain(node, 'world', 'rh_fftip')
        self.mfchain = KinematicChain(node, 'world', 'rh_mftip')
        print("Num DOFS thumb {}".format(self.thchain.dofs))
        print("Num DOFS mf {}".format(self.mfchain.dofs))


        self.joints = dedup(self.pachain.jointnames + self.thchain.jointnames + self.mfchain.jointnames)
        self.lam = 20
        print(self.joints)

        self.q = np.zeros((len(self.joints), 1)).reshape((-1,1))
        self.err = np.zeros((12, 1))

        self.q_centers = np.vstack([np.zeros((N_SHARED_JOINTS, 1)), FINGER_CENTERS, FINGER_CENTERS])
        self.lams = np.vstack([np.zeros((N_SHARED_JOINTS, 1)), 100 * np.ones((N_FINGER_JOINTS * 2, 1))])

    def build_J(self):
        paJw = self.pachain.Jw()
        thJ = self.thchain.Jv()
        mfJ = self.mfchain.Jv()

        # These should always be the same matrix.
        # Might be interesting to try to assert that
        shared = thJ[:, 0:N_SHARED_JOINTS]
        shared1 = mfJ[:, 0:N_SHARED_JOINTS]

        th = thJ[:, N_SHARED_JOINTS:]
        mf = thJ[:, N_SHARED_JOINTS:]

        zeros = np.zeros(th.shape)
        
        pa_orientation = np.hstack([paJw, zeros, zeros])
        full_pa =        np.hstack([shared, zeros, zeros])
        full_th =        np.hstack([shared, th, zeros])
        full_mf =        np.hstack([shared, zeros, mf])

        return np.vstack([pa_orientation, full_pa, full_th, full_mf])

    def ptip(self):
        p0 = self.pachain.ptip()
        p1 = self.thchain.ptip()
        p2 = self.mfchain.ptip()

        return np.vstack([p0, p1,p2])

    def Rtip(self):
        p0 = self.pachain.Rtip()

        return p0


    def jointnames(self):
        return self.joints

    def evaluate(self, t, dt):
        
        # A basic trajectory: try to spin the fingertips in circles

        # Palm face down
        pa_rd =   Rotz(np.pi) @ Rotx(np.pi/2)
        pa_wd = exyz(1, 0, 1)


        
        pa_x_desired = np.array([103 + np.sin(t), 87, 0]).reshape((3,1)) * 0.01
        th_x_desired = np.array([100, 100, 2*np.sin(15*t)]).reshape((3,1)) * 0.01
        mf_x_desired = np.array([105, 100, 2*np.sin(15*t)]).reshape((3,1)) * 0.01
        pa_v_desired = np.array([0,   0,   np.cos(t)]).reshape((3,1)) * 0.01
        th_v_desired = np.array([0,   0,   np.cos(t)]).reshape((3,1)) * 0.01
        mf_v_desired = np.array([0,   0,   np.cos(t)]).reshape((3,1)) * 0.01


        # Grab the last joint value and task error.
        q   = self.q
        err = self.err


        J = self.build_J()
        xdot = np.vstack([pa_wd, pa_v_desired, th_v_desired, mf_v_desired])

        qdot_secondary = self.lams * (self.q_centers - self.q)
        qdot = np.linalg.pinv(J) @ (xdot + self.lam * err)
        qdot = qdot + (np.eye(len(self.joints)) - np.linalg.pinv(J) @ J) @ qdot_secondary

        # Integrate the joint position and update the kin chain data.
        q = q + dt * qdot
        q_shared = q[:N_SHARED_JOINTS]
        q_th = q[N_SHARED_JOINTS:N_SHARED_JOINTS+N_FINGER_JOINTS]
        q_mf = q[N_SHARED_JOINTS+N_FINGER_JOINTS:]

        self.pachain.setjoints(q_shared)
        self.thchain.setjoints(np.vstack([q_shared, q_th]))
        self.mfchain.setjoints(np.vstack([q_shared, q_mf]))

        # Compute the resulting task error (to be used next cycle).
        positional_err  = ep(np.vstack([pa_x_desired, th_x_desired, mf_x_desired]), self.ptip())
        rotational_err = eR(pa_rd, self.Rtip())

        err = np.vstack([rotational_err, positional_err])

        self.q = q
        self.err = err


        # Return the position and velocity as python lists.
        return (self.q.flatten().tolist(), qdot.flatten().tolist())


def main(args=None):
    # Initialize ROS and the generator node (100Hz) for the Trajectory.
    rclpy.init(args=args)
    generator = GeneratorNode('generator', 100, Trajectory)

    # Spin, until interrupted or the trajectory ends.
    generator.spin()

    # Shutdown the node and ROS.
    generator.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
