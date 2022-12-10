'''play.py
Node:        /generator
   Publish:     /joint_states           sensor_msgs/JointState

'''

import rclpy
import numpy as np
from typing import List, Tuple

from GeneratorNode     import GeneratorNode
from KinematicChain    import KinematicChain
from TransformHelpers  import *
from Segments import *
from music import l_theme as NOTES
from music import Note
from keys import KEYS as KEYS_MAPPING

LEFT_NOTES = []
RIGHT_NOTES = []
for note in NOTES:
    if note.finger == 'L':
        LEFT_NOTES.append(note)
    else:
        RIGHT_NOTES.append(note)


N_SHARED_JOINTS = 8
N_FINGER_JOINTS = 4



def spline(t, T, p0, pf):
    p = p0 + (pf-p0) * (3*t**2/T**2 - 2*t**3/T**3)
    v =      (pf-p0) * (6*t   /T**2 - 6*t**2/T**3)
    return (p, v)

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
        self.lam = 60
        print(self.joints)

        self.q = np.zeros((len(self.joints), 1)).reshape((-1,1))
        self.err = np.zeros((10, 1))

        self.th_p0 = self.thchain.fkin(np.zeros((self.thchain.dofs,1)))[0]
        self.mf_p0 = self.mfchain.fkin(np.zeros((self.mfchain.dofs,1)))[0]

        self.th_x_desired_prev = self.th_p0
        self.mf_x_desired_prev = self.mf_p0

        self.left_segments = self.build_segments(self.th_p0, LEFT_NOTES)
        self.right_segments = self.build_segments(self.mf_p0, RIGHT_NOTES)

        self.left_t0 = 0.0
        self.right_t0 = 0.0

        self.q_centers = np.vstack([np.zeros((N_SHARED_JOINTS, 1)), FINGER_CENTERS, FINGER_CENTERS])
        self.lams = np.vstack([np.zeros((N_SHARED_JOINTS, 1)), 0 * np.ones((N_FINGER_JOINTS * 2, 1))])

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
        height_pa =      np.hstack([shared, zeros, zeros])[2, :].reshape((1,16))
        full_th =        np.hstack([shared, th, zeros])
        full_mf =        np.hstack([shared, zeros, mf])

        return np.vstack([pa_orientation, height_pa, full_th, full_mf])

    def ptip(self):
        p0 = self.pachain.ptip()[2]
        p1 = self.thchain.ptip()
        p2 = self.mfchain.ptip()

        return np.vstack([p0, p1,p2])

    def Rtip(self):
        p0 = self.pachain.Rtip()

        return p0


    def jointnames(self):
        return self.joints

    def get_note_position(self, note: Note) -> np.ndarray:
        key_pos = KEYS_MAPPING[note.note]
        return np.array([key_pos.x, key_pos.y, key_pos.z]).reshape((3, 1))

    def build_segments(self, p0: np.ndarray, notes: List[Note]) -> List[SplineCubic]:
        segments = []
        for i, note in enumerate(notes):
            if i == 0:
                pA = p0
                move_duration = 3
            else:
                pA = self.get_note_position(notes[i - 1])
                move_duration = note.start - notes[i - 1].start
            pB = self.get_note_position(note)
            segments.append(GotoCubic(pA, pB, move_duration))
            segments.append(Hold(pB, note.duration))
        segments.append(Stay(pB))
        return segments

    def get_segment_desired(self, t: float, t0: float, segments: List[SplineCubic]) -> Tuple[np.ndarray, np.ndarray]:
        if segments[0].completed(t - t0):
            t0 = t0 + segments[0].duration()
            segments.pop()
        (x, xdot) = segments[0].evaluate(t - t0)
        return (x, xdot, t0)

    def evaluate(self, t, dt):
        
        # A basic trajectory: try to spin the fingertips in circles

        # Palm face down
        pa_rd = Rotz(np.pi) @ Rotx(np.pi/2)
        pa_wd = exyz(1, 0, 1)

        (th_x_desired, th_v_desired, self.left_t0) = self.get_segment_desired(t, self.left_t0, self.left_segments)

        (mf_x_desired, mf_v_desired, self.right_t0) = self.get_segment_desired(t, self.right_t0, self.right_segments)
        
        # th_x_desired = np.array([100, 100, 2*np.sin(15*t)]).reshape((3,1)) * 0.01
        # mf_x_desired = np.array([105, 100, 2*np.sin(15*t)]).reshape((3,1)) * 0.01

        # th_v_desired = np.array([0,   0,   np.cos(t)]).reshape((3,1)) * 0.01
        # mf_v_desired = np.array([0,   0,   np.cos(t)]).reshape((3,1)) * 0.01

        pa_x_desired = np.array([1]).reshape((1,1)) * 0.01
        pa_v_desired = np.array([0]).reshape((1,1)) * 0.01

        # Grab the last joint value and task error.
        q   = self.q
        err = self.err


        J = self.build_J()
        Jinv = J.T @ np.linalg.inv(J @ J.T + 0.0001 * np.eye(10))
        xdot = np.vstack([pa_wd, pa_v_desired, th_v_desired, mf_v_desired])


        qdot_secondary = self.lams * (self.q_centers - self.q)
        qdot = Jinv @ (xdot + self.lam * err)
        qdot = qdot + (np.eye(len(self.joints)) - Jinv @ J) @ qdot_secondary

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
