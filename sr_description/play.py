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
# from music import little_lamb as NOTES
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



def dedup(lst):
    lookup = set()
    return [x for x in lst if not (x in lookup or lookup.add(x))]

FINGER_CENTERS = -1 * np.array([-0.0, -0.2, -0.2, -0.4]).reshape((-1,1))

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

        INITIAL_DURATION = 3.0
        self.left_segments = self.build_segments(self.th_p0, INITIAL_DURATION, LEFT_NOTES)
        self.right_segments = self.build_segments(self.mf_p0, INITIAL_DURATION, RIGHT_NOTES)

        self.left_t0 = 0.0
        self.right_t0 = 0.0

        self.q_centers = np.vstack([np.zeros((N_SHARED_JOINTS, 1)), FINGER_CENTERS, FINGER_CENTERS])
        self.lams = np.vstack([np.zeros((N_SHARED_JOINTS, 1)), 80 * np.ones((N_FINGER_JOINTS * 2, 1))])

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
        bb = KEYS_MAPPING[note.note]
        return bb.center().reshape((3, 1))

    def build_segments(self, p0: np.ndarray, initial_duration, notes: List[Note]) -> List[SplineCubic]:
        segments = []
        delta_z = np.array([0, 0, 0.03]).reshape((-1, 1))

        for i, note in enumerate(notes):
            if i == 0:
                pA = p0
                move_duration = initial_duration

                pB = self.get_note_position(note)
                segments.append(GotoCubic(pA, pB + delta_z, initial_duration))
                segments.append(GotoCubic(pB + delta_z, pB, note.start + 0.2))
                segments.append(Hold(pB, note.duration))
            else:
                pA = self.get_note_position(notes[i - 1])
                move_duration = note.start - notes[i - 1].start - notes[i-1].duration
                pB = self.get_note_position(note)

                segments.append(GotoCubic(pA, pA + delta_z, move_duration / 8))
                segments.append(GotoCubic(pA + delta_z, pB + delta_z, 3 * move_duration / 4))
                segments.append(GotoCubic(pB + delta_z, pB, move_duration / 8))
                segments.append(Hold(pB, note.duration))
        segments.append(Stay(pB))

        return segments

    def get_segment_desired(self, t: float, t0: float, segments: List[SplineCubic]) -> Tuple[np.ndarray, np.ndarray, float]:
        if segments[0].completed(t - t0):
            t0 = t0 + segments[0].duration()
            segments.pop(0)
        (x, xdot) = segments[0].evaluate(t - t0)
        return (x, xdot, t0)

    def evaluate(self, t, dt):
        
        # A basic trajectory: try to spin the fingertips in circles

        # Palm face down
        pa_rd = Rotz(np.pi) @ Rotx(np.pi/2)
        pa_wd = exyz(1, 0, 1)

        (th_x_desired, th_v_desired, self.left_t0) = self.get_segment_desired(t, self.left_t0, self.left_segments)
        (mf_x_desired, mf_v_desired, self.right_t0) = self.get_segment_desired(t, self.right_t0, self.right_segments)

        print(t)
        print("vels", th_v_desired)
        print(mf_v_desired)
        print("pos", th_x_desired)
        print(mf_x_desired)
        if np.max(np.abs(th_v_desired)) > 2 or np.max(np.abs(mf_v_desired)) > 2:
            print("BIGGG ALERT" + 100 * "-")

        if np.max(np.abs(th_v_desired)) > 5:
            th_v_desired = 0.05 * th_v_desired
        if np.max(np.abs(mf_v_desired)) > 5:
            mf_v_desired = 0.05 * mf_v_desired

        pa_x_desired = np.array([1]).reshape((1,1)) * 0.01
        pa_v_desired = np.array([0]).reshape((1,1)) * 0.01

        # Grab the last joint value and task error.
        q   = self.q
        err = self.err


        J = self.build_J()
        Jinv = J.T @ np.linalg.inv(J @ J.T + 0.001 * np.eye(10))
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
