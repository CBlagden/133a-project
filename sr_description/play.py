'''play.py

   Node:        /generator
   Publish:     /joint_states           sensor_msgs/JointState

'''

import rclpy
import numpy as np

from GeneratorNode     import GeneratorNode
from KinematicChain    import KinematicChain
from TransformHelpers  import *


class Trajectory():

    def __init__(self, node):
        self.chain = KinematicChain(node, 'world', 'rh_thtip')
        print("Num DOFS {}".format(self.chain.dofs))
        print(self.chain.jointnames)


    def jointnames(self):
        return self.chain.jointnames

    def evaluate(self, t, dt):
        self.q = np.ones((self.chain.dofs,))
        qdot = np.ones((self.chain.dofs,))

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
