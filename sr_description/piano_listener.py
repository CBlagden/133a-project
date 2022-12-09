import rclpy
from rclpy.node import Node
from sensor_msgs.msg    import JointState
from KinematicChain    import KinematicChain

class BoundingBox():

    def __init__(self, x, y, z, sx, sy, sz):
        assert(sx > 0 and sy > 0 and sz > 0)
        self.x = x
        self.y = y
        self.z = z
        self.sx = sx
        self.sy = sy
        self.sz = sz

    def contains(pos):
        return self.x < pos[0] < self.x + self.sx and \
               self.y < pos[1] < self.y + self.sy and \
               self.z < pos[2] < self.z + self.sz


class PianoKeyTracker():
    PRESSED = 1
    UNPRESSED = 0

    # TODO: Fill in the pseudocode here.
    def __init__(self):
        self.old_keystates = []
        self.keystates = []

    def handle_pos(self, pos):
        for (key, bb) in bounding_boxes:
            if bb.contains(pos):
                self.keys[key] = PRESSED

    def update(self):
        for key in keys:
            if self.old_keystates[key] == UNPRESSED and self.keystates[key] == PRESSED:
                # Emit something saying the key has been pressed
            elif self.old_keystates[key] == PRESSED and self.keystates[key] == UNPRESSED:
                # Emit something saying the key has been released

        self.old_keystates = self.keystates
        # reset self.keystates
        self.keystates = []





class PianoPlayer(Node):

    def __init__(self):
        super().__init__('piano_player')
        self.initialized = False

        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.listener_callback,
            10)

        self.subscription  # prevent unused variable warning

        self.ffchain = KinematicChain(self, 'world', 'rh_fftip')
        self.mfchain = KinematicChain(self, 'world', 'rh_mftip')
        self.initialized = True

    def listener_callback(self, msg):
        if not self.initialized:
            return

        angles = dict(zip(msg.name, msg.position))

        ff_angles = [angles[joint] for joint in self.ffchain.jointnames]
        self.ffchain.setjoints(ff_angles)
        ff_pos = self.ffchain.ptip()

        mf_angles = [angles[joint] for joint in self.mfchain.jointnames]
        self.mfchain.setjoints(mf_angles)
        mf_pos = self.mfchain.ptip()

        # TODO: TRACK POSITIONS TO SEE WHICH KEYS ARE PRESSED


def main(args=None):
    rclpy.init(args=args)
    print("Starting piano player")

    piano_player = PianoPlayer()

    rclpy.spin(piano_player)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    piano_player.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

