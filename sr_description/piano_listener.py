import rclpy
from rclpy.node import Node
from sensor_msgs.msg    import JointState
from KinematicChain    import KinematicChain

import keys

from mingus.containers import Note
from mingus.midi import fluidsynth

fluidsynth.init('/usr/share/sounds/sf2/FluidR3_GM.sf2', 'alsa')

class PianoKeyTracker():
    PRESSED = 1
    UNPRESSED = 0

    def __init__(self):
        self.old_keystates = {key:False for key in keys.KEYS.keys()}
        self.keystates = {key:False for key in keys.KEYS.keys()}

    def handle_pos(self, pos):
        for (key, bb) in keys.KEYS.items():
            if bb.contains(pos):
                self.keystates[key] = self.PRESSED

    def update(self):
        for key in keys.KEYS.keys():
            if self.old_keystates[key] == self.UNPRESSED and self.keystates[key] == self.PRESSED:
                # 127 is max volume
                volume = 127
                fluidsynth.play_Note(Note(f"{key}-4"), volume)
                print(f"{key} was pressed")
            elif self.old_keystates[key] == self.PRESSED and self.keystates[key] == self.UNPRESSED:
                channel = 1
                fluidsynth.stop_Note(Note(f"{key}-4"), channel)
                print(f"{key} was released")

        self.old_keystates = self.keystates

        # reset self.keystates
        self.keystates = {key:False for key in keys.KEYS.keys()}

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

        self.key_tracker = PianoKeyTracker()

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

        self.key_tracker.handle_pos(ff_pos)
        self.key_tracker.handle_pos(mf_pos)
        self.key_tracker.update()


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

