SX = 0.025
SY = 0.09
SZ = 0.025

KEY_SPACING = 0.001

class BoundingBox():

    def __init__(self, x, y, z, sx, sy, sz):
        assert(sx > 0 and sy > 0 and sz > 0)
        self.x = x
        self.y = y
        self.z = z
        self.sx = sx
        self.sy = sy
        self.sz = sz

    def contains(self, pos):
        return self.x < pos[0] < self.x + self.sx and \
               self.y < pos[1] < self.y + self.sy and \
               self.z < pos[2] < self.z + self.sz

    def center(self):
        return np.array([self.x + self.sx/2,
                         self.y + self.sy/2,
                         self.z + self.sz/2])

key_names = ["C","D","E","F","G","A","B"]

KEYS = {}
for (i, key_name) in enumerate(key_names):
    bb = BoundingBox(1.0 + (SX + KEY_SPACING) * i - SX/2, 1.0 - SY/2, -SZ/2, SX, SY, SZ)
    KEYS[key_name] = bb
