"""/map 점유 격자 데이터를 이미지(PNG)로 저장하고 프론티어를 시각화하는 스크립트."""
import numpy as np, rclpy, time
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from scipy import ndimage
from PIL import Image

class P(Node):
    def __init__(self):
        super().__init__('render_map')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.msg = None
        q = QoSProfile(depth=1)
        q.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        q.reliability = QoSReliabilityPolicy.RELIABLE
        self.create_subscription(OccupancyGrid, '/map', self._cb, q)
    def _cb(self, m): self.msg = m

rclpy.init(); n = P(); t = time.time()
while n.msg is None and time.time() - t < 30: rclpy.spin_once(n, timeout_sec=0.2)
m = n.msg; i = m.info
g = np.asarray(m.data, dtype=np.int16).reshape(i.height, i.width)
free = (g >= 0) & (g <= 25); unk = g < 0; occ = g >= 65
img = np.zeros((i.height, i.width, 3), np.uint8)
img[unk] = (110, 110, 130); img[free] = (245, 245, 245); img[occ] = (20, 20, 20)
nb = np.zeros_like(unk)
nb[1:,:] |= unk[:-1,:]; nb[:-1,:] |= unk[1:,:]
nb[:,1:] |= unk[:,:-1]; nb[:,:-1] |= unk[:,1:]
fr = free & nb
img[fr] = (255, 40, 40)
gr = ndimage.binary_dilation(occ, structure=np.ones((3,3),bool), iterations=5)
img[fr & ~gr] = (0, 200, 0)
Image.fromarray(np.flipud(img)).resize((i.width*3, i.height*3), Image.NEAREST).save('explore_map_turn.png')
print('원점(%.2f,%.2f) %dx%d  빨강=프론티어 %d, 초록=여유통과 %d'
      % (i.origin.position.x, i.origin.position.y, i.width, i.height, fr.sum(), (fr & ~gr).sum()))
rclpy.shutdown()
